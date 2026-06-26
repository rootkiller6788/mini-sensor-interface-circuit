/**
 * @file cap_sense_core.c
 * @brief Core Capacitive Sensing Implementation
 *
 * Implements the fundamental physical models and definitions for
 * capacitive touch and proximity sensing.
 *
 * Knowledge Coverage:
 *   L1: typedef implementations, HBM defaults, system init
 *   L2: channel configuration, self/mutual mode selection
 *   L3: parallel-plate model, body-to-earth model, finger-electrode model
 *   L4: kT/C noise, SNR fundamental limit, minimum resolvable deltaC
 *
 * Every function explicitly references its originating physical law
 * and provides the formula in both symbolic and numeric form.
 */

#include "cap_sense_core.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ==========================================================================
 * L1: SYSTEM INITIALIZATION
 * ========================================================================== */

/**
 * cap_sensor_system_init
 *
 * Allocates and initializes a multi-channel capacitive sensor system.
 * Sets default timing parameters: scan period 10 ms (100 Hz), which gives
 * Nyquist frequency 50 Hz for touch motion tracking.
 *
 * Internal reference capacitor C_int is typically 10 nF for charge-transfer
 * methods or 10 pF for sigma-delta CDC. This sets the measurement range.
 *
 * Complexity: O(num_ch) — allocates and initializes each channel.
 */
int cap_sensor_system_init(cap_sensor_system_t *sys, uint8_t num_ch,
                           double v_exc, double c_ref)
{
    if (!sys) return -1;
    if (num_ch == 0 || num_ch > 64) return -1;
    if (v_exc <= 0.0 || c_ref <= 0.0) return -1;

    memset(sys, 0, sizeof(*sys));
    sys->num_channels = num_ch;
    sys->v_excitation = v_exc;
    sys->v_ref = v_exc; /* Default: V_ref = V_exc for ratiometric */
    sys->c_internal_ref_f = c_ref;
    sys->scan_period_us = 10000;      /* 100 Hz default */
    sys->resolution_f = 1.0e-15;      /* 1 fF default resolution */
    sys->auto_cal_enabled = true;
    sys->low_power_mode = false;
    sys->temperature_c = 25.0;
    sys->humidity_pct = 50.0;

    sys->channels = (cap_sensor_channel_t *)calloc(num_ch,
                                                    sizeof(cap_sensor_channel_t));
    if (!sys->channels) return -1;

    /* Initialize each channel with sensible defaults */
    for (uint8_t i = 0; i < num_ch; i++) {
        cap_sensor_channel_t *ch = &sys->channels[i];
        ch->channel_id = i;
        ch->method = CAP_METHOD_CHARGE_TRANSFER;
        ch->mode = CAP_SENSE_SELF;
        ch->electrode.shape = ELEC_RECT;
        ch->electrode.electrode_area_m2 = 1.0e-4;  /* 1 cm2 default */
        ch->electrode.overlay_thickness_m = 1.0e-3; /* 1 mm glass */
        ch->electrode.overlay_epsilon_r = 7.5;      /* Glass */
        ch->electrode.pcb_epsilon_r = 4.5;          /* FR4 */
        ch->state = TOUCH_IDLE;
        ch->debounce_count = 0;
        ch->threshold_f = 100.0e-15;  /* 100 fF default threshold */
        ch->hysteresis_f = 30.0e-15;  /* 30 fF hysteresis */
        ch->is_proximity = false;
        ch->self_meas.c_body_f = 150.0e-12;  /* HBM default */
        ch->self_meas.c_baseline_f = 10.0e-12; /* Initial baseline guess */
    }

    return 0;
}

/**
 * cap_channel_configure
 *
 * Configures a single channel with specific electrode geometry and
 * measurement parameters. The electrode area and overlay parameters
 * determine the baseline capacitance through the parallel-plate model.
 *
 * For a 1 cm² electrode under 1 mm glass (eps_r=7.5):
 * C_parallel = 8.854e-12 * 7.5 * 1e-4 / 1e-3 ≈ 6.64 pF
 */
int cap_channel_configure(cap_sensor_channel_t *chan,
                          cap_measurement_method_t method,
                          cap_sense_mode_t mode,
                          electrode_shape_t el_shape,
                          double el_area, double overlay_t,
                          double overlay_er, double threshold_f)
{
    if (!chan) return -1;
    if (el_area <= 0.0 || overlay_t <= 0.0 || overlay_er <= 1.0) return -1;

    chan->method = method;
    chan->mode = mode;
    chan->electrode.shape = el_shape;
    chan->electrode.electrode_area_m2 = el_area;
    chan->electrode.overlay_thickness_m = overlay_t;
    chan->electrode.overlay_epsilon_r = overlay_er;
    chan->threshold_f = threshold_f;
    chan->hysteresis_f = threshold_f * 0.3; /* 30% hysteresis default */

    /* Update baseline estimate from geometry */
    double c_pp = cap_parallel_plate_c(el_area, overlay_t, overlay_er);
    chan->self_meas.c_baseline_f = c_pp;
    chan->self_meas.c_total_f = c_pp;

    return 0;
}

/**
 * cap_sensor_system_destroy
 *
 * Frees all dynamically allocated resources in the sensor system.
 * Sets num_channels to 0 and channels pointer to NULL to prevent
 * use-after-free.
 */
void cap_sensor_system_destroy(cap_sensor_system_t *sys)
{
    if (sys) {
        free(sys->channels);
        sys->channels = NULL;
        sys->num_channels = 0;
    }
}

/* ==========================================================================
 * L3: PHYSICAL CAPACITANCE MODELS
 * ========================================================================== */

/**
 * cap_parallel_plate_c
 *
 * Computes capacitance using the parallel-plate approximation:
 *
 *   C = epsilon_0 * epsilon_r * A / d
 *
 * This is the fundamental formula derived from Gauss's Law
 * (integral form: surface_integral E·dA = Q_enclosed/epsilon_0) applied
 * to infinite parallel plates, giving uniform field E = sigma/epsilon_0
 * and V = E*d, hence C = Q/V = epsilon_0 * A / d.
 *
 * For finite plates, this underestimates C by 20-80% due to fringe
 * fields (see cap_fringe_field_correction in cap_sensor_geometry.c).
 *
 * Example: A = 1 cm², d = 1 mm, eps_r = 7.5 (glass)
 *   C = 8.854e-12 * 7.5 * 1e-4 / 1e-3 = 6.64e-12 = 6.64 pF
 *
 * @param area       Plate area [m²]
 * @param distance   Plate separation [m]
 * @param epsilon_r  Relative permittivity (dimensionless)
 * @return Capacitance [F]; returns 0.0 if area <= 0 or distance <= 0
 *
 * Ref: Griffiths "Introduction to Electrodynamics" 4th ed, §2.5.3
 *      Coulomb (1785) — inverse-square law
 *      Gauss (1835) — flux theorem
 */
double cap_parallel_plate_c(double area, double distance, double epsilon_r)
{
    if (area <= 0.0 || distance <= 0.0 || epsilon_r < 1.0) {
        return 0.0;
    }
    return CAP_EPSILON0 * epsilon_r * area / distance;
}

/**
 * cap_body_to_earth_c
 *
 * Estimates the human body's capacitance to earth ground using the
 * isolated conducting sphere approximation:
 *
 *   C_body = 4 * pi * epsilon_0 * R_effective
 *
 * where R_effective is derived from body surface area (Mosteller formula):
 *   BSA [m²] = sqrt(height_cm * mass_kg / 3600)
 *   R_eff = sqrt(BSA / (4*pi))    [equivalent sphere radius]
 *
 * When wearing shoes, a series capacitance C_shoe is added:
 *   C_shoe = epsilon_0 * epsilon_r_shoe * A_sole / thickness
 *   C_total = 1 / (1/C_body + 1/C_shoe)
 *
 * For a 1.7 m, 70 kg person barefoot:
 *   BSA ≈ sqrt(170*70/3600) ≈ 1.82 m²
 *   R_eff ≈ sqrt(1.82/(4*pi)) ≈ 0.38 m
 *   C_body ≈ 4*pi*8.854e-12*0.38 ≈ 42 pF (sphere approx is low;
 *     actual human body C_body ≈ 100-300 pF due to non-spherical shape)
 *
 * We use an empirical correction factor of 3.5 to account for
 * the body's non-spherical geometry and limb extensions.
 *
 * Ref: IEC 61340-3-1, ANSI/ESD STM5.1 Human Body Model
 */
double cap_body_to_earth_c(double body_height_m, double body_mass_kg,
                           double shoe_sole_thickness_m)
{
    if (body_height_m <= 0.0 || body_mass_kg <= 0.0) {
        return 150.0e-12; /* Default HBM value */
    }

    /* Mosteller BSA formula */
    double bsa_m2 = sqrt((body_height_m * 100.0) * body_mass_kg / 3600.0);
    double r_eff = sqrt(bsa_m2 / (4.0 * M_PI));
    double c_sphere = 4.0 * M_PI * CAP_EPSILON0 * r_eff;

    /* Empirical correction factor for non-spherical human body */
    double c_body = c_sphere * 3.5;

    if (shoe_sole_thickness_m <= 0.0) {
        return c_body;
    }

    /* Shoe capacitance: typical sole area ~0.02 m2, rubber eps_r ~3 */
    double sole_area = 0.02;
    double shoe_eps_r = 3.0;
    double c_shoe = CAP_EPSILON0 * shoe_eps_r * sole_area / shoe_sole_thickness_m;

    /* Series combination */
    return (c_body * c_shoe) / (c_body + c_shoe);
}

/**
 * cap_finger_electrode_c
 *
 * Computes the capacitance from a finger through the overlay to the
 * sensor electrode. This is the signal capacitance that changes when
 * a user touches the sensor.
 *
 * Model: The finger is treated as a conducting plate of area A_contact
 * separated from the electrode by the overlay of thickness d_overlay
 * and relative permittivity eps_r_overlay.
 *
 *   C_finger = epsilon_0 * eps_r_overlay * A_contact / d_overlay
 *            + C_fringe
 *
 * The fringe component is approximated as fringe_factor * C_parallel.
 * For a typical finger (A_contact ≈ 1 cm²), 1 mm glass overlay (eps_r=7.5):
 *   C_finger ≈ 8.854e-12 * 7.5 * 1e-4 / 1e-3 = 6.64 pF
 *   With fringe_factor=1.2: C_total ≈ 7.97 pF
 *
 * Actual measurement: 0.5-2 pF for finger through 1 mm glass.
 * The parallel-plate model overestimates because:
 * 1. Finger ridge air gaps reduce effective contact area
 * 2. Finger epidermis has finite conductivity
 * 3. Finger is not a perfect equipotential surface at AC
 *
 * We apply an empirical derating factor of 0.4 to match measured values.
 *
 * @param area_contact   Finger contact area [m²], typ 1e-4 (1 cm²)
 * @param overlay_thick  Overlay thickness [m], typ 1e-3 (1 mm)
 * @param overlay_er     Overlay relative permittivity
 * @param fringe_factor  Fringe multiplier (typ 1.15-1.30)
 * @return Finger-to-electrode capacitance [F]
 */
double cap_finger_electrode_c(double area_contact, double overlay_thick,
                              double overlay_er, double fringe_factor)
{
    if (area_contact <= 0.0 || overlay_thick <= 0.0 || overlay_er < 1.0) {
        return 0.0;
    }
    if (fringe_factor < 1.0) fringe_factor = 1.0;

    double c_pp = CAP_EPSILON0 * overlay_er * area_contact / overlay_thick;
    double c_with_fringe = c_pp * fringe_factor;

    /* Empirical derating for non-ideal finger contact (~0.4) */
    return c_with_fringe * 0.4;
}

/* ==========================================================================
 * L4: FUNDAMENTAL NOISE LIMITS
 * ========================================================================== */

/**
 * cap_ktc_noise_voltage
 *
 * Computes the Johnson-Nyquist thermal noise voltage on a capacitor:
 *
 *   v_n_rms = sqrt(k_B * T / C)
 *
 * This is the irreducible thermal noise floor. All measurement methods
 * are subject to this limit. It arises from the equipartition theorem:
 * each degree of freedom (the capacitor voltage) has average thermal
 * energy (1/2)*k_B*T, so (1/2)*C*v_n^2 = (1/2)*k_B*T → v_n^2 = k_B*T/C.
 *
 * At room temperature (T=300K):
 *   For C = 1 pF:  v_n = sqrt(4.14e-21 / 1e-12) ≈ 64.3 µV_rms
 *   For C = 10 pF: v_n = sqrt(4.14e-21 / 1e-11) ≈ 20.3 µV_rms
 *   For C = 100 pF: v_n = sqrt(4.14e-21 / 1e-10) ≈ 6.43 µV_rms
 *
 * The equivalent charge noise is: q_n = C * v_n = sqrt(k_B*T*C)
 *
 * Ref: Nyquist (1928) "Thermal Agitation of Electric Charge in Conductors"
 *      Johnson (1928) "Thermal Agitation of Electricity in Conductors"
 *      Phys Rev 32, 97-109
 */
double cap_ktc_noise_voltage(double temperature_k, double capacitance_f)
{
    if (temperature_k <= 0.0 || capacitance_f <= 0.0) {
        return 0.0;
    }
    return sqrt(CAP_KB * temperature_k / capacitance_f);
}

/**
 * cap_snr_limit_db
 *
 * Computes the maximum achievable SNR for capacitive sensing:
 *
 *   SNR = (delta_C * V_exc) / (v_n * sqrt(1/N))
 *   SNR_dB = 20 * log10(SNR)
 *
 * where:
 *   delta_C = capacitance change from touch [F]
 *   V_exc = excitation voltage [V]
 *   v_n = sqrt(k_B*T/C_total) = kT/C noise [V]
 *   N = number of averaged measurements
 *
 * Derivation: The signal is charge delta_Q = delta_C * V_exc.
 * The noise charge per measurement is q_n = C_total * v_n = sqrt(k_B*T*C_total).
 * After averaging N independent measurements, noise reduces by sqrt(N).
 * SNR = delta_Q / (q_n / sqrt(N)) = delta_C * V_exc * sqrt(N) / sqrt(k_B*T*C_total)
 *
 * Example: delta_C=100 fF, V_exc=3.3V, C_total=10pF, T=300K, N=100:
 *   v_n = 20.3 µV
 *   SNR = 100e-15 * 3.3 * 10 / sqrt(4.14e-21*10e-11)
 *       = 3.3e-13 * 10 / 6.43e-16 = 5132 → 74.2 dB
 *
 * This is the fundamental limit; real systems achieve 10-30 dB less
 * due to amplifier noise, quantization, and interference.
 */
double cap_snr_limit_db(double delta_c_f, double v_exc, double c_total_f,
                        double temperature_k, uint32_t n_samples)
{
    if (delta_c_f <= 0.0 || v_exc <= 0.0 || c_total_f <= 0.0 ||
        temperature_k <= 0.0 || n_samples == 0) {
        return 0.0;
    }

    double v_n = cap_ktc_noise_voltage(temperature_k, c_total_f);
    /* Charge SNR: signal_charge/noise_charge */
    double signal_charge = delta_c_f * v_exc;
    double noise_charge = v_n * c_total_f;
    double snr_linear = (signal_charge / noise_charge) * sqrt((double)n_samples);

    if (snr_linear <= 0.0) return 0.0;
    return 20.0 * log10(snr_linear);
}

/**
 * cap_min_resolvable_delta_c
 *
 * Computes the theoretical minimum capacitance change that can be
 * reliably detected, given a required SNR:
 *
 *   delta_C_min = C_total * v_n * SNR_req / (V_exc * sqrt(N))
 *
 * where v_n = sqrt(k_B*T/C_total).
 *
 * Simplifying: delta_C_min = sqrt(k_B*T*C_total) * SNR_req / (V_exc * sqrt(N))
 *
 * Example: C_total=10pF, T=300K, V_exc=3.3V, SNR_req=5 (14dB), N=100:
 *   delta_C_min = sqrt(4.14e-21*10e-11) * 5 / (3.3 * 10)
 *               = 2.03e-16 * 5 / 33 = 3.08e-17 = 0.0308 fF
 *
 * In practice, achieving sub-fF resolution requires:
 * - Sigma-delta CDC with OSR > 256
 * - Careful PCB layout (guard rings, shielding)
 * - External interference mitigation (50/60 Hz notch filter)
 * - Temperature compensation
 *
 * @return Minimum resolvable delta_C [F]
 */
double cap_min_resolvable_delta_c(double c_total_f, double temperature_k,
                                  double v_exc, double snr_required,
                                  uint32_t n_samples)
{
    if (c_total_f <= 0.0 || temperature_k <= 0.0 || v_exc <= 0.0 ||
        snr_required <= 0.0 || n_samples == 0) {
        return 0.0;
    }

    double v_n = cap_ktc_noise_voltage(temperature_k, c_total_f);
    /* noise charge = C * v_n = sqrt(k_B * T * C) */
    double noise_charge = c_total_f * v_n;
    double min_signal_charge = noise_charge * snr_required / sqrt((double)n_samples);

    return min_signal_charge / v_exc;
}

/**
 * cap_set_human_body_model
 *
 * Configures the human body model parameters for a channel.
 * These parameters affect the expected delta-C for touch detection
 * and proximity range estimation.
 *
 * Default HBM (IEC 61340-3-1):
 *   C_body = 150 pF (standing human to earth)
 *   R_body = 1500 Ohm (skin + internal resistance)
 *   Finger contact area = 1 cm² (adult index finger, light touch)
 */
void cap_set_human_body_model(cap_sensor_channel_t *chan,
                              const cap_human_body_model_t *hbm)
{
    if (!chan || !hbm) return;
    chan->self_meas.c_body_f = hbm->body_to_earth_f;
}
