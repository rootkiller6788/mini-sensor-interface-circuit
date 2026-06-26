/**
 * @file ina_core.h
 * @brief Instrumentation Amplifier Core Definitions
 *
 * Covers L1 Definitions and L2 Core Concepts for instrumentation amplifiers.
 * Reference: Sedra & Smith, "Microelectronic Circuits" (2020), Ch. 2, 8.
 *
 * Key concepts:
 *   - Common-Mode Rejection Ratio (CMRR)
 *   - Differential-mode gain (Ad) vs common-mode gain (Acm)
 *   - Input impedance (differential and common-mode)
 *   - Offset voltage and drift
 *   - Noise models (thermal, flicker, shot)
 *   - Gain-bandwidth product (GBW)
 *   - Slew rate
 *   - Power Supply Rejection Ratio (PSRR)
 *   - Nonlinearity (INL, DNL)
 *
 * @course-alignment
 *   MIT 6.002 Circuits & Electronics: Op-amp circuits, differential amplifiers
 *   Berkeley EE105 Analog IC: Differential pair, CMRR analysis
 *   Stanford EE101B Circuits II: Instrumentation amplifiers
 *   ETH Zurich 227-0455: Precision analog design
 */
#ifndef INA_CORE_H
#define INA_CORE_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*---------------------------------------------------------------------------
 * L1: Core Definitions - Data Types and Constants
 *---------------------------------------------------------------------------*/

/** Op-amp type enumeration for topology selection */
typedef enum {
    INA_OPAMP_IDEAL      = 0,
    INA_OPAMP_REAL       = 1,
    INA_OPAMP_PRECISION  = 2,
    INA_OPAMP_HIGH_SPEED = 3,
    INA_OPAMP_LOW_NOISE  = 4,
    INA_OPAMP_RAIL_RAIL  = 5,
    INA_OPAMP_CHOPPER    = 6,
    INA_OPAMP_AUTOZERO   = 7
} InaOpampType;

/** Instrumentation amplifier topology enumeration */
typedef enum {
    INA_TOPOLOGY_3OPAMP = 0,
    INA_TOPOLOGY_2OPAMP = 1,
    INA_TOPOLOGY_CURRENT_MODE = 2,
    INA_TOPOLOGY_FLYING_CAP = 3,
    INA_TOPOLOGY_INDIRECT_CURRENT = 4
} InaTopology;

/** Gain configuration mode */
typedef enum {
    INA_GAIN_FIXED     = 0,
    INA_GAIN_RESISTOR  = 1,
    INA_GAIN_PROGRAMMABLE = 2,
    INA_GAIN_PIN_STRAP = 3
} InaGainMode;

/**
 * @brief Core instrumentation amplifier parameters structure
 *
 * This structure captures all first-order specifications of an
 * instrumentation amplifier, modeled after datasheet parameters
 * of commercial devices (e.g., AD620, INA128, AD8421).
 */
typedef struct {
    double gain;
    double gain_min;
    double gain_max;
    double gain_error_percent;
    double gain_nonlinearity_ppm;
    double gain_tc_ppm_per_C;
    double cmrr_db;
    double cmrr_at_60hz_db;
    double cmrr_at_gain_1_db;
    double cmrr_at_gain_100_db;
    double cmrr_at_gain_1000_db;
    double zin_differential_ohm;
    double zin_common_mode_ohm;
    double zin_differential_pf;
    double zin_common_mode_pf;
    double vos_uv;
    double vos_drift_nv_per_C;
    double vos_vs_time_nv_per_sqrt_khr;
    double vos_psrr_db;
    double vos_cmrr_db;
    double en_nv_per_sqrt_hz;
    double en_0p1_to_10hz_uv_pp;
    double in_pa_per_sqrt_hz;
    double en_flicker_corner_hz;
    double en_rti_at_gain1_nv_rms;
    double bandwidth_khz;
    double slew_rate_v_per_us;
    double settling_time_us;
    double settling_time_to_0p001_us;
    double overload_recovery_us;
    double supply_voltage_min;
    double supply_voltage_max;
    double supply_current_ma;
    double psrr_plus_db;
    double psrr_minus_db;
    double input_common_mode_min;
    double input_common_mode_max;
    double input_diff_mode_max;
    double output_swing_min;
    double output_swing_max;
    double temp_range_min_C;
    double temp_range_max_C;
    double thermal_resistance_jc;
} InaParameters;

/**
 * @brief Complete DC error budget for an IA
 *
 * All errors are referred to input (RTI) unless noted.
 * Used for L1 definition + L6 canonical problem solving.
 */
typedef struct {
    double vos_error_uv;
    double vos_drift_error_uv;
    double gain_error_uv;
    double cmrr_error_uv;
    double psrr_error_uv;
    double noise_error_uv_rms;
    double nonlinearity_error_uv;
    double ib_offset_error_uv;
    double total_error_rss_uv;
    double total_error_worst_uv;
    double effective_resolution_bits;
} InaErrorBudget;

/**
 * @brief Differential and common-mode signal representation
 *
 * For inputs V1, V2:
 *   Vdm = V1 - V2
 *   Vcm = (V1 + V2)/2
 *   Vout = Ad * Vdm + Acm * Vcm
 */
typedef struct {
    double v_plus;
    double v_minus;
    double v_dm;
    double v_cm;
    double v_ref;
    double v_out;
} InaSignal;

/**
 * @brief Common-mode rejection model
 *
 * CMRR is defined as CMRR = |Ad / Acm|, often expressed in dB.
 *
 * CMRR_total = -20*log10( sqrt( (1/CMRR_res)^2 + (1/(G*CMRR_opamp))^2 ) )
 */
typedef struct {
    double cmrr_resistor_db;
    double cmrr_opamp_db;
    double cmrr_total_db;
    double resistor_mismatch_percent;
    double gain;
    double frequency;
} InaCmrrModel;

/** Noise type enumeration for spectral analysis */
typedef enum {
    INA_NOISE_WHITE = 0,
    INA_NOISE_FLICKER = 1,
    INA_NOISE_SHOT = 2,
    INA_NOISE_BURST = 3,
    INA_NOISE_AVALANCHE = 4
} InaNoiseType;

/**
 * @brief Noise spectral density model
 *
 * Total noise voltage spectral density:
 *   en_total(f) = sqrt( en_white^2 + (en_flicker * fc/f)^2 + en_shot^2 )
 *
 * Reference: Motchenbacher & Connelly, "Low-Noise Electronic System Design"
 */
typedef struct {
    InaNoiseType type;
    double en_white_nv_per_sqrt_hz;
    double fc_hz;
    double en_at_1hz_nv;
    double en_at_10hz_nv;
    double en_at_100hz_nv;
    double en_at_1khz_nv;
    double en_at_10khz_nv;
    double bandwidth_low_hz;
    double bandwidth_high_hz;
} InaNoiseModel;

/**
 * @brief Complex frequency-domain transfer function representation
 *
 * H(s) = N(s)/D(s) where s = j*omega = j*2*pi*f
 * For a 3-op-amp IA with first-order op-amp model:
 *   H(s) = G * omega_a / (s + omega_a/G)
 */
typedef struct {
    double gain_dc;
    double pole_1_hz;
    double pole_2_hz;
    double zero_1_hz;
    double bandwidth_3db_hz;
    double gain_bandwidth_hz;
    double phase_margin_deg;
    double quality_factor;
    int    num_poles;
    int    num_zeros;
} InaTransferFunction;

/*---------------------------------------------------------------------------
 * L4: Fundamental Laws - Signal Decomposition
 *---------------------------------------------------------------------------*/

void ina_decompose_signal(double v_plus, double v_minus,
                          double *v_dm, double *v_cm);

void ina_recompose_signal(double v_dm, double v_cm,
                          double *v_plus, double *v_minus);

/*---------------------------------------------------------------------------
 * L4: CMRR Calculation
 *---------------------------------------------------------------------------*/

double ina_cmrr_from_gains(double ad, double acm);

double ina_cmrr_from_resistor_mismatch(double r2_over_r1, double delta_r);

double ina_cmrr_total(double cmrr_resistor, double cmrr_opamp,
                      double cmrr_layout);

double ina_cmrr_vs_frequency(double cmrr_dc, double frequency,
                             double f_cmrr_pole);

/*---------------------------------------------------------------------------
 * L1/L2: Offset Voltage and Drift
 *---------------------------------------------------------------------------*/

double ina_offset_at_temperature(double vos_25c, double tc_vos_nv_per_c,
                                 double temperature_c,
                                 double tc2_vos_nv_per_c2);

double ina_offset_drift_over_range(double drift_nv_per_c,
                                   double delta_t_c);

/*---------------------------------------------------------------------------
 * L3: Noise Calculations
 *---------------------------------------------------------------------------*/

double ina_rms_noise(double en_noise_density, double f_low, double f_high,
                     double f_corner);

double ina_rms_to_peak_to_peak(double vn_rms, double crest_factor);

double ina_noise_figure(double en_nv_per_sqrt_hz, double source_resistance,
                        double temperature_k, double bandwidth_hz);

/*---------------------------------------------------------------------------
 * L1: Signal-to-Noise Ratio and Dynamic Range
 *---------------------------------------------------------------------------*/

double ina_snr_db(double vsignal_rms, double vnoise_rms);

double ina_enob_from_snr(double snr_db);

double ina_dynamic_range_db(double v_full_scale, double v_noise_floor);

double ina_minimum_detectable_signal(double noise_floor_v, double snr_min);

/*---------------------------------------------------------------------------
 * L5: Error Budget Analysis (Algorithm)
 *---------------------------------------------------------------------------*/

InaErrorBudget ina_compute_error_budget(const InaParameters *params,
                                         double vcm,
                                         double vsupply_deviation,
                                         double temperature,
                                         double rs_source);

/*---------------------------------------------------------------------------
 * L6: Gain Setting Resistor Calculation
 *---------------------------------------------------------------------------*/

double ina_calculate_rg(double desired_gain, double r_internal);

double ina_calculate_gain_from_rg(double rg, double r_internal);

double ina_nearest_standard_resistor(double target_r, int series);

double ina_gain_error_from_rg_tolerance(double rg_nominal,
                                         double rg_tolerance_percent,
                                         double r_internal);

/*---------------------------------------------------------------------------
 * L5: Input Bias Current Return Path Verification
 *---------------------------------------------------------------------------*/

double ina_max_source_resistance(double ib_amps, double allowed_error_uv);

double ina_ib_offset(double ib_plus, double ib_minus,
                     double rs_plus, double rs_minus);

#endif /* INA_CORE_H */
