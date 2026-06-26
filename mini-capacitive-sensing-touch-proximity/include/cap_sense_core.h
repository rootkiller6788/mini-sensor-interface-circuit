/**
 * @file cap_sense_core.h
 * @brief Core Capacitive Sensing Definitions and Data Structures
 *
 * Capacitive sensing detects changes in capacitance caused by human body
 * proximity or touch. The human body acts as a conductor coupling to earth
 * ground (C_body ~ 100-300 pF), shifting the capacitance of a sensor electrode.
 *
 * Knowledge Coverage:
 *   L1 Definitions:  self-capacitance, mutual-capacitance, baseline, delta-C,
 *                    SNR, resolution, sensitivity, scan rate
 *   L2 Concepts:     charge transfer, projected capacitance, differential sensing,
 *                    auto-calibration, touch vs proximity discrimination
 *   L3 Math:         parallel-plate model, fringe field correction, RC dynamics,
 *                    electrode-to-ground coupling models
 *   L4 Laws:         Coulomb's Law, Gauss's Law, Johnson-Nyquist noise,
 *                    kT/C noise limit, charge conservation
 *   L5 Algorithms:   baseline tracking, threshold adaptation, auto-calibration
 *                    sequence, charge-counting method
 *   L6 Problems:     single-touch detection, water rejection, glove handling
 *
 * References:
 *   - Baxter, L.K. "Capacitive Sensors: Design and Applications" (1997)
 *   - Cypress AN64846 "Getting Started with CapSense"
 *   - TI SNOA927 "Capacitive Sensing: Which Architecture Should You Choose?"
 *   - Microchip AN2934 "Capacitive Touch Using Only an ADC"
 *   - Atmel QTAN0079 "Buttons, Sliders and Wheels"
 */

#ifndef CAP_SENSE_CORE_H
#define CAP_SENSE_CORE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ==========================================================================
 * L0: PHYSICAL CONSTANTS
 * ========================================================================== */

/** Vacuum permittivity eps0 [F/m] = 8.854187817e-12 */
#define CAP_EPSILON0     8.854187817e-12
/** Boltzmann constant kB [J/K] = 1.380649e-23 (SI 2019) */
#define CAP_KB           1.380649e-23
/** Electron charge q [C] = 1.602176634e-19 (SI 2019) */
#define CAP_QE           1.602176634e-19
/** Absolute zero in Celsius */
#define CAP_TABS         273.15

/* ==========================================================================
 * L1: ENUMERATED TYPES
 * ========================================================================== */

/** Capacitive sensing measurement method */
typedef enum {
    CAP_METHOD_CHARGE_TRANSFER,  /* Switch C between VDD and sample cap */
    CAP_METHOD_SIGMA_DELTA,      /* C in SigmaDelta modulator feedback */
    CAP_METHOD_RELAXATION_OSC,   /* RC oscillator frequency shift */
    CAP_METHOD_DUAL_SLOPE,       /* Charge to Vref, discharge timing */
    CAP_METHOD_AC_BRIDGE,        /* Balanced bridge imbalance current */
    CAP_METHOD_RESONANT_SHIFT,   /* LC tank resonance aF/sqrt(Hz) */
    CAP_METHOD_COUNT
} cap_measurement_method_t;

/** Sensing mode */
typedef enum {
    CAP_SENSE_SELF,     /* Electrode to earth ground */
    CAP_SENSE_MUTUAL,   /* TX driven, RX sensed */
    CAP_SENSE_ABSOLUTE  /* Direct absolute measurement */
} cap_sense_mode_t;

/** Touch state machine states */
typedef enum {
    TOUCH_IDLE,       /* No touch, baseline tracking */
    TOUCH_APPROACH,   /* Proximity zone entered */
    TOUCH_DETECT,     /* deltaC > threshold, debounce started */
    TOUCH_ACTIVE,     /* Touch confirmed */
    TOUCH_RELEASE,    /* deltaC falling, release debounce */
    TOUCH_HOLD        /* Long-duration touch */
} touch_state_t;

/** Electrode shape */
typedef enum {
    ELEC_DISC,      /* Circular pad, radial symmetry */
    ELEC_RECT,      /* Rectangular pad */
    ELEC_RING,      /* Ring electrode, reduced dead zone */
    ELEC_INTERDIG,  /* Interdigitated TX/RX fingers */
    ELEC_SLIDER,    /* Adjacent segments for linear position */
    ELEC_WHEEL      /* Angular segments for rotary position */
} electrode_shape_t;

/** Noise source classification */
typedef enum {
    NOISE_MAINS,        /* 50/60 Hz power line */
    NOISE_SWITCHING,    /* DC-DC converter, LED PWM */
    NOISE_RF,           /* WiFi/BT/GSM interference */
    NOISE_THERMAL,      /* Johnson-Nyquist irreducible */
    NOISE_1F,           /* Flicker noise in amplifier */
    NOISE_CHARGE_INJ    /* MOSFET charge injection */
} cap_noise_source_t;

/* ==========================================================================
 * L1: CORE DATA STRUCTURES
 * ========================================================================== */

/** Single capacitive sensor electrode physical model */
typedef struct {
    electrode_shape_t shape;
    double  electrode_area_m2;
    double  perimeter_m;
    double  overlay_thickness_m;
    double  overlay_epsilon_r;
    double  pcb_thickness_m;
    double  pcb_epsilon_r;
    double  trace_length_m;
    double  trace_width_m;
    double  ground_gap_m;
    bool    has_hatched_ground;
    double  hatch_fill_ratio;
    bool    has_guard_ring;
    double  guard_ring_gap_m;
} cap_electrode_phys_t;

/** Self-capacitance measurement model */
typedef struct {
    double  c_parasitic_f;
    double  c_body_f;
    double  c_finger_f;
    double  c_total_f;
    double  c_baseline_f;
    double  delta_c_f;
    double  noise_rms_f;
    double  snr_db;
    uint32_t raw_count;
    uint32_t baseline_count;
} cap_self_meas_t;

/** Mutual-capacitance measurement model */
typedef struct {
    double  c_mutual_nominal_f;
    double  c_tx_to_finger_f;
    double  c_finger_to_rx_f;
    double  c_finger_to_gnd_f;
    double  c_mutual_effective_f;
    double  delta_c_f;
    int16_t raw_diff;
    int16_t baseline_diff;
    double  noise_rms_f;
    double  snr_db;
} cap_mutual_meas_t;

/** Complete capacitive sensor channel */
typedef struct {
    uint8_t              channel_id;
    cap_measurement_method_t method;
    cap_sense_mode_t     mode;
    cap_electrode_phys_t electrode;
    cap_self_meas_t      self_meas;
    cap_mutual_meas_t    mutual_meas;
    touch_state_t        state;
    uint32_t             state_entry_ms;
    uint8_t              debounce_count;
    double               threshold_f;
    double               hysteresis_f;
    bool                 is_proximity;
    uint32_t             sample_count;
    uint32_t             last_scan_us;
} cap_sensor_channel_t;

/** Multi-channel capacitive sensor system */
typedef struct {
    uint8_t              num_channels;
    cap_sensor_channel_t *channels;
    double               v_excitation;
    double               v_ref;
    uint32_t             scan_period_us;
    double               c_internal_ref_f;
    double               resolution_f;
    bool                 auto_cal_enabled;
    bool                 low_power_mode;
    uint32_t             uptime_ms;
    double               temperature_c;
    double               humidity_pct;
    uint32_t             total_scans;
    uint64_t             total_charge_transfers;
} cap_sensor_system_t;

/** Calibration parameters */
typedef struct {
    uint32_t raw_baseline_min;
    uint32_t raw_baseline_max;
    uint32_t raw_baseline_mean;
    uint32_t raw_noise_peak;
    double   raw_noise_rms;
    double   gain_trim;
    double   offset_trim;
    double   temp_at_cal_c;
    double   temp_coeff_ppm;
    bool     cal_valid;
} cap_calibration_t;

/* ==========================================================================
 * L1: HUMAN BODY MODEL
 * ========================================================================== */

/** Human Body Model (HBM) for capacitive touch, IEC 61340-3-1 */
typedef struct {
    double body_to_earth_f;
    double finger_to_electrode_f;
    double body_resistance_ohm;
    double finger_contact_area_m2;
    double finger_ridge_gap_m;
} cap_human_body_model_t;

/* ==========================================================================
 * L3: FIELD MODEL STRUCTURES
 * ========================================================================== */

/** Parallel-plate capacitance model: C = eps0 * eps_r * A / d */
typedef struct {
    double area_m2;
    double separation_m;
    double epsilon_r;
    double c_parallel_plate_f;
    double fringe_factor;
    double c_total_f;
} cap_parallel_plate_model_t;

/** Coplanar electrode mutual capacitance model */
typedef struct {
    double electrode_width_m;
    double electrode_length_m;
    double gap_m;
    double substrate_thickness_m;
    double substrate_epsilon_r;
    double overlay_thickness_m;
    double overlay_epsilon_r;
    double c_mutual_f;
} cap_coplanar_mutual_model_t;

/** Shield / guard driver model */
typedef struct {
    bool    enabled;
    double  gain;
    double  gain_error_pct;
    double  phase_error_deg;
    double  bandwidth_hz;
    double  output_impedance_ohm;
    double  c_parasitic_original_f;
    double  c_parasitic_shielded_f;
    double  reduction_ratio;
} cap_shield_driver_t;

/* ==========================================================================
 * L4: FUNDAMENTAL LIMITS STRUCTURES
 * ========================================================================== */

/** kT/C noise limit */
typedef struct {
    double temperature_k;
    double c_sense_f;
    double v_noise_rms;
    double c_resolution_f;
    double n_samples;
    double c_resolution_avg_f;
} cap_ktc_noise_limit_t;

/** SNR fundamental limit for capacitive sensing */
typedef struct {
    double v_excitation;
    double c_sense_f;
    double temperature_k;
    double bandwidth_hz;
    double snr_max_db;
    double c_min_resolvable_f;
} cap_snr_limit_t;

/* ==========================================================================
 * L2-L5: CORE API DECLARATIONS
 * ========================================================================== */

int cap_sensor_system_init(cap_sensor_system_t *sys, uint8_t num_ch,
                           double v_exc, double c_ref);

int cap_channel_configure(cap_sensor_channel_t *chan,
                          cap_measurement_method_t method,
                          cap_sense_mode_t mode,
                          electrode_shape_t el_shape,
                          double el_area, double overlay_t,
                          double overlay_er, double threshold_f);

double cap_parallel_plate_c(double area, double distance, double epsilon_r);

double cap_body_to_earth_c(double body_height_m, double body_mass_kg,
                           double shoe_sole_thickness_m);

double cap_finger_electrode_c(double area_contact, double overlay_thick,
                              double overlay_er, double fringe_factor);

double cap_ktc_noise_voltage(double temperature_k, double capacitance_f);

double cap_snr_limit_db(double delta_c_f, double v_exc, double c_total_f,
                        double temperature_k, uint32_t n_samples);

double cap_min_resolvable_delta_c(double c_total_f, double temperature_k,
                                  double v_exc, double snr_required,
                                  uint32_t n_samples);

void cap_set_human_body_model(cap_sensor_channel_t *chan,
                              const cap_human_body_model_t *hbm);

void cap_sensor_system_destroy(cap_sensor_system_t *sys);

#endif /* CAP_SENSE_CORE_H */
