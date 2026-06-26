/**
 * @file    thermocouple_cjc_rtd.h
 * @brief   Thermocouple Cold-Junction Compensation and RTD Sensor Interface Library
 *
 * Knowledge Coverage:
 *   L1 (Definitions):  Thermocouple types, RTD types, CJC configurations,
 *                       measurement error codes, wiring topologies
 *   L2 (Core Concepts): Seebeck effect, Callendar-Van Dusen equation,
 *                       cold-junction compensation principle, 4-wire measurement
 *   L3 (Math Structures): Polynomial/rational function approximations,
 *                         piecewise interpolation, Newton-Raphson inversion
 *   L4 (Fundamental Laws): ITS-90 standard, IEC 60751, Law of Intermediate
 *                          Metals, Law of Successive Temperatures
 *
 * Reference:
 *   NIST ITS-90 Thermocouple Database (SRD 60)
 *   IEC 60751:2008 Industrial Platinum Resistance Thermometers
 *   ASTM E230/E230M Standard Specification for Temperature-Electromotive Force
 *   Burns, G.W. et al., "Temperature-Electromotive Force Reference Functions"
 *     NIST Monograph 175 (1993)
 *   Callendar, H.L., "On the Practical Measurement of Temperature" (1887)
 *   Van Dusen, M.S., "Platinum-Resistance Thermometry" (1925)
 */

#ifndef THERMOCOUPLE_CJC_RTD_H
#define THERMOCOUPLE_CJC_RTD_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1: Thermocouple Type Definitions
 * ========================================================================= */

/**
 * @brief Thermocouple type enumeration per ASTM E230 / IEC 60584
 *
 * Each type is defined by its positive (KP) and negative (KN) thermoelement
 * composition. The Seebeck coefficient S(T) = dV/dT varies with temperature
 * and junction composition.
 *
 * Type K (Chromel-Alumel): Ni-Cr (+) / Ni-Al (-)
 *   Range: -270 to +1372 C, Sensitivity ~41 uV/C at 25C
 *   Most common general-purpose thermocouple
 *
 * Type J (Iron-Constantan): Fe (+) / Cu-Ni (-)
 *   Range: -210 to +1200 C, Sensitivity ~52 uV/C at 25C
 *   Limited to non-oxidizing atmospheres below 540C
 *
 * Type T (Copper-Constantan): Cu (+) / Cu-Ni (-)
 *   Range: -270 to +400 C, Sensitivity ~41 uV/C at 25C
 *   Best accuracy of base-metal thermocouples
 *
 * Type E (Chromel-Constantan): Ni-Cr (+) / Cu-Ni (-)
 *   Range: -270 to +1000 C, Sensitivity ~68 uV/C at 25C
 *   Highest EMF output per degree of all standard types
 *
 * Type N (Nicrosil-Nisil): Ni-Cr-Si (+) / Ni-Si-Mg (-)
 *   Range: -270 to +1300 C, Sensitivity ~27 uV/C at 25C
 *   Improved stability over Type K at high temperatures
 *
 * Type R (Pt-13%Rh vs Pt): Pt-13%Rh (+) / Pt (-)
 *   Range: -50 to +1768 C, Sensitivity ~10 uV/C at 25C
 *   Noble metal, high temperature, excellent accuracy
 *
 * Type S (Pt-10%Rh vs Pt): Pt-10%Rh (+) / Pt (-)
 *   Range: -50 to +1768 C, Sensitivity ~10 uV/C at 25C
 *   Standard defining the ITS-90 from 630.615C to 1064.18C
 *
 * Type B (Pt-30%Rh vs Pt-6%Rh): Pt-30%Rh (+) / Pt-6%Rh (-)
 *   Range: 0 to +1820 C, Sensitivity ~3 uV/C at 25C
 *   Highest temperature range; reference junction can be at ambient
 *
 * Type C (W-5%Re vs W-26%Re): Refractory for ultra-high temperature
 *   Range: 0 to +2315 C, Sensitivity ~14 uV/C at 25C
 */
typedef enum {
    TC_TYPE_K = 0,  /** Chromel-Alumel (Ni-Cr / Ni-Al) */
    TC_TYPE_J = 1,  /** Iron-Constantan (Fe / Cu-Ni) */
    TC_TYPE_T = 2,  /** Copper-Constantan (Cu / Cu-Ni) */
    TC_TYPE_E = 3,  /** Chromel-Constantan (Ni-Cr / Cu-Ni) */
    TC_TYPE_N = 4,  /** Nicrosil-Nisil (Ni-Cr-Si / Ni-Si-Mg) */
    TC_TYPE_R = 5,  /** Pt-13%Rh vs Pt */
    TC_TYPE_S = 6,  /** Pt-10%Rh vs Pt */
    TC_TYPE_B = 7,  /** Pt-30%Rh vs Pt-6%Rh */
    TC_TYPE_C = 8,  /** W-5%Re vs W-26%Re */
    TC_COUNT   = 9
} tc_type_t;

/* =========================================================================
 * L1: RTD Type Definitions
 * ========================================================================= */

/**
 * @brief RTD type enumeration per IEC 60751
 *
 * Resistance Temperature Detectors exploit the predictable change in
 * electrical resistance with temperature. The Callendar-Van Dusen equation
 * relates resistance to temperature.
 *
 * For T >= 0: R(T) = R0 * (1 + A*T + B*T^2)
 * For T < 0:  R(T) = R0 * (1 + A*T + B*T^2 + C*(T-100)*T^3)
 *
 * Standard alpha per IEC 60751: alpha = (R100 - R0) / (100 * R0) = 0.00385055
 */
typedef enum {
    RTD_TYPE_PT100    = 0,   /** Platinum 100 ohm at 0C (IEC 60751) */
    RTD_TYPE_PT200    = 1,   /** Platinum 200 ohm at 0C */
    RTD_TYPE_PT500    = 2,   /** Platinum 500 ohm at 0C */
    RTD_TYPE_PT1000   = 3,   /** Platinum 1000 ohm at 0C */
    RTD_TYPE_PT2000   = 4,   /** Platinum 2000 ohm at 0C */
    RTD_TYPE_NI100    = 5,   /** Nickel 100 ohm at 0C (DIN 43760) */
    RTD_TYPE_NI120    = 6,   /** Nickel 120 ohm at 0C */
    RTD_TYPE_NI1000   = 7,   /** Nickel 1000 ohm at 0C */
    RTD_TYPE_CU10     = 8,   /** Copper 10 ohm at 25C */
    RTD_TYPE_CU100    = 9,   /** Copper 100 ohm at 0C */
    RTD_COUNT         = 10
} rtd_type_t;

/**
 * @brief RTD alpha coefficient standard
 * The temperature coefficient of resistance alpha defines how rapidly
 * resistance changes with temperature near 0C.
 */
typedef enum {
    RTD_ALPHA_IEC_385  = 0,  /** IEC 60751: 0.00385055 */
    RTD_ALPHA_US_392   = 1,  /** US Industrial: 0.003916 */
    RTD_ALPHA_JP_391   = 2,  /** JIS C1604: 0.003916 */
    RTD_ALPHA_US_390   = 3,  /** Old US: 0.003920 */
    RTD_ALPHA_CUSTOM   = 4   /** User-specified alpha */
} rtd_alpha_standard_t;

/* =========================================================================
 * L1: Cold-Junction Compensation (CJC) Configuration
 * ========================================================================= */

/**
 * @brief CJC sensor type for measuring the reference junction temperature
 *
 * The cold (reference) junction is where the thermocouple wires connect
 * to copper in the measurement system. To compensate for the reference
 * junction not being at 0C, its temperature must be accurately measured.
 */
typedef enum {
    CJC_SENSOR_RTD       = 0,  /** RTD sensor (highest accuracy) */
    CJC_SENSOR_THERMISTOR = 1, /** NTC/PTC thermistor */
    CJC_SENSOR_DIODE     = 2,  /** Silicon diode (Vf ~ 2 mV/C) */
    CJC_SENSOR_IC        = 3,  /** Integrated temperature sensor (LM35 etc) */
    CJC_SENSOR_FIXED     = 4,  /** Fixed reference temperature (ice bath) */
    CJC_SENSOR_NONE      = 5   /** No compensation (assumes 0C reference) */
} cjc_sensor_type_t;

/* =========================================================================
 * L1: Wiring Configuration for RTD Measurement
 * ========================================================================= */

/**
 * @brief RTD wiring topology
 *
 * 2-wire: Simplest but lead resistance adds error directly.
 *   R_measured = R_RTD + 2*R_lead
 *   ~2.6C error per ohm of lead with Pt100
 *
 * 3-wire: Compensates one lead if leads are matched.
 *   Common in industrial applications (adequate accuracy, lower cost)
 *
 * 4-wire: Kelvin connection, eliminates lead resistance entirely.
 *   Force current through one pair, sense voltage through another.
 *   Used for precision laboratory measurements.
 */
typedef enum {
    WIRE_2_WIRE = 2,   /** Two-wire */
    WIRE_3_WIRE = 3,   /** Three-wire: industrial compromise */
    WIRE_4_WIRE = 4    /** Four-wire: Kelvin measurement (precision) */
} rtd_wiring_t;

/* =========================================================================
 * L1: Measurement Error Codes
 * ========================================================================= */

/** Sensor measurement error codes */
typedef enum {
    TC_OK                    =  0,  /** No error */
    TC_ERR_NULL_POINTER      = -1,  /** NULL pointer argument */
    TC_ERR_INVALID_TYPE      = -2,  /** Unrecognized T/C or RTD type */
    TC_ERR_OUT_OF_RANGE      = -3,  /** Temperature outside valid range */
    TC_ERR_VOLTAGE_RANGE     = -4,  /** Voltage outside valid range */
    TC_ERR_RESISTANCE_RANGE  = -5,  /** Resistance outside expected range */
    TC_ERR_OPEN_CIRCUIT      = -6,  /** Open thermocouple or RTD detected */
    TC_ERR_SHORT_CIRCUIT     = -7,  /** Short circuit detected */
    TC_ERR_CONVERGENCE       = -8,  /** Newton solver failed to converge */
    TC_ERR_SELF_HEATING      = -9,  /** Excessive self-heating detected */
    TC_ERR_CJC_RANGE         = -10, /** CJC temperature out of bounds */
    TC_ERR_ADC_SATURATION    = -11, /** ADC input saturated */
    TC_ERR_CALIBRATION       = -12, /** Calibration data invalid */
    TC_ERR_INTERPOLATION     = -13  /** Interpolation failure */
} tc_error_t;

/** Return human-readable error string */
const char *tc_error_string(tc_error_t err);

/* =========================================================================
 * L1: Polynomial Coefficient Data Structures
 * ========================================================================= */

/**
 * @brief ITS-90 polynomial coefficient set for thermocouple conversion
 *
 * E(T) = sum_{i=0}^{N-1} c[i] * T^i  for T in [t_low, t_high]
 * Reference: NIST Monograph 175
 */
typedef struct {
    double   t_low;      /** Lower bound of temperature range [C] */
    double   t_high;     /** Upper bound of temperature range [C] */
    size_t   n_coeffs;   /** Number of polynomial coefficients */
    double  *coeffs;     /** Polynomial coefficients c[0]..c[n-1] */
    double   error_max;  /** Maximum fitting error in this range [C] */
} tc_polynomial_t;

/**
 * @brief Full thermocouple conversion table
 * A thermocouple type may have multiple polynomial subranges.
 */
typedef struct {
    tc_type_t    type;             /** Thermocouple type */
    const char  *name;             /** Human-readable name */
    double       temp_min;         /** Minimum valid temperature [C] */
    double       temp_max;         /** Maximum valid temperature [C] */
    double       emf_min;          /** Minimum valid EMF [mV] */
    double       emf_max;          /** Maximum valid EMF [mV] */
    double       seebeck_typical;  /** Typical Seebeck at 25C [uV/C] */
    size_t       n_forward_ranges; /** Number of forward polynomial ranges */
    size_t       n_inverse_ranges; /** Number of inverse polynomial ranges */
    const tc_polynomial_t **forward_poly;  /** Forward: T -> EMF */
    const tc_polynomial_t **inverse_poly;  /** Inverse: EMF -> T */
} tc_conversion_table_t;

/* =========================================================================
 * L1: RTD Coefficients (Callendar-Van Dusen)
 * ========================================================================= */

/**
 * @brief Callendar-Van Dusen coefficients for RTD
 *
 * CVD equation:
 *   T >= 0: R(T) = R0*(1 + A*T + B*T^2)
 *   T <  0: R(T) = R0*(1 + A*T + B*T^2 + C*(T-100)*T^3)
 *
 * IEC 60751 standard platinum (alpha = 0.00385055):
 *   A = 3.9083e-3, B = -5.775e-7, C = -4.183e-12
 */
typedef struct {
    double r0;      /** Resistance at 0C [ohms] */
    double a;       /** Callendar coefficient A [/C] */
    double b;       /** Callendar coefficient B [/C^2] */
    double c;       /** Van Dusen coefficient C [/C^4] */
    double alpha;   /** Temperature coefficient alpha [/C] */
    double beta;    /** Alternative form beta [/C^2] */
    double delta;   /** Alternative form delta */
} rtd_cvd_coeffs_t;

/* =========================================================================
 * L1: CJC Configuration Structure
 * ========================================================================= */

/**
 * @brief Cold-junction compensation configuration
 *
 * E_corrected(T) = E_measured + E(T_cj, 0)
 * where E(T_cj, 0) = the EMF produced by the T/C with hot junction
 * at T_cj and cold junction at 0C.
 */
typedef struct {
    cjc_sensor_type_t sensor_type;       /** Type of CJC sensor */
    double            cj_temperature;    /** Measured cold-junction temp [C] */
    double            cj_temperature_uncertainty; /** CJC uncertainty [C] */
    double            cj_resistance;     /** Sensor resistance if RTD [ohms] */
    size_t            cj_adc_bits;       /** ADC resolution for CJC [bits] */
    double            cj_vref;           /** ADC reference voltage for CJC [V] */
    double            cj_excitation;     /** RTD excitation current [A] */
    double            pcb_gradient;      /** PCB thermal gradient [C] */
    double            pcb_drift_rate;    /** PCB temperature drift rate [C/min] */
    int               auto_calibrate;    /** Enable periodic auto-calibration */
    size_t            calibration_interval; /** Seconds between calibrations */
} cjc_config_t;

/* =========================================================================
 * L1: Measurement System Configuration and Results
 * ========================================================================= */

typedef struct {
    tc_type_t        tc_type;           /** Thermocouple type */
    rtd_wiring_t     wiring;            /** Wiring configuration */
    double           adc_vref;          /** ADC reference voltage [V] */
    size_t           adc_bits;          /** ADC resolution [bits] */
    double           adc_vmin;          /** ADC minimum input [V] */
    double           adc_vmax;          /** ADC maximum input [V] */
    double           pga_gain;          /** PGA gain */
    int              pga_enabled;       /** PGA enable flag */
    cjc_config_t     cjc;               /** CJC configuration */
    double           filter_alpha;      /** LPF alpha (0-1) */
    size_t           filter_window;     /** Moving average window size */
    double           filter_cutoff_hz;  /** Filter cutoff [Hz] */
    double           sample_rate_hz;    /** Sample rate [Hz] */
    double           open_tc_threshold; /** Open-TC detect threshold [V] */
    double           burnout_current;   /** Burnout detection current [A] */
    int              burnout_enabled;   /** Enable burnout detection */
    double           gain_cal;          /** Gain calibration factor */
    double           offset_cal;        /** Offset calibration [V] */
    double           cal_date;          /** Calibration date (YYYYMMDD) */
    size_t           cal_interval_days; /** Recalibration interval */
} tc_measurement_config_t;

typedef struct {
    double           temperature;       /** Measured temperature [C] */
    double           emf_raw;           /** Raw thermocouple EMF [mV] */
    double           emf_compensated;   /** CJC-compensated EMF [mV] */
    double           cj_temperature;    /** Cold-junction temperature [C] */
    double           uncertainty;       /** Combined uncertainty [C] */
    double           resolution;        /** ADC resolution in temperature [C] */
    tc_error_t       error;             /** Error code */
    size_t           sample_count;      /** Samples averaged */
    double           self_heating_err;  /** Self-heating error [C] */
    double           linearity_err;     /** Linearity error [C] */
} tc_measurement_t;

typedef struct {
    double           resistance;        /** Measured resistance [ohms] */
    double           temperature;       /** Computed temperature [C] */
    double           uncertainty;       /** Temperature uncertainty [C] */
    double           lead_resistance;   /** Estimated lead resistance [ohms] */
    double           excitation_current; /** Measurement current [A] */
    double           power_dissipation; /** Power in RTD element [W] */
    double           self_heating;      /** Self-heating temperature rise [C] */
    tc_error_t       error;             /** Error code */
    rtd_wiring_t     wiring;            /** Wiring configuration used */
    size_t           adc_samples;       /** ADC samples averaged */
} rtd_measurement_t;

/* =========================================================================
 * L2: Seebeck Effect Fundamentals
 * ========================================================================= */

typedef struct {
    double   temperature;         /** Temperature [C] */
    double   seebeck_absolute;    /** Absolute Seebeck coefficient [uV/K] */
    double   seebeck_relative;    /** Relative (pair) Seebeck [uV/K] */
    double   tc_sensitivity;      /** Thermocouple sensitivity dE/dT [uV/K] */
    double   linearity_deviation; /** Deviation from linear EMF [%] */
} tc_seebeck_info_t;

/* =========================================================================
 * L3: Interpolation/Lookup Structures
 * ========================================================================= */

typedef struct {
    double   temperature;       /** Temperature [C] */
    double   emf;               /** Measured EMF [mV] */
    double   uncertainty;       /** Calibration uncertainty [C] */
} tc_cal_point_t;

typedef struct {
    tc_type_t       type;       /** Thermocouple type */
    size_t          n_points;   /** Number of calibration points */
    tc_cal_point_t *points;     /** Calibration points array */
    char            serial[32]; /** Sensor serial number */
    double          cal_date;   /** Calibration date */
    double          cal_expiry; /** Calibration expiry date */
} tc_cal_table_t;

typedef struct {
    double   x_low;      /** Lower bound of input domain */
    double   x_high;     /** Upper bound of input domain */
    double   slope;      /** Segment slope (dy/dx) */
    double   intercept;  /** Segment intercept (y at x=0) */
} tc_piecewise_segment_t;

typedef struct {
    size_t                  n_segments;    /** Number of segments */
    tc_piecewise_segment_t *segments;     /** Array of segments */
    double                  input_min;     /** Minimum input value */
    double                  input_max;     /** Maximum input value */
    double                  error_bound;   /** Max interpolation error [C] */
} tc_piecewise_model_t;

/* =========================================================================
 * L4: Fundamental Thermocouple Laws
 * ========================================================================= */

typedef struct {
    int      homogeneous_law_verified;       /** Law 1: homogeneous material */
    int      intermediate_metal_law_verified; /** Law 2: intermediate metal */
    int      successive_temp_law_verified;    /** Law 3: successive temperatures */
    double   homog_deviation;        /** Deviation from homogeneous law [uV] */
    double   inter_metal_deviation;  /** Deviation from intermediate metal law [uV] */
    double   success_temp_deviation; /** Deviation from successive temp law [uV] */
} tc_fundamental_laws_t;

/* =========================================================================
 * L5: Newton-Raphson Solver Configuration
 * ========================================================================= */

typedef struct {
    size_t   max_iterations;         /** Maximum Newton iterations */
    double   tolerance;              /** Convergence tolerance [C] */
    double   initial_guess;          /** Initial temperature guess [C] */
    double   damping;                /** Damping factor (0 < damp <= 1) */
    int      use_analytic_derivative; /** Use analytic derivative or finite diff */
} tc_newton_config_t;

typedef struct {
    double   solution;       /** Final solution temperature [C] */
    size_t   iterations;     /** Iterations performed */
    double   final_error;    /** |E(T_sol) - V_target| [mV] */
    int      converged;      /** 1 if converged, 0 otherwise */
    double   derivative_used; /** E'(T) at solution [mV/C] */
} tc_newton_result_t;

/* =========================================================================
 * L5: Self-Heating Model
 * ========================================================================= */

typedef struct {
    double   dissipation_constant;  /** Thermal resistance theta [K/W] */
    double   max_current;           /** Maximum allowed current [A] */
    double   max_power;             /** Maximum allowed power [W] */
    double   time_constant;         /** Thermal time constant [s] */
    int      medium;                /** 0=air, 1=oil, 2=water, 3=custom */
    double   medium_conductivity;   /** Medium thermal conductivity [W/(m*K)] */
} rtd_self_heating_t;

/* =========================================================================
 * L5: Noise Model
 * ========================================================================= */

typedef struct {
    double   johnson_noise_vrms;    /** Johnson noise [Vrms] */
    double   quant_noise_vrms;      /** Quantization noise [Vrms] */
    double   amp_noise_vrms;        /** Amplifier noise [Vrms] */
    double   emi_noise_vrms;        /** EMI pickup estimate [Vrms] */
    double   total_noise_vrms;      /** Total RMS noise [Vrms] */
    double   noise_equiv_temp;      /** Noise-equivalent temperature [C] */
    double   effective_bits;        /** Effective number of bits (ENOB) */
    double   snr_db;                /** Signal-to-noise ratio [dB] */
} tc_noise_model_t;

/* =========================================================================
 * Temperature Unit Conversion
 * ========================================================================= */

typedef enum {
    TC_UNIT_CELSIUS    = 0,
    TC_UNIT_KELVIN     = 1,
    TC_UNIT_FAHRENHEIT = 2,
    TC_UNIT_RANKINE    = 3
} tc_temperature_unit_t;

double tc_convert_temperature(double value, tc_temperature_unit_t from,
                               tc_temperature_unit_t to);
const char *tc_type_name(tc_type_t type);
tc_error_t tc_type_range(tc_type_t type, double *t_min, double *t_max);

/* =========================================================================
 * Core API: Thermocouple Conversion Functions
 * ========================================================================= */

tc_error_t tc_temp_to_emf(tc_type_t type, double temp, double *emf);
tc_error_t tc_emf_to_temp(tc_type_t type, double emf, double *temp);
tc_error_t tc_temp_to_emf_calibrated(const tc_cal_table_t *cal,
                                      double temp, double *emf);
tc_error_t tc_emf_to_temp_calibrated(const tc_cal_table_t *cal,
                                      double emf, double *temp);

/* =========================================================================
 * Core API: Cold-Junction Compensation
 * ========================================================================= */

tc_error_t tc_cjc_compensate_emf(tc_type_t type, double emf_raw,
                                  double t_cj, double *emf_corr);
tc_error_t tc_cjc_measure(tc_type_t type, double emf_raw, double t_cj,
                           double *temp);
tc_error_t tc_cjc_voltage(tc_type_t type, double t_cj, double *emf_cj);
tc_error_t tc_cjc_uncertainty(tc_type_t type, double t_cj_uncertainty,
                               double t_hot, double *tc_uncertainty);

/* =========================================================================
 * Core API: RTD Conversion Functions
 * ========================================================================= */

tc_error_t tc_rtd_temp_to_r(const rtd_cvd_coeffs_t *coeffs,
                             double temp, double *r);
tc_error_t tc_rtd_r_to_temp(const rtd_cvd_coeffs_t *coeffs,
                             double r, double *temp);
tc_error_t tc_rtd_get_coeffs(rtd_type_t rt_type,
                              rtd_alpha_standard_t alpha_std,
                              rtd_cvd_coeffs_t *coeffs);
tc_error_t tc_rtd_self_heating(double r, double i_excite,
                                const rtd_self_heating_t *sh,
                                double *delta_t);
tc_error_t tc_rtd_uncertainty(rtd_measurement_t *meas,
                               const rtd_cvd_coeffs_t *coeffs,
                               double r_unc);

/* =========================================================================
 * Core API: Combined Measurement System
 * ========================================================================= */

tc_error_t tc_measurement_init(tc_measurement_config_t *config,
                                tc_type_t tc_type, rtd_wiring_t wiring);
tc_error_t tc_measurement_process(const tc_measurement_config_t *config,
                                   uint32_t adc_code, tc_measurement_t *result);
tc_error_t tc_measurement_process_voltage(
    const tc_measurement_config_t *config,
    double voltage, double cj_temp,
    tc_measurement_t *result);
tc_error_t tc_adc_resolution_required(tc_type_t type, double temp_range,
                                       double target_res_c, size_t *bits_needed);

/* =========================================================================
 * L5: Linearization and Advanced Conversion Methods
 * ========================================================================= */

tc_error_t tc_newton_inverse(tc_type_t type, double v_target,
                              const tc_newton_config_t *config,
                              tc_newton_result_t *result);
double tc_horner_eval(const double *coeffs, size_t n_coeffs, double x);
double tc_horner_derivative(const double *coeffs, size_t n_coeffs, double x);
tc_error_t tc_piecewise_build(const double *x_data, const double *y_data,
                               size_t n_points, tc_piecewise_model_t *model);
tc_error_t tc_piecewise_eval(const tc_piecewise_model_t *model,
                              double x, double *y);
void tc_piecewise_free(tc_piecewise_model_t *model);

/* =========================================================================
 * L5: Noise Analysis and Filtering
 * ========================================================================= */

tc_error_t tc_johnson_noise(double resistance, double temp_k,
                             double bandwidth, double *noise_vrms);
tc_error_t tc_adc_quantization_noise(double vref, size_t bits,
                                      double *noise_vrms);
tc_error_t tc_noise_budget(const tc_measurement_config_t *config,
                            double source_resistance,
                            tc_noise_model_t *noise);
double tc_iir_filter(double alpha, double input, double *state);
double tc_moving_average(double *buffer, size_t window,
                          size_t *write_idx, size_t *count,
                          double sample);

/* =========================================================================
 * L6: Canonical Problem Solvers
 * ========================================================================= */

tc_error_t tc_measure_temperature(const tc_measurement_config_t *config,
                                   uint32_t adc_code,
                                   tc_measurement_t *result);
tc_error_t tc_rtd_4wire_measurement(double v_sense, double i_force,
                                     const rtd_cvd_coeffs_t *coeffs,
                                     rtd_measurement_t *result);
tc_error_t tc_rtd_3wire_measurement(double v_excite_pos, double v_sense,
                                     double v_excite_neg, double i_excite,
                                     double lead_r_match,
                                     const rtd_cvd_coeffs_t *coeffs,
                                     rtd_measurement_t *result);
tc_error_t tc_rtd_2wire_measurement(double v_measured, double i_excite,
                                     double r_lead_est,
                                     const rtd_cvd_coeffs_t *coeffs,
                                     rtd_measurement_t *result);

/* =========================================================================
 * L5: Calibration Table Management
 * ========================================================================= */

tc_error_t tc_cal_table_create(const tc_cal_point_t *points,
                                size_t n_points, tc_type_t type,
                                const char *serial, tc_cal_table_t *cal);
void tc_cal_table_free(tc_cal_table_t *cal);
tc_error_t tc_cal_spline_interpolate(const tc_cal_table_t *cal,
                                      double temp, double *emf);

/* =========================================================================
 * L5: Advanced RTD Functions
 * ========================================================================= */

tc_error_t tc_rtd_compute_alpha(double r0, double r100, double *alpha);
tc_error_t tc_rtd_ratiometric(double v_rtd, double v_ref, double r_ref,
                               const rtd_cvd_coeffs_t *coeffs,
                               rtd_measurement_t *result);

/* =========================================================================
 * L6: Error Propagation and Uncertainty Analysis
 * ========================================================================= */

tc_error_t tc_uncertainty_budget(tc_measurement_t *result,
                                  const tc_measurement_config_t *config);
tc_error_t tc_verify_intermediate_metal_law(tc_type_t type, double t_junc,
                                              tc_fundamental_laws_t *laws);

/* =========================================================================
 * L5: Seebeck Coefficient Calculation
 * ========================================================================= */

tc_error_t tc_seebeck_coefficient(tc_type_t type, double temp,
                                   double *seebeck);
tc_error_t tc_seebeck_info(tc_type_t type, double temp,
                            tc_seebeck_info_t *info);

/* =========================================================================
 * L6: Industrial Application Helpers
 * ========================================================================= */

tc_error_t tc_detect_open_circuit(double voltage,
                                   const tc_measurement_config_t *config,
                                   int *is_open);
tc_error_t tc_linearity_error(tc_type_t type, double t_min, double t_max,
                               double *max_error);

/* =========================================================================
 * L8: Advanced Topics - Kalman Tracking, Robust Regression
 * ========================================================================= */

double tc_kalman_track_temperature(double z, double dt, double Q, double R,
                                    double *T_est, double *P_est);
tc_error_t tc_robust_linear_fit(const double *x_data, const double *y_data,
                                 size_t n, double k, double *slope,
                                 double *intercept, double *mse);

/* =========================================================================
 * L7: Application - Process Control Integration
 * ========================================================================= */

double tc_pid_control(double setpoint, double measured, double dt,
                       double Kp, double Ki, double Kd,
                       double *integral, double *prev_error,
                       const double output_limits[2]);

#ifdef __cplusplus
}
#endif

#endif /* THERMOCOUPLE_CJC_RTD_H */