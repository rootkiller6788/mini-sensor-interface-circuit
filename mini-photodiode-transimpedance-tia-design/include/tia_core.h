/**
 * @file    tia_core.h
 * @brief   Core TIA Definitions ? L1 Definitions + L2 Core Concepts
 *
 * @details Defines the fundamental data structures for photodiode
 *          transimpedance amplifier design: photodiode models, TIA
 *          configurations, gain structures, frequency response, and
 *          signal chain parameters.
 *
 * Knowledge Mapping:
 *   L1 - Definitions:
 *     - Photodiode responsivity R (A/W) and quantum efficiency eta
 *     - Transimpedance gain Z_T = V_out / I_photo (Omega)
 *     - Dark current I_dark, shunt resistance R_sh
 *     - Junction capacitance C_j, terminal capacitance C_t
 *     - Noise Equivalent Power (NEP) in W/sqrt(Hz)
 *     - Specific Detectivity D* in cm*sqrt(Hz)/W (Jones)
 *     - 3-dB bandwidth f_3dB of TIA
 *     - Gain-Bandwidth Product (GBWP)
 *     - Input-referred noise current density (pA/sqrt(Hz))
 *     - Sensitivity in dBm for optical receivers
 *   L2 - Core Concepts:
 *     - Current-to-voltage conversion via feedback resistor R_f
 *     - Virtual ground at inverting input
 *     - Photovoltaic mode (zero bias) vs photoconductive mode (reverse bias)
 *     - Gain-bandwidth tradeoff in TIA design
 *     - Input capacitance and its effect on stability
 *     - Feedback pole-zero compensation
 *
 * References:
 *   - Graeme, "Photodiode Amplifiers" (1996)
 *   - Hobbs, "Building Electro-Optical Systems" (2nd ed, 2011)
 *   - Horowitz & Hill, "The Art of Electronics" (3rd ed, 2015), Ch.8
 *   - Sedra & Smith, "Microelectronic Circuits" (8th ed, 2020), Ch.10
 *   - Agrawal, "Fiber-Optic Communication Systems" (5th ed, 2021), Ch.4
 */

#ifndef TIA_CORE_H
#define TIA_CORE_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ??? Physical Constants ??????????????????????????????????????????????????? */

#define BOLTZMANN_CONSTANT   (1.380649e-23)
#define ELECTRON_CHARGE      (1.602176634e-19)
#define TEMPERATURE_STANDARD (298.15)

/* ??? L1: Photodiode Type Definitions ?????????????????????????????????????? */

/**
 * @brief Photodiode device types with distinct characteristics.
 *
 *        Si PIN:     lambda = 400-1100 nm, R ~ 0.5-0.6 A/W, low cost
 *        InGaAs PIN: lambda = 900-1700 nm, R ~ 0.9-1.0 A/W, telecom
 *        APD:        internal gain M = 10-100, excess noise factor F(M)
 *        SiPM:       Geiger-mode array, single-photon sensitive
 */
typedef enum {
    PHOTODIODE_SI_PIN      = 0,
    PHOTODIODE_INGAAS_PIN  = 1,
    PHOTODIODE_GE_PIN      = 2,
    PHOTODIODE_SI_APD      = 3,
    PHOTODIODE_INGAAS_APD  = 4,
    PHOTODIODE_SIPM        = 5,
    PHOTODIODE_QUAD        = 6,
    PHOTODIODE_CUSTOM      = 7
} photodiode_type_t;

/**
 * @brief Photodiode bias configuration.
 */
typedef enum {
    BIAS_PHOTOVOLTAIC     = 0,
    BIAS_PHOTOCONDUCTIVE  = 1,
    BIAS_BOOTSTRAPPED     = 2
} photodiode_bias_t;

/**
 * @brief Complete photodiode electrical model.
 *
 *        Equivalent circuit (small-signal):
 *          I_photo || C_j || R_sh  (all in parallel, to common)
 *        where I_photo = R(lambda) x P_opt is the signal current source.
 */
typedef struct {
    photodiode_type_t type;
    char   model_name[32];
    double active_area_mm2;
    double responsivity_a_per_w;
    double quantum_efficiency;
    double peak_wavelength_nm;
    double spectral_range_low_nm;
    double spectral_range_high_nm;
    double junction_capacitance_pf;
    double shunt_resistance_ohm;
    double dark_current_na;
    double dark_current_tempco;
    double reverse_bias_voltage;
    double breakdown_voltage;
    double noise_equivalent_power;
    double specific_detectivity;
    double rise_time_ns;
    double package_capacitance_pf;
    photodiode_bias_t bias_mode;
} photodiode_model_t;

/**
 * @brief TIA circuit topology variants.
 */
typedef enum {
    TIA_TOPOLOGY_BASIC       = 0,
    TIA_TOPOLOGY_CASCODE     = 1,
    TIA_TOPOLOGY_BOOTSTRAP   = 2,
    TIA_TOPOLOGY_DIFFERENTIAL = 3,
    TIA_TOPOLOGY_INTEGRATOR  = 4,
    TIA_TOPOLOGY_COMPOSITE   = 5
} tia_topology_t;

/**
 * @brief Operational amplifier parameters relevant to TIA design.
 */
typedef struct {
    char   part_number[32];
    double gain_bandwidth_mhz;
    double unity_gain_stable;
    double open_loop_gain_db;
    double input_voltage_noise_nv;
    double input_current_noise_fa;
    double input_capacitance_cm_pf;
    double input_capacitance_diff_pf;
    double input_resistance_ohm;
    double slew_rate_v_per_us;
    double output_voltage_swing_v;
    double supply_voltage_min;
    double supply_voltage_max;
    double supply_current_ma;
    double corner_freq_1f_hz;
    double output_impedance_ohm;
} opamp_params_t;

/**
 * @brief Complete TIA design parameters and performance metrics.
 */
typedef struct {
    photodiode_model_t photodiode;
    opamp_params_t      opamp;
    tia_topology_t     topology;
    photodiode_bias_t  bias;
    double rf_ohm;
    double cf_pf;
    double rf_parallel_resistance;
    double total_input_capacitance_pf;
    double input_resistance_ohm;
    double transimpedance_gain_ohm;
    double transimpedance_gain_db;
    double bandwidth_3db_hz;
    double bandwidth_3db_mhz;
    double gain_flatness_db;
    double peaking_db;
    double phase_margin_deg;
    double gain_margin_db;
    double signal_bandwidth_product;
    double total_input_noise_pa;
    double total_output_noise_mv;
    double input_noise_density_pa;
    double nepo_w_per_sqrt_hz;
    double sensitivity_dbm;
    double dc_output_offset_v;
    double dc_output_drift_uv_per_c;
    double max_photocurrent_ua;
    double saturation_power_dbm;
    double rise_time_ns;
    double settling_time_ns;
    double overshoot_percent;
    double group_delay_ns;
} tia_design_t;

/**
 * @brief TIA closed-loop frequency response data.
 */
typedef struct {
    size_t num_points;
    double *freq_hz;
    double *magnitude_db;
    double *phase_deg;
    double *group_delay_ns;
    double f_3db_hz;
    double f_peak_hz;
    double peaking_db;
    double phase_at_3db_deg;
} tia_freq_response_t;

/**
 * @brief Bode plot data for open-loop analysis.
 */
typedef struct {
    size_t num_points;
    double *freq_hz;
    double *aol_db;
    double *aol_phase_deg;
    double *beta_db;
    double *loop_gain_db;
    double *loop_phase_deg;
    double phase_margin_deg;
    double gain_margin_db;
    double crossover_freq_hz;
} tia_bode_data_t;

/**
 * @brief Step response data for time-domain TIA characterization.
 */
typedef struct {
    size_t num_points;
    double *time_ns;
    double *output_v;
    double final_value_v;
    double rise_time_10_90_ns;
    double fall_time_90_10_ns;
    double settling_time_1pct_ns;
    double overshoot_pct;
    double undershoot_pct;
    double propagation_delay_ns;
    double slew_rate_v_per_us;
} tia_step_response_t;

/**
 * @brief Optical link budget data structure.
 */
typedef struct {
    double optical_power_w;
    double optical_power_dbm;
    double photocurrent_ua;
    double output_voltage_v;
    double snr_db;
    double ber_estimate;
    double link_margin_db;
    double extinction_ratio_db;
    double responsivity_effective;
} optical_link_budget_t;

/**
 * @brief Complete TIA performance summary for comparison and selection.
 */
typedef struct {
    double transimpedance_dbohm;
    double bandwidth_mhz;
    double input_noise_density;
    double integrated_noise_na;
    double sensitivity_dbm;
    double dynamic_range_db;
    double power_consumption_mw;
    double figure_of_merit;
    double cost_estimate_usd;
} tia_performance_summary_t;

/* ??? Function Declarations ? Core TIA Operations ?????????????????????????? */

photodiode_model_t   photodiode_model_init(photodiode_type_t type,
                                            double area_mm2, double bias_v);

photodiode_model_t   photodiode_scale_area(const photodiode_model_t *pd,
                                            double area_mm2);

double               photodiode_cj_at_bias(const photodiode_model_t *pd, double vr);

opamp_params_t       opamp_params_init(const char *part_number);

tia_design_t         tia_design_basic(const photodiode_model_t *pd,
                                       const opamp_params_t *opa,
                                       double gain_target,
                                       double bw_target_hz);

double               tia_input_capacitance(const photodiode_model_t *pd,
                                            const opamp_params_t *opa,
                                            double stray);

double               tia_compensation_capacitance(const photodiode_model_t *pd,
                                                   const opamp_params_t *opa,
                                                   double rf,
                                                   double target_pm);

double               tia_bandwidth_3db(const tia_design_t *design);

double               tia_gain_at_frequency(const tia_design_t *design,
                                            double freq_hz);

tia_freq_response_t  tia_compute_frequency_response(const tia_design_t *design,
                                                     double freq_start,
                                                     double freq_stop,
                                                     size_t points);

tia_bode_data_t      tia_compute_bode(const tia_design_t *design,
                                       double freq_start, double freq_stop,
                                       size_t points);

tia_step_response_t  tia_compute_step_response(const tia_design_t *design,
                                                double i_step_ua,
                                                double duration_ns,
                                                size_t points);

optical_link_budget_t tia_link_budget(const tia_design_t *design,
                                       double optical_power,
                                       double wavelength_nm);

double               tia_sensitivity(const tia_design_t *design,
                                      double target_ber, double bitrate);

tia_performance_summary_t tia_performance_summary(const tia_design_t *design);

void  tia_freq_response_free(tia_freq_response_t *resp);
void  tia_bode_data_free(tia_bode_data_t *bode);
void  tia_step_response_free(tia_step_response_t *step);

double  tia_snr_compute(double i_photo_ua, double i_noise_pa, double bandwidth_hz);
double  tia_required_photocurrent(double target_snr_db, double i_noise_pa);
double  tia_optical_power_from_current(double i_photo_ua, double responsivity);

#endif /* TIA_CORE_H */
