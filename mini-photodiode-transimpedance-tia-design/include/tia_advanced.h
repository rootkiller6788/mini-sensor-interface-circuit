/**
 * @file    tia_advanced.h
 * @brief   Advanced TIA Topics ? L7 Applications + L8 Advanced Topics
 *
 * @details Advanced transimpedance amplifier techniques including
 *          bootstrap input stages, differential TIAs, composite
 *          amplifiers, temperature compensation, and integrated
 *          CMOS TIA design considerations.
 *
 * Knowledge Mapping:
 *   L7 - Applications:
 *     - Fiber optic receiver (telecom, datacom)
 *     - LIDAR time-of-flight receiver
 *     - Spectrophotometer detector frontend
 *     - Pulse oximeter photoplethysmography
 *     - Industrial optical sensor (position, proximity)
 *   L8 - Advanced Topics:
 *     - Bootstrap input for C_j reduction
 *     - Differential TIA for common-mode rejection
 *     - Composite amplifier TIA for extended GBW
 *     - Temperature compensation techniques
 *     - T-network feedback for high gain with low R values
 *   L9 - Research Frontiers:
 *     - Integrated CMOS TIA design (28nm, 45nm nodes)
 *     - Single-photon avalanche diode (SPAD) quenching
 *     - Quantum-limited optical receivers
 *
 * References:
 *   - Razavi, "Design of Integrated Circuits for Optical Communications" (2nd ed, 2012)
 *   - Sackinger, "Analysis and Design of Transimpedance Amplifiers" (2017)
 *   - Hobbs (2011), Ch.18
 *   - Graeme (1996), Ch.7-8
 */

#ifndef TIA_ADVANCED_H
#define TIA_ADVANCED_H

#include "tia_core.h"
#include "tia_noise.h"

/* ??? L7: Fiber Optic Receiver ????????????????????????????????????????????? */

/**
 * @brief Fiber optic receiver configuration for telecom/datacom.
 */
typedef struct {
    double data_rate_gbps;
    double wavelength_nm;
    double fiber_length_km;
    double fiber_attenuation_db_per_km;
    double connector_loss_db;
    double transmitter_power_dbm;
    double required_ber;
    char   modulation_format[16];
    double extinction_ratio_db;
    double dispersion_penalty_db;
} fiber_receiver_spec_t;

/**
 * @brief Fiber receiver performance metrics.
 */
typedef struct {
    double received_power_dbm;
    double sensitivity_dbm;
    double link_margin_db;
    double estimated_ber;
    double eye_opening_percent;
    double jitter_ps;
    double power_penalty_db;
    double maximum_reach_km;
} fiber_receiver_perf_t;

/* ??? L7: LIDAR Receiver ??????????????????????????????????????????????????? */

/**
 * @brief LIDAR receiver configuration for time-of-flight measurement.
 */
typedef struct {
    double laser_wavelength_nm;
    double pulse_energy_nj;
    double pulse_width_ns;
    double pulse_repetition_khz;
    double target_range_m;
    double target_reflectivity;
    double aperture_diameter_mm;
    double atmospheric_attenuation_db_per_km;
} lidar_receiver_spec_t;

/**
 * @brief LIDAR receiver performance.
 */
typedef struct {
    double received_pulse_energy_fj;
    double peak_photocurrent_ua;
    double snr_db;
    double range_resolution_mm;
    double max_detectable_range_m;
    double false_alarm_rate;
    double detection_probability;
} lidar_receiver_perf_t;

/* ??? L7: Spectrophotometer Frontend ??????????????????????????????????????? */

/**
 * @brief Spectrophotometer detector configuration.
 */
typedef struct {
    double wavelength_start_nm;
    double wavelength_stop_nm;
    double wavelength_step_nm;
    double optical_power_nw;
    double integration_time_ms;
    double dark_subtraction_enabled;
} spectrometer_spec_t;

/**
 * @brief Spectrophotometer measurement result.
 */
typedef struct {
    size_t num_points;
    double *wavelength_nm;
    double *absorbance;
    double *transmittance;
    double *photocurrent_ua;
    double snr_min_db;
    double snr_max_db;
    double stray_light_percent;
} spectrometer_measurement_t;

/* ??? L8: Bootstrap TIA ???????????????????????????????????????????????????? */

/**
 * @brief Bootstrap input stage for C_j reduction.
 *
 *        Bootstrap??: ?????????????????,
 *        ??C_j???????,?????????????
 *
 *        Effective C_j_reduced = C_j * (1 - A_boot)
 *        where A_boot ? 1 is the bootstrap amplifier gain.
 */
typedef struct {
    double bootstrap_gain;
    double bootstrap_bandwidth_hz;
    double effective_cj_reduction_ratio;
    double bootstrapped_cj_pf;
    double added_noise_pa_per_sqrt_hz;
    double power_overhead_mw;
} tia_bootstrap_config_t;

/* ??? L8: Differential TIA ????????????????????????????????????????????????? */

/**
 * @brief Differential TIA configuration.
 *
 *        Uses two matched photodiodes (signal + reference/dummy)
 *        for common-mode rejection of ambient light and supply noise.
 */
typedef struct {
    double common_mode_rejection_db;
    double photodiode_matching_percent;
    double differential_gain_ohm;
    double common_mode_gain_ohm;
    double cmrr_db;
    double offset_voltage_uv;
    double offset_drift_nv_per_c;
} tia_differential_config_t;

/* ??? L8: Composite Amplifier TIA ?????????????????????????????????????????? */

/**
 * @brief Composite amplifier TIA configuration.
 *
 *        Cascades two op-amps to achieve higher effective GBWP.
 *        Stage 1: high-speed, low-gain for wide bandwidth
 *        Stage 2: high-gain for overall transimpedance
 *
 *        Effective GBWP ? GBWP1 * A_v2 (if properly compensated)
 */
typedef struct {
    opamp_params_t stage1_opamp;
    opamp_params_t stage2_opamp;
    double stage1_gain;
    double stage2_gain;
    double effective_gbwp_mhz;
    double interstage_compensation_pf;
    double total_power_mw;
} tia_composite_config_t;

/* ??? L8: T-Network Feedback TIA ??????????????????????????????????????????? */

/**
 * @brief T-network feedback for high gain without large R_f.
 *
 *        Z_T_effective = -R_f * (1 + R2/R1)
 *        where R_f, R1, R2 form the T-network.
 *
 *        Benefit: avoids parasitic capacitance of large R_f
 *        Cost:    multiplies input offset and noise
 */
typedef struct {
    double rf_ohm;
    double r1_ohm;
    double r2_ohm;
    double effective_gain_ohm;
    double gain_enhancement_factor;
    double offset_multiplication;
    double noise_multiplication;
} tia_tnetwork_config_t;

/* ??? L8: Temperature Compensation ????????????????????????????????????????? */

/**
 * @brief Temperature compensation configuration.
 *
 *        Dark current I_dark doubles every ~10?C for Si photodiodes.
 *        Compensation techniques:
 *        - Matched dummy photodiode for dark current subtraction
 *        - Thermistor-based gain adjustment
 *        - Digital calibration with temperature sensor
 */
typedef struct {
    double temp_coefficient_gain_ppm;
    double temp_coefficient_offset_uv;
    double compensation_method;
    double residual_drift_ppm;
    double temperature_range_min_c;
    double temperature_range_max_c;
} tia_temp_compensation_t;

/* ??? L8: CMOS Integrated TIA ?????????????????????????????????????????????? */

/**
 * @brief CMOS integrated TIA design considerations.
 *
 *        Technology node impacts: bandwidth, noise, power.
 *        Typical CMOS TIA architectures:
 *        - Common-gate (CG) input stage
 *        - Regulated cascode (RGC)
 *        - Shunt-shunt feedback
 *        - Inductive peaking for bandwidth extension
 */
typedef struct {
    double technology_node_nm;
    double supply_voltage_v;
    double transistor_ft_ghz;
    double flicker_noise_coefficient;
    double input_referred_noise_pa;
    double bandwidth_ghz;
    double power_consumption_mw;
    double die_area_mm2;
    double transimpedance_gain_dbohm;
} cmos_tia_params_t;

/* ??? Function Declarations ? Advanced TIA ????????????????????????????????? */

/* Fiber optic receiver */
fiber_receiver_perf_t  tia_fiber_receiver_analyze(const tia_design_t *design,
                                                    const fiber_receiver_spec_t *spec);

/* LIDAR receiver */
lidar_receiver_perf_t  tia_lidar_receiver_analyze(const tia_design_t *design,
                                                    const lidar_receiver_spec_t *spec);

/* Spectrophotometer */
spectrometer_measurement_t  tia_spectrometer_measure(
                                 const tia_design_t *design,
                                 const spectrometer_spec_t *spec);

/* Bootstrap TIA */
tia_bootstrap_config_t  tia_bootstrap_design(const photodiode_model_t *pd,
                                               const opamp_params_t *opa,
                                               double bootstrap_gain);

/* Differential TIA */
tia_differential_config_t  tia_differential_design(
                                 const photodiode_model_t *pd_signal,
                                 const photodiode_model_t *pd_ref,
                                 const opamp_params_t *opa);

/* Composite amplifier TIA */
tia_composite_config_t  tia_composite_design(const photodiode_model_t *pd,
                                               const opamp_params_t *stage1,
                                               const opamp_params_t *stage2,
                                               double target_gain, double target_bw);

/* T-network feedback */
tia_tnetwork_config_t  tia_tnetwork_design(double target_gain_ohm,
                                              double max_rf_ohm,
                                              double noise_budget_pa);

/* Temperature compensation */
double  tia_dark_current_at_temperature(const photodiode_model_t *pd,
                                         double temperature_c);

tia_temp_compensation_t  tia_temp_compensation_design(
                              const photodiode_model_t *pd,
                              double temp_min_c, double temp_max_c);

/* CMOS TIA estimation */
cmos_tia_params_t  tia_cmos_estimate(double technology_node_nm,
                                       double target_gain_dbohm,
                                       double target_bw_ghz);

/* LIDAR range equation */
double  tia_lidar_range_equation(double pulse_energy_j,
                                  double aperture_area_m2,
                                  double target_reflectivity,
                                  double atmospheric_transmission,
                                  double min_detectable_power_w);

/* Pulse detection threshold */
double  tia_pulse_detection_threshold(const tia_noise_model_t *noise,
                                       double false_alarm_probability);

/* APD optimal gain */
double  tia_apd_optimal_gain(double k_factor, double thermal_noise,
                               double dark_current, double signal_current);

/* Free spectrometer data */
void  spectrometer_measurement_free(spectrometer_measurement_t *m);

#endif /* TIA_ADVANCED_H */
