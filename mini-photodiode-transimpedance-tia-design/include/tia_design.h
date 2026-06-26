/**
 * @file    tia_design.h
 * @brief   TIA Design Methodology ? L5 Algorithms + L6 Canonical Problems
 *
 * @details Design procedures and optimization algorithms for
 *          transimpedance amplifiers. Covers the complete design
 *          flow from specification to component selection.
 *
 * Knowledge Mapping:
 *   L5 - Algorithms/Methods:
 *     - TIA systematic design procedure (Graeme method)
 *     - Gain-bandwidth optimization algorithm
 *     - Noise-gain peaking avoidance
 *     - Photodiode-OpAmp matching algorithm
 *     - Supply voltage and headroom analysis
 *   L6 - Canonical Problems:
 *     - High-speed TIA (maximize BW for given PD)
 *     - Low-noise TIA (minimize input-referred noise)
 *     - Wide-dynamic-range TIA (handling large I_photo range)
 *     - Low-power TIA (battery-operated sensors)
 *     - Precision TIA (minimize DC errors, drift)
 *   L7 - Applications:
 *     - Fiber optic receiver at 1-10 Gbps
 *     - LIDAR pulse receiver
 *     - Spectrophotometer frontend
 *     - Pulse oximeter photodetector
 *     - Barcode scanner receiver
 *
 * References:
 *   - Graeme, "Photodiode Amplifiers" (1996) - the canonical TIA design text
 *   - Hobbs (2011), Ch.18
 *   - Horowitz & Hill (2015), Ch.8
 */

#ifndef TIA_DESIGN_H
#define TIA_DESIGN_H

#include "tia_core.h"
#include "tia_noise.h"
#include "tia_stability.h"

/* ??? L5: Design Specification ????????????????????????????????????????????? */

/**
 * @brief Complete TIA design specification (input requirements).
 */
typedef struct {
    /* ?? Optical input ?? */
    double wavelength_nm;
    double min_optical_power_nw;
    double max_optical_power_uw;
    double optical_power_dynamic_range_db;

    /* ?? Electrical performance targets ?? */
    double target_gain_v_per_a;
    double target_bandwidth_mhz;
    double target_snr_db;
    double target_sensitivity_dbm;
    double target_phase_margin_deg;

    /* ?? Output requirements ?? */
    double output_voltage_swing_v;
    double output_impedance_ohm;
    double output_dc_offset_max_mv;

    /* ?? Environmental ?? */
    double temperature_min_c;
    double temperature_max_c;
    double supply_voltage_v;

    /* ?? Constraints ?? */
    double max_power_mw;
    double max_cost_usd;
    double max_board_area_mm2;
} tia_specification_t;

/* ??? L5: Design Candidate ????????????????????????????????????????????????? */

/**
 * @brief A single TIA design candidate for comparison/optimization.
 */
typedef struct {
    char            opamp_part[32];
    photodiode_type_t pd_type;
    tia_topology_t    topology;
    double rf_ohm;
    double cf_pf;
    double gain_ohm;
    double bandwidth_mhz;
    double noise_pa_per_sqrt_hz;
    double phase_margin_deg;
    double power_mw;
    double cost_usd;
    double figure_of_merit;
} tia_design_candidate_t;

/* ??? L5: Design Result ???????????????????????????????????????????????????? */

/**
 * @brief Complete design result with all analysis outputs.
 */
typedef struct {
    tia_design_t design;
    tia_noise_model_t noise;
    loop_gain_analysis_t stability;
    tia_freq_response_t freq_resp;
    tia_step_response_t step_resp;
    tia_performance_summary_t summary;
    int meets_specification;
    char failure_reason[128];
} tia_design_result_t;

/* ??? L5: Design Optimization ?????????????????????????????????????????????? */

/**
 * @brief Multi-criteria optimization parameters.
 */
typedef struct {
    double weight_gain;
    double weight_bandwidth;
    double weight_noise;
    double weight_power;
    double weight_cost;
} tia_optimization_weights_t;

/**
 * @brief Pareto-optimal design set.
 */
typedef struct {
    size_t num_designs;
    tia_design_result_t *designs;
    size_t *pareto_front_indices;
    size_t num_pareto;
} tia_pareto_set_t;

/* ??? Function Declarations ? Design Methodology ??????????????????????????? */

/**
 * @brief  Full TIA design flow from specification to complete result.
 * @param  spec    Design specification
 * @param  pd_type Preferred photodiode type (CUSTOM to auto-select)
 * @return         Complete design result with all analyses
 *
 * @note   This is the top-level design function. It:
 *         1. Selects/confirms photodiode based on wavelength
 *         2. Selects optimal op-amp from built-in database
 *         3. Computes R_f for target gain
 *         4. Computes C_f for target phase margin
 *         5. Verifies bandwidth meets target
 *         6. Computes noise performance
 *         7. Verifies stability
 *         8. Computes frequency and step responses
 *
 * Complexity: O(1) - uses analytical closed-form equations
 */
tia_design_result_t  tia_design_from_spec(const tia_specification_t *spec,
                                            photodiode_type_t pd_type);

/**
 * @brief  Design a high-speed TIA (bandwidth-optimized).
 * @param  pd             Photodiode model
 * @param  opa            Op-amp parameters
 * @param  target_bw_mhz  Target -3dB bandwidth (MHz)
 * @return                Optimized TIA design
 *
 * @note   High-speed strategy:
 *         - Use minimum R_f feasible for target BW
 *         - Use decompensated op-amp if beneficial
 *         - Minimize C_in (small area PD, low-C op-amp, short traces)
 *         - Accept higher noise as tradeoff
 *
 *         R_f_max = 1 / (2*pi * C_in * f_3dB * sqrt(GBWP/f_3dB))
 */
tia_design_t  tia_design_high_speed(const photodiode_model_t *pd,
                                     const opamp_params_t *opa,
                                     double target_bw_mhz);

/**
 * @brief  Design a low-noise TIA (noise-optimized).
 * @param  pd            Photodiode model
 * @param  opa           Op-amp parameters
 * @param  target_gain   Target transimpedance gain (ohm)
 * @return               Optimized TIA design
 *
 * @note   Low-noise strategy:
 *         - Use large R_f (reduces Johnson current noise)
 *         - Use photovoltaic mode (eliminates dark current shot noise)
 *         - Select op-amp with minimal e_n * C_in product
 *         - Use JFET/CMOS input for minimal i_n
 *         - Minimize bandwidth to reduce integrated noise
 */
tia_design_t  tia_design_low_noise(const photodiode_model_t *pd,
                                    const opamp_params_t *opa,
                                    double target_gain);

/**
 * @brief  Design a wide-dynamic-range TIA.
 * @param  pd                Photodiode model
 * @param  opa               Op-amp parameters
 * @param  min_current_na    Minimum detectable photocurrent (nA)
 * @param  max_current_ua    Maximum linear photocurrent (uA)
 * @return                   Optimized TIA design
 *
 * @note   Wide dynamic range strategy:
 *         - Adaptive gain switching (multiple R_f via analog switch)
 *         - Logarithmic compression for extreme range
 *         - DC servo loop to cancel ambient light
 *         - Dual-gain path with auto-ranging
 */
tia_design_t  tia_design_wide_dynamic(const photodiode_model_t *pd,
                                       const opamp_params_t *opa,
                                       double min_current_na,
                                       double max_current_ua);

/**
 * @brief  Design a low-power TIA for battery-operated sensors.
 * @param  pd          Photodiode model
 * @param  target_gain Target gain (ohm)
 * @param  max_power   Maximum power budget (mW)
 * @return             Optimized TIA design
 */
tia_design_t  tia_design_low_power(const photodiode_model_t *pd,
                                    const opamp_params_t *opa,
                                    double target_gain,
                                    double max_power);

/**
 * @brief  Evaluate multiple design candidates and rank by figure of merit.
 * @param  candidates    Array of design candidates
 * @param  num_candidates Number of candidates
 * @param  weights        Optimization weights
 * @param  best_index     [out] Index of best candidate
 * @return                Best design result
 */
tia_design_result_t  tia_select_best_design(const tia_design_candidate_t *candidates,
                                              size_t num_candidates,
                                              const tia_optimization_weights_t *weights,
                                              size_t *best_index);

/**
 * @brief  Generate Pareto-optimal designs from a set of candidates.
 * @param  candidates      Array of design candidates
 * @param  num_candidates  Number of candidates
 * @return                 Pareto-optimal set
 */
tia_pareto_set_t  tia_pareto_optimize(const tia_design_candidate_t *candidates,
                                        size_t num_candidates);

/**
 * @brief  Verify that a TIA design meets its specification.
 * @param  design      TIA design
 * @param  spec        Specification to check against
 * @param  fail_reason [out] Description of first failure (if any), 128 chars
 * @return             1 if all specs met, 0 otherwise
 */
int  tia_verify_specification(const tia_design_t *design,
                               const tia_specification_t *spec,
                               char *fail_reason);

/**
 * @brief  Recommend op-amp for given photodiode and requirements.
 * @param  pd               Photodiode model
 * @param  target_gain      Target transimpedance (ohm)
 * @param  target_bw_hz     Target bandwidth (Hz)
 * @param  low_noise        Prioritize noise (1) or speed (0)
 * @return                  Recommended op-amp part number string
 *
 * @note   Built-in database includes: OPA657, OPA656, OPA847, LTC6268,
 *         AD8015, ADA4817, OPA380, LMP7721, OPA827, THS4631
 */
const char*  tia_recommend_opamp(const photodiode_model_t *pd,
                                  double target_gain, double target_bw_hz,
                                  int low_noise);

/**
 * @brief  Recommend photodiode for wavelength and bandwidth.
 * @param  wavelength_nm  Operating wavelength (nm)
 * @param  bw_mhz          Required bandwidth (MHz)
 * @param  low_noise       Prioritize noise (1) or speed (0)
 * @return                 Recommended photodiode type
 */
photodiode_type_t  tia_recommend_photodiode(double wavelength_nm,
                                              double bw_mhz, int low_noise);

/**
 * @brief  Compute TIA figure of merit for comparison.
 * @param  design  TIA design
 * @return         Figure of merit (larger is better)
 *
 * @note   FOM = Z_T * BW / (i_n * sqrt(P_diss))
 *         Balances gain, bandwidth, noise, and power.
 */
double  tia_figure_of_merit(const tia_design_t *design);

/**
 * @brief  Print a human-readable design report.
 * @param  result  Design result to report
 */
void  tia_design_report(const tia_design_result_t *result);

/* ??? Memory Management ???????????????????????????????????????????????????? */

void  tia_design_result_free(tia_design_result_t *result);
void  tia_pareto_set_free(tia_pareto_set_t *pset);

#endif /* TIA_DESIGN_H */
