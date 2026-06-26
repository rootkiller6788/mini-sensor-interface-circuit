/**
 * @file cmr_analysis.h
 * @brief Common Mode Rejection Analysis for Isolated Sensor Interfaces
 *
 * Comprehensive common-mode analysis framework for isolated sensor systems.
 * Covers CMRR degradation mechanisms, common-mode to differential conversion
 * across isolation barriers, and design techniques to maximize rejection.
 *
 * Knowledge Coverage:
 *   L1: CMRR definition, common-mode voltage range, differential-mode range
 *   L2: impedance imbalance as source of CMRR degradation
 *   L3: CMRR(f) transfer function, pole-zero contribution from parasitics
 *   L4: CMRR degradation budget analysis (worst-case stacking)
 *   L5: automated CMRR optimization via impedance matching
 *   L6: CMRR analysis for isolated bridge sensor, iso-thermocouple front-end
 *
 * References:
 *   - Pallás-Areny & Webster "Sensors and Signal Conditioning" (2001)
 *   - Analog Devices "A Practical Review of Common Mode and Instrumentation
 *     Amplifiers" (AN-0992)
 *   - TI "Common Mode Rejection Ratio" (SLOA140)
 */

#ifndef CMR_ANALYSIS_H
#define CMR_ANALYSIS_H

#include "digital_isolator.h"
#include "isolation_amplifier.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* L1: CMR metrics */
typedef struct {
    double cmrr_db;
    double cm_input_range_v;
    double diff_input_range_v;
    double common_mode_gain;
    double differential_mode_gain;
    double cm_to_diff_conversion_db;
} cmr_metrics_t;

typedef enum {
    CMR_SOURCE_INPUT_IMBALANCE,
    CMR_SOURCE_BARRIER_COUPLING,
    CMR_SOURCE_SUPPLY_REJECTION,
    CMR_SOURCE_PCB_PARASITICS,
    CMR_SOURCE_CABLE_MISMATCH,
    CMR_SOURCE_COUNT
} cmr_degradation_source_t;

typedef struct {
    cmr_degradation_source_t source;
    double contribution_db;
    double corner_freq_hz;
    char description[128];
} cmr_contribution_t;

/* L2: Impedance imbalance model */
typedef struct {
    double z_positive_ohm;
    double z_negative_ohm;
    double z_imbalance_pct;
    double z_imbalance_phase_deg;
    double resulting_cmrr_db;
} impedance_imbalance_t;

typedef struct {
    double r_series_ohm;
    double c_parallel_pf;
    double l_series_nh;
    double r_isolation_mohm;
    double c_coupling_pf;
} parasitic_network_t;

/* L3: Frequency-dependent CMRR */
typedef struct {
    size_t num_points;
    double *freq_hz;
    double *cmrr_db;
    double *cm_gain_db;
    double *diff_gain_db;
    double cmrr_dc_db;
    double cmrr_3db_hz;
    double cmrr_rolloff_db_per_decade;
} cmrr_vs_frequency_t;

/* L4: CMRR budget analysis */
typedef struct {
    cmr_contribution_t contributions[CMR_SOURCE_COUNT];
    size_t num_contributions;
    double worst_case_cmrr_db;
    double rss_combined_cmrr_db;
    double target_cmrr_db;
    double margin_db;
    bool meets_target;
} cmrr_budget_t;

/* L5: Optimization */
typedef struct {
    double imbalance_initial_pct;
    double imbalance_optimized_pct;
    double cmrr_improvement_db;
    double matching_resistor_ohm;
    double matching_capacitor_pf;
    bool auto_calibrated;
} cmrr_optimization_t;

/* L6: System-level CMR analysis */
typedef struct {
    cmr_metrics_t overall;
    cmrr_budget_t budget;
    cmrr_vs_frequency_t vs_freq;
    isolation_amplifier_t *isoamp;
    parasitic_network_t input_parasitics;
    parasitic_network_t barrier_parasitics;
    parasitic_network_t output_parasitics;
    impedance_imbalance_t input_imbalance;
    cmrr_optimization_t optimization;
} cmr_analyzer_t;

/* API */
int cmr_analyzer_init(cmr_analyzer_t *ca, isolation_amplifier_t *amp);

double cmrr_from_imbalance(double z_nominal_ohm, double imbalance_pct,
                            double differential_gain);

double cmrr_parallel_combination(double cmrr1_db, double cmrr2_db);

double cmrr_frequency_response(double dc_cmrr_db, double pole_hz,
                                double freq_hz);

int cmr_budget_add_contribution(cmr_analyzer_t *ca,
                                 cmr_degradation_source_t source,
                                 double contribution_db,
                                 double corner_hz,
                                 const char *desc);

int cmr_budget_compute(cmr_analyzer_t *ca, double target_cmrr_db);

int cmr_frequency_sweep(cmr_analyzer_t *ca,
                         double f_start_hz, double f_stop_hz,
                         size_t num_points);

double cmr_effective_cmrr_at_freq(const cmr_analyzer_t *ca, double freq_hz);

double cmr_max_common_mode_for_resolution(const cmr_analyzer_t *ca,
                                           double desired_resolution_v);

int cmr_optimize_impedance_matching(cmr_analyzer_t *ca,
                                     double measured_imbalance_pct);

int cmr_analyze_isolated_bridge(cmr_analyzer_t *ca,
                                 double bridge_impedance_ohm,
                                 double excitation_voltage_v,
                                 double *rejected_cm_uv);

int cmr_analyze_isolated_thermocouple(cmr_analyzer_t *ca,
                                       double tc_impedance_ohm,
                                       double ground_potential_diff_v,
                                       double *rejected_error_uv);

void cmr_analyzer_destroy(cmr_analyzer_t *ca);

#endif /* CMR_ANALYSIS_H */
