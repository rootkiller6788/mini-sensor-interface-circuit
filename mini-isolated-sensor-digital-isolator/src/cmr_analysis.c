/**
 * @file cmr_analysis.c
 * @brief Common-Mode Rejection Analysis for Isolated Sensor Front-Ends
 *
 * Comprehensive CMRR analysis framework: impedance imbalance modeling,
 * frequency-dependent CMRR, budget analysis with worst-case stacking,
 * and automated optimization via impedance matching.
 *
 * Knowledge coverage: L1-L6
 *   L1: CMRR, CM gain, DM gain, CM-to-DM conversion
 *   L2: Impedance imbalance as primary CMRR degradation source
 *   L3: CMRR(f) transfer function with barrier parasitics
 *   L4: CMRR budget ！ worst-case vs RSS combination
 *   L5: Auto-calibration via impedance matching optimization
 *   L6: Bridge sensor and thermocouple applications
 *
 * References:
 *   - Pallas-Areny & Webster "Sensors and Signal Conditioning" (2001)
 *   - ADI AN-0992 "A Practical Review of Common Mode and IAs"
 *   - TI SLOA140 "Common Mode Rejection Ratio"
 */

#include "cmr_analysis.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* L1: cmr_analyzer_init */
int cmr_analyzer_init(cmr_analyzer_t *ca, isolation_amplifier_t *amp)
{
    if (!ca || !amp) return -1;
    memset(ca, 0, sizeof(*ca));
    ca->isoamp = amp;
    ca->overall.cmrr_db = amp->isolation.cmrr_at_dc_db;
    ca->overall.cm_input_range_v = 10.0;
    ca->overall.diff_input_range_v = 5.0;
    ca->overall.common_mode_gain = pow(10.0, -ca->overall.cmrr_db / 20.0);
    ca->overall.differential_mode_gain = amp->dc.nominal_gain_v_per_v;
    ca->overall.cm_to_diff_conversion_db = -ca->overall.cmrr_db;
    ca->input_imbalance.z_positive_ohm = 10000.0;
    ca->input_imbalance.z_negative_ohm = 10010.0;
    ca->input_imbalance.z_imbalance_pct = 0.1;
    ca->input_imbalance.z_imbalance_phase_deg = 0.0;
    ca->input_imbalance.resulting_cmrr_db = ca->overall.cmrr_db;
    ca->barrier_parasitics.r_series_ohm = 0.1;
    ca->barrier_parasitics.c_parallel_pf = 2.0;
    ca->barrier_parasitics.l_series_nh = 0.5;
    ca->barrier_parasitics.r_isolation_mohm = 1e6;
    ca->barrier_parasitics.c_coupling_pf = 1.5;
    return 0;
}

/* L2: cmrr_from_imbalance ！ CMRR limited by impedance mismatch.
 * For a differential amplifier with gain G_diff:
 * CMRR = 20*log10(G_diff * Z_nominal / (2 * delta_Z))
 * where delta_Z = |Z1 - Z2| */
double cmrr_from_imbalance(double z_nom_ohm, double imbalance_pct, double g_diff)
{
    if (z_nom_ohm <= 0.0 || imbalance_pct <= 0.0) return 200.0;
    double delta_z = z_nom_ohm * imbalance_pct / 100.0;
    double cmrr_lin = g_diff * z_nom_ohm / (2.0 * delta_z);
    return 20.0 * log10(cmrr_lin);
}

/* L3: cmrr_parallel_combination ！ When multiple CMRR-limiting factors exist.
 * Total CMRR = -20*log10(10^(-cmrr1/20) + 10^(-cmrr2/20))
 * (Parallel combination of error sources in linear domain) */
double cmrr_parallel_combination(double cmrr1_db, double cmrr2_db)
{
    double err1 = pow(10.0, -cmrr1_db / 20.0);
    double err2 = pow(10.0, -cmrr2_db / 20.0);
    double err_total = err1 + err2;
    if (err_total <= 0.0) return 200.0;
    return -20.0 * log10(err_total);
}

/* L3: cmrr_frequency_response ！ Single-pole roll-off model.
 * CMRR(f) = CMRR_DC / sqrt(1 + (f/f_pole)^2) in linear, then to dB */
double cmrr_frequency_response(double dc_cmrr_db, double pole_hz, double freq_hz)
{
    if (freq_hz <= 0.0) return dc_cmrr_db;
    double cmrr_lin = pow(10.0, dc_cmrr_db / 20.0);
    double degraded = cmrr_lin / sqrt(1.0 + (freq_hz / pole_hz) * (freq_hz / pole_hz));
    return 20.0 * log10(degraded);
}

/* L4: cmr_budget_add_contribution */
int cmr_budget_add_contribution(cmr_analyzer_t *ca, cmr_degradation_source_t src,
                                 double contrib_db, double corner_hz, const char *desc)
{
    if (!ca || ca->budget.num_contributions >= CMR_SOURCE_COUNT) return -1;
    size_t i = ca->budget.num_contributions++;
    ca->budget.contributions[i].source = src;
    ca->budget.contributions[i].contribution_db = contrib_db;
    ca->budget.contributions[i].corner_freq_hz = corner_hz;
    if (desc) snprintf(ca->budget.contributions[i].description,
                       sizeof(ca->budget.contributions[i].description), "%s", desc);
    return 0;
}

/* L4: cmr_budget_compute ！ Combine all contributions.
 * RSS (Root-Sum-Square): realistic for uncorrelated sources.
 * Worst-case: sum of magnitudes, for safety assessment. */
int cmr_budget_compute(cmr_analyzer_t *ca, double target_cmrr_db)
{
    if (!ca) return -1;
    ca->budget.target_cmrr_db = target_cmrr_db;
    double sum_sq = 0.0;
    double sum_lin = 0.0;
    for (size_t i = 0; i < ca->budget.num_contributions; i++) {
        double err = pow(10.0, -ca->budget.contributions[i].contribution_db / 20.0);
        sum_sq += err * err;
        sum_lin += err;
    }
    ca->budget.rss_combined_cmrr_db = (sum_sq > 0.0) ? -20.0 * log10(sqrt(sum_sq)) : 200.0;
    ca->budget.worst_case_cmrr_db = (sum_lin > 0.0) ? -20.0 * log10(sum_lin) : 200.0;
    ca->budget.margin_db = ca->budget.rss_combined_cmrr_db - target_cmrr_db;
    ca->budget.meets_target = ca->budget.margin_db >= 0.0;
    return 0;
}

/* L3: cmr_frequency_sweep ！ Generate CMRR vs frequency data */
int cmr_frequency_sweep(cmr_analyzer_t *ca, double f_start, double f_stop, size_t n)
{
    if (!ca || f_stop <= f_start || n < 2) return -1;
    free(ca->vs_freq.freq_hz);
    free(ca->vs_freq.cmrr_db);
    ca->vs_freq.freq_hz = (double *)malloc(n * sizeof(double));
    ca->vs_freq.cmrr_db = (double *)malloc(n * sizeof(double));
    if (!ca->vs_freq.freq_hz || !ca->vs_freq.cmrr_db) return -1;
    ca->vs_freq.num_points = n;
    double log_f0 = log10(f_start), log_f1 = log10(f_stop);
    double dlog = (log_f1 - log_f0) / (n - 1);
    double dc_cmrr = ca->isoamp->isolation.cmrr_at_dc_db;
    double pole = 1000.0;
    for (size_t i = 0; i < n; i++) {
        double f = pow(10.0, log_f0 + i * dlog);
        ca->vs_freq.freq_hz[i] = f;
        ca->vs_freq.cmrr_db[i] = cmrr_frequency_response(dc_cmrr, pole, f);
    }
    ca->vs_freq.cmrr_dc_db = dc_cmrr;
    ca->vs_freq.cmrr_3db_hz = pole;
    ca->vs_freq.cmrr_rolloff_db_per_decade = 20.0;
    return 0;
}

/* L5: cmr_effective_cmrr_at_freq */
double cmr_effective_cmrr_at_freq(const cmr_analyzer_t *ca, double freq_hz)
{
    if (!ca) return 0.0;
    return cmrr_frequency_response(ca->isoamp->isolation.cmrr_at_dc_db, 1000.0, freq_hz);
}

/* L5: cmr_max_common_mode_for_resolution
 * V_cm_max = V_diff_resolution * 10^(CMRR/20) */
double cmr_max_common_mode_for_resolution(const cmr_analyzer_t *ca, double res_v)
{
    if (!ca || res_v <= 0.0) return 0.0;
    double cmrr = ca->overall.cmrr_db;
    double cmrr_lin = pow(10.0, cmrr / 20.0);
    return res_v * cmrr_lin;
}

/* L6: cmr_optimize_impedance_matching ！ Improve CMRR by reducing imbalance */
int cmr_optimize_impedance_matching(cmr_analyzer_t *ca, double measured_imbalance_pct)
{
    if (!ca) return -1;
    ca->optimization.imbalance_initial_pct = measured_imbalance_pct;
    /* Optimization: reduce imbalance to ~1/10 of original */
    ca->optimization.imbalance_optimized_pct = measured_imbalance_pct * 0.1;
    double cmrr_before = cmrr_from_imbalance(10000.0, measured_imbalance_pct,
                                              ca->isoamp->dc.nominal_gain_v_per_v);
    double cmrr_after = cmrr_from_imbalance(10000.0, ca->optimization.imbalance_optimized_pct,
                                             ca->isoamp->dc.nominal_gain_v_per_v);
    ca->optimization.cmrr_improvement_db = cmrr_after - cmrr_before;
    ca->budget.rss_combined_cmrr_db += ca->optimization.cmrr_improvement_db;
    return 0;
}

/* L6: cmr_analyze_isolated_bridge ！ Bridge sensor CMR analysis.
 * Bridge sensors (strain gauge, pressure) have high CM voltage
 * (typically half the excitation) that must be rejected.
 * Rejected error = V_excitation * 10^(-CMRR/20) */
int cmr_analyze_isolated_bridge(cmr_analyzer_t *ca, double bridge_z_ohm,
                                 double v_exc, double *rejected_uv)
{
    if (!ca || !rejected_uv || bridge_z_ohm <= 0.0) return -1;
    double cm_v = v_exc / 2.0;
    double cmrr = ca->overall.cmrr_db;
    double cm_err = cm_v * pow(10.0, -cmrr / 20.0);
    *rejected_uv = cm_err * 1e6;
    return 0;
}

/* L6: cmr_analyze_isolated_thermocouple ！ TC has high CM noise.
 * Ground potential differences of several volts are common in
 * industrial environments. The iso-amp must reject this. */
int cmr_analyze_isolated_thermocouple(cmr_analyzer_t *ca, double tc_z_ohm,
                                       double gnd_diff_v, double *rejected_uv)
{
    if (!ca || !rejected_uv) return -1;
    (void)tc_z_ohm;
    double cmrr = ca->overall.cmrr_db;
    double cm_err = gnd_diff_v * pow(10.0, -cmrr / 20.0);
    *rejected_uv = cm_err * 1e6;
    return 0;
}

void cmr_analyzer_destroy(cmr_analyzer_t *ca)
{
    if (ca) {
        free(ca->vs_freq.freq_hz);
        free(ca->vs_freq.cmrr_db);
        memset(ca, 0, sizeof(*ca));
    }
}
