/**
 * @file loop_diagnostics.h
 * @brief 4-20mA loop diagnostics, fault detection, and predictive maintenance.
 *
 * Industrial process loops require continuous health monitoring.
 * This module covers electrical fault detection (open/short/ground fault),
 * performance degradation analysis, and predictive maintenance indicators.
 *
 * Reference: NAMUR NE43, NE107 (Self-Monitoring), ISA-18.2 (Alarm Management)
 * Knowledge: L6 fault detection, L7 predictive maintenance, L3 statistical
 *            process control, L8 advanced diagnostics
 */

#ifndef LOOP_DIAGNOSTICS_H
#define LOOP_DIAGNOSTICS_H

#include "current_loop.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Diagnostic event severity levels (L6).
 */
typedef enum {
    DIAG_SEVERITY_OK       = 0,
    DIAG_SEVERITY_INFO     = 1,
    DIAG_SEVERITY_WARNING  = 2,
    DIAG_SEVERITY_ERROR    = 3,
    DIAG_SEVERITY_CRITICAL = 4
} diag_severity_t;

/**
 * @brief Diagnostic event record (L6/L7).
 */
typedef struct {
    uint64_t timestamp_ms;
    uint16_t event_code;
    diag_severity_t severity;
    double loop_current_mA;
    double supply_voltage;
    char description[64];
} diag_event_t;

/**
 * @brief Loop health trend data for predictive maintenance (L7/L8).
 */
typedef struct {
    double mean_current_mA;
    double stddev_current_mA;
    double drift_rate_mA_per_day;
    double noise_increase_rate_pct_per_day;
    double supply_decay_rate_mV_per_day;
    double insulation_resistance_mohm;
    double contact_resistance_ohm;
    uint32_t operating_hours;
    uint32_t fault_count;
    uint32_t namur_alarm_count;
    double health_score;
} loop_health_trend_t;

/**
 * @brief Result of a single loop diagnostic check (L6).
 */
typedef struct {
    uint8_t diagnostic_flags;
    current_loop_state_t state;
    const char *state_description;
    bool is_healthy;
    diag_severity_t max_severity;
    double recommended_action_delay_hours;
} loop_diagnostic_result_t;

/**
 * @brief Run complete loop diagnostic suite (L6).
 *
 * Performs all diagnostic checks: open circuit, short circuit,
 * ground fault, supply voltage, noise level, calibration drift,
 * and NAMUR alarm monitoring.
 *
 * @param loop            Loop configuration
 * @param current_samples Recent current measurements (mA)
 * @param n_samples       Number of samples
 * @param result          Output diagnostic result
 */
void loop_diag_run_complete(const current_loop_t *loop,
                             const double *current_samples,
                             size_t n_samples,
                             loop_diagnostic_result_t *result);

/**
 * @brief Detect ground fault in a current loop (L6).
 *
 * A ground fault occurs when the loop conductor contacts earth ground,
 * creating a parallel path. This is detected by:
 *   1. Measuring current at both ends of the loop
 *   2. If |I_supply - I_return| > threshold, ground fault exists
 *
 * @param current_at_source_mA Current measured at supply end
 * @param current_at_return_mA Current measured at return end
 * @param threshold_mA         Detection threshold (typically 0.1 mA)
 * @return                     true if ground fault detected
 */
bool loop_diag_ground_fault(double current_at_source_mA,
                              double current_at_return_mA,
                              double threshold_mA);

/**
 * @brief Detect intermittent connection faults (L6/L8).
 *
 * Intermittent faults appear as rapid, transient current dips.
 * Detection: count the number of times current drops > X% of span
 * within a time window. If count exceeds threshold, flag intermittent.
 *
 * @param current_samples     Recent current measurements (mA)
 * @param n_samples           Number of samples
 * @param drop_threshold_pct  Threshold for "dip" (% of span, e.g. 2%)
 * @param max_dips            Max allowed dips before alarm
 * @return                    true if intermittent fault detected
 */
bool loop_diag_intermittent(const double *current_samples, size_t n_samples,
                             double drop_threshold_pct, size_t max_dips);

/**
 * @brief Compute loop insulation resistance (L7).
 *
 * Insulation degradation is a leading cause of loop failures.
 * R_insulation = V_supply / I_leakage
 *
 * Where I_leakage = I_supply - I_loop (the difference between supplied
 * and returned current, excluding intentional loads).
 *
 * @param v_supply           Supply voltage (V)
 * @param i_supply_mA        Total supply current (mA)
 * @param i_loop_mA          Measured loop current (mA)
 * @return                   Insulation resistance (Mohm)
 */
double loop_diag_insulation_resistance(double v_supply,
                                        double i_supply_mA,
                                        double i_loop_mA);

/**
 * @brief Trend analysis for predictive maintenance (L7/L8).
 *
 * Analyzes historical fault data to predict remaining useful life.
 * Uses exponential weighted moving average (EWMA) for trend detection:
 *   trend[n] = alpha * value[n] + (1-alpha) * trend[n-1]
 *
 * @param historical_fault_counts Array of daily fault counts
 * @param n_days                  Number of days of history
 * @param ewma_alpha              Smoothing factor (0.05-0.3)
 * @param predicted_faults_tomorrow Output: predicted faults tomorrow
 * @param days_to_critical        Output: estimated days until critical
 */
void loop_diag_predictive_trend(const uint32_t *historical_fault_counts,
                                 size_t n_days, double ewma_alpha,
                                 double *predicted_faults_tomorrow,
                                 double *days_to_critical);

/**
 * @brief NAMUR NE107 self-monitoring status (L7).
 *
 * NE107 defines standardized device status signals:
 *   - Failure: device malfunction, output invalid
 *   - Function Check: maintenance/calibration in progress
 *   - Out of Specification: operating outside rated conditions
 *   - Maintenance Required: still functioning but degrading
 *
 * @param loop               Loop descriptor
 * @param error_budget       Current error budget
 * @param accuracy_limit_pct Maximum allowed error (%)
 * @return                   NE107 status code
 */
#define NE107_STATUS_OK                   0
#define NE107_STATUS_MAINTENANCE_REQUIRED 1
#define NE107_STATUS_OUT_OF_SPEC          2
#define NE107_STATUS_FUNCTION_CHECK       3
#define NE107_STATUS_FAILURE              4

uint8_t loop_diag_ne107_status(const current_loop_t *loop,
                                const loop_error_budget_t *error_budget,
                                double accuracy_limit_pct);

/**
 * @brief Log a diagnostic event to a circular buffer (L7).
 *
 * Maintains a history of recent diagnostic events for later analysis.
 *
 * @param event_buffer     Circular buffer
 * @param buffer_size      Buffer capacity
 * @param write_index      Current write position (updated in-place)
 * @param new_event        Event to log
 */
void loop_diag_log_event(diag_event_t *event_buffer, size_t buffer_size,
                          size_t *write_index, const diag_event_t *new_event);

/**
 * @brief Analyze event buffer for recurring fault patterns (L7/L8).
 *
 * Searches for periodic fault patterns that may indicate
 * systematic issues (e.g., daily thermal cycling, vibration-related).
 *
 * @param event_buffer    Circular buffer of events
 * @param n_events        Number of valid events
 * @param pattern_code    Event code to search for
 * @param period_hours_out Output: detected period in hours (0 if none)
 * @return                true if periodic pattern detected
 */
bool loop_diag_find_pattern(const diag_event_t *event_buffer,
                             size_t n_events, uint16_t pattern_code,
                             double *period_hours_out);

/**
 * @brief Compute loop health score (0-100) for dashboard display (L7).
 *
 * Health score = weighted combination of:
 *   - Current within range: 40 points
 *   - Noise within spec: 20 points
 *   - No recent alarms: 20 points
 *   - Supply voltage nominal: 10 points
 *   - Calibration current: 10 points
 *
 * @param loop             Loop configuration
 * @param trend            Health trend data
 * @return                 Health score (0-100, higher is better)
 */
double loop_diag_health_score(const current_loop_t *loop,
                               const loop_health_trend_t *trend);

/**
 * @brief Check for water ingress / corrosion via leakage current (L7/L8).
 *
 * Water ingress in junction boxes causes leakage current that
 * increases over time. Detection: monitor I_leakage trend.
 *
 * @param leakage_history  Daily leakage current measurements (uA)
 * @param n_days           Number of days
 * @param threshold_uA_per_day Alarm threshold (uA/day increase)
 * @return                 true if water ingress suspected
 */
bool loop_diag_water_ingress(const double *leakage_history,
                              size_t n_days,
                              double threshold_uA_per_day);

#ifdef __cplusplus
}
#endif

#endif /* LOOP_DIAGNOSTICS_H */