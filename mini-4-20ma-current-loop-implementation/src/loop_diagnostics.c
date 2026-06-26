/**
 * @file loop_diagnostics.c
 * @brief Loop diagnostics, fault detection, predictive maintenance.
 * Knowledge: L6 (fault detection), L7 (predictive maintenance), L8 (advanced).
 */
#include "current_loop.h"
#include "loop_diagnostics.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

void loop_diag_run_complete(const current_loop_t *loop,
    const double *samples, size_t n, loop_diagnostic_result_t *result)
{
    if (!loop || !result) return;
    memset(result, 0, sizeof(*result));
    double mean = 0.0;
    if (samples && n > 0) {
        for (size_t i = 0; i < n; i++) mean += samples[i];
        mean /= (double)n;
    }
    double stddev = 0.0;
    if (samples && n > 1) {
        for (size_t i = 0; i < n; i++) {
            double d = samples[i] - mean; stddev += d * d;
        }
        stddev = sqrt(stddev / (double)n);
    }
    result->diagnostic_flags = current_loop_diagnose(loop, mean, loop->supply_voltage, stddev);
    result->state = current_loop_classify_state(mean);
    result->is_healthy = (result->diagnostic_flags == LOOP_DIAG_OK);
    switch (result->state) {
        case LOOP_STATE_NORMAL:
            result->state_description = "Normal operation";
            result->max_severity = DIAG_SEVERITY_OK;
            result->recommended_action_delay_hours = 8760.0; break;
        case LOOP_STATE_OPEN:
            result->state_description = "Open circuit detected";
            result->max_severity = DIAG_SEVERITY_CRITICAL;
            result->recommended_action_delay_hours = 0.0; break;
        case LOOP_STATE_SHORT:
            result->state_description = "Short circuit detected";
            result->max_severity = DIAG_SEVERITY_CRITICAL;
            result->recommended_action_delay_hours = 0.0; break;
        case LOOP_STATE_UNDERRANGE:
            result->state_description = "Current below 4mA";
            result->max_severity = DIAG_SEVERITY_WARNING;
            result->recommended_action_delay_hours = 24.0; break;
        case LOOP_STATE_OVERRANGE:
            result->state_description = "Current above 20mA";
            result->max_severity = DIAG_SEVERITY_WARNING;
            result->recommended_action_delay_hours = 24.0; break;
        case LOOP_STATE_NAMUR_FAIL:
            result->state_description = "NAMUR NE43 failure";
            result->max_severity = DIAG_SEVERITY_ERROR;
            result->recommended_action_delay_hours = 1.0; break;
        default:
            result->state_description = "Unknown";
            result->max_severity = DIAG_SEVERITY_ERROR;
            result->recommended_action_delay_hours = 8.0; break;
    }
}


bool loop_diag_ground_fault(double i_source, double i_return, double threshold)
{ return fabs(i_source - i_return) > threshold; }

bool loop_diag_intermittent(const double *samples, size_t n,
    double drop_threshold_pct, size_t max_dips)
{
    if (!samples || n < 3) return false;
    double drop_thr = 16.0 * drop_threshold_pct / 100.0;
    size_t dips = 0;
    for (size_t i = 1; i < n; i++) {
        if ((samples[i-1] - samples[i]) > drop_thr) dips++;
        if (dips > max_dips) return true;
    }
    return false;
}

double loop_diag_insulation_resistance(double v_supply, double i_supply, double i_loop)
{
    double i_leak = (i_supply - i_loop) / 1000.0;
    if (i_leak <= 0.0) return 1e6;
    return (v_supply / i_leak) / 1e6;
}

void loop_diag_predictive_trend(const uint32_t *faults, size_t n_days,
    double alpha, double *predicted, double *days_to_critical)
{
    if (!faults || n_days < 2) {
        if (predicted) *predicted = 0.0;
        if (days_to_critical) *days_to_critical = 8760.0;
        return;
    }
    double ewma = (double)faults[0];
    for (size_t i = 1; i < n_days; i++)
        ewma = alpha * (double)faults[i] + (1.0 - alpha) * ewma;
    if (predicted) *predicted = ewma;
    double sx = 0.0, sy = 0.0, sxy = 0.0, sx2 = 0.0;
    for (size_t i = 0; i < n_days; i++) {
        double xi = (double)i, yi = (double)faults[i];
        sx += xi; sy += yi; sxy += xi * yi; sx2 += xi * xi;
    }
    double denom = n_days * sx2 - sx * sx;
    double slope = (fabs(denom) > 1e-12) ? (n_days * sxy - sx * sy) / denom : 0.0;
    if (days_to_critical)
        *days_to_critical = (slope > 0.001) ? fmax(0.0, (10.0 - ewma) / slope) : 8760.0;
}

uint8_t loop_diag_ne107_status(const current_loop_t *loop,
    const loop_error_budget_t *eb, double accuracy_limit_pct)
{
    if (!loop || !eb) return NE107_STATUS_FAILURE;
    if (loop->state == LOOP_STATE_OPEN || loop->state == LOOP_STATE_SHORT
        || loop->state == LOOP_STATE_NAMUR_FAIL)
        return NE107_STATUS_FAILURE;
    if (eb->rss_total_percent > accuracy_limit_pct)
        return NE107_STATUS_OUT_OF_SPEC;
    if (fabs(eb->temperature_drift_percent) > accuracy_limit_pct * 0.8)
        return NE107_STATUS_MAINTENANCE_REQUIRED;
    return NE107_STATUS_OK;
}

void loop_diag_log_event(diag_event_t *buf, size_t buf_sz, size_t *idx,
    const diag_event_t *event)
{
    if (!buf || !idx || !event || buf_sz == 0) return;
    buf[*idx] = *event;
    *idx = (*idx + 1) % buf_sz;
}

bool loop_diag_find_pattern(const diag_event_t *buf, size_t n,
    uint16_t pattern_code, double *period_hours_out)
{
    if (!buf || n < 3 || !period_hours_out) return false;
    uint64_t prev_ts = 0;
    size_t count = 0;
    double sum_period = 0.0;
    for (size_t i = 0; i < n; i++) {
        if (buf[i].event_code == pattern_code) {
            if (count > 0)
                sum_period += (double)(buf[i].timestamp_ms - prev_ts);
            prev_ts = buf[i].timestamp_ms;
            count++;
        }
    }
    if (count < 3) return false;
    *period_hours_out = (sum_period / (count - 1)) / 3600000.0;
    return true;
}

double loop_diag_health_score(const current_loop_t *loop, const loop_health_trend_t *trend)
{
    if (!loop || !trend) return 0.0;
    double score = 100.0;
    double i_mA = loop->loop_current_mA;
    if (i_mA < 4.0) score -= 40.0 * (4.0 - i_mA) / 4.0;
    else if (i_mA > 20.0) score -= 40.0 * (i_mA - 20.0) / 4.0;
    if (trend->stddev_current_mA > 0.08) score -= 20.0;
    if (trend->fault_count > 0) score -= 20.0 * fmin(1.0, (double)trend->fault_count / 10.0);
    if (trend->supply_decay_rate_mV_per_day > 10.0) score -= 10.0;
    if (trend->insulation_resistance_mohm < 10.0) score -= 10.0;
    if (score < 0.0) score = 0.0;
    return score;
}

bool loop_diag_water_ingress(const double *leakage, size_t n_days, double thr_uA_day)
{
    if (!leakage || n_days < 3) return false;
    double sx = 0.0, sy = 0.0, sxy = 0.0, sx2 = 0.0;
    for (size_t i = 0; i < n_days; i++) {
        double xi = (double)i, yi = leakage[i];
        sx += xi; sy += yi; sxy += xi * yi; sx2 += xi * xi;
    }
    double denom = n_days * sx2 - sx * sx;
    if (fabs(denom) < 1e-12) return false;
    double slope = (n_days * sxy - sx * sy) / denom;
    return slope > thr_uA_day;
}
