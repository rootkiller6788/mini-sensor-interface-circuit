/**
 * @file current_loop.c
 * @brief Core 4-20mA current loop analysis — KVL solver, transfer function,
 *        state classification, diagnostics, noise analysis, digital filtering.
 * Knowledge: L1-L6 (definitions through canonical problems), L8 (HART basics).
 */
#include "current_loop.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- L4: KVL Loop Solver ---- */
void current_loop_kvl_solve(current_loop_t *loop)
{
    if (!loop) return;
    loop->total_resistance = loop->cable_resistance + loop->shunt_resistance
                           + loop->barrier_resistance;
    loop->compliance_voltage = loop->supply_voltage
                              - loop->transmitter_min_voltage;
    double i_max_a = CURRENT_LOOP_MAX_mA / 1000.0;
    loop->voltage_margin = loop->compliance_voltage
                          - i_max_a * loop->total_resistance;
    double i_loop_a = loop->loop_current_mA / 1000.0;
    loop->power_delivered_mW = i_loop_a * i_loop_a
                              * loop->shunt_resistance * 1000.0;
    double p_supply_mW = loop->supply_voltage * i_loop_a * 1000.0;
    loop->loop_efficiency_percent = (p_supply_mW > 0.0)
        ? (loop->power_delivered_mW / p_supply_mW) * 100.0 : 0.0;
    loop->state = current_loop_classify_state(loop->loop_current_mA);
}

/* ---- L4: Max Cable Length (KVL application) ---- */
double current_loop_max_cable_length(const current_loop_t *loop,
                                      double cable_r_per_meter)
{
    if (!loop || cable_r_per_meter <= 0.0) return 0.0;
    double i_max_a = CURRENT_LOOP_MAX_mA / 1000.0;
    double v_avail = loop->supply_voltage - loop->transmitter_min_voltage
        - i_max_a * (loop->shunt_resistance + loop->barrier_resistance);
    if (v_avail <= 0.0) return 0.0;
    return (v_avail / i_max_a) / (2.0 * cable_r_per_meter);
}

/* ---- L4: Minimum Supply Voltage ---- */
double current_loop_min_supply_voltage(const current_loop_t *loop)
{
    if (!loop) return 0.0;
    double i_max_a = CURRENT_LOOP_MAX_mA / 1000.0;
    return loop->transmitter_min_voltage + i_max_a
        * (loop->cable_resistance + loop->shunt_resistance
           + loop->barrier_resistance);
}

/* ---- L1: Ohm's Law — Shunt Voltage <-> Current ---- */
double current_loop_from_shunt_voltage(double v_shunt, double r_shunt)
{
    if (r_shunt <= 0.0) return 0.0;
    return (v_shunt / r_shunt) * 1000.0;
}

double current_loop_to_shunt_voltage(double current_mA, double r_shunt)
{
    return (current_mA / 1000.0) * r_shunt;
}

/* ---- L3: Transfer Function — Process Value <-> Current ---- */
double current_loop_process_to_current(const current_loop_transfer_t *tf,
                                        double process_value)
{
    if (!tf) return CURRENT_LOOP_NAMUR_LOW_mA;
    double pv_span = tf->process_max - tf->process_min;
    double i_span  = tf->current_max_mA - tf->current_min_mA;
    if (pv_span <= 0.0) return tf->current_min_mA;
    double pv = process_value;
    if (pv < tf->process_min) pv = tf->process_min;
    if (pv > tf->process_max) pv = tf->process_max;
    double frac = (pv - tf->process_min) / pv_span;
    double i_out;
    if (tf->is_inverted)
        i_out = tf->current_max_mA - frac * i_span;
    else
        i_out = tf->current_min_mA + frac * i_span;
    i_out = i_out * tf->gain + tf->offset_mA;
    if (i_out < CURRENT_LOOP_NAMUR_LOW_mA)  i_out = CURRENT_LOOP_NAMUR_LOW_mA;
    if (i_out > CURRENT_LOOP_NAMUR_HIGH_mA) i_out = CURRENT_LOOP_NAMUR_HIGH_mA;
    return i_out;
}

double current_loop_current_to_process(const current_loop_transfer_t *tf,
                                        double current_mA)
{
    if (!tf) return 0.0;
    double pv_span = tf->process_max - tf->process_min;
    double i_span  = tf->current_max_mA - tf->current_min_mA;
    if (i_span <= 0.0) return tf->process_min;
    double i_cal = (current_mA - tf->offset_mA) / tf->gain;
    if (i_cal < tf->current_min_mA) i_cal = tf->current_min_mA;
    if (i_cal > tf->current_max_mA) i_cal = tf->current_max_mA;
    double frac = (i_cal - tf->current_min_mA) / i_span;
    if (tf->is_inverted)
        return tf->process_max - frac * pv_span;
    else
        return tf->process_min + frac * pv_span;
}
/* ---- L3: RMS Noise ---- */
double current_loop_noise_rms(const double *samples, size_t n, double *mean_out)
{
    if (!samples || n == 0) { if (mean_out) *mean_out = 0.0; return 0.0; }
    double sum = 0.0;
    for (size_t i = 0; i < n; i++) sum += samples[i];
    double mean = sum / (double)n;
    if (mean_out) *mean_out = mean;
    double sum_sq = 0.0;
    for (size_t i = 0; i < n; i++) { double d = samples[i] - mean; sum_sq += d * d; }
    return sqrt(sum_sq / (double)n);
}

/* ---- L3/L4: ENOB (Effective Number of Bits) ---- */
double current_loop_enob(double noise_rms_mA)
{
    if (noise_rms_mA <= 0.0) return 24.0;
    double snr_db = 20.0 * log10(CURRENT_LOOP_SPAN_mA / noise_rms_mA);
    double enob = (snr_db - 1.76) / 6.02;
    return (enob < 0.0) ? 0.0 : enob;
}

double current_loop_snr_db(double noise_rms_mA)
{
    if (noise_rms_mA <= 0.0) return 120.0;
    return 20.0 * log10(CURRENT_LOOP_SPAN_mA / noise_rms_mA);
}

/* ---- L6: RC Filter Simulation ---- */
double current_loop_rc_step_response(double t_s, double i0, double i_f, double r, double c)
{
    if (t_s <= 0.0) return i0;
    double tau = r * c;
    if (tau <= 0.0) return i_f;
    return i0 + (i_f - i0) * (1.0 - exp(-t_s / tau));
}

double current_loop_rc_cutoff_frequency(double r, double c)
{
    if (r <= 0.0 || c <= 0.0) return 0.0;
    return 1.0 / (2.0 * M_PI * r * c);
}

double current_loop_rc_settling_time(double r, double c, double pct)
{
    if (r <= 0.0 || c <= 0.0 || pct <= 0.0) return 0.0;
    double tau = r * c;
    double p = pct / 100.0;
    if (p <= 0.0 || p >= 1.0) return 0.0;
    return -tau * log(p);
}

/* ---- L2/L7: Intrinsic Safety per IEC 60079-11 ---- */
double current_loop_isolation_voltage(double v_supply, double sf)
{ return v_supply * sf; }

bool current_loop_verify_intrinsic_safety(double voltage, double current_mA,
    double cap_nF, double ind_mH, char gas_group)
{
    double v_max, i_max, mie_mj;
    switch (gas_group) {
        case 'A': case 'a': v_max = 45.0; i_max = 100.0; mie_mj = 0.25; break;
        case 'B': case 'b': v_max = 30.0; i_max = 100.0; mie_mj = 0.07; break;
        case 'C': case 'c': v_max = 15.0; i_max = 25.0;  mie_mj = 0.017; break;
        default: return false;
    }
    if (voltage > v_max || current_mA > i_max) return false;
    double e_cap = 0.5 * cap_nF * 1e-9 * voltage * voltage * 1000.0;
    if (e_cap > mie_mj) return false;
    double i_a = current_mA / 1000.0;
    double e_ind = 0.5 * ind_mH * 1e-3 * i_a * i_a * 1000.0;
    if (e_ind > mie_mj) return false;
    return true;
}
/* ---- L5: Digital Filtering ---- */
double current_loop_iir_filter(double sample, double prev, double alpha)
{
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    return alpha * sample + (1.0 - alpha) * prev;
}

double current_loop_iir_alpha(double fc, double fs)
{
    if (fc <= 0.0 || fs <= 0.0) return 1.0;
    if (fc >= fs / 2.0) return 1.0;
    double a = 1.0 - exp(-2.0 * M_PI * fc / fs);
    if (a < 0.0) a = 0.0;
    if (a > 1.0) a = 1.0;
    return a;
}

double current_loop_moving_average(double sample, double *buf, size_t winsz, size_t *idx)
{
    if (!buf || !idx || winsz == 0) return sample;
    buf[*idx] = sample;
    *idx = (*idx + 1) % winsz;
    double sum = 0.0;
    for (size_t i = 0; i < winsz; i++) sum += buf[i];
    return sum / (double)winsz;
}

static int cmp_dbl(const void *a, const void *b) {
    double da = *(const double *)a, db = *(const double *)b;
    return (da < db) ? -1 : (da > db) ? 1 : 0;
}

void current_loop_median_filter(double *x, size_t n, size_t ws)
{
    if (!x || n < 3 || ws < 3 || ws % 2 == 0) return;
    double *w = (double*)malloc(ws * sizeof(double));
    double *o = (double*)malloc(n * sizeof(double));
    if (!w || !o) { free(w); free(o); return; }
    size_t h = ws / 2;
    for (size_t i = 0; i < n; i++) {
        size_t a = (i >= h) ? (i - h) : 0;
        size_t b = (i + h < n) ? (i + h) : (n - 1);
        size_t cnt = b - a + 1;
        for (size_t j = 0; j < cnt; j++) w[j] = x[a + j];
        qsort(w, cnt, sizeof(double), cmp_dbl);
        o[i] = w[cnt / 2];
    }
    for (size_t i = 0; i < n; i++) x[i] = o[i];
    free(w); free(o);
}

/* ---- L2: Standard Configurations ---- */
void current_loop_init_standard_24v(current_loop_t *loop)
{
    if (!loop) return;
    memset(loop, 0, sizeof(*loop));
    loop->supply_voltage = 24.0;
    loop->supply_tolerance_percent = 5.0;
    loop->shunt_resistance = 250.0;
    loop->cable_resistance = 10.0;
    loop->barrier_resistance = 0.0;
    loop->transmitter_min_voltage = 8.0;
    loop->transmitter_quiescent_mA = 3.5;
    loop->topology = CURRENT_LOOP_TOPOLOGY_TWO_WIRE;
    loop->loop_current_mA = 4.0;
    loop->loop_current_target_mA = 12.0;
    loop->state = LOOP_STATE_NORMAL;
}

bool current_loop_check_compliance(const current_loop_t *loop, double tgt_mA)
{
    if (!loop) return false;
    double v_req = (tgt_mA / 1000.0) * loop->total_resistance;
    double v_avail = loop->supply_voltage - loop->transmitter_min_voltage;
    return v_req <= v_avail;
}

double current_loop_compute_accuracy(const loop_error_budget_t *eb, bool rss)
{
    if (!eb) return 0.0;
    double e[8] = {eb->sensor_error_percent, eb->transmitter_error_percent,
                   eb->adc_quantization_error_percent, eb->dac_quantization_error_percent,
                   eb->reference_error_percent, eb->temperature_drift_percent,
                   eb->nonlinearity_error_percent, eb->cable_leakage_error_percent};
    if (rss) {
        double s = 0.0;
        for (int i = 0; i < 8; i++) s += e[i] * e[i];
        return sqrt(s);
    } else {
        double s = 0.0;
        for (int i = 0; i < 8; i++) s += fabs(e[i]);
        return s;
    }
}

double current_loop_shunt_power(double i_mA, double r) {
    double ia = i_mA / 1000.0;
    return ia * ia * r;
}

double current_loop_cable_voltage_drop(double i_mA, double r) {
    return (i_mA / 1000.0) * r;
}

void current_loop_power_budget_solve(loop_power_budget_t *b)
{
    if (!b) return;
    b->total_available_mW = (b->supply_voltage - 8.0) * b->loop_current_mA;
    b->total_consumed_mW = b->transmitter_consumed_mW + b->sensor_excitation_mW
        + b->adc_power_mW + b->mcu_power_mW + b->display_power_mW
        + b->hart_modem_power_mW;
    b->margin_mW = b->total_available_mW - b->total_consumed_mW;
    b->is_sustainable = (b->margin_mW >= 0.0);
}

void namur_ne43_init_default(namur_ne43_levels_t *lv)
{
    if (!lv) return;
    lv->low_alarm_mA = 3.6; lv->low_saturation_mA = 3.8;
    lv->high_saturation_mA = 20.5; lv->high_alarm_mA = 21.0;
    lv->short_circuit_mA = 22.0;
}/* ---- L5: ADC/DAC Conversion ---- */
double current_loop_adc_to_current(uint32_t code, uint8_t bits, double vref, double rshunt)
{
    if (bits == 0 || rshunt <= 0.0) return 0.0;
    uint32_t maxc = (1u << bits) - 1;
    if (code > maxc) code = maxc;
    double v_adc = ((double)code * vref) / (double)maxc;
    return (v_adc / rshunt) * 1000.0;
}

uint32_t current_loop_current_to_dac(double i_mA, uint8_t bits)
{
    if (bits == 0) return 0;
    if (i_mA < 4.0) i_mA = 4.0;
    if (i_mA > 20.0) i_mA = 20.0;
    uint32_t maxc = (1u << bits) - 1;
    double frac = (i_mA - 4.0) / 16.0;
    return (uint32_t)(frac * maxc + 0.5);
}/* ---- L8: HART Protocol - Checksum and Frame Utilities ---- */
uint8_t hart_compute_checksum(const hart_frame_t *frame)
{
    if (!frame) return 0;
    uint8_t cs = 0;
    for (uint8_t i = 0; i < frame->preamble_count && i < HART_PREAMBLE_MAX; i++)
        cs ^= 0xFF;
    cs ^= frame->delimiter;
    for (uint8_t i = 0; i < frame->address_length; i++)
        cs ^= frame->address[i];
    cs ^= (uint8_t)frame->command;
    cs ^= frame->byte_count;
    for (uint8_t i = 0; i < frame->data_length; i++)
        cs ^= frame->data[i];
    return cs;
}

bool hart_validate_frame(const hart_frame_t *frame)
{
    if (!frame) return false;
    return hart_compute_checksum(frame) == frame->checksum;
}

float hart_parse_float(const uint8_t bytes[4])
{
    if (!bytes) return 0.0f;
    uint32_t raw = ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16)
                 | ((uint32_t)bytes[2] << 8)  |  (uint32_t)bytes[3];
    float result;
    memcpy(&result, &raw, sizeof(float));
    return result;
}

void hart_encode_float(float value, uint8_t bytes[4])
{
    if (!bytes) return;
    uint32_t raw;
    memcpy(&raw, &value, sizeof(float));
    bytes[0] = (uint8_t)(raw >> 24);
    bytes[1] = (uint8_t)(raw >> 16);
    bytes[2] = (uint8_t)(raw >> 8);
    bytes[3] = (uint8_t)(raw);
}
/* ---- L6: Two-Point Calibration ---- */
void current_loop_two_point_calibration(double rlo, double rhi,
    double mlo, double mhi, double *off, double *gain)
{
    if (!off || !gain) return;
    double span_meas = mhi - mlo;
    if (span_meas <= 0.0) { *off = 0.0; *gain = 1.0; return; }
    *gain = (rhi - rlo) / span_meas;
    *off = rlo - (*gain) * mlo;
}

double current_loop_apply_calibration(double raw, double off, double gain)
{ return gain * raw + off; }


/* ---- L5: Polynomial Evaluation (Horner method, O(N) stable) ---- */
double current_loop_polynomial_eval(double x, const double *c, size_t deg)
{
    if (!c) return 0.0;
    double r = c[deg];
    for (size_t i = deg; i > 0; i--) r = r * x + c[i - 1];
    return r;
}

/* ---- L7: Temperature Compensation (1st-order TC model) ---- */
double current_loop_temp_compensation(double raw, double t, double tref, double alpha)
{ return raw * (1.0 + alpha * (t - tref)); }

/* ---- L7: Percent / EU Conversion ---- */
double current_loop_to_percent(double i_mA)
{
    double p = (i_mA - 4.0) / 16.0 * 100.0;
    if (p < 0.0) p = 0.0;
    if (p > 100.0) p = 100.0;
    return p;
}

double current_loop_from_percent(double pct)
{
    if (pct < 0.0) pct = 0.0;
    if (pct > 100.0) pct = 100.0;
    return 4.0 + (pct / 100.0) * 16.0;
}

int current_loop_infer_topology_from_startup(const double *samples, size_t n, double dt)
{
    if (!samples || n < 3 || dt <= 0.0) return 0;
    double steady = samples[n - 1];
    if (steady < 1.0) return 0;
    double t90 = 0.9 * steady;
    size_t ri = 0;
    for (size_t i = 0; i < n; i++)
        if (samples[i] >= t90) { ri = i; break; }
    double rt = ri * dt;
    if (rt < 0.1) return 2;
    else if (rt < 1.0) return 1;
    else return 3;
}


/* ---- L6: State Classification (NAMUR NE43) ---- */
current_loop_state_t current_loop_classify_state(double current_mA)
{
    if (current_mA <= 0.0)    return LOOP_STATE_OPEN;
    if (current_mA > 22.0)    return LOOP_STATE_SHORT;
    if (current_mA <= CURRENT_LOOP_NAMUR_LOW_mA)  return LOOP_STATE_NAMUR_FAIL;
    if (current_mA >= CURRENT_LOOP_NAMUR_HIGH_mA) return LOOP_STATE_NAMUR_FAIL;
    if (current_mA < CURRENT_LOOP_MIN_mA) return LOOP_STATE_UNDERRANGE;
    if (current_mA > CURRENT_LOOP_MAX_mA) return LOOP_STATE_OVERRANGE;
    return LOOP_STATE_NORMAL;
}

/* ---- L6: Multi-Indicator Loop Diagnostic ---- */
uint8_t current_loop_diagnose(const current_loop_t *loop, double measured_mA,
    double measured_v, double noise_stddev_mA)
{
    uint8_t flags = LOOP_DIAG_OK;
    if (!loop) return flags;
    if (measured_mA < 0.1)        flags |= LOOP_DIAG_OPEN_CIRCUIT;
    if (measured_mA > 22.0)       flags |= LOOP_DIAG_SHORT_CIRCUIT;
    if (measured_mA > 0.1 && measured_mA < 3.8) flags |= LOOP_DIAG_LOW_CURRENT;
    if (measured_mA > 20.5 && measured_mA <= 22.0) flags |= LOOP_DIAG_HIGH_CURRENT;
    if (noise_stddev_mA > 0.08)   flags |= LOOP_DIAG_NOISE_EXCESSIVE;
    if (loop->supply_voltage > 0.0 && measured_v < 0.9 * loop->supply_voltage)
        flags |= LOOP_DIAG_SUPPLY_LOW;
    double v_drop = (measured_mA / 1000.0) * loop->total_resistance;
    double v_expect = loop->transmitter_min_voltage + v_drop;
    if (measured_mA >= 4.0 && measured_mA <= 20.0 && fabs(measured_v - v_expect) > 3.0)
        flags |= LOOP_DIAG_CALIBRATION_DRIFT;
    if (v_drop > 0.1 && fabs((measured_v - loop->transmitter_min_voltage) - v_drop) > v_drop * 0.5)
        flags |= LOOP_DIAG_GROUND_FAULT;
    return flags;
}

/* ---- L5: Piecewise Linearization ---- */
double current_loop_piecewise_linearize(double x, const double *bx, const double *by, size_t n)
{
    if (!bx || !by || n < 2) return x;
    if (x <= bx[0]) { double s = (by[1]-by[0])/(bx[1]-bx[0]); return by[0]+s*(x-bx[0]); }
    if (x >= bx[n-1]) { double s = (by[n-1]-by[n-2])/(bx[n-1]-bx[n-2]); return by[n-1]+s*(x-bx[n-1]); }
    size_t lo=0, hi=n-1;
    while (hi-lo>1) { size_t m=lo+(hi-lo)/2; if(bx[m]<=x)lo=m; else hi=m; }
    double dx=bx[hi]-bx[lo]; if(dx<=0.0)return by[lo];
    double t=(x-bx[lo])/dx; return by[lo]+t*(by[hi]-by[lo]);
}
