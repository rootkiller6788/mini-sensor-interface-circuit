/**
 * @file receiver.c
 * @brief 4-20mA receiver / PLC analog input implementation: shunt selection,
 *        ADC conversion, digital filtering, engineering units, burnout detection.
 * Knowledge: L1 (receiver), L2 (shunt design), L5 (ADC/filtering), L6 (burnout).
 */
#include "current_loop.h"
#include "receiver.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void receiver_init_standard(current_loop_receiver_t *rx)
{
    if (!rx) return;
    memset(rx, 0, sizeof(*rx));
    rx->shunt_resistance = 250.0;
    rx->shunt_power_rating_w = 0.25;
    rx->adc_reference_voltage = 5.0;
    rx->adc_bits = 12;
    rx->iir_alpha = 0.1;
    rx->moving_avg_window = 8;
    rx->engineering_min = 0.0;
    rx->engineering_max = 100.0;
    rx->filter_enabled = true;
    rx->burnout_detection = true;
    rx->burnout_threshold_mA = 3.6;
}

double receiver_optimal_shunt(double adc_v_max)
{
    if (adc_v_max <= 0.0) return 250.0;
    return adc_v_max / 0.020;
}

double receiver_shunt_power_at_max(double r_shunt)
{
    double i_max = 0.020;
    return i_max * i_max * r_shunt;
}

double receiver_adc_to_current(const current_loop_receiver_t *rx, uint32_t adc_code)
{
    if (!rx || rx->adc_bits == 0 || rx->shunt_resistance <= 0.0) return 0.0;
    uint32_t max_code = (1u << rx->adc_bits) - 1;
    if (adc_code > max_code) adc_code = max_code;
    double v_adc = ((double)adc_code * rx->adc_reference_voltage) / (double)max_code;
    return (v_adc / rx->shunt_resistance) * 1000.0;
}

double receiver_adc_resolution(const current_loop_receiver_t *rx)
{
    if (!rx || rx->adc_bits == 0 || rx->shunt_resistance <= 0.0) return 0.0;
    uint32_t max_code = (1u << rx->adc_bits) - 1;
    double v_lsb = rx->adc_reference_voltage / (double)max_code;
    return (v_lsb / rx->shunt_resistance) * 1000.0;
}

bool receiver_detect_burnout(const current_loop_receiver_t *rx,
    const double *readings, size_t n, size_t votes_needed)
{
    if (!rx || !readings || n == 0 || votes_needed == 0) return false;
    size_t consecutive = 0;
    for (size_t i = 0; i < n; i++) {
        if (readings[i] < rx->burnout_threshold_mA) {
            consecutive++;
            if (consecutive >= votes_needed) return true;
        } else {
            consecutive = 0;
        }
    }
    return false;
}

double receiver_oversampling_improvement(const current_loop_receiver_t *rx, size_t ratio)
{
    (void)rx;
    if (ratio < 2) return 0.0;
    return 0.5 * log2((double)ratio);
}

double receiver_current_to_engineering(const current_loop_receiver_t *rx, double current_mA)
{
    if (!rx) return 0.0;
    double eu_span = rx->engineering_max - rx->engineering_min;
    if (current_mA < 4.0) current_mA = 4.0;
    if (current_mA > 20.0) current_mA = 20.0;
    double frac = (current_mA - 4.0) / 16.0;
    return rx->engineering_min + frac * eu_span;
}

double receiver_butterworth_lp2(double x_n, double *x_n1, double *x_n2,
    double *y_n1, double *y_n2, double b0, double b1, double b2,
    double a1, double a2)
{
    double y_n = b0 * x_n + b1 * (*x_n1) + b2 * (*x_n2) - a1 * (*y_n1) - a2 * (*y_n2);
    *x_n2 = *x_n1;
    *x_n1 = x_n;
    *y_n2 = *y_n1;
    *y_n1 = y_n;
    return y_n;
}

void receiver_butterworth_design(double fc, double fs,
    double *b0, double *b1, double *b2, double *a1, double *a2)
{
    if (fc <= 0.0 || fs <= 0.0 || fc >= fs / 2.0) {
        *b0 = 1.0; *b1 = 0.0; *b2 = 0.0; *a1 = 0.0; *a2 = 0.0;
        return;
    }
    double omega = 2.0 * M_PI * fc / fs;
    double sn = sin(omega);
    double cs = cos(omega);
    double alpha = sn / sqrt(2.0);
    double a0 = 1.0 + alpha;
    *b0 = (1.0 - cs) / (2.0 * a0);
    *b1 = (1.0 - cs) / a0;
    *b2 = *b0;
    *a1 = -2.0 * cs / a0;
    *a2 = (1.0 - alpha) / a0;
}

double receiver_aperture_error(double signal_freq, double signal_amp, double t_aperture)
{
    if (signal_freq <= 0.0 || t_aperture <= 0.0) return 0.0;
    double max_slew_rate = 2.0 * M_PI * signal_freq * signal_amp;
    return max_slew_rate * t_aperture;
}
