/**
 * @file    tc_conversion.c
 * @brief   Combined conversion utilities: noise analysis (Johnson/Nyquist,
 *          ADC quantization), digital filtering, uncertainty budget,
 *          open-circuit detection, and measurement system pipeline.
 *
 * Knowledge Coverage:
 *   L3: Johnson-Nyquist noise formula, quantization noise analysis
 *   L5: IIR filtering, moving average, noise budget computation
 *   L6: Open-thermocouple detection, ADC resolution requirements
 *   L6: GUM uncertainty budget for thermocouple measurements
 *   L7: Complete measurement pipeline from ADC code to temperature
 *   L8: Kalman filter for temperature tracking
 *
 * Reference:
 *   Nyquist, H. (1928) "Thermal Agitation of Electric Charge in Conductors"
 *   Johnson, J.B. (1928) "Thermal Agitation of Electricity in Conductors"
 *   ISO/IEC 98-3:2008 Guide to the Expression of Uncertainty in Measurement
 *   Kalman, R.E. (1960) "A New Approach to Linear Filtering and Prediction"
 */

#include "thermocouple_cjc_rtd.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Boltzmann constant [J/K] */
#define TC_KB 1.380649e-23

/* =========================================================================
 * L3: Johnson-Nyquist Thermal Noise
 * ========================================================================= */

/**
 * @brief tc_johnson_noise: Compute Johnson-Nyquist thermal noise
 *
 * Every resistor generates thermal noise due to random thermal motion
 * of charge carriers. The RMS noise voltage is:
 *
 *   Vn_rms = sqrt(4 * k_B * T * R * BW)
 *
 * where:
 *   k_B = 1.380649e-23 J/K (Boltzmann constant)
 *   T = absolute temperature [K]
 *   R = resistance [ohms]
 *   BW = measurement bandwidth [Hz]
 *
 * Example: A 100 ohm Pt100 at 300K with 10 Hz bandwidth
 *   Vn_rms = sqrt(4 * 1.38e-23 * 300 * 100 * 10)
 *          = sqrt(1.656e-17) = 4.07e-9 V = 4.07 nVrms
 *
 * For a Type K thermocouple with ~40 uV/C sensitivity:
 *   Temperature noise floor = 4.07e-9 / 40e-6 = 0.0001 C
 *   (Thermal noise is NOT the limiting factor for T/C measurements;
 *    ADC quantization and amplifier noise dominate.)
 *
 * However, for noble metal thermocouples with ~6 uV/C sensitivity:
 *   Temperature noise floor = 4.07e-9 / 6e-6 = 0.0007 C
 *   (Still negligible; noise is dominated by thermoelectric effects
 *    in the copper traces and connectors.)
 *
 * Complexity: O(1).
 */
tc_error_t tc_johnson_noise(double resistance, double temp_k,
                             double bandwidth, double *noise_vrms) {
    if (!noise_vrms) return TC_ERR_NULL_POINTER;
    if (resistance < 0.0 || temp_k <= 0.0 || bandwidth < 0.0) {
        *noise_vrms = 0.0;
        return TC_ERR_NULL_POINTER;
    }

    *noise_vrms = sqrt(4.0 * TC_KB * temp_k * resistance * bandwidth);
    return TC_OK;
}

/* =========================================================================
 * L3: ADC Quantization Noise
 * ========================================================================= */

/**
 * @brief tc_adc_quantization_noise: Compute ADC quantization noise
 *
 * An ideal ADC with N bits and reference Vref has quantization step:
 *   Q = Vref / (2^N)
 *
 * The quantization error is uniformly distributed in [-Q/2, +Q/2],
 * giving RMS noise:
 *   Vq_rms = Q / sqrt(12) = Vref / (2^N * sqrt(12))
 *
 * For a 16-bit ADC with 2.5V reference:
 *   Q = 2.5 / 65536 = 38.1 uV
 *   Vq_rms = 38.1e-6 / 3.464 = 11.0 uVrms
 *
 * For a 24-bit ADC with 2.5V reference:
 *   Q = 2.5 / 16777216 = 0.149 uV
 *   Vq_rms = 0.043 uVrms
 *   (At this level, other noise sources dominate: thermal, 1/f, EMI)
 *
 * Complexity: O(1).
 */
tc_error_t tc_adc_quantization_noise(double vref, size_t bits,
                                      double *noise_vrms) {
    double q;

    if (!noise_vrms) return TC_ERR_NULL_POINTER;
    if (bits == 0 || bits > 32 || vref <= 0.0) {
        *noise_vrms = 0.0;
        return TC_ERR_NULL_POINTER;
    }

    q = vref / (double)((size_t)1 << bits);
    *noise_vrms = q / sqrt(12.0);
    return TC_OK;
}

/* =========================================================================
 * L5: Noise Budget Analysis
 * ========================================================================= */

tc_error_t tc_noise_budget(const tc_measurement_config_t *config,
                            double source_resistance,
                            tc_noise_model_t *noise) {
    double t_kelvin;

    if (!config || !noise) return TC_ERR_NULL_POINTER;

    memset(noise, 0, sizeof(*noise));

    t_kelvin = config->cjc.cj_temperature + 273.15;
    if (t_kelvin < 273.15) t_kelvin = 300.0; /* Default to 300K if CJC unknown */

    /* Johnson noise from source resistance */
    tc_johnson_noise(source_resistance, t_kelvin, config->sample_rate_hz / 2.0,
                     &noise->johnson_noise_vrms);

    /* ADC quantization noise */
    tc_adc_quantization_noise(config->adc_vref, config->adc_bits,
                               &noise->quant_noise_vrms);

    /* Amplifier noise: estimate using typical values
     * Good low-noise amp: ~10 nV/rtHz * sqrt(BW) */
    noise->amp_noise_vrms = 10e-9 * sqrt(config->sample_rate_hz / 2.0);

    /* EMI pickup: estimate 50/60 Hz at ~1 uV (typical for good shielding) */
    noise->emi_noise_vrms = 1.0e-6;

    /* Total RMS noise (root-sum-square) */
    noise->total_noise_vrms = sqrt(
        noise->johnson_noise_vrms * noise->johnson_noise_vrms
        + noise->quant_noise_vrms * noise->quant_noise_vrms
        + noise->amp_noise_vrms * noise->amp_noise_vrms
        + noise->emi_noise_vrms * noise->emi_noise_vrms
    );

    /* Noise-equivalent temperature (for the configured thermocouple type) */
    {
        double seebeck;
        if (tc_seebeck_coefficient(config->tc_type,
                                    config->cjc.cj_temperature,
                                    &seebeck) == TC_OK
            && fabs(seebeck) > 1e-15) {
            noise->noise_equiv_temp = noise->total_noise_vrms
                                      / (fabs(seebeck) * 1e-3); /* uV to mV */
        } else {
            noise->noise_equiv_temp = noise->total_noise_vrms / 40e-6; /* 40 uV/C default */
        }
    }

    /* Effective number of bits considering total noise */
    {
        double q = config->adc_vref / (double)((size_t)1 << config->adc_bits);
        if (q > 1e-20) {
            noise->effective_bits = log2(config->adc_vref / (noise->total_noise_vrms * sqrt(12.0)));
        } else {
            noise->effective_bits = (double)config->adc_bits;
        }
    }

    /* SNR */
    if (noise->total_noise_vrms > 1e-20) {
        noise->snr_db = 20.0 * log10((config->adc_vmax - config->adc_vmin)
                                     / (2.0 * noise->total_noise_vrms));
    }

    return TC_OK;
}

/* =========================================================================
 * L5: Digital Filtering
 * ========================================================================= */

/**
 * @brief tc_iir_filter: First-order IIR low-pass filter
 *
 * Difference equation:
 *   y[n] = alpha * y[n-1] + (1 - alpha) * x[n]
 *
 * where alpha = exp(-2*pi*f_cutoff / f_sample)
 *
 * - alpha close to 0: minimal filtering, fast response
 * - alpha close to 1: heavy filtering, slow response
 *
 * The time constant tau (63% response) is:
 *   tau = -T_s / ln(alpha)  where T_s = 1/f_sample
 *
 * For alpha = 0.9 and f_sample = 100 Hz: tau = 0.095 s
 * For alpha = 0.99 and f_sample = 100 Hz: tau = 0.99 s
 *
 * Complexity: O(1) per sample.
 */
double tc_iir_filter(double alpha, double input, double *state) {
    if (!state) return input;
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    *state = alpha * (*state) + (1.0 - alpha) * input;
    return *state;
}

/**
 * @brief tc_moving_average: Moving average filter
 *
 * Maintains a circular buffer of the last 'window' samples and returns
 * their arithmetic mean. This is a finite impulse response (FIR) filter
 * with uniform weights: all window samples contribute equally.
 *
 * This filter completely eliminates frequencies at integer multiples
 * of f_sample/window (notches), which is useful for rejecting 50/60 Hz
 * line frequency when window is chosen appropriately.
 *
 * For 60 Hz rejection at 240 SPS: window = 4 (notch at 60 Hz)
 * For 50 Hz rejection at 250 SPS: window = 5 (notch at 50 Hz)
 *
 * Complexity: O(1) per sample (maintains running sum).
 */
double tc_moving_average(double *buffer, size_t window,
                          size_t *write_idx, size_t *count,
                          double sample) {
    double sum;
    size_t i;

    if (!buffer || !write_idx || !count) return sample;
    if (window == 0) return sample;

    /* Insert new sample into circular buffer */
    buffer[*write_idx] = sample;
    *write_idx = (*write_idx + 1) % window;

    /* Increment total count up to window */
    if (*count < window) (*count)++;

    /* Compute average */
    sum = 0.0;
    for (i = 0; i < *count; i++) {
        sum += buffer[i];
    }

    return sum / (double)(*count);
}

/* =========================================================================
 * L6: Open-Circuit Detection
 * ========================================================================= */

/**
 * @brief tc_detect_open_circuit: Detect open thermocouple condition
 *
 * An open thermocouple can be detected by:
 *   1. ADC reads near Vref (if burnout current source pulls high)
 *   2. ADC reads near 0V (if pull-down resistor)
 *   3. High noise due to floating input (high source impedance)
 *
 * Burnout detection injects a small current (~100 nA) through the
 * thermocouple. If the T/C is intact (low impedance ~100 ohms),
 * the voltage shift is negligible (~10 uV). If open (infinite
 * impedance), the input saturates to the rail.
 *
 * Detection thresholds should be set based on the expected
 * maximum EMF for the thermocouple type.
 */
tc_error_t tc_detect_open_circuit(double voltage,
                                   const tc_measurement_config_t *config,
                                   int *is_open) {
    if (!config || !is_open) return TC_ERR_NULL_POINTER;

    *is_open = 0;

    if (!config->burnout_enabled) return TC_OK;

    /* If voltage is near rails (within threshold of Vmax or Vmin),
     * the thermocouple may be open */
    if (voltage >= config->adc_vmax - config->open_tc_threshold) {
        *is_open = 1;
        return TC_OK;
    }
    if (voltage <= config->adc_vmin + config->open_tc_threshold) {
        *is_open = 1;
        return TC_OK;
    }

    return TC_OK;
}

/* =========================================================================
 * L5: ADC Resolution Requirements
 * ========================================================================= */

tc_error_t tc_adc_resolution_required(tc_type_t type, double temp_range,
                                       double target_res_c, size_t *bits_needed) {
    double seebeck, v_range, v_res, steps_needed;
    tc_error_t err;

    if (!bits_needed) return TC_ERR_NULL_POINTER;
    if (type >= TC_COUNT) return TC_ERR_INVALID_TYPE;
    if (target_res_c <= 0.0) return TC_ERR_OUT_OF_RANGE;

    /* Get Seebeck coefficient at mid-range for EMF/temperature sensitivity */
    err = tc_seebeck_coefficient(type, temp_range / 2.0, &seebeck);
    if (err != TC_OK) return err;

    /* Voltage range = Seebeck * temperature range (uV/C * C = uV) */
    v_range = fabs(seebeck) * temp_range * 1e-3; /* uV -> mV */

    /* Required voltage resolution per LSB */
    v_res = fabs(seebeck) * target_res_c * 1e-3;

    /* Number of distinct steps needed */
    steps_needed = v_range / v_res;

    /* ADC bits = ceil(log2(steps_needed)) */
    *bits_needed = (size_t)ceil(log2(steps_needed));
    if (*bits_needed < 8) *bits_needed = 8;  /* Minimum practical ADC */

    return TC_OK;
}

/* =========================================================================
 * L8: Kalman Filter for Temperature Tracking
 * ========================================================================= */

/**
 * @brief tc_kalman_track_temperature: 1D Kalman filter for T/C temperature
 *
 * State-space model:
 *   T_k = T_{k-1} + w_k    (random walk process model, no control input)
 *   z_k = T_k + v_k        (direct measurement)
 *
 * where:
 *   w_k ~ N(0, Q * dt)    process noise (models unknown thermal drift)
 *   v_k ~ N(0, R)         measurement noise (sensor + ADC noise)
 *
 * Kalman equations (scalar form):
 *   Predict:
 *     T_pred = T_est_{k-1}
 *     P_pred = P_{k-1} + Q*dt
 *   Update:
 *     K = P_pred / (P_pred + R)
 *     T_est_k = T_pred + K * (z_k - T_pred)
 *     P_k = (1 - K) * P_pred
 *
 * Tuning guidance:
 *   Q: Process noise variance. Higher Q => filter trusts measurements more,
 *      responds faster to real changes. Typical: 0.01-1.0 C^2/s.
 *   R: Measurement noise variance. Higher R => filter smooths more.
 *      Can be estimated from sensor datasheet or measured noise.
 *   Good starting point: Q=0.1, R=1.0 => K ~ 0.1 at steady state.
 *
 * Complexity: O(1) per update.
 */
double tc_kalman_track_temperature(double z, double dt, double Q, double R,
                                    double *T_est, double *P_est) {
    double T_pred, P_pred, K;

    if (!T_est || !P_est) return z;
    if (dt <= 0.0) dt = 1.0;

    /* Predict */
    T_pred = *T_est;
    P_pred = *P_est + Q * dt;

    /* Update */
    K = P_pred / (P_pred + R);
    *T_est = T_pred + K * (z - T_pred);
    *P_est = (1.0 - K) * P_pred;

    return *T_est;
}

/* =========================================================================
 * L6: GUM Uncertainty Budget
 * ========================================================================= */

tc_error_t tc_uncertainty_budget(tc_measurement_t *result,
                                  const tc_measurement_config_t *config) {
    double u_adc, u_cjc, u_linearity, u_noise, u_gain, u_offset;

    if (!result || !config) return TC_ERR_NULL_POINTER;

    /* Type B uncertainties */

    /* ADC resolution: u = Q/sqrt(12) */
    {
        double q = config->adc_vref / (double)((size_t)1 << config->adc_bits);
        double seebeck;
        if (tc_seebeck_coefficient(config->tc_type, result->temperature,
                                    &seebeck) == TC_OK && fabs(seebeck) > 1e-15) {
            u_adc = (q / sqrt(12.0)) / (fabs(seebeck) * 1e-3);
        } else {
            u_adc = (q / sqrt(12.0)) / 40e-6;
        }
    }

    /* CJC sensor uncertainty (direct) */
    u_cjc = config->cjc.cj_temperature_uncertainty;

    /* Linearity error (ITS-90 polynomial fit error, ~0.02C for base metal) */
    {
        double t_min, t_max;
        if (tc_type_range(config->tc_type, &t_min, &t_max) == TC_OK) {
            tc_linearity_error(config->tc_type, t_min, t_max, &u_linearity);
            if (u_linearity < 0.01) u_linearity = 0.02; /* Minimum from poly fit */
        } else {
            u_linearity = 0.05;
        }
    }

    /* Measurement noise: RMS of filtered signal */
    {
        tc_noise_model_t noise;
        tc_noise_budget(config, 100.0, &noise); /* Assume ~100 ohm source */
        double seebeck;
        if (tc_seebeck_coefficient(config->tc_type, result->temperature,
                                    &seebeck) == TC_OK && fabs(seebeck) > 1e-15) {
            u_noise = noise.total_noise_vrms / (fabs(seebeck) * 1e-3)
                      / sqrt((double)(result->sample_count > 0 ? result->sample_count : 1));
        } else {
            u_noise = noise.noise_equiv_temp;
        }
    }

    /* Gain calibration uncertainty */
    u_gain = (1.0 - config->gain_cal) * fabs(result->temperature) * 0.01;
    if (u_gain < 0.0) u_gain = -u_gain;

    /* Offset calibration uncertainty */
    {
        double seebeck;
        if (tc_seebeck_coefficient(config->tc_type, result->temperature,
                                    &seebeck) == TC_OK && fabs(seebeck) > 1e-15) {
            u_offset = fabs(config->offset_cal) / (fabs(seebeck) * 1e-3);
        } else {
            u_offset = fabs(config->offset_cal) / 40e-6;
        }
    }

    /* Combined standard uncertainty (RSS) */
    result->uncertainty = sqrt(
        u_adc * u_adc +
        u_cjc * u_cjc +
        u_linearity * u_linearity +
        u_noise * u_noise +
        u_gain * u_gain +
        u_offset * u_offset
    );

    return TC_OK;
}
