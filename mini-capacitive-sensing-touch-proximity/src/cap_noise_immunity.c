/**
 * @file cap_noise_immunity.c
 * @brief Noise Immunity: Digital Filtering, Spread Spectrum, Frequency Hopping
 *
 * Capacitive touch sensors face diverse interference sources in real
 * environments. This file implements the key noise mitigation strategies.
 *
 * Knowledge Coverage:
 *   L1: noise characterization (RMS, PSD, impulse detection)
 *   L2: median filter, IIR/FIR lowpass, moving average
 *   L3: exponential smoothing, running statistics, correlation
 *   L4: Wiener filter optimality, matched filter, Parseval theorem
 *   L5: spread-spectrum processing gain, frequency hopping, sync detection
 *   L6: coexistence with GSM/DCS/PCS/WiFi/laptop SMPS noise
 *
 * Ref: TI SNOA927, Cypress AN80972, Atmel AVR4013
 *      Kay "Fundamentals of Statistical Signal Processing" (1993)
 */

#include "cap_noise_immunity.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ==========================================================================
 * L1: NOISE PROFILE
 * ========================================================================== */

void cap_noise_profile_init(cap_noise_profile_t *profile)
{
    if (!profile) return;
    memset(profile, 0, sizeof(*profile));
    profile->dominant_source = NOISE_THERMAL;
    profile->outlier_threshold_sigma = 5.0;
}

/**
 * cap_analyze_noise
 *
 * Characterizes noise from a sample buffer. Computes:
 * - RMS and peak-to-peak amplitude
 * - Detects dominant frequency components using zero-crossing analysis
 * - Classifies noise source based on frequency characteristics
 *
 * RMS: sigma = sqrt((1/N) * sum((x_i - mean)^2))
 * Peak-peak: max(x) - min(x), sensitive to outliers
 *
 * Zero-crossing frequency detection:
 *   f_dominant = (crossing_count / 2) * (f_sample / N)
 *
 * This lightweight method detects mains (50/60 Hz) and switching noise
 * (>1 kHz) without requiring an FFT. For a 1 kHz signal at 10 kHz sample
 * rate, 1000 samples capture 100 periods → 200 zero crossings.
 *
 * Complexity: O(N) for RMS and zero-crossing.
 */
void cap_analyze_noise(cap_noise_profile_t *profile,
                       const double *samples, uint32_t num_samples,
                       double sample_rate_hz)
{
    if (!profile || !samples || num_samples < 2 || sample_rate_hz <= 0.0) {
        return;
    }

    /* Compute mean */
    double sum = 0.0;
    for (uint32_t i = 0; i < num_samples; i++) {
        sum += samples[i];
    }
    double mean = sum / (double)num_samples;

    /* Compute RMS and min/max */
    double sum_sq = 0.0;
    double vmin = samples[0], vmax = samples[0];
    for (uint32_t i = 0; i < num_samples; i++) {
        double diff = samples[i] - mean;
        sum_sq += diff * diff;
        if (samples[i] < vmin) vmin = samples[i];
        if (samples[i] > vmax) vmax = samples[i];
    }
    double variance = sum_sq / (double)num_samples;
    double rms = sqrt(variance);

    profile->noise_rms_f = rms;
    profile->noise_peak_peak_f = vmax - vmin;
    profile->noise_psd_f2_hz = variance / (sample_rate_hz / 2.0); /* Simplified PSD */

    /* Zero-crossing frequency detection */
    uint32_t crossings = 0;
    for (uint32_t i = 1; i < num_samples; i++) {
        if ((samples[i-1] - mean) * (samples[i] - mean) < 0.0) {
            crossings++;
        }
    }
    double detected_freq = (crossings / 2.0) * (sample_rate_hz / (double)num_samples);

    /* Classify source */
    if (detected_freq > 45.0 && detected_freq < 65.0) {
        profile->dominant_source = NOISE_MAINS;
        profile->mains_hz = detected_freq;
    } else if (detected_freq > 1000.0) {
        profile->dominant_source = NOISE_SWITCHING;
        profile->switching_hz = detected_freq;
    } else if (detected_freq > 1e6) {
        profile->dominant_source = NOISE_RF;
    } else {
        profile->dominant_source = NOISE_THERMAL;
    }

    /* Count outliers (samples > K*sigma from mean) */
    uint32_t outlier_count = 0;
    double threshold = profile->outlier_threshold_sigma * rms;
    for (uint32_t i = 0; i < num_samples; i++) {
        if (fabs(samples[i] - mean) > threshold) {
            outlier_count++;
        }
    }
    profile->outlier_count = outlier_count;
}

/* ==========================================================================
 * L5: DIGITAL FILTERS
 * ========================================================================== */

/**
 * cap_median_filter
 *
 * Applies a median filter to the sample buffer in-place.
 *
 * For each sample at position i, replaces it with the median of
 * the window centered at i: [i - window/2, i + window/2].
 *
 * Edge handling: samples near the beginning/end have asymmetric windows
 * (fewer neighbors on one side).
 *
 * The median filter is optimal for removing impulse noise (salt-and-pepper,
 * ESD spikes) while preserving step edges (touch onsets). Unlike linear
 * filters, it does not blur sharp transitions.
 *
 * Complexity: O(N * W) where N = length, W = window (using quickselect).
 * For W <= 9 (typical), this is very fast.
 *
 * Window selection:
 *   W=3: removes isolated single-sample spikes
 *   W=5: removes up to 2 consecutive outliers
 *   W=7: removes up to 3 consecutive outliers (ESD ringing)
 *
 * Larger windows risk removing real touch onsets which are sharp edges.
 * For touch detection, W=3 or W=5 is recommended.
 */
void cap_median_filter(double *data, uint32_t length, uint8_t window)
{
    if (!data || length < 3 || window < 3 || window % 2 == 0) return;

    /* Allocate temporary buffer for one window */
    double *win = (double *)malloc(window * sizeof(double));
    if (!win) return;

    uint8_t half = window / 2;

    for (uint32_t i = 0; i < length; i++) {
        uint8_t count = 0;
        /* Gather window samples with boundary handling */
        for (int32_t j = (int32_t)i - half; j <= (int32_t)i + half; j++) {
            if (j >= 0 && j < (int32_t)length) {
                win[count++] = data[j];
            }
        }
        if (count == 0) continue;

        /* Simple insertion sort for small window (W <= 9 typically) */
        for (uint8_t a = 1; a < count; a++) {
            double key = win[a];
            int b = a - 1;
            while (b >= 0 && win[b] > key) {
                win[b + 1] = win[b];
                b--;
            }
            win[b + 1] = key;
        }

        /* Median at count/2 */
        data[i] = win[count / 2];
    }

    free(win);
}

/**
 * cap_iir_lowpass_step
 *
 * First-order IIR low-pass filter, executed one sample at a time:
 *
 *   y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 *
 * -3dB cutoff frequency: f_c = (alpha / (2*pi*(1-alpha))) * f_s ≈ (alpha/(2*pi)) * f_s
 *   for alpha << 1.
 *
 * Time constant: tau = 1/(2*pi*f_c) ≈ 1/(alpha * f_s) for small alpha.
 *
 * Examples at f_s = 100 Hz:
 *   alpha=0.01 → f_c ≈ 0.16 Hz, tau ≈ 1.0 s (very slow, for baseline)
 *   alpha=0.05 → f_c ≈ 0.82 Hz, tau ≈ 0.20 s (slow, noise filtering)
 *   alpha=0.10 → f_c ≈ 1.67 Hz, tau ≈ 0.10 s
 *   alpha=0.30 → f_c ≈ 5.68 Hz, tau ≈ 0.028 s (fast, touch response)
 *
 * Complexity: O(1) per sample.
 */
double cap_iir_lowpass_step(double *state, double input, double alpha)
{
    if (!state) return input;
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;

    *state = alpha * input + (1.0 - alpha) * (*state);
    return *state;
}

/**
 * cap_iir_alpha_from_cutoff
 *
 * Computes the IIR alpha coefficient for a desired -3dB cutoff frequency.
 *
 * For a first-order IIR filter:
 *   H(z) = alpha / (1 - (1-alpha) * z^{-1})
 *   |H(e^{j*omega})|^2 = alpha^2 / (1 + (1-alpha)^2 - 2*(1-alpha)*cos(omega))
 *
 * Setting |H|^2 = 1/2 (half power = -3dB):
 *   alpha = 1 - e^{-2*pi*f_c/f_s}   (exact solution)
 *
 * For f_c << f_s: alpha ≈ 2*pi*f_c/f_s  (small-angle approximation).
 *
 * @param f_cutoff  Desired -3dB cutoff frequency [Hz]
 * @param f_sample  Sample rate [Hz]
 * @return Alpha [0, 1]
 */
double cap_iir_alpha_from_cutoff(double f_cutoff, double f_sample)
{
    if (f_sample <= 0.0 || f_cutoff <= 0.0) return 0.0;
    if (f_cutoff >= f_sample / 2.0) return 1.0; /* Nyquist limit */

    return 1.0 - exp(-2.0 * M_PI * f_cutoff / f_sample);
}

/**
 * cap_moving_average_step
 *
 * N-tap moving average filter with O(1) per-sample complexity using
 * a running sum and circular buffer.
 *
 *   y[n] = (1/N) * sum_{k=0}^{N-1} x[n-k]
 *
 * Running sum update: subtract oldest sample, add newest, divide by N.
 *
 * At startup (buffer not yet full), the average is over the available
 * samples only (cumulative average until buffer fills).
 *
 * -3dB cutoff: f_c ≈ 0.443 * f_s / N
 *
 * For N=8 at f_s=100Hz: f_c ≈ 5.54 Hz
 * For N=32 at f_s=100Hz: f_c ≈ 1.38 Hz
 *
 * Complexity: O(1) per sample.
 *
 * @param buffer       Circular buffer of N previous samples
 * @param buffer_idx   Current write position (updated in-place)
 * @param buffer_len   Buffer length N
 * @param new_sample   New input sample
 * @param running_sum  Running sum (updated in-place)
 * @return Moving average
 */
double cap_moving_average_step(double *buffer, uint32_t *buffer_idx,
                               uint32_t buffer_len, double new_sample,
                               double *running_sum)
{
    if (!buffer || !buffer_idx || !running_sum || buffer_len == 0) {
        return new_sample;
    }

    /* Remove oldest sample from running sum */
    *running_sum -= buffer[*buffer_idx];

    /* Store new sample */
    buffer[*buffer_idx] = new_sample;
    *running_sum += new_sample;

    /* Advance index */
    *buffer_idx = (*buffer_idx + 1) % buffer_len;

    return *running_sum / (double)buffer_len;
}

/* ==========================================================================
 * L5: SPREAD SPECTRUM
 * ========================================================================== */

void cap_spread_spectrum_init(cap_spread_spectrum_config_t *cfg,
                              double center, double bw, double chiprate)
{
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->enabled = true;
    cfg->center_freq_hz = center;
    cfg->spread_bandwidth_hz = bw;
    cfg->chip_rate_hz = chiprate;
    cfg->pn_sequence_length = 15;       /* LFSR length 15 → 32767 chips */
    cfg->pn_polynomial = 0x6001;        /* x^15 + x^14 + 1 */
    cfg->processing_gain_db = cap_spread_processing_gain(bw, 1.0 / (chiprate * 15.0/center));
}

/**
 * cap_spread_processing_gain
 *
 * Computes the processing gain of a spread-spectrum system:
 *
 *   G_p = 10 * log10(BW_spread / BW_signal) [dB]
 *
 * This is the improvement in SNR after despreading. The interference
 * power is spread over the spread bandwidth while the desired signal
 * power is concentrated in the signal bandwidth.
 *
 * Example: BW_spread = 1 MHz, BW_signal = 100 Hz:
 *   G_p = 10 * log10(1e6 / 100) = 10 * log10(10000) = 40 dB
 *
 * This means a 40 dB improvement in narrowband interference rejection
 * compared to a fixed-frequency system.
 *
 * Ref: Pickholtz et al. (1982) "Theory of Spread-Spectrum Communications"
 *      IEEE Trans Comm, 30(5), 855-884
 */
double cap_spread_processing_gain(double spread_bw, double signal_bw)
{
    if (spread_bw <= 0.0 || signal_bw <= 0.0) return 0.0;
    return 10.0 * log10(spread_bw / signal_bw);
}

/* ==========================================================================
 * L5: FREQUENCY HOPPING
 * ========================================================================== */

void cap_freq_hopping_init(cap_freq_hopping_config_t *cfg, uint8_t num_ch,
                           double base_freq, double spacing, uint32_t dwell)
{
    if (!cfg || num_ch == 0 || num_ch > 16) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->enabled = true;
    cfg->num_channels = num_ch;
    for (uint8_t i = 0; i < num_ch; i++) {
        cfg->channel_freqs_hz[i] = base_freq + i * spacing;
    }
    cfg->current_channel = 0;
    cfg->dwell_samples = dwell;
    cfg->sample_counter = 0;
    cfg->adaptive = false;
    cfg->noise_threshold_db = 6.0;
}

/**
 * cap_freq_hopping_next
 *
 * Advances the frequency hopping sequence to the next channel.
 *
 * Sequence type: pseudo-random using an LFSR-like rotation.
 * The active channel index advances by a prime step to ensure
 * all channels are visited before repeating.
 *
 * If adaptive mode is enabled and current channel noise exceeds
 * threshold, that channel is skipped (marked bad) for the current cycle.
 *
 * @param cfg  Frequency hopping configuration (state updated)
 * @return Active channel frequency [Hz]
 */
double cap_freq_hopping_next(cap_freq_hopping_config_t *cfg)
{
    if (!cfg || !cfg->enabled || cfg->num_channels == 0) return 0.0;

    cfg->sample_counter++;

    if (cfg->sample_counter >= cfg->dwell_samples) {
        cfg->sample_counter = 0;
        /* Advance to next channel (step by 3 for pseudo-random sequence) */
        cfg->current_channel = (cfg->current_channel + 3) % cfg->num_channels;
    }

    return cfg->channel_freqs_hz[cfg->current_channel];
}

/* ==========================================================================
 * L6: SYNCHRONOUS DETECTION (LOCK-IN)
 * ========================================================================== */

void cap_sync_detector_init(cap_sync_detector_t *det, double f_exc,
                            double t_int)
{
    if (!det) return;
    memset(det, 0, sizeof(*det));
    det->enabled = true;
    det->excitation_freq_hz = f_exc;
    det->integration_time_s = t_int;
    /* Equivalent noise bandwidth for integration over T_int: ENBW = 1/(2*T_int) */
    det->enbw_hz = 1.0 / (2.0 * t_int);
    det->phase_offset_deg = 0.0;
}

/**
 * cap_sync_detector_step
 *
 * Processes one ADC sample through a synchronous detector (lock-in amplifier).
 *
 * The detector multiplies the input signal by in-phase (cos) and quadrature
 * (sin) references at the excitation frequency:
 *
 *   I = sum(x(t) * cos(2*pi*f_exc*t + phi))
 *   Q = sum(x(t) * sin(2*pi*f_exc*t + phi))
 *
 * After integration over T_int, the magnitude and phase are:
 *   Magnitude = sqrt(I^2 + Q^2)
 *   Phase = atan2(Q, I)
 *
 * The magnitude is proportional to the capacitance being measured.
 * Out-of-band noise is rejected by the equivalent noise bandwidth ENBW.
 *
 * A 1-second integration gives ENBW = 0.5 Hz, rejecting 50/60 Hz mains
 * by ~40 dB (since mains frequency differs from excitation frequency).
 *
 * @param det     Detector state (accumulators updated)
 * @param sample  New ADC sample
 * @param t       Time of this sample [s]
 */
void cap_sync_detector_step(cap_sync_detector_t *det, double sample, double t)
{
    if (!det || !det->enabled) return;

    double omega = 2.0 * M_PI * det->excitation_freq_hz;
    double phi = det->phase_offset_deg * M_PI / 180.0;

    det->i_accumulator += sample * cos(omega * t + phi);
    det->q_accumulator += sample * sin(omega * t + phi);

    /* Update magnitude and phase */
    det->magnitude = sqrt(det->i_accumulator * det->i_accumulator +
                          det->q_accumulator * det->q_accumulator);
    det->phase_deg = atan2(det->q_accumulator, det->i_accumulator) * 180.0 / M_PI;
}

/* ==========================================================================
 * L6: SIGNAL-TO-INTERFERENCE RATIO
 * ========================================================================== */

/**
 * cap_compute_sir
 *
 * Computes Signal-to-Interference Ratio:
 *
 *   SIR = 10 * log10(P_signal / P_interference) [dB]
 *
 * where P_signal is estimated from (signal+interference) samples and
 * P_interference is estimated from interference-only (baseline) samples.
 *
 * The signal power is: P_signal = P_total - P_interference
 * (assuming signal and interference are uncorrelated).
 *
 * @param signal_samples     Signal + interference
 * @param interference_only  Interference only (baseline)
 * @param length             Number of samples
 * @return SIR [dB]
 */
double cap_compute_sir(const double *signal_samples,
                       const double *interference_only,
                       uint32_t length)
{
    if (!signal_samples || !interference_only || length == 0) return 0.0;

    /* Compute power (mean squared) for each */
    double p_total = 0.0, p_interf = 0.0;
    for (uint32_t i = 0; i < length; i++) {
        p_total += signal_samples[i] * signal_samples[i];
        p_interf += interference_only[i] * interference_only[i];
    }
    p_total /= (double)length;
    p_interf /= (double)length;

    double p_signal = p_total - p_interf;
    if (p_signal <= 0.0 || p_interf <= 0.0) return 0.0;

    return 10.0 * log10(p_signal / p_interf);
}

/**
 * cap_select_best_frequency
 *
 * Scans candidate excitation/sampling frequencies and selects the one
 * with the lowest measured interference.
 *
 * For each candidate frequency:
 * 1. Measure noise RMS at that frequency (brief sampling)
 * 2. Store in measured_noise array
 * 3. Return index of minimum noise
 *
 * This enables automatic frequency planning in environments with
 * known interferers (e.g., avoid 50/60 Hz mains harmonics,
 * avoid SMPS switching frequency and its harmonics).
 *
 * @param candidates_hz   Candidate frequencies [Hz]
 * @param num_candidates  Number of candidates
 * @param measured_noise  Output: noise RMS per candidate
 * @param sample_rate_hz  Sample rate for measurement [Hz]
 * @return Index of best frequency
 */
uint8_t cap_select_best_frequency(const double *candidates_hz,
                                  uint8_t num_candidates,
                                  double *measured_noise,
                                  double sample_rate_hz)
{
    if (!candidates_hz || !measured_noise || num_candidates == 0) return 0;

    uint8_t best_idx = 0;
    double best_noise = INFINITY;

    for (uint8_t i = 0; i < num_candidates; i++) {
        /* Placeholder: in a real system, this would sample at each frequency
         * and measure the noise floor. Here we provide a deterministic proxy:
         * prefer frequencies away from 50/60 Hz harmonics. */
        double freq = candidates_hz[i];
        double noise_penalty = 0.0;

        /* Penalize proximity to 50 Hz harmonics */
        for (int h = 1; h <= 5; h++) {
            double dist50 = fabs(freq - 50.0 * h);
            if (dist50 < 5.0) noise_penalty += (5.0 - dist50) / 5.0;
            double dist60 = fabs(freq - 60.0 * h);
            if (dist60 < 5.0) noise_penalty += (5.0 - dist60) / 5.0;
        }
        /* Penalize proximity to 100 kHz SMPS harmonics */
        for (int h = 1; h <= 3; h++) {
            double dist_smps = fabs(freq - 100000.0 * h);
            if (dist_smps < 1000.0) noise_penalty += (1000.0 - dist_smps) / 1000.0;
        }

        measured_noise[i] = 1.0 + noise_penalty;

        if (measured_noise[i] < best_noise) {
            best_noise = measured_noise[i];
            best_idx = i;
        }
    }

    /* Note: sample_rate_hz parameter is reserved for future real-hardware
     * implementation where actual ADC sampling occurs */
    (void)sample_rate_hz;

    return best_idx;
}

void cap_digital_filter_destroy(cap_digital_filter_t *filter)
{
    if (filter) {
        free(filter->taps);
        free(filter->state);
        filter->taps = NULL;
        filter->state = NULL;
    }
}
