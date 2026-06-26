/**
 * bridge_conditioning.c — Bridge Signal Conditioning Implementation
 *
 * Knowledge Coverage:
 *   L2: Gain staging, signal chain analysis
 *   L3: Noise analysis, SNR optimization, anti-alias filter design
 *   L4: Nyquist theorem application, quantization noise analysis
 *   L5: Digital filtering (moving average, median, FIR, decimation)
 *
 * Reference:
 *   - Kester, "Sensor Signal Conditioning", Analog Devices, 1999
 *   - Oppenheim & Schafer, "Discrete-Time Signal Processing", 3rd ed.
 *   - Hogenauer, "An Economical Class of Digital Filters for Decimation
 *                  and Interpolation", IEEE ASSP 1981
 */

#include "bridge_conditioning.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ========================================================================
 * L2: Signal Chain Analysis
 * ======================================================================== */

double conditioning_required_gain(double vref_adc, double vex, double gf,
                                  double eps_fs_ue, bridge_config_t config)
{
    /* Compute required amplifier gain to match bridge output
     * to ADC input range.
     *
     * The full-scale bridge output:
     *   Vout_FS = Vexc * S * eps_FS
     *   where S = GF/N (N = 4, 2, 1 depending on config)
     *
     * Required gain:
     *   Gain = Vref_ADC / Vout_FS
     *
     * This assumes the ADC input range is 0 to +Vref.
     * For bipolar ADC: Gain = Vref_ADC / (2 * Vout_FS).
     */

    if (vex <= 0.0 || gf <= 0.0 || vref_adc <= 0.0) return 1.0;

    double eps_abs = eps_fs_ue * 1.0e-6;
    double n_factor;

    switch (config) {
        case BRIDGE_QUARTER:  n_factor = 4.0; break;
        case BRIDGE_HALF:     n_factor = 2.0; break;
        case BRIDGE_FULL:     n_factor = 1.0; break;
        default:              n_factor = 4.0; break;
    }

    double vout_fs = vex * (gf / n_factor) * eps_abs;
    if (vout_fs <= 0.0) return 1.0e9;  /* Avoid division by zero */

    return vref_adc / vout_fs;
}

double conditioning_total_noise(const signal_chain_t *chain,
                                double bridge_r_ohm, double temp_k,
                                double in_noise_pa)
{
    /* Total input-referred noise of the complete signal chain.
     *
     * Noise sources (all referred to input):
     *
     * 1. Bridge Johnson noise:
     *    Vn_johnson = sqrt(4 * kB * T * R_bridge * BW)
     *
     * 2. Amplifier input voltage noise:
     *    Vn_amp = e_n * sqrt(BW)
     *    where e_n is the input voltage noise density [V/rtHz]
     *
     * 3. Amplifier current noise flowing through bridge impedance:
     *    Vn_i = i_n * R_bridge * sqrt(BW)
     *    where i_n is the input current noise density [A/rtHz]
     *
     * 4. ADC quantization noise (referred to input):
     *    Vn_adc_rti = Vref_ADC / (2^N * sqrt(12)) / Gain
     *
     * All uncorrelated → RSS combination:
     *    Vn_total_rti = sqrt(Vn1^2 + Vn2^2 + Vn3^2 + Vn4^2)
     */

    double bw = chain->bandwidth_hz;
    if (bw <= 0.0) bw = chain->sample_rate_hz / 2.0;

    /* Johnson noise */
    double vn_johnson = sqrt(4.0 * 1.380649e-23 * temp_k * bridge_r_ohm * bw);

    /* Amplifier voltage noise */
    double en_v_per_rt_hz = chain->noise_rti_nv_per_rt_hz * 1.0e-9;
    double vn_amp = en_v_per_rt_hz * sqrt(bw);

    /* Amplifier current noise */
    double in_a_per_rt_hz = in_noise_pa * 1.0e-12;
    double vn_i = in_a_per_rt_hz * bridge_r_ohm * sqrt(bw);

    /* ADC quantization noise (referred to input) */
    double adc_vn = chain->adc_reference_v /
                    (pow(2.0, chain->adc_resolution_bits) * sqrt(12.0));
    double vn_adc_rti = adc_vn / chain->total_gain;

    /* RSS total */
    return sqrt(vn_johnson * vn_johnson +
                vn_amp    * vn_amp +
                vn_i      * vn_i +
                vn_adc_rti * vn_adc_rti);
}

double conditioning_enob(double v_signal_fs, double v_noise_rms)
{
    /* Effective Number of Bits (ENOB).
     *
     * ENOB = (SNR_dB - 1.76) / 6.02
     *
     * where SNR_dB = 20*log10(V_signal_FS / V_noise_rms)
     *
     * The 1.76 dB offset accounts for the quantization noise of
     * an ideal ADC: SNR_ideal = 6.02*N + 1.76 [dB]
     *
     * So: N_effective = (SNR_actual - 1.76) / 6.02
     *
     * If ENOB < ADC native resolution, noise is the limiter.
     * If ENOB ≈ ADC resolution, the ADC is the limiter.
     *
     * For precision bridge measurement with 24-bit sigma-delta ADC:
     *   AD779x/AD7124 achieve ENOB ~ 16-20 bits at low data rates.
     *
     * Reference: IEEE Std 1241-2010
     */

    if (v_signal_fs <= 0.0 || v_noise_rms <= 0.0) return 0.0;

    double snr_db = 20.0 * log10(v_signal_fs / v_noise_rms);
    return (snr_db - 1.76) / 6.02;
}

double conditioning_snr(const signal_chain_t *chain,
                        const bridge_state_t *bridge,
                        double strain_ue)
{
    /* Signal-to-Noise Ratio for a given strain.
     *
     * SNR = V_signal / V_noise_total
     *
     * V_signal = Vexc * GF/N * eps * Gain (amplified output)
     * V_noise = total RTI noise * Gain (same gain factor cancels!)
     *
     * Therefore SNR_db = 20*log10(V_signal_RTI / V_noise_RTI)
     * and gain does NOT affect SNR (first-order).
     *
     * SNR improvement strategies:
     * 1. Higher Vexc (increases signal linearly)
     * 2. Lower bridge R (lower Johnson noise) — trade-off with self-heating
     * 3. Narrower bandwidth (reduce noise bandwidth)
     * 4. Use full bridge (4x signal vs quarter bridge)
     * 5. Oversampling + averaging (improves SNR by sqrt(OSR))
     */

    double eps_abs = strain_ue * 1.0e-6;
    double n_factor;
    switch (bridge->config) {
        case BRIDGE_QUARTER:  n_factor = 4.0; break;
        case BRIDGE_HALF:     n_factor = 2.0; break;
        case BRIDGE_FULL:     n_factor = 1.0; break;
        default:              n_factor = 4.0; break;
    }

    /* Assume GF=2.05 for SNR estimation */
    double v_signal_rti = bridge->v_excitation * (2.05 / n_factor) * eps_abs;

    /* Rough noise estimate */
    double bridge_r = bridge_output_impedance(bridge);
    double v_noise_rti = conditioning_total_noise(chain, bridge_r, 300.0, 1.0);

    if (v_noise_rti <= 0.0) return 200.0;  /* No noise = infinite SNR */
    return 20.0 * log10(v_signal_rti / v_noise_rti);
}

/* ========================================================================
 * L3: Anti-Alias Filter Design
 * ======================================================================== */

void conditioning_design_antialias(double sample_rate_hz,
                                   double signal_bw_hz,
                                   anti_alias_filter_t *filter_out)
{
    /* Design anti-aliasing filter for given ADC sample rate.
     *
     * Nyquist theorem: sample rate Fs must be > 2 * signal BW
     * to avoid aliasing (Nyquist 1928, Shannon 1949).
     *
     * Practical anti-alias filter design:
     * - Cutoff Fc = signal BW (preserve signal)
     * - Stopband starts at Fs - Fc (oversampling eases filter)
     * - Stopband attenuation > ADC SNR + 10 dB margin
     *
     * For oversampled systems (Fs >> 2*BW), the transition band
     * is wide, allowing low-order filters.
     *
     * For Nyquist-rate systems (Fs ≈ 2.2*BW), the transition
     * is very narrow, requiring high-order filters.
     */

    memset(filter_out, 0, sizeof(*filter_out));

    filter_out->type          = FILTER_BUTTERWORTH;
    filter_out->cutoff_hz     = signal_bw_hz;
    filter_out->sample_rate_hz = sample_rate_hz;

    /* Stopband starts at Fs - Fc (Nyquist frequency minus cutoff) */
    filter_out->stopband_hz   = sample_rate_hz - signal_bw_hz;

    /* Target stopband attenuation: 80 dB typical for 16-bit systems */
    filter_out->stopband_atten_db = 80.0;
    filter_out->passband_ripple_db = 0.1;

    /* Determine required order */
    filter_out->order = conditioning_filter_order(
        filter_out->cutoff_hz,
        filter_out->stopband_hz,
        filter_out->stopband_atten_db,
        filter_out->passband_ripple_db);

    /* Clamp to reasonable range */
    if (filter_out->order < 1) filter_out->order = 1;
    if (filter_out->order > 8) filter_out->order = 8;
}

int conditioning_filter_order(double f_pass_hz, double f_stop_hz,
                              double a_stop_db, double a_pass_db)
{
    /* Compute required Butterworth filter order.
     *
     * Butterworth filter magnitude response:
     *   |H(f)| = 1 / sqrt(1 + (f/fc)^(2N))
     *
     * Attenuation at stopband:
     *   A_stop = 10*log10(1 + (f_stop/fc)^(2N))
     *
     * Solving for N:
     *   N >= log10((10^(A_stop/10) - 1) / (10^(A_pass/10) - 1))
     *        / (2 * log10(f_stop/f_pass))
     *
     * This gives the minimum integer order.
     */

    if (f_pass_hz <= 0.0 || f_stop_hz <= f_pass_hz) return 1;

    double a_stop_linear  = pow(10.0, a_stop_db / 10.0) - 1.0;
    double a_pass_linear  = pow(10.0, a_pass_db / 10.0) - 1.0;
    double ratio          = f_stop_hz / f_pass_hz;

    if (a_pass_linear <= 0.0) a_pass_linear = 1.0e-6;

    double n_float = log10(a_stop_linear / a_pass_linear)
                     / (2.0 * log10(ratio));

    int n = (int)ceil(n_float);
    return (n < 1) ? 1 : n;
}

/* ========================================================================
 * L5: Digital Signal Processing
 * ======================================================================== */

int conditioning_maf_init(moving_average_filter_t *maf, int window_size)
{
    /* Initialize moving average filter.
     *
     * Allocates circular buffer for O(1) incremental updates.
     * The moving average is the optimal filter for white noise
     * reduction when the signal is stationary (constant).
     *
     * For step changes, the moving average has a linear ramp
     * response (N samples to fully respond to a step).
     */

    if (maf == NULL || window_size < 1) return -1;

    maf->window_size = window_size;
    maf->index = 0;
    maf->sum   = 0.0;
    maf->count = 0;

    maf->buffer = (double *)calloc((size_t)window_size, sizeof(double));
    if (maf->buffer == NULL) return -1;

    return 0;
}

double conditioning_maf_process(moving_average_filter_t *maf, double sample)
{
    /* Process one sample through moving average filter.
     *
     * Uses O(1) incremental algorithm:
     *
     * When buffer not yet full (count < window_size):
     *   buffer[count] = sample
     *   sum += sample
     *   count++
     *   return sum / count
     *
     * When buffer full:
     *   oldest = buffer[index]
     *   buffer[index] = sample
     *   sum = sum - oldest + sample
     *   index = (index + 1) % window_size
     *   return sum / window_size
     *
     * This avoids O(N) per sample.
     */

    if (maf == NULL || maf->buffer == NULL) return sample;

    if (maf->count < maf->window_size) {
        /* Buffer not yet full */
        maf->buffer[maf->count] = sample;
        maf->sum += sample;
        maf->count++;
        return maf->sum / (double)maf->count;
    } else {
        /* Buffer full — replace oldest */
        double oldest = maf->buffer[maf->index];
        maf->buffer[maf->index] = sample;
        maf->sum = maf->sum - oldest + sample;
        maf->index = (maf->index + 1) % maf->window_size;
        return maf->sum / (double)maf->window_size;
    }
}

void conditioning_maf_reset(moving_average_filter_t *maf)
{
    if (maf == NULL) return;
    maf->index = 0;
    maf->sum   = 0.0;
    maf->count = 0;
    if (maf->buffer != NULL) {
        memset(maf->buffer, 0, (size_t)maf->window_size * sizeof(double));
    }
}

void conditioning_maf_free(moving_average_filter_t *maf)
{
    if (maf != NULL) {
        free(maf->buffer);
        maf->buffer = NULL;
        maf->window_size = 0;
        maf->count = 0;
    }
}

void conditioning_median_filter(double *data, int n_samples, int window_size)
{
    /* Median filter — reject impulse noise while preserving edges.
     *
     * For each sample i, we take the window of window_size samples
     * centered at i (or near the edges), sort them, and take the
     * median value.
     *
     * For odd window_size = 2k+1, the median is the (k+1)-th
     * smallest element.
     *
     * The median filter is nonlinear — its frequency response
     * depends on the signal statistics. It excels at removing
     * salt-and-pepper noise and single-sample spikes.
     *
     * Complexity: O(N * W * log W) using sorting within each window.
     * For real-time implementation, use a more efficient O(N * log W)
     * algorithm based on balanced trees or heaps.
     */

    if (data == NULL || n_samples < 1 || window_size < 3) return;
    if (window_size % 2 == 0) window_size++;  /* Ensure odd */

    double *window = (double *)malloc((size_t)window_size * sizeof(double));
    if (window == NULL) return;

    double *output = (double *)malloc((size_t)n_samples * sizeof(double));
    if (output == NULL) {
        free(window);
        return;
    }

    int half = window_size / 2;
    int i, j, k;

    for (i = 0; i < n_samples; i++) {
        /* Collect window samples with proper boundary handling */
        int w_count = 0;
        for (j = i - half; j <= i + half; j++) {
            if (j >= 0 && j < n_samples) {
                window[w_count++] = data[j];
            }
        }

        /* Sort the window (simple insertion sort for small windows) */
        for (j = 1; j < w_count; j++) {
            double key = window[j];
            k = j - 1;
            while (k >= 0 && window[k] > key) {
                window[k + 1] = window[k];
                k--;
            }
            window[k + 1] = key;
        }

        /* Median is the middle element */
        output[i] = window[w_count / 2];
    }

    /* Copy back */
    memcpy(data, output, (size_t)n_samples * sizeof(double));

    free(window);
    free(output);
}

int conditioning_decimate(const double *input, int n_input,
                          int decim_factor, double *output)
{
    /* Decimation by integer factor M.
     *
     * output[k] = input[k * M] for k = 0, 1, ..., n_input/M - 1
     *
     * Pre-condition: input has been low-pass filtered to Fs/(2M)
     * to prevent aliasing.
     *
     * For bridge sensor oversampling:
     * - ADC samples at Fs_adc (e.g., 9600 SPS)
     * - Signal BW is Fs_sig (e.g., 10 Hz)
     * - Anti-alias filter at 10 Hz (analog)
     * - Decimate by M = Fs_adc / (2.56 * Fs_sig) for good margin
     *   (2.56 = 2 * 1.28 for practical filter transition)
     * - Output rate = Fs_adc / M (e.g., 9600/375 ≈ 25.6 SPS)
     */

    if (input == NULL || output == NULL || n_input < 1 || decim_factor < 1) {
        return 0;
    }

    int n_output = n_input / decim_factor;
    int i;

    for (i = 0; i < n_output; i++) {
        output[i] = input[i * decim_factor];
    }

    return n_output;
}

int conditioning_oversampling_ratio(int adc_bits, int target_bits)
{
    /* Required oversampling ratio for target ENOB.
     *
     * Oversampling and averaging reduces uncorrelated noise:
     *   SNR improvement = 10*log10(OSR) [dB]
     *   ENOB improvement = log2(OSR) / 2 [bits]
     *
     * Therefore:
     *   OSR = 4^(target_bits - adc_bits)
     *
     * Example: 12-bit ADC → 16-bit effective
     *   delta = 4 bits
     *   OSR = 4^4 = 256
     *
     * The ADC must sample 256x faster than the Nyquist rate.
     * For 10 Hz signal BW: Fs_nyq = 20 Hz, Fs_oversampled = 5120 Hz.
     * This is easily achievable with modern ADCs (> 100 kSPS).
     *
     * Caveat: oversampling only reduces uncorrelated noise (thermal,
     * quantization). It does NOT reduce correlated noise (power supply
     * ripple, EMI) or improve INL/DNL of the ADC.
     */

    if (target_bits <= adc_bits) return 1;

    int delta_bits = target_bits - adc_bits;

    /* OSR = 4^delta_bits. Use pow() to handle large deltas. */
    return (int)pow(4.0, (double)delta_bits);
}

int conditioning_fir_design(double cutoff_hz, double sample_rate_hz,
                            double stopband_db, double *coeffs, int max_taps)
{
    /* FIR filter design using Kaiser window method.
     *
     * The Kaiser window provides near-optimal trade-off between
     * main lobe width and side lobe level.
     *
     * Kaiser window parameter beta:
     *   beta = 0.1102 * (A - 8.7)        for A > 50 dB
     *   beta = 0.5842 * (A - 21)^0.4 + 0.07886 * (A - 21)  for 21 < A <= 50
     *   beta = 0                           for A <= 21
     *
     * Filter length:
     *   N = (A - 7.95) / (14.36 * dF) + 1
     *   where dF = (stopband - passband) / Fs (normalized transition width)
     *
     * Simplified: N = (A - 8) / (2.285 * dOmega) + 1
     *   where dOmega = 2*pi*(f_stop - f_cutoff)/Fs
     *
     * For our purpose, we estimate the order and design a low-pass
     * FIR with specified cutoff.
     */

    if (coeffs == NULL || max_taps < 3) return -1;

    /* Normalized cutoff frequency */
    double fc_norm = cutoff_hz / sample_rate_hz;
    if (fc_norm <= 0.0 || fc_norm >= 0.5) return -1;

    /* Estimate filter length using Harris approximation */
    double transition_bw = 0.1 * fc_norm;  /* Assume 10% transition */
    double domega = 2.0 * M_PI * transition_bw;

    int n_taps = (int)((stopband_db - 8.0) / (2.285 * domega)) + 1;
    if (n_taps > max_taps) n_taps = max_taps;
    if (n_taps < 3) n_taps = 3;
    if (n_taps % 2 == 0) n_taps++;  /* Prefer odd length for symmetry */

    /* Compute Kaiser window parameter beta */
    double beta;
    if (stopband_db > 50.0) {
        beta = 0.1102 * (stopband_db - 8.7);
    } else if (stopband_db > 21.0) {
        beta = 0.5842 * pow(stopband_db - 21.0, 0.4)
               + 0.07886 * (stopband_db - 21.0);
    } else {
        beta = 0.0;
    }

    /* Design ideal low-pass filter with Kaiser window */
    int half = (n_taps - 1) / 2;
    int i;

    /* Compute Kaiser window */
    double *window = (double *)malloc((size_t)n_taps * sizeof(double));
    if (window == NULL) return -1;

    /* Bessel function I0(x) approximation for Kaiser window */
    double i0_beta = 1.0;  /* I0(beta) */
    {
        double term = 1.0;
        double x = beta / 2.0;
        int k;
        for (k = 1; k <= 20; k++) {
            term *= (x * x) / (double)(k * k);
            i0_beta += term;
        }
    }

    for (i = 0; i < n_taps; i++) {
        double n = (double)(i - half);
        double arg = beta * sqrt(1.0 - (n*n)/((double)(half*half)));
        /* I0(arg) via series */
        double i0_arg = 1.0;
        double term = 1.0;
        double x = arg / 2.0;
        int k;
        for (k = 1; k <= 20; k++) {
            term *= (x * x) / (double)(k * k);
            i0_arg += term;
        }
        window[i] = i0_arg / i0_beta;
    }

    /* Ideal low-pass impulse response:
     * h[n] = sin(2*pi*fc*(n-half)) / (pi*(n-half)) for n != half
     * h[half] = 2*fc
     */
    for (i = 0; i < n_taps; i++) {
        if (i == half) {
            coeffs[i] = 2.0 * fc_norm * window[i];
        } else {
            double n = (double)(i - half);
            coeffs[i] = sin(2.0 * M_PI * fc_norm * n)
                        / (M_PI * n) * window[i];
        }
    }

    /* Normalize for unity DC gain */
    double sum = 0.0;
    for (i = 0; i < n_taps; i++) sum += coeffs[i];
    if (fabs(sum) > 1.0e-12) {
        for (i = 0; i < n_taps; i++) coeffs[i] /= sum;
    }

    free(window);
    return n_taps;
}

void conditioning_fir_apply(const double *input, double *output,
                            int n_samples, const double *coeffs, int n_taps)
{
    /* Apply FIR filter: y[n] = sum(h[k] * x[n-k])
     *
     * This is the direct-form convolution.
     * For long filters, use FFT-based fast convolution (overlap-add).
     */

    if (input == NULL || output == NULL || coeffs == NULL) return;

    int i, k;
    for (i = 0; i < n_samples; i++) {
        double acc = 0.0;
        for (k = 0; k < n_taps; k++) {
            int idx = i - k;
            if (idx >= 0) {
                acc += coeffs[k] * input[idx];
            }
            /* idx < 0: assume input pre-padded with zeros */
        }
        output[i] = acc;
    }
}

void signal_chain_init(signal_chain_t *chain)
{
    /* Initialize signal chain with sensible defaults for
     * precision bridge measurement.
     *
     * Target: 16-bit effective resolution at 10 Hz BW
     */

    memset(chain, 0, sizeof(*chain));

    chain->instrumentation_gain  = 100.0;     /* First stage gain */
    chain->adc_driver_gain       = 2.0;       /* Second stage gain */
    chain->total_gain            = 200.0;     /* 100 * 2 */
    chain->bandwidth_hz          = 10.0;      /* 10 Hz signal BW */
    chain->sample_rate_hz        = 100.0;     /* 10x oversampling */
    chain->adc_resolution_bits   = 24;        /* Sigma-delta ADC */
    chain->adc_reference_v       = 5.0;       /* 5V reference */
    chain->input_impedance_mohm  = 100.0;     /* 100 MOhm typical in-amp */
    chain->cmrr_db               = 120.0;     /* High CMRR in-amp */
    chain->offset_voltage_uv     = 50.0;      /* 50 uV max offset */
    chain->offset_drift_uv_per_c = 0.5;       /* 0.5 uV/K drift */
    chain->noise_rti_nv_per_rt_hz = 8.0;      /* 8 nV/rtHz (low noise) */
    chain->nonlinearity_percent  = 0.001;     /* 0.001% typ in-amp */
}
