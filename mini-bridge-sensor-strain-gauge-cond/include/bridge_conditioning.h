/**
 * bridge_conditioning.h — Bridge Signal Conditioning Chain
 *
 * Knowledge Coverage:
 *   L2: Signal conditioning chain, anti-alias filtering,
 *       ADC interfacing, gain staging
 *   L3: Noise analysis, SNR optimization, filter design
 *   L4: Nyquist sampling theorem, quantization noise
 *   L5: Digital filter design, moving average, decimation
 *
 * Reference:
 *   - Kester, "Sensor Signal Conditioning", Analog Devices, 1999
 *   - Sheingold, "Transducer Interfacing Handbook", Analog Devices
 *   - Oppenheim & Schafer, "Discrete-Time Signal Processing", 3rd ed.
 *
 * Course: MIT 6.003, Stanford EE102A, Berkeley EE123
 */

#ifndef BRIDGE_CONDITIONING_H
#define BRIDGE_CONDITIONING_H

#include "bridge_core.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * L2: Signal Conditioning Chain Architecture
 * ======================================================================== */

/**
 * Signal conditioning chain descriptor.
 *
 * Typical bridge-to-digital signal chain:
 *
 *   Bridge → In-Amp → Anti-Alias Filter → ADC Driver → ADC → Digital Filter
 *            (Gain)   (Low-pass)          (Buffer)             (Decimation)
 *
 * Each stage adds noise, offset, and drift. The total error budget
 * must be allocated across stages.
 */
typedef struct {
    double instrumentation_gain;       /* In-amp gain [V/V] */
    double adc_driver_gain;           /* ADC driver gain [V/V] */
    double total_gain;                /* Cascaded gain [V/V] */
    double bandwidth_hz;              /* Signal bandwidth of interest [Hz] */
    double sample_rate_hz;            /* ADC sample rate [SPS] */
    int    adc_resolution_bits;       /* ADC resolution [bits] */
    double adc_reference_v;           /* ADC reference voltage [V] */
    double input_impedance_mohm;      /* Amplifier input Z [MOhm] */
    double cmrr_db;                   /* Amplifier CMRR [dB] */
    double offset_voltage_uv;         /* Input offset voltage [uV] */
    double offset_drift_uv_per_c;     /* Offset drift [uV/K] */
    double noise_rti_nv_per_rt_hz;    /* Input noise density [nV/rtHz] */
    double nonlinearity_percent;      /* Amplifier nonlinearity [%] */
} signal_chain_t;

/**
 * Compute required gain for the signal chain.
 *
 * The ADC input span should match the amplified bridge output:
 *
 *   Gain = Vref_ADC / (Vexc * S * epsilon_FS)
 *
 * where S = sensitivity [V/V per unit strain]
 *
 * Example: Vref=5V, Vexc=5V, GF=2, eps_FS=1000 ue, quarter bridge
 *   Vout_FS = 5 * 2/4 * 0.001 = 2.5 mV
 *   Gain = 5 / 0.0025 = 2000 V/V
 *
 * This is a very high gain! In practice, split across two stages.
 *
 * @param vref_adc       ADC reference voltage [V]
 * @param vex            Bridge excitation [V]
 * @param gf             Gauge factor
 * @param eps_fs_ue      Full-scale strain [microstrain]
 * @param config         Bridge configuration
 * @return               Required total gain [V/V]
 *
 * Complexity: O(1).
 */
double conditioning_required_gain(double vref_adc, double vex, double gf,
                                  double eps_fs_ue, bridge_config_t config);

/**
 * Compute total input-referred noise of the signal chain.
 *
 * Noise sources (uncorrelated, RSS addition):
 * 1. Bridge Johnson noise: Vn_bridge = sqrt(4*kB*T*R*BW)
 * 2. Amplifier voltage noise: Vn_amp = en * sqrt(BW)
 * 3. Amplifier current noise × bridge impedance: Vn_i = in * R * sqrt(BW)
 * 4. ADC quantization noise: Vn_ADC = Vref/(2^N * sqrt(12))
 *
 * Total RTI: Vn_total_rti = sqrt(Vn1^2 + Vn2^2 + Vn3^2 + Vn4^2/G^2)
 *
 * @param chain        Signal chain parameters
 * @param bridge_r_ohm Bridge output resistance [Ohm]
 * @param temp_k       Absolute temperature [K]
 * @param in_noise_pa  Amplifier current noise density [pA/rtHz]
 * @return             Total input-referred noise [Vrms]
 *
 * Complexity: O(1).
 */
double conditioning_total_noise(const signal_chain_t *chain,
                                double bridge_r_ohm, double temp_k,
                                double in_noise_pa);

/**
 * Compute effective number of bits (ENOB) for the measurement system.
 *
 * ENOB = (SNR_dB - 1.76) / 6.02
 *
 * where SNR_dB = 20*log10(V_signal_FS / V_noise_rms)
 *
 * ENOB < ADC resolution due to noise and distortion.
 * For precision bridge measurement: target ENOB >= 16 bits.
 *
 * @param v_signal_fs    Full-scale signal voltage [V]
 * @param v_noise_rms    Total RMS noise [Vrms]
 * @return               Effective number of bits
 *
 * Complexity: O(1).
 * Reference: IEEE Std 1241-2010, "ADC Terminology and Test Methods"
 */
double conditioning_enob(double v_signal_fs, double v_noise_rms);

/**
 * Compute signal-to-noise ratio for bridge measurement.
 *
 * SNR_dB = 20*log10(V_signal / V_noise_rms)
 *
 * For bridge sensors:
 *   V_signal = Vexc * GF/4 * epsilon  [quarter bridge]
 *
 * Noise includes Johnson, amplifier, and ADC contributions.
 *
 * @param chain        Signal chain
 * @param bridge       Bridge state
 * @param strain_ue    Applied strain [microstrain]
 * @return             SNR [dB]
 *
 * Complexity: O(1).
 */
double conditioning_snr(const signal_chain_t *chain,
                        const bridge_state_t *bridge,
                        double strain_ue);

/* ========================================================================
 * L3: Anti-Alias Filter Design
 * ======================================================================== */

/**
 * Anti-aliasing filter type
 */
typedef enum {
    FILTER_BUTTERWORTH,  /* Maximally flat passband, moderate roll-off */
    FILTER_BESSEL,       /* Linear phase, gentle roll-off */
    FILTER_CHEBYSHEV,    /* Steep roll-off, passband ripple */
    FILTER_ELLIPTIC      /* Steepest roll-off, ripple in both bands */
} filter_type_t;

/**
 * Anti-aliasing filter specification.
 *
 * Placed between amplifier and ADC to:
 * 1. Prevent aliasing of out-of-band noise into signal BW
 * 2. Limit noise bandwidth for RMS noise calculation
 * 3. Reject 50/60 Hz power line interference
 */
typedef struct {
    filter_type_t type;
    int    order;              /* Filter order (1-8) */
    double cutoff_hz;          /* -3 dB cutoff frequency [Hz] */
    double stopband_hz;        /* Stopband start frequency [Hz] */
    double stopband_atten_db;  /* Required stopband attenuation [dB] */
    double passband_ripple_db; /* Passband ripple (Chebyshev/Elliptic) [dB] */
    double sample_rate_hz;     /* ADC sample rate [Hz] */
} anti_alias_filter_t;

/**
 * Design anti-alias filter for given sample rate.
 *
 * Rule: cutoff < Fs/2 to satisfy Nyquist, typically cutoff = Fs/5 to Fs/10
 * for reasonable filter order.
 *
 * @param sample_rate_hz  ADC sampling rate [Hz]
 * @param signal_bw_hz    Signal bandwidth [Hz]
 * @param filter_out      Output filter specification
 *
 * Complexity: O(1).
 */
void conditioning_design_antialias(double sample_rate_hz,
                                   double signal_bw_hz,
                                   anti_alias_filter_t *filter_out);

/**
 * Required filter order for given attenuation spec.
 *
 * Butterworth filter:
 *   N >= log10((10^(A_s/10)-1)/(10^(A_p/10)-1)) / (2*log10(f_s/f_c))
 *
 * where A_s = stopband attenuation [dB], A_p = passband ripple [dB],
 * f_s = stopband frequency, f_c = cutoff frequency.
 *
 * @param f_pass_hz       Passband edge [Hz]
 * @param f_stop_hz       Stopband edge [Hz]
 * @param a_stop_db       Stopband attenuation [dB]
 * @param a_pass_db       Passband ripple [dB]
 * @return                Minimum required filter order
 *
 * Complexity: O(1).
 */
int conditioning_filter_order(double f_pass_hz, double f_stop_hz,
                              double a_stop_db, double a_pass_db);

/* ========================================================================
 * L5: Digital Signal Processing for Bridge Sensors
 * ======================================================================== */

/**
 * Moving average filter state for bridge signal smoothing.
 *
 * The moving average filter is the optimal filter for reducing
 * random white noise while preserving a step response.
 *
 * y[n] = (1/N) * sum_{k=0}^{N-1} x[n-k]
 *
 * Noise reduction: SNR improvement = sqrt(N)
 * Settling time: N samples to reach final value after step
 */
typedef struct {
    double *buffer;        /* Circular buffer of samples */
    int     window_size;   /* Number of samples in average */
    int     index;         /* Current write position */
    double  sum;           /* Running sum for O(1) update */
    int     count;         /* Samples accumulated so far */
} moving_average_filter_t;

/**
 * Initialize moving average filter.
 *
 * @param maf          Filter state to initialize
 * @param window_size  Number of samples in averaging window
 * @return             0 on success, -1 on allocation failure
 *
 * Complexity: O(1).
 */
int conditioning_maf_init(moving_average_filter_t *maf, int window_size);

/**
 * Process one sample through moving average filter.
 *
 * Uses O(1) incremental update:
 *   sum = sum - oldest_sample + new_sample
 *   average = sum / N
 *
 * @param maf     Filter state
 * @param sample  New ADC sample [LSB or voltage]
 * @return        Filtered output value
 *
 * Complexity: O(1) amortized.
 */
double conditioning_maf_process(moving_average_filter_t *maf, double sample);

/**
 * Reset moving average filter to initial state.
 *
 * @param maf  Filter state to reset
 *
 * Complexity: O(1).
 */
void conditioning_maf_reset(moving_average_filter_t *maf);

/**
 * Free resources associated with moving average filter.
 *
 * @param maf  Filter state to free
 *
 * Complexity: O(1).
 */
void conditioning_maf_free(moving_average_filter_t *maf);

/**
 * Median filter for rejecting impulse noise (spikes).
 *
 * The median filter replaces each sample with the median of
 * its local neighborhood. Excellent for removing salt-and-pepper
 * noise and single-sample glitches without blurring edges.
 *
 * Unlike the moving average, the median filter preserves edges
 * while removing outliers.
 *
 * @param data         Input/output array of samples
 * @param n_samples    Number of samples
 * @param window_size  Median window size (odd, 3-9 typical)
 *
 * Complexity: O(N * W * log(W)) where W = window_size.
 */
void conditioning_median_filter(double *data, int n_samples, int window_size);

/**
 * Decimation filter — reduce sample rate by integer factor.
 *
 * The decimation process:
 * 1. Low-pass filter to prevent aliasing at new rate
 * 2. Downsample by keeping every M-th sample
 *
 * This is used after oversampling to increase ENOB:
 *   ENOB_gain = 0.5 * log2(OSR) [bits per doubling of OSR]
 *
 * @param input         Input sample array
 * @param n_input       Number of input samples
 * @param decim_factor  Decimation factor M (keep every M-th)
 * @param output        Output decimated array (size >= n_input/M)
 * @return              Number of output samples
 *
 * Complexity: O(N/M).
 */
int conditioning_decimate(const double *input, int n_input,
                          int decim_factor, double *output);

/**
 * Calculate oversampling ratio for target ENOB improvement.
 *
 * Each 4x oversampling improves SNR by 1 bit (6.02 dB):
 *   OSR_needed = 4^(target_bits - adc_bits)
 *
 * Example: 12-bit ADC, target 16 bits → delta = 4 bits
 *   OSR = 4^4 = 256 → sample 256x faster than Nyquist
 *
 * This makes 16-bit measurement from a 12-bit ADC feasible
 * for slow-changing bridge signals (typical BW < 10 Hz).
 *
 * @param adc_bits      ADC native resolution [bits]
 * @param target_bits   Desired effective resolution [bits]
 * @return              Required oversampling ratio
 *
 * Complexity: O(1).
 */
int conditioning_oversampling_ratio(int adc_bits, int target_bits);

/**
 * FIR low-pass filter for bridge signal conditioning.
 *
 * Design using window method (Kaiser window) for arbitrary
 * stopband attenuation with minimum filter length.
 *
 * The Kaiser window provides the optimal trade-off between
 * main lobe width and side lobe level among all windows.
 *
 * @param cutoff_hz       -3 dB cutoff [Hz]
 * @param sample_rate_hz  Sampling rate [Hz]
 * @param stopband_db     Stopband attenuation [dB]
 * @param coeffs          Output coefficient array (caller allocates)
 * @param max_taps        Maximum number of taps
 * @return                Actual number of taps used, -1 on error
 *
 * Complexity: O(N) where N = number of taps.
 */
int conditioning_fir_design(double cutoff_hz, double sample_rate_hz,
                            double stopband_db, double *coeffs, int max_taps);

/**
 * Apply FIR filter to a signal.
 *
 * y[n] = sum_{k=0}^{N-1} h[k] * x[n-k]
 *
 * @param input    Input signal array
 * @param output   Output filtered array
 * @param n_samples Number of samples
 * @param coeffs   FIR coefficients
 * @param n_taps   Number of filter taps
 *
 * Complexity: O(N_samples * N_taps).
 */
void conditioning_fir_apply(const double *input, double *output,
                            int n_samples, const double *coeffs, int n_taps);

/**
 * Initialize signal chain with sensible defaults for bridge measurement.
 */
void signal_chain_init(signal_chain_t *chain);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_CONDITIONING_H */
