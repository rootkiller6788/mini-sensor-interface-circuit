/**
 * @file cap_noise_immunity.h
 * @brief Noise Immunity: Filtering, Spread Spectrum, Frequency Hopping
 *
 * Capacitive touch sensors must operate in electrically noisy environments:
 * - AC mains (50/60 Hz + harmonics), up to 50 V/m conducted/radiated
 * - Switched-mode power supplies (100 kHz - 2 MHz switching noise)
 * - Display and backlight drivers (kHz PWM, row/column switching)
 * - RF interference (WiFi 2.4/5 GHz, BT, cellular 700 MHz - 2.6 GHz)
 *
 * Noise couples into the sensor through:
 * 1. Conducted: via power supply and ground
 * 2. Capacitive coupling: displacement current through overlay
 * 3. Inductive coupling: magnetic field from nearby inductors
 *
 * Knowledge Coverage:
 *   L1: SNR, CMRR, noise floor, EMI susceptibility, conducted/radiated
 *   L2: spread-spectrum sensing, frequency hopping, differential sensing
 *   L3: FIR/IIR filtering, FFT-based noise analysis, correlation techniques
 *   L4: Parseval theorem, Wiener-Khinchine, matched filter optimality
 *   L5: Median/mean filter, chirp-spread, auto-frequency select, sync rect
 *   L6: coexistence with GSM buzz, mains-immune sensing, display noise cancel
 *
 * Ref: TI SNOA927 "Capacitive Sensing Noise Immunity"
 *      Cypress AN80972 "EMC Design Considerations for CapSense"
 *      Atmel AVR4013 "Noise Tolerant Capacitive Sensing"
 */

#ifndef CAP_NOISE_IMMUNITY_H
#define CAP_NOISE_IMMUNITY_H

#include "cap_sense_core.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * L1: NOISE CHARACTERIZATION
 * ========================================================================== */

/** Noise profile for a sensing channel.
 *
 *  Characterizes noise in both time domain (RMS, peak-peak) and frequency
 *  domain (dominant frequencies, spectral density).
 */
typedef struct {
    cap_noise_source_t dominant_source; /**< Primary noise source */
    double   noise_rms_f;         /**< RMS noise [F] */
    double   noise_peak_peak_f;   /**< Peak-to-peak noise [F] */
    double   noise_psd_f2_hz;     /**< Noise power spectral density [F2/Hz] */
    double   mains_hz;            /**< Detected mains frequency [Hz] (0 if none) */
    double   switching_hz;        /**< Detected SMPS frequency [Hz] */
    double   signal_to_interference_db; /**< SIR [dB] */
    uint32_t outlier_count;       /**< Count of outlier samples */
    double   outlier_threshold_sigma; /**< Outlier threshold in sigmas */
} cap_noise_profile_t;

/** Digital filter configuration for capacitive sensing.
 *
 *  Filter types available:
 *  - MOVING_AVG: Simple N-tap moving average, -3dB at 0.443/N*f_s
 *  - MEDIAN: Non-linear, excellent for impulse noise, N must be odd
 *  - IIR_LOWPASS: First-order IIR, y[n] = alpha*x[n] + (1-alpha)*y[n-1]
 *  - FIR_LOWPASS: N-tap symmetric FIR, linear phase
 *  - DECIMATING: Cascaded CIC or half-band filter for oversampled systems
 */
typedef enum {
    CAP_FILTER_MOVING_AVG,
    CAP_FILTER_MEDIAN,
    CAP_FILTER_IIR_LOWPASS,
    CAP_FILTER_FIR_LOWPASS,
    CAP_FILTER_DECIMATING
} cap_filter_type_t;

typedef struct {
    cap_filter_type_t type;
    uint8_t  filter_order;       /**< Number of taps / order */
    double   alpha;              /**< IIR coefficient (0-1) */
    uint32_t sample_rate_hz;     /**< Input sample rate [Hz] */
    double   cutoff_hz;          /**< -3dB cutoff frequency [Hz] */
    double   *taps;              /**< FIR filter coefficients */
    double   *state;             /**< Filter state buffer (for IIR/FIR) */
    uint32_t state_index;        /**< Circular buffer index */
} cap_digital_filter_t;

/** Spread-spectrum sensing configuration.
 *
 *  Instead of sensing at a single frequency (vulnerable to narrowband
 *  interference), the excitation signal is spread across a bandwidth.
 *  The receiver despreads the signal, spreading any narrowband interference
 *  and reducing its power in the measurement bandwidth by the processing gain:
 *
 *  G_p = 10 * log10(BW_spread / BW_signal) [dB]
 *
 *  For chirp spread with BW=1MHz over T=1ms: G_p ≈ 30 dB.
 */
typedef struct {
    bool    enabled;             /**< Spread-spectrum enabled */
    double  center_freq_hz;      /**< Center frequency [Hz] */
    double  spread_bandwidth_hz; /**< Spread bandwidth [Hz] */
    double  chip_rate_hz;        /**< PN sequence chip rate [Hz] */
    uint16_t pn_sequence_length; /**< PN sequence length (typ 15-31 for LFSR) */
    uint16_t pn_polynomial;      /**< LFSR feedback polynomial */
    double  processing_gain_db;  /**< Theoretical processing gain [dB] */
} cap_spread_spectrum_config_t;

/** Frequency hopping configuration.
 *
 *  Hops the excitation/sampling frequency among N channels to avoid
 *  persistent narrowband interference. Channel dwell time must be short
 *  enough to sample all channels within one scan period.
 *
 *  Hop sequence can be: sequential, pseudo-random (LFSR-based), or
 *  adaptive (avoid channels with detected interference).
 */
typedef struct {
    bool    enabled;
    uint8_t num_channels;        /**< Number of frequency hop channels */
    double  channel_freqs_hz[16]; /**< Hop frequencies [Hz] (max 16) */
    uint8_t current_channel;     /**< Current hop channel index */
    uint32_t dwell_samples;      /**< Samples per hop channel */
    uint32_t sample_counter;     /**< Samples taken on current channel */
    bool    adaptive;            /**< Skip channels with high noise */
    double  noise_threshold_db;  /**< Skip channel if noise > threshold [dB] */
} cap_freq_hopping_config_t;

/** Synchronous detection (lock-in amplifier) configuration.
 *
 *  Drives the excitation at frequency f_exc and synchronously demodulates
 *  the received signal. Rejects out-of-band noise with equivalent noise
 *  bandwidth ENBW = 1/(2*T_int), where T_int is the integration time.
 *
 *  This is the most noise-immune capacitive sensing method.
 */
typedef struct {
    bool    enabled;
    double  excitation_freq_hz;  /**< Excitation frequency [Hz] */
    double  integration_time_s;  /**< Integration time [s] */
    double  enbw_hz;             /**< Equivalent noise bandwidth [Hz] */
    double  phase_offset_deg;    /**< Demodulation phase [deg] (compensates delay) */
    double  i_accumulator;       /**< In-phase accumulator */
    double  q_accumulator;       /**< Quadrature accumulator */
    double  magnitude;           /**< sqrt(I^2 + Q^2) */
    double  phase_deg;           /**< atan2(Q, I) */
} cap_sync_detector_t;

/* ==========================================================================
 * L2-L5: NOISE IMMUNITY API
 * ========================================================================== */

/** Initialize noise profile.
 *
 *  @param profile  Profile to initialize
 */
void cap_noise_profile_init(cap_noise_profile_t *profile);

/** Analyze raw sample buffer to characterize noise.
 *
 *  Computes RMS, peak-peak, and detects dominant frequency components
 *  using zero-crossing analysis (lightweight alternative to FFT).
 *
 *  @param profile    Noise profile to populate
 *  @param samples    Raw sample buffer
 *  @param num_samples Number of samples
 *  @param sample_rate_hz Sample rate [Hz]
 */
void cap_analyze_noise(cap_noise_profile_t *profile,
                       const double *samples, uint32_t num_samples,
                       double sample_rate_hz);

/** Apply median filter to sample buffer (in-place).
 *
 *  For each element, replaces with median of window_size neighbors.
 *  Window must be odd. Excellent for impulse/click noise removal.
 *
 *  Complexity: O(N * W * log W) with sorting, O(N * W) with selection.
 *
 *  @param data        Sample buffer (modified in-place)
 *  @param length      Number of samples
 *  @param window      Window size (odd number)
 */
void cap_median_filter(double *data, uint32_t length, uint8_t window);

/** Apply first-order IIR lowpass filter to a single sample.
 *
 *  y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 *
 *  alpha = 1 - exp(-2*pi*f_c/f_s)  for -3dB cutoff f_c at sample rate f_s.
 *
 *  @param state    Filter state (updated in-place)
 *  @param input    New input sample
 *  @param alpha    Smoothing factor [0-1]
 *  @return Filtered output
 */
double cap_iir_lowpass_step(double *state, double input, double alpha);

/** Compute IIR alpha from cutoff frequency.
 *
 *  alpha = 1 - exp(-2*pi*f_cutoff / f_sample)
 *
 *  @param f_cutoff   Desired -3dB cutoff [Hz]
 *  @param f_sample   Sample rate [Hz]
 *  @return Alpha coefficient [0-1]
 */
double cap_iir_alpha_from_cutoff(double f_cutoff, double f_sample);

/** Apply N-tap moving average filter.
 *
 *  Maintains a circular buffer of the last N samples. Each call returns
 *  the current average and updates the buffer with the new sample.
 *
 *  Complexity: O(1) per sample (using running sum).
 *
 *  @param buffer      Circular buffer of N previous samples
 *  @param buffer_idx  Current buffer position (updated in-place)
 *  @param buffer_len  Buffer length N
 *  @param new_sample  New input sample
 *  @param running_sum Running sum (updated in-place)
 *  @return Moving average
 */
double cap_moving_average_step(double *buffer, uint32_t *buffer_idx,
                               uint32_t buffer_len, double new_sample,
                               double *running_sum);

/** Initialize spread-spectrum configuration.
 *
 *  @param cfg      Spread-spectrum config
 *  @param center   Center frequency [Hz]
 *  @param bw       Spread bandwidth [Hz]
 *  @param chiprate PN chip rate [Hz]
 */
void cap_spread_spectrum_init(cap_spread_spectrum_config_t *cfg,
                              double center, double bw, double chiprate);

/** Compute processing gain of spread-spectrum system.
 *
 *  G_p = 10 * log10(BW_spread / BW_signal)
 *
 *  @param spread_bw   Spread bandwidth [Hz]
 *  @param signal_bw   Signal (post-despread) bandwidth [Hz]
 *  @return Processing gain [dB]
 */
double cap_spread_processing_gain(double spread_bw, double signal_bw);

/** Initialize frequency hopping configuration.
 *
 *  @param cfg          Frequency hopping config
 *  @param num_ch       Number of channels (max 16)
 *  @param base_freq    Base frequency [Hz]
 *  @param spacing      Channel spacing [Hz]
 *  @param dwell        Dwell time in samples
 */
void cap_freq_hopping_init(cap_freq_hopping_config_t *cfg, uint8_t num_ch,
                           double base_freq, double spacing, uint32_t dwell);

/** Advance frequency hopping to next channel.
 *
 *  Returns the frequency of the new active channel.
 *
 *  @param cfg     Frequency hopping config (state updated)
 *  @return Active channel frequency [Hz]
 */
double cap_freq_hopping_next(cap_freq_hopping_config_t *cfg);

/** Initialize synchronous detector.
 *
 *  @param det    Sync detector to initialize
 *  @param f_exc  Excitation frequency [Hz]
 *  @param t_int  Integration time [s]
 */
void cap_sync_detector_init(cap_sync_detector_t *det, double f_exc,
                            double t_int);

/** Process one sample through synchronous detector.
 *
 *  @param det     Detector state
 *  @param sample  New ADC sample
 *  @param t       Time of this sample [s]
 */
void cap_sync_detector_step(cap_sync_detector_t *det, double sample, double t);

/** Compute signal-to-interference ratio.
 *
 *  SIR = 10 * log10(P_signal / P_interference)
 *
 *  @param signal_samples    Signal+interference buffer
 *  @param interference_only Interference-only buffer (baseline)
 *  @param length            Number of samples
 *  @return SIR [dB]
 */
double cap_compute_sir(const double *signal_samples,
                       const double *interference_only,
                       uint32_t length);

/** Select optimal excitation frequency avoiding interference.
 *
 *  Scans candidate frequencies and returns the one with lowest
 *  measured interference.
 *
 *  @param candidates_hz  Array of candidate frequencies [Hz]
 *  @param num_candidates Number of candidates
 *  @param measured_noise Array of noise measurements per candidate
 *  @param sample_rate_hz Sample rate for noise measurement [Hz]
 *  @return Index of best frequency in candidates_hz array
 */
uint8_t cap_select_best_frequency(const double *candidates_hz,
                                  uint8_t num_candidates,
                                  double *measured_noise,
                                  double sample_rate_hz);

/** Destroy digital filter and free resources.
 *
 *  @param filter  Filter to destroy
 */
void cap_digital_filter_destroy(cap_digital_filter_t *filter);

#endif /* CAP_NOISE_IMMUNITY_H */
