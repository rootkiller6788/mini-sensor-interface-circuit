/**
 * @file ina_filter.h
 * @brief Filtering Techniques for Instrumentation Amplifier Systems
 *
 * Covers L5 Algorithms for anti-aliasing, EMI/RFI filtering, and
 * noise bandwidth limiting in IA-based sensor signal chains.
 *
 * Reference:
 *   Zumbahlen, "Linear Circuit Design Handbook" (ADI, 2008)
 *   Kester, "Data Conversion Handbook" (ADI, 2005)
 *   Van Valkenburg, "Analog Filter Design" (1982)
 *
 * @course-alignment
 *   MIT 6.003 Signal Processing: Filter design, frequency response
 *   Berkeley EE123 Digital Signal Processing: Anti-aliasing theory
 *   Stanford EE102A: Continuous-time filters
 *   Georgia Tech ECE 4270: DSP fundamentals - sampling and aliasing
 */
#ifndef INA_FILTER_H
#define INA_FILTER_H

#include "ina_core.h"
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Filter Type Definitions (L1)
 *---------------------------------------------------------------------------*/

typedef enum {
    FILTER_LOWPASS = 0,
    FILTER_HIGHPASS = 1,
    FILTER_BANDPASS = 2,
    FILTER_BANDSTOP = 3,
    FILTER_NOTCH = 4,
    FILTER_ALLPASS = 5
} FilterType;

typedef enum {
    FILTER_APPROX_BUTTERWORTH = 0,   /**< Maximally flat passband */
    FILTER_APPROX_BESSEL = 1,        /**< Maximally flat group delay */
    FILTER_APPROX_CHEBYSHEV = 2,     /**< Equal-ripple passband */
    FILTER_APPROX_ELLIPTIC = 3,      /**< Equal-ripple passband and stopband */
    FILTER_APPROX_SALLEN_KEY = 4,    /**< Sallen-Key topology */
    FILTER_APPROX_MFB = 5            /**< Multiple Feedback topology */
} FilterApproximation;

/**
 * @brief Analog filter specification
 *
 * L1 definitions for filter design parameters.
 */
typedef struct {
    FilterType type;                /**< Filter type */
    FilterApproximation approx;     /**< Approximation method */
    double cutoff_frequency_hz;     /**< -3dB cutoff (lowpass/highpass) or center */
    double cutoff_frequency_2_hz;   /**< Second cutoff for bandpass/bandstop */
    double passband_ripple_db;      /**< Passband ripple for Chebyshev/Elliptic */
    double stopband_atten_db;       /**< Minimum stopband attenuation */
    double stopband_frequency_hz;   /**< Frequency where stopband atten starts */
    double sampling_frequency_hz;   /**< Sampling freq for anti-aliasing design */
    int    order;                   /**< Filter order */
} FilterSpec;

/**
 * @brief Sallen-Key filter component values
 *
 * Sallen-Key is a popular 2nd-order active filter topology using
 * a single op-amp. Available in lowpass and highpass configurations.
 *
 * Transfer function (lowpass, unity gain):
 *   H(s) = 1 / (s^2*R1*R2*C1*C2 + s*(R1+R2)*C2 + 1)
 *
 * Natural frequency: omega_0 = 1/sqrt(R1*R2*C1*C2)
 * Q factor: Q = sqrt(R1*R2*C1*C2) / ((R1+R2)*C2)
 */
typedef struct {
    double r1;           /**< First resistor (ohm) */
    double r2;           /**< Second resistor (ohm) */
    double c1;           /**< First capacitor (F) */
    double c2;           /**< Second capacitor (F) */
    double gain;         /**< Passband gain (V/V) */
    double q_factor;     /**< Quality factor */
    double f0;           /**< Natural frequency (Hz) */
} SallenKeyFilter;

/*---------------------------------------------------------------------------
 * L5: Anti-Aliasing Filter Design Algorithm
 *---------------------------------------------------------------------------*/

/**
 * @brief Design anti-aliasing filter for IA output
 *
 * Per Nyquist sampling theorem (L4), signals above fs/2 must be
 * attenuated below the ADC's noise floor (or at least below 0.5 LSB).
 *
 * Design algorithm (L5):
 *   1. Determine required attenuation at fs/2
 *   2. Select filter order: n >= log10(A_stop)/log10(fs/(2*fc))
 *   3. Choose approximation (Butterworth for flat passband)
 *   4. Cascade 2nd-order stages
 *
 * @param signal_bw_hz Signal bandwidth of interest (Hz)
 * @param sampling_freq_hz ADC sampling frequency (Hz)
 * @param adc_bits ADC resolution (bits)
 * @param spec Output filter specification
 */
void ina_design_antialias_filter(double signal_bw_hz,
                                  double sampling_freq_hz,
                                  int adc_bits,
                                  FilterSpec *spec);

/**
 * @brief Compute required filter order for anti-aliasing
 *
 * n_required = ceil( log10(A_stop) / log10(fs / (2*fsig)) )
 *
 * where:
 *   A_stop = attenuation needed at fs/2 (typically 2^(ADC_bits))
 *   fs = sampling frequency
 *   fsig = signal bandwidth
 */
int ina_antialias_filter_order(double signal_bw_hz,
                                double sampling_freq_hz,
                                double required_attenuation_db);

/**
 * @brief Compute aliased frequency
 *
 * For an input frequency f_in and sampling frequency fs:
 *   f_alias = |f_in - n*fs| where n = round(f_in/fs)
 *
 * This is the fundamental aliasing relationship (L2).
 * All input frequencies map to [0, fs/2] after sampling.
 */
double ina_aliased_frequency(double input_freq_hz, double sampling_freq_hz);

/*---------------------------------------------------------------------------
 * L5: EMI/RFI Filter Design (Input Filtering)
 *---------------------------------------------------------------------------*/

/**
 * @brief RFI/EMI filter specification
 *
 * Instrumentation amplifiers are susceptible to EMI/RFI rectification
 * at their input stage. The differential pair rectifies high-frequency
 * signals, producing a DC offset error.
 *
 * An input RFI filter must:
 *   1. Attenuate frequencies above the IA bandwidth
 *   2. Maintain high CMRR (matched components on both inputs)
 *   3. Not degrade DC accuracy
 *   4. Present low impedance for input bias currents
 */
typedef struct {
    double r_series[2];       /**< Series resistors (one per input) (ohm) */
    double c_diff;            /**< Differential capacitor (F) */
    double c_cm[2];           /**< Common-mode capacitors (F) */
    double differential_cutoff_hz; /**< Diff-mode -3dB frequency */
    double common_mode_cutoff_hz;  /**< CM-mode -3dB frequency */
    double cmrr_degradation_db;    /**< CMRR loss due to mismatch */
} RfiFilter;

/**
 * @brief Design RFI filter for IA input
 *
 * Algorithm (L5):
 *   1. Choose differential cutoff: fc_diff = BW_signal * (10 to 100)
 *   2. Choose CM cutoff: fc_cm = fc_diff / 100 (much higher freq)
 *   3. Select R_series: limited by DC error from Ib
 *   4. Compute C_diff: C = 1/(2*pi*R*2*fc_diff)
 *   5. Compute C_cm: C = 1/(2*pi*R*fc_cm)
 *
 * Common-mode chokes can be added for improved CM filtering
 * without affecting the differential signal.
 *
 * @param signal_bw_hz Signal bandwidth (Hz)
 * @param ia_bandwidth_hz IA -3dB bandwidth (Hz)
 * @param ib_max_na Max input bias current (nA)
 * @param allowed_dc_error_uv Allowed DC error from Ib*R (?V)
 * @param filter Output filter design
 */
void ina_design_rfi_filter(double signal_bw_hz,
                            double ia_bandwidth_hz,
                            double ib_max_na,
                            double allowed_dc_error_uv,
                            RfiFilter *filter);

/**
 * @brief Compute RFI rectification offset estimate
 *
 * The DC offset produced by RFI rectification is approximately:
 *   V_offset = Vrf^2 / (2 * Vt) * (1/(1 + f/fc_diff)^2)
 *
 * where Vrf is the RF amplitude, Vt is the thermal voltage (~26mV),
 * and fc_diff is the differential filter cutoff.
 *
 * Reference: Rich, "Understanding Interference-Type Noise",
 *   Analog Dialogue 16-3 (1982)
 */
double ina_rfi_rectification_offset(double rfi_amplitude_v,
                                     double rfi_frequency_hz,
                                     const RfiFilter *filter);

/*---------------------------------------------------------------------------
 * L5: Noise Bandwidth and Filtering
 *---------------------------------------------------------------------------*/

/**
 * @brief Compute noise bandwidth for a given filter order
 *
 * The noise bandwidth (NBW) is the equivalent rectangular bandwidth
 * that passes the same total noise power as the actual filter.
 *
 * For an nth-order Butterworth lowpass:
 *   NBW = fc * pi/(2*n) / sin(pi/(2*n))
 *
 * For 1st-order: NBW = fc * 1.57
 * For 2nd-order: NBW = fc * 1.11
 * For 3rd-order: NBW = fc * 1.05
 * For 4th-order: NBW = fc * 1.025
 *
 * This is used to convert noise density to total noise.
 */
double ina_noise_bandwidth(double cutoff_frequency_hz,
                            FilterApproximation approx,
                            int order);

/**
 * @brief Compute brick-wall correction factor
 *
 * Converts noise density to total RMS noise:
 *   Vn_rms = en * sqrt(NBW)
 *
 * where NBW = fc * K_n (K_n from ina_noise_bandwidth).
 *
 * This is more accurate than using the -3dB bandwidth directly.
 */
double ina_noise_brickwall_factor(FilterApproximation approx, int order);

/**
 * @brief Determine optimum filter cutoff for SNR
 *
 * Given a signal of known bandwidth and noise spectrum,
 * find the lowpass cutoff that maximizes SNR.
 *
 * Trade-off:
 *   - Higher fc ? more signal power but also more noise
 *   - Lower fc ? less noise but signal may be attenuated
 *
 * For white noise: optimum fc = signal bandwidth
 * For 1/f noise: optimum fc shifts lower
 */
double ina_optimal_filter_cutoff(double signal_bandwidth_hz,
                                  double noise_corner_hz,
                                  double signal_atten_tolerance_db);

/*---------------------------------------------------------------------------
 * L6: Canonical Filter Implementation - Analog Active Filters
 *---------------------------------------------------------------------------*/

/**
 * @brief Design a Sallen-Key lowpass filter
 *
 * Given desired fc and Q, compute R and C values.
 *
 * With equal-R and equal-C design:
 *   R1 = R2 = R, C1 = C2 = C
 *   fc = 1/(2*pi*R*C)
 *   Q = 1/(3 - Gain)  ? Gain = 3 - 1/Q
 *
 * For Butterworth (Q = 1/sqrt(2) ? 0.707):
 *   Gain = 3 - sqrt(2) ? 1.586
 */
SallenKeyFilter ina_design_sallen_key_lowpass(double cutoff_hz,
                                                double q_factor,
                                                double c_value);

/**
 * @brief Design a Sallen-Key highpass filter
 *
 * With equal-R and equal-C design:
 *   fc = 1/(2*pi*R*C)  (same as lowpass)
 *   Q = 1/(3 - Gain)   (same relationship)
 */
SallenKeyFilter ina_design_sallen_key_highpass(double cutoff_hz,
                                                 double q_factor,
                                                 double c_value);

/**
 * @brief Compute Sallen-Key frequency response at given frequency
 *
 * |H(f)| = Gain / sqrt( (1 - (f/fc)^2)^2 + ((f/fc)/Q)^2 )
 *
 * Phase: phi(f) = -atan2( (f/fc)/Q, 1 - (f/fc)^2 )
 */
void ina_sallen_key_response(const SallenKeyFilter *filter,
                              double frequency,
                              double *magnitude,
                              double *phase_rad);

/**
 * @brief Cascade multiple Sallen-Key stages for higher-order filter
 *
 * For nth-order filter, cascade n/2 2nd-order stages.
 * Each stage has its own Q and fc values (from filter tables).
 *
 * This function computes the overall transfer function magnitude
 * at a given frequency.
 */
double ina_cascaded_filter_response(const SallenKeyFilter *stages,
                                     int num_stages,
                                     double frequency);

/*---------------------------------------------------------------------------
 * L6: 50/60 Hz Notch Filter for Power Line Rejection
 *---------------------------------------------------------------------------*/

/**
 * @brief Design Twin-T notch filter
 *
 * The Twin-T notch filter provides high attenuation at a single
 * frequency while passing all others.
 *
 * Notch frequency: f_notch = 1/(2*pi*R*C)
 * At f_notch: |H| = 0 (ideal)
 *
 * Components:
 *   Two T-networks: series R-R with shunt C (first T)
 *                  series C-C with shunt R/2 (second T)
 *
 * Q can be adjusted by adding feedback around the Twin-T.
 */
typedef struct {
    double r_value;          /**< Resistor value (ohm) */
    double c_value;          /**< Capacitor value (F) */
    double notch_frequency;  /**< Notch center frequency (Hz) */
    double q_factor;         /**< Quality factor (higher = sharper notch) */
    double notch_depth_db;   /**< Attenuation at notch frequency (dB) */
} TwinTNotchFilter;

TwinTNotchFilter ina_design_notch_filter(double notch_frequency_hz,
                                           double q_factor,
                                           double c_value);

double ina_notch_filter_response(const TwinTNotchFilter *notch,
                                  double frequency);

/*---------------------------------------------------------------------------
 * L7: Application - Complete IA Signal Chain Filter Design
 *---------------------------------------------------------------------------*/

/**
 * @brief Design complete signal chain filtering for an IA-based system
 *
 * This is a canonical L7 application that combines:
 *   1. Input RFI/EMI filter
 *   2. Anti-aliasing filter before ADC
 *   3. Optional 50/60 Hz notch filter
 *
 * The function coordinates filter designs to avoid interactions
 * and ensure overall system performance.
 */
typedef struct {
    RfiFilter input_rfi;            /**< Input RFI/EMI filter */
    FilterSpec antialias;           /**< Anti-aliasing filter specification */
    TwinTNotchFilter notch;         /**< Optional power line notch */
    double total_noise_bw_hz;       /**< Total noise bandwidth (Hz) */
    double total_latency_us;        /**< Total filter group delay (?s) */
} InaSignalChainFilters;

void ina_design_signal_chain_filters(double signal_bw_hz,
                                      double sampling_freq_hz,
                                      int adc_bits,
                                      double ib_max_na,
                                      int enable_notch,
                                      InaSignalChainFilters *chain);

#endif /* INA_FILTER_H */
