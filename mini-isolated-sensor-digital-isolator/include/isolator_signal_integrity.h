/**
 * @file isolator_signal_integrity.h
 * @brief Signal Integrity Analysis for Isolated Data Links
 *
 * Evaluates signal quality across the isolation barrier, including eye
 * diagrams, jitter decomposition, BER bathtub curves, and EMI coupling
 * analysis. Critical for ensuring reliable data transmission in noisy
 * industrial environments with high common-mode transients.
 *
 * Knowledge Coverage:
 *   L1: eye height, eye width, jitter (RJ, DJ), BER, SNR
 *   L2: inter-symbol interference (ISI), crosstalk, ground bounce
 *   L3: probability density functions for jitter, Q-factor, bathtub curves
 *   L4: dual-Dirac model for jitter decomposition
 *   L5: FFT-based jitter spectrum, tail-fit BER extrapolation
 *
 * References:
 *   - M.P. Li "Jitter, Noise, and Signal Integrity at High-Speed" (2007)
 *   - IEEE 802.3 Annex 68B (Dual-Dirac jitter model)
 *   - Bogatin "Signal and Power Integrity — Simplified" (2018)
 */

#ifndef ISOLATOR_SIGNAL_INTEGRITY_H
#define ISOLATOR_SIGNAL_INTEGRITY_H

#include "digital_isolator.h"
#include <stdint.h>
#include <stddef.h>

/* L1: Signal quality metrics */
typedef struct {
    double eye_height_v;
    double eye_width_ui;
    double eye_amplitude_v;
    double eye_rise_time_ui;
    double eye_fall_time_ui;
    double q_factor;
    double snr_db;
    double extinction_ratio_db;
} eye_diagram_metrics_t;

typedef struct {
    double total_jitter_rms_ps;
    double total_jitter_pp_ps;
    double random_jitter_rms_ps;
    double deterministic_jitter_pp_ps;
    double data_dependent_jitter_pp_ps;
    double periodic_jitter_pp_ps;
    double duty_cycle_distortion_ps;
    double bounded_uncorrelated_jitter_pp_ps;
} jitter_decomposition_t;

typedef struct {
    double bathtub_left_ui;
    double bathtub_right_ui;
    double bathtub_bottom_ber;
    double eye_opening_at_1e12;
    double eye_opening_at_1e15;
    double timing_margin_ui_at_1e12;
} bathtub_curve_t;

/* L2: Crosstalk and EMI */
typedef enum {
    XTALK_NEAR_END,
    XTALK_FAR_END,
    XTALK_COUPLING_THROUGH_BARRIER
} crosstalk_type_t;

typedef struct {
    crosstalk_type_t type;
    double coupling_coefficient;
    double victim_channel_id;
    double aggressor_channel_id;
    double near_end_crosstalk_db;
    double far_end_crosstalk_db;
    double crosstalk_vs_freq_hz[64];
    double crosstalk_vs_freq_db[64];
    size_t num_freq_points;
} crosstalk_model_t;

typedef struct {
    double common_mode_voltage_v;
    double common_mode_freq_hz;
    double differential_mode_voltage_v;
    double cmrr_db;
    double common_mode_to_diff_conversion_db;
    double transient_slew_rate_v_per_us;
} common_mode_coupling_t;

/* L3: Statistical models */
typedef struct {
    double mean_ps;
    double sigma_ps;
    double skewness;
    double kurtosis;
    size_t num_samples;
} gaussian_jitter_params_t;

typedef struct {
    double left_mean;
    double left_sigma;
    double right_mean;
    double right_sigma;
    double dj_separation_ps;
} dual_dirac_params_t;

/* L4: Signal integrity analyzer */
typedef struct {
    double *voltage_samples;
    double *time_samples_ns;
    size_t num_samples;
    double sample_period_ps;
    double bit_period_ps;
    uint64_t total_bits_analyzed;
    eye_diagram_metrics_t eye;
    jitter_decomposition_t jitter;
    bathtub_curve_t bathtub;
    gaussian_jitter_params_t rj_params;
    dual_dirac_params_t dd_params;
} signal_integrity_analyzer_t;

/* API */
int si_analyzer_init(signal_integrity_analyzer_t *sia,
                     double bit_period_ps,
                     double sample_period_ps,
                     size_t max_samples);

int si_analyzer_feed_waveform(signal_integrity_analyzer_t *sia,
                               const double *voltage,
                               const double *time_ns,
                               size_t n_samples);

int si_analyzer_compute_eye(signal_integrity_analyzer_t *sia);

int si_analyzer_decompose_jitter(signal_integrity_analyzer_t *sia);

int si_analyzer_extrapolate_ber(signal_integrity_analyzer_t *sia,
                                 double target_ber);

double si_q_factor_from_eye(const signal_integrity_analyzer_t *sia);

double si_ber_from_q_factor(double q);

double si_jitter_transfer_function(double freq_hz,
                                    double pll_bandwidth_hz,
                                    double damping_factor);

int si_crosstalk_compute(crosstalk_model_t *xtalk,
                          double aggressor_amplitude_v,
                          double aggressor_rise_time_ns);

double si_common_mode_rejection(double cm_voltage_v,
                                 double cm_freq_hz,
                                 double barrier_capacitance_ff);

void si_analyzer_destroy(signal_integrity_analyzer_t *sia);

#endif /* ISOLATOR_SIGNAL_INTEGRITY_H */
