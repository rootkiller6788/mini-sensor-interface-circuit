/**
 * @file cap_touch_detection.h
 * @brief Touch Detection Algorithms, Baseline Tracking, and Debounce Logic
 *
 * Knowledge Coverage:
 *   L1: touch threshold, hysteresis, debounce, sensitivity, false-touch rate
 *   L2: baseline tracking, EMA, asymmetric filters, noise-adaptive threshold
 *   L3: exponential moving average, variance tracking, derivative estimation
 *   L4: Neyman-Pearson detection criterion, constant false-alarm rate
 *   L5: auto-threshold, dual-threshold, state-machine debounce, drift comp
 *   L6: single-touch robust detection, water-film rejection, glove-pass-through
 *
 * Ref: Microchip AN1478 "mTouch Sensing Solution Acquisition Methods"
 *      Cypress AN2397 "CapSense Sigma-Delta Data Modulation"
 */

#ifndef CAP_TOUCH_DETECTION_H
#define CAP_TOUCH_DETECTION_H

#include "cap_sense_core.h"
#include <stdint.h>
#include <stdbool.h>

/* Touch detection parameters */
typedef struct {
    double  touch_threshold_f;
    double  release_threshold_f;
    double  proximity_threshold_f;
    double  noise_sigma_f;
    double  threshold_sigma_mult;
    uint8_t debounce_samples;
    uint8_t release_debounce;
    uint8_t fast_debounce_samples;
    bool    adaptive_threshold;
    double  max_threshold_f;
    double  min_threshold_f;
} cap_touch_detect_config_t;

/* Baseline tracking with asymmetric EMA */
typedef struct {
    double  ema_alpha_slow;
    double  ema_alpha_fast;
    double  ema_alpha_init;
    double  max_drift_per_scan_f;
    uint32_t init_settle_samples;
    bool    baseline_frozen;
    uint32_t freeze_delay_samples;
} cap_baseline_tracker_config_t;

/* Touch event record */
typedef struct {
    uint8_t   channel_id;
    touch_state_t event_type;
    uint32_t  timestamp_ms;
    double    peak_delta_c_f;
    double    snr_at_detect_db;
    double    duration_ms;
    double    touch_pressure_est;
} cap_touch_event_t;

/* Touch detection statistics for production test (Cpk) */
typedef struct {
    uint32_t total_scans;
    uint32_t touch_events;
    uint32_t false_positives;
    uint32_t false_negatives;
    double   mean_delta_c_touch_f;
    double   mean_delta_c_idle_f;
    double   mean_noise_f;
    double   max_noise_f;
    double   min_snr_db;
    double   thd_settle_time_ms;
} cap_touch_stats_t;

/* L2-L5 API */
void cap_touch_detect_config_init(cap_touch_detect_config_t *cfg);

void cap_baseline_tracker_config_init(cap_baseline_tracker_config_t *cfg);

uint32_t cap_baseline_update_ema(uint32_t baseline, uint32_t raw,
                                 const cap_baseline_tracker_config_t *cfg,
                                 bool touch_active);

int32_t cap_compute_delta(uint32_t raw, uint32_t baseline, cap_sense_mode_t mode);

double cap_delta_count_to_farad(int32_t delta_count, double c_ref_f,
                                uint8_t resolution_bits, double gain);

double cap_adaptive_threshold(double noise_rms_f, double sigma_mult,
                              double min_thresh_f, double max_thresh_f);

void cap_update_noise_estimate(double *current_noise_var, double *current_mean,
                               double new_sample, double alpha_var,
                               double alpha_mean);

touch_state_t cap_touch_state_machine(cap_sensor_channel_t *chan,
                                      double delta_c_f,
                                      const cap_touch_detect_config_t *config,
                                      cap_touch_event_t *event,
                                      uint32_t current_time_ms);

double cap_estimate_touch_pressure(double delta_c_f, double c_max_f,
                                   double pressure_ref_f);

bool cap_detect_water_film(double delta_self, double delta_mutual,
                           double water_thresh_self, double water_thresh_mut);

double cap_glove_touch_confidence(double delta_c_f, double expected_bare_f,
                                  double rise_time_ms, double bare_rise_ms);

void cap_touch_stats_reset(cap_touch_stats_t *stats);

void cap_touch_stats_accumulate(cap_touch_stats_t *stats, double delta_c_f,
                                bool is_touch, bool is_valid);

#endif /* CAP_TOUCH_DETECTION_H */
