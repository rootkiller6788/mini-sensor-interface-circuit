/**
 * @file isolator_channel.h
 * @brief Isolation Channel Model — Signal Propagation Across Barrier
 *
 * Models the complete signal path from input to output across an isolation
 * barrier, including modulation, coupling, demodulation, and signal
 * conditioning stages.
 *
 * Knowledge Coverage:
 *   L1: channel bandwidth, group delay, insertion loss, return loss
 *   L2: signal chain across barrier, edge detection, refresh circuits
 *   L3: Bode plots, Nyquist diagram for isolation channel, pole-zero analysis
 *   L4: Shannon-Hartley applied to isolation channel capacity
 *   L5: adaptive threshold detection, refresh/retransmit algorithms
 *
 * References:
 *   - Pozar "Microwave Engineering" Ch.4 (Network Analysis)
 *   - Johnson & Graham "High-Speed Signal Propagation" (2003)
 *   - TI "Understanding and Interpreting Standard-Logic Data Sheets" (SZZA036)
 */

#ifndef ISOLATOR_CHANNEL_H
#define ISOLATOR_CHANNEL_H

#include "digital_isolator.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ==========================================================================
 * L1: Channel metrics definitions
 * ========================================================================== */

typedef enum {
    CHAN_STATE_IDLE,
    CHAN_STATE_ACTIVE,
    CHAN_STATE_REFRESH,
    CHAN_STATE_ERROR,
    CHAN_STATE_FAILSAFE
} channel_state_t;

typedef struct {
    double insertion_loss_db;
    double return_loss_db;
    double reverse_isolation_db;
    double bandwidth_3db_hz;
    double resonance_freq_hz;
    double q_factor;
} channel_frequency_response_t;

typedef struct {
    double propagation_delay_ns;
    double rise_time_ns;
    double fall_time_ns;
    double overshoot_pct;
    double undershoot_pct;
    double settling_time_ns;
    double ringing_freq_hz;
} channel_transient_response_t;

typedef struct {
    double rms_jitter_ps;
    double peak_peak_jitter_ps;
    double period_jitter_ps;
    double cycle_cycle_jitter_ps;
    double phase_noise_dbc_hz_at_1k;
    double phase_noise_dbc_hz_at_10k;
    double phase_noise_dbc_hz_at_100k;
} channel_jitter_metrics_t;

/* ==========================================================================
 * L2: Channel model structures
 * ========================================================================== */

typedef struct {
    double freq_hz;
    double mag_response;
    double phase_response_rad;
} bode_point_t;

typedef struct {
    bode_point_t *points;
    size_t num_points;
    double dc_gain;
    double dominant_pole_hz;
    double dominant_zero_hz;
    bool is_minimum_phase;
} channel_bode_t;

typedef struct {
    size_t num_poles;
    double poles_hz[8];
    size_t num_zeros;
    double zeros_hz[8];
    double dc_gain;
} pole_zero_model_t;

/* ==========================================================================
 * L3: Signal chain stages
 * ========================================================================== */

typedef enum {
    STAGE_INPUT_BUFFER,
    STAGE_MODULATOR,
    STAGE_BARRIER_COUPLING,
    STAGE_DEMODULATOR,
    STAGE_OUTPUT_BUFFER,
    STAGE_COUNT
} signal_chain_stage_t;

typedef struct {
    signal_chain_stage_t stage;
    double gain_db;
    double noise_figure_db;
    double bandwidth_hz;
    double propagation_delay_ns;
    double power_consumption_mw;
    bool is_active;
} signal_stage_params_t;

/* ==========================================================================
 * L4: Channel capacity model
 * ========================================================================== */

typedef struct {
    double channel_bandwidth_hz;
    double snr_linear;
    double snr_db;
    double channel_capacity_bps;
    double spectral_efficiency_bps_per_hz;
    double actual_data_rate_bps;
    double margin_db;
} channel_capacity_t;

/* ==========================================================================
 * L5: Adaptive threshold and refresh
 * ========================================================================== */

typedef struct {
    double initial_threshold_v;
    double current_threshold_v;
    double adaptation_rate;
    double min_threshold_v;
    double max_threshold_v;
    uint32_t samples_since_adapt;
    uint64_t total_errors;
    bool adaptation_enabled;
} adaptive_threshold_t;

typedef struct {
    double refresh_interval_us;
    uint64_t last_refresh_timestamp;
    bool refresh_in_progress;
    uint32_t consecutive_timeouts;
    uint32_t max_timeouts;
    double dc_balance_target;
    double current_dc_balance;
} refresh_controller_t;

/* ==========================================================================
 * Complete isolation channel
 * ========================================================================== */

typedef struct {
    uint8_t channel_id;
    channel_state_t state;
    digital_isolator_t *isolator;
    channel_frequency_response_t freq_resp;
    channel_transient_response_t trans_resp;
    channel_jitter_metrics_t jitter;
    pole_zero_model_t pole_zero;
    signal_stage_params_t stages[STAGE_COUNT];
    channel_capacity_t capacity;
    adaptive_threshold_t threshold_ctrl;
    refresh_controller_t refresh;
    double input_voltage_v;
    double output_voltage_v;
    uint64_t total_bits_transferred;
    uint32_t bit_errors;
} isolation_channel_t;

/* API */
int isolation_channel_init(isolation_channel_t *ch, uint8_t ch_id,
                           digital_isolator_t *isolator);

void isolation_channel_set_state(isolation_channel_t *ch, channel_state_t st);

int isolation_channel_compute_frequency_response(isolation_channel_t *ch,
                                                  const double *freqs_hz,
                                                  size_t n_freqs);

double isolation_channel_insertion_loss(const isolation_channel_t *ch,
                                        double freq_hz);

double isolation_channel_propagation_delay(const isolation_channel_t *ch,
                                           double input_rise_time_ns);

int isolation_channel_capacity_compute(isolation_channel_t *ch,
                                       double bandwidth_hz,
                                       double snr_db);

double isolation_channel_bit_error_rate(const isolation_channel_t *ch,
                                        double snr_per_bit_db);

int isolation_channel_simulate_pulse(isolation_channel_t *ch,
                                     double pulse_width_ns,
                                     double *out_rise_ns,
                                     double *out_fall_ns,
                                     double *out_delay_ns);

int adaptive_threshold_init(adaptive_threshold_t *at,
                            double initial_v, double rate);

double adaptive_threshold_update(adaptive_threshold_t *at,
                                  double measured_amplitude_v);

int refresh_controller_init(refresh_controller_t *rc,
                            double interval_us, uint32_t max_timeouts);

bool refresh_controller_needs_refresh(const refresh_controller_t *rc);

void refresh_controller_mark_refreshed(refresh_controller_t *rc);

void isolation_channel_destroy(isolation_channel_t *ch);

#endif /* ISOLATOR_CHANNEL_H */
