/**
 * @file isolation_amplifier.h
 * @brief Isolation Amplifier Design — Analog Signal Isolation
 *
 * Models isolation amplifiers that provide galvanic isolation for analog
 * signals. Used in sensor interfaces where analog conditioning is required
 * before digitization, particularly for high-CMRR applications such as
 * current sensing, medical ECG/EEG, and industrial 4-20mA loops.
 *
 * Knowledge Coverage:
 *   L1: CMRR, input offset voltage, gain error, nonlinearity, IMRR
 *   L2: three-port isolation (input/output/power), carrier-based modulation
 *   L3: transfer function including isolation barrier poles
 *   L4: isolation-mode rejection ratio (IMRR) vs frequency
 *   L5: auto-zero/chopper stabilization for offset drift
 *   L6: isolated current sensing (shunt + iso-amp), isolated voltage sensing
 *
 * References:
 *   - Kitchin & Counts "A Designer's Guide to Instrumentation Amplifiers" (ADI)
 *   - TI "Precision Isolated Amplifier" (ISO224 datasheet, SBAS910)
 *   - Sedra & Smith "Microelectronic Circuits" Ch.2 (Op-Amps)
 */

#ifndef ISOLATION_AMPLIFIER_H
#define ISOLATION_AMPLIFIER_H

#include "digital_isolator.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* L1: Core iso-amp parameters */
typedef enum {
    ISOAMP_ARCH_CARRIER_MODULATED,
    ISOAMP_ARCH_SIGMA_DELTA,
    ISOAMP_ARCH_CHOPPER_STABILIZED,
    ISOAMP_ARCH_TRANSFORMER_COUPLED,
    ISOAMP_ARCH_CAPACITIVE_COUPLED
} isoamp_architecture_t;

typedef struct {
    double nominal_gain_v_per_v;
    double gain_error_pct;
    double gain_drift_ppm_per_c;
    double gain_nonlinearity_pct;
    double input_offset_uv;
    double offset_drift_uv_per_c;
    double input_bias_current_na;
    double input_impedance_mohm;
    double input_capacitance_pf;
} isoamp_dc_params_t;

typedef struct {
    double bandwidth_khz;
    double gain_bandwidth_product_mhz;
    double slew_rate_v_per_us;
    double settling_time_us;
    double noise_density_nv_per_rt_hz;
    double noise_0_1_to_10_hz_uv_pp;
    double thd_db;
    double thd_n_db;
} isoamp_ac_params_t;

typedef struct {
    double cmrr_at_dc_db;
    double cmrr_at_60hz_db;
    double cmrr_at_1khz_db;
    double cmrr_at_10khz_db;
    double imrr_at_60hz_db;
    double imrr_at_1khz_db;
    double psrr_db;
    double isolation_voltage_kv;
} isoamp_isolation_params_t;

/* L2: Three-port isolation model */
typedef struct {
    double input_common_mode_v;
    double output_common_mode_v;
    double isolation_barrier_impedance_ohm;
    double barrier_capacitance_pf;
    double leakage_current_ua;
} isoamp_three_port_t;

/* L3: Transfer function */
typedef struct {
    size_t num_poles;
    double poles_hz[6];
    size_t num_zeros;
    double zeros_hz[4];
    double dc_gain;
    double gbw_hz;
    double phase_margin_deg;
    double gain_margin_db;
} isoamp_transfer_function_t;

/* L4: IMRR model */
typedef struct {
    double freq_hz;
    double imrr_db;
    double barrier_impedance_mohm;
    double input_imbalance_pct;
} imrr_model_t;

/* L5: Auto-zero / chopper */
typedef struct {
    double chopper_freq_khz;
    double residual_offset_uv;
    double ripple_uv_pp;
    double notch_freq_hz;
    double noise_reduction_factor;
    bool auto_zero_enabled;
    uint32_t auto_zero_interval_us;
} auto_zero_config_t;

/* L6: Complete isolation amplifier */
typedef struct {
    char device_name[32];
    isoamp_architecture_t architecture;
    isoamp_dc_params_t dc;
    isoamp_ac_params_t ac;
    isoamp_isolation_params_t isolation;
    isoamp_three_port_t three_port;
    isoamp_transfer_function_t transfer;
    auto_zero_config_t auto_zero;
    digital_isolator_t *back_channel;
    double input_voltage_v;
    double output_voltage_v;
    double supply_voltage_side1_v;
    double supply_voltage_side2_v;
    double temperature_c;
} isolation_amplifier_t;

/* API */
int isoamp_init(isolation_amplifier_t *amp,
                isoamp_architecture_t arch,
                double nominal_gain,
                double bandwidth_khz);

int isoamp_set_gain(isolation_amplifier_t *amp, double gain);

double isoamp_transfer_gain(const isolation_amplifier_t *amp, double freq_hz);

double isoamp_transfer_phase(const isolation_amplifier_t *amp, double freq_hz);

double isoamp_cmrr_at_freq(const isolation_amplifier_t *amp, double freq_hz);

double isoamp_imrr_at_freq(const isolation_amplifier_t *amp,
                            double freq_hz, double barrier_z_mohm);

double isoamp_total_output_noise(const isolation_amplifier_t *amp,
                                  double freq_start_hz, double freq_stop_hz);

double isoamp_effective_resolution(const isolation_amplifier_t *amp,
                                    double input_range_v);

int isoamp_compute_transfer_function(isolation_amplifier_t *amp,
                                      const double *poles_hz, size_t n_poles,
                                      const double *zeros_hz, size_t n_zeros,
                                      double dc_gain);

int isoamp_configure_auto_zero(isolation_amplifier_t *amp,
                                double chopper_freq_khz,
                                bool enabled);

double isoamp_offset_after_drift(const isolation_amplifier_t *amp,
                                  double delta_temp_c);

double isoamp_isolation_leakage_current(double barrier_voltage_v,
                                         double barrier_impedance_ohm);

void isoamp_destroy(isolation_amplifier_t *amp);

#endif /* ISOLATION_AMPLIFIER_H */
