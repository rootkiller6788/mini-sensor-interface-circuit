/**
 * @file isolation_amplifier.c
 * @brief Isolation Amplifier Implementation
 *
 * Models isolation amplifiers for analog sensor signals. Covers:
 * DC/ac parameter models, three-port isolation, transfer functions,
 * IMRR analysis, auto-zero/chopper stabilization, and noise analysis.
 *
 * Knowledge coverage: L1-L6
 *   L1: CMRR, offset, gain error, nonlinearity, IMRR
 *   L2: Three-port isolation, carrier-based analog modulation
 *   L3: Pole-zero transfer functions, gain/phase vs frequency
 *   L4: IMRR vs frequency and barrier impedance
 *   L5: Chopper/auto-zero for offset drift cancellation
 *   L6: Complete iso-amp modeling (ISO224-class)
 *
 * References:
 *   - Kitchin & Counts "A Designer's Guide to Instrumentation Amplifiers"
 *   - Sedra & Smith "Microelectronic Circuits" Ch.2
 *   - TI ISO224 datasheet (SBAS910)
 */

#include "isolation_amplifier.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* L1: isoamp_init */
int isoamp_init(isolation_amplifier_t *amp, isoamp_architecture_t arch,
                double nominal_gain, double bandwidth_khz)
{
    if (!amp || nominal_gain <= 0.0 || bandwidth_khz <= 0.0) return -1;
    memset(amp, 0, sizeof(*amp));
    amp->architecture = arch;
    snprintf(amp->device_name, sizeof(amp->device_name), "ISOAMP-%s",
             arch == ISOAMP_ARCH_CARRIER_MODULATED ? "CM" :
             arch == ISOAMP_ARCH_SIGMA_DELTA ? "SD" :
             arch == ISOAMP_ARCH_CHOPPER_STABILIZED ? "CH" : "GEN");
    amp->dc.nominal_gain_v_per_v = nominal_gain;
    amp->dc.gain_error_pct = 0.1;
    amp->dc.gain_drift_ppm_per_c = 5.0;
    amp->dc.gain_nonlinearity_pct = 0.01;
    amp->dc.input_offset_uv = 100.0;
    amp->dc.offset_drift_uv_per_c = 2.0;
    amp->dc.input_bias_current_na = 10.0;
    amp->dc.input_impedance_mohm = 10.0;
    amp->dc.input_capacitance_pf = 5.0;
    amp->ac.bandwidth_khz = bandwidth_khz;
    amp->ac.gain_bandwidth_product_mhz = nominal_gain * bandwidth_khz / 1000.0;
    amp->ac.slew_rate_v_per_us = 2.0 * M_PI * bandwidth_khz * 1e3 * 5.0 / 1e6;
    amp->ac.settling_time_us = 0.35 / (bandwidth_khz * 1e3) * 1e6 * 5.0;
    amp->ac.noise_density_nv_per_rt_hz = 12.0;
    amp->ac.noise_0_1_to_10_hz_uv_pp = 2.0;
    amp->ac.thd_db = -100.0;
    amp->ac.thd_n_db = -95.0;
    amp->isolation.cmrr_at_dc_db = 120.0;
    amp->isolation.cmrr_at_60hz_db = 110.0;
    amp->isolation.cmrr_at_1khz_db = 90.0;
    amp->isolation.cmrr_at_10khz_db = 70.0;
    amp->isolation.imrr_at_60hz_db = 140.0;
    amp->isolation.imrr_at_1khz_db = 120.0;
    amp->isolation.psrr_db = 100.0;
    amp->isolation.isolation_voltage_kv = 5.0;
    amp->three_port.isolation_barrier_impedance_ohm = 1e12;
    amp->three_port.barrier_capacitance_pf = 2.0;
    amp->three_port.leakage_current_ua = 0.1;
    amp->temperature_c = 25.0;
    return 0;
}

/* L2: isoamp_set_gain */
int isoamp_set_gain(isolation_amplifier_t *amp, double gain)
{
    if (!amp || gain <= 0.0) return -1;
    amp->dc.nominal_gain_v_per_v = gain;
    amp->ac.gain_bandwidth_product_mhz = gain * amp->ac.bandwidth_khz / 1000.0;
    return 0;
}

/* L3: isoamp_transfer_gain ！ Magnitude response at freq_hz.
 * |H(f)| = DC_gain / sqrt(1 + (f/f_pole)^2) for dominant pole */
double isoamp_transfer_gain(const isolation_amplifier_t *amp, double freq_hz)
{
    if (!amp) return 0.0;
    double dc = amp->dc.nominal_gain_v_per_v;
    double fp = amp->ac.bandwidth_khz * 1000.0;
    if (freq_hz <= 0.0) return dc;
    return dc / sqrt(1.0 + (freq_hz / fp) * (freq_hz / fp));
}

/* L3: isoamp_transfer_phase ！ Phase shift in degrees */
double isoamp_transfer_phase(const isolation_amplifier_t *amp, double freq_hz)
{
    if (!amp) return 0.0;
    double fp = amp->ac.bandwidth_khz * 1000.0;
    if (freq_hz <= 0.0) return 0.0;
    return -atan(freq_hz / fp) * 180.0 / M_PI;
}

/* L4: isoamp_cmrr_at_freq ！ CMRR degrades with frequency.
 * CMRR(f) = CMRR_DC / sqrt(1 + (f/f_cm_pole)^2)
 * where f_cm_pole is typically 100Hz-1kHz due to input capacitance mismatch. */
double isoamp_cmrr_at_freq(const isolation_amplifier_t *amp, double freq_hz)
{
    if (!amp) return 0.0;
    double cmrr_dc = amp->isolation.cmrr_at_dc_db;
    double fp_cm = 100.0; /* Common-mode pole, Hz */
    if (freq_hz <= fp_cm) return cmrr_dc;
    double cmrr_linear = pow(10.0, cmrr_dc / 20.0);
    double degraded = cmrr_linear / sqrt(1.0 + (freq_hz / fp_cm) * (freq_hz / fp_cm));
    return 20.0 * log10(degraded);
}

/* L4: isoamp_imrr_at_freq ！ Isolation Mode Rejection Ratio.
 * IMRR quantifies rejection of voltage across the isolation barrier.
 * It depends on barrier impedance: lower Z_barrier ★ more leakage ★ worse IMRR.
 * IMRR(f) = 20*log10(Z_barrier(f) / Z_input_imbalance) */
double isoamp_imrr_at_freq(const isolation_amplifier_t *amp, double freq_hz __attribute__((unused)),
                            double barrier_z_mohm)
{
    if (!amp || barrier_z_mohm <= 0.0) return 140.0;
    double z_barrier_ohm = barrier_z_mohm * 1e6;
    double z_imbalance_ohm = 10.0; /* typical input impedance mismatch */
    return 20.0 * log10(z_barrier_ohm / z_imbalance_ohm);
}

/* L5: isoamp_total_output_noise ！ Integrated output noise.
 * E_out_total = gain * sqrt(en^2 * BW_noise + (in*R_source)^2 * BW_noise)
 * BW_noise = 1.57 * BW_3dB (brick-wall equivalent for 1st-order) */
double isoamp_total_output_noise(const isolation_amplifier_t *amp,
                                  double f_start_hz, double f_stop_hz)
{
    if (!amp || f_stop_hz <= f_start_hz) return 0.0;
    double bw = f_stop_hz - f_start_hz;
    double en = amp->ac.noise_density_nv_per_rt_hz * 1e-9;
    double gain = amp->dc.nominal_gain_v_per_v;
    double vn_rms = en * sqrt(bw * 1.57) * gain;
    return vn_rms * 1e6; /* uV rms */
}

/* L5: isoamp_effective_resolution ！ Resolution limited by noise.
 * Effective bits = log2(full_scale_range / (6.6 * noise_rms)) */
double isoamp_effective_resolution(const isolation_amplifier_t *amp, double input_range_v)
{
    if (!amp || input_range_v <= 0.0) return 0.0;
    double bw = amp->ac.bandwidth_khz * 1000.0;
    double vn_rms = amp->ac.noise_density_nv_per_rt_hz * 1e-9 * sqrt(bw * 1.57);
    double noise_pp = vn_rms * 6.6;
    double res = log2(input_range_v / noise_pp);
    return res > 0.0 ? res : 0.0;
}

/* L5: isoamp_compute_transfer_function ！ Build pole-zero model */
int isoamp_compute_transfer_function(isolation_amplifier_t *amp,
                                      const double *poles_hz, size_t n_poles,
                                      const double *zeros_hz, size_t n_zeros,
                                      double dc_gain)
{
    if (!amp || n_poles > 6 || n_zeros > 4) return -1;
    amp->transfer.dc_gain = dc_gain;
    amp->transfer.num_poles = n_poles;
    amp->transfer.num_zeros = n_zeros;
    for (size_t i = 0; i < n_poles && i < 6; i++)
        amp->transfer.poles_hz[i] = poles_hz ? poles_hz[i] : 0.0;
    for (size_t i = 0; i < n_zeros && i < 4; i++)
        amp->transfer.zeros_hz[i] = zeros_hz ? zeros_hz[i] : 0.0;
    amp->transfer.gbw_hz = dc_gain * (n_poles > 0 ? amp->transfer.poles_hz[0] : 1e6);
    amp->transfer.phase_margin_deg = 60.0;
    amp->transfer.gain_margin_db = 10.0;
    return 0;
}

/* L5: isoamp_configure_auto_zero ！ Chopper/auto-zero setup.
 * Chopping modulates offset to f_chop, then LP filtered.
 * Residual offset = DC_offset * (1 - duty_cycle_efficiency) */
int isoamp_configure_auto_zero(isolation_amplifier_t *amp,
                                double chopper_freq_khz, bool enabled)
{
    if (!amp || chopper_freq_khz <= 0.0) return -1;
    amp->auto_zero.chopper_freq_khz = chopper_freq_khz;
    amp->auto_zero.auto_zero_enabled = enabled;
    amp->auto_zero.residual_offset_uv = enabled ? 5.0 : amp->dc.input_offset_uv;
    amp->auto_zero.ripple_uv_pp = enabled ? amp->dc.input_offset_uv * 0.01 : 0.0;
    amp->auto_zero.notch_freq_hz = chopper_freq_khz * 1000.0;
    amp->auto_zero.noise_reduction_factor = enabled ? 10.0 : 1.0;
    amp->auto_zero.auto_zero_interval_us = enabled ? 1000000.0 / chopper_freq_khz / 1000.0 : 0;
    return 0;
}

/* L6: isoamp_offset_after_drift ！ Thermal drift model.
 * V_os(T) = V_os(T0) + drift * (T - T0) */
double isoamp_offset_after_drift(const isolation_amplifier_t *amp, double delta_temp_c)
{
    if (!amp) return 0.0;
    return amp->dc.input_offset_uv + amp->dc.offset_drift_uv_per_c * delta_temp_c;
}

/* L6: isoamp_isolation_leakage_current ！ Barrier leakage.
 * I_leak = V_barrier / Z_barrier */
double isoamp_isolation_leakage_current(double barrier_voltage_v,
                                         double barrier_impedance_ohm)
{
    if (barrier_impedance_ohm <= 0.0) return 0.0;
    return barrier_voltage_v / barrier_impedance_ohm * 1e6; /* uA */
}

void isoamp_destroy(isolation_amplifier_t *amp)
{ if (amp) memset(amp, 0, sizeof(*amp)); }
