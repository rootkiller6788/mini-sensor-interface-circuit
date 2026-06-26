/**
 * @file isolated_adc_interface.c
 * @brief Isolated ADC Interface Implementation
 *
 * Implements SPI/I2C interface isolation, clock jitter analysis,
 * aperture uncertainty modeling, and SNR/ENOB degradation from
 * isolator-induced jitter.
 *
 * Knowledge coverage:
 *   L1: ENOB, SINAD, SFDR, aperture jitter, clock phase noise
 *   L2: Isolated SPI (3-wire/4-wire), isolated I2C with clock stretch
 *   L3: Jitter-to-SNR: SNR_jitter = -20*log10(2*pi*f_in*t_jitter_rms)
 *   L4: Nyquist sampling with isolated clock, aperture uncertainty
 *   L5: Clock-forward vs clock-recovery architectures
 *   L6: Complete 24-bit isolated ADC interface modeling
 *
 * References:
 *   - Kester "Data Conversion Handbook" (ADI, 2005)
 *   - IEEE 1241-2010 ADC Test Standard
 *   - TI "Clocking for High-Speed ADCs" (SNAA127)
 */

#include "isolated_adc_interface.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* L1: isolated_adc_init */
int isolated_adc_init(isolated_adc_interface_t *iface,
                       adc_interface_type_t if_type,
                       digital_isolator_t *isolator,
                       uint8_t resolution_bits,
                       double sample_rate_sps)
{
    if (!iface || !isolator || resolution_bits == 0 || sample_rate_sps <= 0.0)
        return -1;
    memset(iface, 0, sizeof(*iface));
    iface->if_type = if_type;
    iface->isolator = isolator;
    iface->adc.resolution_bits = resolution_bits;
    iface->adc.sample_rate_sps = sample_rate_sps;
    iface->adc.enob_bits = (double)resolution_bits;
    iface->adc.sinad_db = 6.02 * resolution_bits + 1.76;
    iface->adc.snr_db = iface->adc.sinad_db;
    iface->adc.sfdr_db = 6.02 * (resolution_bits + 2);
    iface->adc.thd_db = -80.0;
    iface->adc.dnl_lsb = 0.5;
    iface->adc.inl_lsb = 1.0;
    iface->adc.offset_error_mv = 1.0;
    iface->adc.gain_error_pct = 0.1;
    iface->adc.input_bandwidth_hz = sample_rate_sps / 2.0 * 0.8;
    iface->adc.aperture_jitter_ps_rms = 0.5;
    iface->clock.isolated_clock_jitter_ps_rms = 5.0;
    iface->clock.source_clock_jitter_ps_rms = 1.0;
    iface->clock.isolator_additive_jitter_ps_rms = 4.9;
    iface->clock.clock_phase_noise_dbc_at_1k = -120.0;
    iface->clock.clock_phase_noise_dbc_at_10k = -140.0;
    iface->clock.clock_phase_noise_dbc_at_100k = -155.0;
    iface->clock.pll_bandwidth_hz = 100e3;
    return 0;
}

/* L2: SPI configuration for isolated ADC interface */
int isolated_adc_spi_config(isolated_adc_interface_t *iface,
                             uint8_t sclk_ch, uint8_t mosi_ch,
                             uint8_t miso_ch, uint8_t cs_ch,
                             double sclk_freq_hz, bool cpol, bool cpha)
{
    if (!iface || sclk_freq_hz <= 0.0) return -1;
    iface->config.spi.sclk_channel = sclk_ch;
    iface->config.spi.mosi_channel = mosi_ch;
    iface->config.spi.miso_channel = miso_ch;
    iface->config.spi.cs_channel = cs_ch;
    iface->config.spi.sclk_freq_hz = sclk_freq_hz;
    iface->config.spi.data_rate_sps = sclk_freq_hz / 16.0;
    iface->config.spi.cpol = cpol;
    iface->config.spi.cpha = cpha;
    iface->config.spi.bits_per_transfer = 24;
    iface->config.spi.double_data_rate = false;
    return 0;
}

/* L2: I2C configuration */
int isolated_adc_i2c_config(isolated_adc_interface_t *iface,
                             uint8_t scl_ch, uint8_t sda_ch,
                             uint8_t slave_addr, double scl_freq_hz)
{
    if (!iface || scl_freq_hz <= 0.0) return -1;
    iface->config.i2c.scl_channel = scl_ch;
    iface->config.i2c.sda_channel = sda_ch;
    iface->config.i2c.slave_addr_7bit = slave_addr;
    iface->config.i2c.scl_freq_hz = scl_freq_hz;
    iface->config.i2c.clock_stretching_enabled = true;
    iface->config.i2c.clock_stretch_timeout_us = 1000;
    return 0;
}

/* L3: jitter_limited_snr ¡ª SNR_dB = -20*log10(2*pi*f_signal*t_jitter_rms)
 * For f_signal=10kHz, t_jitter=5ps: SNR = -20*log10(2*pi*1e4*5e-12) = 70.0 dB
 * For f_signal=100kHz, t_jitter=5ps: SNR = 50.0 dB */
double jitter_limited_snr(double signal_freq_hz, double jitter_rms_ps)
{
    if (signal_freq_hz <= 0.0 || jitter_rms_ps <= 0.0) return 200.0;
    double arg = 2.0 * M_PI * signal_freq_hz * jitter_rms_ps * 1e-12;
    if (arg >= 1.0) return 0.0;
    return -20.0 * log10(arg);
}

/* L3: jitter_limited_enob ¡ª ENOB = (SNR_jitter - 1.76) / 6.02 */
double jitter_limited_enob(double signal_freq_hz, double jitter_rms_ps)
{
    double snr = jitter_limited_snr(signal_freq_hz, jitter_rms_ps);
    if (snr <= 1.76) return 0.0;
    return (snr - 1.76) / 6.02;
}

/* L4: aperture_jitter_from_phase_noise
 * Integrated phase noise from f_offset: L(f) in dBc/Hz
 * t_jitter_rms = sqrt(2 * 10^(L(f)/10) * f_offset) / (2*pi*f_carrier) */
double aperture_jitter_from_phase_noise(double freq_offset_hz,
                                         double phase_noise_dbc,
                                         double carrier_freq_hz)
{
    if (carrier_freq_hz <= 0.0) return 0.0;
    double l_linear = pow(10.0, phase_noise_dbc / 10.0);
    double phase_noise_power = 2.0 * l_linear * freq_offset_hz;
    double phase_rad_rms = sqrt(phase_noise_power);
    return phase_rad_rms / (2.0 * M_PI * carrier_freq_hz) * 1e12;
}

/* L5: clock degradation analysis */
int isolated_adc_compute_clock_degradation(isolated_adc_interface_t *iface,
                                            jitter_snr_impact_t *impact)
{
    if (!iface || !impact) return -1;
    double f_in = iface->adc.input_bandwidth_hz * 0.5;
    double jitter = iface->clock.isolated_clock_jitter_ps_rms;
    impact->signal_freq_hz = f_in;
    impact->jitter_rms_ps = jitter;
    impact->snr_degradation_db = jitter_limited_snr(f_in, jitter);
    impact->max_enob = jitter_limited_enob(f_in, jitter);
    return 0;
}

/* L5: ENOB degradation estimator */
int isolated_adc_estimate_enob_degradation(const isolated_adc_interface_t *iface,
                                            double *enob_without, double *enob_with)
{
    if (!iface || !enob_without || !enob_with) return -1;
    double f_in = iface->adc.input_bandwidth_hz * 0.3;
    double jitter_native = iface->adc.aperture_jitter_ps_rms;
    double jitter_isolated = iface->clock.isolated_clock_jitter_ps_rms;
    double jitter_total = sqrt(jitter_native * jitter_native
                               + jitter_isolated * jitter_isolated);
    *enob_without = jitter_limited_enob(f_in, jitter_native);
    *enob_with = jitter_limited_enob(f_in, jitter_total);
    return 0;
}

/* L6: max sample rate through isolated interface */
double isolated_adc_max_sample_rate(const isolated_adc_interface_t *iface)
{
    if (!iface) return 0.0;
    if (iface->if_type == ADC_IF_SPI_3WIRE || iface->if_type == ADC_IF_SPI_4WIRE) {
        return iface->config.spi.sclk_freq_hz / (iface->config.spi.bits_per_transfer + 4);
    }
    return iface->adc.sample_rate_sps;
}

/* L6: transfer simulation with latency */
int isolated_adc_transfer_simulate(isolated_adc_interface_t *iface,
                                    const uint8_t *data, size_t len, double *lat_ns)
{
    if (!iface || !data || !lat_ns || len == 0) return -1;
    double prop_delay = iface->isolator->barrier.propagation_delay_ns;
    double bits_total = (double)(len * 8);
    double bit_time_ns = (iface->if_type == ADC_IF_SPI_3WIRE)
                         ? 1e9 / iface->config.spi.sclk_freq_hz
                         : 1e9 / iface->config.i2c.scl_freq_hz;
    *lat_ns = prop_delay * 2.0 + bits_total * bit_time_ns + 50.0;
    iface->samples_transferred++;
    return 0;
}

void isolated_adc_destroy(isolated_adc_interface_t *iface)
{
    if (iface) { free(iface->tx_buffer); free(iface->rx_buffer); memset(iface, 0, sizeof(*iface)); }
}
