/**
 * @file isolated_adc_interface.h
 * @brief Isolated ADC Interface — Clock/Data Isolation for Precision ADCs
 *
 * Implements the complete interface for isolating high-precision ADC digital
 * interfaces (SPI, I2C, parallel) across galvanic isolation barriers.
 * Addresses clock jitter transfer, phase noise coupling, and sample timing
 * integrity — critical for >16-bit ADC performance.
 *
 * Knowledge Coverage:
 *   L1: ENOB, SINAD, SFDR, aperture jitter, clock phase noise
 *   L2: isolated SPI with bidirectional clock, isolated I2C with stretch
 *   L3: jitter-to-SNR degradation: SNR_jitter = -20*log10(2*pi*f_in*t_jitter)
 *   L4: aperture uncertainty and sampling theorem in isolated context
 *   L5: clock-forward vs clock-recovery in isolated ADC links
 *   L6: isolated 24-bit sigma-delta ADC interface, isolated SAR ADC
 *
 * References:
 *   - Kester "The Data Conversion Handbook" (ADI, 2005)
 *   - TI "Clocking for High-Speed ADCs" (SNAA127)
 *   - IEEE 1241-2010 ADC Test Standard
 */

#ifndef ISOLATED_ADC_INTERFACE_H
#define ISOLATED_ADC_INTERFACE_H

#include "digital_isolator.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* L1: ADC performance metrics */
typedef struct {
    uint8_t resolution_bits;
    double enob_bits;
    double sinad_db;
    double snr_db;
    double sfdr_db;
    double thd_db;
    double dnl_lsb;
    double inl_lsb;
    double offset_error_mv;
    double gain_error_pct;
    double sample_rate_sps;
    double input_bandwidth_hz;
    double aperture_jitter_ps_rms;
} adc_performance_t;

typedef enum {
    ADC_IF_SPI_3WIRE,
    ADC_IF_SPI_4WIRE,
    ADC_IF_I2C,
    ADC_IF_PARALLEL,
    ADC_IF_LVDS_SERIAL
} adc_interface_type_t;

/* L2: Isolated ADC interface configuration */
typedef struct {
    adc_interface_type_t if_type;
    uint8_t sclk_channel;
    uint8_t mosi_channel;
    uint8_t miso_channel;
    uint8_t cs_channel;
    double sclk_freq_hz;
    double data_rate_sps;
    bool cpol;
    bool cpha;
    uint8_t bits_per_transfer;
    bool double_data_rate;
} isolated_spi_config_t;

typedef struct {
    uint8_t scl_channel;
    uint8_t sda_channel;
    uint8_t slave_addr_7bit;
    double scl_freq_hz;
    bool clock_stretching_enabled;
    uint32_t clock_stretch_timeout_us;
} isolated_i2c_config_t;

/* L3: Clock isolation and jitter */
typedef struct {
    double isolated_clock_jitter_ps_rms;
    double source_clock_jitter_ps_rms;
    double isolator_additive_jitter_ps_rms;
    double clock_phase_noise_dbc_at_1k;
    double clock_phase_noise_dbc_at_10k;
    double clock_phase_noise_dbc_at_100k;
    double pll_bandwidth_hz;
} isolated_clock_params_t;

typedef struct {
    double signal_freq_hz;
    double jitter_rms_ps;
    double snr_degradation_db;
    double max_enob;
} jitter_snr_impact_t;

/* L4: Aperture uncertainty model */
typedef struct {
    double aperture_delay_ps;
    double aperture_uncertainty_ps_rms;
    double clock_to_data_skew_ps;
    double sample_to_sample_jitter_ps;
    double temperature_coeff_ps_per_c;
} aperture_timing_t;

/* L5: Clock distribution */
typedef enum {
    CLOCK_METHOD_FORWARD,
    CLOCK_METHOD_RECOVERY,
    CLOCK_METHOD_FREE_RUNNING,
    CLOCK_METHOD_STROBE
} clock_distribution_method_t;

typedef struct {
    clock_distribution_method_t method;
    double forward_clock_freq_hz;
    double recovered_clock_jitter_ps;
    double free_run_accuracy_ppm;
    double strobe_window_ns;
    bool uses_pll;
} clock_distribution_t;

/* L6: Complete isolated ADC interface */
typedef struct {
    adc_performance_t adc;
    adc_interface_type_t if_type;
    union {
        isolated_spi_config_t spi;
        isolated_i2c_config_t i2c;
    } config;
    isolated_clock_params_t clock;
    aperture_timing_t aperture;
    clock_distribution_t clock_dist;
    digital_isolator_t *isolator;
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    size_t buffer_size;
    uint32_t samples_transferred;
    uint32_t transfer_errors;
} isolated_adc_interface_t;

/* API */
int isolated_adc_init(isolated_adc_interface_t *iface,
                       adc_interface_type_t if_type,
                       digital_isolator_t *isolator,
                       uint8_t resolution_bits,
                       double sample_rate_sps);

int isolated_adc_spi_config(isolated_adc_interface_t *iface,
                             uint8_t sclk_ch, uint8_t mosi_ch,
                             uint8_t miso_ch, uint8_t cs_ch,
                             double sclk_freq_hz, bool cpol, bool cpha);

int isolated_adc_i2c_config(isolated_adc_interface_t *iface,
                             uint8_t scl_ch, uint8_t sda_ch,
                             uint8_t slave_addr, double scl_freq_hz);

double jitter_limited_snr(double signal_freq_hz, double jitter_rms_ps);

double jitter_limited_enob(double signal_freq_hz, double jitter_rms_ps);

double aperture_jitter_from_phase_noise(double freq_offset_hz,
                                         double phase_noise_dbc,
                                         double carrier_freq_hz);

int isolated_adc_compute_clock_degradation(isolated_adc_interface_t *iface,
                                            jitter_snr_impact_t *impact);

int isolated_adc_estimate_enob_degradation(const isolated_adc_interface_t *iface,
                                            double *enob_without_iso,
                                            double *enob_with_iso);

double isolated_adc_max_sample_rate(const isolated_adc_interface_t *iface);

int isolated_adc_transfer_simulate(isolated_adc_interface_t *iface,
                                    const uint8_t *data, size_t len,
                                    double *latency_ns);

void isolated_adc_destroy(isolated_adc_interface_t *iface);

#endif /* ISOLATED_ADC_INTERFACE_H */
