/**
 * @file example_isolated_spi.c
 * @brief L7: Complete isolated SPI ADC data acquisition example
 *
 * Demonstrates an isolated 24-bit sigma-delta ADC interface over SPI
 * with clock isolation, jitter analysis, and ENOB calculation.
 * This is a realistic representation of an industrial sensor interface
 * using TI ISO7842-class digital isolators with an ADS1256 ADC.
 */
#include "digital_isolator.h"
#include "isolated_adc_interface.h"
#include <stdio.h>
#include <math.h>

int main(void) {
    printf("=== Isolated SPI ADC Interface Example (L7) ===

");

    /* Initialize reinforced capacitive isolator (ISO7842-class) */
    digital_isolator_t iso;
    digital_isolator_init(&iso, ISOL_TECH_CAPACITIVE, ISOL_CLASS_REINFORCED, 4);
    printf("Isolator: %s
", iso.part_number);
    printf("  Viso = %.1f kVrms, CMTI = %.0f kV/us
",
           iso.barrier.viso_rms_kv, iso.barrier.cmti_kv_us);
    printf("  Data rate = %.0f Mbps, Prop delay = %.1f ns
",
           iso.barrier.data_rate_mbps, iso.barrier.propagation_delay_ns);

    /* Configure isolated ADC interface */
    isolated_adc_interface_t adc_if;
    isolated_adc_init(&adc_if, ADC_IF_SPI_4WIRE, &iso, 24, 30000.0);
    isolated_adc_spi_config(&adc_if, 0, 1, 2, 3, 4e6, false, true);
    printf("
ADC: %u-bit, %.0f SPS
",
           adc_if.adc.resolution_bits, adc_if.adc.sample_rate_sps);
    printf("  SPI: %.1f MHz SCLK
", adc_if.config.spi.sclk_freq_hz / 1e6);

    /* Analyze jitter impact on ADC performance */
    jitter_snr_impact_t impact;
    isolated_adc_compute_clock_degradation(&adc_if, &impact);
    printf("
Clock jitter analysis:
");
    printf("  Signal freq: %.1f kHz
", impact.signal_freq_hz / 1000.0);
    printf("  Isolated clock jitter: %.2f ps RMS
", impact.jitter_rms_ps);
    printf("  Jitter-limited SNR: %.1f dB
", impact.snr_degradation_db);
    printf("  Jitter-limited ENOB: %.2f bits
", impact.max_enob);

    /* Calculate ENOB degradation */
    double enob_native, enob_isolated;
    isolated_adc_estimate_enob_degradation(&adc_if, &enob_native, &enob_isolated);
    printf("
ENOB comparison:
");
    printf("  Native (no isolation): %.2f bits
", enob_native);
    printf("  With isolation:        %.2f bits
", enob_isolated);
    printf("  Degradation:           %.2f bits
", enob_native - enob_isolated);

    /* Simulate a data transfer */
    uint8_t test_data[] = {0x12, 0x34, 0x56};
    double latency_ns;
    isolated_adc_transfer_simulate(&adc_if, test_data, 3, &latency_ns);
    printf("
SPI transfer latency: %.1f ns
", latency_ns);

    /* Verify isolation safety */
    printf("
Safety verification:
");
    printf("  Reinforced isolation: %s
",
           is_reinforced_isolation(&iso) ? "PASS" : "FAIL");

    isolated_adc_destroy(&adc_if);
    digital_isolator_destroy(&iso);

    printf("
=== Example complete ===
");
    return 0;
}