/**
 * @file    ex1_ktype_measurement.c
 * @brief   Example 1: Type K thermocouple measurement with CJC using Pt100 RTD
 *
 * Demonstrates a complete temperature measurement pipeline:
 *   1. Initialize measurement config for Type K with 4-wire RTD CJC
 *   2. Simulate ADC readings for different hot-junction temperatures
 *   3. Apply cold-junction compensation (25C ambient)
 *   4. Display results with uncertainty estimates
 *
 * Knowledge: L6 - Canonical thermocouple measurement problem
 * Reference: NIST ITS-90, IEC 60751
 */

#include "thermocouple_cjc_rtd.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    tc_measurement_config_t config;
    tc_measurement_t result;
    tc_error_t err;
    double test_temps[] = { 0.0, 25.0, 100.0, 250.0, 500.0, 750.0 };
    size_t i, n_tests = 6;
    double ambient = 25.0; /* Cold junction at room temperature */

    printf("=== Example 1: Type K Thermocouple with RTD CJC ===\n");
    printf("Cold Junction Temperature: %.1f C\n\n", ambient);

    /* Initialize measurement system */
    err = tc_measurement_init(&config, TC_TYPE_K, WIRE_4_WIRE);
    if (err != TC_OK) {
        printf("ERROR: Failed to initialize config: %s\n", tc_error_string(err));
        return 1;
    }

    config.cjc.cj_temperature = ambient;
    config.cjc.sensor_type = CJC_SENSOR_RTD;
    config.pga_gain = 32.0;
    config.pga_enabled = 1;
    config.open_tc_threshold = 0.001; /* Low threshold for demo */

    printf("%-10s %-12s %-12s %-12s %-12s\n",
           "T_hot(C)", "EMF_raw(mV)", "EMF_corr(mV)", "T_meas(C)", "Uncert(C)");
    printf("--------------------------------------------------------------\n");

    for (i = 0; i < n_tests; i++) {
        double emf_hot, emf_raw, v_input;
        uint32_t adc_code;

        /* Calculate what the thermocouple would produce */
        tc_temp_to_emf(TC_TYPE_K, test_temps[i], &emf_hot);

        /* The measured EMF is the difference between hot and cold junction EMFs */
        {
            double emf_cj;
            tc_temp_to_emf(TC_TYPE_K, ambient, &emf_cj);
            emf_raw = emf_hot - emf_cj;
        }

        /* Convert EMF to ADC code */
        v_input = emf_raw * 1e-3; /* mV to V */
        if (config.pga_enabled) v_input *= config.pga_gain;
        adc_code = (uint32_t)(v_input / config.adc_vref * 65536.0);

        /* Process through measurement pipeline */
        err = tc_measure_temperature(&config, adc_code, &result);

        printf("%-10.1f %-12.4f %-12.4f %-12.2f %-12.4f\n",
               test_temps[i],
               result.emf_raw,
               result.emf_compensated,
               result.temperature,
               result.uncertainty);

        if (err != TC_OK) {
            printf("  [Warning: %s]\n", tc_error_string(err));
        }
    }

    printf("\n=== Measurement Complete ===\n");
    printf("ADC: %zu-bit, Vref=%.1fV, PGA=%.0fx\n",
           config.adc_bits, config.adc_vref, config.pga_gain);

    return 0;
}
