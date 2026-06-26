/**
 * @file    ex3_cjc_pipeline.c
 * @brief   Example 3: Multi-type thermocouple comparison with CJC uncertainty
 *
 * Demonstrates:
 *   1. CJC compensation for all 9 thermocouple types
 *   2. CJC uncertainty propagation analysis
 *   3. Seebeck coefficient variation across types
 *   4. Noise budget analysis for measurement system design
 *
 * Knowledge: L4 - CJC principle, L5 - uncertainty propagation, L6 - system design
 */

#include "thermocouple_cjc_rtd.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    tc_type_t types[] = { TC_TYPE_K, TC_TYPE_J, TC_TYPE_T, TC_TYPE_E,
                           TC_TYPE_N, TC_TYPE_R, TC_TYPE_S, TC_TYPE_B, TC_TYPE_C };
    size_t i, n_types = 9;
    double t_hot = 200.0;   /* Hot junction temperature */
    double t_cj  = 30.0;    /* Cold junction at 30C */
    double cj_uncertainty = 0.5; /* CJC sensor accuracy +/-0.5C */

    printf("=== Example 3: Multi-Type Thermocouple CJC Pipeline ===\n");
    printf("Hot junction: %.1f C, Cold junction: %.1f C\n", t_hot, t_cj);
    printf("CJC sensor uncertainty: +/-%.2f C\n\n", cj_uncertainty);

    printf("%-12s %-14s %-14s %-10s %-12s %-14s\n",
           "Type", "EMF_raw(mV)", "EMF_corr(mV)", "T_meas(C)", "S(uV/C)",
           "u_CJC(C)");
    printf("-------------------------------------------------------------------\n");

    for (i = 0; i < n_types; i++) {
        tc_measurement_config_t config;
        double emf_hot, emf_cj, emf_raw, temp_measured;
        double seebeck, cjc_unc;
        tc_error_t err;

        /* Get EMF values */
        tc_temp_to_emf(types[i], t_hot, &emf_hot);
        tc_temp_to_emf(types[i], t_cj, &emf_cj);
        emf_raw = emf_hot - emf_cj;

        /* CJC measurement */
        err = tc_cjc_measure(types[i], emf_raw, t_cj, &temp_measured);

        /* Seebeck at hot junction */
        tc_seebeck_coefficient(types[i], t_hot, &seebeck);

        /* CJC uncertainty propagation */
        tc_cjc_uncertainty(types[i], cj_uncertainty, t_hot, &cjc_unc);

        printf("%-12s %-14.4f %-14.4f %-10.2f %-12.1f %-14.3f",
               tc_type_name(types[i]), emf_raw, emf_hot,
               temp_measured, seebeck * 1000.0, cjc_unc);

        if (err != TC_OK) printf("  [%s]", tc_error_string(err));
        printf("\n");
    }

    /* Noise budget for Type K measurement */
    printf("\n--- Noise Budget: Type K, 100C, BW=10Hz ---\n");
    {
        tc_measurement_config_t config;
        tc_noise_model_t noise;
        double seebeck;

        tc_measurement_init(&config, TC_TYPE_K, WIRE_4_WIRE);
        config.cjc.cj_temperature = 25.0;
        config.sample_rate_hz = 100.0;
        config.adc_bits = 16;
        config.adc_vref = 2.5;

        tc_noise_budget(&config, 100.0, &noise);
        tc_seebeck_coefficient(TC_TYPE_K, 100.0, &seebeck);

        printf("  Johnson noise:       %.2f nVrms\n", noise.johnson_noise_vrms * 1e9);
        printf("  ADC quantization:    %.2f nVrms\n", noise.quant_noise_vrms * 1e9);
        printf("  Amplifier noise:     %.2f nVrms\n", noise.amp_noise_vrms * 1e9);
        printf("  Total noise:         %.2f nVrms\n", noise.total_noise_vrms * 1e9);
        printf("  Noise-equiv temp:    %.4f C\n", noise.noise_equiv_temp);
        printf("  Effective bits:      %.1f\n", noise.effective_bits);
        printf("  SNR:                 %.1f dB\n", noise.snr_db);
    }

    printf("\n=== CJC Pipeline Complete ===\n");
    return 0;
}
