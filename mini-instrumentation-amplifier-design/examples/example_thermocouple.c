/**
 * @file example_thermocouple.c
 * @brief End-to-End Example: Thermocouple Measurement System
 *
 * Designs a complete K-type thermocouple measurement chain:
 *   1. Type K thermocouple (Chromel-Alumel, ~41 uV/°C)
 *   2. Cold junction compensation (CJC) using NIST ITS-90
 *   3. Instrumentation amplifier (G=200) for signal conditioning
 *   4. RFI filter for industrial noise environment
 *   5. Burnout detection
 *
 * L7 Application: Industrial temperature measurement
 *
 * Usage: make && ./build/example_thermocouple
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../include/ina_core.h"
#include "../include/ina_sensor.h"
#include "../include/ina_filter.h"
#include "../include/ina_advanced.h"

int main(void) {
    printf("==============================================================\n");
    printf("  Thermocouple Measurement System Design\n");
    printf("  Type K with Cold Junction Compensation (L7 Application)\n");
    printf("==============================================================\n\n");

    /*========================================================================
     * Step 1: TC sensor definition
     *========================================================================*/
    printf("--- Step 1: Sensor Characteristics ---\n");

    double seebeck_25c = thermocouple_seebeck(TC_TYPE_K, 25.0);
    printf("  Type K Seebeck at 25C: %.1f uV/degC\n", seebeck_25c);

    /* Temperature range of interest */
    double t_min = 0.0;     /* 0°C */
    double t_max = 500.0;   /* 500°C */
    printf("  Measurement range: %.0f to %.0f degC\n", t_min, t_max);

    /* At 500°C, Type K produces approximately */
    double v_at_500c = thermocouple_temperature_to_voltage(TC_TYPE_K, 500.0);
    printf("  TC voltage at 500C: %.0f uV\n", v_at_500c);
    printf("  TC voltage at 0C:   0 uV (by definition)\n\n");

    /*========================================================================
     * Step 2: IA gain design
     *========================================================================*/
    printf("--- Step 2: IA Gain Design ---\n");

    /* Target: 0.1°C resolution with 12-bit ADC, 2.5V ref */
    int adc_bits = 12;
    double adc_ref = 2.5;
    double adc_lsb = adc_ref / (1 << adc_bits);
    printf("  ADC: %d-bit, Vref=%.1fV, LSB=%.0f uV\n",
           adc_bits, adc_ref, adc_lsb * 1e6);

    double resolution_c = 0.1;
    double gain_needed = thermocouple_design_ia_gain(TC_TYPE_K,
        500.0, resolution_c, adc_lsb);
    printf("  Required gain for %.1fC resolution: %.0f\n",
           resolution_c, gain_needed);

    /* AD620 IA configuration */
    double r_internal = 24700.0;
    double rg = ina_calculate_rg(gain_needed, r_internal);
    double rg_std = ina_nearest_standard_resistor(rg, 96);
    double gain_actual = ina_calculate_gain_from_rg(rg_std, r_internal);
    printf("  IA: AD620, Rg=%.0f ohm, G=%.1f\n\n", rg_std, gain_actual);

    /*========================================================================
     * Step 3: Cold Junction Compensation examples
     *========================================================================*/
    printf("--- Step 3: Cold Junction Compensation ---\n");
    printf("  T_cj (C) | V_measured (uV) | V_cj (uV) | V_total (uV) | T_hot (C)\n");
    printf("  ---------+------------------+------------+--------------+----------\n");

    /* Simulate CJC at different cold junction temperatures */
    double t_targets[] = {0.0, 100.0, 250.0, 500.0};
    double t_cjs[] = {25.0, 25.0, 25.0, 25.0};

    for (int i = 0; i < 4; i++) {
        /* The TC actually measures (T_hot - T_cj) */
        double v_measured = thermocouple_temperature_to_voltage(
            TC_TYPE_K, t_targets[i] - t_cjs[i]);
        double t_computed = thermocouple_cjc(TC_TYPE_K, v_measured, t_cjs[i]);

        double v_cj = thermocouple_temperature_to_voltage(TC_TYPE_K, t_cjs[i]);
        double v_total = v_measured + v_cj;

        printf("  %8.1f | %15.0f | %10.0f | %12.0f | %9.1f\n",
               t_cjs[i], v_measured, v_cj, v_total, t_computed);
    }
    printf("\n");

    /*========================================================================
     * Step 4: Signal chain filter design
     *========================================================================*/
    printf("--- Step 4: Signal Chain Filtering ---\n");

    /* Industrial environment: need RFI filter + anti-alias */
    double signal_bw = 10.0;  /* 10 Hz (temperature changes slowly) */
    double sample_rate = 100.0;  /* 100 SPS (oversampled) */

    InaSignalChainFilters chain;
    ina_design_signal_chain_filters(signal_bw, sample_rate, adc_bits,
                                     1.0, 0, &chain);

    printf("  RFI Filter:\n");
    printf("    Diff cutoff: %.1f Hz\n",
           chain.input_rfi.differential_cutoff_hz);
    printf("    CM cutoff:   %.1f Hz\n",
           chain.input_rfi.common_mode_cutoff_hz);
    printf("    R_series:    %.0f ohm\n\n",
           chain.input_rfi.r_series[0]);

    printf("  Anti-Alias Filter:\n");
    printf("    Type: Butterworth\n");
    printf("    Order: %d\n", chain.antialias.order);
    printf("    fc: %.1f Hz\n", chain.antialias.cutoff_frequency_hz);
    printf("    Stopband atten: %.1f dB at %.1f Hz\n",
           chain.antialias.stopband_atten_db,
           chain.antialias.stopband_frequency_hz);
    printf("    Total noise BW: %.1f Hz\n\n", chain.total_noise_bw_hz);

    /*========================================================================
     * Step 5: Noise analysis
     *========================================================================*/
    printf("--- Step 5: Noise and SNR Analysis ---\n");

    /* AD620 noise: 9 nV/rtHz at 1 kHz, 0.28 uVpp 0.1-10 Hz */
    double en_ia = 9.0;  /* nV/rtHz */
    double vn_total = ina_rms_noise(en_ia, 0.1, chain.total_noise_bw_hz, 0.0);
    printf("  IA noise density: %.0f nV/rtHz\n", en_ia);
    printf("  Total RMS noise:  %.2f uVrms\n", vn_total * 1e6);

    /* Noise equivalent temperature */
    double t_noise = vn_total * 1e6 / seebeck_25c;
    printf("  Noise-equivalent temperature: %.3f C RMS\n\n", t_noise);

    /*========================================================================
     * Step 6: System performance summary
     *========================================================================*/
    printf("==============================================================\n");
    printf("  System Performance Summary\n");
    printf("==============================================================\n");
    printf("  Sensor:     Type K Thermocouple\n");
    printf("  Range:      %.0f to %.0f degC\n", t_min, t_max);
    printf("  IA:         AD620, G=%.0f\n", gain_actual);
    printf("  CJC:        NIST ITS-90 polynomial\n");
    printf("  Resolution: %.1f degC (target)\n", resolution_c);
    printf("  Noise:      %.3f degC RMS\n", t_noise);
    printf("  SNR at max: %.0f dB\n",
           20.0 * log10(v_at_500c / (vn_total * 1e6)));
    printf("  Filter:     %d-order Butterworth, fc=%.1f Hz\n",
           chain.antialias.order, chain.antialias.cutoff_frequency_hz);
    printf("  ADC:        %d-bit, %.1fV ref\n", adc_bits, adc_ref);
    printf("==============================================================\n");

    return 0;
}