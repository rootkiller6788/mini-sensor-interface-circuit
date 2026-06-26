/**
 * @file bench_cdc_throughput.c
 * @brief Performance benchmark: Sigma-delta CDC conversion time vs resolution
 *
 * Measures the computational throughput of cap_sigma_delta_convert
 * across OSR values from 64 to 4096. Reports ENOB and estimated
 * conversion time for each configuration.
 *
 * L8: Monte Carlo simulation of quantization noise.
 */
#include "cap_measurement_circuit.h"
#include <stdio.h>
#include <math.h>
#include <time.h>

int main(void)
{
    printf("=== CDC Throughput Benchmark ===\n");
    printf("OSR   | ENOB (1st) | ENOB (2nd) | C_res (fF) | Est Time (us)\n");
    printf("------|------------|------------|------------|-------------\n");

    uint32_t osr_values[] = {64, 128, 256, 512, 1024, 2048, 4096};
    double c_ref = 10e-12;  /* 10 pF reference */
    double f_sample = 1e6;  /* 1 MHz sample rate */

    for (int i = 0; i < 7; i++) {
        uint32_t osr = osr_values[i];

        double enob1 = cap_sigma_delta_enob(osr, 1);
        double enob2 = cap_sigma_delta_enob(osr, 2);

        /* Resolution: C_ref / 2^ENOB */
        double res_f = c_ref / pow(2.0, enob1);

        /* Conversion time: osr / f_sample */
        double conv_time_us = (double)osr / f_sample * 1e6;

        printf("%-6u| %-10.1f | %-10.1f | %-10.2f | %-10.1f\n",
               osr, enob1, enob2, res_f * 1e15, conv_time_us);
    }

    /* Monte Carlo noise simulation */
    printf("\n=== Monte Carlo: Quantization Noise Distribution ===\n");
    printf("Running 10000 trials at OSR=256, C_sense=5pF...\n");

    cap_sigma_delta_cdc_t cdc;
    cap_sigma_delta_cdc_init(&cdc, c_ref, 3.3, 256, 1);

    double sum_c = 0.0, sum_c2 = 0.0;
    int trials = 10000;

    for (int t = 0; t < trials; t++) {
        cap_sigma_delta_convert(&cdc, 5e-12, 0.5e-15);
        sum_c += cdc.c_measured_f;
        sum_c2 += cdc.c_measured_f * cdc.c_measured_f;
    }

    double mean_c = sum_c / trials;
    double var_c = sum_c2 / trials - mean_c * mean_c;
    double sigma_c = sqrt(var_c > 0 ? var_c : 0);

    printf("  Mean C_measured: %.3f pF (true=5.000 pF)\n", mean_c * 1e12);
    printf("  Sigma: %.3f fF\n", sigma_c * 1e15);
    printf("  SNR: %.1f dB\n", 20.0 * log10(mean_c / (sigma_c > 0 ? sigma_c : 1e-30)));

    printf("\n=== Benchmark Complete ===\n");
    return 0;
}
