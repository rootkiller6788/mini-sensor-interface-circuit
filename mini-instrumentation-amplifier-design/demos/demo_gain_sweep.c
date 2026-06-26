/**
 * @file demo_gain_sweep.c
 * @brief Demo: IA Gain vs CMRR and Bandwidth Visualization
 *
 * Demonstrates how CMRR and bandwidth change with gain for
 * the 3-op-amp instrumentation amplifier topology.
 * Generates tabular data suitable for plotting (gnuplot).
 *
 * This is a teaching demo for L2 (Core Concepts) and L6 (Canonical Problems).
 */
#include <stdio.h>
#include <math.h>
#include "../include/ina_core.h"
#include "../include/ina_topology.h"

int main(void) {
    printf("# Instrumentation Amplifier: Gain vs Performance Trade-offs\n");
    printf("# 3-op-amp topology, AD620-like (Rf=24.7k, GBW=10MHz)\n");
    printf("#\n");
    printf("# Gain | Rg(ohm) | CMRR(dB) | BW(kHz) | FOM\n");
    printf("#------+---------+-----------+---------+--------\n");

    double r_feedback = 24700.0;   /* AD620 internal Rf */
    double opamp_gbw = 1e7;       /* 10 MHz GBW per op-amp */
    double opamp_cmrr = 100.0;    /* 100 dB per op-amp */

    /* Sweep gain from 1 to 1000 */
    double gains[] = {1.0, 2.0, 5.0, 10.0, 20.0, 50.0,
                      100.0, 200.0, 500.0, 1000.0};
    int n_gains = 10;

    for (int i = 0; i < n_gains; i++) {
        double g = gains[i];
        double rg = ina_calculate_rg(g, r_feedback);
        double cmrr = ina_3opamp_cmrr(r_feedback, rg, 0.1, opamp_cmrr);
        double bw = ina_3opamp_bandwidth(r_feedback, rg, opamp_gbw);

        /* Figure of Merit: CMRR * BW / (1) */
        double fom = cmrr * bw / 1000.0;

        printf("  %.0f | %.1f | %.1f | %.1f | %.1f\n",
               g, rg, cmrr, bw / 1000.0, fom);
    }

    printf("\n# Key observations:\n");
    printf("# 1. CMRR improves with gain (Stage 1 pre-amplification)\n");
    printf("# 2. Bandwidth decreases with gain (GBW/G relationship)\n");
    printf("# 3. Rg decreases as G increases (inverse relationship)\n");
    printf("# 4. FOM peaks at moderate gains (G=10-100)\n");

    return 0;
}