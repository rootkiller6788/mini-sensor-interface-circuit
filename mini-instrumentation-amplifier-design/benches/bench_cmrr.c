/**
 * @file bench_cmrr.c
 * @brief Benchmark: CMRR Computation Performance
 *
 * Measures compute performance of CMRR calculation algorithms
 * for various topologies under realistic design scenarios.
 */
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include "../include/ina_core.h"
#include "../include/ina_topology.h"

#define BENCH_ITERATIONS 1000000

static double bench_3opamp_cmrr(int n_iter) {
    volatile double sum = 0.0;
    clock_t start = clock();
    for (int i = 0; i < n_iter; i++) {
        double cmrr = ina_3opamp_cmrr(24700.0, 500.0 + (i % 100) * 50.0,
                                       0.1, 90.0);
        sum += cmrr;
    }
    clock_t end = clock();
    double elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    printf("  3-op-amp CMRR:  %d iterations in %.3f s (%.0f ops/s, sum=%.0f)\n",
           n_iter, elapsed, n_iter / elapsed, sum);
    return elapsed;
}

static double bench_error_budget(int n_iter) {
    InaParameters params;
    memset(&params, 0, sizeof(params));
    params.gain = 100.0;
    params.vos_uv = 50.0;
    params.cmrr_db = 100.0;
    params.psrr_plus_db = 90.0;
    params.gain_error_percent = 0.1;
    params.output_swing_max = 5.0;
    params.output_swing_min = 0.0;

    volatile double sum = 0.0;
    clock_t start = clock();
    for (int i = 0; i < n_iter; i++) {
        InaErrorBudget budget = ina_compute_error_budget(
            &params, 2.5 + i * 0.001, 0.1, 25.0 + i * 0.01, 350.0);
        sum += budget.total_error_rss_uv;
    }
    clock_t end = clock();
    double elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    printf("  Error budget:   %d iterations in %.3f s (%.0f ops/s, sum=%.0f)\n",
           n_iter, elapsed, n_iter / elapsed, sum);
    return elapsed;
}

static double bench_gain_calculation(int n_iter) {
    volatile double sum = 0.0;
    clock_t start = clock();
    for (int i = 0; i < n_iter; i++) {
        double rg = ina_calculate_rg(10.0 + (i % 990), 24700.0);
        double g = ina_calculate_gain_from_rg(rg, 24700.0);
        sum += g;
    }
    clock_t end = clock();
    double elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    printf("  Gain calc:      %d iterations in %.3f s (%.0f ops/s, sum=%.0f)\n",
           n_iter, elapsed, n_iter / elapsed, sum);
    return elapsed;
}

int main(void) {
    printf("=== Instrumentation Amplifier Compute Benchmarks ===\n");
    printf("Iterations per test: %d\n\n", BENCH_ITERATIONS);

    double t1 = bench_3opamp_cmrr(BENCH_ITERATIONS);
    double t2 = bench_error_budget(BENCH_ITERATIONS);
    double t3 = bench_gain_calculation(BENCH_ITERATIONS);

    printf("\n--- Summary ---\n");
    printf("  Total time: %.3f s\n", t1 + t2 + t3);
    printf("  Average:    %.1f M ops/s\n",
           (3.0 * BENCH_ITERATIONS) / (t1 + t2 + t3) / 1e6);

    return 0;
}