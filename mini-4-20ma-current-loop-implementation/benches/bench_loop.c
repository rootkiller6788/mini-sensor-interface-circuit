/**
 * @file bench_loop.c
 * @brief Performance benchmarks for 4-20mA current loop operations.
 *
 * Benchmarks measure throughput of key loop analysis and filtering operations
 * for real-time PLC/embedded implementation feasibility assessment.
 */
#include <stdio.h>
#include <time.h>
#include <math.h>
#include "../include/current_loop.h"
#include "../include/transmitter.h"
#include "../include/receiver.h"

#define BENCH_ITERATIONS 1000000

static double get_time_sec(void) {
    return (double)clock() / CLOCKS_PER_SEC;
}

static void bench_kvl(void) {
    current_loop_t loop;
    current_loop_init_standard_24v(&loop);
    double start = get_time_sec();
    for (int i = 0; i < BENCH_ITERATIONS; i++) {
        loop.loop_current_mA = 4.0 + (i % 16000) / 1000.0;
        current_loop_kvl_solve(&loop);
    }
    double elapsed = get_time_sec() - start;
    printf("KVL solve: %.2f ns/call (%d iterations in %.3f s)\n",
           elapsed / BENCH_ITERATIONS * 1e9, BENCH_ITERATIONS, elapsed);
}

static void bench_iir(void) {
    double filtered = 4.0;
    double start = get_time_sec();
    for (int i = 0; i < BENCH_ITERATIONS; i++) {
        filtered = current_loop_iir_filter(12.0, filtered, 0.1);
    }
    double elapsed = get_time_sec() - start;
    printf("IIR filter: %.2f ns/call (%d iterations in %.3f s)\n",
           elapsed / BENCH_ITERATIONS * 1e9, BENCH_ITERATIONS, elapsed);
}

static void bench_ma(void) {
    double buf[8] = {0};
    size_t idx = 0;
    double start = get_time_sec();
    for (int i = 0; i < BENCH_ITERATIONS; i++) {
        current_loop_moving_average(12.0, buf, 8, &idx);
    }
    double elapsed = get_time_sec() - start;
    printf("Moving avg: %.2f ns/call (%d iterations in %.3f s)\n",
           elapsed / BENCH_ITERATIONS * 1e9, BENCH_ITERATIONS, elapsed);
}

int main(void) {
    printf("=== 4-20mA Loop Performance Benchmarks ===\n\n");
    bench_kvl();
    bench_iir();
    bench_ma();
    printf("\n=== Benchmarks Complete ===\n");
    return 0;
}
