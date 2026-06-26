/**
 * @file bench_channel_throughput.c
 * @brief Benchmark: Channel throughput vs CMTI margin
 */
#include "digital_isolator.h"
#include "isolator_channel.h"
#include <stdio.h>
#include <time.h>

int main(void) {
    printf("=== Channel Throughput Benchmark ===

");
    digital_isolator_t iso;
    digital_isolator_init(&iso, ISOL_TECH_CAPACITIVE, ISOL_CLASS_REINFORCED, 4);

    isolation_channel_t ch;
    isolation_channel_init(&ch, 0, &iso);

    clock_t start = clock();
    int iterations = 100000;
    for (int i = 0; i < iterations; i++) {
        isolation_channel_capacity_compute(&ch, 100e6, 20.0 + (i%20)*0.5);
    }
    clock_t end = clock();
    double elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    printf("Channel capacity computed %d times in %.3f s
", iterations, elapsed);
    printf("Average: %.0f ns per computation
", elapsed / iterations * 1e9);

    start = clock();
    for (int i = 0; i < iterations; i++) {
        isolation_channel_bit_error_rate(&ch, 10.0 + (i%20)*0.5);
    }
    end = clock();
    elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    printf("BER computed %d times in %.3f s
", iterations, elapsed);
    printf("Average: %.0f ns per BER computation
", elapsed / iterations * 1e9);

    isolation_channel_destroy(&ch);
    digital_isolator_destroy(&iso);
    printf("
=== Benchmark complete ===
");
    return 0;
}