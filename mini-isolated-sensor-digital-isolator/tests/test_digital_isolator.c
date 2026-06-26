/**
 * @file test_digital_isolator.c
 */
#include "digital_isolator.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

double barrier_capacitance_compute(double a, double t, double e);
double barrier_impedance_at_freq(double c, double f);
double isolator_compare_fitness(const digital_isolator_t *iso, double r, double c, double v);

static void test_init(void) {
    digital_isolator_t iso;
    int rc = digital_isolator_init(&iso, ISOL_TECH_CAPACITIVE, ISOL_CLASS_REINFORCED, 4);
    assert(rc == 0);
    assert(iso.barrier.viso_rms_kv > 4.0);
    digital_isolator_destroy(&iso);
    printf("PASS: init\n");
}

static void test_paschen(void) {
    double vb = paschen_breakdown_voltage(0.001, 101325.0);
    assert(vb > 1000.0);
    assert(paschen_breakdown_voltage(0.0, 101325.0) == 0.0);
    printf("PASS: paschen\n");
}

static void test_creepage(void) {
    double c = minimum_creepage_distance(250.0, POLLUTION_DEG_2, 1);
    assert(c > 2.0 && c < 5.0);
    printf("PASS: creepage\n");
}

static void test_arrhenius(void) {
    double af = arrhenius_acceleration_factor(55.0, 125.0, 0.7);
    assert(af > 10.0 && af < 200.0);
    printf("PASS: arrhenius\n");
}

static void test_reinforced(void) {
    digital_isolator_t iso;
    digital_isolator_init(&iso, ISOL_TECH_CAPACITIVE, ISOL_CLASS_REINFORCED, 4);
    assert(is_reinforced_isolation(&iso));
    digital_isolator_destroy(&iso);
    printf("PASS: reinforced\n");
}

int main(void) {
    printf("=== digital_isolator tests ===\n");
    test_init();
    test_paschen();
    test_creepage();
    test_arrhenius();
    test_reinforced();
    printf("=== All tests passed ===\n");
    return 0;
}
