/**
 * @file isolator_test_patterns.h
 * @brief Production Test and Verification Patterns for Digital Isolators
 *
 * Defines standardized test patterns, production test sequences, and
 * reliability verification methods for digital isolator ICs. Covers
 * both wafer-level and packaged-device testing per IEC 60747-5-5.
 *
 * Knowledge Coverage:
 *   L1: PRBS patterns, stuck-at fault, transition fault coverage
 *   L2: production test flow (wafer sort, final test, burn-in)
 *   L3: statistical test limits (Cpk, guard-banding, DPM)
 *   L4: reliability bathtub curve, Weibull distribution for failures
 *   L5: BIST (Built-In Self-Test) for isolation barrier integrity
 *   L6: production testing for ISO7842-class devices
 *
 * References:
 *   - Bushnell & Agrawal "Essentials of Electronic Testing" (2000)
 *   - IEC 60747-5-5 Annex E (Production test methods)
 *   - JEDEC JESD22 reliability test methods
 */

#ifndef ISOLATOR_TEST_PATTERNS_H
#define ISOLATOR_TEST_PATTERNS_H

#include "digital_isolator.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* L1: Test pattern types */
typedef enum {
    PATTERN_PRBS7,
    PATTERN_PRBS9,
    PATTERN_PRBS15,
    PATTERN_PRBS23,
    PATTERN_PRBS31,
    PATTERN_STUCK_AT_ZERO,
    PATTERN_STUCK_AT_ONE,
    PATTERN_TOGGLE,
    PATTERN_WALKING_ONE,
    PATTERN_WALKING_ZERO,
    PATTERN_ALL_ZERO,
    PATTERN_ALL_ONE,
    PATTERN_CHECKERBOARD,
    PATTERN_CUSTOM,
    PATTERN_COUNT
} test_pattern_type_t;

typedef struct {
    test_pattern_type_t type;
    uint32_t pattern_length;
    uint32_t seed;
    uint32_t current_state;
    uint8_t *pattern_data;
    bool auto_advance;
} test_pattern_t;

/* L2: Production test flow */
typedef enum {
    TEST_STAGE_WAFER_SORT,
    TEST_STAGE_FINAL_TEST,
    TEST_STAGE_BURN_IN,
    TEST_STAGE_QUALIFICATION,
    TEST_STAGE_PRODUCTION,
    TEST_STAGE_COUNT
} test_stage_t;

typedef struct {
    test_stage_t stage;
    double temperature_c;
    double supply_voltage_v;
    uint32_t test_duration_ms;
    bool high_voltage_stress;
    double stress_voltage_kv;
} test_conditions_t;

/* L3: Statistical limits */
typedef struct {
    double mean;
    double std_dev;
    double usl;
    double lsl;
    double cp;
    double cpk;
    double dpmo;
    size_t sample_size;
} statistical_limits_t;

/* L4: Reliability models */
typedef struct {
    double scale_parameter_hours;
    double shape_parameter;
    double infant_mortality_rate;
    double useful_life_fit;
    double wearout_onset_hours;
    double mttf_hours;
} weibull_reliability_t;

typedef struct {
    double early_fail_rate_ppm;
    double intrinsic_fail_rate_fit;
    double wearout_fail_rate_fit;
    double confidence_level_pct;
    uint64_t device_hours;
} reliability_bathtub_t;

/* L5: BIST for isolation */
typedef struct {
    bool barrier_integrity_check;
    bool channel_functional_check;
    bool cmti_margin_check;
    bool refresh_monitor;
    double barrier_leakage_threshold_ua;
    double cmti_test_slew_rate_kv_per_us;
    uint32_t test_interval_ms;
    uint32_t consecutive_failures;
    uint32_t max_consecutive_failures;
} isolation_bist_t;

/* L6: Complete test system */
typedef struct {
    digital_isolator_t *dut;
    test_pattern_t *current_pattern;
    test_conditions_t conditions;
    statistical_limits_t stats;
    weibull_reliability_t reliability;
    reliability_bathtub_t bathtub;
    isolation_bist_t bist;
    uint64_t total_bits_tested;
    uint32_t total_errors;
    uint32_t pattern_count;
    double test_time_elapsed_ms;
    bool test_running;
} isolator_test_system_t;

/* API */
int test_pattern_init(test_pattern_t *pattern, test_pattern_type_t type,
                       uint32_t seed);

uint8_t test_pattern_next_bit(test_pattern_t *pattern);

void test_pattern_reset(test_pattern_t *pattern);

int test_system_init(isolator_test_system_t *sys,
                      digital_isolator_t *dut,
                      test_stage_t stage);

int test_system_run_pattern(isolator_test_system_t *sys,
                             test_pattern_type_t pattern_type,
                             uint32_t num_bits);

double test_system_bit_error_rate(const isolator_test_system_t *sys);

int test_system_statistical_limits(isolator_test_system_t *sys,
                                    const double *measurements,
                                    size_t n_measurements,
                                    double usl, double lsl);

int test_system_weibull_fit(isolator_test_system_t *sys,
                             const double *failure_times_hours,
                             size_t n_failures);

double test_system_mttf_estimate(const isolator_test_system_t *sys,
                                  double confidence_level);

int test_system_bist_run(isolator_test_system_t *sys);

bool test_system_bist_pass(const isolator_test_system_t *sys);

int test_system_high_voltage_stress(isolator_test_system_t *sys,
                                     double voltage_kv,
                                     double duration_s);

bool test_system_production_pass(const isolator_test_system_t *sys);

int test_system_partial_discharge_check(isolator_test_system_t *sys,
                                         double test_voltage_kv,
                                         double *apparent_charge_pc);

void test_system_destroy(isolator_test_system_t *sys);

void test_pattern_destroy(test_pattern_t *pattern);

#endif /* ISOLATOR_TEST_PATTERNS_H */
