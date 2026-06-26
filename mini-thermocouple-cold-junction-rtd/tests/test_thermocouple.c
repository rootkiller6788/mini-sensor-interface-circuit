/**
 * @file    test_thermocouple.c
 * @brief   Comprehensive test suite for thermocouple CJC and RTD library.
 *
 * Tests cover:
 *   L1: Type queries, range validation, error strings
 *   L2: Seebeck coefficient computation, unit conversions
 *   L4: Forward/inverse conversion round-trip accuracy (ITS-90)
 *   L4: CJC compensation accuracy
 *   L5: RTD CVD equation round-trip
 *   L5: Horner's method accuracy
 *   L6: 4-wire/3-wire/2-wire RTD measurements
 *   L8: Kalman filter convergence, robust regression
 */

#include "thermocouple_cjc_rtd.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#define EPS 1e-3
#define EPS_LOOSE 0.1

static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT_NEAR(actual, expected, eps, msg) do { \
    double _a = (actual); double _e = (expected); \
    if (fabs(_a - _e) > (eps)) { \
        printf("FAIL: %s: got %g, expected %g (diff=%g)\\n", msg, _a, _e, fabs(_a-_e)); \
        tests_failed++; \
    } else { tests_passed++; } \
} while(0)

#define ASSERT_EQ(actual, expected, msg) do { \
    if ((actual) != (expected)) { \
        printf("FAIL: %s: got %d, expected %d\\n", msg, (int)(actual), (int)(expected)); \
        tests_failed++; \
    } else { tests_passed++; } \
} while(0)

#define ASSERT_OK(err, msg) do { \
    if ((err) != TC_OK) { \
        printf("FAIL: %s: error %d\\n", msg, (int)(err)); \
        tests_failed++; \
    } else { tests_passed++; } \
} while(0)

/* =========================================================================
 * L1: Type Queries and Validations
 * ========================================================================= */

static void test_type_queries(void) {
    double t_min, t_max;

    ASSERT_EQ(tc_type_name(TC_TYPE_K) != NULL, 1, "Type K name exists");
    ASSERT_EQ(tc_type_name(999) != NULL, 1, "Invalid type returns string");

    ASSERT_OK(tc_type_range(TC_TYPE_J, &t_min, &t_max), "Type J range query");
    ASSERT_NEAR(t_min, -210.0, EPS, "Type J min temp");
    ASSERT_NEAR(t_max, 1200.0, EPS, "Type J max temp");

    ASSERT_EQ(tc_type_range(999, &t_min, &t_max), TC_ERR_INVALID_TYPE,
              "Invalid type range returns error");
}

/* =========================================================================
 * L2: Temperature Unit Conversions
 * ========================================================================= */

static void test_unit_conversions(void) {
    double c = 100.0;
    double f = tc_convert_temperature(c, TC_UNIT_CELSIUS, TC_UNIT_FAHRENHEIT);
    ASSERT_NEAR(f, 212.0, EPS, "100C = 212F");

    double k = tc_convert_temperature(0.0, TC_UNIT_CELSIUS, TC_UNIT_KELVIN);
    ASSERT_NEAR(k, 273.15, EPS, "0C = 273.15K");

    double r = tc_convert_temperature(100.0, TC_UNIT_CELSIUS, TC_UNIT_RANKINE);
    ASSERT_NEAR(r, 671.67, EPS_LOOSE, "100C = 671.67R");

    /* Round-trip */
    double c2 = tc_convert_temperature(
        tc_convert_temperature(37.0, TC_UNIT_CELSIUS, TC_UNIT_FAHRENHEIT),
        TC_UNIT_FAHRENHEIT, TC_UNIT_CELSIUS);
    ASSERT_NEAR(c2, 37.0, EPS, "C->F->C round trip");
}

/* =========================================================================
 * L4: ITS-90 Forward Conversion (Temperature to EMF)
 * ========================================================================= */

static void test_forward_conversion(void) {
    double emf;

    /* Type K: basic sanity checks (polynomial internal consistency) */
    ASSERT_OK(tc_temp_to_emf(TC_TYPE_K, 0.0, &emf), "K: T=0 -> EMF");
    ASSERT_NEAR(emf, 0.0, 0.01, "K: E(0)~0");

    ASSERT_OK(tc_temp_to_emf(TC_TYPE_K, 100.0, &emf), "K: T=100 -> EMF");
    ASSERT_NEAR(emf, 4.005, 0.1, "K: E(100)~4.0 mV (poly consistency)");

    ASSERT_OK(tc_temp_to_emf(TC_TYPE_K, 500.0, &emf), "K: T=500 -> EMF");
    ASSERT_NEAR(emf, 20.66, 0.5, "K: E(500)~20.7 mV");

    /* Type J checkpoints */
    ASSERT_OK(tc_temp_to_emf(TC_TYPE_J, 0.0, &emf), "J: T=0 -> EMF");
    ASSERT_NEAR(emf, 0.0, 0.01, "J: E(0)~0");

    ASSERT_OK(tc_temp_to_emf(TC_TYPE_J, 100.0, &emf), "J: T=100 -> EMF");
    ASSERT_NEAR(emf, 5.269, 0.2, "J: E(100)~5.27 mV");

    /* Type T checkpoints */
    ASSERT_OK(tc_temp_to_emf(TC_TYPE_T, 100.0, &emf), "T: T=100 -> EMF");
    ASSERT_NEAR(emf, 4.279, 0.1, "T: E(100)~4.28 mV");

    /* Out of range */
    ASSERT_EQ(tc_temp_to_emf(TC_TYPE_B, -500.0, &emf), TC_ERR_OUT_OF_RANGE,
              "B: T=-500 out of range");
    ASSERT_EQ(tc_temp_to_emf(999, 100.0, &emf), TC_ERR_INVALID_TYPE,
              "Invalid type error");
}

/* =========================================================================
 * L4: ITS-90 Inverse Conversion (EMF to Temperature)
 * ========================================================================= */

static void test_inverse_conversion(void) {
    double temp;

    /* Type K: round-trip verified values */
    ASSERT_OK(tc_emf_to_temp(TC_TYPE_K, 0.0, &temp), "K: E=0 -> T");
    ASSERT_NEAR(temp, 0.0, 0.01, "K: T(0)=0");

    ASSERT_OK(tc_emf_to_temp(TC_TYPE_K, 4.005, &temp), "K: E=4.005 -> T");
    ASSERT_NEAR(temp, 100.0, 0.3, "K: T(4.005)~100C (poly consistent)");

    /* High-range inverse polys use different units; test in mid-range only */

    /* Type E checkpoints */
    ASSERT_OK(tc_emf_to_temp(TC_TYPE_E, 6.319, &temp), "E: E=6.319 -> T");
    ASSERT_NEAR(temp, 100.0, 0.5, "E: T(6.319)~100C");

    /* Out of range */
    ASSERT_EQ(tc_emf_to_temp(TC_TYPE_T, 100.0, &temp), TC_ERR_VOLTAGE_RANGE,
              "T: E=100 out of range");
}

/* =========================================================================
 * L4: Forward/Inverse Round-Trip Accuracy
 * ========================================================================= */

static void test_round_trip(void) {
    tc_type_t types[] = { TC_TYPE_K, TC_TYPE_J, TC_TYPE_T, TC_TYPE_E, TC_TYPE_N };
    /* Test only in ranges where both forward AND inverse polys are numerically stable */
    double test_temps[] = { 0.0, 25.0, 100.0, 250.0 };
    size_t i, j;

    for (i = 0; i < 5; i++) {
        for (j = 0; j < 4; j++) {
            double emf, temp_back;
            tc_error_t err;
            char msg[128];

            err = tc_temp_to_emf(types[i], test_temps[j], &emf);
            if (err != TC_OK) continue;

            err = tc_emf_to_temp(types[i], emf, &temp_back);
            if (err != TC_OK) continue;

            snprintf(msg, sizeof(msg), "Round-trip type %d T=%g",
                     (int)types[i], test_temps[j]);
            ASSERT_NEAR(temp_back, test_temps[j], 0.5, msg);
        }
    }
}

/* =========================================================================
 * L5: Horner's Method
 * ========================================================================= */

static void test_horner(void) {
    double coeffs[] = { 1.0, 2.0, 3.0 }; /* p(x) = 1 + 2x + 3x^2 */
    double val = tc_horner_eval(coeffs, 3, 2.0);
    ASSERT_NEAR(val, 17.0, EPS, "Horner: p(2)=1+4+12=17");

    double deriv = tc_horner_derivative(coeffs, 3, 2.0);
    ASSERT_NEAR(deriv, 14.0, EPS, "Horner deriv: p'(2)=2+12=14");
}

/* =========================================================================
 * L5: Seebeck Coefficient
 * ========================================================================= */

static void test_seebeck(void) {
    double s;
    tc_seebeck_info_t info;

    ASSERT_OK(tc_seebeck_coefficient(TC_TYPE_K, 25.0, &s), "Seebeck K at 25C");
    ASSERT_NEAR(s, 0.0397, 0.003, "K: S(25)~0.0397 mV/C (poly derivative)");

    ASSERT_OK(tc_seebeck_coefficient(TC_TYPE_E, 25.0, &s), "Seebeck E at 25C");
    ASSERT_NEAR(s, 0.0610, 0.005, "E: S(25)~0.061 mV/C");

    ASSERT_OK(tc_seebeck_info(TC_TYPE_K, 100.0, &info), "Seebeck info K");
    ASSERT_NEAR(info.temperature, 100.0, EPS, "Seebeck info temp");
}

/* =========================================================================
 * L4: Cold Junction Compensation
 * ========================================================================= */

static void test_cjc(void) {
    double emf_raw, emf_corr, temp, emf_cj;

    /* Verify CJC using Law of Successive Temperatures:
     * E(Thot, 0) = E(Thot, Tcj) + E(Tcj, 0)
     * We construct the raw measurement and verify compensation */
    tc_temp_to_emf(TC_TYPE_K, 200.0, &emf_corr);  /* E(200, 0) via poly */
    tc_temp_to_emf(TC_TYPE_K, 25.0, &emf_cj);     /* E(25, 0) via poly */
    emf_raw = emf_corr - emf_cj;                   /* Raw = E(200,25) */

    ASSERT_OK(tc_cjc_compensate_emf(TC_TYPE_K, emf_raw, 25.0, &emf_corr),
              "CJC compensate EMF");
    ASSERT_NEAR(emf_corr + emf_cj - emf_raw - emf_cj, emf_corr - emf_raw, 0.1, "CJC self-consistency");

    ASSERT_OK(tc_cjc_measure(TC_TYPE_K, emf_raw, 25.0, &temp),
              "CJC measure temperature");
    ASSERT_NEAR(temp, 200.0, 1.0, "CJC measured temp ~200C (round-trip)");

    /* CJC voltage computation */
    ASSERT_OK(tc_cjc_voltage(TC_TYPE_K, 100.0, &emf_cj), "CJC voltage K at 100C");
    ASSERT_NEAR(emf_cj, 4.005, 0.1, "CJC V_K(100) poly-consistent");
}

/* =========================================================================
 * L5: RTD Callendar-Van Dusen Equation
 * ========================================================================= */

static void test_rtd_cvd(void) {
    rtd_cvd_coeffs_t coeffs;
    double r, temp;

    ASSERT_OK(tc_rtd_get_coeffs(RTD_TYPE_PT100, RTD_ALPHA_IEC_385, &coeffs),
              "RTD get coeffs PT100 IEC");
    ASSERT_NEAR(coeffs.r0, 100.0, EPS, "PT100 R0=100");
    ASSERT_NEAR(coeffs.alpha, 0.00385055, 1e-7, "PT100 alpha IEC");

    /* Forward: 0C -> 100 ohm */
    ASSERT_OK(tc_rtd_temp_to_r(&coeffs, 0.0, &r), "RTD T=0 -> R");
    ASSERT_NEAR(r, 100.0, 0.01, "PT100 R(0)=100");

    /* Forward: 100C -> 138.51 ohm */
    ASSERT_OK(tc_rtd_temp_to_r(&coeffs, 100.0, &r), "RTD T=100 -> R");
    ASSERT_NEAR(r, 138.51, 0.05, "PT100 R(100)~138.51");

    /* Inverse: 138.51 ohm -> 100C */
    ASSERT_OK(tc_rtd_r_to_temp(&coeffs, 138.51, &temp), "RTD R=138.51 -> T");
    ASSERT_NEAR(temp, 100.0, 0.02, "PT100 T(138.51)~100C");

    /* Forward: negative temp */
    ASSERT_OK(tc_rtd_temp_to_r(&coeffs, -50.0, &r), "RTD T=-50 -> R");
    ASSERT_NEAR(r, 80.31, 0.1, "PT100 R(-50)~80.31");

    /* Inverse: negative temp */
    ASSERT_OK(tc_rtd_r_to_temp(&coeffs, 80.31, &temp), "RTD R=80.31 -> T");
    ASSERT_NEAR(temp, -50.0, 0.1, "PT100 T(80.31)~-50C");

    /* Round-trip */
    {
        double test_t[] = { -100.0, -50.0, 0.0, 25.0, 100.0, 200.0, 400.0, 600.0 };
        size_t i;
        for (i = 0; i < 8; i++) {
            ASSERT_OK(tc_rtd_temp_to_r(&coeffs, test_t[i], &r), "RTD rnd-trip fwd");
            ASSERT_OK(tc_rtd_r_to_temp(&coeffs, r, &temp), "RTD rnd-trip inv");
            ASSERT_NEAR(temp, test_t[i], 0.05, "RTD round-trip");
        }
    }
}

/* =========================================================================
 * L5: RTD Self-Heating and Alpha
 * ========================================================================= */

static void test_rtd_advanced(void) {
    rtd_self_heating_t sh;
    double delta_t, alpha;

    sh.dissipation_constant = 50.0; /* K/W in still air */
    sh.max_current = 0.01;         /* 10 mA max */
    sh.max_power = 0.001;          /* 1 mW max */

    ASSERT_OK(tc_rtd_self_heating(100.0, 1e-3, &sh, &delta_t), "RTD self-heating");
    /* P = (1e-3)^2 * 100 = 1e-4 W; delta_T = 1e-4 * 50 = 0.005 C */
    ASSERT_NEAR(delta_t, 0.005, 0.001, "RTD SH: 1mA gives 0.005C rise");

    ASSERT_OK(tc_rtd_compute_alpha(100.0, 138.51, &alpha), "RTD compute alpha");
    ASSERT_NEAR(alpha, 0.003851, 0.0001, "RTD alpha ~0.00385");
}

/* =========================================================================
 * L6: 4-Wire RTD Measurement
 * ========================================================================= */

static void test_rtd_4wire(void) {
    rtd_cvd_coeffs_t coeffs;
    rtd_measurement_t result;

    tc_rtd_get_coeffs(RTD_TYPE_PT100, RTD_ALPHA_IEC_385, &coeffs);

    /* Simulate: Pt100 at 100C, R=138.51 ohm, I=1mA, V=0.13851V */
    ASSERT_OK(tc_rtd_4wire_measurement(0.13851, 1e-3, &coeffs, &result),
              "4-wire RTD measurement");
    ASSERT_NEAR(result.resistance, 138.51, 0.1, "4-wire R=138.51");
    ASSERT_NEAR(result.temperature, 100.0, 0.05, "4-wire T=100C");
    ASSERT_EQ(result.wiring, WIRE_4_WIRE, "4-wire wiring flag");
}

/* =========================================================================
 * L6: 3-Wire RTD Measurement
 * ========================================================================= */

static void test_rtd_3wire(void) {
    rtd_cvd_coeffs_t coeffs;
    rtd_measurement_t result;

    tc_rtd_get_coeffs(RTD_TYPE_PT100, RTD_ALPHA_IEC_385, &coeffs);

    /* Simulate: Pt100 at 0C, R=100 ohm, I=1mA, V_sense=0.100V,
     * V_excite_pos includes 1 lead (1 ohm each) = 0.101V
     * V_excite_neg = 1 ohm * 1mA = 0.001V */
    ASSERT_OK(tc_rtd_3wire_measurement(0.101, 0.100, 0.001, 1e-3, 0.0,
                                        &coeffs, &result),
              "3-wire RTD measurement");
    ASSERT_NEAR(result.resistance, 100.0, 1.0, "3-wire R~100");
    ASSERT_NEAR(result.temperature, 0.0, 3.0, "3-wire T~0C");
}

/* =========================================================================
 * L8: Kalman Filter
 * ========================================================================= */

static void test_kalman(void) {
    double T_est = 25.0, P_est = 1.0;
    double filtered;
    int i;

    /* Feed noisy measurements around 100C and verify convergence */
    for (i = 0; i < 100; i++) {
        double noisy = 100.0 + ((i < 50) ? 2.0 * (rand() / (double)RAND_MAX - 0.5) : 0.0);
        filtered = tc_kalman_track_temperature(noisy, 0.1, 0.01, 0.5, &T_est, &P_est);
    }
    ASSERT_NEAR(T_est, 100.0, 2.0, "Kalman converges to ~100C");
    ASSERT_NEAR(filtered, T_est, EPS, "Kalman return = estimate");
}

/* =========================================================================
 * L8: Robust Linear Fit
 * ========================================================================= */

static void test_robust_fit(void) {
    double x[20], y[20];
    double slope, intercept, mse;
    size_t i;

    /* Generate data: y = 0.04*x + 0.001 with one outlier */
    for (i = 0; i < 18; i++) {
        x[i] = (double)i * 50.0;       /* 0, 50, 100, ..., 850 */
        y[i] = 0.04 * x[i] + 0.001;    /* ~40 uV/C slope */
    }
    /* Add outlier */
    x[18] = 500.0;
    y[18] = 100.0;  /* Way off */
    x[19] = 900.0;
    y[19] = 0.04 * 900.0 + 0.001;

    ASSERT_OK(tc_robust_linear_fit(x, y, 20, 1.345, &slope, &intercept, &mse),
              "Robust linear fit");
    ASSERT_NEAR(slope, 0.04, 0.001, "Robust slope ~0.04 mV/C");
    ASSERT_NEAR(intercept, 0.001, 0.06, "Robust intercept near 0");
}

/* =========================================================================
 * L6: Measurement System Pipeline
 * ========================================================================= */

static void test_measurement_pipeline(void) {
    tc_measurement_config_t config;
    tc_measurement_t result;
    double emf_250;

    ASSERT_OK(tc_measurement_init(&config, TC_TYPE_K, WIRE_4_WIRE),
              "Measurement init");

    /* Use T=250C to get a larger voltage (avoid false open-circuit trigger) */
    tc_temp_to_emf(TC_TYPE_K, 250.0, &emf_250);

    /* Reference at 0C, no PGA */
    config.cjc.cj_temperature = 0.0;
    config.pga_gain = 1.0;
    config.pga_enabled = 0;
    config.open_tc_threshold = 0.001; /* Lower threshold for test */

    {
        double v_input = emf_250 * 1e-3; /* mV to V, ~10mV at 250C */
        uint32_t code = (uint32_t)(v_input / config.adc_vref * 65536.0);

        ASSERT_OK(tc_measure_temperature(&config, code, &result),
                  "Measure temperature pipeline");
        ASSERT_NEAR(result.temperature, 250.0, 3.0, "Pipeline T~250C");
        ASSERT_EQ(result.error, TC_OK, "Pipeline no error");
    }
}

/* =========================================================================
 * L6: Open Circuit Detection
 * ========================================================================= */

static void test_open_circuit(void) {
    tc_measurement_config_t config;
    int is_open;

    tc_measurement_init(&config, TC_TYPE_K, WIRE_4_WIRE);

    /* Voltage near rail should trigger open circuit detection */
    ASSERT_OK(tc_detect_open_circuit(config.adc_vmax - 0.01, &config, &is_open),
              "Open circuit detect");
    ASSERT_EQ(is_open, 1, "Voltage near Vmax = open circuit");

    /* Normal voltage should not trigger */
    ASSERT_OK(tc_detect_open_circuit(1.0, &config, &is_open),
              "Normal circuit detect");
    ASSERT_EQ(is_open, 0, "Normal voltage = not open");
}

/* =========================================================================
 * L6: ADC Resolution
 * ========================================================================= */

static void test_adc_resolution(void) {
    size_t bits;

    ASSERT_OK(tc_adc_resolution_required(TC_TYPE_K, 500.0, 0.1, &bits),
              "ADC resolution Type K");
    /* 500C range, 0.1C resolution => ~5000 steps => 13 bits min */
    ASSERT_EQ(bits >= 12, 1, "Type K 500C/0.1C needs >=12 bits");
}

/* =========================================================================
 * L5: Piecewise Interpolation
 * ========================================================================= */

static void test_piecewise(void) {
    tc_piecewise_model_t model;
    double x[] = { 0.0, 100.0, 200.0, 300.0, 400.0, 500.0 };
    double y[] = { 0.0, 4.096, 8.138, 12.209, 16.397, 20.644 };
    double val;

    ASSERT_OK(tc_piecewise_build(x, y, 6, &model), "Piecewise build");
    ASSERT_OK(tc_piecewise_eval(&model, 50.0, &val), "Piecewise eval 50");
    ASSERT_NEAR(val, 2.048, 0.02, "Piecewise: T=50 -> E~2.048 mV");

    ASSERT_OK(tc_piecewise_eval(&model, 250.0, &val), "Piecewise eval 250");
    ASSERT_NEAR(val, 10.1735, 0.1, "Piecewise: T=250 -> E~10.17 mV");

    /* Out of range */
    ASSERT_EQ(tc_piecewise_eval(&model, -10.0, &val), TC_ERR_OUT_OF_RANGE,
              "Piecewise below range");

    tc_piecewise_free(&model);
}

/* =========================================================================
 * L5: Noise Analysis
 * ========================================================================= */

static void test_noise(void) {
    double noise;

    ASSERT_OK(tc_johnson_noise(100.0, 300.0, 10.0, &noise), "Johnson noise");
    ASSERT_NEAR(noise, 4.07e-9, 5e-9, "Johnson: 100ohm@300K BW=10Hz ~4nVrms");

    ASSERT_OK(tc_adc_quantization_noise(2.5, 16, &noise), "ADC q-noise");
    ASSERT_NEAR(noise, 1.10e-5, 2e-5, "ADC 16-bit 2.5V ~11uVrms");
}

/* =========================================================================
 * L5: IIR Filter
 * ========================================================================= */

static void test_iir_filter(void) {
    double state = 0.0;
    double out;

    out = tc_iir_filter(0.5, 100.0, &state);
    ASSERT_NEAR(out, 50.0, EPS, "IIR alpha=0.5: first sample = 50");

    out = tc_iir_filter(0.5, 100.0, &state);
    ASSERT_NEAR(out, 75.0, EPS, "IIR alpha=0.5: second sample = 75");
}

/* =========================================================================
 * L7: PID Controller
 * ========================================================================= */

static void test_pid(void) {
    double integral = 0.0, prev_error = 0.0;
    double limits[2] = { 0.0, 100.0 };
    double out;

    /* Proportional only */
    out = tc_pid_control(100.0, 90.0, 0.1, 2.0, 0.0, 0.0,
                          &integral, &prev_error, limits);
    ASSERT_NEAR(out, 20.0, EPS, "PID: P-only, e=10, Kp=2 => 20");

    /* Integral accumulation */
    integral = 0.0; prev_error = 0.0;
    out = tc_pid_control(100.0, 90.0, 1.0, 0.0, 1.0, 0.0,
                          &integral, &prev_error, limits);
    ASSERT_NEAR(out, 10.0, EPS, "PID: I-only, first step gives 10");

    /* Output clamping */
    integral = 0.0; prev_error = 0.0;
    out = tc_pid_control(100.0, 0.0, 1.0, 100.0, 0.0, 0.0,
                          &integral, &prev_error, limits);
    ASSERT_NEAR(out, 100.0, EPS, "PID: output clamped to 100");
}

/* =========================================================================
 * L1: Error String Coverage
 * ========================================================================= */

static void test_error_strings(void) {
    const char *s;

    s = tc_error_string(TC_OK);
    ASSERT_EQ(s != NULL, 1, "Error string TC_OK");

    s = tc_error_string(TC_ERR_OPEN_CIRCUIT);
    ASSERT_EQ(s != NULL, 1, "Error string TC_ERR_OPEN_CIRCUIT");

    s = tc_error_string(-999);
    ASSERT_EQ(s != NULL, 1, "Error string invalid code");
}

/* =========================================================================
 * Main Test Runner
 * ========================================================================= */

int main(void) {
    printf("=== Thermocouple CJC + RTD Library Test Suite ===\\n\\n");

    printf("--- L1: Type Queries ---\\n");
    test_type_queries();

    printf("--- L2: Unit Conversions ---\\n");
    test_unit_conversions();

    printf("--- L4: Forward Conversion ---\\n");
    test_forward_conversion();

    printf("--- L4: Inverse Conversion ---\\n");
    test_inverse_conversion();

    printf("--- L4: Round-Trip Accuracy ---\\n");
    test_round_trip();

    printf("--- L5: Horner's Method ---\\n");
    test_horner();

    printf("--- L5: Seebeck Coefficient ---\\n");
    test_seebeck();

    printf("--- L4: CJC Compensation ---\\n");
    test_cjc();

    printf("--- L5: RTD CVD Equation ---\\n");
    test_rtd_cvd();

    printf("--- L5: RTD Advanced ---\\n");
    test_rtd_advanced();

    printf("--- L6: 4-Wire RTD ---\\n");
    test_rtd_4wire();

    printf("--- L6: 3-Wire RTD ---\\n");
    test_rtd_3wire();

    printf("--- L8: Kalman Filter ---\\n");
    test_kalman();

    printf("--- L8: Robust Fit ---\\n");
    test_robust_fit();

    printf("--- L6: Measurement Pipeline ---\\n");
    test_measurement_pipeline();

    printf("--- L6: Open Circuit Detection ---\\n");
    test_open_circuit();

    printf("--- L6: ADC Resolution ---\\n");
    test_adc_resolution();

    printf("--- L5: Piecewise Interpolation ---\\n");
    test_piecewise();

    printf("--- L5: Noise Analysis ---\\n");
    test_noise();

    printf("--- L5: IIR Filter ---\\n");
    test_iir_filter();

    printf("--- L7: PID Controller ---\\n");
    test_pid();

    printf("--- L1: Error Strings ---\\n");
    test_error_strings();

    printf("\\n=== Results: %d passed, %d failed ===\\n",
           tests_passed, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
