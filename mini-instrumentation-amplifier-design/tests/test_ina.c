/**
 * @file test_ina.c
 * @brief Comprehensive tests for Instrumentation Amplifier module
 *
 * Tests cover L1-L9 knowledge levels through assert-based verification.
 * Each test validates a specific knowledge point with mathematical correctness.
 *
 * Build: make test
 * Run: make test && ./build/test_ina
 */
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include "../include/ina_core.h"
#include "../include/ina_topology.h"
#include "../include/ina_sensor.h"
#include "../include/ina_filter.h"
#include "../include/ina_calibration.h"
#include "../include/ina_advanced.h"

#define EPS 1e-9
#define EPS_FLOAT 1e-5

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) do { printf("  TEST: %s ... ", name); } while(0)
#define PASS() do { printf("PASSED\n"); tests_passed++; } while(0)
#define FAIL(msg) do { printf("FAILED: %s\n", msg); tests_failed++; } while(0)
#define ASSERT_NEAR(a, b, eps, msg) do { \
    if (fabs((a)-(b)) > (eps)) { FAIL(msg); return; } } while(0)
#define ASSERT_TRUE(cond, msg) do { \
    if (!(cond)) { FAIL(msg); return; } } while(0)

/*===========================================================================
 * L1: Core Definitions Tests
 *===========================================================================*/

static void test_signal_decomposition(void) {
    TEST("Signal decomposition (L4 Superposition)");
    double vdm, vcm, vp, vm;
    ina_decompose_signal(3.0, 1.0, &vdm, &vcm);
    ASSERT_NEAR(vdm, 2.0, EPS, "Vdm should be 2.0V");
    ASSERT_NEAR(vcm, 2.0, EPS, "Vcm should be 2.0V");

    ina_recompose_signal(vdm, vcm, &vp, &vm);
    ASSERT_NEAR(vp, 3.0, EPS, "Recomposed V+ should be 3.0V");
    ASSERT_NEAR(vm, 1.0, EPS, "Recomposed V- should be 1.0V");
    PASS();
}

static void test_cmrr_from_gains(void) {
    TEST("CMRR from gains (L4 Fundamental Law)");
    double cmrr = ina_cmrr_from_gains(100.0, 0.01);
    ASSERT_NEAR(cmrr, 80.0, 0.1, "CMRR(100/0.01) = 80 dB");

    /* Ideal case: zero common-mode gain -> infinite CMRR */
    double cmrr_ideal = ina_cmrr_from_gains(100.0, 0.0);
    ASSERT_TRUE(isinf(cmrr_ideal), "Ideal IA has infinite CMRR");
    PASS();
}

static void test_cmrr_resistor_mismatch(void) {
    TEST("CMRR from resistor mismatch (L5 Algorithm)");
    /* Unity-gain diff amp, 0.1% mismatch */
    double cmrr_val = ina_cmrr_from_resistor_mismatch(1.0, 0.001);
    /* Expected: (1+1)/(4*0.001) = 500 linear = 54 dB */
    ASSERT_NEAR(cmrr_val, 500.0, 1.0, "CMRR for 0.1%% mismatch ~= 500 (54 dB)");

    /* 0.01% mismatch */
    cmrr_val = ina_cmrr_from_resistor_mismatch(1.0, 0.0001);
    ASSERT_NEAR(cmrr_val, 5000.0, 10.0, "CMRR for 0.01%% mismatch ~= 5000");
    PASS();
}

static void test_offset_at_temperature(void) {
    TEST("Offset voltage at temperature (L2 Core Concept)");
    double vos = ina_offset_at_temperature(50.0, 600.0, 85.0, 0.0);
    /* Vos(85C) = 50 + 0.6*(85-25) = 50 + 36 = 86 uV */
    ASSERT_NEAR(vos, 86.0, 0.1, "Vos at 85C");

    vos = ina_offset_at_temperature(50.0, -200.0, 0.0, 0.0);
    /* Vos(0C) = 50 + (-0.2)*(0-25) = 50 + 5 = 55 uV */
    ASSERT_NEAR(vos, 55.0, 0.1, "Vos at 0C with negative drift");
    PASS();
}

static void test_rms_noise(void) {
    TEST("RMS noise calculation (L3 Mathematical Structure)");
    /* White noise only: en=10 nV/rtHz, BW=10 kHz */
    double vn = ina_rms_noise(10.0, 0.1, 10000.0, 0.0);
    /* Expected: 10e-9 * sqrt(10000-0.1) = 10e-9 * 100 = 1 uV */
    ASSERT_NEAR(vn, 1.0e-6, 1e-8, "White noise RMS");

    /* With 1/f noise: en=10 nV/rtHz, corner=100 Hz, BW=0.1-10kHz */
    vn = ina_rms_noise(10.0, 0.1, 10000.0, 100.0);
    /* Has both white and 1/f contributions */
    ASSERT_TRUE(vn > 1.0e-6, "1/f noise increases total RMS");
    PASS();
}

static void test_enob_from_snr(void) {
    TEST("ENOB from SNR (L1 Definition)");
    /* Ideal 16-bit: SNR = 6.02*16 + 1.76 = 98.08 dB */
    double enob = ina_enob_from_snr(98.08);
    ASSERT_NEAR(enob, 16.0, 0.1, "ENOB for ideal 16-bit ADC");

    /* 12-bit system with 66 dB SNR */
    enob = ina_enob_from_snr(66.0);
    ASSERT_NEAR(enob, 10.67, 0.1, "ENOB for 66 dB SNR");
    PASS();
}

/*===========================================================================
 * L5: Algorithms Tests
 *===========================================================================*/

static void test_gain_resistor_calculation(void) {
    TEST("Gain resistor Rg calculation (L6 Canonical)");
    /* AD620: R_internal = 24.7k */
    double rg = ina_calculate_rg(10.0, 24700.0);
    ASSERT_NEAR(rg, 5488.89, 0.1, "Rg for G=10, AD620");

    rg = ina_calculate_rg(100.0, 24700.0);
    ASSERT_NEAR(rg, 498.99, 0.1, "Rg for G=100, AD620");

    /* Gain from Rg */
    double g = ina_calculate_gain_from_rg(5488.89, 24700.0);
    ASSERT_NEAR(g, 10.0, 0.01, "Gain from Rg=5.489k");
    PASS();
}

static void test_nearest_standard_resistor(void) {
    TEST("Nearest standard resistor (L5 Algorithm)");
    double r = ina_nearest_standard_resistor(5489.0, 96);
    /* E96 nearest to 5489: ~5490 */
    ASSERT_TRUE(r > 5000.0 && r < 6000.0,
                "Standard resistor in correct range");
    PASS();
}

static void test_error_budget(void) {
    TEST("Error budget analysis (L5 Algorithm)");
    InaParameters params;
    memset(&params, 0, sizeof(params));
    params.vos_uv = 50.0;
    params.vos_drift_nv_per_C = 600.0;
    params.cmrr_db = 100.0;
    params.psrr_plus_db = 90.0;
    params.gain = 100.0;
    params.gain_error_percent = 0.1;
    params.en_rti_at_gain1_nv_rms = 100.0;
    params.output_swing_max = 4.5;
    params.output_swing_min = 0.5;

    InaErrorBudget budget = ina_compute_error_budget(&params,
        5.0, 0.1, 85.0, 350.0);

    ASSERT_TRUE(budget.total_error_rss_uv > 0.0,
                "Total error budget is non-zero");
    ASSERT_TRUE(budget.total_error_worst_uv >= budget.total_error_rss_uv,
                "Worst-case >= RSS");
    PASS();
}

/*===========================================================================
 * L4/L6: Topology Tests
 *===========================================================================*/

static void test_3opamp_gain(void) {
    TEST("3-op-amp IA gain (L6 Canonical)");
    double g = ina_3opamp_gain(24700.0, 5489.0);
    ASSERT_NEAR(g, 10.0, 0.01, "G = 1 + 2*24.7k/5.489k = 10");

    g = ina_3opamp_gain(25000.0, INFINITY);
    ASSERT_NEAR(g, 1.0, EPS, "G=1 when Rg is open");
    PASS();
}

static void test_3opamp_cmrr(void) {
    TEST("3-op-amp IA CMRR (L6 Canonical)");
    double cmrr = ina_3opamp_cmrr(24700.0, 5489.0, 0.1, 90.0);
    /* High gain + moderate matching => good CMRR */
    /* With 0.1% resistor matching at G=10, CMRR ~54 dB
     * (limited by Stage 2 diff amp, not Stage 1 gain) */
    ASSERT_TRUE(cmrr > 50.0, "CMRR > 50 dB for G=10, 0.1%% matching");
    PASS();
}

static void test_topology_selection(void) {
    TEST("Topology selection (L6 Canonical Problem)");
    InaTopology topo = ina_select_topology(120.0, 500.0, 100.0, 10.0, 5.0);
    ASSERT_TRUE(topo == INA_TOPOLOGY_3OPAMP ||
                topo == INA_TOPOLOGY_INDIRECT_CURRENT,
                "High CMRR selects 3-op-amp or ICF");
    PASS();
}

/*===========================================================================
 * L7: Applications Tests (Sensor Interfaces)
 *===========================================================================*/

static void test_bridge_differential_voltage(void) {
    TEST("Wheatstone bridge output (L7 Application)");
    BridgeSensor bridge;
    memset(&bridge, 0, sizeof(bridge));
    bridge.type = BRIDGE_QUARTER;
    bridge.excitation_voltage = 5.0;
    bridge.nominal_resistance = 350.0;
    bridge.sensor_resistance = 353.5;  /* ~1% change */

    double vdiff = bridge_differential_voltage(&bridge);
    /* Vdiff = 5 * 0.01 / 4.02 = ~12.44 mV */
    ASSERT_NEAR(vdiff, 0.01244, 0.001, "Quarter-bridge output");

    /* Balanced bridge: zero output */
    bridge.sensor_resistance = 350.0;
    vdiff = bridge_differential_voltage(&bridge);
    ASSERT_NEAR(vdiff, 0.0, 1e-10, "Balanced bridge = 0V");
    PASS();
}

static void test_bridge_sensitivity(void) {
    TEST("Bridge sensitivity (L7 Application)");
    BridgeSensor bridge;
    memset(&bridge, 0, sizeof(bridge));

    bridge.type = BRIDGE_QUARTER;
    double s = bridge_sensitivity(&bridge, 2.0);
    ASSERT_NEAR(s, 0.5, EPS, "Quarter-bridge sensitivity = GF/4");

    bridge.type = BRIDGE_FULL;
    s = bridge_sensitivity(&bridge, 2.0);
    ASSERT_NEAR(s, 2.0, EPS, "Full-bridge sensitivity = GF");
    PASS();
}

static void test_strain_gauge(void) {
    TEST("Strain gauge conversion (L7 Application)");
    StrainGauge gauge;
    memset(&gauge, 0, sizeof(gauge));
    gauge.gauge_factor = 2.0;
    gauge.nominal_resistance = 350.0;
    gauge.excitation_voltage = 5.0;
    gauge.poisson_ratio = 0.3;

    /* 1000 ue strain */
    double vout = strain_to_bridge_output(1000.0, &gauge, BRIDGE_QUARTER);
    /* Expected: 5 * 2 * 0.001 / 4 = 2.5 mV */
    ASSERT_NEAR(vout, 0.0025, 0.0001, "1000 ue quarter-bridge output");

    /* Convert back */
    double strain = bridge_output_to_strain(vout, 5.0, 2.0, BRIDGE_QUARTER);
    ASSERT_NEAR(strain, 1000.0, 1.0, "Round-trip strain conversion");
    PASS();
}

static void test_rtd_resistance(void) {
    TEST("RTD Callendar-Van Dusen (L7 Application)");
    RtdSensor rtd;
    memset(&rtd, 0, sizeof(rtd));
    rtd.r0 = PT100_R0;
    rtd.coeff_a = PT100_COEFF_A;
    rtd.coeff_b = PT100_COEFF_B;
    rtd.coeff_c = PT100_COEFF_C;
    rtd.connection = RTD_4WIRE;
    rtd.excitation_current = 0.001;

    /* PT100 at 0 C */
    double r = rtd_resistance_at_temperature(0.0, &rtd);
    ASSERT_NEAR(r, 100.0, 0.01, "PT100 at 0C = 100 ohm");

    /* PT100 at 100 C */
    r = rtd_resistance_at_temperature(100.0, &rtd);
    ASSERT_NEAR(r, 138.51, 0.1, "PT100 at 100C ~= 138.51 ohm");

    /* Inverse: temperature from resistance */
    double t = rtd_temperature_from_resistance(138.51, &rtd);
    ASSERT_NEAR(t, 100.0, 0.1, "Inverse: 138.51 ohm -> 100C");
    PASS();
}

static void test_thermocouple_cjc(void) {
    TEST("Thermocouple CJC (L7 Application)");
    /* Type K, V_measured = 4.096 mV, T_cj = 25 C */
    double t_hot = thermocouple_cjc(TC_TYPE_K, 4096.0, 25.0);
    /* V_cj = 40.6*25 = 1015 uV; V_total = 5111 uV; T_hot ~ 125.9 C */
    ASSERT_TRUE(t_hot > 100.0 && t_hot < 150.0, "CJC produces valid result");
    PASS();
}

/*===========================================================================
 * L5: Filter Tests
 *===========================================================================*/

static void test_antialias_filter_design(void) {
    TEST("Anti-alias filter design (L5 Algorithm)");
    FilterSpec spec;
    ina_design_antialias_filter(1000.0, 10000.0, 12, &spec);
    ASSERT_TRUE(spec.order >= 1, "Filter order >= 1");
    ASSERT_TRUE(spec.order <= 8, "Filter order <= 8 practical limit");
    ASSERT_NEAR(spec.cutoff_frequency_hz, 1000.0, EPS,
                "Cutoff = signal BW");
    PASS();
}

static void test_noise_bandwidth(void) {
    TEST("Noise bandwidth (L5 Algorithm)");
    double nbw = ina_noise_bandwidth(1000.0, FILTER_APPROX_BUTTERWORTH, 1);
    ASSERT_NEAR(nbw, 1571.0, 1.0, "1st-order NBW ~ 1.571 * fc");

    nbw = ina_noise_bandwidth(1000.0, FILTER_APPROX_BUTTERWORTH, 4);
    ASSERT_NEAR(nbw, 1026.0, 5.0, "4th-order NBW ~ 1.026 * fc");
    PASS();
}

static void test_sallen_key_design(void) {
    TEST("Sallen-Key filter design (L6 Canonical)");
    SallenKeyFilter sk = ina_design_sallen_key_lowpass(1000.0, 0.7071, 1e-9);
    ASSERT_NEAR(sk.f0, 1000.0, 1.0, "Cutoff frequency");
    ASSERT_NEAR(sk.gain, 1.586, 0.01, "Butterworth gain = 1.586");

    /* Frequency response at fc */
    double mag, phase;
    ina_sallen_key_response(&sk, 1000.0, &mag, &phase);
    ASSERT_NEAR(mag, 1.0 / sqrt(2.0) * 1.586, 0.01,
                "Magnitude at fc = -3dB * passband gain");
    PASS();
}

static void test_notch_filter(void) {
    TEST("50 Hz notch filter (L6 Canonical)");
    TwinTNotchFilter notch = ina_design_notch_filter(50.0, 5.0, 0.1e-6);

    /* At notch frequency: deep attenuation */
    double mag = ina_notch_filter_response(&notch, 50.0);
    ASSERT_TRUE(mag < 0.01, "Notch depth < -40 dB at f0");

    /* At other frequencies: near unity */
    mag = ina_notch_filter_response(&notch, 200.0);
    ASSERT_TRUE(mag > 0.9, "Passband near unity");
    PASS();
}

/*===========================================================================
 * L5: Calibration Tests
 *===========================================================================*/

static void test_two_point_calibration(void) {
    TEST("Two-point calibration (L5 Algorithm)");
    /* Zero point: input=0, measured=0.005 (5mV offset) */
    /* FS point: input=5.0, measured=4.995 */
    CalLinearModel cal = ina_calibrate_two_point(0.0, 0.005, 5.0, 4.995);
    ASSERT_NEAR(cal.gain, 0.998, 0.001, "Gain ~ 0.998");
    ASSERT_NEAR(cal.offset, 0.005, 0.001, "Offset ~ 0.005");

    /* Apply calibration */
    double corrected = ina_apply_linear_calibration(4.995, &cal);
    ASSERT_NEAR(corrected, 5.0, 0.001, "Corrected = 5.0");
    PASS();
}

static void test_polynomial_calibration(void) {
    TEST("Polynomial calibration (L5 Algorithm)");
    CalPoint points[5];
    /* y = 2x + 0.1x^2 */
    for (int i = 0; i < 5; i++) {
        points[i].input_value = i * 1.0;
        points[i].measured_value = 2.0 * points[i].input_value
                                   + 0.1 * points[i].input_value
                                     * points[i].input_value;
        points[i].uncertainty = 0.01;
    }

    CalPolynomialModel model = ina_calibrate_polynomial(points, 5, 2);
    ASSERT_TRUE(model.order == 2, "Polynomial order = 2");
    ASSERT_TRUE(model.r_squared > 0.99, "Good fit R^2 > 0.99");

    /* Apply correction */
    double corrected = ina_apply_polynomial_calibration(3.0, &model);
    ASSERT_NEAR(corrected, 6.9, 0.1, "y = 2*3 + 0.1*9 = 6.9");
    PASS();
}

/*===========================================================================
 * L8: Advanced Topics Tests
 *===========================================================================*/

static void test_chopper_analysis(void) {
    TEST("Chopper-stabilized IA analysis (L8 Advanced)");
    ChopperIaConfig config;
    ina_chopper_analyze(1000.0, 10e6, 10000.0, &config);
    ASSERT_TRUE(config.residual_offset_nv < 1000.0,
                "Chopper reduces offset to sub-uV range");
    ASSERT_TRUE(config.bandwidth_hz > 0.0,
                "Usable bandwidth > 0");
    PASS();
}

static void test_pga_design(void) {
    TEST("PGA gain table design (L8 Advanced)");
    int num_steps = 4;
    PgaGainStep steps[4];
    ina_pga_design_gain_table(1.0, 1000.0, num_steps, 1, 24700.0, steps);

    ASSERT_NEAR(steps[0].gain_setting, 1.0, 0.1, "Min gain");
    ASSERT_NEAR(steps[num_steps-1].gain_setting, 1000.0, 10.0, "Max gain");

    /* Check monotonic */
    for (int i = 1; i < num_steps; i++) {
        ASSERT_TRUE(steps[i].gain_setting > steps[i-1].gain_setting,
                    "Gain steps are monotonic");
    }
    PASS();
}

static void test_kalman_filter(void) {
    TEST("Kalman filter offset tracking (L8 Advanced)");
    KalmanCalState state;
    ina_kalman_offset_init(&state, 50.0, 10.0, 1.0);

    /* Steady state: should track measurements */
    for (int i = 0; i < 10; i++) {
        ina_kalman_offset_update(&state, 55.0, 4.0);
    }
    ASSERT_NEAR(state.state_estimate, 55.0, 1.0,
                "Kalman converged near measurement");
    ASSERT_TRUE(state.state_variance < 10.0,
                "Variance reduced by filtering");
    PASS();
}

/*===========================================================================
 * Main Test Runner
 *===========================================================================*/

int main(void) {
    printf("=== Instrumentation Amplifier Module Tests ===\n\n");

    printf("--- L1/L2/L3/L4: Core Definitions and Fundamental Laws ---\n");
    test_signal_decomposition();
    test_cmrr_from_gains();
    test_cmrr_resistor_mismatch();
    test_offset_at_temperature();
    test_rms_noise();
    test_enob_from_snr();

    printf("\n--- L5: Algorithms ---\n");
    test_gain_resistor_calculation();
    test_nearest_standard_resistor();
    test_error_budget();

    printf("\n--- L4/L6: Topology and Canonical Problems ---\n");
    test_3opamp_gain();
    test_3opamp_cmrr();
    test_topology_selection();

    printf("\n--- L7: Sensor Interface Applications ---\n");
    test_bridge_differential_voltage();
    test_bridge_sensitivity();
    test_strain_gauge();
    test_rtd_resistance();
    test_thermocouple_cjc();

    printf("\n--- L5: Filter Design ---\n");
    test_antialias_filter_design();
    test_noise_bandwidth();
    test_sallen_key_design();
    test_notch_filter();

    printf("\n--- L5: Calibration ---\n");
    test_two_point_calibration();
    test_polynomial_calibration();

    printf("\n--- L8: Advanced Topics ---\n");
    test_chopper_analysis();
    test_pga_design();
    test_kalman_filter();

    printf("\n==========================================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("==========================================\n");

    return tests_failed > 0 ? 1 : 0;
}