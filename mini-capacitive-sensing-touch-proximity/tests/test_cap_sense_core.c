/**
 * @file test_cap_sense_core.c
 * @brief Unit tests for core capacitive sensing module
 *
 * Tests physical models, noise limits, and system initialization.
 * Each test includes a reference to the physical law being verified.
 */

#include "cap_sense_core.h"
#include "cap_touch_detection.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) do { printf("  TEST: %s ... ", name); } while(0)
#define PASS() do { printf("PASS\n"); tests_passed++; } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); tests_failed++; } while(0)
#define CHECK(cond, msg) do { if (!(cond)) { FAIL(msg); return; } } while(0)
#define CHECK_NEAR(a, b, tol, msg) do { \
    if (fabs((a)-(b)) > (tol)) { \
        printf("FAIL: %s (got %e, expected %e, diff %e)\n", msg, a, b, fabs((a)-(b))); \
        tests_failed++; return; \
    } \
} while(0)

/* ==========================================================================
 * L3: Parallel-plate capacitance model (Maxwell/Gauss)
 * ========================================================================== */
static void test_parallel_plate_c(void)
{
    TEST("Parallel-plate capacitance (Gauss Law)");

    /* 1 cm^2 electrode, 1 mm glass (eps_r=7.5) */
    double area = 1.0e-4;    /* 1 cm^2 */
    double dist = 1.0e-3;    /* 1 mm */
    double er = 7.5;         /* glass */
    double c = cap_parallel_plate_c(area, dist, er);
    /* C = 8.854e-12 * 7.5 * 1e-4 / 1e-3 = 6.6405e-12 = 6.64 pF */
    double expected = 6.6405e-12;
    CHECK_NEAR(c, expected, 0.1e-12, "C_parallel_plate for 1cm2 glass");

    /* Zero distance should return 0 (no short circuit) */
    double c_zero = cap_parallel_plate_c(area, 0.0, er);
    CHECK(c_zero == 0.0, "Zero distance returns 0");

    /* Zero area should return 0 */
    double c_zero_area = cap_parallel_plate_c(0.0, dist, er);
    CHECK(c_zero_area == 0.0, "Zero area returns 0");

    PASS();
}

/* ==========================================================================
 * L3: Human body capacitance model (HBM, IEC 61340-3-1)
 * ========================================================================== */
static void test_body_to_earth_c(void)
{
    TEST("Human body capacitance (HBM)");

    /* 1.7 m, 70 kg person barefoot */
    double c_bare = cap_body_to_earth_c(1.7, 70.0, 0.0);
    /* Expected range: 100-300 pF, our model gives ~150 pF */
    CHECK(c_bare > 50.0e-12 && c_bare < 500.0e-12,
          "Body capacitance within expected range (50-500 pF)");

    /* With thick shoes, should be lower (series capacitance) */
    double c_shoe = cap_body_to_earth_c(1.7, 70.0, 0.02);
    CHECK(c_shoe > 0.0 && c_shoe < c_bare,
          "With shoes, capacitance decreases");

    /* Invalid input returns default HBM value (~150 pF) */
    double c_default = cap_body_to_earth_c(0.0, 0.0, 0.0);
    CHECK_NEAR(c_default, 150.0e-12, 10e-12, "Default HBM value for invalid input");

    PASS();
}

/* ==========================================================================
 * L3: Finger-to-electrode capacitance
 * ========================================================================== */
static void test_finger_electrode_c(void)
{
    TEST("Finger-to-electrode capacitance");

    /* 1 cm^2 finger contact, 1 mm glass, fringe=1.2 */
    double c = cap_finger_electrode_c(1.0e-4, 1.0e-3, 7.5, 1.2);
    /* Expected: C_pp = 6.64 pF, with fringe = 7.97 pF, derated 0.4x = 3.19 pF */
    /* Real measurement: 0.5-2 pF, our model: ~3.2 pF (within order of magnitude) */
    CHECK(c > 0.1e-12 && c < 10.0e-12,
          "Finger capacitance in expected range (0.1-10 pF)");

    PASS();
}

/* ==========================================================================
 * L4: kT/C noise limit (Johnson-Nyquist, 1928)
 * ========================================================================== */
static void test_ktc_noise(void)
{
    TEST("kT/C noise (Johnson-Nyquist)");

    /* At 300K, C=10pF: v_n = sqrt(1.38e-23 * 300 / 1e-11) = 20.3 uV */
    double vn = cap_ktc_noise_voltage(300.0, 10.0e-12);
    double expected = sqrt(1.380649e-23 * 300.0 / 10.0e-12);
    CHECK_NEAR(vn, expected, 1e-7, "kT/C noise at 300K, 10pF");

    /* Larger C = lower noise: C=100pF should have 1/sqrt(10) vn */
    double vn_large = cap_ktc_noise_voltage(300.0, 100.0e-12);
    double ratio = vn / vn_large;
    CHECK_NEAR(ratio, sqrt(10.0), 0.01, "Noise scales as 1/sqrt(C)");

    /* At T=0K, noise should be 0 */
    double vn_zero = cap_ktc_noise_voltage(0.0, 10.0e-12);
    CHECK(vn_zero == 0.0, "Zero noise at 0K");

    PASS();
}

/* ==========================================================================
 * L4: SNR fundamental limit
 * ========================================================================== */
static void test_snr_limit(void)
{
    TEST("SNR fundamental limit");

    /* deltaC=100fF, Vexc=3.3V, C=10pF, T=300K, N=100 */
    double snr = cap_snr_limit_db(100e-15, 3.3, 10e-12, 300.0, 100);
    /* SNR should be positive and reasonable (< 100 dB) */
    CHECK(snr > 0.0 && snr < 100.0, "SNR in reasonable range");

    /* More samples = higher SNR */
    double snr_more = cap_snr_limit_db(100e-15, 3.3, 10e-12, 300.0, 400);
    CHECK(snr_more > snr, "SNR increases with averaging");

    /* Zero deltaC = zero SNR */
    double snr_zero = cap_snr_limit_db(0.0, 3.3, 10e-12, 300.0, 100);
    CHECK(snr_zero == 0.0, "Zero deltaC gives zero SNR");

    PASS();
}

/* ==========================================================================
 * L4: Minimum resolvable deltaC
 * ========================================================================== */
static void test_min_resolvable_delta_c(void)
{
    TEST("Minimum resolvable deltaC");

    /* C=10pF, T=300K, Vexc=3.3V, SNR_req=5, N=100 */
    double min_dc = cap_min_resolvable_delta_c(10e-12, 300.0, 3.3, 5.0, 100);
    /* Expect sub-fF resolution: ~0.03 fF = 3e-17 F */
    CHECK(min_dc > 1e-18 && min_dc < 1e-15,
          "Min resolvable deltaC is sub-fF");

    /* Larger SNR requirement = larger min deltaC */
    double min_dc_high = cap_min_resolvable_delta_c(10e-12, 300.0, 3.3, 10.0, 100);
    CHECK(min_dc_high > min_dc, "Higher SNR requirement needs larger deltaC");

    PASS();
}

/* ==========================================================================
 * L1: System initialization and channel configuration
 * ========================================================================== */
static void test_system_init(void)
{
    TEST("System initialization");

    cap_sensor_system_t sys;
    int ret = cap_sensor_system_init(&sys, 8, 3.3, 10e-12);
    CHECK(ret == 0, "System init returns 0");
    CHECK(sys.num_channels == 8, "8 channels allocated");
    CHECK(sys.v_excitation == 3.3, "Vexc = 3.3V");
    CHECK(sys.channels != NULL, "Channel array allocated");
    CHECK(sys.channels[0].state == TOUCH_IDLE, "Initial state is IDLE");
    CHECK(sys.channels[0].method == CAP_METHOD_CHARGE_TRANSFER,
          "Default method is charge transfer");

    /* Init with invalid params */
    ret = cap_sensor_system_init(&sys, 0, 3.3, 10e-12);
    CHECK(ret == -1, "Init fails with 0 channels");

    cap_sensor_system_destroy(&sys);
    CHECK(sys.channels == NULL, "Channels freed after destroy");

    PASS();
}

/* ==========================================================================
 * L2: Channel configuration
 * ========================================================================== */
static void test_channel_configure(void)
{
    TEST("Channel configuration");

    cap_sensor_channel_t chan;
    memset(&chan, 0, sizeof(chan));

    int ret = cap_channel_configure(&chan, CAP_METHOD_SIGMA_DELTA,
        CAP_SENSE_MUTUAL, ELEC_DISC, 1.0e-4, 1.0e-3, 7.5, 50e-15);
    CHECK(ret == 0, "Channel config returns 0");
    CHECK(chan.method == CAP_METHOD_SIGMA_DELTA, "Method set");
    CHECK(chan.mode == CAP_SENSE_MUTUAL, "Mode set to mutual");
    CHECK(chan.electrode.shape == ELEC_DISC, "Shape set to disc");
    CHECK(chan.threshold_f == 50e-15, "Threshold set to 50 fF");

    /* Null pointer check */
    ret = cap_channel_configure(NULL, CAP_METHOD_CHARGE_TRANSFER,
        CAP_SENSE_SELF, ELEC_RECT, 1e-4, 1e-3, 7.5, 100e-15);
    CHECK(ret == -1, "Config fails with NULL channel");

    PASS();
}

/* ==========================================================================
 * L2: Human body model configuration
 * ========================================================================== */
static void test_human_body_model(void)
{
    TEST("Human body model configuration");

    cap_sensor_channel_t chan;
    memset(&chan, 0, sizeof(chan));

    cap_human_body_model_t hbm;
    hbm.body_to_earth_f = 200.0e-12;
    hbm.finger_to_electrode_f = 2.0e-12;
    hbm.body_resistance_ohm = 1000.0;
    hbm.finger_contact_area_m2 = 1.5e-4;
    hbm.finger_ridge_gap_m = 3.0e-6;

    cap_set_human_body_model(&chan, &hbm);
    CHECK(chan.self_meas.c_body_f == 200.0e-12, "Body C set to 200 pF");

    /* NULL safety */
    cap_set_human_body_model(NULL, &hbm);
    cap_set_human_body_model(&chan, NULL);

    PASS();
}

/* ==========================================================================
 * L5: Baseline tracking (EMA)
 * ========================================================================== */
static void test_baseline_ema(void)
{
    TEST("Baseline tracking (asymmetric EMA)");

    cap_baseline_tracker_config_t cfg;
    cap_baseline_tracker_config_init(&cfg);

    uint32_t baseline = 1000;
    uint32_t raw = 1100; /* Above baseline (possible touch) */
    uint32_t new_bl = cap_baseline_update_ema(baseline, raw, &cfg, true);
    /* During touch, with slow alpha, baseline barely moves */
    CHECK(new_bl >= baseline && new_bl <= raw,
          "Baseline stays between old value and raw");

    /* No touch, raw below baseline: fast recovery */
    raw = 950;
    new_bl = cap_baseline_update_ema(baseline, raw, &cfg, false);
    CHECK(new_bl < baseline && new_bl >= raw,
          "Fast recovery when raw below baseline");

    PASS();
}

/* ==========================================================================
 * L5: Adaptive threshold (Neyman-Pearson)
 * ========================================================================== */
static void test_adaptive_threshold(void)
{
    TEST("Adaptive threshold (Neyman-Pearson CFAR)");

    /* noise=20fF, K=5 -> threshold=100fF */
    double thresh = cap_adaptive_threshold(20e-15, 5.0, 5e-15, 50e-12);
    CHECK_NEAR(thresh, 100e-15, 1e-15, "Threshold = 5*sigma = 100 fF");

    /* Below minimum clamp */
    double thresh_low = cap_adaptive_threshold(0.01e-15, 5.0, 5e-15, 50e-12);
    CHECK_NEAR(thresh_low, 5e-15, 1e-15, "Clamped to minimum 5 fF");

    /* Above maximum clamp */
    double thresh_high = cap_adaptive_threshold(1e-9, 5.0, 5e-15, 50e-12);
    CHECK_NEAR(thresh_high, 50e-12, 1e-15, "Clamped to maximum 50 pF");

    PASS();
}

/* ==========================================================================
 * L5: Noise estimation
 * ========================================================================== */
static void test_noise_estimate(void)
{
    TEST("Running noise estimation");

    double var = 0.0, mean = 10.0;  /* Pre-seed near expected value */
    /* Feed 500 samples with sigma=1.0 noise around mean=10.0 */
    for (int i = 0; i < 500; i++) {
        double sample = 10.0 + ((i % 7) - 3) * 0.5; /* ~sigma=1.0 */
        cap_update_noise_estimate(&var, &mean, sample, 0.02, 0.005);
    }
    /* Mean should stay near 10.0 */
    CHECK(fabs(mean - 10.0) < 0.5, "Mean stays near 10.0");
    /* Variance should be positive */
    CHECK(var > 0.0, "Variance estimate is positive");

    PASS();
}

/* ==========================================================================
 * L6: Touch state machine
 * ========================================================================== */
static void test_touch_state_machine(void)
{
    TEST("Touch state machine");

    cap_sensor_channel_t chan;
    memset(&chan, 0, sizeof(chan));
    chan.channel_id = 0;
    chan.state = TOUCH_IDLE;

    cap_touch_detect_config_t cfg;
    cap_touch_detect_config_init(&cfg);
    cfg.debounce_samples = 2;
    cfg.release_debounce = 1;

    cap_touch_event_t event;
    memset(&event, 0, sizeof(event));

    /* Feed above-threshold samples: should detect touch */
    touch_state_t st;

    /* First above-threshold: still IDLE (debounce not met) */
    st = cap_touch_state_machine(&chan, 200e-15, &cfg, &event, 1000);
    CHECK(st == TOUCH_IDLE, "First above-threshold stays IDLE");

    /* Second above-threshold: should go to DETECT then ACTIVE */
    st = cap_touch_state_machine(&chan, 200e-15, &cfg, &event, 1000);
    CHECK(st == TOUCH_DETECT || st == TOUCH_ACTIVE,
          "Second above-threshold transitions to touch");

    /* Feed below-threshold to release */
    st = cap_touch_state_machine(&chan, 10e-15, &cfg, &event, 1000);
    CHECK(st == TOUCH_RELEASE || st == TOUCH_IDLE,
          "Below-threshold releases");

    PASS();
}

/* ==========================================================================
 * L6: Touch pressure estimation
 * ========================================================================== */
static void test_touch_pressure(void)
{
    TEST("Touch pressure estimation");

    /* C_max=2pF, pressure_ref=1pF */
    double p1 = cap_estimate_touch_pressure(1.0e-12, 2.0e-12, 1.0e-12);
    CHECK(p1 > 0.0 && p1 <= 1.0, "Pressure in [0,1]");

    /* deltaC = 0 -> pressure = 0 */
    double p0 = cap_estimate_touch_pressure(0.0, 2.0e-12, 1.0e-12);
    CHECK(p0 == 0.0, "Zero deltaC = zero pressure");

    /* deltaC = C_max -> pressure = 1.0 (approximately) */
    double p_max = cap_estimate_touch_pressure(1.99e-12, 2.0e-12, 1.0e-12);
    CHECK(p_max > 0.8, "Near-saturation gives high pressure");

    PASS();
}

/* ==========================================================================
 * L6: Water film detection
 * ========================================================================== */
static void test_water_film_detection(void)
{
    TEST("Water film detection");

    /* Water: both self and mutual INCREASE */
    bool is_water = cap_detect_water_film(500e-15, 100e-15, 100e-15, 50e-15);
    CHECK(is_water, "Both increase = water detected");

    /* Finger: self increases, mutual decreases */
    bool is_finger = cap_detect_water_film(500e-15, -100e-15, 100e-15, 50e-15);
    CHECK(!is_finger, "Self up + mutual down = not water (finger)");

    PASS();
}

/* ==========================================================================
 * L2: Delta count to capacitance conversion
 * ========================================================================== */
static void test_delta_conversion(void)
{
    TEST("Delta count to capacitance conversion");

    /* C_ref=10pF, 16-bit, gain=1, delta=1000 counts */
    double dc = cap_delta_count_to_farad(1000, 10e-12, 16, 1.0);
    /* 1000/65536 * 10pF = 0.0153 * 10pF = 0.153 pF = 153 fF */
    double expected = (1000.0 / 65536.0) * 10e-12;
    CHECK_NEAR(dc, expected, 0.01e-15, "Delta count conversion");

    /* Zero resolution bits = error */
    double dc_err = cap_delta_count_to_farad(1000, 10e-12, 0, 1.0);
    CHECK(dc_err == 0.0, "Zero bits returns 0");

    PASS();
}

int main(void)
{
    printf("=== Capacitive Sensing Core Unit Tests ===\n\n");

    test_parallel_plate_c();
    test_body_to_earth_c();
    test_finger_electrode_c();
    test_ktc_noise();
    test_snr_limit();
    test_min_resolvable_delta_c();
    test_system_init();
    test_channel_configure();
    test_human_body_model();
    test_baseline_ema();
    test_adaptive_threshold();
    test_noise_estimate();
    test_touch_state_machine();
    test_touch_pressure();
    test_water_film_detection();
    test_delta_conversion();

    printf("\n=== Results: %d passed, %d failed ===\n", tests_passed, tests_failed);
    return (tests_failed > 0) ? 1 : 0;
}
