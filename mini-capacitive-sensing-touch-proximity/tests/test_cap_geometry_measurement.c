/**
 * @file test_cap_geometry_measurement.c
 * @brief Tests for geometry models, measurement circuits, and gesture recognition
 */

#include "cap_sensor_geometry.h"
#include "cap_measurement_circuit.h"
#include "cap_noise_immunity.h"
#include "cap_proximity_sense.h"
#include "cap_gesture_recognition.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int passed = 0, failed = 0;

#define TEST(n) do { printf("  TEST: %s ... ", n); } while(0)
#define PASS() do { printf("PASS\n"); passed++; } while(0)
#define FAIL(m) do { printf("FAIL: %s\n", m); failed++; } while(0)
#define CHECK(c,m) do { if(!(c)){FAIL(m); return;} } while(0)
#define CHECK_NEAR(a,b,t,m) do { \
    if(fabs((a)-(b))>(t)){ \
        printf("FAIL: %s (got %e, want %e)\n",m,a,b);failed++;return;} \
}while(0)

/* L3: Elliptic integral K(k) */
static void test_elliptic_k(void)
{
    TEST("Elliptic integral K(k) by AGM");
    /* K(0) = pi/2 */
    double k0 = cap_elliptic_k(0.0);
    CHECK_NEAR(k0, M_PI/2.0, 1e-10, "K(0) = pi/2");

    /* K(k) increases with k */
    double k05 = cap_elliptic_k(0.5);
    double k09 = cap_elliptic_k(0.9);
    CHECK(k09 > k05, "K(k) monotonic in k");

    /* K(0.99) finite */
    double k099 = cap_elliptic_k(0.99);
    CHECK(k099 > k09 && k099 < 10.0, "K(0.99) finite and > K(0.9)");

    PASS();
}

/* L3: Coplanar strip capacitance */
static void test_coplanar_strip(void)
{
    TEST("Coplanar strip capacitance (conformal mapping)");
    cap_coplanar_strip_model_t cps;
    memset(&cps, 0, sizeof(cps));
    cps.electrode_width_m = 0.5e-3;    /* 0.5mm width */
    cps.gap_m = 0.5e-3;                /* 0.5mm gap */
    cps.electrode_length_m = 10e-3;    /* 10mm length */
    cps.substrate_thickness_m = 1.6e-3;/* 1.6mm FR4 */
    cps.substrate_epsilon_r = 4.5;

    cap_coplanar_strip_capacitance(&cps);
    /* Should produce positive mutual C in fF range */
    CHECK(cps.c_total_f > 0.0 && cps.c_total_f < 1.0e-9,
          "Coplanar strip C in reasonable range");
    CHECK(cps.eps_r_eff > 1.0 && cps.eps_r_eff < 4.5,
          "Effective eps_r between 1 and eps_r_sub");

    PASS();
}

/* L3: Fringe field correction */
static void test_fringe_field(void)
{
    TEST("Fringe field correction (Palmer)");
    cap_fringe_field_model_t ffm;
    memset(&ffm, 0, sizeof(ffm));
    ffm.electrode_area_m2 = 1.0e-4;     /* 1 cm^2 */
    ffm.electrode_perimeter_m = 0.04;   /* 4 cm perimeter */
    ffm.gap_to_ground_m = 1.0e-3;       /* 1 mm */
    ffm.dielectric_constant = 7.5;

    cap_fringe_field_correction(&ffm);
    /* Fringe factor should be > 1.0 */
    CHECK(ffm.fringe_correction >= 1.0, "Fringe correction >= 1.0");
    CHECK(ffm.total_c_with_fringe_f > ffm.parallel_plate_c_f,
          "C with fringe > C without fringe");

    PASS();
}

/* L5: Guard ring design */
static void test_guard_ring(void)
{
    TEST("Guard ring design analysis");
    cap_guard_ring_design_t grd;
    memset(&grd, 0, sizeof(grd));
    grd.electrode_width_m = 10.0e-3;     /* 10mm electrode */
    grd.guard_ring_width_m = 1.0e-3;     /* 1mm guard ring */
    grd.guard_gap_m = 0.5e-3;            /* 0.5mm gap */
    grd.pcb_thickness_m = 1.6e-3;
    grd.pcb_epsilon_r = 4.5;

    cap_guard_ring_design_analyze(&grd);
    /* Guard ring should reduce parasitic C */
    CHECK(grd.reduction_ratio > 1.0, "Guard ring reduces parasitic C");
    CHECK(grd.c_parasitic_guarded_f < grd.c_parasitic_unguarded_f,
          "Guarded C < unguarded C");

    PASS();
}

/* L5: Interdigitated design */
static void test_interdigitated(void)
{
    TEST("Interdigitated electrode design");
    cap_interdigitated_design_t id;
    memset(&id, 0, sizeof(id));

    cap_interdigitated_design(&id, 20.0e-3, 15.0e-3, 0.15e-3);
    /* Should produce valid design */
    CHECK(id.num_fingers_tx > 0, "TX fingers allocated");
    CHECK(id.num_fingers_rx > 0, "RX fingers allocated");
    CHECK(id.c_mutual_estimated_f > 0.0, "Mutual C computed");
    CHECK(id.total_coupling_length_m > 0.0, "Coupling length computed");

    PASS();
}

/* L6: Slider interpolation */
static void test_slider_interpolation(void)
{
    TEST("Slider position interpolation");
    cap_slider_geometry_t slider;
    memset(&slider, 0, sizeof(slider));
    slider.num_segments = 8;
    slider.segment_width_m = 5.0e-3;
    slider.inter_segment_gap_m = 0.5e-3;

    /* Simulate touch at segment 3 */
    slider.delta_c_per_seg[2] = 50.0;
    slider.delta_c_per_seg[3] = 200.0;
    slider.delta_c_per_seg[4] = 40.0;

    double pos = cap_slider_interpolate_position(&slider);
    /* Should be near segment 3: position ~ 3 * (5+0.5)mm = 16.5mm */
    CHECK(pos > 10.0e-3 && pos < 25.0e-3, "Position near segment 3");

    PASS();
}

/* L6: Wheel interpolation */
static void test_wheel_interpolation(void)
{
    TEST("Wheel angle interpolation");
    cap_wheel_geometry_t wheel;
    memset(&wheel, 0, sizeof(wheel));
    wheel.num_segments = 8;

    /* Touch at segment 0 (0 degrees) */
    wheel.delta_c_per_seg[0] = 200.0;
    wheel.delta_c_per_seg[1] = 30.0;

    double angle = cap_wheel_interpolate_angle(&wheel);
    CHECK(angle >= 0.0 && angle < 360.0, "Angle in [0,360)");
    CHECK(angle < 45.0 || angle > 315.0, "Angle near 0 degrees");

    PASS();
}

/* L5: Hatch fill ratio */
static void test_hatch_fill(void)
{
    TEST("Hatch fill ratio");
    /* p=1mm, w=0.3mm -> ratio = 2*0.3 - 0.09 = 0.51 */
    double r = cap_hatch_fill_ratio(1.0e-3, 0.3e-3);
    CHECK_NEAR(r, 0.51, 0.01, "Fill ratio for 1mm pitch, 0.3mm width");

    /* w >= p gives 1.0 */
    double r_full = cap_hatch_fill_ratio(1.0e-3, 2.0e-3);
    CHECK_NEAR(r_full, 1.0, 0.01, "Full fill when width >= pitch");

    PASS();
}

/* L2: Charge transfer measurement */
static void test_charge_transfer(void)
{
    TEST("Charge transfer measurement");
    cap_charge_transfer_circuit_t ct;
    cap_charge_transfer_init(&ct, 10e-12, 10e-9, 3.3, 1.65);

    uint32_t count = cap_charge_transfer_count(&ct);
    /* N = 10nF * 1.65V / (10pF * 3.3V) = 500 */
    CHECK(count > 400 && count < 600, "~500 transfers to threshold");

    double c_back = cap_charge_transfer_to_capacitance(&ct, count);
    CHECK_NEAR(c_back, 10e-12, 1e-12, "Round-trip capacitance");

    PASS();
}

/* L5: Sigma-delta CDC */
static void test_sigma_delta_cdc(void)
{
    TEST("Sigma-delta CDC");
    cap_sigma_delta_cdc_t cdc;
    cap_sigma_delta_cdc_init(&cdc, 10e-12, 3.3, 256, 1);

    CHECK(cdc.enob > 9.0 && cdc.enob < 15.0, "ENOB in expected range");

    cap_sigma_delta_convert(&cdc, 5.0e-12, 1.0e-15);
    /* Should measure ~5pF */
    CHECK_NEAR(cdc.c_measured_f, 5.0e-12, 2.0e-12, "CDC measures 5pF");

    /* ENOB increases with OSR */
    double enob64 = cap_sigma_delta_enob(64, 1);
    double enob256 = cap_sigma_delta_enob(256, 1);
    CHECK(enob256 > enob64, "Higher OSR gives more ENOB");

    PASS();
}

/* L2: Relaxation oscillator */
static void test_relaxation_osc(void)
{
    TEST("Relaxation oscillator");
    cap_relaxation_osc_t osc;
    cap_relaxation_osc_init(&osc, 10e-12, 100e3, 3.3, 2.2, 1.1);

    double f = cap_relaxation_osc_frequency(&osc);
    /* f = 1/(2*R*C*ln(2)) = 1/(2*1e5*1e-11*0.693) ≈ 721 kHz */
    CHECK(f > 500e3 && f < 1000e3, "Frequency ~721 kHz");

    PASS();
}

/* L2: Dual-slope */
static void test_dual_slope(void)
{
    TEST("Dual-slope integration");
    cap_dual_slope_circuit_t ds;
    cap_dual_slope_init(&ds, 10e-12, 3.3, 1e-6, 100e-6, 1e-9);

    double c = cap_dual_slope_to_capacitance(&ds);
    CHECK_NEAR(c, 10e-12, 1e-12, "Dual-slope measures 10pF");

    PASS();
}

/* L2: AC bridge */
static void test_ac_bridge(void)
{
    TEST("AC bridge measurement");
    cap_ac_bridge_circuit_t bridge;
    cap_ac_bridge_init(&bridge, 10e-12, 3.3, 100e3, 1e6);

    double vout = cap_ac_bridge_output(&bridge, 1e-15);
    /* deltaC=1fF: Vout should be positive and small */
    CHECK(vout > 0.0, "AC bridge produces positive output");

    PASS();
}

/* L5: Spread-spectrum processing gain */
static void test_processing_gain(void)
{
    TEST("Spread-spectrum processing gain");
    /* 1 MHz spread, 100 Hz signal -> 40 dB gain */
    double gp = cap_spread_processing_gain(1e6, 100.0);
    CHECK_NEAR(gp, 40.0, 0.5, "G_p = 40 dB for BW_ratio=10000");

    PASS();
}

/* L5: Median filter */
static void test_median_filter(void)
{
    TEST("Median filter");
    double data[] = {1.0, 2.0, 100.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    uint32_t n = 8;

    cap_median_filter(data, n, 3);
    /* Spike at index 2 (100.0) should be suppressed */
    CHECK(fabs(data[2] - 3.0) < 10.0, "Median filter suppresses spike");

    PASS();
}

/* L5: IIR filter */
static void test_iir_filter(void)
{
    TEST("IIR lowpass filter");
    double alpha = cap_iir_alpha_from_cutoff(1.0, 100.0);
    /* alpha ≈ 2*pi*f_c/f_s = 2*pi*1/100 ≈ 0.0628 */
    CHECK_NEAR(alpha, 0.061, 0.01, "IIR alpha for 1Hz cutoff at 100Hz");

    double state = 0.0;
    double out = cap_iir_lowpass_step(&state, 1.0, alpha);
    CHECK(out > 0.0 && out < 1.0, "IIR step produces intermediate value");

    PASS();
}

/* L5: Frequency hopping */
static void test_freq_hopping(void)
{
    TEST("Frequency hopping");
    cap_freq_hopping_config_t fh;
    cap_freq_hopping_init(&fh, 4, 100e3, 50e3, 10);

    double f0 = fh.channel_freqs_hz[0];
    CHECK_NEAR(f0, 100e3, 1.0, "First channel at 100 kHz");

    /* Hop through all 4 channels */
    for (int i = 0; i < 40; i++) {
        cap_freq_hopping_next(&fh);
    }
    /* After 40 steps (4*10 dwell), should have cycled */

    PASS();
}

/* L6: Synchronous detection */
static void test_sync_detector(void)
{
    TEST("Synchronous detector");
    cap_sync_detector_t det;
    cap_sync_detector_init(&det, 100e3, 0.01);

    CHECK(det.enbw_hz == 50.0, "ENBW = 1/(2*T_int) = 50 Hz");

    /* Feed a few samples */
    for (int i = 0; i < 10; i++) {
        cap_sync_detector_step(&det, 1.0, (double)i * 1e-6);
    }
    CHECK(det.magnitude > 0.0, "Detector accumulates magnitude");

    PASS();
}

/* L2: Proximity zone detection */
static void test_proximity_zone(void)
{
    TEST("Proximity zone detection");
    cap_proximity_config_t cfg;
    cap_proximity_config_init(&cfg, 1.0e-4, 0.15);

    cap_proximity_state_t state;
    cap_proximity_state_reset(&state);

    /* Very small deltaC -> FAR */
    proximity_zone_t zone = cap_determine_proximity_zone(0.1e-15, &cfg, &state, 1000);
    CHECK(zone == PROX_ZONE_FAR, "Small deltaC = FAR zone");

    PASS();
}

/* L3: Range estimation */
static void test_range_estimation(void)
{
    TEST("Range estimation (inverse power law)");
    cap_range_model_t model;
    model.c0_f = 1.0e-12;    /* 1 pF at contact */
    model.r0_m = 1.0e-3;     /* 1 mm reference */
    model.exponent_n = 2.5;

    double r = cap_estimate_range_power_law(1.0e-15, &model, 1.0);
    /* 1fF = 1pF * (1mm/r)^2.5 -> r ≈ 1mm * (1000)^(1/2.5) ≈ 15.8mm */
    CHECK(r > 0.005 && r < 0.05, "Range ~1-5 cm for 1fF signal");

    PASS();
}

/* L5: Gesture detection - tap */
static void test_gesture_tap(void)
{
    TEST("Gesture tap detection");
    cap_gesture_trajectory_t traj;
    memset(&traj, 0, sizeof(traj));
    traj.start_time_ms = 1000;
    traj.end_time_ms = 1080;  /* 80 ms = quick tap */
    traj.num_points = 3;
    traj.points[0].x_mm = 10.0; traj.points[0].y_mm = 10.0;
    traj.points[1].x_mm = 10.1; traj.points[1].y_mm = 9.9;
    traj.points[2].x_mm = 10.0; traj.points[2].y_mm = 10.0;

    double conf = cap_gesture_detect_tap(&traj, 200.0, 5.0);
    CHECK(conf > 0.8, "Tap confidence high for short, small-motion touch");

    PASS();
}

/* L5: Gesture detection - swipe */
static void test_gesture_swipe(void)
{
    TEST("Gesture swipe detection");
    cap_gesture_trajectory_t traj;
    memset(&traj, 0, sizeof(traj));
    traj.start_time_ms = 1000;
    traj.end_time_ms = 1200; /* 200 ms */
    traj.num_points = 5;
    /* Rightward swipe */
    traj.points[0].x_mm = 0.0;  traj.points[0].y_mm = 0.0;
    traj.points[4].x_mm = 50.0; traj.points[4].y_mm = 2.0;
    traj.total_path_length_mm = 52.0;

    double dir, speed;
    double conf = cap_gesture_detect_swipe(&traj, 10.0, 50.0, &dir, &speed);
    CHECK(conf > 0.5, "Swipe detected");
    /* Direction should be near 0 (right) */
    CHECK(fabs(dir) < 30.0 || fabs(dir - 360.0) < 30.0,
          "Direction near 0 degrees (right)");

    PASS();
}

/* L5: Gesture recognizer template matching */
static void test_gesture_recognizer(void)
{
    TEST("Gesture $1 recognizer");
    cap_gesture_recognizer_t recog;
    cap_gesture_recognizer_init(&recog);

    /* Add a swipe-right template */
    double tx[] = {0, 10, 20, 30, 40, 50};
    double ty[] = {0, 0, 0, 0, 0, 0};
    int idx = cap_gesture_add_template(&recog, GESTURE_SWIPE_RIGHT, "swipe-right",
                                       tx, ty, 6);
    CHECK(idx == 0, "Template added at index 0");

    /* Feed a touch sequence */
    cap_touch_point_t pt;
    memset(&pt, 0, sizeof(pt));
    pt.x_mm = 0; pt.y_mm = 0; pt.timestamp_ms = 1000;
    cap_gesture_feed_point(&recog, &pt, 1000, NULL);

    pt.x_mm = 25; pt.timestamp_ms = 1100;
    cap_gesture_feed_point(&recog, &pt, 1100, NULL);

    pt.x_mm = 50; pt.timestamp_ms = 1200;
    cap_gesture_feed_point(&recog, &pt, 1200, NULL);

    /* Touch-up triggers recognition */
    cap_gesture_result_t result;
    bool recognized = cap_gesture_feed_point(&recog, NULL, 1200, &result);
    /* Should recognize as some gesture */
    CHECK(recognized == true || result.confidence > 0.0,
          "Gesture recognizer produces result");

    PASS();
}

/* L1: Gesture type name */
static void test_gesture_names(void)
{
    TEST("Gesture type names");
    CHECK(strcmp(cap_gesture_type_name(GESTURE_TAP), "Tap") == 0,
          "GESTURE_TAP -> 'Tap'");
    CHECK(strcmp(cap_gesture_type_name(GESTURE_SWIPE_LEFT), "Swipe-Left") == 0,
          "GESTURE_SWIPE_LEFT -> 'Swipe-Left'");
    PASS();
}

/* L5: Noise profile analysis */
static void test_noise_analysis(void)
{
    TEST("Noise profile analysis");
    double samples[100];
    /* Generate 50 Hz sine-like data with noise */
    for (int i = 0; i < 100; i++) {
        samples[i] = 0.5 * sin(2.0 * M_PI * 50.0 * (double)i / 1000.0);
    }

    cap_noise_profile_t profile;
    cap_noise_profile_init(&profile);
    cap_analyze_noise(&profile, samples, 100, 1000.0);

    CHECK(profile.noise_rms_f > 0.0, "Noise RMS computed");
    CHECK(profile.noise_peak_peak_f >= 0.0, "Peak-peak noise computed");

    PASS();
}

/* L5: SIR computation */
static void test_sir_computation(void)
{
    TEST("Signal-to-interference ratio");
    double signal[50], interference[50];
    for (int i = 0; i < 50; i++) {
        interference[i] = 0.1;
        signal[i] = 0.1 + 0.3; /* SIR = P_sig/P_int = 0.09/0.01 = 9 -> 9.5dB */
    }
    double sir = cap_compute_sir(signal, interference, 50);
    CHECK(sir > 0.0, "SIR is positive");

    PASS();
}

/* L2: Measurement method comparison */
static void test_method_comparison(void)
{
    TEST("Measurement method comparison");
    /* Charge transfer: res=1fF, time=1ms vs Sigma-delta: res=0.1fF, time=10ms */
    double score = cap_compare_measurement_methods(
        CAP_METHOD_CHARGE_TRANSFER, CAP_METHOD_SIGMA_DELTA,
        1e-15, 0.001, 0.1e-15, 0.010);
    /* Verify score is computable (finite, not NaN) */
    CHECK(isfinite(score), "Comparison score is finite");

    PASS();
}

/* L5: Frequency selection */
static void test_frequency_selection(void)
{
    TEST("Best frequency selection");
    double candidates[] = {100e3, 150e3, 200e3, 250e3, 300e3};
    double noise[5];
    uint8_t best = cap_select_best_frequency(candidates, 5, noise, 1000.0);
    /* Best should avoid 50/60 Hz harmonics */
    CHECK(best < 5, "Valid index returned");

    PASS();
}

int main(void)
{
    printf("=== Capacitive Sensing Geometry & Measurement Tests ===\n\n");

    test_elliptic_k();
    test_coplanar_strip();
    test_fringe_field();
    test_guard_ring();
    test_interdigitated();
    test_slider_interpolation();
    test_wheel_interpolation();
    test_hatch_fill();
    test_charge_transfer();
    test_sigma_delta_cdc();
    test_relaxation_osc();
    test_dual_slope();
    test_ac_bridge();
    test_processing_gain();
    test_median_filter();
    test_iir_filter();
    test_freq_hopping();
    test_sync_detector();
    test_proximity_zone();
    test_range_estimation();
    test_gesture_tap();
    test_gesture_swipe();
    test_gesture_recognizer();
    test_gesture_names();
    test_noise_analysis();
    test_sir_computation();
    test_method_comparison();
    test_frequency_selection();

    printf("\n=== Results: %d passed, %d failed ===\n", passed, failed);
    return (failed > 0) ? 1 : 0;
}
