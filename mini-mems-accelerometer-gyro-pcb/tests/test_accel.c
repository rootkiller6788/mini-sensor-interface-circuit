#include "mems_accel.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { tests_run++; printf("  %s... ", #name); } while(0)
#define PASS() do { tests_passed++; printf("PASS\n"); } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); } while(0)

int main(void)
{
    mems_accel_model_t m;
    double tol = 0.01;

    /* Test model initialization */
    TEST(model_init);
    mems_accel_model_init(&m, ACCEL_FS_16G, ACCEL_MODE_OPEN_LOOP);
    assert(fabs(m.proof_mass_kg - 2.0e-9) < 1e-20);
    assert(fabs(m.spring_constant - 8.0) < 0.01);
    assert(m.op_mode == ACCEL_MODE_OPEN_LOOP);
    PASS();

    /* Test displacement (Hooke's law: x = F/k = ma/k) */
    TEST(displacement);
    double a = 9.80665;
    double x = mems_accel_displacement(&m, a);
    double x_expected = m.proof_mass_kg * a / m.spring_constant;
    assert(fabs(x - x_expected) < tol * fabs(x_expected));
    PASS();

    /* Test pull-in check */
    TEST(pullin_check);
    assert(mems_accel_pullin_check(&m, 0.0) == 0);
    double big_disp = m.capacitive_gap_um * 1e-6 * 0.5;
    assert(mems_accel_pullin_check(&m, big_disp) == 1);
    PASS();

    /* Test transfer function at DC */
    TEST(transfer_fn_dc);
    double hr, hi;
    mems_accel_transfer_fn(&m, 0.0, 0.0, &hr, &hi);
    assert(fabs(hr - 1.0) < tol);
    assert(fabs(hi) < tol);
    PASS();

    /* Test frequency response at resonance */
    TEST(freq_response_resonance);
    double mag, ph;
    double actual_resonance = sqrt(m.spring_constant / m.proof_mass_kg) / (2.0 * M_PI);
    mems_accel_freq_response(&m, actual_resonance, &mag, &ph);
    double Q = m.quality_factor;
    assert(fabs(mag - Q) < Q * 0.3);
    PASS();

    /* Test Brownian noise at room temperature */
    TEST(brownian_noise);
    double nea = mems_accel_brownian_noise(&m, 300.0);
    assert(nea > 0.0);
    assert(nea < 1.0);
    PASS();

    /* Test resolution */
    TEST(resolution);
    double res = mems_accel_resolution(100e-6, 200.0);
    assert(res > 0.0);
    PASS();

    /* Test dynamic range */
    TEST(dynamic_range);
    double dr = mems_accel_dynamic_range_db(16.0 * 9.80665, 0.001);
    assert(dr > 80.0);
    PASS();

    /* Test tilt angles (1g straight down = 0 pitch, 0 roll) */
    TEST(tilt_zero);
    double pitch, roll;
    int valid = mems_accel_tilt_angles(0.0, 0.0, 1.0, &pitch, &roll);
    assert(valid == 1);
    assert(fabs(pitch) < tol);
    assert(fabs(roll) < tol);
    PASS();

    /* Test tilt 90 degrees */
    TEST(tilt_90);
    valid = mems_accel_tilt_angles(1.0, 0.0, 0.0, &pitch, &roll);
    assert(valid == 1);
    assert(fabs(pitch - 90.0) < 5.0);
    PASS();

    /* Test magnitude */
    TEST(magnitude);
    double mag_val = mems_accel_magnitude(1.0, 0.0, 0.0);
    assert(fabs(mag_val - 1.0) < tol);
    PASS();

    /* Test free-fall detection */
    TEST(freefall);
    assert(mems_accel_freefall_detect(0.0, 0.0, 0.0, 0.3) == 1);
    assert(mems_accel_freefall_detect(0.0, 0.0, 1.0, 0.3) == 0);
    PASS();

    /* Test RMS */
    TEST(rms);
    double samples[] = {1.0, -1.0, 1.0, -1.0};
    double rms_v = mems_accel_rms(samples, 4);
    assert(fabs(rms_v - 1.0) < tol);
    PASS();

    /* Test peak-to-peak */
    TEST(peak_to_peak);
    double pp = mems_accel_peak_to_peak(samples, 4);
    assert(fabs(pp - 2.0) < tol);
    PASS();

    /* Test crest factor */
    TEST(crest_factor);
    double cf = mems_accel_crest_factor(samples, 4);
    assert(fabs(cf - 1.0) < tol);
    PASS();

    /* Test null pointer safety */
    TEST(null_safety);
    double v = mems_accel_displacement(NULL, 1.0);
    assert(fabs(v) < tol);
    PASS();

    printf("\nAccelerometer tests: %d/%d passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
