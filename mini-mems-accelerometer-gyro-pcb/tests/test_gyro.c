#include "mems_gyro.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>

int main(void)
{
    mems_gyro_model_t m;
    double tol = 0.01;
    int passed = 0, total = 0;

    #define T(name, expr) do { total++; if (expr) { passed++; printf("  PASS: %s\n", #name); } else { printf("  FAIL: %s\n", #name); } } while(0)

    mems_gyro_model_init(&m, GYRO_FS_250DPS, GYRO_MODE_TUNING_FORK);
    T(model_init, fabs(m.drive_freq_hz - 15000.0) < 100.0);

    double fc = mems_gyro_coriolis_force(1e-9, 0.1, 1.0);
    T(coriolis_force_pos, fc > 0.0);

    double disp = mems_gyro_coriolis_displacement(&m, 0.1);
    T(coriolis_displacement_finite, fabs(disp) < 1e-3);

    double mg, ph;
    mems_gyro_mode_transfer(&m, 100.0, &mg, &ph);
    T(mode_transfer_mag, mg >= 0.0);

    double arw = mems_gyro_angle_random_walk(&m, 300.0);
    T(arw_positive, arw > 0.0);

    double means[] = {0.1, 0.12, 0.09, 0.11, 0.1};
    double bi = mems_gyro_bias_instability(means, 5, 1.0);
    T(bias_instability, bi > 0.0 && bi < 0.1);

    double data[] = {0.0, 0.01, -0.01, 0.02, -0.02, 0.01, 0.0, -0.01};
    double av = mems_gyro_allan_variance(data, 8, 2.0, 0.01);
    T(allan_variance, av >= 0.0);

    double arw_out, bi_out, rrw_out;
    double av_arr[] = {0.01, 0.005, 0.008};
    int ret = mems_gyro_identify_noise_sources(av_arr, 3, &arw_out, &bi_out, &rrw_out);
    T(identify_noise, ret == 0);

    double angle = mems_gyro_integrate_angle(90.0, 0.01);
    T(integrate_angle, fabs(angle - 0.9) < 0.1);

    double rates[] = {1.0, 0.0, 0.0};
    double turn = mems_gyro_turn_rate(rates);
    T(turn_rate, fabs(turn - 1.0) < tol);

    double heading = 45.0;
    mems_gyro_heading_update(&heading, 10.0, 0.1);
    T(heading_update, fabs(heading - 46.0) < 0.1);

    printf("\nGyro tests: %d/%d passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
