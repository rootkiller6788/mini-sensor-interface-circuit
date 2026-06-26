#include "mems_accel.h"
#include "mems_gyro.h"
#include "mems_sensor_fusion.h"
#include <stdio.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(void)
{
    printf("=== MEMS IMU Sensor Fusion Demo ===\n\n");

    /* Initialize sensor models */
    mems_accel_model_t accel;
    mems_gyro_model_t gyro;
    mems_accel_model_init(&accel, ACCEL_FS_16G, ACCEL_MODE_OPEN_LOOP);
    mems_gyro_model_init(&gyro, GYRO_FS_250DPS, GYRO_MODE_TUNING_FORK);

    /* Print model parameters (knowledge L1) */
    printf("L1 - Accelerometer Model:\n");
    printf("  Proof mass: %.3e kg\n", accel.proof_mass_kg);
    printf("  Resonance: %.1f Hz\n", accel.resonance_freq_hz);
    printf("  Q factor: %.1f\n", accel.quality_factor);
    printf("  Noise density: %.1f ug/sqrt(Hz)\n", accel.noise_density_ug_per_sqrt_hz);

    printf("\nL1 - Gyroscope Model:\n");
    printf("  Drive freq: %.1f Hz\n", gyro.drive_freq_hz);
    printf("  Drive Q: %.1f\n", gyro.drive_q);
    printf("  ARW: %.3f dps/sqrt(h)\n", gyro.angle_random_walk_dps_per_sqrt_h);
    printf("  Bias instability: %.2f dps/h\n", gyro.bias_instability_dps_per_h);

    /* L4 - Noise analysis */
    printf("\nL4 - Noise Analysis:\n");
    double nea = mems_accel_brownian_noise(&accel, 300.0);
    printf("  Brownian NEA: %.2f ug/sqrt(Hz)\n", nea * 1e6);
    double res = mems_accel_resolution(nea, accel.bandwidth_hz);
    printf("  Resolution: %.2f ug\n", res * 1e6);
    double dr = mems_accel_dynamic_range_db(16.0*9.80665, res);
    printf("  Dynamic range: %.1f dB\n", dr);
    double arw = mems_gyro_angle_random_walk(&gyro, 300.0);
    printf("  Gyro ARW: %.4f dps/sqrt(h)\n", arw * 3600.0);

    /* L5 - Quaternion operations demo */
    printf("\nL5 - Quaternion Algebra:\n");
    quaternion_t q0 = {1.0, 0.0, 0.0, 0.0};
    quaternion_t q90 = quat_from_axis_angle(0.0, 0.0, 1.0, M_PI / 2.0);
    printf("  Identity: w=%.3f x=%.3f y=%.3f z=%.3f\n", q0.w, q0.x, q0.y, q0.z);
    printf("  90deg Z-axis: w=%.3f x=%.3f y=%.3f z=%.3f\n", q90.w, q90.x, q90.y, q90.z);
    quaternion_t q180 = quat_multiply(q90, q90);
    euler_angles_t e180 = quat_to_euler(q180);
    printf("  90+90 = %.1f deg yaw\n", e180.yaw_deg);

    /* L6 - Tilt from accelerometer */
    printf("\nL6 - Tilt Measurement:\n");
    double pitch, roll;
    mems_accel_tilt_angles(0.0, 0.0, 1.0, &pitch, &roll);
    printf("  Flat: pitch=%.1f roll=%.1f deg\n", pitch, roll);
    mems_accel_tilt_angles(0.0, 0.5, 0.866, &pitch, &roll);
    printf("  Tilted: pitch=%.1f roll=%.1f deg\n", pitch, roll);

    /* L5 - Complementary filter demo */
    printf("\nL5 - Complementary Filter:\n");
    double angle = 0.0;
    int i;
    for (i = 0; i < 10; i++) {
        angle = complementary_filter(30.0, 0.0, 0.01, 0.98);
    }
    printf("  Converged angle: %.2f deg\n", angle);

    /* L7 - Vibration analysis */
    printf("\nL7 - Vibration Analysis (ISO 10816):\n");
    double vib_data[] = {0.1, -0.2, 0.15, -0.1, 0.2, -0.15, 0.1, -0.2};
    double rms_v = mems_accel_rms(vib_data, 8);
    double pp_v = mems_accel_peak_to_peak(vib_data, 8);
    double cf_v = mems_accel_crest_factor(vib_data, 8);
    printf("  RMS: %.3f g, Peak-to-Peak: %.3f g, Crest Factor: %.2f\n", rms_v, pp_v, cf_v);

    printf("\n=== Demo Complete ===\n");
    return 0;
}
