#ifndef MEMS_SENSOR_FUSION_H
#define MEMS_SENSOR_FUSION_H
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    double w, x, y, z;
} quaternion_t;

typedef struct {
    double roll_deg, pitch_deg, yaw_deg;
} euler_angles_t;

typedef struct {
    double kp, ki;
    double integral_fb_x, integral_fb_y, integral_fb_z;
    double dt;
} mahony_params_t;

typedef struct {
    double beta;
    double dt;
} madgwick_params_t;

typedef struct {
    double P[49];
    double Q_diag[7];
    double R_diag[6];
    double x[7];
    double dt;
} kalman_imu_params_t;

quaternion_t quat_from_euler(double roll, double pitch, double yaw);
euler_angles_t quat_to_euler(quaternion_t q);
quaternion_t quat_multiply(quaternion_t a, quaternion_t b);
quaternion_t quat_conjugate(quaternion_t q);
void quat_normalize(quaternion_t *q);
quaternion_t quat_from_axis_angle(double ax, double ay, double az, double angle);
void quat_to_rotation_matrix(quaternion_t q, double R[9]);
void rotation_matrix_to_quat(double R[9], quaternion_t *q);
void vec_rotate_by_quat(double v[3], quaternion_t q, double out[3]);
double complementary_filter(double accel_angle, double gyro_rate, double dt, double alpha);
void mahony_filter_update(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz, mahony_params_t *params, quaternion_t *q);
void madgwick_filter_update(double ax, double ay, double az, double gx, double gy, double gz, madgwick_params_t *params, quaternion_t *q);
void kalman_imu_init(kalman_imu_params_t *k, double dt);
void kalman_imu_predict(kalman_imu_params_t *k, double gx, double gy, double gz);
void kalman_imu_update_accel(kalman_imu_params_t *k, double ax, double ay, double az);
void kalman_imu_update_mag(kalman_imu_params_t *k, double mx, double my, double mz);
void kalman_imu_get_quat(kalman_imu_params_t *k, quaternion_t *q);
void ahrs_compute_heading(quaternion_t q, double mag_x, double mag_y, double mag_z, double *heading);
void ahrs_gravity_vector(quaternion_t q, double g[3]);
double ahrs_vertical_accel(quaternion_t q, double ax, double ay, double az);
double pdr_step_detect(double accel_mag, double threshold_g, double min_step_s);
double pdr_step_length(double step_freq, double accel_var, double K);
void pdr_position_update(double *x, double *y, double heading, double step_len);
void fusion_gd_orient(double ax, double ay, double az, double mx, double my, double mz, quaternion_t *q, double learning_rate, int iterations);

#endif
