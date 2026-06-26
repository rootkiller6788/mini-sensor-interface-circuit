#ifndef MEMS_CALIBRATION_H
#define MEMS_CALIBRATION_H
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "mems_accel.h"
#include "mems_gyro.h"

typedef struct {
    double temp_c;
    double bias_x, bias_y, bias_z;
    double scale_x, scale_y, scale_z;
} temp_cal_point_t;

typedef struct {
    double tc_bias[2][3];
    double tc_scale[2][3];
    double t_ref_c;
    temp_cal_point_t points[10];
    int n_points;
} temp_cal_model_t;

typedef struct {
    double align_matrix[9];
    double soft_iron[9];
    double hard_iron[3];
} mag_calib_t;

void mems_cal_build_cross_matrix(double *cross, double *matrix_out);
int mems_cal_lstsq(double *A, int m, int n, double *b, double *x);
int mems_cal_accel_6pos(double pos[6][3], int32_t raw[6][3], accel_calib_t *out);
int mems_cal_gyro_rate_table(double rates[], int32_t raw[][3], int n_rates, gyro_calib_t *out);
int mems_cal_temp_model_fit(temp_cal_point_t *pts, int n, temp_cal_model_t *out);
void mems_cal_temp_apply_accel(temp_cal_model_t *m, double T, accel_calib_t *base, accel_calib_t *out);
void mems_cal_temp_apply_gyro(temp_cal_model_t *m, double T, gyro_calib_t *base, gyro_calib_t *out);
int mems_cal_auto_bias(double samples[][3], int n, double bias_out[3]);
double mems_cal_gravity_magnitude_check(double ax, double ay, double az);
int mems_cal_mag_ellipsoid_fit(double mag_samples[][3], int n, mag_calib_t *out);
void mems_cal_mag_apply(mag_calib_t *cal, double mx, double my, double mz, double *ox, double *oy, double *oz);

#endif
