#ifndef MEMS_ACCEL_H
#define MEMS_ACCEL_H
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef enum {
    ACCEL_DAMPING_VISCOUS = 0,
    ACCEL_DAMPING_STRUCTURAL = 1,
    ACCEL_DAMPING_THERMOELASTIC = 2,
    ACCEL_DAMPING_ANCHOR = 3
} accel_damping_type_t;

typedef enum {
    ACCEL_MODE_OPEN_LOOP = 0,
    ACCEL_MODE_CLOSED_LOOP = 1,
    ACCEL_MODE_AMPLITUDE_MOD = 2
} accel_operating_mode_t;

typedef enum {
    ACCEL_FS_2G = 2, ACCEL_FS_4G = 4, ACCEL_FS_8G = 8,
    ACCEL_FS_16G = 16, ACCEL_FS_200G = 200
} accel_fullscale_t;

typedef struct {
    double proof_mass_kg;
    double spring_constant;
    double damping_coeff;
    double sense_capacitance;
    double capacitive_gap_um;
    double resonance_freq_hz;
    double quality_factor;
    double damping_ratio;
    double sensitivity_v_g;
    double sensitivity_lsb_g;
    double noise_density_ug_per_sqrt_hz;
    double bandwidth_hz;
    double dynamic_range_db;
    accel_damping_type_t damping_type;
    accel_operating_mode_t op_mode;
    accel_fullscale_t fullscale;
} mems_accel_model_t;

typedef struct {
    int32_t raw_x, raw_y, raw_z;
    double accel_x_mg, accel_y_mg, accel_z_mg;
    double temperature_c;
    uint32_t timestamp_us;
} accel_sample_t;

typedef struct {
    double bias_x_mg, bias_y_mg, bias_z_mg;
    double scale_x, scale_y, scale_z;
    double cross_xy, cross_xz, cross_yx, cross_yz, cross_zx, cross_zy;
} accel_calib_t;

double mems_accel_delta_c(mems_accel_model_t *model, double accel_ms2);
double mems_accel_displacement(mems_accel_model_t *model, double accel_ms2);
int mems_accel_pullin_check(mems_accel_model_t *model, double displacement);
void mems_accel_transfer_fn(mems_accel_model_t *m, double sr, double si, double *hr, double *hi);
void mems_accel_freq_response(mems_accel_model_t *m, double f, double *mag, double *ph);
double mems_accel_step_response(mems_accel_model_t *m, double t);
double mems_accel_settling_time(mems_accel_model_t *m, double tol);
double mems_accel_brownian_noise(mems_accel_model_t *m, double T);
double mems_accel_total_noise(mems_accel_model_t *m, double vn, double T);
double mems_accel_resolution(double nea, double bw);
double mems_accel_dynamic_range_db(double fs, double res);
int mems_accel_six_pos_calibrate(double eg[][3], int32_t mc[][3], int n, accel_calib_t *out);
void mems_accel_apply_calib(accel_calib_t *cal, int32_t rx, int32_t ry, int32_t rz, double *cx, double *cy, double *cz);
void mems_accel_temp_compensate(accel_calib_t *cal, double T, double T0, double tb[2], double ts[2], accel_calib_t *out);
int mems_accel_tilt_angles(double ax, double ay, double az, double *p, double *r);
double mems_accel_magnitude(double ax, double ay, double az);
int mems_accel_freefall_detect(double ax, double ay, double az, double th);
int mems_accel_tap_detect(double ax, double ay, double az, double th);
double mems_accel_rms(double *s, size_t n);
double mems_accel_peak_to_peak(double *s, size_t n);
double mems_accel_crest_factor(double *s, size_t n);
void mems_accel_model_init(mems_accel_model_t *m, accel_fullscale_t fs, accel_operating_mode_t mode);

#endif /* MEMS_ACCEL_H */
