#ifndef MEMS_GYRO_H
#define MEMS_GYRO_H
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef enum {
    GYRO_MODE_TUNING_FORK = 0,
    GYRO_MODE_VIBRATING_RING = 1,
    GYRO_MODE_QUADRATURE_MASS = 2,
    GYRO_MODE_BAW_DISK = 3
} gyro_architecture_t;

typedef enum {
    GYRO_FS_125DPS = 125,
    GYRO_FS_250DPS = 250,
    GYRO_FS_500DPS = 500,
    GYRO_FS_1000DPS = 1000,
    GYRO_FS_2000DPS = 2000
} gyro_fullscale_t;

typedef struct {
    double drive_mass_kg;
    double sense_mass_kg;
    double drive_stiffness;
    double sense_stiffness;
    double drive_q;
    double sense_q;
    double drive_freq_hz;
    double sense_freq_hz;
    double coriolis_coupling;
    double angular_gain;
    double sensitivity_lsb_per_dps;
    double noise_density_dps_per_sqrt_hz;
    double bandwidth_hz;
    double bias_instability_dps_per_h;
    double angle_random_walk_dps_per_sqrt_h;
    gyro_architecture_t arch;
    gyro_fullscale_t fullscale;
} mems_gyro_model_t;

typedef struct {
    int32_t raw_x, raw_y, raw_z;
    double rate_x_dps, rate_y_dps, rate_z_dps;
    double temperature_c;
    uint32_t timestamp_us;
} gyro_sample_t;

typedef struct {
    double bias_x_dps, bias_y_dps, bias_z_dps;
    double scale_x, scale_y, scale_z;
    double g_sensitivity_xy, g_sensitivity_xz;
    double g_sensitivity_yx, g_sensitivity_yz;
    double g_sensitivity_zx, g_sensitivity_zy;
    double misalign_xy, misalign_xz;
    double misalign_yx, misalign_yz;
    double misalign_zx, misalign_zy;
} gyro_calib_t;

/* L2: Coriolis effect */
double mems_gyro_coriolis_force(double mass, double velocity, double angular_rate);
double mems_gyro_coriolis_displacement(mems_gyro_model_t *m, double angular_rate);

/* L3: Coupled-mode dynamics */
void mems_gyro_mode_transfer(mems_gyro_model_t *m, double f, double *mag, double *ph);

/* L4: Noise analysis */
double mems_gyro_angle_random_walk(mems_gyro_model_t *m, double T);
double mems_gyro_bias_instability(double *sample_means, int n_clusters, double dt);
double mems_gyro_total_noise(mems_gyro_model_t *m, double vn, double T);

/* L5: Allan variance */
double mems_gyro_allan_variance(double *data, int n, double tau_idx, double dt);
int mems_gyro_identify_noise_sources(double *allan_vals, int n_taus, double *arw, double *bi, double *rrw);

/* L5: Calibration */
int mems_gyro_static_calibrate(double rates[][3], int32_t cnts[][3], int n, gyro_calib_t *out);
void mems_gyro_apply_calib(gyro_calib_t *cal, int32_t rx, int32_t ry, int32_t rz, double *cx, double *cy, double *cz);
void mems_gyro_temp_compensate(gyro_calib_t *cal, double T, double T0, double tb[2], double ts[2], gyro_calib_t *out);

/* L6: Angular rate integration */
double mems_gyro_integrate_angle(double rate_dps, double dt_s);
void mems_gyro_euler_integrate(double rates[3], double dt, double angles[3]);

/* L7: Dead reckoning */
void mems_gyro_heading_update(double *heading_deg, double rate_z_dps, double dt_s);
double mems_gyro_turn_rate(double rates[3]);
void mems_gyro_model_init(mems_gyro_model_t *m, gyro_fullscale_t fs, gyro_architecture_t arch);

#endif
