#include "mems_accel.h"
#include <stdlib.h>
#include <string.h>

#define G_STD 9.80665
#define KB 1.380649e-23

void mems_accel_model_init(mems_accel_model_t *m, accel_fullscale_t fs, accel_operating_mode_t mode)
{
    if (!m) return;
    memset(m, 0, sizeof(*m));
    m->fullscale = fs;
    m->op_mode = mode;
    m->proof_mass_kg = 2.0e-9;
    m->spring_constant = 8.0;
    m->damping_coeff = 1.2e-6;
    m->sense_capacitance = 2.0e-12;
    m->capacitive_gap_um = 2.0;
    m->resonance_freq_hz = 10000.0;
    m->quality_factor = 100.0;
    m->damping_ratio = 1.0 / (2.0 * m->quality_factor);
    m->sensitivity_v_g = 0.3;
    m->sensitivity_lsb_g = (double)((1 << 15) - 1) / (double)fs;
    m->noise_density_ug_per_sqrt_hz = 100.0;
    m->bandwidth_hz = 200.0;
    m->damping_type = ACCEL_DAMPING_VISCOUS;
}

double mems_accel_delta_c(mems_accel_model_t *m, double a)
{
    if (!m || m->spring_constant <= 0.0) return 0.0;
    double wn2 = m->spring_constant / m->proof_mass_kg;
    if (wn2 <= 0.0) return 0.0;
    return m->sense_capacitance * a / (m->capacitive_gap_um * 1e-6 * wn2);
}

double mems_accel_displacement(mems_accel_model_t *m, double a)
{
    if (!m || m->spring_constant <= 0.0) return 0.0;
    return m->proof_mass_kg * a / m->spring_constant;
}

int mems_accel_pullin_check(mems_accel_model_t *m, double x)
{
    if (!m || m->capacitive_gap_um <= 0.0) return 0;
    double gap_m = m->capacitive_gap_um * 1e-6;
    double pullin_limit = gap_m / 3.0;
    return (fabs(x) > pullin_limit) ? 1 : 0;
}

void mems_accel_transfer_fn(mems_accel_model_t *m, double sr, double si, double *hr, double *hi)
{
    if (!m || !hr || !hi) return;
    double wn2 = m->spring_constant / m->proof_mass_kg;
    double wn = sqrt(wn2);
    double z = m->damping_ratio;
    double den_re = wn2 - sr*sr + si*si + 2.0*z*wn*sr;
    double den_im = -2.0*sr*si - 2.0*z*wn*si;
    double den_m2 = den_re*den_re + den_im*den_im;
    if (den_m2 < 1e-30) { *hr = *hi = 0.0; return; }
    *hr = wn2 * den_re / den_m2;
    *hi = -wn2 * den_im / den_m2;
}

void mems_accel_freq_response(mems_accel_model_t *m, double f, double *mag, double *ph)
{
    if (!m || !mag || !ph) return;
    double wn2 = m->spring_constant / m->proof_mass_kg;
    double wn = sqrt(wn2);
    double z = m->damping_ratio;
    double w = 2.0 * M_PI * f;
    double w2 = w * w;
    double den = sqrt((wn2 - w2)*(wn2 - w2) + 4.0*z*z*wn2*w2);
    if (den < 1e-30) { *mag = 1.0; *ph = 0.0; return; }
    *mag = wn2 / den;
    *ph = atan2(-2.0*z*wn*w, wn2 - w2);
}

double mems_accel_step_response(mems_accel_model_t *m, double t)
{
    if (!m || t < 0.0) return 0.0;
    double wn2 = m->spring_constant / m->proof_mass_kg;
    double wn = sqrt(wn2);
    double z = m->damping_ratio;
    double wd = wn * sqrt(1.0 - z*z);
    double env = exp(-z * wn * t);
    double factor = z / sqrt(1.0 - z*z);
    return 1.0 - env * (cos(wd * t) + factor * sin(wd * t));
}

double mems_accel_settling_time(mems_accel_model_t *m, double tol)
{
    if (!m || tol <= 0.0 || tol >= 1.0) return 0.0;
    double wn = sqrt(m->spring_constant / m->proof_mass_kg);
    double z = m->damping_ratio;
    if (z <= 0.0) return HUGE_VAL;
    return -log(tol) / (z * wn);
}

double mems_accel_brownian_noise(mems_accel_model_t *m, double T)
{
    if (!m || T <= 0.0 || m->proof_mass_kg <= 0.0) return 0.0;
    double force_noise = sqrt(4.0 * KB * T * m->damping_coeff);
    return force_noise / m->proof_mass_kg;
}

double mems_accel_total_noise(mems_accel_model_t *m, double vn, double T)
{
    if (!m) return 0.0;
    double nea_b = mems_accel_brownian_noise(m, T);
    double sens = m->sensitivity_v_g * G_STD;
    if (sens <= 0.0) return nea_b;
    double nea_e = vn * m->sense_capacitance / sens;
    return sqrt(nea_b*nea_b + nea_e*nea_e);
}

double mems_accel_resolution(double nea, double bw)
{
    if (nea < 0.0 || bw < 0.0) return -1.0;
    return nea * sqrt(bw > 0.0 ? bw : 1.0);
}

double mems_accel_dynamic_range_db(double fs, double res)
{
    if (fs <= 0.0 || res <= 0.0) return 0.0;
    return 20.0 * log10(fs / res);
}
static int solve_2x2(double a, double b, double c, double d, double e, double f, double *x, double *y)
{
    double det = a*d - b*c;
    if (fabs(det) < 1e-15) return -1;
    *x = (e*d - b*f) / det;
    *y = (a*f - e*c) / det;
    return 0;
}

int mems_accel_six_pos_calibrate(double eg[][3], int32_t mc[][3], int n, accel_calib_t *out)
{
    if (!eg || !mc || !out || n < 3) return -1;
    memset(out, 0, sizeof(*out));
    double sr[3] = {0}, se[3] = {0}, srs[3] = {0}, sre[3] = {0};
    int i, ax;
    for (i = 0; i < n; i++) {
        for (ax = 0; ax < 3; ax++) {
            double rv = (double)mc[i][ax];
            double ev = eg[i][ax];
            sr[ax] += rv; se[ax] += ev;
            srs[ax] += rv * rv; sre[ax] += rv * ev;
        }
    }
    double nn = (double)n;
    for (ax = 0; ax < 3; ax++) {
        double sc, bi;
        if (solve_2x2(srs[ax], sr[ax], sr[ax], nn, sre[ax], se[ax], &sc, &bi) != 0)
            return -1;
        if (ax == 0) { out->scale_x = sc; out->bias_x_mg = -bi / sc * 1000.0; }
        if (ax == 1) { out->scale_y = sc; out->bias_y_mg = -bi / sc * 1000.0; }
        if (ax == 2) { out->scale_z = sc; out->bias_z_mg = -bi / sc * 1000.0; }
    }
    return 0;
}

void mems_accel_apply_calib(accel_calib_t *cal, int32_t rx, int32_t ry, int32_t rz,
                            double *cx, double *cy, double *cz)
{
    if (!cal || !cx || !cy || !cz) return;
    *cx = cal->scale_x * ((double)rx - cal->bias_x_mg)
        + cal->cross_xy * (double)ry + cal->cross_xz * (double)rz;
    *cy = cal->scale_y * ((double)ry - cal->bias_y_mg)
        + cal->cross_yx * (double)rx + cal->cross_yz * (double)rz;
    *cz = cal->scale_z * ((double)rz - cal->bias_z_mg)
        + cal->cross_zx * (double)rx + cal->cross_zy * (double)ry;
}

void mems_accel_temp_compensate(accel_calib_t *cal, double T, double T0,
                                double tb[2], double ts[2], accel_calib_t *out)
{
    if (!cal || !out || !tb || !ts) return;
    double dT = T - T0;
    double dT2 = dT * dT;
    *out = *cal;
    out->bias_x_mg += tb[0]*dT + tb[1]*dT2;
    out->bias_y_mg += tb[0]*dT + tb[1]*dT2;
    out->bias_z_mg += tb[0]*dT + tb[1]*dT2;
    double sf = 1.0 + ts[0]*dT*1e-6 + ts[1]*dT2*1e-6;
    out->scale_x *= sf; out->scale_y *= sf; out->scale_z *= sf;
}

int mems_accel_tilt_angles(double ax, double ay, double az, double *p, double *r)
{
    if (!p || !r) return 0;
    double mag = sqrt(ax*ax + ay*ay + az*az);
    if (mag < 0.01) return 0;
    ax /= mag; ay /= mag; az /= mag;
    *p = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / M_PI;
    *r = atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / M_PI;
    return (fabs(mag - 1.0) < 0.3) ? 1 : 0;
}

double mems_accel_magnitude(double ax, double ay, double az)
{
    return sqrt(ax*ax + ay*ay + az*az);
}

int mems_accel_freefall_detect(double ax, double ay, double az, double th)
{
    return (sqrt(ax*ax + ay*ay + az*az) < th) ? 1 : 0;
}

int mems_accel_tap_detect(double ax, double ay, double az, double th)
{
    return (fabs(sqrt(ax*ax + ay*ay + az*az) - 1.0) > th) ? 1 : 0;
}

double mems_accel_rms(double *s, size_t n)
{
    if (!s || n == 0) return 0.0;
    double sum_sq = 0.0;
    size_t i;
    for (i = 0; i < n; i++) sum_sq += s[i] * s[i];
    return sqrt(sum_sq / (double)n);
}

double mems_accel_peak_to_peak(double *s, size_t n)
{
    if (!s || n == 0) return 0.0;
    double vmin = s[0], vmax = s[0];
    size_t i;
    for (i = 1; i < n; i++) {
        if (s[i] < vmin) vmin = s[i];
        if (s[i] > vmax) vmax = s[i];
    }
    return vmax - vmin;
}

double mems_accel_crest_factor(double *s, size_t n)
{
    if (!s || n == 0) return -1.0;
    double rms_val = mems_accel_rms(s, n);
    if (rms_val < 1e-15) return -1.0;
    double peak = 0.0;
    size_t i;
    for (i = 0; i < n; i++) {
        double av = fabs(s[i]);
        if (av > peak) peak = av;
    }
    return peak / rms_val;
}

/* L6: Acceleration-to-velocity integration (trapezoidal) */
double mems_accel_integrate_velocity(double v0, double a_prev, double a_curr, double dt)
{
    return v0 + 0.5 * (a_prev + a_curr) * dt;
}

/* L6: Acceleration-to-position double integration */
double mems_accel_integrate_position(double p0, double v0, double a_prev, double a_curr, double dt)
{
    double v = v0 + 0.5*(a_prev + a_curr)*dt;
    return p0 + 0.5*(v0 + v)*dt;
}

/* L6: Coordinate frame transformation (body to NED) */
void mems_accel_body_to_ned(double ax, double ay, double az,
                            double roll, double pitch, double yaw,
                            double *ax_ned, double *ay_ned, double *az_ned)
{
    if (!ax_ned || !ay_ned || !az_ned) return;
    double cr=cos(roll), sr=sin(roll);
    double cp=cos(pitch), sp=sin(pitch);
    double cy=cos(yaw), sy=sin(yaw);
    double R[9] = {
        cp*cy, cp*sy, -sp,
        sr*sp*cy-cr*sy, sr*sp*sy+cr*cy, sr*cp,
        cr*sp*cy+sr*sy, cr*sp*sy-sr*cy, cr*cp
    };
    *ax_ned = R[0]*ax + R[1]*ay + R[2]*az;
    *ay_ned = R[3]*ax + R[4]*ay + R[5]*az;
    *az_ned = R[6]*ax + R[7]*ay + R[8]*az;
}

/* L7: Significant motion detection */
int mems_accel_significant_motion(double *buffer, int buf_len, double threshold_mg, int min_count)
{
    if (!buffer || buf_len < 1) return 0;
    int count = 0, i;
    for (i = 0; i < buf_len; i++) {
        if (fabs(buffer[i]) > threshold_mg) count++;
    }
    return (count >= min_count) ? 1 : 0;
}

/* L7: Stationary detection via variance */
int mems_accel_stationary_detect(double *buffer, int buf_len, double var_threshold)
{
    if (!buffer || buf_len < 2) return 0;
    double mean = 0.0, var = 0.0;
    int i;
    for (i = 0; i < buf_len; i++) mean += buffer[i];
    mean /= (double)buf_len;
    for (i = 0; i < buf_len; i++) { double d = buffer[i] - mean; var += d*d; }
    var /= (double)(buf_len - 1);
    return (var < var_threshold) ? 1 : 0;
}

/* L7: Shock event detection (ISO 1413 shock resistance) */
int mems_accel_shock_detect(double peak_accel_g, double duration_ms, double peak_threshold_g, double dur_threshold_ms)
{
    if (fabs(peak_accel_g) > peak_threshold_g && duration_ms < dur_threshold_ms) return 1;
    return 0;
}

/* L8: Closed-loop force-rebalance model */
double mems_accel_force_rebalance(mems_accel_model_t *m, double displacement, double loop_gain)
{
    if (!m) return 0.0;
    return m->spring_constant * displacement * loop_gain;
}

/* L8: Damping ratio from logarithmic decrement */
double mems_accel_log_decrement_zeta(double x1, double x2, int n_cycles)
{
    if (n_cycles <= 0 || x1 <= 0.0 || x2 <= 0.0) return -1.0;
    double delta = log(x1 / x2) / (double)n_cycles;
    return delta / sqrt(4.0*M_PI*M_PI + delta*delta);
}

/* L6: Orientation tracking with gyro-assisted tilt */
void mems_accel_gyro_tilt(double ax, double ay, double az,
                          double gx, double gy, double gz,
                          double dt, double alpha,
                          double *pitch, double *roll)
{
    if (!pitch || !roll) return;
    double accel_pitch, accel_roll;
    int valid = mems_accel_tilt_angles(ax, ay, az, &accel_pitch, &accel_roll);
    if (valid) {
        *pitch = alpha * (*pitch + gx * dt) + (1.0 - alpha) * accel_pitch;
        *roll  = alpha * (*roll  + gy * dt) + (1.0 - alpha) * accel_roll;
    } else {
        *pitch += gx * dt;
        *roll  += gy * dt;
    }
}

/* L7: Activity classification from accelerometer features */
typedef enum {
    ACTIVITY_STATIONARY = 0,
    ACTIVITY_WALKING = 1,
    ACTIVITY_RUNNING = 2,
    ACTIVITY_CYCLING = 3,
    ACTIVITY_VEHICLE = 4,
    ACTIVITY_UNKNOWN = 5
} activity_class_t;

int mems_accel_classify_activity(double *samples, int n, double fs_hz, activity_class_t *result)
{
    if (!samples || n < 10 || !result) return -1;
    double rms = mems_accel_rms(samples, (size_t)n);
    double pp = mems_accel_peak_to_peak(samples, (size_t)n);
    double cf = mems_accel_crest_factor(samples, (size_t)n);
    if (rms < 0.05) *result = ACTIVITY_STATIONARY;
    else if (cf > 6.0) *result = ACTIVITY_VEHICLE;
    else if (pp > 3.0) *result = ACTIVITY_RUNNING;
    else if (rms > 0.3) *result = ACTIVITY_WALKING;
    else if (rms < 0.15 && cf < 3.0) *result = ACTIVITY_CYCLING;
    else *result = ACTIVITY_UNKNOWN;
    return 0;
}

/* L7: Inclination accuracy under vibration */
double mems_accel_inclination_error(double vibration_rms_g, double noise_density_ugrtHz, double bw)
{
    if (bw <= 0.0) return 0.0;
    double total_noise = sqrt(vibration_rms_g*vibration_rms_g + (noise_density_ugrtHz*1e-6)*(noise_density_ugrtHz*1e-6)*bw);
    return atan(total_noise) * 180.0 / M_PI;
}

/* L8: MEMS accelerometer scale factor temperature modeling (3rd order) */
void mems_accel_advanced_temp_model(mems_accel_model_t *m, double T_c, double T_ref,
                                    double tc_sf[3], double *sf_corrected)
{
    if (!m || !sf_corrected) return;
    double dT = T_c - T_ref;
    *sf_corrected = 1.0 + tc_sf[0]*dT + tc_sf[1]*dT*dT + tc_sf[2]*dT*dT*dT;
}

/* L8: Vibration rectification error (VRE) model */
double mems_accel_vre(double vibration_amplitude_g, double vibration_freq_hz, double vre_coeff_ug_per_g2)
{
    double g2 = vibration_amplitude_g * vibration_amplitude_g;
    return vre_coeff_ug_per_g2 * g2;
}

/* L8: Offset drift from package stress */
double mems_accel_package_stress_drift(double delta_T, double pcb_cte_ppm, double mems_cte_ppm, double stiffness_N_per_m)
{
    double strain = (pcb_cte_ppm - mems_cte_ppm) * 1e-6 * delta_T;
    return strain * 1e6 / stiffness_N_per_m;
}

/* L8: Sensitivity matrix from linear acceleration */
void mems_accel_sensitivity_matrix(double ax, double ay, double az,
                                   double *S_xx, double *S_yy, double *S_zz,
                                   double *S_xy, double *S_xz, double *S_yz)
{
    if (!S_xx || !S_yy || !S_zz) return;
    double g = 9.80665;
    double ax_g = ax / g, ay_g = ay / g, az_g = az / g;
    *S_xx = fabs(ax_g) > 0.01 ? ax / ax_g : 1.0;
    *S_yy = fabs(ay_g) > 0.01 ? ay / ay_g : 1.0;
    *S_zz = fabs(az_g) > 0.01 ? az / az_g : 1.0;
    if (S_xy) *S_xy = (fabs(ax_g) > 0.01 && fabs(ay_g) > 0.01) ? (ax/ax_g)/(ay/ay_g) : 0.0;
    if (S_xz) *S_xz = (fabs(ax_g) > 0.01 && fabs(az_g) > 0.01) ? (ax/ax_g)/(az/az_g) : 0.0;
    if (S_yz) *S_yz = (fabs(ay_g) > 0.01 && fabs(az_g) > 0.01) ? (ay/ay_g)/(az/az_g) : 0.0;
}

/* L8: Allan variance for accelerometer (static bias stability) */
double mems_accel_allan_variance(double *data, int n, int cluster_size)
{
    if (!data || n < 4 || cluster_size < 1) return -1.0;
    int m = n / cluster_size;
    if (m < 2) return -1.0;
    double *clusters = (double *)calloc((size_t)m, sizeof(double));
    if (!clusters) return -1.0;
    int i, j;
    for (i = 0; i < m; i++) {
        double sum = 0.0;
        for (j = 0; j < cluster_size; j++) sum += data[i*cluster_size + j];
        clusters[i] = sum / (double)cluster_size;
    }
    double avar = 0.0;
    for (i = 0; i < m - 1; i++) {
        double diff = clusters[i+1] - clusters[i];
        avar += diff * diff;
    }
    avar /= (2.0 * (double)(m - 1));
    free(clusters);
    return sqrt(avar);
}

/* L8: Frequency response from impulse response via FFT (simplified) */
void mems_accel_impulse_to_freq(double *impulse, int n, double dt, double *freq, double *mag, int n_freq)
{
    if (!impulse || !freq || !mag || n < 2 || n_freq < 1 || dt <= 0.0) return;
    int k;
    for (k = 0; k < n_freq; k++) {
        double f = freq[k];
        double real = 0.0, imag = 0.0;
        int i;
        for (i = 0; i < n; i++) {
            double angle = -2.0 * M_PI * f * (double)i * dt;
            real += impulse[i] * cos(angle);
            imag += impulse[i] * sin(angle);
        }
        mag[k] = sqrt(real*real + imag*imag) * dt;
    }
}

/* L8: Overload recovery time estimation */
double mems_accel_overload_recovery(double overload_g, double resonance_hz, double Q)
{
    if (resonance_hz <= 0.0 || Q <= 0.0) return 0.0;
    double wn = 2.0 * M_PI * resonance_hz;
    double zeta = 1.0 / (2.0 * Q);
    double tau = 1.0 / (zeta * wn);
    return 5.0 * tau;
}
