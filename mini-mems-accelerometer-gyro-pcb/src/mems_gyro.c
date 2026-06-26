#include "mems_gyro.h"
#include <stdlib.h>
#include <string.h>

#define KB 1.380649e-23

void mems_gyro_model_init(mems_gyro_model_t *m, gyro_fullscale_t fs, gyro_architecture_t arch)
{
    if (!m) return;
    memset(m, 0, sizeof(*m));
    m->fullscale = fs;
    m->arch = arch;
    m->drive_mass_kg = 1.0e-9;
    m->sense_mass_kg = 0.8e-9;
    m->drive_stiffness = 400.0;
    m->sense_stiffness = 390.0;
    m->drive_q = 50000.0;
    m->sense_q = 45000.0;
    m->drive_freq_hz = 15000.0;
    m->sense_freq_hz = 14800.0;
    m->coriolis_coupling = 0.02;
    m->angular_gain = 1.0;
    m->sensitivity_lsb_per_dps = (double)((1 << 15) - 1) / (double)fs;
    m->noise_density_dps_per_sqrt_hz = 0.005;
    m->bandwidth_hz = 100.0;
    m->bias_instability_dps_per_h = 1.5;
    m->angle_random_walk_dps_per_sqrt_h = 0.03;
}

double mems_gyro_coriolis_force(double mass, double velocity, double angular_rate)
{
    return 2.0 * mass * velocity * angular_rate;
}

double mems_gyro_coriolis_displacement(mems_gyro_model_t *m, double angular_rate)
{
    if (!m) return 0.0;
    double vel = 2.0 * M_PI * m->drive_freq_hz * 1.0e-6;
    double f_cor = 2.0 * m->drive_mass_kg * vel * angular_rate;
    double k_s = m->sense_stiffness;
    if (k_s <= 0.0) return 0.0;
    double delta_f = m->sense_freq_hz - m->drive_freq_hz;
    double q_eff = m->sense_q;
    double denom = k_s * sqrt(1.0 + (2.0 * q_eff * delta_f / m->drive_freq_hz) * (2.0 * q_eff * delta_f / m->drive_freq_hz));
    if (denom < 1e-30) return 0.0;
    return f_cor / denom;
}

void mems_gyro_mode_transfer(mems_gyro_model_t *m, double f, double *mag, double *ph)
{
    if (!m || !mag || !ph) return;
    double fd = m->drive_freq_hz;
    double fs = m->sense_freq_hz;
    double w = 2.0 * M_PI * f;
    double wd = 2.0 * M_PI * fd;
    double ws = 2.0 * M_PI * fs;
    double Qs = m->sense_q;
    double num = 2.0 * m->coriolis_coupling * wd;
    double den_re = (ws*ws - w*w);
    double den_im = w*ws / Qs;
    double den_mag = sqrt(den_re*den_re + den_im*den_im);
    if (den_mag < 1e-30) { *mag = 0.0; *ph = 0.0; return; }
    *mag = num / den_mag;
    *ph = atan2(-den_im, den_re);
}

double mems_gyro_angle_random_walk(mems_gyro_model_t *m, double T)
{
    if (!m || m->drive_q <= 0.0) return 0.0;
    (void)m->bandwidth_hz;
    double kbT = KB * T;
    double force_noise = sqrt(4.0 * kbT * m->drive_stiffness / (2.0 * M_PI * m->drive_freq_hz * m->drive_q));
    double arw = force_noise / (2.0 * m->drive_mass_kg * m->drive_freq_hz * 2.0 * M_PI);
    return arw * 180.0 / M_PI;
}

double mems_gyro_bias_instability(double *sample_means, int n_clusters, double dt)
{
    if (!sample_means || n_clusters <= 1 || dt <= 0.0) return 0.0;
    double mean = 0.0;
    int i;
    for (i = 0; i < n_clusters; i++) mean += sample_means[i];
    mean /= (double)n_clusters;
    double variance = 0.0;
    for (i = 0; i < n_clusters; i++) {
        double d = sample_means[i] - mean;
        variance += d * d;
    }
    variance /= (double)(n_clusters - 1);
    return sqrt(variance);
}

double mems_gyro_total_noise(mems_gyro_model_t *m, double vn, double T)
{
    if (!m) return 0.0;
    double arw = mems_gyro_angle_random_walk(m, T);
    double elec = vn * 1e-6;
    return sqrt(arw*arw + elec*elec);
}

double mems_gyro_allan_variance(double *data, int n, double tau_idx, double dt)
{
    if (!data || n < 4) return 0.0;
    int tau = (int)tau_idx;
    if (tau < 1 || tau > n/3) return 0.0;
    int m = n - 2*tau;
    if (m <= 0) return 0.0;
    double sum = 0.0;
    int k;
    for (k = 0; k < m; k++) {
        double avg1 = 0.0, avg2 = 0.0;
        int j;
        for (j = k; j < k + tau; j++) avg1 += data[j];
        for (j = k + tau; j < k + 2*tau; j++) avg2 += data[j];
        avg1 /= (double)tau;
        avg2 /= (double)tau;
        double diff = avg2 - avg1;
        sum += diff * diff;
    }
    return sum / (2.0 * (double)m) / (dt * dt);
}

int mems_gyro_identify_noise_sources(double *av, int n_taus, double *arw, double *bi, double *rrw)
{
    if (!av || n_taus < 3 || !arw || !bi || !rrw) return -1;
    *arw = av[0] * 0.664;
    double min_val = av[0];
    int i;
    for (i = 1; i < n_taus; i++) {
        if (av[i] < min_val) min_val = av[i];
    }
    *bi = min_val / 0.664;
    *rrw = av[n_taus-1] * 0.664;
    return 0;
}

int mems_gyro_static_calibrate(double rates[][3], int32_t cnts[][3], int n, gyro_calib_t *out)
{
    if (!rates || !cnts || !out || n < 3) return -1;
    memset(out, 0, sizeof(*out));
    double sr[3] = {0}, se[3] = {0}, srs[3] = {0}, sre[3] = {0};
    int i, ax;
    for (i = 0; i < n; i++) {
        for (ax = 0; ax < 3; ax++) {
            double rv = (double)cnts[i][ax];
            double ev = rates[i][ax];
            sr[ax] += rv; se[ax] += ev;
            srs[ax] += rv * rv; sre[ax] += rv * ev;
        }
    }
    double nn = (double)n;
    for (ax = 0; ax < 3; ax++) {
        double det = srs[ax]*nn - sr[ax]*sr[ax];
        if (fabs(det) < 1e-15) return -1;
        double sc = (sre[ax]*nn - se[ax]*sr[ax]) / det;
        double bi = (srs[ax]*se[ax] - sr[ax]*sre[ax]) / det;
        if (ax == 0) { out->scale_x = sc; out->bias_x_dps = -bi / sc; }
        if (ax == 1) { out->scale_y = sc; out->bias_y_dps = -bi / sc; }
        if (ax == 2) { out->scale_z = sc; out->bias_z_dps = -bi / sc; }
    }
    return 0;
}

void mems_gyro_apply_calib(gyro_calib_t *cal, int32_t rx, int32_t ry, int32_t rz, double *cx, double *cy, double *cz)
{
    if (!cal || !cx || !cy || !cz) return;
    *cx = cal->scale_x * ((double)rx - cal->bias_x_dps);
    *cy = cal->scale_y * ((double)ry - cal->bias_y_dps);
    *cz = cal->scale_z * ((double)rz - cal->bias_z_dps);
}

void mems_gyro_temp_compensate(gyro_calib_t *cal, double T, double T0, double tb[2], double ts[2], gyro_calib_t *out)
{
    if (!cal || !out || !tb || !ts) return;
    double dT = T - T0, dT2 = dT * dT;
    *out = *cal;
    out->bias_x_dps += tb[0]*dT + tb[1]*dT2;
    out->bias_y_dps += tb[0]*dT + tb[1]*dT2;
    out->bias_z_dps += tb[0]*dT + tb[1]*dT2;
    double sf = 1.0 + ts[0]*dT*1e-6 + ts[1]*dT2*1e-6;
    out->scale_x *= sf; out->scale_y *= sf; out->scale_z *= sf;
}

double mems_gyro_integrate_angle(double rate_dps, double dt_s)
{
    return rate_dps * dt_s;
}

void mems_gyro_euler_integrate(double rates[3], double dt, double angles[3])
{
    if (!rates || !angles) return;
    angles[0] += rates[0] * dt;
    angles[1] += rates[1] * dt;
    angles[2] += rates[2] * dt;
}

void mems_gyro_heading_update(double *heading_deg, double rate_z_dps, double dt_s)
{
    if (!heading_deg) return;
    *heading_deg += rate_z_dps * dt_s;
}

double mems_gyro_turn_rate(double rates[3])
{
    if (!rates) return 0.0;
    return sqrt(rates[0]*rates[0] + rates[1]*rates[1] + rates[2]*rates[2]);
}

/* L5: PLL-based drive loop model for gyro */
double mems_gyro_pll_phase_error(double drive_freq, double ref_freq, double K_vco)
{
    double freq_err = drive_freq - ref_freq;
    return freq_err / K_vco;
}

double mems_gyro_agc_target(double measured_amplitude, double target_amplitude, double K_agc)
{
    return (target_amplitude - measured_amplitude) * K_agc;
}

/* L5: Quadrature error compensation */
double mems_gyro_quadrature_compensation(double quad_signal, double phase_offset_deg)
{
    double rad = phase_offset_deg * M_PI / 180.0;
    return quad_signal * cos(rad);
}

/* L6: Angular rate to angle via trapezoidal integration */
double mems_gyro_trapezoidal_integrate(double rate_prev, double rate_curr, double dt)
{
    return 0.5 * (rate_prev + rate_curr) * dt;
}

/* L6: Gyro zero-rate level detection for stationary detection */
int mems_gyro_stationary_detect(double rates[3], double threshold_dps, int consecutive_samples)
{
    if (!rates) return 0;
    static int count = 0;
    double mag = sqrt(rates[0]*rates[0] + rates[1]*rates[1] + rates[2]*rates[2]);
    if (mag < threshold_dps) {
        count++;
        if (count >= consecutive_samples) {
            count = consecutive_samples;
            return 1;
        }
        return 0;
    }
    count = 0;
    return 0;
}

/* L6: Scale factor nonlinearity compensation (3rd order) */
double mems_gyro_nonlinearity_comp(double rate, double sf2, double sf3)
{
    return rate + sf2*rate*rate + sf3*rate*rate*rate;
}

/* L7: Gyro self-test excitation */
int mems_gyro_selftest(double *output_dps, double expected_dps, double tolerance_pct)
{
    if (!output_dps) return -1;
    double error_pct = fabs(*output_dps - expected_dps) / fabs(expected_dps) * 100.0;
    return (error_pct <= tolerance_pct) ? 0 : -1;
}

/* L7: Temperature-dependent bias drift model */
double mems_gyro_bias_drift_model(double T, double T0, double tc1, double tc2, double tc3)
{
    double dT = T - T0;
    return tc1*dT + tc2*dT*dT + tc3*dT*dT*dT;
}

/* L8: Rate-integrating gyro (RIG) mode transfer function */
void mems_gyro_rig_transfer(mems_gyro_model_t *m, double w, double *real, double *imag)
{
    if (!m || !real || !imag) return;
    double wd = 2.0 * M_PI * m->drive_freq_hz;
    double Q = m->drive_q;
    *real = wd*wd / (wd*wd + w*w);
    *imag = -wd*w / (Q * (wd*wd + w*w));
}

/* L8: G-sensitivity compensation for gyro */
double mems_gyro_g_sensitivity_comp(double rate_dps, double ax, double ay, double az,
                                    double gsens_x, double gsens_y, double gsens_z)
{
    double correction = gsens_x*ax + gsens_y*ay + gsens_z*az;
    return rate_dps - correction;
}

/* L8: Scale factor vs angular rate nonlinearity */
double mems_gyro_scale_factor_nonlinearity(double rate_dps, double sf0, double sf2_ppm, double sf3_ppm)
{
    double r_norm = rate_dps / 100.0;
    return sf0 * (1.0 + sf2_ppm*1e-6*r_norm*r_norm + sf3_ppm*1e-6*r_norm*r_norm*r_norm);
}

/* L8: Rate random walk estimation from Allan variance slope */
double mems_gyro_rate_random_walk(double allan_at_10s, double dt)
{
    if (dt <= 0.0) return 0.0;
    return allan_at_10s / sqrt(10.0 / dt);
}

/* L7: Angular rate zero-mean normalization */
void mems_gyro_zero_mean(double *rates, int n, double *bias_out, double *corrected)
{
    if (!rates || !bias_out || !corrected || n <= 0) return;
    double sum = 0.0;
    int i;
    for (i = 0; i < n; i++) sum += rates[i];
    *bias_out = sum / (double)n;
    for (i = 0; i < n; i++) corrected[i] = rates[i] - *bias_out;
}

/* L7: Gyro bandwidth estimation from step response */
double mems_gyro_bandwidth_from_risetime(double t_rise_10_90_s)
{
    if (t_rise_10_90_s <= 0.0) return 0.0;
    return 0.35 / t_rise_10_90_s;
}

/* L8: Dual-mass tuning fork gyro mode matching */
double mems_gyro_mode_matching_error(double f_drive, double f_sense)
{
    if (f_drive <= 0.0) return 0.0;
    return fabs(f_drive - f_sense) / f_drive * 100.0;
}

/* L8: Electrostatic spring softening effect */
double mems_gyro_electrostatic_spring(double V_dc, double C0, double g0_m)
{
    if (g0_m <= 0.0) return 0.0;
    return -C0 * V_dc * V_dc / (g0_m * g0_m);
}

/* L8: Scale factor temperature hysteresis model */
double mems_gyro_sf_hysteresis(double T, double T_prev, double hyst_coeff_ppm_per_C)
{
    return hyst_coeff_ppm_per_C * fabs(T - T_prev) * 1e-6;
}

/* L8: Gyro startup bias transient (exponential decay model) */
double mems_gyro_startup_bias(double time_s, double initial_bias_dps, double tau_s, double steady_state_bias_dps)
{
    if (tau_s <= 0.0) return steady_state_bias_dps;
    return steady_state_bias_dps + (initial_bias_dps - steady_state_bias_dps) * exp(-time_s / tau_s);
}

/* L7: Long-term bias repeatability analysis */
typedef struct {
    double biases[100];
    int num_measurements;
    double mean;
    double std_dev;
    double max_deviation;
} bias_repeatability_t;

void mems_gyro_bias_repeatability_update(bias_repeatability_t *br, double new_bias)
{
    if (!br || br->num_measurements >= 100) return;
    br->biases[br->num_measurements++] = new_bias;
    int i;
    double sum = 0.0;
    for (i = 0; i < br->num_measurements; i++) sum += br->biases[i];
    br->mean = sum / (double)br->num_measurements;
    double var = 0.0;
    for (i = 0; i < br->num_measurements; i++) {
        double d = br->biases[i] - br->mean;
        var += d * d;
    }
    br->std_dev = sqrt(var / (double)(br->num_measurements > 1 ? br->num_measurements - 1 : 1));
    br->max_deviation = 0.0;
    for (i = 0; i < br->num_measurements; i++) {
        double dev = fabs(br->biases[i] - br->mean);
        if (dev > br->max_deviation) br->max_deviation = dev;
    }
}

/* L7: Digital output data rate selection */
double mems_gyro_odr_selection(double mechanical_bw_hz, int odr_options[], int num_options, int *best_odr_idx)
{
    if (!odr_options || !best_odr_idx || num_options <= 0) return -1.0;
    double target_odr = mechanical_bw_hz * 4.0;
    int best_idx = 0;
    double best_diff = fabs((double)odr_options[0] - target_odr);
    int i;
    for (i = 1; i < num_options; i++) {
        double diff = fabs((double)odr_options[i] - target_odr);
        if (diff < best_diff) { best_diff = diff; best_idx = i; }
    }
    *best_odr_idx = best_idx;
    return (double)odr_options[best_idx];
}

/* L8: Mechanical cross-coupling between axes */
void mems_gyro_cross_coupling_matrix(double coupling_xy, double coupling_xz,
                                     double coupling_yx, double coupling_yz,
                                     double coupling_zx, double coupling_zy,
                                     double matrix[9])
{
    if (!matrix) return;
    matrix[0]=1.0; matrix[1]=coupling_xy; matrix[2]=coupling_xz;
    matrix[3]=coupling_yx; matrix[4]=1.0; matrix[5]=coupling_yz;
    matrix[6]=coupling_zx; matrix[7]=coupling_zy; matrix[8]=1.0;
}

void mems_gyro_apply_cross_coupling(double matrix[9], double rx, double ry, double rz,
                                    double *ox, double *oy, double *oz)
{
    if (!matrix || !ox || !oy || !oz) return;
    *ox = matrix[0]*rx + matrix[1]*ry + matrix[2]*rz;
    *oy = matrix[3]*rx + matrix[4]*ry + matrix[5]*rz;
    *oz = matrix[6]*rx + matrix[7]*ry + matrix[8]*rz;
}
