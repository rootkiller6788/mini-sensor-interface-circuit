#include "mems_calibration.h"
#include <stdlib.h>
#include <string.h>

void mems_cal_build_cross_matrix(double *cross, double *matrix_out)
{
    if (!cross || !matrix_out) return;
    matrix_out[0]=1.0; matrix_out[1]=cross[0]; matrix_out[2]=cross[1];
    matrix_out[3]=cross[2]; matrix_out[4]=1.0; matrix_out[5]=cross[3];
    matrix_out[6]=cross[4]; matrix_out[7]=cross[5]; matrix_out[8]=1.0;
}

int mems_cal_lstsq(double *A, int m, int n, double *b, double *x)
{
    if (!A || !b || !x || m < n || n <= 0) return -1;
    double *ATA = (double *)calloc((size_t)(n * n), sizeof(double));
    double *ATb = (double *)calloc((size_t)n, sizeof(double));
    if (!ATA || !ATb) { free(ATA); free(ATb); return -1; }
    int i, j, k;
    for (i=0; i<m; i++) {
        for (j=0; j<n; j++) {
            ATb[j] += A[i*n+j] * b[i];
            for (k=0; k<n; k++) ATA[j*n+k] += A[i*n+j] * A[i*n+k];
        }
    }
    for (i=0; i<n; i++) {
        double pivot = ATA[i*n+i];
        if (fabs(pivot) < 1e-15) { free(ATA); free(ATb); return -1; }
        for (j=i+1; j<n; j++) {
            double factor = ATA[j*n+i] / pivot;
            for (k=i; k<n; k++) ATA[j*n+k] -= factor * ATA[i*n+k];
            ATb[j] -= factor * ATb[i];
        }
    }
    for (i=n-1; i>=0; i--) {
        double sum = ATb[i];
        for (j=i+1; j<n; j++) sum -= ATA[i*n+j] * x[j];
        x[i] = sum / ATA[i*n+i];
    }
    free(ATA); free(ATb);
    return 0;
}

int mems_cal_accel_6pos(double pos[6][3], int32_t raw[6][3], accel_calib_t *out)
{
    if (!pos || !raw || !out) return -1;
    memset(out, 0, sizeof(*out));
    double *A = (double *)calloc(36, sizeof(double));
    double *b = (double *)calloc(6, sizeof(double));
    double *x = (double *)calloc(6, sizeof(double));
    if (!A || !b || !x) { free(A); free(b); free(x); return -1; }
    int i;
    for (i=0; i<6; i++) {
        A[i*6+0]=(double)raw[i][0]; A[i*6+1]=(double)raw[i][1];
        A[i*6+2]=(double)raw[i][2]; A[i*6+3]=1.0; A[i*6+4]=0.0; A[i*6+5]=0.0;
        b[i]=pos[i][0];
    }
    if (mems_cal_lstsq(A,6,6,b,x)!=0) { free(A); free(b); free(x); return -1; }
    out->scale_x=x[0]; out->cross_xy=x[1]; out->cross_xz=x[2]; out->bias_x_mg=x[3];
    for (i=0; i<6; i++) b[i]=pos[i][1];
    if (mems_cal_lstsq(A,6,6,b,x)!=0) { free(A); free(b); free(x); return -1; }
    out->cross_yx=x[0]; out->scale_y=x[1]; out->cross_yz=x[2]; out->bias_y_mg=x[3];
    for (i=0; i<6; i++) b[i]=pos[i][2];
    if (mems_cal_lstsq(A,6,6,b,x)!=0) { free(A); free(b); free(x); return -1; }
    out->cross_zx=x[0]; out->cross_zy=x[1]; out->scale_z=x[2]; out->bias_z_mg=x[3];
    free(A); free(b); free(x);
    return 0;
}

int mems_cal_gyro_rate_table(double rates[], int32_t raw[][3], int n_rates, gyro_calib_t *out)
{
    if (!rates || !raw || !out || n_rates < 2) return -1;
    memset(out, 0, sizeof(*out));
    int axis;
    for (axis = 0; axis < 3; axis++) {
        double sx=0, sy=0, sxx=0, sxy=0;
        int i;
        for (i = 0; i < n_rates; i++) {
            double rv = (double)raw[i][axis];
            double ev = rates[i];
            sx += rv; sy += ev; sxx += rv*rv; sxy += rv*ev;
        }
        double nn = (double)n_rates;
        double det = sxx*nn - sx*sx;
        if (fabs(det) < 1e-15) return -1;
        double sc = (sxy*nn - sy*sx) / det;
        double bi = (sxx*sy - sx*sxy) / det;
        if (axis == 0) { out->scale_x = sc; out->bias_x_dps = -bi/sc; }
        if (axis == 1) { out->scale_y = sc; out->bias_y_dps = -bi/sc; }
        if (axis == 2) { out->scale_z = sc; out->bias_z_dps = -bi/sc; }
    }
    return 0;
}

int mems_cal_temp_model_fit(temp_cal_point_t *pts, int n, temp_cal_model_t *out)
{
    if (!pts || !out || n < 3) return -1;
    memset(out, 0, sizeof(*out));
    out->t_ref_c = pts[0].temp_c;
    out->n_points = (n > 10) ? 10 : n;
    memcpy(out->points, pts, (size_t)out->n_points * sizeof(temp_cal_point_t));
    int i;
    double sx=0, sy=0, sx2=0, sx3=0, sx4=0, sxy=0, sx2y=0;
    for (i = 0; i < out->n_points; i++) {
        double t = pts[i].temp_c - out->t_ref_c;
        double t2 = t * t;
        double y = pts[i].bias_x;
        sx += t; sy += y; sx2 += t2; sx3 += t*t2; sx4 += t2*t2;
        sxy += t*y; sx2y += t2*y;
    }
    double denom = sx2*sx4 - sx3*sx3;
    if (fabs(denom) > 1e-15) {
        out->tc_bias[0][0] = (sxy*sx4 - sx2y*sx3) / denom;
        out->tc_bias[1][0] = (sx2*sx2y - sxy*sx3) / denom;
    }
    return 0;
}

void mems_cal_temp_apply_accel(temp_cal_model_t *m, double T, accel_calib_t *base, accel_calib_t *out)
{
    if (!m || !base || !out) return;
    double dT = T - m->t_ref_c, dT2 = dT * dT;
    *out = *base;
    out->bias_x_mg += m->tc_bias[0][0]*dT + m->tc_bias[1][0]*dT2;
    out->bias_y_mg += m->tc_bias[0][1]*dT + m->tc_bias[1][1]*dT2;
    out->bias_z_mg += m->tc_bias[0][2]*dT + m->tc_bias[1][2]*dT2;
    double sfx = 1.0 + m->tc_scale[0][0]*dT*1e-6 + m->tc_scale[1][0]*dT2*1e-6;
    double sfy = 1.0 + m->tc_scale[0][1]*dT*1e-6 + m->tc_scale[1][1]*dT2*1e-6;
    double sfz = 1.0 + m->tc_scale[0][2]*dT*1e-6 + m->tc_scale[1][2]*dT2*1e-6;
    out->scale_x *= sfx; out->scale_y *= sfy; out->scale_z *= sfz;
}

void mems_cal_temp_apply_gyro(temp_cal_model_t *m, double T, gyro_calib_t *base, gyro_calib_t *out)
{
    if (!m || !base || !out) return;
    double dT = T - m->t_ref_c, dT2 = dT * dT;
    *out = *base;
    out->bias_x_dps += m->tc_bias[0][0]*dT + m->tc_bias[1][0]*dT2;
    out->bias_y_dps += m->tc_bias[0][1]*dT + m->tc_bias[1][1]*dT2;
    out->bias_z_dps += m->tc_bias[0][2]*dT + m->tc_bias[1][2]*dT2;
    double sf = 1.0 + m->tc_scale[0][0]*dT*1e-6 + m->tc_scale[1][0]*dT2*1e-6;
    out->scale_x *= sf; out->scale_y *= sf; out->scale_z *= sf;
}

int mems_cal_auto_bias(double samples[][3], int n, double bias_out[3])
{
    if (!samples || !bias_out || n < 1) return -1;
    bias_out[0]=bias_out[1]=bias_out[2]=0.0;
    int i;
    for (i=0; i<n; i++) {
        bias_out[0]+=samples[i][0]; bias_out[1]+=samples[i][1]; bias_out[2]+=samples[i][2];
    }
    double inv=1.0/(double)n;
    bias_out[0]*=inv; bias_out[1]*=inv; bias_out[2]*=inv;
    return 0;
}

double mems_cal_gravity_magnitude_check(double ax, double ay, double az)
{
    return sqrt(ax*ax + ay*ay + az*az);
}

int mems_cal_mag_ellipsoid_fit(double mag_samples[][3], int n, mag_calib_t *out)
{
    if (!mag_samples || !out || n < 10) return -1;
    memset(out, 0, sizeof(*out));
    double mx_min=mag_samples[0][0], mx_max=mag_samples[0][0];
    double my_min=mag_samples[0][1], my_max=mag_samples[0][1];
    double mz_min=mag_samples[0][2], mz_max=mag_samples[0][2];
    double sx=0, sy=0, sz=0;
    int i;
    for (i=0; i<n; i++) {
        double mx=mag_samples[i][0], my=mag_samples[i][1], mz=mag_samples[i][2];
        sx+=mx; sy+=my; sz+=mz;
        { if (mx<mx_min) mx_min=mx; } { if (mx>mx_max) mx_max=mx; }
        { if (my<my_min) my_min=my; } { if (my>my_max) my_max=my; }
        { if (mz<mz_min) mz_min=mz; } { if (mz>mz_max) mz_max=mz; }
    }
    double inv=1.0/(double)n;
    out->hard_iron[0]=sx*inv; out->hard_iron[1]=sy*inv; out->hard_iron[2]=sz*inv;
    out->soft_iron[0]=2.0/(mx_max-mx_min+1e-9);
    out->soft_iron[4]=2.0/(my_max-my_min+1e-9);
    out->soft_iron[8]=2.0/(mz_max-mz_min+1e-9);
    out->align_matrix[0]=1.0; out->align_matrix[4]=1.0; out->align_matrix[8]=1.0;
    return 0;
}

void mems_cal_mag_apply(mag_calib_t *cal, double mx, double my, double mz, double *ox, double *oy, double *oz)
{
    if (!cal || !ox || !oy || !oz) return;
    double dx=mx-cal->hard_iron[0], dy=my-cal->hard_iron[1], dz=mz-cal->hard_iron[2];
    *ox=cal->soft_iron[0]*dx+cal->soft_iron[1]*dy+cal->soft_iron[2]*dz;
    *oy=cal->soft_iron[3]*dx+cal->soft_iron[4]*dy+cal->soft_iron[5]*dz;
    *oz=cal->soft_iron[6]*dx+cal->soft_iron[7]*dy+cal->soft_iron[8]*dz;
}

void mems_cal_ema_bias(double new_sample[3], double bias[3], double alpha)
{
    if (!new_sample || !bias) return;
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    bias[0] = alpha*new_sample[0] + (1.0-alpha)*bias[0];
    bias[1] = alpha*new_sample[1] + (1.0-alpha)*bias[1];
    bias[2] = alpha*new_sample[2] + (1.0-alpha)*bias[2];
}

int mems_cal_gravity_cal(double samples[][3], int n, double gravity_mag, accel_calib_t *out)
{
    if (!samples || !out || n < 4 || gravity_mag <= 0.0) return -1;
    double sum[3]={0,0,0}; int i;
    for(i=0;i<n;i++){sum[0]+=samples[i][0];sum[1]+=samples[i][1];sum[2]+=samples[i][2];}
    double inv=1.0/(double)n;
    out->bias_x_mg=sum[0]*inv; out->bias_y_mg=sum[1]*inv; out->bias_z_mg=sum[2]*inv-gravity_mag;
    out->scale_x=1.0; out->scale_y=1.0; out->scale_z=1.0;
    return 0;
}

int mems_cal_iterative_refine(double pos[][3], int32_t raw[][3], int n, accel_calib_t *out, int iterations)
{
    if (!pos || !raw || !out || n < 3 || iterations < 1) return -1;
    accel_calib_t current;
    mems_cal_accel_6pos(pos, raw, out);
    int iter, i;
    for(iter=1; iter<iterations; iter++){
        current=*out;
        double residual=0.0;
        for(i=0;i<n;i++){
            double cx,cy,cz;
            mems_accel_apply_calib(&current,raw[i][0],raw[i][1],raw[i][2],&cx,&cy,&cz);
            double dx=cx-pos[i][0],dy=cy-pos[i][1],dz=cz-pos[i][2];
            residual+=dx*dx+dy*dy+dz*dz;
        }
        if(residual<1e-9) break;
    }
    return 0;
}

int mems_cal_cross_axis_svd(double M[4], double *scale1, double *scale2, double *angle)
{
    if(!M||!scale1||!scale2||!angle)return -1;
    double a=M[0],b=M[1],c=M[2],d=M[3];
    double trace=a+d, det=a*d-b*c;
    double disc=sqrt(fabs(trace*trace-4.0*det));
    *scale1=(trace+disc)/2.0; *scale2=(trace-disc)/2.0;
    *angle=atan2(c,a)*180.0/M_PI;
    return 0;
}

/* L8: Robust calibration using RANSAC (simplified) */
int mems_cal_ransac_3d(double samples[][3], int n, int max_iterations,
                       double inlier_threshold, double bias_out[3])
{
    if (!samples || !bias_out || n < 3 || max_iterations < 1) return -1;
    int best_inliers = 0;
    double best_bias[3] = {0,0,0};
    int iter;
    for (iter = 0; iter < max_iterations; iter++) {
        int idx1 = iter % n;
        int idx2 = (iter * 3 + 1) % n;
        if (idx2 == idx1) idx2 = (idx2 + 1) % n;
        double candidate[3] = {samples[idx1][0], samples[idx1][1], samples[idx1][2]};
        int inliers = 0, i;
        for (i = 0; i < n; i++) {
            double dx = samples[i][0] - candidate[0];
            double dy = samples[i][1] - candidate[1];
            double dz = samples[i][2] - candidate[2];
            if (sqrt(dx*dx+dy*dy+dz*dz) < inlier_threshold) inliers++;
        }
        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_bias[0] = candidate[0]; best_bias[1] = candidate[1]; best_bias[2] = candidate[2];
        }
    }
    bias_out[0]=best_bias[0]; bias_out[1]=best_bias[1]; bias_out[2]=best_bias[2];
    return best_inliers;
}

/* L8: Calibration quality metrics */
typedef struct {
    double rms_error;
    double max_error;
    double correlation_r2;
    int num_outliers;
} cal_quality_t;

void mems_cal_quality_assess(double expected[][3], double measured[][3], int n, cal_quality_t *q)
{
    if (!expected || !measured || !q || n <= 0) return;
    memset(q, 0, sizeof(*q));
    double sum_sq = 0.0, sum_exp = 0.0, sum_meas = 0.0, sum_exp2 = 0.0, sum_meas2 = 0.0, sum_em = 0.0;
    int i;
    for (i = 0; i < n; i++) {
        double dx = measured[i][0] - expected[i][0];
        double dy = measured[i][1] - expected[i][1];
        double dz = measured[i][2] - expected[i][2];
        double error = sqrt(dx*dx + dy*dy + dz*dz);
        sum_sq += error * error;
        if (error > q->max_error) q->max_error = error;
        double em = expected[i][0], mm = measured[i][0];
        sum_exp += em; sum_meas += mm; sum_exp2 += em*em; sum_meas2 += mm*mm; sum_em += em*mm;
    }
    q->rms_error = sqrt(sum_sq / (double)n);
    double denom_r2 = (n*sum_exp2 - sum_exp*sum_exp) * (n*sum_meas2 - sum_meas*sum_meas);
    if (denom_r2 > 1e-15) {
        q->correlation_r2 = (n*sum_em - sum_exp*sum_meas) * (n*sum_em - sum_exp*sum_meas) / denom_r2;
    }
}

/* L8: Kalman filter based bias tracking */
typedef struct {
    double bias[3];
    double P[9];
    double Q;
    double R;
} bias_kalman_t;

void mems_cal_bias_kalman_init(bias_kalman_t *bk, double process_noise, double measurement_noise)
{
    if (!bk) return;
    memset(bk, 0, sizeof(*bk));
    bk->Q = process_noise;
    bk->R = measurement_noise;
    bk->P[0]=bk->P[4]=bk->P[8]=1.0;
}

void mems_cal_bias_kalman_update(bias_kalman_t *bk, double measurement[3])
{
    if (!bk || !measurement) return;
    int i;
    for (i = 0; i < 3; i++) {
        double p = bk->P[i*3+i] + bk->Q;
        double k = p / (p + bk->R);
        bk->bias[i] += k * (measurement[i] - bk->bias[i]);
        bk->P[i*3+i] = (1.0 - k) * p;
    }
}

/* L8: IMU systematic error model (scale + misalignment + bias) */
typedef struct {
    double scale[3];
    double misalignment[9];
    double bias[3];
    double g_sensitivity[9];
} imu_error_model_t;

void imu_error_model_apply(imu_error_model_t *em, double raw[3], double gravity[3], double corrected[3])
{
    if (!em || !raw || !corrected) return;
    int i, j;
    for (i = 0; i < 3; i++) {
        corrected[i] = raw[i] - em->bias[i];
        for (j = 0; j < 3; j++) {
            corrected[i] += em->g_sensitivity[i*3+j] * gravity[j];
        }
        double aligned = 0.0;
        for (j = 0; j < 3; j++) {
            aligned += em->misalignment[i*3+j] * corrected[j];
        }
        corrected[i] = em->scale[i] * aligned;
    }
}

void imu_error_model_init_default(imu_error_model_t *em)
{
    if (!em) return;
    memset(em, 0, sizeof(*em));
    em->scale[0] = em->scale[1] = em->scale[2] = 1.0;
    em->misalignment[0] = em->misalignment[4] = em->misalignment[8] = 1.0;
}
