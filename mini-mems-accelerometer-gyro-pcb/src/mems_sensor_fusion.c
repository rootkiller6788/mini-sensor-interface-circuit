#include "mems_sensor_fusion.h"
#include <stdlib.h>
#include <string.h>

/* L2: Euler angles to quaternion */
quaternion_t quat_from_euler(double roll, double pitch, double yaw)
{
    quaternion_t q;
    double cr = cos(roll * 0.5 * M_PI / 180.0);
    double sr = sin(roll * 0.5 * M_PI / 180.0);
    double cp = cos(pitch * 0.5 * M_PI / 180.0);
    double sp = sin(pitch * 0.5 * M_PI / 180.0);
    double cy = cos(yaw * 0.5 * M_PI / 180.0);
    double sy = sin(yaw * 0.5 * M_PI / 180.0);
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;
    return q;
}

/* L2: Quaternion to Euler angles */
euler_angles_t quat_to_euler(quaternion_t q)
{
    euler_angles_t e;
    double sinr_cosp = 2.0*(q.w*q.x + q.y*q.z);
    double cosr_cosp = 1.0 - 2.0*(q.x*q.x + q.y*q.y);
    e.roll_deg = atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;
    double sinp = 2.0*(q.w*q.y - q.z*q.x);
    if (fabs(sinp) >= 1.0) e.pitch_deg = copysign(90.0, sinp);
    else e.pitch_deg = asin(sinp) * 180.0 / M_PI;
    double siny_cosp = 2.0*(q.w*q.z + q.x*q.y);
    double cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z);
    e.yaw_deg = atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
    return e;
}

/* L2: Hamilton quaternion multiplication */
quaternion_t quat_multiply(quaternion_t a, quaternion_t b)
{
    quaternion_t q;
    q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return q;
}

/* L2: Quaternion conjugate */
quaternion_t quat_conjugate(quaternion_t q)
{
    quaternion_t r;
    r.w = q.w; r.x = -q.x; r.y = -q.y; r.z = -q.z;
    return r;
}

/* L2: Normalize quaternion */
void quat_normalize(quaternion_t *q)
{
    if (!q) return;
    double n = sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (n < 1e-15) { q->w = 1.0; q->x = q->y = q->z = 0.0; return; }
    double inv = 1.0 / n;
    q->w *= inv; q->x *= inv; q->y *= inv; q->z *= inv;
}

/* L2: Quaternion from axis-angle representation */
quaternion_t quat_from_axis_angle(double ax, double ay, double az, double angle)
{
    quaternion_t q;
    double n = sqrt(ax*ax + ay*ay + az*az);
    if (n < 1e-15) { q.w = 1.0; q.x = q.y = q.z = 0.0; return q; }
    double inv = 1.0 / n;
    ax *= inv; ay *= inv; az *= inv;
    double half = angle * 0.5;
    double s = sin(half);
    q.w = cos(half);
    q.x = ax * s;
    q.y = ay * s;
    q.z = az * s;
    return q;
}

/* L3: Quaternion to rotation matrix */
void quat_to_rotation_matrix(quaternion_t q, double R[9])
{
    if (!R) return;
    double w=q.w, x=q.x, y=q.y, z=q.z;
    double ww=w*w, xx=x*x, yy=y*y, zz=z*z;
    R[0] = ww+xx-yy-zz; R[1] = 2.0*(x*y-w*z); R[2] = 2.0*(x*z+w*y);
    R[3] = 2.0*(x*y+w*z); R[4] = ww-xx+yy-zz; R[5] = 2.0*(y*z-w*x);
    R[6] = 2.0*(x*z-w*y); R[7] = 2.0*(y*z+w*x); R[8] = ww-xx-yy+zz;
}

/* L3: Rotation matrix to quaternion */
void rotation_matrix_to_quat(double R[9], quaternion_t *q)
{
    if (!R || !q) return;
    double trace = R[0] + R[4] + R[8];
    if (trace > 0.0) {
        double s = sqrt(trace + 1.0) * 2.0;
        q->w = 0.25 * s;
        q->x = (R[7] - R[5]) / s;
        q->y = (R[2] - R[6]) / s;
        q->z = (R[3] - R[1]) / s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        double s = sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0;
        q->w = (R[7] - R[5]) / s;
        q->x = 0.25 * s;
        q->y = (R[1] + R[3]) / s;
        q->z = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        double s = sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0;
        q->w = (R[2] - R[6]) / s;
        q->x = (R[1] + R[3]) / s;
        q->y = 0.25 * s;
        q->z = (R[5] + R[7]) / s;
    } else {
        double s = sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0;
        q->w = (R[3] - R[1]) / s;
        q->x = (R[2] + R[6]) / s;
        q->y = (R[5] + R[7]) / s;
        q->z = 0.25 * s;
    }
}

/* L3: Rotate vector by quaternion */
void vec_rotate_by_quat(double v[3], quaternion_t q, double out[3])
{
    if (!v || !out) return;
    quaternion_t vq = {0.0, v[0], v[1], v[2]};
    quaternion_t qc = quat_conjugate(q);
    quaternion_t t = quat_multiply(q, vq);
    quaternion_t r = quat_multiply(t, qc);
    out[0] = r.x; out[1] = r.y; out[2] = r.z;
}

/* L5: Complementary filter for 1D angle estimation */
double complementary_filter(double accel_angle, double gyro_rate, double dt, double alpha)
{
    static double angle = 0.0;
    if (dt <= 0.0 || alpha < 0.0 || alpha > 1.0) return angle;
    angle = alpha * (angle + gyro_rate * dt) + (1.0 - alpha) * accel_angle;
    return angle;
}

/* L5: Mahony explicit complementary filter for attitude estimation */
void mahony_filter_update(double ax, double ay, double az,
                          double gx, double gy, double gz,
                          double mx, double my, double mz,
                          mahony_params_t *params, quaternion_t *q)
{
    if (!params || !q) return;
    double norm = sqrt(ax*ax + ay*ay + az*az);
    if (norm < 1e-6) return;
    double inv_norm = 1.0 / norm;
    ax *= inv_norm; ay *= inv_norm; az *= inv_norm;
    double vx = 2.0*(q->x*q->z - q->w*q->y);
    double vy = 2.0*(q->w*q->x + q->y*q->z);
    double vz = q->w*q->w - q->x*q->x - q->y*q->y + q->z*q->z;
    double ex = ay*vz - az*vy;
    double ey = az*vx - ax*vz;
    double ez = ax*vy - ay*vx;
    if (mx != 0.0 || my != 0.0 || mz != 0.0) {
        norm = sqrt(mx*mx + my*my + mz*mz);
        if (norm > 1e-6) {
            inv_norm = 1.0 / norm;
            mx *= inv_norm; my *= inv_norm; mz *= inv_norm;
            double hx = mx*(q->w*q->w+q->x*q->x-q->y*q->y-q->z*q->z)
                      + my*2.0*(q->x*q->y-q->w*q->z) + mz*2.0*(q->x*q->z+q->w*q->y);
            double hy = mx*2.0*(q->x*q->y+q->w*q->z)
                      + my*(q->w*q->w-q->x*q->x+q->y*q->y-q->z*q->z) + mz*2.0*(q->y*q->z-q->w*q->x);
            double bx = sqrt(hx*hx + hy*hy);
            double bz = mx*2.0*(q->x*q->z-q->w*q->y) + my*2.0*(q->y*q->z+q->w*q->x)
                      + mz*(q->w*q->w-q->x*q->x-q->y*q->y+q->z*q->z);
            double wx = 2.0*bx*(0.5 - q->y*q->y - q->z*q->z) + 2.0*bz*(q->x*q->z - q->w*q->y);
            double wy = 2.0*bx*(q->x*q->y - q->w*q->z) + 2.0*bz*(q->w*q->x + q->y*q->z);
            double wz = 2.0*bx*(q->w*q->y + q->x*q->z) + 2.0*bz*(0.5 - q->x*q->x - q->y*q->y);
            ex += (mx*wx + my*wy + mz*wz); ey += (mx*wy - my*wx); ez += (mx*wz - mz*wx);
        }
    }
    params->integral_fb_x += params->ki * ex * params->dt;
    params->integral_fb_y += params->ki * ey * params->dt;
    params->integral_fb_z += params->ki * ez * params->dt;
    gx += params->kp*ex + params->integral_fb_x;
    gy += params->kp*ey + params->integral_fb_y;
    gz += params->kp*ez + params->integral_fb_z;
    double qw=q->w, qx=q->x, qy=q->y, qz=q->z;
    double half_dt = 0.5 * params->dt;
    q->w += half_dt * (-gx*qx - gy*qy - gz*qz);
    q->x += half_dt * ( gx*qw + gz*qy - gy*qz);
    q->y += half_dt * ( gy*qw - gz*qx + gx*qz);
    q->z += half_dt * ( gz*qw + gy*qx - gx*qy);
    quat_normalize(q);
}

/* L5: Madgwick gradient descent orientation filter */
void madgwick_filter_update(double ax, double ay, double az,
                            double gx, double gy, double gz,
                            madgwick_params_t *params, quaternion_t *q)
{
    if (!params || !q) return;
    double norm = sqrt(ax*ax + ay*ay + az*az);
    if (norm < 1e-6) return;
    double inv_norm = 1.0 / norm;
    ax *= inv_norm; ay *= inv_norm; az *= inv_norm;
    double qw=q->w, qx=q->x, qy=q->y, qz=q->z;
    double f1 = 2.0*(qx*qz - qw*qy) - ax;
    double f2 = 2.0*(qw*qx + qy*qz) - ay;
    double f3 = 2.0*(0.5 - qx*qx - qy*qy) - az;
    double J11 = -2.0*qy; double J12 =  2.0*qz; double J13 = -2.0*qw; double J14 = 2.0*qx;
    double J21 =  2.0*qx; double J22 =  2.0*qw; double J23 =  2.0*qz; double J24 = 2.0*qy;
    double J31 =  2.0*qw; double J32 = -4.0*qx; double J33 = -4.0*qy; double J34 = 0.0;
    double grad_w = J11*f1 + J21*f2 + J31*f3;
    double grad_x = J12*f1 + J22*f2 + J32*f3;
    double grad_y = J13*f1 + J23*f2 + J33*f3;
    double grad_z = J14*f1 + J24*f2 + J34*f3;
    double grad_norm = sqrt(grad_w*grad_w + grad_x*grad_x + grad_y*grad_y + grad_z*grad_z);
    if (grad_norm < 1e-6) return;
    double step = params->beta * params->dt;
    double inv_grad = step / grad_norm;
    q->w = qw - grad_w * inv_grad;
    q->x = qx - grad_x * inv_grad;
    q->y = qy - grad_y * inv_grad;
    q->z = qz - grad_z * inv_grad;
    double dqw = 0.5*(-gx*qx - gy*qy - gz*qz);
    double dqx = 0.5*( gx*qw + gz*qy - gy*qz);
    double dqy = 0.5*( gy*qw - gz*qx + gx*qz);
    double dqz = 0.5*( gz*qw + gy*qx - gx*qy);
    q->w += dqw * params->dt;
    q->x += dqx * params->dt;
    q->y += dqy * params->dt;
    q->z += dqz * params->dt;
    quat_normalize(q);
}

/* L5: Kalman filter for IMU ¡ª initialization */
void kalman_imu_init(kalman_imu_params_t *k, double dt)
{
    if (!k) return;
    memset(k, 0, sizeof(*k));
    k->dt = dt;
    k->x[6] = 1.0;
    int i;
    for (i = 0; i < 4; i++) k->P[i*4+i] = 1.0;
    k->Q_diag[0] = k->Q_diag[1] = k->Q_diag[2] = 0.001;
    k->Q_diag[3] = 0.0001;
    k->R_diag[0] = k->R_diag[1] = k->R_diag[2] = 0.01;
    k->R_diag[3] = k->R_diag[4] = k->R_diag[5] = 0.01;
}

/* L5: Kalman filter predict step (gyro integration) */
void kalman_imu_predict(kalman_imu_params_t *k, double gx, double gy, double gz)
{
    if (!k) return;
    double dt=k->dt;
    double qw=k->x[3], qx=k->x[4], qy=k->x[5], qz=k->x[6];
    double half=0.5*dt;
    double dqw=half*(-gx*qx-gy*qy-gz*qz);
    double dqx=half*( gx*qw+gz*qy-gy*qz);
    double dqy=half*( gy*qw-gz*qx+gx*qz); (void)dqy;
    double dqz=half*( gz*qw+gy*qx-gx*qy);
    k->x[3]+=dqw; k->x[4]+=dqx; k->x[5]+=dqz; k->x[6]+=dqz;
    double norm=sqrt(k->x[3]*k->x[3]+k->x[4]*k->x[4]+k->x[5]*k->x[5]+k->x[6]*k->x[6]);
    if (norm>1e-8) { double inv=1.0/norm; k->x[3]*=inv; k->x[4]*=inv; k->x[5]*=inv; k->x[6]*=inv; }
    int i;
    for (i=0; i<7; i++) k->P[i*7+i]+=k->Q_diag[i<4?i:0];
}

/* L5: Kalman update from accelerometer measurement */
void kalman_imu_update_accel(kalman_imu_params_t *k, double ax, double ay, double az)
{
    if (!k) return;
    double norm=sqrt(ax*ax+ay*ay+az*az);
    if (norm<1e-6) return;
    ax/=norm; ay/=norm; az/=norm;
    double qw=k->x[3], qx=k->x[4], qy=k->x[5], qz=k->x[6];
    double hx=2.0*(qx*qz-qw*qy);
    double hy=2.0*(qw*qx+qy*qz);
    double hz=qw*qw-qx*qx-qy*qy+qz*qz;
    double z[3]={ax-hx, ay-hy, az-hz};
    double S[9]={0};
    S[0]=k->P[3*7+3]*0.01+k->R_diag[0]; S[4]=k->P[4*7+4]*0.01+k->R_diag[1]; S[8]=k->P[5*7+5]*0.01+k->R_diag[2];
    double K[3]={k->P[3*7+3]/S[0], k->P[4*7+4]/S[4], k->P[5*7+5]/S[8]};
    k->x[3]+=K[0]*z[0]; k->x[4]+=K[1]*z[1]; k->x[5]+=K[2]*z[2];
    k->P[3*7+3]*=(1.0-K[0]); k->P[4*7+4]*=(1.0-K[1]); k->P[5*7+5]*=(1.0-K[2]);
    quat_normalize((quaternion_t*)&k->x[3]);
}

void kalman_imu_update_mag(kalman_imu_params_t *k, double mx, double my, double mz)
{
    if (!k) return;
}

void kalman_imu_get_quat(kalman_imu_params_t *k, quaternion_t *q)
{
    if (!k || !q) return;
    q->w=k->x[3]; q->x=k->x[4]; q->y=k->x[5]; q->z=k->x[6];
}

/* L6: AHRS heading from magnetometer */
void ahrs_compute_heading(quaternion_t q, double mag_x, double mag_y, double mag_z, double *heading)
{
    if (!heading) return;
    double R[9];
    quat_to_rotation_matrix(q, R);
    double mx_rot = R[0]*mag_x + R[1]*mag_y + R[2]*mag_z;
    double my_rot = R[3]*mag_x + R[4]*mag_y + R[5]*mag_z;
    *heading = atan2(my_rot, mx_rot) * 180.0 / M_PI;
    if (*heading < 0.0) *heading += 360.0;
}

/* L6: Gravity vector in body frame from attitude */
void ahrs_gravity_vector(quaternion_t q, double g[3])
{
    if (!g) return;
    g[0] = 2.0*(q.x*q.z - q.w*q.y);
    g[1] = 2.0*(q.w*q.x + q.y*q.z);
    g[2] = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
}

double ahrs_vertical_accel(quaternion_t q, double ax, double ay, double az)
{
    double g[3];
    ahrs_gravity_vector(q, g);
    return ax*g[0] + ay*g[1] + az*g[2];
}

/* L7: Pedestrian dead reckoning ¡ª step detection */
double pdr_step_detect(double accel_mag, double threshold_g, double min_step_s)
{
    static double last_step_time = 0.0; (void)last_step_time;
    if (accel_mag > threshold_g) {
        last_step_time = 0.0;
        return 1.0;
    }
    return 0.0;
}

/* L7: Step length estimation (Weinberg model) */
double pdr_step_length(double step_freq, double accel_var, double K)
{
    if (step_freq <= 0.0) return 0.0;
    return K * pow(accel_var, 0.25) / step_freq;
}

/* L7: Position update from step and heading */
void pdr_position_update(double *x, double *y, double heading, double step_len)
{
    if (!x || !y) return;
    double heading_rad = heading * M_PI / 180.0;
    *x += step_len * cos(heading_rad);
    *y += step_len * sin(heading_rad);
}

/* L8: Gradient descent orientation from accel/mag */
void fusion_gd_orient(double ax, double ay, double az,
                      double mx, double my, double mz,
                      quaternion_t *q, double learning_rate, int iterations)
{
    if (!q) return;
    int iter;
    for (iter = 0; iter < iterations; iter++) {
        double norm_a = sqrt(ax*ax + ay*ay + az*az);
        if (norm_a < 1e-6) continue;
        double axn=ax/norm_a, ayn=ay/norm_a, azn=az/norm_a;
        double qw=q->w, qx=q->x, qy=q->y, qz=q->z;
        double f1=2.0*(qx*qz - qw*qy) - axn;
        double f2=2.0*(qw*qx + qy*qz) - ayn;
        double f3=2.0*(0.5 - qx*qx - qy*qy) - azn;
        double grad_w=-4.0*qy*f1 + 4.0*qx*f2 - 4.0*qx*f3;
        double grad_x= 4.0*qz*f1 + 4.0*qw*f2 - 8.0*qx*f3;
        double grad_y=-4.0*qw*f1 + 4.0*qz*f2 - 8.0*qy*f3;
        double grad_z= 4.0*qx*f1 + 4.0*qy*f2;
        q->w -= learning_rate * grad_w;
        q->x -= learning_rate * grad_x;
        q->y -= learning_rate * grad_y;
        q->z -= learning_rate * grad_z;
        quat_normalize(q);
    }
}

double fusion_wrap_angle_180(double angle_deg)
{
    angle_deg = fmod(angle_deg, 360.0);
    if (angle_deg > 180.0) angle_deg -= 360.0;
    if (angle_deg < -180.0) angle_deg += 360.0;
    return angle_deg;
}

double fusion_wrap_angle_360(double angle_deg)
{
    angle_deg = fmod(angle_deg, 360.0);
    if (angle_deg < 0.0) angle_deg += 360.0;
    return angle_deg;
}

quaternion_t quat_slerp(quaternion_t q0, quaternion_t q1, double t)
{
    quaternion_t result;
    if (t <= 0.0) return q0;
    if (t >= 1.0) return q1;
    double cos_theta = q0.w*q1.w + q0.x*q1.x + q0.y*q1.y + q0.z*q1.z;
    quaternion_t q1_adj = q1;
    if (cos_theta < 0.0) {
        q1_adj.w=-q1.w; q1_adj.x=-q1.x; q1_adj.y=-q1.y; q1_adj.z=-q1.z;
        cos_theta = -cos_theta;
    }
    double theta = acos(cos_theta);
    if (fabs(sin(theta)) < 1e-15) return q0;
    double st = sin(theta);
    double s0 = sin((1.0-t)*theta)/st;
    double s1 = sin(t*theta)/st;
    result.w=s0*q0.w+s1*q1_adj.w; result.x=s0*q0.x+s1*q1_adj.x;
    result.y=s0*q0.y+s1*q1_adj.y; result.z=s0*q0.z+s1*q1_adj.z;
    return result;
}

quaternion_t quat_from_two_vectors(double v1[3], double v2[3])
{
    quaternion_t q = {1.0, 0.0, 0.0, 0.0};
    if (!v1 || !v2) return q;
    double n1=sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2]);
    double n2=sqrt(v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2]);
    if (n1<1e-15||n2<1e-15) return q;
    double u1[3]={v1[0]/n1,v1[1]/n1,v1[2]/n1};
    double u2[3]={v2[0]/n2,v2[1]/n2,v2[2]/n2};
    double dot=u1[0]*u2[0]+u1[1]*u2[1]+u1[2]*u2[2];
    if(dot>0.999999)return q;
    if(dot<-0.999999){q.w=0.0;q.x=1.0;return q;}
    double cross[3]={u1[1]*u2[2]-u1[2]*u2[1],u1[2]*u2[0]-u1[0]*u2[2],u1[0]*u2[1]-u1[1]*u2[0]};
    q.w=1.0+dot; q.x=cross[0]; q.y=cross[1]; q.z=cross[2];
    quat_normalize(&q);
    return q;
}

void fusion_triad_attitude(double ax,double ay,double az,double mx,double my,double mz,quaternion_t *q)
{
    if(!q)return;
    double gn=sqrt(ax*ax+ay*ay+az*az), mn=sqrt(mx*mx+my*my+mz*mz);
    if(gn<1e-6||mn<1e-6)return;
    double g[3]={ax/gn,ay/gn,az/gn};
    double m[3]={mx/mn,my/mn,mz/mn};
    double me[3]={g[1]*m[2]-g[2]*m[1],g[2]*m[0]-g[0]*m[2],g[0]*m[1]-g[1]*m[0]};
    double en=sqrt(me[0]*me[0]+me[1]*me[1]+me[2]*me[2]);
    if(en<1e-6)return;
    me[0]/=en;me[1]/=en;me[2]/=en;
    double mn2[3]={me[1]*g[2]-me[2]*g[1],me[2]*g[0]-me[0]*g[2],me[0]*g[1]-me[1]*g[0]};
    double R[9]={mn2[0],me[0],-g[0],mn2[1],me[1],-g[1],mn2[2],me[2],-g[2]};
    rotation_matrix_to_quat(R,q);
}

double fusion_pressure_to_altitude(double pressure_hPa, double sea_level_hPa, double temp_c)
{
    if(pressure_hPa<=0.0||sea_level_hPa<=0.0)return 0.0;
    double temp_k=temp_c+273.15;
    double R=287.058,g=9.80665,L=0.0065;
    return(temp_k/L)*(1.0-pow(pressure_hPa/sea_level_hPa,R*L/g));
}

void fusion_linear_accel(quaternion_t q, double ax, double ay, double az, double lin[3])
{
    if(!lin)return;
    double g[3]; ahrs_gravity_vector(q,g);
    lin[0]=ax-g[0]; lin[1]=ay-g[1]; lin[2]=az-g[2];
}

quaternion_t quat_average(quaternion_t *quats, int n)
{
    quaternion_t result={1.0,0.0,0.0,0.0};
    if(!quats||n<=0)return result;
    double M[9]={0}; int k;
    for(k=0;k<n;k++){
        double qx=quats[k].x,qy=quats[k].y,qz=quats[k].z; (void)quats[k].w;
        M[0]+=qx*qx;M[1]+=qx*qy;M[2]+=qx*qz;M[3]+=qx*qy;M[4]+=qy*qy;M[5]+=qy*qz;M[6]+=qx*qz;M[7]+=qy*qz;M[8]+=qz*qz;
    }
    double inv=1.0/(double)n; int i; for(i=0;i<9;i++)M[i]*=inv;
    double eig1=M[0]+M[4]+M[8];
    double eig2=M[0]*M[4]+M[0]*M[8]+M[4]*M[8]-M[1]*M[3]-M[2]*M[6]-M[5]*M[7];
    double max_eig=(eig1+sqrt(fabs(eig1*eig1-4.0*eig2)))/2.0;
    result.w=sqrt(fabs(max_eig));
    if(result.w<1e-10){result.w=1.0;return result;}
    result.x=M[1]/result.w; result.y=M[2]/result.w; result.z=M[5]/result.w;
    quat_normalize(&result);
    return result;
}

/* L8: Gyro drift compensation using zero velocity updates */
typedef struct {
    double drift[3];
    double stationary_bias[3];
    int stationary_count;
    double confidence;
} zupt_state_t;

void zupt_init(zupt_state_t *z)
{
    if (!z) return;
    memset(z, 0, sizeof(*z));
    z->confidence = 0.0;
}

int zupt_detect(double accel_mag, double accel_var, double gyro_mag, double threshold)
{
    double metric = accel_var + gyro_mag * 0.1;
    return (fabs(accel_mag - 1.0) < 0.05 && metric < threshold) ? 1 : 0;
}

void zupt_update(zupt_state_t *z, double gyro[3], int is_stationary, double alpha)
{
    if (!z || !gyro) return;
    if (is_stationary) {
        z->stationary_count++;
        int i;
        for (i = 0; i < 3; i++) {
            z->stationary_bias[i] = alpha*gyro[i] + (1.0-alpha)*z->stationary_bias[i];
        }
        z->confidence = 1.0 - 1.0/(double)(z->stationary_count + 1);
    } else {
        z->stationary_count = 0;
        z->confidence *= 0.9;
    }
}

void zupt_correct(double gyro[3], zupt_state_t *z, double corrected[3])
{
    if (!gyro || !z || !corrected) return;
    corrected[0] = gyro[0] - z->stationary_bias[0] * z->confidence;
    corrected[1] = gyro[1] - z->stationary_bias[1] * z->confidence;
    corrected[2] = gyro[2] - z->stationary_bias[2] * z->confidence;
}

/* L8: MEMS sensor fusion with barometer for altitude hold */
typedef struct {
    double altitude;
    double climb_rate;
    double P[4];
    double Q_alt;
    double Q_climb;
    double R_alt;
} baro_kalman_t;

void baro_kalman_init(baro_kalman_t *bk, double alt, double q_alt, double q_climb, double r_alt)
{
    if (!bk) return;
    memset(bk, 0, sizeof(*bk));
    bk->altitude = alt;
    bk->Q_alt = q_alt;
    bk->Q_climb = q_climb;
    bk->R_alt = r_alt;
    bk->P[0] = bk->P[3] = 1.0;
}

void baro_kalman_predict(baro_kalman_t *bk, double vert_accel, double dt)
{
    if (!bk || dt <= 0.0) return;
    bk->altitude += bk->climb_rate * dt + 0.5 * vert_accel * dt * dt;
    bk->climb_rate += vert_accel * dt;
    bk->P[0] += bk->P[1]*dt + (bk->P[1]+bk->P[3]*dt)*dt + bk->Q_alt;
    bk->P[1] += bk->P[3]*dt;
    bk->P[2] += bk->P[3]*dt;
    bk->P[3] += bk->Q_climb;
}

void baro_kalman_update(baro_kalman_t *bk, double measured_alt)
{
    if (!bk) return;
    double S = bk->P[0] + bk->R_alt;
    if (S < 1e-15) return;
    double K0 = bk->P[0] / S;
    double K1 = bk->P[2] / S;
    bk->altitude += K0 * (measured_alt - bk->altitude);
    bk->climb_rate += K1 * (measured_alt - bk->altitude);
    bk->P[0] *= (1.0 - K0);
    bk->P[1] *= (1.0 - K0);
    bk->P[2] -= K1 * bk->P[0];
    bk->P[3] -= K1 * bk->P[2];
}
