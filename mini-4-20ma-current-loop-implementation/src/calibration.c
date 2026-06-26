/**
 * @file calibration.c
 * @brief 4-20mA loop calibration: linear regression, polynomial fit, spline
 *        interpolation, zero/span trim, temperature coefficient analysis.
 * Knowledge: L5 (calibration algorithms), L6 (transmitter calibration),
 *            L7 (industrial metrology), L3 (polynomial/NL models).
 */
#include "current_loop.h"
#include "calibration.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

void calibration_linear_regression(const double *x, const double *y,
    size_t n, double *slope, double *inter, double *r2)
{
    if (!x || !y || n < 2) {
        if (slope) *slope = 0.0;
        if (inter) *inter = 0.0;
        if (r2) *r2 = 0.0;
        return;
    }
    double sx = 0.0, sy = 0.0, sxy = 0.0, sx2 = 0.0, sy2 = 0.0;
    for (size_t i = 0; i < n; i++) {
        sx += x[i]; sy += y[i];
        sxy += x[i] * y[i];
        sx2 += x[i] * x[i];
        sy2 += y[i] * y[i];
    }
    double denom = n * sx2 - sx * sx;
    if (fabs(denom) < 1e-12) {
        if (slope) *slope = 0.0;
        if (inter) *inter = sy / n;
        if (r2) *r2 = 0.0;
        return;
    }
    double s = (n * sxy - sx * sy) / denom;
    double i = (sy - s * sx) / n;
    if (slope) *slope = s;
    if (inter) *inter = i;
    if (r2) {
        double ss_res = 0.0, ss_tot = 0.0;
        double mean_y = sy / n;
        for (size_t j = 0; j < n; j++) {
            double y_pred = s * x[j] + i;
            ss_res += (y[j] - y_pred) * (y[j] - y_pred);
            ss_tot += (y[j] - mean_y) * (y[j] - mean_y);
        }
        *r2 = (ss_tot > 1e-12) ? 1.0 - ss_res / ss_tot : 0.0;
    }
}

double calibration_linearity_error(const calibration_point_t *points, size_t n, double span)
{
    if (!points || n < 2 || span <= 0.0) return 0.0;
    double x0 = points[0].reference_value;
    double y0 = points[0].measured_value;
    double x1 = points[n-1].reference_value;
    double y1 = points[n-1].measured_value;
    double slope = (y1 - y0) / (x1 - x0);
    double max_dev = 0.0;
    for (size_t i = 0; i < n; i++) {
        double y_line = y0 + slope * (points[i].reference_value - x0);
        double dev = fabs(points[i].measured_value - y_line);
        if (dev > max_dev) max_dev = dev;
    }
    return (max_dev / span) * 100.0;
}

double calibration_hysteresis_error(const double *upscale, const double *downscale,
    size_t n, double span)
{
    if (!upscale || !downscale || n < 2 || span <= 0.0) return 0.0;
    double max_hyst = 0.0;
    for (size_t i = 0; i < n; i++) {
        double hyst = fabs(upscale[i] - downscale[i]);
        if (hyst > max_hyst) max_hyst = hyst;
    }
    return (max_hyst / span) * 100.0;
}

bool calibration_polynomial_fit(const double *x, const double *y,
    size_t n, size_t degree, double *coeffs)
{
    if (!x || !y || !coeffs || n <= degree || degree < 1) return false;
    size_t m = degree + 1;
    double *A = (double*)calloc(m * m, sizeof(double));
    double *B = (double*)calloc(m, sizeof(double));
    if (!A || !B) { free(A); free(B); return false; }
    for (size_t i = 0; i < n; i++) {
        double xi_pow = 1.0;
        for (size_t p = 0; p < m; p++) {
            B[p] += xi_pow * y[i];
            double xj_pow = 1.0;
            for (size_t q = 0; q < m; q++) {
                A[p * m + q] += xi_pow * xj_pow;
                xj_pow *= x[i];
            }
            xi_pow *= x[i];
        }
    }
    for (size_t k = 0; k < m; k++) {
        double max_val = fabs(A[k * m + k]);
        size_t max_row = k;
        for (size_t r = k + 1; r < m; r++) {
            if (fabs(A[r * m + k]) > max_val) {
                max_val = fabs(A[r * m + k]);
                max_row = r;
            }
        }
        if (max_val < 1e-12) { free(A); free(B); return false; }
        if (max_row != k) {
            for (size_t c = 0; c < m; c++) {
                double tmp = A[k * m + c];
                A[k * m + c] = A[max_row * m + c];
                A[max_row * m + c] = tmp;
            }
            double tmp = B[k]; B[k] = B[max_row]; B[max_row] = tmp;
        }
        for (size_t r = k + 1; r < m; r++) {
            double factor = A[r * m + k] / A[k * m + k];
            for (size_t c = k; c < m; c++)
                A[r * m + c] -= factor * A[k * m + c];
            B[r] -= factor * B[k];
        }
    }
    for (int k = (int)m - 1; k >= 0; k--) {
        double sum = B[k];
        for (size_t c = k + 1; c < m; c++)
            sum -= A[k * m + c] * coeffs[c];
        coeffs[k] = sum / A[k * m + k];
    }
    free(A); free(B);
    return true;
}

double calibration_apply_polynomial(double x, const double *coeffs, size_t degree)
{
    if (!coeffs) return 0.0;
    double r = coeffs[degree];
    for (size_t i = degree; i > 0; i--)
        r = r * x + coeffs[i - 1];
    return r;
}

double calibration_repeatability(const double *repeated, size_t n, double span)
{
    if (!repeated || n < 2 || span <= 0.0) return 0.0;
    double mean = 0.0;
    for (size_t i = 0; i < n; i++) mean += repeated[i];
    mean /= (double)n;
    double variance = 0.0;
    for (size_t i = 0; i < n; i++) {
        double d = repeated[i] - mean;
        variance += d * d;
    }
    variance /= (double)(n - 1);
    return (sqrt(variance) / span) * 100.0;
}

void calibration_zero_span_trim(double m_zero, double m_span,
    double *zero_trim, double *span_trim)
{
    if (!zero_trim || !span_trim) return;
    *zero_trim = 4.0 - m_zero;
    /* span_error tracks deviation from ideal 16mA span */
    if (fabs(m_span - m_zero) > 0.001)
        *span_trim = 16.0 / (m_span - m_zero);
    else
        *span_trim = 1.0;
}

double calibration_tempco(double i_t1, double i_t2, double t1, double t2)
{
    double dt = t2 - t1;
    if (fabs(dt) < 0.01) return 0.0;
    return (i_t2 - i_t1) / (16.0 * dt) * 100.0;
}

bool calibration_validate(const calibration_data_t *cal, double tolerance_pct)
{
    if (!cal || !cal->is_valid) return false;
    return cal->total_accuracy_percent <= tolerance_pct;
}

void calibration_spline_init(const double *x, const double *y, size_t n, double *y2)
{
    if (!x || !y || !y2 || n < 3) return;
    double *u = (double*)malloc(n * sizeof(double));
    if (!u) return;
    y2[0] = 0.0; u[0] = 0.0;
    for (size_t i = 1; i < n - 1; i++) {
        double sig = (x[i] - x[i-1]) / (x[i+1] - x[i-1]);
        double p = sig * y2[i-1] + 2.0;
        y2[i] = (sig - 1.0) / p;
        u[i] = (y[i+1] - y[i]) / (x[i+1] - x[i])
             - (y[i] - y[i-1]) / (x[i] - x[i-1]);
        u[i] = (6.0 * u[i] / (x[i+1] - x[i-1]) - sig * u[i-1]) / p;
    }
    y2[n-1] = 0.0;
    for (size_t k = n - 2; k < n; k--) {
        y2[k] = y2[k] * y2[k+1] + u[k];
    }
    free(u);
}

double calibration_spline_interpolate(double x, const double *x_pts,
    const double *y_pts, size_t n, const double *y2)
{
    if (!x_pts || !y_pts || !y2 || n < 2) return 0.0;
    size_t lo = 0, hi = n - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (x_pts[mid] <= x) lo = mid; else hi = mid;
    }
    double h = x_pts[hi] - x_pts[lo];
    if (h <= 0.0) return y_pts[lo];
    double a = (x_pts[hi] - x) / h;
    double b = (x - x_pts[lo]) / h;
    return a * y_pts[lo] + b * y_pts[hi]
         + ((a*a*a - a) * y2[lo] + (b*b*b - b) * y2[hi]) * (h*h) / 6.0;
}
