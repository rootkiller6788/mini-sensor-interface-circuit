/**
 * @file    tc_linearization.c
 * @brief   Linearization methods for thermocouple and RTD sensors:
 *          piecewise linear interpolation, calibration table management,
 *          cubic spline interpolation, and linearity error analysis.
 *
 * Knowledge Coverage:
 *   L3: Piecewise linear interpolation, binary search in sorted segments
 *   L5: Cubic spline interpolation for smooth calibration curves
 *   L5: Calibration table lifecycle (create, query, free)
 *   L6: Linearity error analysis across operating temperature range
 *   L8: Robust linear regression with Huber loss function (M-estimator)
 *
 * Reference:
 *   de Boor, C. (1978) "A Practical Guide to Splines"
 *   Press, W.H. et al. (2007) "Numerical Recipes", 3rd ed., Section 3.3
 *   Huber, P.J. (1964) "Robust Estimation of a Location Parameter"
 *   NIST/SEMATECH e-Handbook of Statistical Methods, Section 2.4
 */

#include "thermocouple_cjc_rtd.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

/* =========================================================================
 * L3: Piecewise Linear Interpolation
 * ========================================================================= */

/**
 * @brief tc_piecewise_build: Construct piecewise linear interpolation model
 *
 * Given sorted (x, y) data points, builds an array of linear segments.
 * Each segment covers [x[i], x[i+1]] with:
 *   slope = (y[i+1] - y[i]) / (x[i+1] - x[i])
 *   intercept = y[i] - slope * x[i]
 *
 * For a thermocouple calibration table, this means each segment
 * approximates the EMF-T relationship as linear between adjacent
 * calibration points. Accuracy improves with denser calibration points.
 *
 * Complexity: O(N) time, O(N) space for N calibration points.
 */
tc_error_t tc_piecewise_build(const double *x_data, const double *y_data,
                               size_t n_points, tc_piecewise_model_t *model) {
    size_t i;

    if (!x_data || !y_data || !model) return TC_ERR_NULL_POINTER;
    if (n_points < 2) return TC_ERR_INTERPOLATION;

    model->n_segments = n_points - 1;
    model->segments = (tc_piecewise_segment_t *)malloc(
        model->n_segments * sizeof(tc_piecewise_segment_t));
    if (!model->segments) return TC_ERR_INTERPOLATION;

    model->input_min = x_data[0];
    model->input_max = x_data[n_points - 1];
    model->error_bound = 0.0;

    for (i = 0; i < model->n_segments; i++) {
        double dx = x_data[i + 1] - x_data[i];
        double dy = y_data[i + 1] - y_data[i];

        if (fabs(dx) < 1e-20) {
            /* Zero-width segment: treat as vertical (unlikely) */
            model->segments[i].x_low = x_data[i];
            model->segments[i].x_high = x_data[i];
            model->segments[i].slope = 0.0;
            model->segments[i].intercept = y_data[i];
        } else {
            model->segments[i].x_low = x_data[i];
            model->segments[i].x_high = x_data[i + 1];
            model->segments[i].slope = dy / dx;
            model->segments[i].intercept = y_data[i]
                - model->segments[i].slope * x_data[i];
        }
    }

    return TC_OK;
}

/**
 * @brief tc_piecewise_eval: Evaluate piecewise linear model
 *
 * Uses binary search to find the containing segment in O(log N) time,
 * then performs a simple linear interpolation.
 *
 * For out-of-range inputs, returns NaN and TC_ERR_OUT_OF_RANGE.
 * This is a safety feature: interpolation should never extrapolate.
 */
tc_error_t tc_piecewise_eval(const tc_piecewise_model_t *model,
                              double x, double *y) {
    size_t lo, hi, mid;

    if (!model || !model->segments || !y) return TC_ERR_NULL_POINTER;
    if (x < model->input_min || x > model->input_max) {
        *y = NAN;
        return TC_ERR_OUT_OF_RANGE;
    }

    /* Binary search for the containing segment */
    lo = 0;
    hi = model->n_segments - 1;
    while (lo < hi) {
        mid = lo + (hi - lo) / 2;
        if (x < model->segments[mid].x_high) {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }

    /* Linear interpolation: y = slope * x + intercept */
    *y = model->segments[lo].slope * x + model->segments[lo].intercept;
    return TC_OK;
}

void tc_piecewise_free(tc_piecewise_model_t *model) {
    if (model && model->segments) {
        free(model->segments);
        model->segments = NULL;
        model->n_segments = 0;
    }
}

/* =========================================================================
 * L5: Calibration Table Management
 * ========================================================================= */

tc_error_t tc_cal_table_create(const tc_cal_point_t *points,
                                size_t n_points, tc_type_t type,
                                const char *serial, tc_cal_table_t *cal) {
    if (!points || !cal || n_points == 0) return TC_ERR_NULL_POINTER;
    if (type >= TC_COUNT) return TC_ERR_INVALID_TYPE;

    memset(cal, 0, sizeof(*cal));
    cal->type = type;
    cal->n_points = n_points;
    cal->points = (tc_cal_point_t *)malloc(n_points * sizeof(tc_cal_point_t));
    if (!cal->points) return TC_ERR_CALIBRATION;

    memcpy(cal->points, points, n_points * sizeof(tc_cal_point_t));

    if (serial) {
        strncpy(cal->serial, serial, sizeof(cal->serial) - 1);
        cal->serial[sizeof(cal->serial) - 1] = '\0';
    }

    return TC_OK;
}

void tc_cal_table_free(tc_cal_table_t *cal) {
    if (cal && cal->points) {
        free(cal->points);
        cal->points = NULL;
        cal->n_points = 0;
    }
}

/**
 * @brief tc_temp_to_emf_calibrated: Calibrated temperature to EMF conversion
 *
 * Uses the calibration table with piecewise linear interpolation
 * between measured calibration points instead of the standard
 * ITS-90 polynomials. This corrects for the specific sensor's
 * deviations from the ideal characteristic.
 *
 * For best results, calibration points should be densely spaced
 * in the temperature region of interest (e.g., every 50C).
 */
tc_error_t tc_temp_to_emf_calibrated(const tc_cal_table_t *cal,
                                      double temp, double *emf) {
    tc_piecewise_model_t model;
    tc_error_t err;
    size_t i;

    if (!cal || !cal->points || !emf) return TC_ERR_NULL_POINTER;

    /* Build piecewise model on the fly from calibration data */
    {
        double *x = (double *)malloc(cal->n_points * sizeof(double));
        double *y = (double *)malloc(cal->n_points * sizeof(double));
        if (!x || !y) { free(x); free(y); return TC_ERR_CALIBRATION; }

        for (i = 0; i < cal->n_points; i++) {
            x[i] = cal->points[i].temperature;
            y[i] = cal->points[i].emf;
        }

        err = tc_piecewise_build(x, y, cal->n_points, &model);
        free(x); free(y);
    }

    if (err != TC_OK) return err;

    err = tc_piecewise_eval(&model, temp, emf);
    tc_piecewise_free(&model);
    return err;
}

/**
 * @brief tc_emf_to_temp_calibrated: Calibrated EMF to temperature conversion
 *
 * Inverse of tc_temp_to_emf_calibrated using linear search followed
 * by linear interpolation since the calibration data may not be
 * in EMF-sorted order for all thermocouple types (e.g., Type B).
 */
tc_error_t tc_emf_to_temp_calibrated(const tc_cal_table_t *cal,
                                      double emf, double *temp) {
    size_t i;
    double *e_arr, *t_arr;

    if (!cal || !cal->points || !temp) return TC_ERR_NULL_POINTER;
    if (cal->n_points < 2) return TC_ERR_INTERPOLATION;

    e_arr = (double *)malloc(cal->n_points * sizeof(double));
    t_arr = (double *)malloc(cal->n_points * sizeof(double));
    if (!e_arr || !t_arr) { free(e_arr); free(t_arr); return TC_ERR_CALIBRATION; }

    for (i = 0; i < cal->n_points; i++) {
        e_arr[i] = cal->points[i].emf;
        t_arr[i] = cal->points[i].temperature;
    }

    /* Find the containing interval in EMF space */
    if (emf <= e_arr[0]) {
        *temp = t_arr[0];
        free(e_arr); free(t_arr);
        return TC_OK;
    }
    if (emf >= e_arr[cal->n_points - 1]) {
        *temp = t_arr[cal->n_points - 1];
        free(e_arr); free(t_arr);
        return TC_OK;
    }

    for (i = 0; i < cal->n_points - 1; i++) {
        if (emf >= e_arr[i] && emf <= e_arr[i + 1]) {
            double frac = (emf - e_arr[i]) / (e_arr[i + 1] - e_arr[i]);
            *temp = t_arr[i] + frac * (t_arr[i + 1] - t_arr[i]);
            free(e_arr); free(t_arr);
            return TC_OK;
        }
        /* Handle non-monotonic EMF (Type B at low temp) */
        if (emf >= e_arr[i + 1] && emf <= e_arr[i]) {
            double frac = (emf - e_arr[i + 1]) / (e_arr[i] - e_arr[i + 1]);
            *temp = t_arr[i + 1] + frac * (t_arr[i] - t_arr[i + 1]);
            free(e_arr); free(t_arr);
            return TC_OK;
        }
    }

    free(e_arr); free(t_arr);
    return TC_ERR_INTERPOLATION;
}

/* =========================================================================
 * L5: Cubic Spline Interpolation for Calibration Tables
 * ========================================================================= */

/**
 * @brief tc_cal_spline_interpolate: Natural cubic spline interpolation
 *
 * Fits a C2-continuous piecewise cubic polynomial through the
 * calibration points. The "natural" spline has zero second derivative
 * at the endpoints (y''(x0) = y''(xn) = 0).
 *
 * For thermocouple calibration data, spline interpolation provides
 * smoother EMF estimates than linear interpolation, especially when
 * calibration points are sparse. However, splines can overshoot
 * if the underlying function has sharp changes.
 *
 * Algorithm (from Numerical Recipes):
 *   1. Set up tridiagonal system for second derivatives y''_i
 *   2. Solve tridiagonal system
 *   3. For query point x in [x_j, x_{j+1}]:
 *        h = x_{j+1} - x_j
 *        a = (x_{j+1} - x) / h
 *        b = (x - x_j) / h
 *        y = a*y_j + b*y_{j+1} + ((a^3-a)*y''_j + (b^3-b)*y''_{j+1})*h^2/6
 *
 * Complexity: O(N) for setup, O(log N) for each evaluation.
 */
tc_error_t tc_cal_spline_interpolate(const tc_cal_table_t *cal,
                                      double temp, double *emf) {
    size_t n, i;
    double *h, *alpha, *m, *l, *mu, *z;
    double a, b, result;

    if (!cal || !cal->points || !emf) return TC_ERR_NULL_POINTER;
    n = cal->n_points;
    if (n < 3) {
        /* Fall back to linear for small tables */
        return tc_temp_to_emf_calibrated(cal, temp, emf);
    }

    /* Allocate spline workspace */
    h     = (double *)malloc(n * sizeof(double));
    alpha = (double *)malloc(n * sizeof(double));
    m     = (double *)malloc(n * sizeof(double));
    l     = (double *)malloc(n * sizeof(double));
    mu    = (double *)malloc(n * sizeof(double));
    z     = (double *)malloc(n * sizeof(double));

    if (!h || !alpha || !m || !l || !mu || !z) {
        free(h); free(alpha); free(m); free(l); free(mu); free(z);
        return TC_ERR_INTERPOLATION;
    }

    /* Step 1: Compute interval widths */
    for (i = 0; i < n - 1; i++) {
        h[i] = cal->points[i + 1].temperature - cal->points[i].temperature;
        if (h[i] <= 0.0) {
            free(h); free(alpha); free(m); free(l); free(mu); free(z);
            return TC_ERR_INTERPOLATION;
        }
    }

    /* Step 2: Set up tridiagonal system */
    for (i = 1; i < n - 1; i++) {
        alpha[i] = (3.0 / h[i]) * (cal->points[i + 1].emf - cal->points[i].emf)
                 - (3.0 / h[i - 1]) * (cal->points[i].emf - cal->points[i - 1].emf);
    }

    /* Natural spline boundary conditions: m[0] = m[n-1] = 0 */
    l[0] = 1.0; mu[0] = 0.0; z[0] = 0.0;

    /* Forward elimination */
    for (i = 1; i < n - 1; i++) {
        l[i] = 2.0 * (cal->points[i + 1].temperature - cal->points[i - 1].temperature)
               - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0; z[n - 1] = 0.0;
    m[n - 1] = 0.0;

    /* Back substitution */
    for (i = n - 1; i > 0; i--) {
        m[i - 1] = z[i - 1] - mu[i - 1] * m[i];
    }

    /* Evaluate spline at query point */
    /* Binary search for containing interval */
    {
        size_t lo = 0, hi = n - 1;
        while (hi - lo > 1) {
            size_t mid = lo + (hi - lo) / 2;
            if (temp <= cal->points[mid].temperature)
                hi = mid;
            else
                lo = mid;
        }
        i = lo;
    }

    a = (cal->points[i + 1].temperature - temp) / h[i];
    b = (temp - cal->points[i].temperature) / h[i];

    result = a * cal->points[i].emf + b * cal->points[i + 1].emf
             + ((a * a * a - a) * m[i] + (b * b * b - b) * m[i + 1])
               * h[i] * h[i] / 6.0;

    *emf = result;

    free(h); free(alpha); free(m); free(l); free(mu); free(z);
    return TC_OK;
}

/* =========================================================================
 * L6: Linearity Error Analysis
 * ========================================================================= */

/**
 * @brief tc_linearity_error: Compute maximum linearity error
 *
 * Evaluates the maximum deviation between the true ITS-90 EMF function
 * and a linear approximation using the average Seebeck coefficient:
 *
 *   E_linear(T) = S_avg * T
 *   Error(T) = |E_ITS90(T) - E_linear(T)|
 *
 * The maximum error over the specified range is returned. This tells
 * the user whether a simple linear gain stage is adequate or whether
 * full polynomial linearization is needed.
 *
 * Example: Type K, 0 to 100C
 *   S_avg ~ 40.6 uV/C
 *   Max error ~ 1.0C (nonlinearity of the EMF-T curve)
 *
 * Uses 1000 evaluation points across the range for accuracy.
 */
tc_error_t tc_linearity_error(tc_type_t type, double t_min, double t_max,
                               double *max_error) {
    double s_avg, s_min, s_max, emf_true, emf_linear, error;
    size_t i, n_points = 1000;
    double t_range;

    if (!max_error) return TC_ERR_NULL_POINTER;
    if (type >= TC_COUNT) return TC_ERR_INVALID_TYPE;
    if (t_min >= t_max) return TC_ERR_OUT_OF_RANGE;

    /* Compute average Seebeck for the linear model */
    tc_seebeck_coefficient(type, t_min, &s_min);
    tc_seebeck_coefficient(type, t_max, &s_max);
    s_avg = (s_min + s_max) / 2.0;

    /* Scan the range for maximum deviation */
    *max_error = 0.0;
    t_range = t_max - t_min;

    for (i = 0; i <= n_points; i++) {
        double t = t_min + (t_range * (double)i) / (double)n_points;
        double emf_err;
        tc_temp_to_emf(type, t, &emf_true);
        emf_linear = s_avg * t;
        emf_err = fabs(emf_true - emf_linear);

        /* Convert EMF error to approximate temperature error */
        {
            double s_at_t;
            if (tc_seebeck_coefficient(type, t, &s_at_t) == TC_OK
                && fabs(s_at_t) > 1e-15) {
                error = emf_err / s_at_t;
            } else {
                error = emf_err / fabs(s_avg);
            }
        }

        if (error > *max_error) *max_error = error;
    }

    return TC_OK;
}

/* =========================================================================
 * L8: Robust Linear Regression (Huber M-Estimator)
 * ========================================================================= */

/**
 * @brief tc_robust_linear_fit: Huber robust regression for linear model
 *
 * Fits y = slope*x + intercept using iteratively reweighted least squares
 * with Huber's loss function, which handles outliers gracefully:
 *
 *   rho(r) = { 0.5*r^2               if |r| <= k
 *            { k*(|r| - 0.5*k)       if |r| > k
 *
 * where r = y_i - (slope*x_i + intercept) is the residual.
 *
 * This is useful for calibrating thermocouple measurements when the
 * data may contain occasional glitches (EMI spikes, poor connections)
 * that would corrupt a standard least-squares fit.
 *
 * Threshold k is typically 1.345 * sigma for 95% efficiency vs OLS.
 * For thermocouple calibration, k ~ 1.0-5.0 uV is typical.
 *
 * Iterative reweighting:
 *   1. OLS fit for initial slope/intercept
 *   2. Compute residuals r_i
 *   3. Compute Huber weights w_i = psi(r_i) / r_i
 *   4. Weighted least squares for new slope/intercept
 *   5. Repeat 2-4 until convergence
 *
 * @param x_data     Temperature array [n points]
 * @param y_data     EMF array [n points]
 * @param n          Number of data points
 * @param k          Huber threshold
 * @param slope      Output: robust slope [mV/C]
 * @param intercept  Output: robust intercept [mV]
 * @param mse        Output: robust MSE estimate
 * @return TC_OK on success
 */
tc_error_t tc_robust_linear_fit(const double *x_data, const double *y_data,
                                 size_t n, double k, double *slope,
                                 double *intercept, double *mse) {
    double *weights, *residuals;
    double sx, sy, sxx, sxy, sw, swx, swy, swxx, swxy;
    double b0, b1, r, w, sum_sq;
    size_t iter, i;

    if (!x_data || !y_data || !slope || !intercept || !mse || n < 3)
        return TC_ERR_NULL_POINTER;

    weights   = (double *)malloc(n * sizeof(double));
    residuals = (double *)malloc(n * sizeof(double));
    if (!weights || !residuals) {
        free(weights); free(residuals);
        return TC_ERR_NULL_POINTER;
    }

    /* Initial OLS fit */
    sx = sy = sxx = sxy = 0.0;
    for (i = 0; i < n; i++) {
        sx  += x_data[i];
        sy  += y_data[i];
        sxx += x_data[i] * x_data[i];
        sxy += x_data[i] * y_data[i];
    }
    b1 = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    b0 = (sy - b1 * sx) / n;

    /* Iteratively reweighted least squares (max 20 iterations) */
    for (iter = 0; iter < 20; iter++) {
        double b1_prev = b1;
        double b0_prev = b0;

        /* Compute residuals and Huber weights */
        for (i = 0; i < n; i++) {
            r = y_data[i] - (b1 * x_data[i] + b0);
            residuals[i] = r;
            /* Huber weight: psi(r)/r = min(1, k/|r|) */
            if (fabs(r) <= k) {
                w = 1.0;
            } else {
                w = k / fabs(r);
            }
            weights[i] = w;
        }

        /* Weighted least squares */
        sw = swx = swy = swxx = swxy = 0.0;
        for (i = 0; i < n; i++) {
            w = weights[i];
            sw   += w;
            swx  += w * x_data[i];
            swy  += w * y_data[i];
            swxx += w * x_data[i] * x_data[i];
            swxy += w * x_data[i] * y_data[i];
        }

        if (sw < 1e-10) break;
        b1 = (sw * swxy - swx * swy) / (sw * swxx - swx * swx);
        b0 = (swy - b1 * swx) / sw;

        /* Convergence check */
        if (fabs(b1 - b1_prev) < 1e-12 && fabs(b0 - b0_prev) < 1e-12) break;
    }

    /* Compute robust MSE using Huber loss */
    sum_sq = 0.0;
    for (i = 0; i < n; i++) {
        r = y_data[i] - (b1 * x_data[i] + b0);
        if (fabs(r) <= k) {
            sum_sq += 0.5 * r * r;
        } else {
            sum_sq += k * (fabs(r) - 0.5 * k);
        }
    }
    *mse = sum_sq / (double)n;
    *slope = b1;
    *intercept = b0;

    free(weights); free(residuals);
    return TC_OK;
}
