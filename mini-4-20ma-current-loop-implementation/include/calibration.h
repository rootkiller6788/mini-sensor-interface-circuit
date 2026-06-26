/**
 * @file calibration.h
 * @brief 4-20mA loop calibration, linearization, and compensation.
 *
 * Industrial transmitters require periodic calibration to maintain accuracy.
 * This module covers zero/span calibration, multi-point linearization,
 * temperature compensation, and calibration certificate generation.
 *
 * Reference: ISA-50.1, ISO/IEC 17025 (Calibration Laboratory Requirements)
 * Knowledge: L5 calibration algorithms, L6 transmitter calibration,
 *            L7 industrial metrology, L3 polynomial/NL models
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "current_loop.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calibration point — a single (reference, measured) pair (L5).
 */
typedef struct {
    double reference_value;
    double measured_value;
    double error;
    double error_percent_of_span;
} calibration_point_t;

/**
 * @brief Complete calibration data set (L5/L6).
 *
 * Stores the results of a multi-point calibration procedure
 * for a 4-20mA transmitter or receiver.
 */
typedef struct {
    calibration_point_t *points;
    size_t n_points;
    double zero_offset;
    double span_gain;
    double linearity_error_percent;
    double hysteresis_error_percent;
    double repeatability_error_percent;
    double total_accuracy_percent;
    bool   is_valid;
} calibration_data_t;

/**
 * @brief Best-fit linear regression for calibration (L5).
 *
 * Ordinary Least Squares (OLS) to find slope (gain) and intercept (offset):
 *   slope = (N*sum(xy) - sum(x)*sum(y)) / (N*sum(x^2) - sum(x)^2)
 *   intercept = (sum(y) - slope*sum(x)) / N
 *
 * Where x = reference values, y = measured values.
 *
 * @param x          Reference values array
 * @param y          Measured values array
 * @param n          Number of calibration points
 * @param slope_out  Output: best-fit slope (gain)
 * @param inter_out  Output: best-fit intercept (offset)
 * @param r2_out     Output: R-squared goodness-of-fit
 */
void calibration_linear_regression(const double *x, const double *y,
                                    size_t n, double *slope_out,
                                    double *inter_out, double *r2_out);

/**
 * @brief Compute maximum linearity error (end-point method) (L5).
 *
 * Linearity = maximum deviation from the best-fit straight line,
 * expressed as percentage of full-scale span.
 *
 * The end-point method draws a line between the first and last
 * calibration points and measures deviation from it.
 *
 * @param points   Array of calibration points
 * @param n        Number of points
 * @param span     Full-scale span (e.g., 16.0 mA)
 * @return         Maximum linearity error as % of span
 */
double calibration_linearity_error(const calibration_point_t *points,
                                    size_t n, double span);

/**
 * @brief Compute hysteresis error (L5/L6).
 *
 * Hysteresis = maximum difference between up-scale and down-scale
 * measurements at the same reference value, as % of span.
 *
 * @param upscale   Measurements taken while increasing
 * @param downscale Measurements taken while decreasing
 * @param n         Number of points
 * @param span      Full-scale span
 * @return          Maximum hysteresis as % of span
 */
double calibration_hysteresis_error(const double *upscale,
                                     const double *downscale,
                                     size_t n, double span);

/**
 * @brief Polynomial fit for non-linear sensor calibration (L5).
 *
 * Fits a polynomial of specified degree using least-squares.
 * Uses Gaussian elimination with partial pivoting to solve the
 * normal equations.
 *
 * @param x        Reference values
 * @param y        Measured values
 * @param n        Number of data points
 * @param degree   Polynomial degree (1=linear, 2=quadratic, etc.)
 * @param coeffs   Output: polynomial coefficients [c0, c1, ..., c_degree]
 * @return         true if fit succeeded
 */
bool calibration_polynomial_fit(const double *x, const double *y,
                                 size_t n, size_t degree, double *coeffs);

/**
 * @brief Apply polynomial calibration correction (L5).
 *
 * Uses Horner's method for stable evaluation:
 *   y = c0 + x*(c1 + x*(c2 + x*(c3 + ...)))
 *
 * @param x        Raw measured value
 * @param coeffs   Polynomial coefficients
 * @param degree   Polynomial degree
 * @return         Corrected value
 */
double calibration_apply_polynomial(double x, const double *coeffs,
                                     size_t degree);

/**
 * @brief Compute calibration repeatability (L5/L6).
 *
 * Repeatability = standard deviation of repeated measurements
 * at the same calibration point, expressed as % of span.
 * Typically requires >= 5 repeat measurements at each point.
 *
 * @param repeated   Array of repeated measurements at one point
 * @param n_repeats  Number of repeats
 * @param span       Full-scale span
 * @return           Repeatability as % of span
 */
double calibration_repeatability(const double *repeated,
                                  size_t n_repeats, double span);

/**
 * @brief Zero and span adjustment (L6).
 *
 * Classic two-point calibration for 4-20mA transmitters:
 *   1. Apply 0% input -> adjust zero trim for 4.000 mA output
 *   2. Apply 100% input -> adjust span trim for 20.000 mA output
 *   3. Repeat until both converge (interaction between adjustments)
 *
 * This function computes the required trim values.
 *
 * @param measured_at_zero  Measured output at 0% input (mA)
 * @param measured_at_span  Measured output at 100% input (mA)
 * @param zero_trim         Output: recommended zero trim adjustment (mA)
 * @param span_trim         Output: recommended span trim multiplier
 */
void calibration_zero_span_trim(double measured_at_zero,
                                 double measured_at_span,
                                 double *zero_trim, double *span_trim);

/**
 * @brief Temperature coefficient calibration (L7).
 *
 * Measures output drift vs. temperature to characterize the
 * transmitter's temperature coefficient.
 *
 * alpha = (I_hot - I_cold) / (I_span * (T_hot - T_cold))
 *
 * Typical spec: < 0.01% of span per degC.
 *
 * @param current_at_t1   Loop current at temperature T1 (mA)
 * @param current_at_t2   Loop current at temperature T2 (mA)
 * @param t1              Temperature 1 (degC)
 * @param t2              Temperature 2 (degC)
 * @return                Temperature coefficient (% span per degC)
 */
double calibration_tempco(double current_at_t1, double current_at_t2,
                           double t1, double t2);

/**
 * @brief Generate a calibration certificate summary (L7).
 *
 * ISO/IEC 17025 compliant calibration requires documenting:
 * - Reference standards used (traceable to national standards)
 * - Environmental conditions (temperature, humidity)
 * - Measurement uncertainty budget
 * - Pass/fail criteria
 *
 * @param cal_data     Calibration data
 * @param tolerance_pct Acceptance tolerance (% of span)
 * @return             true if calibration passes acceptance criteria
 */
bool calibration_validate(const calibration_data_t *cal_data,
                           double tolerance_pct);

/**
 * @brief Interpolate between calibration points using cubic spline (L5).
 *
 * Natural cubic spline interpolation for smooth calibration curves.
 * The spline passes exactly through all calibration points and has
 * continuous first and second derivatives.
 *
 * @param x         Target x value
 * @param x_points  Calibration x breakpoints (monotonic increasing)
 * @param y_points  Calibration y values
 * @param n         Number of points
 * @param y2        Second derivatives (precomputed by spline_init)
 * @return          Interpolated y value
 */
double calibration_spline_interpolate(double x, const double *x_points,
                                       const double *y_points, size_t n,
                                       const double *y2);

/**
 * @brief Initialize second derivatives for cubic spline (L5).
 *
 * Solves the tridiagonal system for natural cubic spline.
 * y2[0] = y2[n-1] = 0 (natural boundary conditions).
 *
 * @param x_points  x coordinates (length n)
 * @param y_points  y coordinates (length n)
 * @param n         Number of points
 * @param y2        Output: second derivatives (length n)
 */
void calibration_spline_init(const double *x_points, const double *y_points,
                              size_t n, double *y2);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_H */