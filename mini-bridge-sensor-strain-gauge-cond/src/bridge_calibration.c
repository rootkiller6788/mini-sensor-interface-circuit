/**
 * bridge_calibration.c — Bridge Sensor Calibration Implementation
 *
 * Knowledge Coverage:
 *   L2: Shunt calibration, two-point calibration
 *   L3: Least-squares linear regression, polynomial fitting
 *   L5: Multi-point calibration, cross-validation for model selection
 *
 * Reference:
 *   - ASTM E74 — "Calibration of Force-Measuring Instruments"
 *   - ISO 376 — "Calibration of Force-Proving Instruments"
 *   - ISO/IEC Guide 98-3 (GUM) — "Uncertainty in Measurement"
 *   - Micro-Measurements Tech Note TN-514, "Shunt Calibration"
 */

#include "bridge_calibration.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

double calibration_shunt_strain(const strain_gauge_t *gauge,
                                double shunt_r_ohm, int arm)
{
    /* Compute simulated strain from shunt calibration.
     *
     * A shunt resistor R_shunt connected in parallel with a
     * strain gauge of resistance R_gauge changes the effective
     * resistance to:
     *
     *   R_eff = R_gauge * R_shunt / (R_gauge + R_shunt)
     *
     * The change in resistance: dR = R_eff - R_gauge
     *                          = -R_gauge^2 / (R_gauge + R_shunt)
     *
     * The simulated strain:
     *   eps_sim = (dR / R_gauge) / GF
     *           = -R_gauge / (GF * (R_gauge + R_shunt))
     *
     * For R_shunt >> R_gauge (typical):
     *   eps_sim ≈ -R_gauge / (GF * R_shunt)
     *
     * Example: R_gauge=350, R_shunt=350k, GF=2.0
     *   eps_sim = -350 / (2.0 * 350350) = -350 / 700700
     *           = -4.995e-4 = -499.5 microstrain
     *
     * The shunt resistor is typically connected across one arm
     * to simulate either tension or compression, depending on
     * which arm is shunted and the bridge wiring.
     *
     * Arm convention (shunt across R1):
     *   R1 decreases → V_left decreases → Vout changes
     *   (sign depends on bridge wiring convention)
     *
     * (void)arm — reserved for arm-specific shunt routing analysis.
     */

    if (gauge == NULL || shunt_r_ohm <= 0.0) return 0.0;

    double r_gauge = gauge->nominal_resistance;
    double gf = gauge->gauge_factor;

    if (gf <= 0.0) return 0.0;

    double r_total = r_gauge + shunt_r_ohm;
    double eps_sim = -r_gauge / (gf * r_total);

    (void)arm;  /* Reserved */

    return eps_sim * 1.0e6;  /* Convert to microstrain */
}

void calibration_two_point(const cal_data_point_t *zero_point,
                           const cal_data_point_t *span_point,
                           calibration_result_t *result)
{
    /* Two-point calibration: zero and span.
     *
     * This is the simplest quantitative calibration method.
     * It determines the slope (sensitivity) and offset (zero)
     * of a linear transfer function:
     *
     *   output = slope * measurand + offset
     *
     * slope = (V_span - V_zero) / (M_span - M_zero)
     * offset = V_zero - slope * M_zero
     *
     * where V = bridge output voltage, M = applied measurand.
     *
     * The inverse transfer function (for measurement):
     *   measurand = (output - offset) / slope
     *
     * Assumptions:
     * - The transfer function is linear between zero and span.
     * - No hysteresis (apply loading only, or average load/unload).
     * - Temperature is constant during calibration.
     */

    if (zero_point == NULL || span_point == NULL || result == NULL) return;

    memset(result, 0, sizeof(*result));

    double dx = span_point->measurand - zero_point->measurand;
    double dy = span_point->output_voltage - zero_point->output_voltage;

    if (fabs(dx) < 1.0e-15) {
        result->slope = 0.0;
        result->offset = zero_point->output_voltage;
    } else {
        result->slope = dy / dx;
        result->offset = zero_point->output_voltage
                         - result->slope * zero_point->measurand;
    }

    result->method = CAL_TWO_POINT;
    result->n_points = 2;
    result->nonlinearity_percent = 0.0;  /* Cannot assess with 2 points */
    result->hysteresis_percent = 0.0;     /* Cannot assess with 2 points */
    result->r_squared = 1.0;              /* 2 points always fit perfectly */
    result->temperature_c = zero_point->temperature_c;
}

double calibration_shunt_resistor(const strain_gauge_t *gauge,
                                  double strain_ue, int arm)
{
    /* Calculate shunt resistor value for desired simulated strain.
     *
     * From the shunt calibration equation:
     *   eps_sim = -R_gauge / (GF * (R_gauge + R_shunt))
     *
     * Solving for R_shunt:
     *   GF * eps_sim * (R_gauge + R_shunt) = -R_gauge
     *   GF * eps_sim * R_gauge + GF * eps_sim * R_shunt + R_gauge = 0
     *   R_shunt * (GF * eps_sim) = -R_gauge - GF * eps_sim * R_gauge
     *   R_shunt = -R_gauge * (1 + GF * eps_sim) / (GF * eps_sim)
     *
     * Since eps_sim is negative (shunt always decreases resistance),
     * the denominator and numerator are negative, giving positive R_shunt.
     *
     * Example: R_gauge=350, GF=2.0, want eps=-500 ue = -0.0005
     *   R_shunt = -350 * (1 + 2*(-0.0005)) / (2 * (-0.0005))
     *           = -350 * (1 - 0.001) / (-0.001)
     *           = -350 * 0.999 / (-0.001)
     *           = 349.65 / 0.001 = 349,650 Ohm ≈ 350 kOhm ✓
     */

    if (gauge == NULL || strain_ue >= 0.0) return HUGE_VAL;

    double gf = gauge->gauge_factor;
    double r_gauge = gauge->nominal_resistance;
    double eps_abs = strain_ue * 1.0e-6;  /* Negative number */

    if (gf * eps_abs >= 0.0) return HUGE_VAL;  /* Prevent division by zero */

    double r_shunt = -r_gauge * (1.0 + gf * eps_abs) / (gf * eps_abs);

    (void)arm;

    return r_shunt;
}

int calibration_linear_fit(const cal_data_point_t *points,
                           int n_points,
                           calibration_result_t *result)
{
    /* Linear least-squares regression calibration.
     *
     * Fits y = a + b*x to n data points (x_i, y_i) where:
     *   x = measurand (known standard)
     *   y = output voltage (measured)
     *
     * Formulas:
     *   b = [n*sum(x_i*y_i) - sum(x_i)*sum(y_i)] /
     *       [n*sum(x_i^2) - (sum(x_i))^2]
     *
     *   a = [sum(y_i) - b*sum(x_i)] / n
     *
     *   R^2 = 1 - SS_res / SS_tot
     *
     * Residual standard deviation:
     *   s = sqrt(SS_res / (n - 2))
     *
     * Standard error of slope:
     *   SE_b = s / sqrt(sum((x_i - x_mean)^2))
     */

    if (points == NULL || result == NULL || n_points < 2) return -1;

    memset(result, 0, sizeof(*result));

    double sx = 0.0, sy = 0.0, sxy = 0.0, sx2 = 0.0;
    int i;

    for (i = 0; i < n_points; i++) {
        double x = points[i].measurand;
        double y = points[i].output_voltage;
        sx += x;
        sy += y;
        sxy += x * y;
        sx2 += x * x;
    }

    double n = (double)n_points;
    double denom = n * sx2 - sx * sx;

    if (fabs(denom) < 1.0e-15) {
        /* All x values are the same — cannot fit slope */
        result->slope = 0.0;
        result->offset = sy / n;
        return -1;
    }

    result->slope = (n * sxy - sx * sy) / denom;
    result->offset = (sy - result->slope * sx) / n;

    /* Compute R^2 and nonlinearity */
    double y_mean = sy / n;
    double ss_res = 0.0, ss_tot = 0.0;
    double max_dev = 0.0;

    for (i = 0; i < n_points; i++) {
        double y_pred = result->slope * points[i].measurand + result->offset;
        double resid = points[i].output_voltage - y_pred;
        ss_res += resid * resid;
        ss_tot += (points[i].output_voltage - y_mean)
                  * (points[i].output_voltage - y_mean);

        double dev = fabs(resid);
        if (dev > max_dev) max_dev = dev;
    }

    result->r_squared = (ss_tot > 0.0) ? 1.0 - ss_res / ss_tot : 1.0;

    /* Nonlinearity as max deviation / full scale */
    double fs_output = result->slope * (points[n_points-1].measurand
                                        - points[0].measurand);
    if (fabs(fs_output) > 1.0e-15) {
        result->nonlinearity_percent = (max_dev / fabs(fs_output)) * 100.0;
    }

    result->method = CAL_MULTI_POINT;
    result->n_points = n_points;
    result->temperature_c = points[0].temperature_c;

    return 0;
}

int calibration_polynomial_fit(const cal_data_point_t *points,
                               int n_points, int order, double *coeffs)
{
    /* Polynomial regression calibration.
     *
     * Fits y = c0 + c1*x + c2*x^2 + ... + ck*x^k
     *
     * Uses normal equations: (X^T * X) * c = X^T * y
     * and solves via Gaussian elimination with partial pivoting.
     */

    if (points == NULL || coeffs == NULL || n_points < order + 1 || order < 1)
        return -1;

    int m = order + 1;  /* Number of unknowns */
    double ata[8][9] = {{0.0}};  /* Augmented matrix (max order 7) */
    int i, j, p;

    if (m > 8) return -1;  /* Max order 7 */

    /* Build normal equations */
    for (i = 0; i < n_points; i++) {
        double x = points[i].measurand;
        double y = points[i].output_voltage;

        /* Precompute powers of x up to 2*order */
        double xpow[16];
        xpow[0] = 1.0;
        for (j = 1; j <= 2 * order; j++) {
            xpow[j] = xpow[j-1] * x;
        }

        for (j = 0; j < m; j++) {
            for (p = 0; p < m; p++) {
                ata[j][p] += xpow[j + p];
            }
            ata[j][m] += y * xpow[j];
        }
    }

    /* Gaussian elimination with partial pivoting */
    for (i = 0; i < m; i++) {
        int max_row = i;
        double max_val = fabs(ata[i][i]);
        for (j = i + 1; j < m; j++) {
            if (fabs(ata[j][i]) > max_val) {
                max_val = fabs(ata[j][i]);
                max_row = j;
            }
        }

        if (max_val < 1.0e-15) return -1;

        if (max_row != i) {
            for (j = i; j <= m; j++) {
                double tmp = ata[i][j];
                ata[i][j] = ata[max_row][j];
                ata[max_row][j] = tmp;
            }
        }

        for (j = i + 1; j < m; j++) {
            double factor = ata[j][i] / ata[i][i];
            for (p = i; p <= m; p++) {
                ata[j][p] -= factor * ata[i][p];
            }
        }
    }

    /* Back substitution */
    for (i = m - 1; i >= 0; i--) {
        double sum = ata[i][m];
        for (j = i + 1; j < m; j++) {
            sum -= ata[i][j] * coeffs[j];
        }
        coeffs[i] = sum / ata[i][i];
    }

    return 0;
}

double calibration_uncertainty(double stddev_repeat, int n_repeats,
                               double ref_uncert_pct, double temp_uncert_ue)
{
    /* Measurement uncertainty per GUM (ISO/IEC Guide 98-3).
     *
     * Type A (statistical):
     *   u_A = s / sqrt(n)
     *
     * Type B (systematic, assumed uniform distribution):
     *   u_B_ref = ref_uncert / sqrt(3)     [reference standard]
     *   u_B_temp = temp_uncert / sqrt(3)   [temperature]
     *
     * Combined standard uncertainty:
     *   u_c = sqrt(u_A^2 + u_B_ref^2 + u_B_temp^2)
     *
     * Expanded uncertainty (k=2, ~95% confidence):
     *   U = 2 * u_c
     */

    double u_A = (n_repeats > 0) ? stddev_repeat / sqrt((double)n_repeats) : stddev_repeat;

    double u_B_ref = ref_uncert_pct / sqrt(3.0);
    double u_B_temp = temp_uncert_ue / sqrt(3.0);

    double u_c = sqrt(u_A * u_A + u_B_ref * u_B_ref + u_B_temp * u_B_temp);

    return 2.0 * u_c;  /* Expanded uncertainty, k=2 */
}

void calibration_temp_compensate(calibration_result_t *result,
                                 double temperature_c,
                                 double tc_slope_ppm,
                                 double tc_offset_ue_per_c)
{
    /* Temperature-compensate calibration coefficients.
     *
     * slope(T) = slope(T0) * [1 + TC_slope * (T - T0)]
     * offset(T) = offset(T0) + TC_offset * (T - T0)
     *
     * where TC_slope is in ppm/K and TC_offset in ue/K.
     */

    if (result == NULL) return;

    double dt = temperature_c - result->temperature_c;

    double tc_slope = tc_slope_ppm * 1.0e-6;
    result->slope = result->slope * (1.0 + tc_slope * dt);
    result->offset = result->offset + tc_offset_ue_per_c * dt;
}

double calibration_cross_validate(const cal_data_point_t *points,
                                  int n_points, int max_order,
                                  int k_folds, int *best_order)
{
    /* K-fold cross-validation for polynomial order selection.
     *
     * Splits data into K folds. For each polynomial order:
     * 1. Train on K-1 folds, test on held-out fold
     * 2. Repeat K times (each fold serves as test once)
     * 3. Average prediction error across all K tests
     *
     * Select the order with minimum cross-validation error.
     * This prevents overfitting — higher orders will have
     * increasing CV error due to high variance.
     */

    if (points == NULL || n_points < 10 || max_order < 1 || k_folds < 2) {
        if (best_order) *best_order = 1;
        return HUGE_VAL;
    }

    if (k_folds > n_points) k_folds = n_points;

    double min_cv_error = HUGE_VAL;
    int best = 1;

    int fold;
    for (fold = 1; fold <= max_order && fold <= 5; fold++) {
        double total_error = 0.0;
        int valid_folds = 0;

        int k;
        for (k = 0; k < k_folds; k++) {
            /* Simple split: every k-th point is test */
            int n_train = 0, n_test = 0;
            cal_data_point_t train[100], test[100];

            int i;
            for (i = 0; i < n_points && n_train < 100 && n_test < 100; i++) {
                if (i % k_folds == k) {
                    test[n_test++] = points[i];
                } else {
                    train[n_train++] = points[i];
                }
            }

            if (n_train < fold + 1 || n_test < 1) continue;

            /* Fit polynomial on training data */
            double coeffs[8];
            if (calibration_polynomial_fit(train, n_train, fold, coeffs) != 0)
                continue;

            /* Test error on held-out fold */
            double fold_error = 0.0;
            for (i = 0; i < n_test; i++) {
                double x = test[i].measurand;
                double y_pred = 0.0;
                double xp = 1.0;
                int j;
                for (j = 0; j <= fold; j++) {
                    y_pred += coeffs[j] * xp;
                    xp *= x;
                }
                double resid = test[i].output_voltage - y_pred;
                fold_error += resid * resid;
            }
            total_error += sqrt(fold_error / n_test);
            valid_folds++;
        }

        if (valid_folds > 0) {
            double avg_error = total_error / valid_folds;
            if (avg_error < min_cv_error) {
                min_cv_error = avg_error;
                best = fold;
            }
        }
    }

    if (best_order) *best_order = best;
    return min_cv_error;
}

double calibration_lookup(double measurand,
                          const cal_data_point_t *points,
                          int n_points)
{
    /* Linear interpolation lookup in calibration table.
     *
     * Uses binary search to find the interval containing
     * the measurand value, then linear interpolation within
     * that interval.
     *
     * O(log N) search + O(1) interpolation.
     *
     * For evenly-spaced calibration tables, use direct indexing
     * for O(1) lookup.
     */

    if (points == NULL || n_points < 2) return 0.0;

    /* Binary search for interval */
    int lo = 0, hi = n_points - 1;

    if (measurand <= points[0].measurand) return points[0].output_voltage;
    if (measurand >= points[hi].measurand) return points[hi].output_voltage;

    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (points[mid].measurand <= measurand) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    /* Linear interpolation between lo and hi */
    double x0 = points[lo].measurand;
    double x1 = points[hi].measurand;
    double y0 = points[lo].output_voltage;
    double y1 = points[hi].output_voltage;

    double t = (measurand - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
}

void cal_data_point_init(cal_data_point_t *point, double measurand,
                         double output_v, double excitation_v,
                         double temp_c)
{
    memset(point, 0, sizeof(*point));
    point->measurand      = measurand;
    point->output_voltage = output_v;
    point->excitation     = excitation_v;
    point->temperature_c  = temp_c;
}

void calibration_result_init(calibration_result_t *result)
{
    memset(result, 0, sizeof(*result));
    result->method = CAL_TWO_POINT;
    result->temperature_c = 25.0;
    result->r_squared = 1.0;
}
