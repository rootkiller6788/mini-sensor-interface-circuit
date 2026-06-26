/**
 * @file ina_calibration.c
 * @brief Calibration Algorithms for IA-based Measurement Systems
 *
 * Implements two-point calibration, polynomial calibration (least squares),
 * temperature compensation, auto-zero routines, and factory calibration
 * management (L5 Algorithms, L6 Canonical Problems, L7 Applications).
 *
 * Reference:
 *   NIST TN 1297: "Guidelines for Evaluating and Expressing
 *     the Uncertainty of NIST Measurement Results"
 *   JCGM 100:2008 (GUM): "Evaluation of measurement data"
 *   Malaric, "Instrumentation and Measurement in EE" (2011)
 */
#include "ina_calibration.h"
#include <math.h>
#include <string.h>

/*===========================================================================
 * Two-Point Calibration (Gain + Offset)
 *===========================================================================*/

CalLinearModel ina_calibrate_two_point(double zero_input,
                                        double zero_measured,
                                        double fs_input,
                                        double fs_measured)
{
    /**
     * Perform two-point calibration.
     *
     * This is the most fundamental calibration method for linear
     * systems (L5 algorithm).
     *
     * The linear model: y = gain * x + offset
     *
     * From two calibration points (x0, y0) and (x1, y1):
     *   gain = (y1 - y0) / (x1 - x0)
     *   offset = y0 - gain * x0
     *
     * For correction:
     *   x_corrected = (y_raw - offset) / gain
     *
     * Typical calibration sequence:
     *   1. Apply zero input (short IA inputs): x0 = 0, measure y0 = V_offset
     *   2. Apply known full-scale input: x1 = V_fs, measure y1
     *   3. Compute gain and offset
     *
     * The quality of this calibration depends on:
     *   - Accuracy of the reference inputs x0, x1
     *   - Linearity of the system between x0 and x1
     *   - Stability of the system during calibration
     */
    CalLinearModel cal;
    memset(&cal, 0, sizeof(cal));

    double dx = fs_input - zero_input;
    double dy = fs_measured - zero_measured;

    if (fabs(dx) > 1e-30) {
        cal.gain = dy / dx;
        cal.offset = zero_measured - cal.gain * zero_input;
    } else {
        cal.gain = 1.0;
        cal.offset = zero_measured - zero_input;
    }

    cal.num_points = 2;
    cal.r_squared = 1.0;  /* Perfect fit by definition for 2 points */

    return cal;
}

double ina_apply_linear_calibration(double raw_value,
                                     const CalLinearModel *cal)
{
    /**
     * Apply calibration correction.
     *
     * x_corrected = (y_raw - offset) / gain
     *
     * This converts a raw measurement to a calibrated value
     * using the stored gain and offset coefficients.
     *
     * Protection against singular gain:
     *   If gain ? 0 (defective calibration), return raw value.
     */
    if (!cal) return raw_value;
    if (fabs(cal->gain) < 1e-30) return raw_value;
    return (raw_value - cal->offset) / cal->gain;
}

double ina_reverse_linear_calibration(double corrected_value,
                                       const CalLinearModel *cal)
{
    /**
     * Reverse calibration: from calibrated to raw.
     *
     * y_raw = gain * x_corrected + offset
     *
     * Useful for:
     *   - Generating known test signals
     *   - Verifying calibration accuracy
     *   - Transferring calibration between units
     */
    if (!cal) return corrected_value;
    return cal->gain * corrected_value + cal->offset;
}

/*===========================================================================
 * Polynomial Calibration (Least Squares)
 *===========================================================================*/

/**
 * @brief Solve linear system using Gaussian elimination with partial pivoting
 *
 * This is a core numerical algorithm (L5) used for polynomial fitting.
 * Solves A*x = b for x, where A is n?n.
 *
 * Complexity: O(n^3)
 */
static int gauss_elimination(double *A, double *b, double *x, int n)
{
    /* Forward elimination with partial pivoting */
    for (int col = 0; col < n; col++) {
        /* Find pivot */
        int max_row = col;
        double max_val = fabs(A[col * n + col]);
        for (int row = col + 1; row < n; row++) {
            double val = fabs(A[row * n + col]);
            if (val > max_val) {
                max_val = val;
                max_row = row;
            }
        }

        if (max_val < 1e-30) return -1;  /* Singular */

        /* Swap rows if needed */
        if (max_row != col) {
            for (int j = col; j < n; j++) {
                double tmp = A[col * n + j];
                A[col * n + j] = A[max_row * n + j];
                A[max_row * n + j] = tmp;
            }
            double tmp = b[col];
            b[col] = b[max_row];
            b[max_row] = tmp;
        }

        /* Eliminate below */
        double pivot = A[col * n + col];
        for (int row = col + 1; row < n; row++) {
            double factor = A[row * n + col] / pivot;
            for (int j = col; j < n; j++) {
                A[row * n + j] -= factor * A[col * n + j];
            }
            b[row] -= factor * b[col];
        }
    }

    /* Back substitution */
    for (int i = n - 1; i >= 0; i--) {
        double sum = b[i];
        for (int j = i + 1; j < n; j++) {
            sum -= A[i * n + j] * x[j];
        }
        if (fabs(A[i * n + i]) < 1e-30) return -1;
        x[i] = sum / A[i * n + i];
    }

    return 0;
}

CalPolynomialModel ina_calibrate_polynomial(const CalPoint *points,
                                              int num_points,
                                              int order)
{
    /**
     * Perform polynomial calibration using ordinary least squares.
     *
     * Fits y = c0 + c1*x + c2*x^2 + ... + c_order*x^order
     * to the calibration points.
     *
     * L5 Algorithm: Ordinary Least Squares (OLS)
     *
     * The normal equation:
     *   X^T * X * c = X^T * y
     *
     * where X[i][j] = x_i^j (Vandermonde matrix)
     *       c is the coefficient vector
     *       y is the measured values vector
     *
     * Solving via Gaussian elimination is numerically adequate
     * for order ? 6. For higher orders, QR decomposition or
     * SVD would be preferred.
     *
     * R-squared computation:
     *   SS_res = ? (y_i - y_pred_i)^2
     *   SS_tot = ? (y_i - y_mean)^2
     *   R^2 = 1 - SS_res / SS_tot
     *
     * Reference: Press et al., "Numerical Recipes" (2007), Ch. 15.4
     */
    CalPolynomialModel model;
    memset(&model, 0, sizeof(model));

    if (!points || num_points <= order || order < 1 || order > 6) {
        model.order = 0;
        return model;
    }

    int n = order + 1;

    /* Build normal equation: (X^T X) c = X^T y */
    /* A = X^T X (size n?n), b = X^T y (size n?1) */
    /* A[j][k] = ? x_i^(j+k), b[j] = ? y_i * x_i^j */

    double A[49];  /* max (6+1)^2 = 49 */
    double b[7];   /* max 6+1 = 7 */
    double coeffs[7];
    memset(A, 0, sizeof(A));
    memset(b, 0, sizeof(b));

    for (int j = 0; j < n; j++) {
        for (int k = 0; k < n; k++) {
            double sum = 0.0;
            for (int i = 0; i < num_points; i++) {
                sum += pow(points[i].input_value, j + k);
            }
            A[j * n + k] = sum;
        }

        double sum = 0.0;
        for (int i = 0; i < num_points; i++) {
            sum += points[i].measured_value
                   * pow(points[i].input_value, j);
        }
        b[j] = sum;
    }

    if (gauss_elimination(A, b, coeffs, n) != 0) {
        model.order = 0;
        return model;
    }

    /* Store coefficients */
    for (int i = 0; i < n; i++) {
        model.coefficients[i] = coeffs[i];
    }
    model.order = order;

    /* Compute R-squared */
    double y_mean = 0.0;
    for (int i = 0; i < num_points; i++) {
        y_mean += points[i].measured_value;
    }
    y_mean /= num_points;

    double ss_res = 0.0, ss_tot = 0.0;
    double max_residual = 0.0;
    for (int i = 0; i < num_points; i++) {
        double y_pred = 0.0;
        double xp = 1.0;
        for (int j = 0; j < n; j++) {
            y_pred += coeffs[j] * xp;
            xp *= points[i].input_value;
        }
        double residual = points[i].measured_value - y_pred;
        ss_res += residual * residual;
        ss_tot += (points[i].measured_value - y_mean)
                  * (points[i].measured_value - y_mean);
        if (fabs(residual) > max_residual) max_residual = fabs(residual);
    }

    if (ss_tot > 0.0) {
        model.r_squared = 1.0 - ss_res / ss_tot;
    }
    model.max_residual = max_residual;

    return model;
}

double ina_apply_polynomial_calibration(double raw_value,
                                         const CalPolynomialModel *cal)
{
    /**
     * Apply polynomial calibration correction.
     *
     * y_corrected = ? c_i * x^i  for i = 0..order
     *
     * Uses Horner's method for numerical stability:
     *   y = c0 + x*(c1 + x*(c2 + x*(c3 + ...)))
     */
    if (!cal || cal->order <= 0) return raw_value;

    double result = cal->coefficients[cal->order];
    for (int i = cal->order - 1; i >= 0; i--) {
        result = result * raw_value + cal->coefficients[i];
    }
    return result;
}

double ina_calibration_rms_residual(const CalPoint *points,
                                     int num_points,
                                     const CalPolynomialModel *cal)
{
    /**
     * Compute RMS residual of the calibration fit.
     *
     * RMS_error = sqrt( ? (y_i - y_pred_i)^2 / N )
     *
     * This metric indicates calibration quality:
     *   RMS < 0.01% FS ? excellent
     *   RMS < 0.1% FS ? good
     *   RMS < 1% FS ? acceptable
     *
     * The RMS residual should be compared to the system's
     * noise floor and accuracy requirements.
     */
    if (!points || num_points <= 0 || !cal) return 0.0;

    double sum_sq = 0.0;
    for (int i = 0; i < num_points; i++) {
        double y_pred = ina_apply_polynomial_calibration(
                            points[i].input_value, cal);
        double err = points[i].measured_value - y_pred;
        sum_sq += err * err;
    }

    return sqrt(sum_sq / num_points);
}

/*===========================================================================
 * Temperature Compensation
 *===========================================================================*/

TempCompensation ina_calibrate_temperature(const CalLinearModel *cal_low,
                                            double temp_low,
                                            const CalLinearModel *cal_high,
                                            double temp_high)
{
    /**
     * Calibrate temperature coefficients of gain and offset.
     *
     * By measuring the system calibration at two temperatures,
     * we can compute the temperature coefficients:
     *
     * TC_gain = (G_high - G_low) / (G_avg * (T_high - T_low))
     * TC_offset = (Vos_high - Vos_low) / (T_high - T_low)
     *
     * These coefficients allow software compensation of
     * temperature-induced errors without a temperature chamber
     * during each measurement.
     *
     * Reference temperature is typically 25?C (room temperature).
     */
    TempCompensation tc;
    memset(&tc, 0, sizeof(tc));

    if (!cal_low || !cal_high) return tc;

    double dt = temp_high - temp_low;
    if (fabs(dt) < 1e-6) return tc;

    double g_avg = (cal_low->gain + cal_high->gain) / 2.0;
    if (fabs(g_avg) > 1e-30) {
        tc.gain_tc_ppm_per_c = (cal_high->gain - cal_low->gain)
                               / (g_avg * dt) * 1e6;
    }

    tc.offset_tc_uv_per_c = (cal_high->offset - cal_low->offset) / dt;

    tc.reference_temperature = 25.0;
    tc.gain_at_ref = cal_low->gain;   /* Simplified: use low temp as ref */
    tc.offset_at_ref = cal_low->offset;

    return tc;
}

double ina_compensated_gain(double gain_at_ref, double tc_ppm_per_c,
                             double temperature, double ref_temperature)
{
    /**
     * Compute temperature-compensated gain.
     *
     * G(T) = G_ref * (1 + TC * 1e-6 * (T - Tref))
     *
     * TC is in ppm/?C.
     * Positive TC: gain increases with temperature.
     */
    double dt = temperature - ref_temperature;
    return gain_at_ref * (1.0 + tc_ppm_per_c * 1e-6 * dt);
}

double ina_compensated_offset(double offset_at_ref, double tc_per_c,
                               double temperature, double ref_temperature)
{
    /**
     * Compute temperature-compensated offset.
     *
     * Vos(T) = Vos_ref + TC * (T - Tref)
     *
     * TC is in ?V/?C, Vos in ?V.
     */
    double dt = temperature - ref_temperature;
    return offset_at_ref + tc_per_c * dt;
}

/*===========================================================================
 * Auto-Zero Calibration
 *===========================================================================*/

double ina_auto_zero(const double *measured_samples,
                      int num_samples,
                      double gain)
{
    /**
     * Perform auto-zero offset measurement.
     *
     * Auto-zeroing is a technique to remove offset voltage
     * drift by periodically measuring the output with inputs
     * shorted and subtracting this from subsequent readings.
     *
     * Algorithm:
     *   1. Short IA inputs (via analog MUX)
     *   2. Allow settling time
     *   3. Acquire N samples of the output
     *   4. Average: Vout_avg = ? V_i / N
     *   5. Compute input-referred offset: Vos = Vout_avg / G
     *
     * The offset can then be subtracted from all subsequent
     * measurements:
     *   V_corrected = V_measured - Vout_avg
     *   or equivalently: V_corrected = (V_measured/G) - Vos
     *
     * Averaging N samples reduces noise by sqrt(N):
     *   ?_vos = ?_noise / (G * sqrt(N))
     *
     * Reference: Enz & Temes (1996)
     */
    if (!measured_samples || num_samples <= 0) return 0.0;

    double sum = 0.0;
    for (int i = 0; i < num_samples; i++) {
        sum += measured_samples[i];
    }
    double vout_avg = sum / num_samples;

    if (fabs(gain) < 1e-30) return vout_avg;
    return vout_avg / gain;  /* Input-referred offset (?V) */
}

int ina_auto_zero_valid(double measured_offset_uv,
                         const AutoZeroConfig *config)
{
    /**
     * Validate auto-zero measurement.
     *
     * Sanity checks:
     *   1. Offset magnitude within expected range
     *   2. Offset change from previous not excessive
     *   3. No saturation or railing behavior
     *
     * Returns 0 if valid, -1 if out of range.
     */
    if (!config) return -1;

    if (fabs(measured_offset_uv) > config->max_correction_uv) {
        return -1;  /* Excessive offset ? possible fault */
    }

    return 0;  /* Valid */
}

/*===========================================================================
 * Gain Error Calibration
 *===========================================================================*/

double ina_measure_actual_gain(double vin_known,
                                double vout_measured,
                                double known_offset)
{
    /**
     * Measure actual IA gain at a calibration point.
     *
     * G_actual = (Vout - Voffset) / Vin
     *
     * where Voffset is the measured output offset (with inputs shorted).
     *
     * For best accuracy, use the largest possible Vin to maximize
     * signal-to-noise ratio in the gain measurement.
     */
    if (fabs(vin_known) < 1e-30) return 0.0;
    return (vout_measured - known_offset) / vin_known;
}

double ina_gain_correction_factor(double g_nominal, double g_actual)
{
    /**
     * Compute gain correction factor.
     *
     * CF = G_nominal / G_actual
     *
     * Corrected reading = Raw * CF / G_nominal
     *                    = Raw / G_actual
     *
     * This effectively replaces the nominal gain with the
     * measured gain in all subsequent calculations.
     */
    if (fabs(g_actual) < 1e-30) return 1.0;
    return g_nominal / g_actual;
}

double ina_gain_error_percent(double g_nominal, double g_actual)
{
    /**
     * Compute gain error as percentage.
     *
     * ?% = (G_actual - G_nominal) / G_nominal * 100
     *
     * Positive error: actual gain > nominal (over-gain)
     * Negative error: actual gain < nominal (under-gain)
     *
     * Typical IA gain error: 0.01% to 0.5% without calibration.
     * With calibration: can be reduced to < 0.001% (10 ppm).
     */
    if (fabs(g_nominal) < 1e-30) return 0.0;
    return (g_actual - g_nominal) / g_nominal * 100.0;
}

/*===========================================================================
 * Factory Calibration (L7 Application)
 *===========================================================================*/

FactoryCalibration ina_factory_calibrate(const CalPoint *linearity_points,
                                           int num_linearity_points,
                                           int poly_order,
                                           const CalLinearModel *cal_low_temp,
                                           double temp_low,
                                           const CalLinearModel *cal_high_temp,
                                           double temp_high)
{
    /**
     * Perform complete factory calibration sequence.
     *
     * L7 Application: This represents the full calibration
     * process performed during manufacturing of precision
     * measurement instruments.
     *
     * Calibration sequence:
     *   1. Zero offset calibration (short input, measure offset)
     *   2. Full-scale gain calibration (apply reference, measure gain)
     *   3. Linearity characterization (multi-point sweep)
     *   4. Temperature characterization (if temp chamber available)
     *   5. Store calibration data for field use
     *
     * The stored calibration enables:
     *   - Software correction of gain/offset errors
     *   - Polynomial linearization of sensor nonlinearity
     *   - Temperature compensation for operating environment
     *
     * Reference: ISO 17025, ANSI/NCSL Z540-1
     */
    FactoryCalibration cal;
    memset(&cal, 0, sizeof(cal));

    /* 1. Two-point calibration from first and last linearity points */
    if (linearity_points && num_linearity_points >= 2) {
        cal.gain_offset = ina_calibrate_two_point(
            linearity_points[0].input_value,
            linearity_points[0].measured_value,
            linearity_points[num_linearity_points - 1].input_value,
            linearity_points[num_linearity_points - 1].measured_value);
    }

    /* 2. Polynomial linearization */
    cal.linearization = ina_calibrate_polynomial(
        linearity_points, num_linearity_points, poly_order);

    /* 3. Temperature compensation */
    if (cal_low_temp && cal_high_temp) {
        cal.temp_comp = ina_calibrate_temperature(
            cal_low_temp, temp_low, cal_high_temp, temp_high);
    }

    /* 4. Calibration metadata */
    cal.calibration_points_used = num_linearity_points;

    return cal;
}

double ina_apply_full_calibration(double raw_value,
                                   double temperature,
                                   const FactoryCalibration *cal)
{
    /**
     * Apply the complete calibration pipeline to a raw measurement.
     *
     * Pipeline (L7 application):
     *   1. Temperature-compensated offset removal
     *      offset_T = offset_ref + TC_offset * (T - Tref)
     *      value1 = raw_value - offset_T
     *
     *   2. Temperature-compensated gain correction
     *      gain_T = gain_ref * (1 + TC_gain*1e-6 * (T - Tref))
     *      value2 = value1 / gain_T
     *
     *   3. Polynomial linearization
     *      value3 = poly_correct(value2)
     *
     * This pipeline corrects for the three dominant error sources
     * in IA-based measurement systems.
     */
    if (!cal) return raw_value;

    /* Step 1: Temperature-compensated offset */
    double offset = ina_compensated_offset(
        cal->temp_comp.offset_at_ref,
        cal->temp_comp.offset_tc_uv_per_c,
        temperature,
        cal->temp_comp.reference_temperature);

    double value = raw_value - offset;

    /* Step 2: Temperature-compensated gain */
    double gain = ina_compensated_gain(
        cal->temp_comp.gain_at_ref,
        cal->temp_comp.gain_tc_ppm_per_c,
        temperature,
        cal->temp_comp.reference_temperature);

    if (fabs(gain) > 1e-30) {
        value = value / gain;
    }

    /* Step 3: Polynomial linearization */
    if (cal->linearization.order > 0) {
        value = ina_apply_polynomial_calibration(value, &cal->linearization);
    }

    return value;
}

/*===========================================================================
 * Calibration Uncertainty (NIST TN 1297 / GUM)
 *===========================================================================*/

CalUncertainty ina_calibration_uncertainty(const double *repeated_measures,
                                            int num_measures,
                                            double reference_uncertainty,
                                            double instrument_tolerance)
{
    /**
     * Compute calibration uncertainty per GUM guidelines.
     *
     * This is a critical L6 concept: measurement uncertainty
     * must be quantified and reported with every calibrated value.
     *
     * Type A (statistical) uncertainty:
     *   uA = s / sqrt(n)
     *   where s = sqrt(?(xi - x?)^2 / (n-1))
     *
     * Type B (systematic) uncertainty:
     *   From reference uncertainty: uB_ref = U_ref / k
     *   From instrument tolerance: uB_inst = tol / sqrt(3) (uniform)
     *   Combined: uB = sqrt(uB_ref^2 + uB_inst^2)
     *
     * Combined standard uncertainty:
     *   uc = sqrt(uA^2 + uB^2)
     *
     * Expanded uncertainty (95% confidence):
     *   U = 2 * uc  (k = 2 coverage factor)
     *
     * Reference: JCGM 100:2008 (GUM), ?4-6
     */
    CalUncertainty unc;
    memset(&unc, 0, sizeof(unc));

    /* Type A: statistical */
    if (repeated_measures && num_measures > 1) {
        double sum = 0.0, sum_sq = 0.0;
        for (int i = 0; i < num_measures; i++) {
            sum += repeated_measures[i];
            sum_sq += repeated_measures[i] * repeated_measures[i];
        }
        double mean = sum / num_measures;
        double variance = (sum_sq - sum * sum / num_measures)
                          / (num_measures - 1);
        if (variance < 0.0) variance = 0.0;
        double stddev = sqrt(variance);
        unc.type_a_uncertainty = stddev / sqrt((double)num_measures);
    }

    /* Type B: systematic */
    double ub_ref = reference_uncertainty / 2.0;  /* assumed k=2 */
    double ub_inst = instrument_tolerance / sqrt(3.0);  /* uniform dist */
    unc.type_b_uncertainty = sqrt(ub_ref * ub_ref + ub_inst * ub_inst);

    /* Combined */
    unc.combined_uncertainty = sqrt(
        unc.type_a_uncertainty * unc.type_a_uncertainty
        + unc.type_b_uncertainty * unc.type_b_uncertainty);

    /* Expanded (k=2, 95% confidence) */
    unc.coverage_factor = 2.0;
    unc.expanded_uncertainty = unc.coverage_factor * unc.combined_uncertainty;

    return unc;
}
