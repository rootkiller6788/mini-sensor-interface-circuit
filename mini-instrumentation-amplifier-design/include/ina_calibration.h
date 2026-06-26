/**
 * @file ina_calibration.h
 * @brief Calibration and Error Correction for IA-based Systems
 *
 * Covers L5 algorithms (calibration methods) and L6 canonical problems
 * (system calibration, gain/offset error correction).
 *
 * Reference:
 *   Malaric, "Instrumentation and Measurement in Electrical Engineering"
 *   Taylor & Kuyatt, "NIST Technical Note 1297: Guidelines for Evaluating
 *     and Expressing the Uncertainty of NIST Measurement Results"
 *   ISO/IEC 17025: General requirements for testing and calibration
 *
 * @course-alignment
 *   MIT 2.671 Measurement and Instrumentation: Calibration methods
 *   Berkeley EE143 Microfabrication Technology: Process calibration
 *   Michigan EECS 461: Sensor calibration in embedded control
 *   ETH Zurich 227-0455: Precision analog system calibration
 */
#ifndef INA_CALIBRATION_H
#define INA_CALIBRATION_H

#include "ina_core.h"
#include <stdint.h>

/*---------------------------------------------------------------------------
 * L1: Calibration Terms and Definitions
 *---------------------------------------------------------------------------*/

/**
 * @brief Calibration point (x, y) for fitting
 *
 * Represents a known input (reference) and measured output pair.
 */
typedef struct {
    double input_value;       /**< Known reference input value */
    double measured_value;    /**< System measured output */
    double uncertainty;       /**< Measurement uncertainty (1-sigma) */
} CalPoint;

/**
 * @brief Gain/offset calibration model
 *
 * Linear model: y_corrected = (y_raw - offset) / gain
 *
 * This is the most common calibration for IA systems.
 * Two calibration points (typically zero and full-scale) are
 * sufficient to determine offset and gain.
 */
typedef struct {
    double offset;            /**< Offset (y-intercept in raw units) */
    double gain;              /**< Gain (slope, dimensionless) */
    double offset_uncertainty;/**< Uncertainty in offset (1-sigma) */
    double gain_uncertainty;  /**< Uncertainty in gain (1-sigma) */
    double r_squared;         /**< R? goodness of fit */
    int    num_points;        /**< Number of calibration points used */
} CalLinearModel;

/**
 * @brief Polynomial calibration model for nonlinear sensors
 *
 * y_corrected = c0 + c1*x + c2*x^2 + ... + cn*x^n
 *
 * Used when the IA + sensor system has significant nonlinearity.
 * RTDs and thermocouples are inherently nonlinear and benefit from
 * polynomial calibration.
 */
typedef struct {
    double coefficients[7];    /**< Polynomial coefficients c0..c6 */
    int    order;              /**< Polynomial order (1-6) */
    double r_squared;          /**< Goodness of fit */
    double max_residual;       /**< Maximum residual (in output units) */
} CalPolynomialModel;

/*---------------------------------------------------------------------------
 * L5: Two-Point Calibration (Gain + Offset Correction)
 *---------------------------------------------------------------------------*/

/**
 * @brief Perform two-point (gain + offset) calibration
 *
 * Algorithm:
 *   1. Apply known zero input (or short input), measure output ? offset
 *   2. Apply known full-scale input, measure output
 *   3. Gain = (FS_ref - Zero_ref) / (FS_measured - Zero_measured)
 *   4. Corrected = (Raw - Offset) / Gain
 *
 * This is the fundamental calibration method per L5.
 * For an ideal IA:
 *   Vout = G * Vin + Voffset
 *   Vin_corrected = (Vout - Voffset) / G
 *
 * @param zero_input Known zero reference value
 * @param zero_measured Measured output at zero input
 * @param fs_input Known full-scale reference value
 * @param fs_measured Measured output at full-scale input
 * @return Calibration model
 */
CalLinearModel ina_calibrate_two_point(double zero_input,
                                        double zero_measured,
                                        double fs_input,
                                        double fs_measured);

/**
 * @brief Apply two-point calibration correction
 *
 * y_corrected = (y_raw - offset) / gain
 */
double ina_apply_linear_calibration(double raw_value,
                                     const CalLinearModel *cal);

/**
 * @brief Reverse calibration (convert calibrated output back to raw)
 *
 * y_raw = y_corrected * gain + offset
 *
 * Useful for testing and verification.
 */
double ina_reverse_linear_calibration(double corrected_value,
                                       const CalLinearModel *cal);

/*---------------------------------------------------------------------------
 * L5: Multi-Point Polynomial Calibration
 *---------------------------------------------------------------------------*/

/**
 * @brief Perform polynomial calibration using least squares
 *
 * Uses ordinary least squares (OLS) to fit a polynomial of given order
 * to a set of calibration points.
 *
 * The normal equation: (X^T * X) * c = X^T * y
 * Solved via Gaussian elimination (for order ? 6).
 *
 * This is a core L5 algorithm: polynomial regression for sensor
 * linearization.
 *
 * @param points Array of calibration points
 * @param num_points Number of points (must be > order)
 * @param order Polynomial order (1-6)
 * @return Polynomial calibration model
 */
CalPolynomialModel ina_calibrate_polynomial(const CalPoint *points,
                                              int num_points,
                                              int order);

/**
 * @brief Apply polynomial calibration correction
 *
 * y = ? ci * x^i  for i = 0..order
 */
double ina_apply_polynomial_calibration(double raw_value,
                                         const CalPolynomialModel *cal);

/**
 * @brief Compute residual error at each calibration point
 *
 * Stores residuals in the CalPoint array for analysis.
 * RMS residual indicates calibration quality.
 */
double ina_calibration_rms_residual(const CalPoint *points,
                                     int num_points,
                                     const CalPolynomialModel *cal);

/*---------------------------------------------------------------------------
 * L5: Temperature Compensation
 *---------------------------------------------------------------------------*/

/**
 * @brief Temperature compensation parameters
 *
 * IA gain and offset drift with temperature. Calibration at multiple
 * temperatures enables temperature-compensated measurements.
 *
 * Compensation model:
 *   Gain(T) = Gain(T0) * (1 + TC_Gain * (T - T0))
 *   Offset(T) = Offset(T0) + TC_Offset * (T - T0)
 */
typedef struct {
    double gain_tc_ppm_per_c;      /**< Gain temperature coefficient */
    double offset_tc_uv_per_c;     /**< Offset temperature coefficient */
    double reference_temperature;  /**< Reference temperature T0 (?C) */
    double gain_at_ref;            /**< Gain at reference temperature */
    double offset_at_ref;          /**< Offset at reference temperature (?V) */
} TempCompensation;

/**
 * @brief Perform temperature calibration
 *
 * Measures gain and offset at two temperatures, computes
 * temperature coefficients.
 *
 * @param cal_low Calibration at low temperature
 * @param temp_low Low temperature (?C)
 * @param cal_high Calibration at high temperature
 * @param temp_high High temperature (?C)
 * @return Temperature compensation model
 */
TempCompensation ina_calibrate_temperature(const CalLinearModel *cal_low,
                                            double temp_low,
                                            const CalLinearModel *cal_high,
                                            double temp_high);

/**
 * @brief Apply temperature compensation to gain
 *
 * gain(T) = gain_ref * (1 + TC * 1e-6 * (T - Tref))
 */
double ina_compensated_gain(double gain_at_ref, double tc_ppm_per_c,
                             double temperature, double ref_temperature);

/**
 * @brief Apply temperature compensation to offset
 *
 * offset(T) = offset_ref + TC * (T - Tref)
 */
double ina_compensated_offset(double offset_at_ref, double tc_per_c,
                               double temperature, double ref_temperature);

/*---------------------------------------------------------------------------
 * L6: Auto-Zero Calibration Routine
 *---------------------------------------------------------------------------*/

/**
 * @brief Auto-zero configuration
 *
 * Auto-zeroing removes offset and low-frequency drift by periodically
 * shorting inputs and measuring the residual (which is the total offset).
 */
typedef struct {
    int enabled;                /**< Auto-zero enable flag */
    double interval_seconds;    /**< Time between auto-zero cycles (s) */
    double average_samples;     /**< Number of samples to average */
    double settle_time_us;      /**< Settling time after switching (?s) */
    double stored_offset;       /**< Last measured offset (?V) */
    double max_correction_uv;   /**< Maximum allowed correction (?V) */
} AutoZeroConfig;

/**
 * @brief Perform auto-zero measurement
 *
 * This function models the auto-zero process:
 *   1. Short amplifier inputs via MUX
 *   2. Wait for settling
 *   3. Measure output = total system offset (referred to output)
 *   4. Store offset for subsequent subtraction
 *
 * In a real system, this is implemented with an analog MUX
 * that shorts the IA inputs periodically.
 *
 * @param measured_samples Array of measured offset samples
 * @param num_samples Number of samples
 * @param gain Current IA gain setting
 * @return Computed input-referred offset (?V)
 */
double ina_auto_zero(const double *measured_samples,
                      int num_samples,
                      double gain);

/**
 * @brief Check if auto-zero correction is within valid range
 *
 * If the measured offset exceeds max_correction, it may indicate
 * a fault condition rather than normal drift.
 */
int ina_auto_zero_valid(double measured_offset_uv,
                         const AutoZeroConfig *config);

/*---------------------------------------------------------------------------
 * L6: Gain Error Calibration
 *---------------------------------------------------------------------------*/

/**
 * @brief Measure actual gain at a calibration point
 *
 * Inject known differential voltage and measure output:
 *   G_actual = (Vout_measured - Voffset) / Vin_known
 */
double ina_measure_actual_gain(double vin_known,
                                double vout_measured,
                                double known_offset);

/**
 * @brief Compute gain correction factor
 *
 * G_correction = G_nominal / G_actual
 * Corrected output = (V_raw - Offset) * G_correction / G_nominal
 */
double ina_gain_correction_factor(double g_nominal, double g_actual);

/**
 * @brief Compute gain error in percent
 *
 * Gain_error% = (G_actual - G_nominal) / G_nominal * 100
 */
double ina_gain_error_percent(double g_nominal, double g_actual);

/*---------------------------------------------------------------------------
 * L7: System-Level Calibration Strategy
 *---------------------------------------------------------------------------*/

/**
 * @brief Factory calibration data
 *
 * Stored in non-volatile memory, contains calibration coefficients
 * determined during manufacturing.
 */
typedef struct {
    CalLinearModel gain_offset;        /**< Gain and offset calibration */
    CalPolynomialModel linearization;  /**< Nonlinearity correction */
    TempCompensation temp_comp;        /**< Temperature compensation */
    uint32_t calibration_date;         /**< Calibration date (epoch) */
    uint32_t calibration_due_date;     /**< Next calibration due date */
    char serial_number[32];            /**< Unit serial number */
    char operator_id[16];              /**< Calibration technician ID */
    int calibration_points_used;       /**< Number of points in calibration */
} FactoryCalibration;

/**
 * @brief Perform complete factory calibration
 *
 * Executes the full calibration sequence:
 *   1. Zero offset calibration (short input)
 *   2. Full-scale gain calibration
 *   3. Linearity sweep (multiple points)
 *   4. Temperature characterization (if thermal chamber available)
 *   5. Store calibration in non-volatile memory format
 */
FactoryCalibration ina_factory_calibrate(const CalPoint *linearity_points,
                                           int num_linearity_points,
                                           int poly_order,
                                           const CalLinearModel *cal_low_temp,
                                           double temp_low,
                                           const CalLinearModel *cal_high_temp,
                                           double temp_high);

/**
 * @brief Apply complete calibration chain to raw measurement
 *
 * Applies in order:
 *   1. Temperature-compensated offset removal
 *   2. Temperature-compensated gain correction
 *   3. Polynomial linearization
 *
 * This is the complete L7 application-level calibration pipeline.
 */
double ina_apply_full_calibration(double raw_value,
                                   double temperature,
                                   const FactoryCalibration *cal);

/**
 * @brief Compute calibration uncertainty (per NIST TN 1297 / GUM)
 *
 * Type A uncertainty (statistical):
 *   uA = s / sqrt(n)  where s = sample standard deviation
 *
 * Type B uncertainty (systematic):
 *   uB = tolerance / sqrt(3)  (uniform distribution)
 *   uB = tolerance / 2  (for 95% confidence normal)
 *
 * Combined: uc = sqrt(uA^2 + uB^2)
 * Expanded: U = k * uc  (k=2 for 95% confidence)
 */
typedef struct {
    double type_a_uncertainty;   /**< Statistical (Type A) uncertainty */
    double type_b_uncertainty;   /**< Systematic (Type B) uncertainty */
    double combined_uncertainty; /**< Combined standard uncertainty */
    double expanded_uncertainty; /**< Expanded uncertainty (k=2) */
    double coverage_factor;      /**< Coverage factor k */
} CalUncertainty;

CalUncertainty ina_calibration_uncertainty(const double *repeated_measures,
                                            int num_measures,
                                            double reference_uncertainty,
                                            double instrument_tolerance);

#endif /* INA_CALIBRATION_H */
