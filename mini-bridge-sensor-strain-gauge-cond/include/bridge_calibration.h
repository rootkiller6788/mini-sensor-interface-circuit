/**
 * bridge_calibration.h — Bridge Sensor Calibration Methods
 *
 * Knowledge Coverage:
 *   L2: Shunt calibration, dead-weight calibration, two-point cal
 *   L3: Least-squares fitting, calibration uncertainty analysis
 *   L5: Multi-point calibration, polynomial regression
 *   L6: Load cell calibration, pressure sensor calibration
 *
 * Reference:
 *   - ASTM E74 — "Standard Practice for Calibration of Force-Measuring
 *                  Instruments for Verifying the Force Indication"
 *   - ISO 376 — "Metallic Materials — Calibration of Force-Proving
 *                Instruments"
 *   - Micro-Measurements Tech Note TN-514, "Shunt Calibration"
 *
 * Course: TU Munich EI0430 Messtechnik, THU Sensor Technology
 */

#ifndef BRIDGE_CALIBRATION_H
#define BRIDGE_CALIBRATION_H

#include "bridge_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * L2: Calibration Methods
 * ======================================================================== */

/**
 * Calibration method enumeration
 */
typedef enum {
    CAL_SHUNT,          /* Shunt resistor calibration */
    CAL_DEAD_WEIGHT,    /* Dead weight / force standard */
    CAL_TWO_POINT,      /* Two-point (zero + span) */
    CAL_MULTI_POINT,    /* Multi-point with polynomial fit */
    CAL_COMPARISON,     /* Comparison to reference transducer */
    CAL_SELF_CAL        /* Built-in electrical calibration */
} calibration_method_t;

/**
 * Calibration data point — one measurement at known stimulus
 */
typedef struct {
    double measurand;       /* Known applied stimulus [engineering units] */
    double output_voltage;  /* Measured bridge output [V] */
    double excitation;      /* Excitation during measurement [V] */
    double temperature_c;   /* Temperature [degC] */
} cal_data_point_t;

/**
 * Calibration result — the complete output of a calibration run
 */
typedef struct {
    calibration_method_t method;
    cal_data_point_t *data_points;
    int    n_points;
    double slope;              /* Sensitivity [output units / measurand unit] */
    double offset;             /* Zero offset [output units] */
    double nonlinearity_percent; /* Max deviation from linear fit [%FS] */
    double hysteresis_percent;   /* Max loading/unloading difference [%FS] */
    double uncertainty_percent;  /* Expanded uncertainty (k=2) [%FS] */
    double r_squared;           /* R^2 goodness of linear fit */
    double temperature_c;       /* Calibration temperature [degC] */
    int    calibration_date;    /* YYYYMMDD format */
} calibration_result_t;

/**
 * Perform shunt calibration.
 *
 * Shunt calibration simulates a known strain by connecting a
 * precision resistor in parallel with one bridge arm:
 *
 *   R_parallel = R_gauge * R_shunt / (R_gauge + R_shunt)
 *
 * The simulated strain is:
 *   eps_sim = -(1/GF) * (R_shunt - R_parallel) / R_shunt
 *           = -(1/GF) * R_gauge / (R_gauge + R_shunt)
 *
 * For a common example: R_gauge=350 Ohm, R_shunt=350 kOhm, GF=2.0
 *   eps_sim = -(1/2) * 350/(350+350000)
 *           = -0.5 * 0.000999 = -499.5 microstrain
 *
 * Shunt cal is used for:
 * 1. Field verification of calibration without physical loading
 * 2. Remote calibration of inaccessible installations
 * 3. Quick system check before measurement
 *
 * @param gauge        Strain gauge parameters
 * @param shunt_r_ohm  Shunt resistor value [Ohm]
 * @param arm          Which bridge arm (1-4) gets shunt
 * @return             Simulated strain [microstrain]
 *
 * Complexity: O(1).
 * Reference: Micro-Measurements TN-514
 */
double calibration_shunt_strain(const strain_gauge_t *gauge,
                                double shunt_r_ohm, int arm);

/**
 * Perform two-point calibration (zero and span).
 *
 * The simplest quantitative calibration:
 *   slope = (output_span - output_zero) / (measurand_span - measurand_zero)
 *   offset = output_zero - slope * measurand_zero
 *
 * Assumes linear transfer function. Good for:
 * - Instruments with known linearity
 * - Quick field calibration
 * - Limited calibration equipment availability
 *
 * @param zero_point     Measurement at zero stimulus
 * @param span_point     Measurement at full-scale stimulus
 * @param result         Output: calibration result
 *
 * Complexity: O(1).
 */
void calibration_two_point(const cal_data_point_t *zero_point,
                           const cal_data_point_t *span_point,
                           calibration_result_t *result);

/**
 * Calculate shunt resistor value for desired simulated strain.
 *
 * Inverse of shunt calibration formula:
 *   R_shunt = R_gauge * (1 - GF*eps_sim) / (GF*eps_sim)
 *
 * For negative eps_sim (compression simulation), R_shunt is positive.
 *
 * Example: R_gauge=350, GF=2.0, want eps_sim=-1000 ue
 *   R_shunt = 350 * (1 - 2*(-0.001)) / (2*(-0.001))
 *           = 350 * 1.002 / (-0.002)
 *           = 350 * (-501) = -175350 Ohm → invalid (negative)
 *
 * Wait — this formula has sign issues. The correct approach:
 * For shunt across R1, we want R1 to DECREASE (tension simulated
 * for R1 if R1 normally increases with tension).
 *
 * Let's derive properly:
 *   R_effective = R_gauge || R_shunt = R_gauge*R_shunt/(R_gauge+R_shunt)
 *   delta_R = R_effective - R_gauge
 *   eps = (delta_R/R_gauge)/GF
 *
 *   R_shunt = R_gauge * (1/(1 + GF*eps) - 1)^(-1)
 *
 * @param gauge        Strain gauge parameters
 * @param strain_ue    Desired simulated strain [microstrain]
 * @param arm          Which bridge arm (1-4)
 * @return             Required shunt resistor [Ohm]
 *
 * Complexity: O(1).
 */
double calibration_shunt_resistor(const strain_gauge_t *gauge,
                                  double strain_ue, int arm);

/* ========================================================================
 * L3: Least-Squares Calibration Fitting
 * ======================================================================== */

/**
 * Linear regression (least-squares) calibration fit.
 *
 * For N data points (x_i, y_i):
 *
 *   slope = [N*sum(x_i*y_i) - sum(x_i)*sum(y_i)] /
 *           [N*sum(x_i^2) - (sum(x_i))^2]
 *
 *   offset = [sum(y_i) - slope*sum(x_i)] / N
 *
 *   R^2 = 1 - SS_res / SS_tot
 *   where SS_res = sum((y_i - y_pred_i)^2)
 *         SS_tot = sum((y_i - y_mean)^2)
 *
 * Standard error of the slope:
 *   SE_slope = sqrt[SS_res/(N-2) / sum((x_i-x_mean)^2)]
 *
 * @param points       Array of calibration data points
 * @param n_points     Number of points
 * @param result       Output: filled calibration result
 * @return             0 on success, -1 if n_points < 2
 *
 * Complexity: O(N) where N = number of points.
 */
int calibration_linear_fit(const cal_data_point_t *points,
                           int n_points,
                           calibration_result_t *result);

/**
 * Polynomial regression calibration fit.
 *
 * Fits y = c0 + c1*x + c2*x^2 + ... + ck*x^k
 * using least-squares polynomial regression.
 *
 * Solves the normal equations: (X^T * X) * c = X^T * y
 * where X is the Vandermonde matrix.
 *
 * Uses Gaussian elimination with partial pivoting for numerical
 * stability at moderate polynomial orders (k <= 5).
 *
 * @param points       Calibration data points
 * @param n_points     Number of points (>= order+1)
 * @param order        Polynomial order (1-5)
 * @param coeffs       Output: fitted coefficients [order+1]
 * @return             0 on success, -1 on singular matrix
 *
 * Complexity: O(N*k^2 + k^3) where k = order.
 */
int calibration_polynomial_fit(const cal_data_point_t *points,
                               int n_points, int order, double *coeffs);

/**
 * Compute calibration uncertainty (GUM method).
 *
 * According to ISO/IEC Guide 98-3 (GUM):
 *
 *   Type A (statistical): u_A = s / sqrt(N)
 *   Type B (systematic): u_B from instrument specs
 *   Combined: u_c = sqrt(u_A^2 + u_B^2)
 *   Expanded (k=2, 95% confidence): U = 2 * u_c
 *
 * Sources of uncertainty in bridge calibration:
 * - Reference standard uncertainty
 * - Repeatability (Type A)
 * - Temperature effects
 * - Excitation stability
 * - Amplifier gain uncertainty
 * - ADC quantization
 *
 * @param stddev_repeat  Standard deviation of repeated measurements [ue]
 * @param n_repeats      Number of repeated measurements
 * @param ref_uncert_pct Uncertainty of reference standard [%]
 * @param temp_uncert_ue Temperature-induced uncertainty [ue]
 * @return               Expanded uncertainty (k=2) [ue]
 *
 * Complexity: O(1).
 */
double calibration_uncertainty(double stddev_repeat, int n_repeats,
                               double ref_uncert_pct, double temp_uncert_ue);

/**
 * Temperature compensation for calibration data.
 *
 * Corrects calibration coefficients for temperature:
 *
 *   slope(T) = slope(T0) * [1 + TC_slope * (T - T0)]
 *   offset(T) = offset(T0) + TC_offset * (T - T0)
 *
 * where TC_slope and TC_offset are determined by calibration
 * at multiple temperatures.
 *
 * @param result          Calibration at reference temperature
 * @param temperature_c   Current temperature [degC]
 * @param tc_slope_ppm    Temperature coeff of sensitivity [ppm/K]
 * @param tc_offset_ue_per_c  Temperature coeff of zero [ue/K]
 *
 * Complexity: O(1).
 */
void calibration_temp_compensate(calibration_result_t *result,
                                 double temperature_c,
                                 double tc_slope_ppm,
                                 double tc_offset_ue_per_c);

/* ========================================================================
 * L5: Advanced Calibration Methods
 * ======================================================================== */

/**
 * Cross-validation for calibration model selection.
 *
 * K-fold cross-validation splits data into K subsets, trains on
 * K-1 subsets and tests on the remaining one, repeating K times.
 * The model with lowest average prediction error is selected.
 *
 * This prevents over-fitting when choosing polynomial order.
 *
 * @param points         Calibration data array
 * @param n_points       Total number of points
 * @param max_order      Maximum polynomial order to test
 * @param k_folds        Number of CV folds (typ 5 or 10)
 * @param best_order     Output: recommended polynomial order
 * @return               Minimum cross-validation error [ue RMS]
 *
 * Complexity: O(K * max_order * N * ord^2).
 */
double calibration_cross_validate(const cal_data_point_t *points,
                                  int n_points, int max_order,
                                  int k_folds, int *best_order);

/**
 * Interpolation-based calibration lookup.
 *
 * For highly nonlinear sensors where polynomial fit is inadequate,
 * piecewise linear interpolation between calibration points provides
 * a model-free transfer function.
 *
 * Uses binary search for O(log N) per lookup.
 *
 * @param measurand    Measured output value [engineering units]
 * @param points       Sorted calibration data (by measurand)
 * @param n_points     Number of calibration points
 * @return             Interpolated measurand value
 *
 * Complexity: O(log N).
 */
double calibration_lookup(double measurand,
                          const cal_data_point_t *points,
                          int n_points);

/**
 * Initialize calibration data point with values.
 */
void cal_data_point_init(cal_data_point_t *point, double measurand,
                         double output_v, double excitation_v,
                         double temp_c);

/**
 * Initialize calibration result structure.
 */
void calibration_result_init(calibration_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_CALIBRATION_H */
