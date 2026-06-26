/**
 * @file    tc_rtd.c
 * @brief   RTD sensor interface: Callendar-Van Dusen equation implementation,
 *          temperature-to-resistance and resistance-to-temperature conversion,
 *          self-heating correction, wiring configuration support, and
 *          ratio-metric measurement.
 *
 * Knowledge Coverage:
 *   L1: RTD type definitions, CVD coefficients, alpha standards (IEC 60751)
 *   L2: Callendar-Van Dusen equation, TCR physics, quadratic/Newton inversion
 *   L3: Newton-Raphson for quartic CVD equation, quadratic formula for T>=0
 *   L4: IEC 60751 standard, Callendar model derivation, Van Dusen extension
 *   L5: Self-heating correction, alpha computation, uncertainty propagation
 *   L6: 4-wire/3-wire/2-wire measurement, ratio-metric technique
 *
 * Reference:
 *   IEC 60751:2008 Industrial Platinum Resistance Thermometers
 *   Callendar, H.L. (1887) "On the Practical Measurement of Temperature"
 *     Philosophical Transactions of the Royal Society A, vol. 178, pp. 161-230
 *   Van Dusen, M.S. (1925) "Platinum-Resistance Thermometry at Low Temperatures"
 *     Journal of the American Chemical Society, vol. 47, pp. 326-332
 *   ASTM E1137/E1137M-17 Standard Specification for Industrial Platinum
 *     Resistance Thermometers
 */

#include "thermocouple_cjc_rtd.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* =========================================================================
 * L1: RTD Metadata Tables
 * ========================================================================= */

/** Nominal resistance at 0C for each RTD type */
static const double rtd_r0_values[RTD_COUNT] = {
    100.0,    /* PT100 */
    200.0,    /* PT200 */
    500.0,    /* PT500 */
    1000.0,   /* PT1000 */
    2000.0,   /* PT2000 */
    100.0,    /* NI100 (DIN 43760) */
    120.0,    /* NI120 */
    1000.0,   /* NI1000 */
     10.0,    /* CU10 (r0 at 25C) */
    100.0     /* CU100 */
};

/** Human-readable RTD type names */
static const char *rtd_names[RTD_COUNT] = {
    "Pt100", "Pt200", "Pt500", "Pt1000", "Pt2000",
    "Ni100 (DIN 43760)", "Ni120", "Ni1000",
    "Cu10 (at 25C)", "Cu100"
};

/** Valid temperature range per RTD type [C min, C max] */
static const double rtd_temp_min[RTD_COUNT] = {
    -200.0, -200.0, -200.0, -200.0, -200.0,   /* Pt family */
    -60.0,  -60.0,  -60.0,                     /* Ni family */
    -100.0, -100.0                              /* Cu family */
};

static const double rtd_temp_max[RTD_COUNT] = {
    850.0,  850.0,  850.0,  850.0,  850.0,    /* Pt family */
    180.0,  180.0,  180.0,                      /* Ni family */
    260.0,  260.0                               /* Cu family */
};

/* =========================================================================
 * L4: Standard Callendar-Van Dusen Coefficients
 * ========================================================================= */

/**
 * @brief IEC 60751 standard platinum (alpha = 0.00385055 /C)
 *
 * Derived from the Callendar-Van Dusen equation:
 *   A = alpha * (1 + delta/100)
 *   B = -alpha * delta / 10^4
 *   C = -alpha * beta / 10^8
 *
 * where delta = 1.49979, beta = 0.10863 for standard platinum.
 *
 * These values are traceable to ITS-90 fixed points:
 *   - Triple point of water:      0.01 C
 *   - Melting point of gallium:  29.7646 C
 *   - Freezing point of indium: 156.5985 C
 *   - Freezing point of tin:    231.928 C
 *   - Freezing point of zinc:   419.527 C
 *   - Freezing point of aluminum: 660.323 C
 *   - Freezing point of silver: 961.78 C
 */
static const rtd_cvd_coeffs_t cvd_iec_pt = {
    0.0,               /* r0 set per type at runtime */
    3.9083000000E-03,  /* A = alpha*(1+delta/100) */
   -5.7750000000E-07,  /* B = -alpha*delta/10000 */
   -4.1830000000E-12,  /* C = -alpha*beta/1e8 */
    0.00385055,        /* alpha */
    0.10863,           /* beta */
    1.49979            /* delta */
};

/** US Industrial Standard (alpha = 0.003916, common in legacy US systems) */
static const rtd_cvd_coeffs_t cvd_us_pt = {
    0.0,
    3.9786938627E-03,
   -5.8686153526E-07,
   -4.1666666667E-12,
    0.003916,
    0.11000,
    1.50583
};

/** Old US / SAMA standard (alpha = 0.003920) */
static const rtd_cvd_coeffs_t cvd_oldus_pt = {
    0.0,
    3.9827066360E-03,
   -5.8750000000E-07,
   -4.1500000000E-12,
    0.003920,
    0.11020,
    1.50700
};

/**
 * @brief Nickel RTD coefficients (DIN 43760)
 *
 * Nickel RTDs have a higher TCR than platinum (~0.00618 vs 0.00385)
 * but a more limited temperature range (-60 to +180C).
 * The Ni RTD response is inherently more nonlinear.
 */
static const rtd_cvd_coeffs_t cvd_ni = {
    0.0,
    5.4850000000E-03,
   -6.6500000000E-06,
    0.0,
    0.006180,
    0.0,
    0.0
};

/**
 * @brief Copper RTD coefficients
 *
 * Copper has a highly linear R-T characteristic up to ~260C
 * (B and C terms are effectively zero for practical purposes).
 * Low resistance values require 4-wire measurement.
 */
static const rtd_cvd_coeffs_t cvd_cu = {
    0.0,
    4.2740000000E-03,
    0.0,
    0.0,
    0.004270,
    0.0,
    0.0
};

/* =========================================================================
 * L2: RTD Coefficient Retrieval
 * ========================================================================= */

/**
 * @brief tc_rtd_get_coeffs: Get standard CVD coefficients for an RTD type
 *
 * Selects the appropriate coefficient template based on the RTD element
 * material (Pt, Ni, Cu) and the alpha coefficient standard. Sets the R0
 * field from the RTD type's nominal resistance at 0C.
 */
tc_error_t tc_rtd_get_coeffs(rtd_type_t rt_type,
                              rtd_alpha_standard_t alpha_std,
                              rtd_cvd_coeffs_t *coeffs) {
    const rtd_cvd_coeffs_t *tmpl;

    if (!coeffs) return TC_ERR_NULL_POINTER;
    if (rt_type >= RTD_COUNT) return TC_ERR_INVALID_TYPE;

    switch (rt_type) {
    case RTD_TYPE_PT100:
    case RTD_TYPE_PT200:
    case RTD_TYPE_PT500:
    case RTD_TYPE_PT1000:
    case RTD_TYPE_PT2000:
        switch (alpha_std) {
        case RTD_ALPHA_IEC_385: tmpl = &cvd_iec_pt;   break;
        case RTD_ALPHA_US_392:
        case RTD_ALPHA_JP_391:  tmpl = &cvd_us_pt;    break;
        case RTD_ALPHA_US_390:  tmpl = &cvd_oldus_pt; break;
        case RTD_ALPHA_CUSTOM:
        default:                tmpl = &cvd_iec_pt;    break;
        }
        break;
    case RTD_TYPE_NI100:
    case RTD_TYPE_NI120:
    case RTD_TYPE_NI1000:
        tmpl = &cvd_ni;
        break;
    case RTD_TYPE_CU10:
    case RTD_TYPE_CU100:
        tmpl = &cvd_cu;
        break;
    default:
        return TC_ERR_INVALID_TYPE;
    }

    memcpy(coeffs, tmpl, sizeof(rtd_cvd_coeffs_t));
    coeffs->r0 = rtd_r0_values[rt_type];
    return TC_OK;
}

/* =========================================================================
 * L4: Forward CVD - Temperature to Resistance
 * ========================================================================= */

/**
 * @brief tc_rtd_temp_to_r: Temperature to RTD resistance using CVD equation
 *
 * For T >= 0C:  R(T) = R0 * (1 + A*T + B*T^2)
 * For T <  0C:  R(T) = R0 * (1 + A*T + B*T^2 + C*(T-100)*T^3)
 *
 * The Van Dusen term C*(T-100)*T^3 accounts for the departure from
 * the quadratic Callendar model at low temperatures, caused by
 * increased lattice defect scattering in platinum.
 *
 * For nickel and copper RTDs, the C term is zero (different physics).
 *
 * Complexity: O(1) - 10 floating-point operations.
 */
tc_error_t tc_rtd_temp_to_r(const rtd_cvd_coeffs_t *coeffs,
                             double temp, double *r) {
    double ratio;

    if (!coeffs || !r) return TC_ERR_NULL_POINTER;
    if (coeffs->r0 <= 0.0) return TC_ERR_INVALID_TYPE;

    if (temp >= 0.0) {
        ratio = 1.0 + coeffs->a * temp + coeffs->b * temp * temp;
    } else {
        double t2 = temp * temp;
        double t3 = t2 * temp;
        ratio = 1.0 + coeffs->a * temp + coeffs->b * t2
                + coeffs->c * (temp - 100.0) * t3;
    }

    *r = coeffs->r0 * ratio;
    return TC_OK;
}

/* =========================================================================
 * L5: Inverse CVD - Resistance to Temperature
 * ========================================================================= */

/**
 * @brief tc_rtd_r_to_temp: RTD resistance to temperature
 *
 * For R >= R0 (T >= 0C), the CVD equation is quadratic and has
 * a closed-form solution:
 *   B*T^2 + A*T + (1 - R/R0) = 0
 *   T = [-A + sqrt(A^2 - 4*B*(1 - R/R0))] / (2*B)
 *
 * For R < R0 (T < 0C), the full quartic CVD equation requires
 * Newton-Raphson iteration:
 *   f(T) = R0*(1 + A*T + B*T^2 + C*(T-100)*T^3) - R
 *   f'(T) = R0*(A + 2*B*T + C*(4*T^3 - 300*T^2))
 *
 * For linear RTDs (copper: B=0, C=0), a simple inversion is used.
 *
 * Complexity: O(1) for T>=0, O(I) for T<0 with I<=20 iterations.
 */
tc_error_t tc_rtd_r_to_temp(const rtd_cvd_coeffs_t *coeffs,
                             double r, double *temp) {
    double ratio, r0, a, b, c_val;

    if (!coeffs || !temp) return TC_ERR_NULL_POINTER;
    if (coeffs->r0 <= 0.0) return TC_ERR_INVALID_TYPE;
    if (r <= 0.0) return TC_ERR_RESISTANCE_RANGE;

    r0 = coeffs->r0;
    ratio = r / r0;
    a = coeffs->a;
    b = coeffs->b;
    c_val = coeffs->c;

    if (ratio >= 1.0) {
        /* T >= 0: Quadratic closed-form solution */
        if (fabs(b) < 1e-20) {
            /* Linear RTD: T = (R/R0 - 1) / A */
            *temp = (ratio - 1.0) / a;
            return TC_OK;
        }
        {
            double disc = a * a - 4.0 * b * (1.0 - ratio);
            if (disc < 0.0) {
                *temp = 0.0;
                return TC_ERR_RESISTANCE_RANGE;
            }
            *temp = (-a + sqrt(disc)) / (2.0 * b);
        }
        return TC_OK;
    } else {
        /* T < 0: Newton-Raphson iteration on quartic CVD equation */
        double t_est = (ratio - 1.0) / a; /* Linear initial guess */
        size_t iter;

        for (iter = 0; iter < 20; iter++) {
            double t2 = t_est * t_est;
            double t3 = t2 * t_est;
            double f_val, f_deriv, t_new;

            f_val = r0 * (1.0 + a * t_est + b * t2
                          + c_val * (t_est - 100.0) * t3) - r;

            f_deriv = r0 * (a + 2.0 * b * t_est
                            + c_val * (4.0 * t3 - 300.0 * t2));

            if (fabs(f_deriv) < 1e-20) break;

            t_new = t_est - f_val / f_deriv;

            if (fabs(t_new - t_est) < 1e-10) {
                *temp = t_new;
                return TC_OK;
            }
            t_est = t_new;
        }
        *temp = t_est;
        return TC_ERR_CONVERGENCE;
    }
}

/* =========================================================================
 * L5: Self-Heating Correction
 * ========================================================================= */

/**
 * @brief tc_rtd_self_heating: Compute self-heating temperature rise
 *
 * I^2*R heating in the RTD element raises its temperature above ambient:
 *   Delta_T = I^2 * R * theta
 *
 * where theta is the thermal dissipation constant (K/W).
 *
 * IEC 60751 specifies that the measurement current should be <= 1 mA
 * for a Pt100 to limit self-heating to < 0.2C at 0C.
 *
 * For a Pt1000, the recommended maximum current is <= 0.3 mA.
 */
tc_error_t tc_rtd_self_heating(double r, double i_excite,
                                const rtd_self_heating_t *sh,
                                double *delta_t) {
    double power;

    if (!sh || !delta_t) return TC_ERR_NULL_POINTER;
    if (r <= 0.0 || i_excite < 0.0) return TC_ERR_RESISTANCE_RANGE;

    power = i_excite * i_excite * r;
    *delta_t = power * sh->dissipation_constant;

    if (i_excite > sh->max_current || power > sh->max_power) {
        if (*delta_t > 1.0) {
            return TC_ERR_SELF_HEATING;
        }
    }
    return TC_OK;
}

/* =========================================================================
 * L5: Alpha Coefficient Computation
 * ========================================================================= */

/**
 * @brief tc_rtd_compute_alpha: Compute alpha from two calibration points
 *
 * Definition: alpha = (R100 - R0) / (100 * R0)
 *
 * This is the fundamental temperature coefficient of resistance,
 * determined by measuring the RTD at the ice point (0C) and
 * boiling point of water (100C).
 *
 * For IEC 60751 standard Pt: R100/R0 = 1.385055, alpha = 0.00385055
 */
tc_error_t tc_rtd_compute_alpha(double r0, double r100, double *alpha) {
    if (!alpha) return TC_ERR_NULL_POINTER;
    if (r0 <= 0.0) return TC_ERR_RESISTANCE_RANGE;
    if (r100 <= r0) return TC_ERR_RESISTANCE_RANGE;
    *alpha = (r100 - r0) / (100.0 * r0);
    return TC_OK;
}

/* =========================================================================
 * L5: RTD Measurement Uncertainty
 * ========================================================================= */

/**
 * @brief tc_rtd_uncertainty: Estimate temperature measurement uncertainty
 *
 * Sensitivity: dT/dR = 1 / (dR/dT)
 * dR/dT = R0 * [A + 2*B*T + C*(4*T^3 - 300*T^2)] for T<0
 * dR/dT = R0 * (A + 2*B*T)                              for T>=0
 *
 * At 0C for Pt100: dR/dT = 100 * 0.0039083 = 0.39083 ohm/C
 *                   dT/dR = 1/0.39083 = 2.56 C/ohm
 *
 * Combined uncertainty (RSS):
 *   u_T = sqrt((dT/dR * u_R)^2 + u_CVD^2 + u_SH^2)
 */
tc_error_t tc_rtd_uncertainty(rtd_measurement_t *meas,
                               const rtd_cvd_coeffs_t *coeffs,
                               double r_unc) {
    double dR_dT, sens;

    if (!meas || !coeffs) return TC_ERR_NULL_POINTER;

    if (meas->temperature >= 0.0) {
        dR_dT = coeffs->r0 * (coeffs->a + 2.0 * coeffs->b * meas->temperature);
    } else {
        double t = meas->temperature;
        double t2 = t * t;
        double t3 = t2 * t;
        dR_dT = coeffs->r0 * (coeffs->a + 2.0 * coeffs->b * t
                + coeffs->c * (4.0 * t3 - 300.0 * t2));
    }

    if (fabs(dR_dT) < 1e-15) {
        meas->uncertainty = 999.0;
        return TC_ERR_CONVERGENCE;
    }

    sens = 1.0 / dR_dT;  /* C/ohm */

    /* Root-sum-square combination of uncertainty sources */
    meas->uncertainty = sqrt(
        (sens * r_unc) * (sens * r_unc)          /* Resistance measurement */
        + (0.01 * 0.01)                           /* CVD equation fit */
        + (meas->self_heating * 0.2) * (meas->self_heating * 0.2) /* SH correction */
    );
    return TC_OK;
}

/* =========================================================================
 * L6: 4-Wire (Kelvin) RTD Measurement
 * ========================================================================= */

/**
 * @brief tc_rtd_4wire_measurement: Kelvin 4-wire RTD measurement
 *
 * Forces a known current through the RTD via one pair of leads and
 * measures the voltage across the RTD via a separate pair.
 *
 * R_RTD = V_sense / I_force
 *
 * Since the sense leads carry negligible current, their resistance
 * does not affect the measurement. This eliminates the lead resistance
 * error entirely - the gold standard for precision RTD measurement.
 *
 * Used in: laboratory metrology, calibration standards, precision
 * industrial processes requiring <0.01C accuracy.
 */
tc_error_t tc_rtd_4wire_measurement(double v_sense, double i_force,
                                     const rtd_cvd_coeffs_t *coeffs,
                                     rtd_measurement_t *result) {
    double r_rtd;
    tc_error_t err;

    if (!coeffs || !result) return TC_ERR_NULL_POINTER;
    if (i_force <= 0.0 || v_sense < 0.0) return TC_ERR_RESISTANCE_RANGE;

    memset(result, 0, sizeof(*result));

    r_rtd = v_sense / i_force;
    result->resistance = r_rtd;
    result->excitation_current = i_force;
    result->wiring = WIRE_4_WIRE;
    result->lead_resistance = 0.0;
    result->power_dissipation = i_force * i_force * r_rtd;

    err = tc_rtd_r_to_temp(coeffs, r_rtd, &result->temperature);
    result->error = err;

    return err;
}

/* =========================================================================
 * L6: 3-Wire RTD Measurement
 * ========================================================================= */

/**
 * @brief tc_rtd_3wire_measurement: 3-wire RTD compensation
 *
 * Three-wire measurement compensates for lead resistance by assuming
 * the two current-carrying leads have equal resistance.
 *
 * Measurement:
 *   R_total1 = V_excite_pos / I_excite = R_RTD + R_lead1
 *   R_total2 = V_excite_neg / I_excite = R_lead2
 *   R_RTD = R_total1 - R_total2 = R_RTD + (R_lead1 - R_lead2)
 *
 * If R_lead1 = R_lead2, the lead resistance cancels exactly.
 * Any mismatch (lead_r_match) contributes directly to error.
 *
 * Common in industrial PLC and DCS temperature input modules.
 */
tc_error_t tc_rtd_3wire_measurement(double v_excite_pos, double v_sense,
                                     double v_excite_neg, double i_excite,
                                     double lead_r_match,
                                     const rtd_cvd_coeffs_t *coeffs,
                                     rtd_measurement_t *result) {
    double r_high_leg, r_low_leg, r_rtd;
    tc_error_t err;

    (void)v_sense; /* Sense voltage reserved for advanced 3-wire compensation */

    if (!coeffs || !result) return TC_ERR_NULL_POINTER;
    if (i_excite <= 0.0) return TC_ERR_RESISTANCE_RANGE;

    memset(result, 0, sizeof(*result));

    r_high_leg = v_excite_pos / i_excite;
    r_low_leg = v_excite_neg / i_excite;
    r_rtd = r_high_leg - r_low_leg - lead_r_match;

    if (r_rtd <= 0.0) {
        result->error = TC_ERR_RESISTANCE_RANGE;
        return TC_ERR_RESISTANCE_RANGE;
    }

    result->resistance = r_rtd;
    result->excitation_current = i_excite;
    result->wiring = WIRE_3_WIRE;
    result->lead_resistance = (r_high_leg + r_low_leg - r_rtd) / 2.0;
    result->power_dissipation = i_excite * i_excite * r_rtd;

    err = tc_rtd_r_to_temp(coeffs, r_rtd, &result->temperature);
    result->error = err;

    return err;
}

/* =========================================================================
 * L6: 2-Wire RTD Measurement
 * ========================================================================= */

/**
 * @brief tc_rtd_2wire_measurement: 2-wire RTD with lead compensation
 *
 * Simplest wiring. Measures total loop resistance and subtracts
 * estimated lead resistance.
 *
 * R_measured = R_RTD + 2*R_lead
 * R_RTD = R_measured - 2*R_lead_est
 *
 * Accuracy depends heavily on lead resistance matching and stability.
 * For Pt100: each ohm of uncompensated lead = ~2.6C error.
 *
 * This method should only be used when lead resistance is well-known
 * and stable, or when measurement accuracy requirements are relaxed
 * (>2C acceptable error).
 */
tc_error_t tc_rtd_2wire_measurement(double v_measured, double i_excite,
                                     double r_lead_est,
                                     const rtd_cvd_coeffs_t *coeffs,
                                     rtd_measurement_t *result) {
    double r_total, r_rtd;
    tc_error_t err;

    if (!coeffs || !result) return TC_ERR_NULL_POINTER;
    if (i_excite <= 0.0) return TC_ERR_RESISTANCE_RANGE;

    memset(result, 0, sizeof(*result));

    r_total = v_measured / i_excite;
    r_rtd = r_total - 2.0 * r_lead_est;

    if (r_rtd <= 0.0) {
        result->error = TC_ERR_RESISTANCE_RANGE;
        return TC_ERR_RESISTANCE_RANGE;
    }

    result->resistance = r_rtd;
    result->excitation_current = i_excite;
    result->wiring = WIRE_2_WIRE;
    result->lead_resistance = r_lead_est;

    err = tc_rtd_r_to_temp(coeffs, r_rtd, &result->temperature);
    result->error = err;

    return err;
}

/* =========================================================================
 * L6: Ratio-metric RTD Measurement
 * ========================================================================= */

/**
 * @brief tc_rtd_ratiometric: Precision ratio-metric RTD measurement
 *
 * Uses a precision reference resistor R_ref in series with the RTD.
 * Both voltages are measured with the same ADC reference, canceling
 * ADC reference drift and excitation current error:
 *
 *   I = V_ref / R_ref
 *   R_rtd = V_rtd / I = R_ref * (V_rtd / V_ref)
 *
 * This is the preferred method in high-accuracy RTD measurement ICs
 * (e.g., ADS1248, ADS1220, MAX31865) because the ratiometric
 * cancellation eliminates two major error sources simultaneously.
 *
 * The reference resistor should have low TCR (<5 ppm/C) and be
 * placed near the RTD for thermal tracking.
 */
tc_error_t tc_rtd_ratiometric(double v_rtd, double v_ref, double r_ref,
                               const rtd_cvd_coeffs_t *coeffs,
                               rtd_measurement_t *result) {
    double r_rtd, i_excite;
    tc_error_t err;

    if (!coeffs || !result) return TC_ERR_NULL_POINTER;
    if (v_ref <= 0.0 || r_ref <= 0.0) return TC_ERR_RESISTANCE_RANGE;

    memset(result, 0, sizeof(*result));

    i_excite = v_ref / r_ref;
    r_rtd = r_ref * (v_rtd / v_ref);

    result->resistance = r_rtd;
    result->excitation_current = i_excite;
    result->wiring = WIRE_4_WIRE;
    result->lead_resistance = 0.0;
    result->power_dissipation = i_excite * i_excite * r_rtd;

    err = tc_rtd_r_to_temp(coeffs, r_rtd, &result->temperature);
    result->error = err;

    return err;
}
