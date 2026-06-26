/**
 * @file ina_sensor.c
 * @brief Sensor Bridge Interface Implementation
 *
 * Implements Wheatstone bridge, strain gauge, RTD, and thermocouple
 * signal conditioning algorithms (L6 Canonical Problems, L7 Applications).
 *
 * Reference:
 *   Fraden, "Handbook of Modern Sensors" (2016, 5th Ed.)
 *   NIST ITS-90 Thermocouple Reference Functions
 *   IEC 60751: Industrial Platinum Resistance Thermometers
 */
#include "ina_sensor.h"
#include <math.h>
#include <string.h>

/*===========================================================================
 * Wheatstone Bridge Analysis
 *===========================================================================*/

double bridge_differential_voltage(const BridgeSensor *bridge)
{
    /**
     * Compute the differential output voltage of a Wheatstone bridge.
     *
     * General bridge equation:
     *   V_diff = Vex * [R3/(R2+R3) - R4/(R1+R4)]
     *
     * For a quarter-bridge (R1=R_sensor, R2=R3=R4=R0):
     *   V_diff = Vex * [R0/(R0+R0) - R0/(Rs+R0)]
     *          = Vex * (0.5 - R0/(Rs+R0))
     *          = Vex * (Rs - R0) / (2*(Rs+R0))
     *
     * Let delta = (Rs-R0)/R0:
     *   V_diff = Vex * delta / (4 + 2*delta)
     *          ? Vex * delta / 4    (for small delta)
     *
     * For a full-bridge (push-pull, diagonal pairs):
     *   Rs1 = R0(1+delta), Rs2 = R0(1-delta)
     *   Rs3 = R0(1-delta), Rs4 = R0(1+delta)
     *   V_diff = Vex * delta    (exact, no approximation needed)
     *
     * The full bridge is inherently linear and has 4x sensitivity.
     *
     * Reference: Fraden, ?5.2
     */
    if (!bridge) return 0.0;

    double r1, r2, r3, r4;
    double r0 = bridge->nominal_resistance;

    switch (bridge->type) {
        case BRIDGE_QUARTER:
            r1 = bridge->sensor_resistance;  /* Active sensor */
            r2 = r0; r3 = r0; r4 = r0;
            break;
        case BRIDGE_HALF:
            /* Adjacent arms, same sense */
            r1 = bridge->sensor_resistance;
            r2 = bridge->sensor_2_resistance;
            r3 = r0; r4 = r0;
            break;
        case BRIDGE_HALF_OPPOSITE:
            /* Opposite arms */
            r1 = bridge->sensor_resistance;
            r2 = r0;
            r3 = r0;
            r4 = bridge->sensor_2_resistance;
            break;
        case BRIDGE_FULL:
            r1 = bridge->sensor_resistance;
            r2 = bridge->sensor_2_resistance;
            r3 = bridge->sensor_3_resistance;
            r4 = bridge->sensor_4_resistance;
            break;
        case BRIDGE_FULL_DIAGONAL:
            /* Push-pull diagonal: Rs1=R0(1+d), Rs2=R0(1-d),
               Rs3=R0(1-d), Rs4=R0(1+d) */
            r1 = bridge->sensor_resistance;
            r2 = bridge->sensor_2_resistance;
            r3 = bridge->sensor_3_resistance;
            r4 = bridge->sensor_4_resistance;
            break;
        default:
            return 0.0;
    }

    double vex = bridge->excitation_voltage;
    double v_plus  = vex * r3 / (r2 + r3);   /* Non-inverting node */
    double v_minus = vex * r4 / (r1 + r4);    /* Inverting node */

    return v_plus - v_minus;
}

double bridge_sensitivity(const BridgeSensor *bridge, double gauge_factor)
{
    /**
     * Compute bridge sensitivity: mV of output per V of excitation
     * per unit strain (or per unit delta_R/R for general sensors).
     *
     * Quarter-bridge:
     *   S = GF/4  (mV/V per unit strain)
     *
     * Half-bridge (adjacent, same sense):
     *   S = GF/2
     *
     * Full-bridge (push-pull):
     *   S = GF
     *
     * For GF = 2 (metal foil strain gauge):
     *   Quarter: S = 0.5 mV/V/??  (actually 0.5 mV/V per 1000 ??)
     *   Half:    S = 1.0 mV/V/??
     *   Full:    S = 2.0 mV/V/??
     *
     * Multiply by Vex to get mV output per unit strain:
     *   Vout(mV) = S * Vex * strain(??/1000)
     */
    if (!bridge) return 0.0;

    switch (bridge->type) {
        case BRIDGE_QUARTER:
            return gauge_factor / 4.0;
        case BRIDGE_HALF:
            return gauge_factor / 2.0;
        case BRIDGE_HALF_OPPOSITE:
            return gauge_factor / 2.0;
        case BRIDGE_FULL:
        case BRIDGE_FULL_DIAGONAL:
            return gauge_factor;
        default:
            return 0.0;
    }
}

double bridge_output_impedance(const BridgeSensor *bridge)
{
    /**
     * Thevenin equivalent output resistance of the bridge.
     *
     * This determines the source resistance seen by the IA,
     * which is critical for noise analysis.
     *
     * For quarter-bridge (all arms = R0):
     *   Rth = (R0||R0) || (R0||R0) = R0/4 || R0/4 = R0/2
     *   (Actually: looking into the two bridge output nodes,
     *    each node has R0||R0 = R0/2 to the supply rails,
     *    so Rth = R0/2 || R0/2 = R0, wait no...)
     *
     * Let's derive properly:
     *   Short Vex, look into output terminals:
     *   Top terminal: R2||R3 (to ground via shorted Vex)
     *   Bottom terminal: R1||R4 (to ground via shorted Vex)
     *   Total: Rth = (R2||R3) + (R1||R4)
     *
     * For balanced bridge with all arms = R0:
     *   Rth = R0/2 + R0/2 = R0
     *
     * Thermal noise: Vn = sqrt(4*k*T*Rth*BW)
     * For R0 = 350 ? @ 25?C, BW = 10 kHz:
     *   Vn = sqrt(4 * 1.38e-23 * 298 * 350 * 10000)
     *      = sqrt(5.76e-14) ? 240 nV RMS
     */
    if (!bridge) return 0.0;

    double r1 = bridge->sensor_resistance;
    double r2 = bridge->nominal_resistance;
    double r3 = bridge->nominal_resistance;
    double r4 = bridge->nominal_resistance;

    if (bridge->type >= BRIDGE_HALF) {
        r2 = bridge->sensor_2_resistance;
    }

    double r_top = (r2 * r3) / (r2 + r3);      /* R2||R3 */
    double r_bottom = (r1 * r4) / (r1 + r4);    /* R1||R4 */

    return r_top + r_bottom;
}

double bridge_nonlinearity_error(const BridgeSensor *bridge)
{
    /**
     * Compute bridge nonlinearity error in ppm.
     *
     * Quarter-bridge has inherent nonlinearity:
     *   V_actual = Vex * delta / (4 + 2*delta)
     *   V_linear = Vex * delta / 4   (small-delta approximation)
     *   Error = V_actual - V_linear = -Vex * delta^2 / (8 + 4*delta)
     *
     * NL_percent ? -50*delta (%) for small delta
     *             = -500000*delta ppm
     *
     * For delta = 0.005 (0.5% change): NL ? -0.25% = -2500 ppm
     * For delta = 0.001 (0.1% change): NL ? -0.05% = -500 ppm
     *
     * Half-bridge and full-bridge are inherently linear
     * (no approximation needed in their output equations).
     */
    if (!bridge) return 0.0;

    if (bridge->type == BRIDGE_QUARTER) {
        double r0 = bridge->nominal_resistance;
        if (r0 <= 0.0) return 0.0;
        double delta = (bridge->sensor_resistance - r0) / r0;
        double v_actual = bridge_differential_voltage(bridge);
        double vex = bridge->excitation_voltage;
        double v_linear = vex * delta / 4.0;

        if (fabs(v_linear) < 1e-30) return 0.0;
        return (v_actual - v_linear) / v_linear * 1e6;  /* ppm */
    }

    /* Half-bridge and full-bridge are linear */
    return 0.0;
}

double bridge_design_ia_gain(double bridge_output_max_v,
                              double adc_full_scale_v,
                              double noise_requirement_uv)
{
    /**
     * Design IA gain for bridge sensor interface.
     *
     * G = V_adc_fs / V_bridge_max
     *
     * Additional considerations:
     *   - Noise floor after gain should be < 1 LSB
     *   - CMRR at chosen gain must reject expected CM interference
     *   - Gain should not amplify offset beyond ADC range
     *
     * Gain margin: include 10-20% headroom to avoid clipping.
     */
    if (bridge_output_max_v <= 0.0) return 1.0;

    double g = adc_full_scale_v / bridge_output_max_v;

    /* Apply 15% headroom */
    g *= 0.85;

    return g;
}

/*===========================================================================
 * Strain Gauge Signal Conditioning
 *===========================================================================*/

double strain_to_resistance_change(double strain_ue,
                                    const StrainGauge *gauge)
{
    /**
     * Compute resistance change from strain.
     *
     * ?R = R0 * GF * ?
     *
     * where ? is strain (dimensionless, in ?e ? ? = strain_ue * 1e-6)
     *
     * For GF = 2, ? = 1000 ?e:
     *   ?R = R0 * 2 * 0.001 = 0.002 * R0
     *
     * For R0 = 350 ?: ?R = 0.7 ? (very small change!)
     * For R0 = 120 ?: ?R = 0.24 ?
     *
     * This tiny resistance change produces the microvolt-level
     * signals that IAs are designed to amplify.
     */
    if (!gauge) return 0.0;
    double strain = strain_ue * 1e-6;  /* ?? ? dimensionless */
    return gauge->nominal_resistance * gauge->gauge_factor * strain;
}

double strain_to_bridge_output(double strain_ue,
                                const StrainGauge *gauge,
                                BridgeType bridge_config)
{
    /**
     * Compute bridge output voltage for given strain.
     *
     * Quarter-bridge:
     *   Rs = R0 + ?R = R0(1 + GF*?)
     *   Vout = Vex * GF*? / 4
     *
     * Half-bridge (Poisson arrangement, adjacent arms):
     *   Rs1 = R0(1 + GF*?)       (axial gauge)
     *   Rs2 = R0(1 - ?*GF*?)    (transverse/compensation gauge)
     *   Vout = Vex * GF*?*(1+?) / 4
     *
     * Full-bridge (push-pull):
     *   Vout = Vex * GF * ?
     */
    if (!gauge) return 0.0;

    double strain = strain_ue * 1e-6;
    double delta_r = gauge->gauge_factor * strain;

    BridgeSensor bridge;
    memset(&bridge, 0, sizeof(bridge));
    bridge.type = bridge_config;
    bridge.excitation_voltage = gauge->excitation_voltage;
    bridge.nominal_resistance = gauge->nominal_resistance;

    switch (bridge_config) {
        case BRIDGE_QUARTER:
            bridge.sensor_resistance = gauge->nominal_resistance * (1.0 + delta_r);
            break;
        case BRIDGE_HALF:
            bridge.sensor_resistance = gauge->nominal_resistance * (1.0 + delta_r);
            bridge.sensor_2_resistance = gauge->nominal_resistance
                                         * (1.0 - gauge->poisson_ratio * delta_r);
            break;
        case BRIDGE_FULL:
        case BRIDGE_FULL_DIAGONAL:
            /* Push-pull: R1 and R4 increase, R2 and R3 decrease */
            bridge.sensor_resistance = gauge->nominal_resistance * (1.0 + delta_r);
            bridge.sensor_2_resistance = gauge->nominal_resistance * (1.0 - delta_r);
            bridge.sensor_3_resistance = gauge->nominal_resistance * (1.0 - delta_r);
            bridge.sensor_4_resistance = gauge->nominal_resistance * (1.0 + delta_r);
            break;
        default:
            return 0.0;
    }

    return bridge_differential_voltage(&bridge);
}

double bridge_output_to_strain(double v_bridge, double v_excitation,
                                double gauge_factor, BridgeType bridge_config)
{
    /**
     * Convert bridge output voltage back to strain.
     *
     * Inverse of strain_to_bridge_output.
     *
     * Quarter-bridge (small signal approximation):
     *   ? = 4 * Vout / (Vex * GF)
     *
     * Quarter-bridge (exact, solving the quadratic):
     *   Vout = Vex * delta / (4 + 2*delta)
     *   ? delta = 4*Vout / (Vex - 2*Vout)
     *   ? ? = delta / GF
     *
     * Full-bridge:
     *   ? = Vout / (Vex * GF)
     */
    if (v_excitation <= 0.0 || gauge_factor <= 0.0) return 0.0;

    double epsilon;
    switch (bridge_config) {
        case BRIDGE_QUARTER: {
            double denom = v_excitation - 2.0 * v_bridge;
            if (fabs(denom) < 1e-30) return 0.0;
            double delta = 4.0 * v_bridge / denom;
            epsilon = delta / gauge_factor;
            break;
        }
        case BRIDGE_HALF:
        case BRIDGE_HALF_OPPOSITE:
            epsilon = 2.0 * v_bridge / (v_excitation * gauge_factor);
            break;
        case BRIDGE_FULL:
        case BRIDGE_FULL_DIAGONAL:
            epsilon = v_bridge / (v_excitation * gauge_factor);
            break;
        default:
            return 0.0;
    }

    return epsilon * 1e6;  /* Convert to ?? */
}

double strain_to_stress_mpa(double strain_ue, double youngs_modulus_gpa)
{
    /**
     * Convert strain to stress using Hooke's Law.
     *
     * ? = E * ?
     *
     * where:
     *   ? = stress (Pa or MPa)
     *   E = Young's modulus (Pa or GPa)
     *   ? = strain (dimensionless)
     *
     * For steel (E = 200 GPa = 200,000 MPa):
     *   1000 ?? ? ? = 200,000 * 0.001 = 200 MPa
     *
     * For aluminum (E = 69 GPa):
     *   1000 ?? ? ? = 69 MPa
     */
    double strain = strain_ue * 1e-6;
    return youngs_modulus_gpa * 1000.0 * strain;  /* GPa ? MPa, strain dimensionless */
}

/*===========================================================================
 * RTD Interface (Callendar-Van Dusen Equation)
 *===========================================================================*/

double rtd_resistance_at_temperature(double temperature_c,
                                      const RtdSensor *sensor)
{
    /**
     * Compute RTD resistance using the Callendar-Van Dusen equation.
     *
     * IEC 60751 standard for platinum RTDs:
     *
     * For T ? 0?C:
     *   R(T) = R0 * (1 + A*T + B*T^2)
     *
     * For T < 0?C:
     *   R(T) = R0 * (1 + A*T + B*T^2 + C*(T-100)*T^3)
     *
     * Standard coefficients (platinum, ? = 0.00385):
     *   A = 3.9083 ? 10^-3 ?C^-1
     *   B = -5.775 ? 10^-7 ?C^-2
     *   C = -4.183 ? 10^-12 ?C^-4
     *
     * For PT100 at 100?C:
     *   R = 100 * (1 + 3.9083e-3*100 + (-5.775e-7)*10000)
     *     = 100 * (1 + 0.39083 - 0.005775)
     *     = 100 * 1.38506 = 138.506 ?
     *
     * Reference: IEC 60751:2008
     */
    if (!sensor) return 0.0;

    double t = temperature_c;
    double rt;

    if (t >= 0.0) {
        rt = sensor->r0 * (1.0 + sensor->coeff_a * t
                              + sensor->coeff_b * t * t);
    } else {
        rt = sensor->r0 * (1.0 + sensor->coeff_a * t
                              + sensor->coeff_b * t * t
                              + sensor->coeff_c * (t - 100.0) * t * t * t);
    }

    /* Add lead resistance for 2-wire connection */
    if (sensor->connection == RTD_2WIRE) {
        rt += 2.0 * sensor->lead_resistance;
    } else if (sensor->connection == RTD_3WIRE) {
        /* 3-wire: ideally cancels, but add small residual */
        rt += sensor->lead_resistance * 0.01;  /* 1% residual mismatch */
    }
    /* 4-wire: no lead error */

    return rt;
}

double rtd_temperature_from_resistance(double resistance,
                                        const RtdSensor *sensor)
{
    /**
     * Compute temperature from RTD resistance using Newton-Raphson iteration.
     *
     * We need to solve R(T) - R_measured = 0 for T.
     *
     * Newton-Raphson: T_{n+1} = T_n - f(T_n) / f'(T_n)
     *
     * where:
     *   f(T) = R(T) - R_measured
     *   f'(T) = R0 * (A + 2*B*T) for T >= 0
     *           R0 * (A + 2*B*T + C*(4*T-300)*T^2) for T < 0
     *
     * Initial guess from linear approximation:
     *   T0 = (R/R0 - 1) / A
     *
     * Typically converges in 3-5 iterations.
     */
    if (!sensor) return 0.0;

    /* Remove lead resistance effects first */
    double r_actual = resistance;
    if (sensor->connection == RTD_2WIRE) {
        r_actual -= 2.0 * sensor->lead_resistance;
    }

    /* Initial guess (linear) */
    double t = (r_actual / sensor->r0 - 1.0) / sensor->coeff_a;

    /* Newton-Raphson iteration */
    for (int iter = 0; iter < 20; iter++) {
        double rt, drdt;

        if (t >= 0.0) {
            rt = sensor->r0 * (1.0 + sensor->coeff_a * t
                                  + sensor->coeff_b * t * t);
            drdt = sensor->r0 * (sensor->coeff_a + 2.0 * sensor->coeff_b * t);
        } else {
            double t2 = t * t;
            double t3 = t2 * t;
            rt = sensor->r0 * (1.0 + sensor->coeff_a * t
                                  + sensor->coeff_b * t2
                                  + sensor->coeff_c * (t - 100.0) * t3);
            drdt = sensor->r0 * (sensor->coeff_a + 2.0 * sensor->coeff_b * t
                                 + sensor->coeff_c * (4.0 * t - 300.0) * t2);
        }

        double error = rt - r_actual;
        if (fabs(error) < 1e-8) break;

        if (fabs(drdt) < 1e-30) break;
        t = t - error / drdt;
    }

    return t;
}

double rtd_lead_error_temperature(double lead_resistance,
                                   RtdConnection connection,
                                   const RtdSensor *sensor)
{
    /**
     * Temperature error caused by lead resistance.
     *
     * For 2-wire:
     *   ?R = 2 * R_lead
     *   ?T ? ?R / (R0 * A)   (linear approximation)
     *
     * Example: PT100 with R_lead = 0.5 ? per wire (1 ? total):
     *   ?T ? 1 / (100 * 3.908e-3) ? 2.56?C  (huge error!)
     *
     * This is why 2-wire RTD is only used for short distances
     * or low-accuracy applications. 3-wire and 4-wire connections
     * virtually eliminate this error.
     */
    if (!sensor) return 0.0;

    double delta_r = 0.0;
    switch (connection) {
        case RTD_2WIRE:
            delta_r = 2.0 * lead_resistance;
            break;
        case RTD_3WIRE:
            delta_r = 0.0;  /* Ideally canceled */
            break;
        case RTD_4WIRE:
            delta_r = 0.0;  /* Kelvin connection */
            break;
    }

    return delta_r / (sensor->r0 * sensor->coeff_a);
}

double rtd_design_ia_gain(double t_min_c, double t_max_c,
                           const RtdSensor *sensor,
                           double adc_reference_v)
{
    /**
     * Design IA gain for RTD measurement.
     *
     * 1. Compute R_min and R_max over temperature range
     * 2. With excitation current I_exc:
     *      V_min = I_exc * R_min
     *      V_max = I_exc * R_max
     * 3. G = V_adc_fs / (V_max - V_min)
     *
     * For ratiometric operation, the excitation current is derived
     * from the ADC reference, so excitation drift cancels.
     *
     * Example: PT100, 0-100?C, I_exc = 1 mA, ADC_ref = 2.5V:
     *   R_0C = 100 ?, R_100C = 138.5 ?
     *   V_0C = 100 mV, V_100C = 138.5 mV
     *   ?V = 38.5 mV
     *   G = 2.5 / 0.0385 = 64.9
     */
    if (!sensor) return 1.0;

    /* Create a copy to change temperature */
    RtdSensor s = *sensor;
    s.connection = RTD_4WIRE;  /* Ignore lead effects for gain calc */

    double r_min = rtd_resistance_at_temperature(t_min_c, &s);
    double r_max = rtd_resistance_at_temperature(t_max_c, &s);

    double v_range = sensor->excitation_current * (r_max - r_min);

    if (v_range <= 0.0) return 1.0;
    return adc_reference_v / v_range * 0.85;  /* 15% margin */
}

void rtd_ratiometric_config(double *r_bias, double *r_ref,
                             const RtdSensor *sensor,
                             double adc_vref, double ia_gain)
{
    /**
     * Design ratiometric RTD measurement circuit.
     *
     * Ratiometric principle:
     *   I_exc = V_ref / R_bias
     *   V_rtd = I_exc * R_rtd = V_ref * R_rtd / R_bias
     *   ADC counts = (V_rtd * G_ia) / V_ref * 2^N
     *              = (R_rtd * G_ia / R_bias) * 2^N
     *
     * The V_ref cancels! ADC reading is independent of reference
     * voltage drift (to first order).
     *
     * R_bias = V_ref / I_exc
     * R_ref = (V_ref * R0) / (I_exc * R_bias) * scaling
     */
    if (!sensor || !r_bias || !r_ref) return;

    double i_exc = sensor->excitation_current;
    if (i_exc <= 0.0) return;

    *r_bias = adc_vref / i_exc;

    /* R_ref sets the zero-scale offset for the diff amp */
    *r_ref = sensor->r0;
}

/*===========================================================================
 * Thermocouple Interface (NIST ITS-90)
 *===========================================================================*/

/**
 * NIST ITS-90 thermocouple polynomial coefficient tables.
 *
 * Direct polynomials: T = ? c_i * V^i  (for i = 0..n)
 * Inverse polynomials: V = ? c_i * T^i (for i = 0..n)
 *
 * Each TC type has its own coefficient set and temperature range.
 * Reference: NIST Monograph 175 (1993)
 */

/**
 * Type K (Chromel-Alumel) inverse coefficients
 * Range: -200?C to 0?C, 0?C to 500?C, 500?C to 1372?C
 */
static const double tc_k_coeffs_0_to_500[] = {
     0.0,           /* c0 = 0 (at 0?C) */
     2.508355E+01,   /* c1 */
     7.860106E-02,   /* c2 */
    -2.503131E-01,  /* c3 */
     8.315270E-02,  /* c4 */
    -1.228034E-02,  /* c5 */
     9.804036E-04,  /* c6 */
    -4.413030E-05,  /* c7 */
     1.057734E-06,  /* c8 */
    -1.052755E-08   /* c9 */
};

static const int tc_k_ncoeffs = 10;

double thermocouple_seebeck(ThermocoupleType type, double temperature_c)
{
    /**
     * Compute Seebeck coefficient (thermoelectric sensitivity)
     * for a given TC type at a given temperature.
     *
     * The Seebeck coefficient S(T) = dV/dT.
     *
     * Approximate values near 25?C:
     *   Type K: ~40.6 ?V/?C
     *   Type J: ~51.7 ?V/?C
     *   Type T: ~40.9 ?V/?C
     *   Type E: ~61.8 ?V/?C
     *   Type N: ~26.9 ?V/?C (lower than K, better stability)
     *   Type B: ~0 ?V/?C at 0?C, ~6 ?V/?C at 600?C
     *   Type R: ~5.3 ?V/?C at 0?C
     *   Type S: ~5.4 ?V/?C at 0?C
     *
     * Platinum-based TCs (B, R, S) have much lower output but
     * can measure higher temperatures.
     *
     * For simplified implementation, return nominal coefficients.
     * A full implementation would use the NIST polynomial derivative.
     */
    fabs(temperature_c);  /* suppress unused warning for simple impl */

    switch (type) {
        case TC_TYPE_K: return 40.6;
        case TC_TYPE_J: return 51.7;
        case TC_TYPE_T: return 40.9;
        case TC_TYPE_E: return 61.8;
        case TC_TYPE_N: return 26.9;
        case TC_TYPE_B: return 6.0;
        case TC_TYPE_R: return 10.0;
        case TC_TYPE_S: return 10.0;
        default:        return 40.0;
    }
}

double thermocouple_voltage_to_temperature(ThermocoupleType type,
                                            double voltage_uv)
{
    /**
     * Convert thermocouple voltage (with CJC applied) to temperature
     * using NIST ITS-90 inverse polynomials.
     *
     * This is a simplified implementation using a polynomial
     * approximation. A production implementation would use
     * the full segmented NIST polynomial tables.
     *
     * For Type K (0?C to 500?C range):
     *   T = c0 + c1*V + c2*V^2 + c3*V^3 + ... + c9*V^9
     *   where V is in mV (voltage_uv / 1000)
     *
     * For a simple linear approximation:
     *   T ? V_uv / S(T)  where S is the Seebeck coefficient
     *
     * This is sufficient for first-order estimates.
     */
    double v_mv = voltage_uv / 1000.0;

    /* Linear approximation based on nominal Seebeck coefficient */
    double seebeck = thermocouple_seebeck(type, 25.0);

    if (fabs(seebeck) < 1e-30) return 0.0;

    double t_linear = voltage_uv / seebeck;

    /* For Type K with polynomial (0 to 500?C) */
    if (type == TC_TYPE_K && t_linear >= 0.0 && t_linear <= 500.0) {
        double t_poly = 0.0;
        double v_pow = 1.0;
        for (int i = 0; i < tc_k_ncoeffs; i++) {
            t_poly += tc_k_coeffs_0_to_500[i] * v_pow;
            v_pow *= v_mv;
        }
        return t_poly;
    }

    return t_linear;
}

double thermocouple_temperature_to_voltage(ThermocoupleType type,
                                            double temperature_c)
{
    /**
     * Convert temperature to thermocouple voltage.
     *
     * Forward NIST ITS-90 polynomial.
     * This is used in CJC to convert cold junction temperature
     * to equivalent voltage.
     *
     * Simple linear approximation:
     *   V_uv = S * T
     *
     * Where S depends on temperature (Seebeck coefficient varies
     * with temperature, but for narrow ranges the nominal value
     * is adequate).
     */
    double seebeck = thermocouple_seebeck(type, temperature_c);
    return seebeck * temperature_c;
}

double thermocouple_cjc(ThermocoupleType type,
                         double measured_voltage_uv,
                         double cold_junction_temperature_c)
{
    /**
     * Cold Junction Compensation (CJC) algorithm.
     *
     * Thermocouples measure the temperature DIFFERENCE between
     * the hot junction (measurement point) and the cold junction
     * (where the TC wires connect to copper).
     *
     * CJC algorithm (L5):
     *   1. Measure T_cj (cold junction temperature, usually via
     *      thermistor or IC temperature sensor)
     *   2. Compute V_cj = f(T_cj)  -- voltage TC would produce
     *      if hot junction were at T_cj and cold at 0?C
     *   3. V_total = V_measured + V_cj
     *   4. T_hot = f^(-1)(V_total)
     *
     * The key insight: TC voltage is additive.
     *   V(T_hot, 0?C) = V(T_hot, T_cj) + V(T_cj, 0?C)
     *
     * Therefore:
     *   V_total = V_measured + V_cj
     *
     * Reference: NIST ITS-90, "NIST Monograph 175"
     *
     * Example (Type K, T_cj = 25?C, V_measured = 4.096 mV):
     *   V_cj = 40.6 * 25 = 1015 ?V
     *   V_total = 4096 + 1015 = 5111 ?V
     *   T_hot ? 5111 / 40.6 ? 125.9?C
     */
    /* Step 2: cold junction voltage */
    double v_cj_uv = thermocouple_temperature_to_voltage(type,
                                          cold_junction_temperature_c);

    /* Step 3: total equivalent voltage */
    double v_total_uv = measured_voltage_uv + v_cj_uv;

    /* Step 4: convert to temperature */
    return thermocouple_voltage_to_temperature(type, v_total_uv);
}

double thermocouple_design_ia_gain(ThermocoupleType type,
                                    double temperature_range_c,
                                    double resolution_c,
                                    double adc_lsb_v)
{
    /**
     * Design IA gain for thermocouple measurement.
     *
     * TC output is small (tens of ?V/?C). High gain is needed.
     *
     * V_per_degree = S(T)
     * V_per_resolution = S * resolution
     * G_required = ADC_LSB / V_per_resolution
     *
     * Example: Type K, 0.1?C resolution, 12-bit ADC, 2.5V ref:
     *   V_per_0.1C = 40.6 * 0.1 = 4.06 ?V
     *   ADC_LSB = 2.5 / 4096 = 610 ?V
     *   G = 610 / 4.06 = 150
     *
     * Practical gain range for TC: 50-500.
     */
    double seebeck = thermocouple_seebeck(type, 25.0);
    double v_signal = seebeck * resolution_c;  /* ?V */

    if (v_signal <= 0.0) return 1.0;

    double g = adc_lsb_v * 1e6 / v_signal;  /* convert adc_lsb V to ?V */
    return g;
}

int thermocouple_burnout_detect(double measured_voltage_uv,
                                 double expected_voltage_uv,
                                 double threshold_uv)
{
    /**
     * Detect thermocouple burnout (open circuit).
     *
     * When a TC opens, the measured voltage typically goes
     * to one of the supply rails (due to input bias current
     * flowing through the open circuit).
     *
     * Detection methods:
     *   1. Out-of-range voltage check
     *   2. Drift rate check
     *   3. Bias current test (inject current, check compliance)
     *
     * Returns 1 if burnout detected.
     */
    double deviation = fabs(measured_voltage_uv - expected_voltage_uv);
    return (deviation > threshold_uv) ? 1 : 0;
}
