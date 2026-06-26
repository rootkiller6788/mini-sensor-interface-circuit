/**
 * bridge_core.c - Wheatstone Bridge Core Implementation
 *
 * Implements fundamental bridge equations, strain gauge computations,
 * and initialization functions. Each function covers an independent
 * knowledge point from bridge circuit theory and stress analysis.
 *
 * Knowledge Coverage:
 *   L3: Bridge output equations (exact, linearized, inverse)
 *   L4: Hooke's law, piezoresistance, temperature effects
 *   L1/L2: Initialization and rosette resolution
 *
 * Reference:
 *   - Doebelin, "Measurement Systems", 5th ed., McGraw-Hill
 *   - Hoffmann, "Stress Analysis using Strain Gauges", HBM
 *   - Dally & Riley, "Experimental Stress Analysis", 4th ed.
 */

#include "bridge_core.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ========================================================================
 * L3: Bridge Output Equations - exact, linearized, inverse
 * ======================================================================== */

double bridge_output_voltage(const bridge_state_t *bridge)
{
    /* Christie-Wheatstone equation (1833/1843):
     * Vout = Vexc * [R3/(R1+R3) - R4/(R2+R4)]
     *
     * This is the exact equation with no approximations.
     * The bridge acts as a differential voltage divider.
     * When R1*R4 = R2*R3, the bridge is "balanced" and Vout = 0.
     */
    double denom_left  = bridge->r1 + bridge->r3;
    double denom_right = bridge->r2 + bridge->r4;

    if (denom_left <= 0.0 || denom_right <= 0.0) return 0.0;

    double v_left  = bridge->r3 / denom_left;
    double v_right = bridge->r4 / denom_right;
    return bridge->v_excitation * (v_left - v_right);
}

double bridge_output_linear(const bridge_state_t *bridge,
                            const double strains[4], double gf)
{
    /* Linearized bridge output for small strain (|GF*eps| << 1).
     * Vout = Vexc * GF/4 * (eps1 - eps2 + eps3 - eps4)
     * This is the first-order Taylor expansion around eps_i = 0.
     */
    double sum = strains[0] - strains[1] + strains[2] - strains[3];
    return bridge->v_excitation * (gf / 4.0) * sum;
}

double bridge_nonlinearity_error(double strain_ue, double gf,
                                 bridge_config_t config)
{
    /* Nonlinearity error as percentage of reading.
     *
     * Quarter bridge exact output: Vout/Vexc = 1/(2+GF*eps) - 1/2
     * Taylor expansion: = -GF*eps/4 + (GF*eps)^2/8 - ...
     * Fractional NL = (quadratic term)/(linear term) = GF*eps/2
     * NL(%) = 50*GF*eps
     *
     * For GF=2, eps=1000 ue: NL = 50*2*0.001 = 0.1% of reading.
     *
     * Half bridge: adjacent arms with equal+opposite strain
     * cancels first-order nonlinearity. Residual ~ (GF*eps)^2/4.
     *
     * Full bridge: exact linearity from symmetry.
     */
    double eps_abs = strain_ue * 1.0e-6;

    switch (config) {
        case BRIDGE_QUARTER:
        case BRIDGE_DOUBLE_QUARTER:
            return fabs(0.5 * gf * eps_abs * 100.0);
        case BRIDGE_HALF:
        case BRIDGE_BENDING: {
            double x = gf * eps_abs;
            return (x * x / 4.0) * 100.0;
        }
        case BRIDGE_FULL:
        case BRIDGE_TORSION:
        case BRIDGE_SHEAR:
            return 0.0;
        default:
            return 0.0;
    }
}

double bridge_output_to_strain(double vout, double vex, double gf,
                               bridge_config_t config)
{
    /* Exact inverse bridge equation: strain from output voltage.
     *
     * Quarter bridge (R1 varies, R2=R3=R4=R0):
     *   Vout/Vexc = 1/(2+GF*eps) - 1/2
     *   Solving: eps = -4*(Vout/Vexc) / [GF * (2*Vout/Vexc + 1)]
     *
     * Half bridge: eps = 2*Vout/(GF*Vexc)   [exact, linear]
     * Full bridge: eps = Vout/(GF*Vexc)     [exact, linear]
     */
    if (vex <= 0.0 || gf <= 0.0) return 0.0;
    double ratio = vout / vex;

    switch (config) {
        case BRIDGE_QUARTER:
        case BRIDGE_DOUBLE_QUARTER: {
            double denom = gf * (2.0 * ratio + 1.0);
            if (fabs(denom) < 1.0e-15) return 0.0;
            return (-4.0 * ratio / denom) * 1.0e6;
        }
        case BRIDGE_HALF:
        case BRIDGE_BENDING:
            if (fabs(gf) < 1.0e-15) return 0.0;
            return (2.0 * ratio / gf) * 1.0e6;
        case BRIDGE_FULL:
        case BRIDGE_TORSION:
        case BRIDGE_SHEAR:
            if (fabs(gf) < 1.0e-15) return 0.0;
            return (ratio / gf) * 1.0e6;
        default:
            return 0.0;
    }
}

double bridge_input_impedance(const bridge_state_t *bridge)
{
    /* Zin = (R1+R3) || (R2+R4)
     *     = (R1+R3)*(R2+R4) / (R1+R2+R3+R4)
     * For balanced bridge: Zin = R0.
     */
    double r_left  = bridge->r1 + bridge->r3;
    double r_right = bridge->r2 + bridge->r4;
    double r_total = r_left + r_right;
    if (r_total <= 0.0) return 0.0;
    return (r_left * r_right) / r_total;
}

double bridge_output_impedance(const bridge_state_t *bridge)
{
    /* Zout = R1||R3 + R2||R4
     *      = R1*R3/(R1+R3) + R2*R4/(R2+R4)
     * For balanced bridge: Zout = R0/2 + R0/2 = R0.
     */
    double r13_par = 0.0, r24_par = 0.0;
    if (bridge->r1 + bridge->r3 > 0.0)
        r13_par = (bridge->r1 * bridge->r3) / (bridge->r1 + bridge->r3);
    if (bridge->r2 + bridge->r4 > 0.0)
        r24_par = (bridge->r2 * bridge->r4) / (bridge->r2 + bridge->r4);
    return r13_par + r24_par;
}

void bridge_power_dissipation(const bridge_state_t *bridge, double power[4])
{
    /* Power in each bridge arm using current-based formulation.
     * I_left = Vexc/(R1+R3), P_R1 = I_left^2 * R1, P_R3 = I_left^2 * R3
     * I_right = Vexc/(R2+R4), P_R2 = I_right^2 * R2, P_R4 = I_right^2 * R4
     */
    double r_left  = bridge->r1 + bridge->r3;
    double r_right = bridge->r2 + bridge->r4;
    if (r_left > 0.0) {
        double i_left = bridge->v_excitation / r_left;
        double i2_left = i_left * i_left;
        power[0] = i2_left * bridge->r1;
        power[2] = i2_left * bridge->r3;
    } else { power[0] = power[2] = 0.0; }
    if (r_right > 0.0) {
        double i_right = bridge->v_excitation / r_right;
        double i2_right = i_right * i_right;
        power[1] = i2_right * bridge->r2;
        power[3] = i2_right * bridge->r4;
    } else { power[1] = power[3] = 0.0; }
}

double bridge_sensitivity_mv_per_v(bridge_config_t config, double gf,
                                   double epsilon_fs_ue)
{
    /* Sensitivity in mV/V at full scale.
     * Quarter: S = GF*eps_FS/4 * 1000
     * Half:    S = GF*eps_FS/2 * 1000
     * Full:    S = GF*eps_FS * 1000
     */
    double eps_abs = epsilon_fs_ue * 1.0e-6;
    switch (config) {
        case BRIDGE_QUARTER:
        case BRIDGE_DOUBLE_QUARTER:
            return (gf * eps_abs / 4.0) * 1000.0;
        case BRIDGE_HALF:
        case BRIDGE_BENDING:
            return (gf * eps_abs / 2.0) * 1000.0;
        case BRIDGE_FULL:
        case BRIDGE_TORSION:
        case BRIDGE_SHEAR:
            return gf * eps_abs * 1000.0;
        default: return 0.0;
    }
}

double bridge_common_mode_voltage(const bridge_state_t *bridge)
{
    /* Vcm = (V+ + V-) / 2 = Vexc/2 * [R3/(R1+R3) + R4/(R2+R4)]
     * For balanced bridge: Vcm = Vexc/2.
     */
    double denom_left  = bridge->r1 + bridge->r3;
    double denom_right = bridge->r2 + bridge->r4;
    if (denom_left <= 0.0 || denom_right <= 0.0)
        return bridge->v_excitation / 2.0;
    double v_plus  = bridge->v_excitation * bridge->r3 / denom_left;
    double v_minus = bridge->v_excitation * bridge->r4 / denom_right;
    return (v_plus + v_minus) / 2.0;
}

double bridge_required_cmrr(double vcm_variation, double vout_resolution)
{
    /* CMRR[dB] = 20 * log10(Vcm_variation / Vout_resolution)
     * The amplifier's finite CMRR converts Vcm variation to
     * differential error: Vout_err = Vcm_var / 10^(CMRR/20)
     */
    if (vout_resolution <= 0.0) return 200.0;
    double ratio = vcm_variation / vout_resolution;
    return 20.0 * log10(ratio);
}

double bridge_min_detectable_strain(double noise_voltage_rms, double vex,
                                    double gf, bridge_config_t config)
{
    /* Noise-equivalent strain: eps_min = (Vnoise/Vexc) * (N/GF)
     * N = 4 (quarter), 2 (half), 1 (full)
     */
    if (vex <= 0.0 || gf <= 0.0) return 1.0e6;
    double n_factor;
    switch (config) {
        case BRIDGE_QUARTER:
        case BRIDGE_DOUBLE_QUARTER: n_factor = 4.0; break;
        case BRIDGE_HALF:
        case BRIDGE_BENDING:        n_factor = 2.0; break;
        case BRIDGE_FULL:
        case BRIDGE_TORSION:
        case BRIDGE_SHEAR:          n_factor = 1.0; break;
        default:                     n_factor = 4.0; break;
    }
    return (noise_voltage_rms / vex) * (n_factor / gf) * 1.0e6;
}

/* ========================================================================
 * L4: Fundamental Laws - Hooke, Piezoresistance, Temperature
 * ======================================================================== */

double hookes_law_stress(double strain_ue, double E_gpa)
{
    /* sigma [MPa] = E [GPa] * eps [ue] * 0.001
     * Because 1 GPa * 1 ue = 1e9 Pa * 1e-6 = 1000 Pa = 0.001 MPa
     */
    return strain_ue * E_gpa * 0.001;
}

void hookes_law_plane_stress(const strain_state_t *strain,
                             double E_gpa, double nu,
                             stress_state_t *stress_out)
{
    /* Generalized Hooke's Law for plane stress.
     * Q11 = E/(1-nu^2), Q12 = nu*Q11, Q66 = E/[2(1+nu)]
     * sigma_x = Q11*eps_x + Q12*eps_y
     * sigma_y = Q12*eps_x + Q11*eps_y
     * tau_xy  = Q66*gamma_xy
     */
    double ex  = strain->epsilon_x * 1.0e-6;
    double ey  = strain->epsilon_y * 1.0e-6;
    double gxy = strain->gamma_xy * 1.0e-6;
    double nu2 = nu * nu;
    double denom = 1.0 - nu2;
    if (denom <= 0.0) denom = 1.0;
    double q11 = E_gpa / denom;
    double q12 = nu * q11;
    double q66 = E_gpa / (2.0 * (1.0 + nu));
    double sx  = q11 * ex + q12 * ey;
    double sy  = q12 * ex + q11 * ey;
    double txy = q66 * gxy;
    stress_out->sigma_x = sx * 1000.0;
    stress_out->sigma_y = sy * 1000.0;
    stress_out->tau_xy  = txy * 1000.0;
    double avg  = (stress_out->sigma_x + stress_out->sigma_y) / 2.0;
    double diff = (stress_out->sigma_x - stress_out->sigma_y) / 2.0;
    double r    = sqrt(diff * diff + stress_out->tau_xy * stress_out->tau_xy);
    stress_out->sigma_1 = avg + r;
    stress_out->sigma_2 = avg - r;
    double s1 = stress_out->sigma_1;
    double s2 = stress_out->sigma_2;
    stress_out->sigma_von_mises = sqrt(s1*s1 - s1*s2 + s2*s2);
}

double temperature_apparent_strain(double delta_T_c,
                                   double cte_specimen_ppm,
                                   double cte_gauge_ppm,
                                   double gf,
                                   double tc_gf_ppm)
{
    /* eps_app = (alpha_s - alpha_g) * dT + (beta_g / GF) * dT
     * First term: differential thermal expansion
     * Second term: gauge factor temperature dependence
     */
    double term1 = (cte_specimen_ppm - cte_gauge_ppm) * delta_T_c;
    double term2 = (tc_gf_ppm / gf) * delta_T_c;
    return term1 + term2;
}

double gauge_resistance_from_strain(double r0, double gf, double strain)
{
    /* R = R0 * (1 + GF * epsilon), strain in microstrain */
    double eps_abs = strain * 1.0e-6;
    return r0 * (1.0 + gf * eps_abs);
}

double strain_from_gauge_resistance(double r_current, double r0, double gf)
{
    /* eps = (R - R0) / (R0 * GF), output in microstrain */
    if (r0 <= 0.0 || gf <= 0.0) return 0.0;
    double delta_r = r_current - r0;
    return (delta_r / (r0 * gf)) * 1.0e6;
}

double gauge_factor_from_material(double nu, double piezo_coeff, double E_gpa)
{
    /* GF = 1 + 2*nu + pi_11 * E
     * Contributions: (1) length change, (2) area change,
     * (3) resistivity change (piezoresistive effect)
     */
    return 1.0 + 2.0 * nu + piezo_coeff * E_gpa;
}

double thermal_stress_mpa(double E_gpa, double cte_specimen_ppm,
                          double cte_gauge_ppm, double delta_T_c)
{
    /* sigma_T = E * (alpha_s - alpha_g) * dT
     * Real mechanical stress from differential thermal expansion.
     */
    double strain_thermal = (cte_specimen_ppm - cte_gauge_ppm) * 1.0e-6 * delta_T_c;
    return E_gpa * strain_thermal * 1000.0;
}

double von_mises_stress(double sigma_1, double sigma_2)
{
    /* sigma_vm = sqrt(sigma_1^2 - sigma_1*sigma_2 + sigma_2^2)
     * Plane stress von Mises (sigma_3 = 0).
     * R. von Mises, 1913.
     */
    return sqrt(sigma_1 * sigma_1 - sigma_1 * sigma_2 + sigma_2 * sigma_2);
}

double von_mises_strain(const strain_state_t *s)
{
    /* eps_vm = 1/(1+nu) * sqrt[(eps_x-eps_y)^2 + eps_x*eps_y + 0.75*gamma_xy^2]
     * nu = 0.3 default for structural metals.
     */
    double nu = 0.3;
    double dx = s->epsilon_x - s->epsilon_y;
    double inner = dx*dx + s->epsilon_x*s->epsilon_y + 0.75*s->gamma_xy*s->gamma_xy;
    if (inner < 0.0) inner = 0.0;
    return sqrt(inner) / (1.0 + nu);
}

double transverse_sensitivity_correction(double eps_measured, double kt,
                                         double nu_cal, double nu_test)
{
    /* eps_true = eps_meas * (1 - nu_cal*Kt) / (1 - Kt*nu_test)
     * Micro-Measurements TN-509 correction formula.
     */
    double denom = 1.0 - kt * nu_test;
    if (fabs(denom) < 1.0e-10) return eps_measured;
    double num = 1.0 - nu_cal * kt;
    return eps_measured * num / denom;
}

/* ========================================================================
 * Initialization and Rosette Resolution
 * ======================================================================== */

void strain_gauge_init(strain_gauge_t *gauge, double nominal_r,
                       double gf, const char *material)
{
    /* Initialize strain gauge with standard catalog defaults. */
    memset(gauge, 0, sizeof(*gauge));
    gauge->nominal_resistance = nominal_r;
    gauge->gauge_factor       = gf;
    gauge->tolerance_percent  = 0.3;
    gauge->material           = material;
    if (nominal_r >= 300.0) {
        gauge->active_length_mm  = 6.0;
        gauge->width_mm          = 3.0;
        gauge->backing_length_mm = 12.0;
    } else {
        gauge->active_length_mm  = 3.0;
        gauge->width_mm          = 2.0;
        gauge->backing_length_mm = 8.0;
    }
    gauge->max_strain_ue         = 50000.0;
    gauge->fatigue_life_cycles   = 1.0e7;
    gauge->temp_coeff_gf_ppm     = 50.0;
    gauge->temp_coeff_resist_ppm = 10.0;
    gauge->thermal_output_ue_per_c = 1.8;
    gauge->backing               = "polyimide";
    gauge->pattern               = "linear";
}

void bridge_state_init(bridge_state_t *bridge, double r_nominal,
                       double v_exc, bridge_config_t config)
{
    /* Initialize bridge to balanced null state. */
    memset(bridge, 0, sizeof(*bridge));
    bridge->r1 = bridge->r2 = bridge->r3 = bridge->r4 = r_nominal;
    bridge->v_excitation = v_exc;
    bridge->config       = config;
    bridge->input_impedance  = bridge_input_impedance(bridge);
    bridge->output_impedance = bridge_output_impedance(bridge);
}

void bridge_sensor_init(bridge_sensor_t *sensor)
{
    /* Initialize complete bridge sensor with sensible defaults:
     * 350 Ohm full bridge, constantan foil, 4-wire Kelvin,
     * 5V ratiometric, steel specimen at 25 degC.
     */
    memset(sensor, 0, sizeof(*sensor));
    int i;
    for (i = 0; i < 4; i++)
        strain_gauge_init(&sensor->gauge[i], 350.0, 2.05, "constantan");
    bridge_state_init(&sensor->bridge, 350.0, 5.0, BRIDGE_FULL);
    sensor->config     = BRIDGE_FULL;
    sensor->completion = COMPLETION_EXTERNAL_FULL;
    sensor->exc_mode   = EXCITATION_RATIOMETRIC;
    sensor->leadwire   = LEADWIRE_4WIRE;
    sensor->sens.mv_per_volt        = 2.05;
    sensor->sens.mv_per_volt_per_ue = 0.00205;
    sensor->sens.uv_per_volt_per_ue = 2.05;
    sensor->sens.ue_full_scale      = 1000.0;
    sensor->sens.nonlinearity_percent  = 0.02;
    sensor->sens.hysteresis_percent    = 0.01;
    sensor->sens.repeatability_percent = 0.005;
    sensor->sens.creep_percent_30min   = 0.02;
    sensor->balance.zero_offset_uv_per_v      = 10.0;
    sensor->balance.zero_drift_uv_per_v_per_c = 0.5;
    sensor->balance.residual_offset_percent   = 0.01;
    sensor->temperature_c           = 25.0;
    sensor->reference_temperature_c = 25.0;
    sensor->specimen_mat.name                 = "steel-1018";
    sensor->specimen_mat.youngs_modulus_gpa   = 200.0;
    sensor->specimen_mat.poisson_ratio        = 0.29;
    sensor->specimen_mat.thermal_expansion_ppm = 11.7;
    sensor->specimen_mat.density_g_cm3        = 7.85;
    sensor->specimen_mat.thermal_conductivity = 51.9;
    sensor->cable_resistance_ohm  = 0.5;
    sensor->shunt_cal_resistor_ohm = 350000.0;
}

void rosette_resolve_strain(rosette_data_t *rosette)
{
    /* Resolve 3-gauge rosette to complete 2D strain state.
     *
     * Strain transformation: eps(theta) = eps_x*cos^2(theta)
     *   + eps_y*sin^2(theta) + gamma_xy*sin(theta)*cos(theta)
     *
     * Rectangular (0/45/90): most common
     * Delta (0/60/120): equiangular
     * T-delta (0/90): no shear determination
     */
    double eps_x, eps_y, gamma_xy;
    switch (rosette->type) {
        case ROSETTE_RECTANGULAR:
            eps_x = rosette->ea;
            eps_y = rosette->ec;
            gamma_xy = 2.0 * rosette->eb - rosette->ea - rosette->ec;
            break;
        case ROSETTE_DELTA:
            eps_x = rosette->ea;
            eps_y = (2.0*rosette->eb + 2.0*rosette->ec - rosette->ea)/3.0;
            gamma_xy = 2.0*(rosette->eb - rosette->ec)/sqrt(3.0);
            break;
        case ROSETTE_T_DELTA:
            eps_x = rosette->ea; eps_y = rosette->eb; gamma_xy = 0.0;
            break;
        case ROSETTE_STACKED:
            eps_x = rosette->ea; eps_y = rosette->ec;
            gamma_xy = 2.0*rosette->eb - rosette->ea - rosette->ec;
            break;
        default:
            eps_x = eps_y = gamma_xy = 0.0; break;
    }
    rosette->resolved.epsilon_x = eps_x;
    rosette->resolved.epsilon_y = eps_y;
    rosette->resolved.gamma_xy  = gamma_xy;
    double avg  = (eps_x + eps_y)/2.0;
    double diff = (eps_x - eps_y)/2.0;
    double r    = sqrt(diff*diff + (gamma_xy/2.0)*(gamma_xy/2.0));
    rosette->resolved.epsilon_1 = avg + r;
    rosette->resolved.epsilon_2 = avg - r;
    if (fabs(eps_x-eps_y) > 1.0e-12)
        rosette->resolved.angle_deg = 0.5*atan2(gamma_xy, eps_x-eps_y)*180.0/M_PI;
    else if (fabs(gamma_xy) > 1.0e-12)
        rosette->resolved.angle_deg = 45.0;
    else
        rosette->resolved.angle_deg = 0.0;
    rosette->resolved.epsilon_von_mises = von_mises_strain(&rosette->resolved);
}

int bridge_fit_nl_correction(const double *vout_array,
                             const double *strain_array,
                             int n_points,
                             nl_correction_poly_t *poly)
{
    /* Least-squares polynomial fit for bridge nonlinearity correction.
     * Uses normal equations with Gaussian elimination.
     */
    if (n_points < 2 || poly->order < 1 || poly->order > 5) return -1;
    int k = poly->order, m = k + 1;
    if (n_points < m) return -1;
    double ata[6][7] = {{0}};
    int i, j, p;
    for (i = 0; i < n_points; i++) {
        double x = strain_array[i], y = vout_array[i];
        double xpow[12]; xpow[0] = 1.0;
        for (j = 1; j <= 2*k; j++) xpow[j] = xpow[j-1]*x;
        for (j = 0; j < m; j++) {
            for (p = 0; p < m; p++) ata[j][p] += xpow[j+p];
            ata[j][m] += y * xpow[j];
        }
    }
    for (i = 0; i < m; i++) {
        int max_row = i;
        double max_val = fabs(ata[i][i]);
        for (j = i+1; j < m; j++)
            if (fabs(ata[j][i]) > max_val) { max_val = fabs(ata[j][i]); max_row = j; }
        if (max_val < 1.0e-15) return -1;
        if (max_row != i)
            for (j = i; j <= m; j++) {
                double tmp = ata[i][j]; ata[i][j] = ata[max_row][j]; ata[max_row][j] = tmp; }
        for (j = i+1; j < m; j++) {
            double factor = ata[j][i]/ata[i][i];
            for (p = i; p <= m; p++) ata[j][p] -= factor*ata[i][p];
        }
    }
    for (i = m-1; i >= 0; i--) {
        double sum = ata[i][m];
        for (j = i+1; j < m; j++) sum -= ata[i][j]*poly->coeffs[j];
        poly->coeffs[i] = sum/ata[i][i];
    }
    double ss_res=0, ss_tot=0, y_mean=0;
    for (i = 0; i < n_points; i++) y_mean += vout_array[i];
    y_mean /= n_points;
    for (i = 0; i < n_points; i++) {
        double y_pred=0, xp=1;
        for (j = 0; j < m; j++) { y_pred += poly->coeffs[j]*xp; xp *= strain_array[i]; }
        double resid = vout_array[i]-y_pred;
        ss_res += resid*resid;
        ss_tot += (vout_array[i]-y_mean)*(vout_array[i]-y_mean);
    }
    poly->fit_r_squared = (ss_tot>0)? 1.0-ss_res/ss_tot : 1.0;
    poly->max_residual_ue = 0;
    for (i = 0; i < n_points; i++) {
        double y_pred=0, xp=1;
        for (j = 0; j < m; j++) { y_pred += poly->coeffs[j]*xp; xp *= strain_array[i]; }
        double resid = fabs(vout_array[i]-y_pred);
        if (resid > poly->max_residual_ue) poly->max_residual_ue = resid;
    }
    return 0;
}

double bridge_apply_nl_correction(double eps_measured,
                                  const nl_correction_poly_t *poly)
{
    /* Horner's method: eps_corr = c0 + eps*(c1 + eps*(c2 + ...)) */
    double result = 0.0;
    int i;
    for (i = poly->order; i >= 0; i--)
        result = result * eps_measured + poly->coeffs[i];
    return result;
}
