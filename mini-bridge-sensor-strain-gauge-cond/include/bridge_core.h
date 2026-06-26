/**
 * bridge_core.h — Wheatstone Bridge Sensor Core Definitions
 *
 * Covers the Wheatstone bridge (Christie 1833, Wheatstone 1843)
 * and its application to resistive strain gauge sensors.
 *
 * Knowledge Coverage:
 *   L1: Wheatstone bridge, gauge factor (GF), bridge ratio,
 *       sensitivity, strain, stress, bridge balance
 *   L2: Quarter/half/full bridge topologies, lead wire
 *       compensation, ratiometric measurement, self-heating
 *   L3: Bridge output equation (exact + linearized), inverse
 *       bridge equation, impedance analysis
 *   L4: Hooke's law, piezoresistive effect, Ohm's law for
 *       bridge networks, thermal expansion law
 *
 * Reference:
 *   - Doebelin, "Measurement Systems", 5th ed., McGraw-Hill
 *   - Hoffmann, "Stress Analysis using Strain Gauges", HBM
 *   - Sedra & Smith, "Microelectronic Circuits", 8th ed., Ch.2
 *
 * University Course Mapping:
 *   MIT 6.002, Berkeley EE16A, ETH 227-0116, TU Munich EI0430,
 *   Georgia Tech ECE 3042, Michigan EECS 215, Stanford EE101A,
 *   Illinois ECE 110, THU Sensor Technology
 */

#ifndef BRIDGE_CORE_H
#define BRIDGE_CORE_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * L1: Core Definitions — Wheatstone Bridge and Strain Gauge Structures
 * ======================================================================== */

/**
 * Bridge arm configuration — defines how many active strain-sensing
 * elements participate in the Wheatstone bridge circuit.
 *
 * Sensitivity scales with active arm count:
 *   Quarter bridge: S = GF * epsilon / 4
 *   Half bridge:    S = GF * epsilon / 2
 *   Full bridge:    S = GF * epsilon
 */
typedef enum {
    BRIDGE_QUARTER        = 1,
    BRIDGE_HALF           = 2,
    BRIDGE_FULL           = 4,
    BRIDGE_DOUBLE_QUARTER = 5,
    BRIDGE_BENDING        = 6,
    BRIDGE_TORSION        = 7,
    BRIDGE_SHEAR          = 8
} bridge_config_t;

typedef enum {
    COMPLETION_INTERNAL_HALF,
    COMPLETION_EXTERNAL_FULL,
    COMPLETION_DUMMY_GAUGE,
    COMPLETION_ACTIVE_DUMMY
} bridge_completion_t;

/**
 * Wheatstone bridge electrical state descriptor.
 *
 * Topology (arms numbered clockwise from top-left):
 *          Vexc+
 *           o
 *          / \
 *         R1  R2
 *        /     \
 *   Vout- o     o Vout+
 *        \     /
 *         R3  R4
 *          \ /
 *           o
 *          GND
 *
 * Exact differential output (Christie 1833, Wheatstone 1843):
 *   Vout = Vexc * [R3/(R1+R3) - R4/(R2+R4)]
 *
 * Balance condition: R1*R4 = R2*R3
 */
typedef struct {
    double r1, r2, r3, r4;
    double v_excitation;
    double i_excitation;
    double v_output;
    double output_impedance;
    double input_impedance;
    bridge_config_t config;
} bridge_state_t;

/**
 * Metal foil strain gauge physical descriptor.
 *
 * Gauge Factor (GF):
 *   GF = (dR/R0) / epsilon
 *      = fractional resistance change per unit strain
 *
 * Physical origin (Bridgman piezoresistance theory):
 *   GF = 1 + 2*nu + pi_11 * E
 * where nu = Poisson ratio, pi_11 = piezoresistive coeff, E = Young's modulus
 *
 * Metallic foil: GF ~ 2.0-2.1 (constantan, karma)
 * Semiconductor:  GF ~ 50-200 (silicon, germanium)
 */
typedef struct {
    double nominal_resistance;
    double gauge_factor;
    double tolerance_percent;
    double active_length_mm;
    double width_mm;
    double backing_length_mm;
    double max_strain_ue;
    double fatigue_life_cycles;
    double temp_coeff_gf_ppm;
    double temp_coeff_resist_ppm;
    double thermal_output_ue_per_c;
    const char *material;
    const char *backing;
    const char *pattern;
} strain_gauge_t;

/**
 * Material properties for gauge grid and structural specimen.
 */
typedef struct {
    const char *name;
    double youngs_modulus_gpa;
    double poisson_ratio;
    double resistivity_uohm_cm;
    double temp_coeff_resist_ppm;
    double thermal_conductivity;
    double thermal_expansion_ppm;
    double density_g_cm3;
    double gauge_factor;
    double piezoresist_coeff;
} gauge_material_t;

/**
 * Bridge sensitivity specification.
 * Rated output in mV/V at full scale.
 */
typedef struct {
    double mv_per_volt;
    double mv_per_volt_per_ue;
    double uv_per_volt_per_ue;
    double ue_full_scale;
    double nonlinearity_percent;
    double hysteresis_percent;
    double repeatability_percent;
    double creep_percent_30min;
} bridge_sensitivity_t;

/* ========================================================================
 * L2: Core Concepts — Excitation, Wiring, Balance, Strain State
 * ======================================================================== */

typedef enum {
    EXCITATION_VOLTAGE,
    EXCITATION_CURRENT,
    EXCITATION_RATIOMETRIC
} excitation_mode_t;

typedef enum {
    LEADWIRE_2WIRE,
    LEADWIRE_3WIRE,
    LEADWIRE_4WIRE,
    LEADWIRE_6WIRE
} leadwire_config_t;

typedef struct {
    double zero_offset_uv_per_v;
    double zero_drift_uv_per_v_per_c;
    double balance_resistor_ohm;
    int    balance_arm;
    double residual_offset_percent;
} bridge_balance_t;

/**
 * Complete 2D strain state at a point (tensor representation).
 *
 * Principal strains (eigenvalues):
 *   eps_1,2 = (eps_x+eps_y)/2
 *             +/- sqrt[((eps_x-eps_y)/2)^2 + (gamma_xy/2)^2]
 *
 * Principal angle: theta_p = 0.5 * atan2(gamma_xy, eps_x - eps_y)
 */
typedef struct {
    double epsilon_x;
    double epsilon_y;
    double gamma_xy;
    double epsilon_1;
    double epsilon_2;
    double angle_deg;
    double epsilon_von_mises;
} strain_state_t;

/**
 * 2D stress state derived from strain via Hooke's law.
 */
typedef struct {
    double sigma_x;
    double sigma_y;
    double tau_xy;
    double sigma_1;
    double sigma_2;
    double sigma_von_mises;
} stress_state_t;

typedef enum {
    ROSETTE_RECTANGULAR,
    ROSETTE_DELTA,
    ROSETTE_T_DELTA,
    ROSETTE_STACKED
} rosette_type_t;

typedef struct {
    rosette_type_t type;
    double ea, eb, ec;
    double angle_b_deg, angle_c_deg;
    strain_state_t resolved;
} rosette_data_t;

typedef enum {
    NL_CORRECTION_NONE,
    NL_CORRECTION_ANALYTIC,
    NL_CORRECTION_POLYNOMIAL,
    NL_CORRECTION_LOOKUP
} nl_correction_method_t;

typedef struct {
    double coeffs[6];
    int    order;
    double fit_r_squared;
    double max_residual_ue;
} nl_correction_poly_t;

typedef struct {
    double sample_rate_hz;
    double averaging_window_ms;
    double settling_time_us;
    int    oversampling_ratio;
    int    samples_per_reading;
} daq_timing_t;

/**
 * Complete bridge sensor system descriptor.
 * Bundles all parameters for a fully characterized bridge sensor.
 */
typedef struct {
    strain_gauge_t gauge[4];
    bridge_state_t bridge;
    bridge_config_t config;
    bridge_completion_t completion;
    excitation_mode_t exc_mode;
    leadwire_config_t leadwire;
    bridge_sensitivity_t sens;
    bridge_balance_t balance;
    nl_correction_method_t nl_method;
    nl_correction_poly_t nl_poly;
    daq_timing_t timing;
    double temperature_c;
    double reference_temperature_c;
    gauge_material_t specimen_mat;
    double cable_resistance_ohm;
    double shunt_cal_resistor_ohm;
} bridge_sensor_t;

/* ========================================================================
 * L3: Mathematical Structures — Bridge Equation API
 * ======================================================================== */

/**
 * Exact Wheatstone bridge output voltage.
 * Vout = Vexc * [R3/(R1+R3) - R4/(R2+R4)]
 * This is the primary measurement equation. O(1).
 */
double bridge_output_voltage(const bridge_state_t *bridge);

/**
 * Linearized bridge output for small strain.
 * Vout = Vexc * GF/4 * (eps1 - eps2 + eps3 - eps4)
 * Valid when |GF*epsilon| << 1. O(1).
 */
double bridge_output_linear(const bridge_state_t *bridge,
                            const double strains[4], double gf);

/**
 * Nonlinearity error as percentage of reading.
 * Quarter bridge: NL(%) = GF*epsilon/2 * 100. O(1).
 */
double bridge_nonlinearity_error(double strain_ue, double gf,
                                 bridge_config_t config);

/**
 * Recover strain from bridge output (exact inverse formula).
 * Quarter: eps = -(4*Vout/Vex) / (GF * (1 + 2*Vout/Vex)). O(1).
 */
double bridge_output_to_strain(double vout, double vex, double gf,
                               bridge_config_t config);

/**
 * Bridge input impedance: Zin = (R1+R3)||(R2+R4). O(1).
 */
double bridge_input_impedance(const bridge_state_t *bridge);

/**
 * Bridge output Thevenin impedance: Zout = R1||R3 + R2||R4. O(1).
 */
double bridge_output_impedance(const bridge_state_t *bridge);

/**
 * Power dissipated in each bridge arm. P_i = V_i^2 / R_i. O(1).
 */
void bridge_power_dissipation(const bridge_state_t *bridge, double power[4]);

/**
 * Bridge sensitivity in mV/V at full scale. O(1).
 */
double bridge_sensitivity_mv_per_v(bridge_config_t config, double gf,
                                   double epsilon_fs_ue);

/**
 * Bridge common-mode voltage: Vcm = Vexc*(R3/(R1+R3) + R4/(R2+R4))/2. O(1).
 */
double bridge_common_mode_voltage(const bridge_state_t *bridge);

/**
 * Required amplifier CMRR for bridge measurement [dB]. O(1).
 */
double bridge_required_cmrr(double vcm_variation, double vout_resolution);

/**
 * Minimum detectable strain from noise analysis. O(1).
 */
double bridge_min_detectable_strain(double noise_voltage_rms, double vex,
                                    double gf, bridge_config_t config);

/* ========================================================================
 * L4: Fundamental Laws — Stress-Strain, Piezoresistance, Temperature
 * ======================================================================== */

/**
 * Hooke's Law (1678): sigma = E * epsilon. O(1).
 */
double hookes_law_stress(double strain_ue, double E_gpa);

/**
 * Generalized Hooke's Law for plane stress (2D isotropic).
 * Stiffness matrix formulation. O(1).
 */
void hookes_law_plane_stress(const strain_state_t *strain,
                             double E_gpa, double nu,
                             stress_state_t *stress_out);

/**
 * Temperature-induced apparent strain (thermal output).
 * eps_app = (alpha_s - alpha_g)*dT + (beta_g/GF)*dT. O(1).
 */
double temperature_apparent_strain(double delta_T_c,
                                   double cte_specimen_ppm,
                                   double cte_gauge_ppm,
                                   double gf,
                                   double tc_gf_ppm);

/**
 * Gauge resistance under strain: R = R0 * (1 + GF*epsilon). O(1).
 */
double gauge_resistance_from_strain(double r0, double gf, double strain);

/**
 * Strain from resistance change: eps = (R-R0)/(R0*GF). O(1).
 */
double strain_from_gauge_resistance(double r_current, double r0, double gf);

/**
 * Theoretical gauge factor: GF = 1 + 2*nu + pi*E. O(1).
 */
double gauge_factor_from_material(double nu, double piezo_coeff, double E_gpa);

/**
 * Thermal stress from differential expansion. O(1).
 */
double thermal_stress_mpa(double E_gpa, double cte_specimen_ppm,
                          double cte_gauge_ppm, double delta_T_c);

/**
 * Von Mises stress: sqrt(sig1^2 - sig1*sig2 + sig2^2). O(1).
 */
double von_mises_stress(double sigma_1, double sigma_2);

/**
 * Von Mises equivalent strain. O(1).
 */
double von_mises_strain(const strain_state_t *s);

/**
 * Transverse sensitivity correction for strain gauges.
 * Corrects for gauge response to perpendicular strain. O(1).
 */
double transverse_sensitivity_correction(double eps_measured, double kt,
                                         double nu_cal, double nu_test);

/* ========================================================================
 * Initialization and Utility Functions
 * ======================================================================== */

void strain_gauge_init(strain_gauge_t *gauge, double nominal_r,
                       double gf, const char *material);

void bridge_state_init(bridge_state_t *bridge, double r_nominal,
                       double v_exc, bridge_config_t config);

void bridge_sensor_init(bridge_sensor_t *sensor);

/**
 * Resolve strain gauge rosette to complete 2D strain state.
 *
 * Rectangular (0/45/90):
 *   eps_x = e_a, eps_y = e_c, gamma_xy = 2*e_b - e_a - e_c
 *
 * Delta (0/60/120):
 *   eps_x = e_a, eps_y = (2*e_b + 2*e_c - e_a)/3
 *   gamma_xy = 2*(e_b - e_c)/sqrt(3)
 *
 * Principal strains and angle computed from resolved components.
 * Reference: Dally & Riley, "Experimental Stress Analysis", Ch.8
 */
void rosette_resolve_strain(rosette_data_t *rosette);

int bridge_fit_nl_correction(const double *vout_array,
                             const double *strain_array,
                             int n_points,
                             nl_correction_poly_t *poly);

double bridge_apply_nl_correction(double eps_measured,
                                  const nl_correction_poly_t *poly);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_CORE_H */
