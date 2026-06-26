/**
 * bridge_applications.c — Bridge Sensor Application Implementations
 *
 * Knowledge Coverage:
 *   L6: Load cell, pressure sensor, torque sensor design
 *   L7: Industrial weighing (OIML/NTEP), automotive MAP sensing,
 *       aerospace structural health monitoring (SHM)
 *   L8: Wireless IoT sensor nodes, edge processing
 *
 * Reference:
 *   - OIML R60 — Metrological Regulation for Load Cells
 *   - SAE J1763 — Automotive Pressure Sensor Standard
 *   - MIL-STD-810 — Environmental Test Methods for Aerospace
 *   - Miner, "Cumulative Damage in Fatigue", J Appl Mech 1945
 */

#include "bridge_applications.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <math.h>
#include <string.h>

double application_loadcell_output(const loadcell_spec_t *loadcell,
                                   double applied_force_kg, double vex)
{
    /* Load cell output for applied force.
     *
     * Vout = Vexc * S * (F / F_rated)
     *
     * where S = rated output [mV/V] (converted to V/V).
     *
     * This is the linear load cell model. High-quality load cells
     * maintain linearity within 0.02% FS.
     *
     * Example: 100 kg cell, S=2 mV/V, Vexc=10V, F=75 kg
     *   Vout = 10 * 0.002 * 75/100 = 0.015 V = 15 mV
     */

    if (loadcell == NULL) return 0.0;

    double sensitivity_v_per_v = loadcell->rated_output_mv_per_v / 1000.0;

    if (loadcell->rated_capacity_kg <= 0.0) return 0.0;

    double load_fraction = applied_force_kg / loadcell->rated_capacity_kg;
    return vex * sensitivity_v_per_v * load_fraction;
}

double application_loadcell_force(const loadcell_spec_t *loadcell,
                                  double vout, double vex)
{
    /* Force from load cell output (inverse of above).
     *
     * F = F_rated * Vout / (Vexc * S)
     */

    if (loadcell == NULL || vex <= 0.0) return 0.0;

    double sensitivity_v_per_v = loadcell->rated_output_mv_per_v / 1000.0;

    if (sensitivity_v_per_v <= 0.0) return 0.0;

    return loadcell->rated_capacity_kg * vout / (vex * sensitivity_v_per_v);
}

double application_multicell_total(const double individual_forces[4],
                                   const double corner_factors[4])
{
    /* Total force from multi-cell platform with corner correction.
     *
     * F_total = sum(K_i * F_i) for i=1..4
     *
     * Corner factors K_i are determined during calibration by
     * placing the same weight at each corner and adjusting
     * factors so the indicated total is the same regardless
     * of weight position.
     *
     * This is a KEY requirement for legal-for-trade scales:
     * the weight indication must be independent of load position
     * within specified eccentricity limits.
     */

    double total = 0.0;
    int i;
    for (i = 0; i < 4; i++) {
        total += individual_forces[i] * corner_factors[i];
    }
    return total;
}

/* ========================================================================
 * L6: Pressure Sensor
 * ======================================================================== */

double application_pressure_read(const pressure_sensor_spec_t *sensor,
                                 double vout, double vex,
                                 double temp_c, double ref_temp_c)
{
    /* Compute pressure from MEMS bridge sensor output.
     *
     * P_raw = (Vout / Vexc) * (1 / S) * P_FS
     *
     * where S = sensitivity [mV/V/kPa].
     *
     * Temperature compensation:
     *   P = P_raw + TC_zero*P_FS*(T-Tref) + TC_span*(T-Tref)*P_raw
     *
     * MEMS pressure sensors exhibit significant temperature
     * dependence due to:
     * 1. Piezoresistive coefficient variation with T
     * 2. Diaphragm stiffness change with T (E varies with T)
     * 3. Thermal expansion of the diaphragm and package
     */

    if (sensor == NULL || vex <= 0.0) return 0.0;

    double s = sensor->sensitivity_mv_per_v_per_kpa / 1000.0;  /* V/V/kPa */
    if (fabs(s) < 1.0e-15) return 0.0;

    double ratio = vout / vex;
    double p_raw = ratio / s;  /* Already in kPa if s is in V/V/kPa */

    /* Temperature compensation */
    double dt = temp_c - ref_temp_c;
    double tc_zero = sensor->tempco_zero_percent_fs_per_c / 100.0;
    double tc_span = sensor->tempco_span_percent_fs_per_c / 100.0;

    double p_comp = p_raw
                    + tc_zero * sensor->pressure_range_kpa * dt
                    + tc_span * p_raw * dt;

    return p_comp;
}

double application_pressure_diaphragm_thickness(double edge_length_um,
                                                 double max_pressure_kpa,
                                                 double target_strain_ue,
                                                 double E_gpa, double nu)
{
    /* Design MEMS pressure sensor diaphragm thickness.
     *
     * For a square diaphragm (edge a) under uniform pressure P,
     * clamped on all four edges:
     *
     * Maximum stress at center of edge:
     *   sigma_max = beta * P * (a/t)^2
     *   where beta ≈ 0.308 for square diaphragm
     *
     * Maximum deflection at center:
     *   w_max = alpha * P * a^4 / (E * t^3)
     *   where alpha ≈ 0.0138 for square diaphragm
     *
     * Maximum strain at center (biaxial):
     *   eps_max = gamma * P * (a/t)^2 * (1-nu^2) / E
     *   where gamma ≈ 0.138 for square diaphragm
     *
     * Solving for required thickness:
     *   t = a * sqrt[gamma * P * (1-nu^2) / (E * eps_max)]
     *
     * Units: edge_length in um, pressure in kPa, E in GPa
     * Output: thickness in um
     */

    if (edge_length_um <= 0.0 || E_gpa <= 0.0) return 0.0;

    double p_pa = max_pressure_kpa * 1000.0;   /* kPa → Pa */
    double E_pa = E_gpa * 1.0e9;               /* GPa → Pa */
    double eps_abs = target_strain_ue * 1.0e-6;
    double nu2 = nu * nu;

    double gamma = 0.138;  /* For square diaphragm, clamped edges */

    double a_m = edge_length_um * 1.0e-6;  /* um → m */

    double t_m = a_m * sqrt(gamma * p_pa * (1.0 - nu2) / (E_pa * eps_abs));

    return t_m * 1.0e6;  /* m → um */
}

/* ========================================================================
 * L6: Torque Sensor
 * ======================================================================== */

double application_torque_shear_stress(double torque_nm, double diameter_mm)
{
    /* Shear stress on surface of circular shaft under torsion.
     *
     * tau = T * r / J
     *
     * For solid circular shaft:
     *   r = d/2
     *   J = pi * d^4 / 32  (polar moment of inertia)
     *
     * tau_max = T * (d/2) / (pi * d^4 / 32)
     *         = T * 16 / (pi * d^3)
     *
     * Units: T in N*m, d in mm → tau in Pa
     *   tau = 16 * T / (pi * d^3)  [T in N*m, d in m]
     *
     * With d in mm: d_m = d_mm * 0.001
     *   tau = 16 * T / (pi * (d_mm*0.001)^3)
     *       = 16 * T * 10^9 / (pi * d_mm^3)  [Pa]
     *       = 16000 * T / (pi * d_mm^3)       [MPa]
     */

    if (diameter_mm <= 0.0) return 0.0;

    double d_m = diameter_mm * 0.001;
    double tau_pa = 16.0 * torque_nm / (M_PI * d_m * d_m * d_m);

    return tau_pa * 1.0e-6;  /* Pa → MPa */
}

double application_torque_strain(double torque_nm, double diameter_mm,
                                 double E_gpa, double nu)
{
    /* Strain at +/-45 degrees from shaft axis under pure torsion.
     *
     * Under pure torsion, the principal stresses are:
     *   sigma_1 = +tau (at +45 degrees)
     *   sigma_2 = -tau (at -45 degrees)
     *
     * Strain at +45 degrees (using Hooke's law):
     *   eps_45 = (sigma_1 - nu*sigma_2) / E
     *          = (tau + nu*tau) / E
     *          = tau * (1 + nu) / E
     *
     * This is the strain that a 45-degree gauge measures.
     *
     * Using the shear stress formula:
     *   tau = 16 * T / (pi * d^3)
     *
     * eps_45 = 16 * T * (1+nu) / (pi * d^3 * E)
     */

    if (diameter_mm <= 0.0 || E_gpa <= 0.0) return 0.0;

    double d_m = diameter_mm * 0.001;
    double E_pa = E_gpa * 1.0e9;
    double tau_pa = 16.0 * torque_nm / (M_PI * d_m * d_m * d_m);
    double eps_abs = tau_pa * (1.0 + nu) / E_pa;

    return eps_abs * 1.0e6;  /* → microstrain */
}

double application_torque_shaft_diameter(double max_torque_nm,
                                         double target_strain_ue,
                                         double E_gpa, double nu)
{
    /* Design shaft diameter for desired full-scale strain.
     *
     * From eps_45 = 16*T*(1+nu) / (pi * d^3 * E):
     *
     *   d^3 = 16*T*(1+nu) / (pi * eps_target * E)
     *   d = cbrt[16*T*(1+nu) / (pi * eps_target * E)]
     *
     * Output in mm.
     */

    if (target_strain_ue <= 0.0 || E_gpa <= 0.0) return 0.0;

    double eps_abs = target_strain_ue * 1.0e-6;
    double E_pa = E_gpa * 1.0e9;

    double d3_m3 = 16.0 * max_torque_nm * (1.0 + nu)
                   / (M_PI * eps_abs * E_pa);

    double d_m = cbrt(d3_m3);  /* cbrt = cube root */
    return d_m * 1000.0;  /* m → mm */
}

/* ========================================================================
 * L7: Industrial Weighing System
 * ======================================================================== */

double application_weighing_total(const weighing_system_t *system,
                                  const double *readings, int n_readings)
{
    /* Compute total weight from multi-cell platform.
     *
     * Total = sum(reading_i * corner_factor_i) for all cells.
     *
     * The corner factors are determined during calibration by
     * placing a known weight sequentially at each corner position
     * and adjusting the factors so the indicated total weight
     * is correct and independent of weight position.
     *
     * This is a standard requirement for legal-for-trade scales:
     * eccentricity error must be within specified tolerance
     * per OIML R76 / NIST Handbook 44.
     */

    if (system == NULL || readings == NULL) return 0.0;

    double total = 0.0;
    int n_cells = system->n_cells;
    if (n_readings < n_cells) n_cells = n_readings;

    int i;
    for (i = 0; i < n_cells; i++) {
        total += readings[i] * system->corner_factors[i];
    }

    return total;
}

/* ========================================================================
 * L7: Automotive MAP Sensor
 * ======================================================================== */

double application_automotive_map(const automotive_pressure_t *sensor,
                                  int adc_reading, double temp_c,
                                  int *fault_code)
{
    /* Process automotive manifold air pressure (MAP) sensor reading
     * with diagnostic fault detection.
     *
     * Automotive pressure sensors typically use a three-wire
     * interface: Vcc (+5V), GND, Vout (ratiometric, 0.5-4.5V range).
     *
     * The ADC reading is converted to voltage:
     *   Vout = adc_reading * Vref / (2^N - 1)
     *
     * Diagnostic checks:
     * 1. Open circuit: Vout near Vcc (pull-up) or near 0 (pull-down)
     * 2. Short to ground: Vout < 2% Vcc
     * 3. Short to Vbat: Vout > 98% Vcc
     * 4. Overpressure: computed pressure > max safe pressure
     *
     * These diagnostics are required for OBD-II compliance.
     *
     * Returns: pressure in kPa, sets fault_code.
     * Returns NaN if sensor fault detected.
     */

    if (sensor == NULL) {
        if (fault_code) *fault_code = 1;
        return NAN;
    }

    double vout = (double)adc_reading * sensor->adc_reference_v
                  / ((1 << sensor->adc_bits) - 1);

    /* Diagnostic checks */
    double vcc = sensor->supply_voltage_v;

    if (vout < 0.02 * vcc) {
        /* Short to ground or open circuit (pulldown) */
        if (fault_code) *fault_code = 2;
        return NAN;
    }

    if (vout > 0.98 * vcc) {
        /* Short to Vbat or open circuit (pullup) */
        if (fault_code) *fault_code = 3;
        return NAN;
    }

    /* Clamp range to valid output window (10-90% Vcc) */
    if (vout < sensor->clamp_voltage_min_v || vout > sensor->clamp_voltage_max_v) {
        if (fault_code) *fault_code = 1;
        return NAN;
    }

    /* Compute pressure with temperature compensation */
    double pressure = application_pressure_read(&sensor->sensor, vout,
                                                 sensor->supply_voltage_v,
                                                 temp_c, 25.0);

    /* Overpressure check */
    if (pressure > sensor->sensor.overpressure_kpa) {
        if (fault_code) *fault_code = 4;
        return NAN;
    }

    if (fault_code) *fault_code = 0;  /* OK */
    return pressure;
}

/* ========================================================================
 * L7: Aerospace Structural Health Monitoring
 * ======================================================================== */

void application_aerospace_fatigue(aerospace_shm_t *shm,
                                   double strain_amp_ue, int n_cycles,
                                   double fatigue_exp_m,
                                   double ref_strain_ue, double ref_cycles)
{
    /* Accumulate fatigue damage using Miner's rule.
     *
     * Miner's linear damage rule (Miner, 1945):
     *   D = sum(n_i / N_i)
     *
     * where:
     *   n_i = number of cycles experienced at strain level i
     *   N_i = number of cycles to failure at strain level i
     *
     * Failure is predicted when D >= 1.0 (though in practice,
     * failure often occurs at D values ranging from 0.5 to 2.0).
     *
     * S-N curve (Basquin's law / Wohler curve):
     *   N_f = N_ref * (eps_ref / eps)^m
     *
     * where m = fatigue strength exponent (Basquin exponent):
     *   m ≈ 3-5 for high-cycle fatigue of steels
     *   m ≈ 5-10 for aluminum alloys
     *   m ≈ 8-15 for welded structures
     *
     * Reference strain and cycles define a known point on the
     * S-N curve, typically the fatigue limit (endurance limit)
     * or a known fatigue life point.
     */

    if (shm == NULL || n_cycles <= 0) return;

    if (strain_amp_ue <= 0.0) return;  /* No damage from zero/compressive mean */

    /* Cycles to failure at this strain amplitude */
    double ratio = ref_strain_ue / strain_amp_ue;
    double n_f = ref_cycles * pow(ratio, fatigue_exp_m);

    /* Damage increment */
    double damage_increment = (double)n_cycles / n_f;

    shm->cumulative_damage += damage_increment;
    shm->n_cycles_recorded += n_cycles;

    /* Update peak strain */
    if (strain_amp_ue > shm->max_recorded_strain_ue) {
        shm->max_recorded_strain_ue = strain_amp_ue;
    }
}

/* ========================================================================
 * L8: Wireless IoT Sensor Node
 * ======================================================================== */

double application_wireless_battery_life(const wireless_sensor_node_t *node)
{
    /* Estimate battery life for wireless bridge sensor node.
     *
     * Duty-cycled operation:
     *
     *   T_total = T_sleep + T_meas + T_tx
     *
     * Average current:
     *   I_avg = (I_sleep*T_sleep + I_active*T_meas + I_tx*T_tx) / T_total
     *
     * Battery life:
     *   Life_hours = Capacity_mAh / I_avg_mA
     *
     * This assumes the battery capacity is fully usable.
     * In practice:
     * - Li-SOCl2 primary cells: 90-95% usable capacity
     * - Li-Ion rechargeable: 80% after 500 cycles
     * - Alkaline: 60-70% at low temperatures
     * - Self-discharge limits life to 5-10 years for primary cells
     */

    if (node == NULL) return 0.0;

    double t_total_s = node->transmit_interval_s;
    double t_meas_s = node->measurement_time_ms / 1000.0;
    double t_tx_s   = node->tx_time_ms / 1000.0;
    double t_sleep_s = t_total_s - t_meas_s - t_tx_s;

    if (t_sleep_s < 0.0) t_sleep_s = 0.0;

    /* Charge consumed per cycle [mC] */
    double q_per_cycle_mc =
        node->sleep_current_ua * t_sleep_s / 1000.0 +      /* uA*s → mC */
        node->active_current_ma * t_meas_s +                /* mA*s = mC */
        node->tx_current_ma * t_tx_s;                       /* mA*s = mC */

    /* Average current [mA] */
    double i_avg_ma = q_per_cycle_mc / t_total_s;

    if (i_avg_ma <= 0.0) return HUGE_VAL;  /* Infinite life */

    return node->battery_capacity_mah / i_avg_ma;
}

double application_edge_process(double raw_sample, double prev_reported,
                                double threshold_ue, int *should_report)
{
    /* Edge processor: decide whether to transmit a sample.
     *
     * Delta-based reporting:
     *   If |raw_sample - prev_reported| > threshold → report
     *   Else → suppress transmission
     *
     * This is the simplest form of data compression for
     * wireless sensor nodes. More sophisticated methods
     * include:
     * - LTC (Lightweight Temporal Compression)
     * - Predictive coding
     * - Event detection (report only on significant events)
     * - Histogram accumulation (report statistical summaries)
     *
     * Delta compression can reduce wireless data by 10-100x
     * for slowly-varying signals like structural strain.
     */

    double delta = fabs(raw_sample - prev_reported);

    if (should_report != NULL) {
        *should_report = (delta > threshold_ue) ? 1 : 0;
    }

    return raw_sample;
}

/* ========================================================================
 * Initialization Functions
 * ======================================================================== */

void loadcell_spec_init(loadcell_spec_t *lc, loadcell_type_t type)
{
    memset(lc, 0, sizeof(*lc));
    lc->type = type;
    lc->rated_capacity_kg        = 100.0;
    lc->rated_output_mv_per_v    = 2.0;
    lc->max_intervals            = 3000;
    lc->min_dead_load_percent    = 0.0;
    lc->safe_overload_percent    = 150.0;
    lc->ultimate_overload_percent = 300.0;
    lc->combined_error_percent   = 0.02;
    lc->creep_percent_30min      = 0.02;
    lc->temp_effect_zero_percent = 0.002;
    lc->temp_effect_span_percent = 0.002;
    lc->excitation_max_v         = 15.0;
    lc->input_resistance_ohm     = 400.0;
    lc->output_resistance_ohm    = 350.0;
    lc->insulation_resistance_mohm = 5000.0;
}

void pressure_sensor_spec_init(pressure_sensor_spec_t *ps, pressure_type_t type)
{
    memset(ps, 0, sizeof(*ps));
    ps->type = type;
    ps->pressure_range_kpa             = 100.0;   /* 100 kPa ≈ 1 atm */
    ps->overpressure_kpa               = 300.0;
    ps->burst_pressure_kpa             = 500.0;
    ps->sensitivity_mv_per_v_per_kpa   = 0.05;     /* 0.05 mV/V/kPa */
    ps->nonlinearity_percent           = 0.1;
    ps->bridge_resistance_ohm          = 5000.0;    /* MEMS typical */
    ps->tempco_zero_percent_fs_per_c   = 0.04;
    ps->tempco_span_percent_fs_per_c   = -0.20;     /* Negative for Si */
    ps->response_time_ms               = 1.0;
    ps->resonant_frequency_khz         = 50.0;
    ps->media_compatibility            = "dry air, non-corrosive gas";
}

void torque_sensor_spec_init(torque_sensor_spec_t *ts)
{
    memset(ts, 0, sizeof(*ts));
    ts->torque_range_nm          = 100.0;
    ts->max_speed_rpm            = 10000.0;
    ts->shaft_diameter_mm        = 25.0;
    ts->shaft_material_E_gpa     = 200.0;
    ts->shaft_material_nu        = 0.29;
    ts->rated_output_mv_per_v    = 1.5;
    ts->bridge_resistance_ohm    = 350.0;
    ts->nonlinearity_percent     = 0.05;
    ts->bandwidth_khz            = 5.0;
}

void weighing_system_init(weighing_system_t *ws)
{
    memset(ws, 0, sizeof(*ws));
    ws->n_cells = 4;
    int i;
    for (i = 0; i < 4; i++) {
        loadcell_spec_init(&ws->cells[i], LOADCELL_SHEAR_BEAM);
        ws->corner_factors[i] = 1.0;
    }
    ws->platform_mass_kg    = 50.0;
    ws->max_capacity_kg     = 1000.0;
    ws->resolution_kg       = 0.1;
    ws->legal_for_trade     = 1;
    ws->zero_tracking_kg    = 2.0;
    ws->tare_max_percent    = 100.0;
    ws->motion_detection_kg = 0.5;
}

void automotive_pressure_init(automotive_pressure_t *ap)
{
    memset(ap, 0, sizeof(*ap));
    pressure_sensor_spec_init(&ap->sensor, PRESSURE_ABSOLUTE);
    ap->supply_voltage_v     = 5.0;
    ap->adc_reference_v      = 5.0;
    ap->adc_bits             = 10;
    ap->clamp_voltage_min_v  = 0.25;
    ap->clamp_voltage_max_v  = 4.75;
    ap->pullup_resistor_ohm  = 10000.0;
    ap->pulldown_resistor_ohm = 100000.0;
}

void aerospace_shm_init(aerospace_shm_t *shm)
{
    memset(shm, 0, sizeof(*shm));
    bridge_sensor_init(&shm->sensor_installation);
    shm->fatigue_notch_factor   = 1.0;
    shm->max_recorded_strain_ue = 0.0;
    shm->cumulative_damage      = 0.0;
    shm->n_cycles_recorded      = 0;
    shm->install_date           = 24000.0;  /* Julian date placeholder */
    shm->last_cal_date          = 24000.0;
}

void wireless_sensor_node_init(wireless_sensor_node_t *node)
{
    memset(node, 0, sizeof(*node));
    bridge_sensor_init(&node->sensor);
    signal_chain_init(&node->signal_chain);
    node->transmit_interval_s   = 60.0;       /* 1 minute */
    node->battery_capacity_mah  = 2000.0;     /* AA Li-SOCl2 */
    node->sleep_current_ua      = 5.0;
    node->active_current_ma     = 2.0;
    node->measurement_time_ms   = 10.0;
    node->tx_time_ms            = 5.0;
    node->tx_current_ma         = 15.0;
    node->data_points_per_tx    = 1;
}
