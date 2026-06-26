/**
 * @file    tia_core.c
 * @brief   Core TIA Implementations - L1 through L6
 *
 * @details Photodiode modeling, op-amp parameter lookup, basic TIA
 *          design equations, frequency response, step response, and
 *          optical link budget for transimpedance amplifier design.
 *
 * Knowledge: L1 photodiode/opamp models, L2 virtual ground+GBW tradeoff,
 *            L3 s-domain transfer functions, L4 Johnson noise+GBW relation,
 *            L5 TIA design methodology, L6 basic TIA design problem.
 *
 * References: Graeme (1996), Hobbs (2011), Horowitz & Hill (2015),
 *             Sedra & Smith (2020), Agrawal (2021).
 */

#include "tia_core.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

static void* safe_malloc(size_t sz) {
    void *p = malloc(sz);
    if (!p && sz > 0) { fprintf(stderr, "tia_core: malloc(%zu) failed\n", sz); abort(); }
    return p;
}

/* ==== L1: Photodiode Model Initialization ==== */

photodiode_model_t photodiode_model_init(photodiode_type_t type,
                                          double area_mm2, double bias_v)
{
    photodiode_model_t pd;
    memset(&pd, 0, sizeof(pd));
    pd.type = type;
    pd.active_area_mm2 = (area_mm2 > 0.0) ? area_mm2 : 1.0;
    pd.reverse_bias_voltage = bias_v;
    pd.bias_mode = (bias_v > 0.01) ? BIAS_PHOTOCONDUCTIVE : BIAS_PHOTOVOLTAIC;
    switch (type) {
    case PHOTODIODE_SI_PIN:
        snprintf(pd.model_name, sizeof(pd.model_name), "Si-PIN-%.0fmm2", area_mm2);
        pd.responsivity_a_per_w = 0.55;
        pd.quantum_efficiency = 0.8;
        pd.peak_wavelength_nm = 850.0;
        pd.spectral_range_low_nm = 320.0;
        pd.spectral_range_high_nm = 1100.0;
        pd.junction_capacitance_pf = 10.0 * area_mm2;
        pd.shunt_resistance_ohm = 1000000000.0 / area_mm2;
        pd.dark_current_na = 0.02 * area_mm2;
        pd.dark_current_tempco = 10.0;
        pd.breakdown_voltage = 60.0;
        pd.noise_equivalent_power = 1.5e-14 * sqrt(area_mm2);
        pd.specific_detectivity = 5000000000000.0 * sqrt(area_mm2);
        pd.rise_time_ns = 5.0 * sqrt(area_mm2);
        pd.package_capacitance_pf = 0.5;
        break;
    case PHOTODIODE_INGAAS_PIN:
        snprintf(pd.model_name, sizeof(pd.model_name), "InGaAs-PIN-%.0fmm2", area_mm2);
        pd.responsivity_a_per_w = 0.95;
        pd.quantum_efficiency = 0.85;
        pd.peak_wavelength_nm = 1550.0;
        pd.spectral_range_low_nm = 800.0;
        pd.spectral_range_high_nm = 1700.0;
        pd.junction_capacitance_pf = 5.0 * area_mm2;
        pd.shunt_resistance_ohm = 100000000.0 / area_mm2;
        pd.dark_current_na = 0.5 * area_mm2;
        pd.dark_current_tempco = 8.0;
        pd.breakdown_voltage = 25.0;
        pd.noise_equivalent_power = 5e-15 * sqrt(area_mm2);
        pd.specific_detectivity = 10000000000000.0 * sqrt(area_mm2);
        pd.rise_time_ns = 1.0 * sqrt(area_mm2);
        pd.package_capacitance_pf = 0.3;
        break;
    case PHOTODIODE_GE_PIN:
        snprintf(pd.model_name, sizeof(pd.model_name), "Ge-PIN-%.0fmm2", area_mm2);
        pd.responsivity_a_per_w = 0.7;
        pd.quantum_efficiency = 0.65;
        pd.peak_wavelength_nm = 1550.0;
        pd.spectral_range_low_nm = 800.0;
        pd.spectral_range_high_nm = 1800.0;
        pd.junction_capacitance_pf = 8.0 * area_mm2;
        pd.shunt_resistance_ohm = 10000000.0 / area_mm2;
        pd.dark_current_na = 100.0 * area_mm2;
        pd.dark_current_tempco = 8.0;
        pd.breakdown_voltage = 20.0;
        pd.noise_equivalent_power = 2e-14 * sqrt(area_mm2);
        pd.specific_detectivity = 3000000000000.0 * sqrt(area_mm2);
        pd.rise_time_ns = 2.0 * sqrt(area_mm2);
        pd.package_capacitance_pf = 0.5;
        break;
    case PHOTODIODE_SI_APD:
        snprintf(pd.model_name, sizeof(pd.model_name), "Si-APD-%.0fmm2", area_mm2);
        pd.responsivity_a_per_w = 50.0;
        pd.quantum_efficiency = 0.75;
        pd.peak_wavelength_nm = 800.0;
        pd.spectral_range_low_nm = 400.0;
        pd.spectral_range_high_nm = 1000.0;
        pd.junction_capacitance_pf = 1.5 * area_mm2;
        pd.shunt_resistance_ohm = 1000000000.0 / area_mm2;
        pd.dark_current_na = 0.5 * area_mm2;
        pd.dark_current_tempco = 10.0;
        pd.breakdown_voltage = 200.0;
        pd.noise_equivalent_power = 1e-15 * sqrt(area_mm2);
        pd.specific_detectivity = 100000000000000.0 * sqrt(area_mm2);
        pd.rise_time_ns = 0.5 * sqrt(area_mm2);
        pd.package_capacitance_pf = 0.3;
        break;
    case PHOTODIODE_INGAAS_APD:
        snprintf(pd.model_name, sizeof(pd.model_name), "InGaAs-APD-%.0fmm2", area_mm2);
        pd.responsivity_a_per_w = 8.5;
        pd.quantum_efficiency = 0.8;
        pd.peak_wavelength_nm = 1550.0;
        pd.spectral_range_low_nm = 900.0;
        pd.spectral_range_high_nm = 1700.0;
        pd.junction_capacitance_pf = 0.5 * area_mm2;
        pd.shunt_resistance_ohm = 100000000.0 / area_mm2;
        pd.dark_current_na = 10.0 * area_mm2;
        pd.dark_current_tempco = 8.0;
        pd.breakdown_voltage = 60.0;
        pd.noise_equivalent_power = 5e-16 * sqrt(area_mm2);
        pd.specific_detectivity = 20000000000000.0 * sqrt(area_mm2);
        pd.rise_time_ns = 0.3 * sqrt(area_mm2);
        pd.package_capacitance_pf = 0.2;
        break;
    case PHOTODIODE_SIPM:
        snprintf(pd.model_name, sizeof(pd.model_name), "SiPM-%.0fmm2", area_mm2);
        pd.responsivity_a_per_w = 100000.0;
        pd.quantum_efficiency = 0.4;
        pd.peak_wavelength_nm = 450.0;
        pd.spectral_range_low_nm = 300.0;
        pd.spectral_range_high_nm = 900.0;
        pd.junction_capacitance_pf = 35.0 * area_mm2;
        pd.shunt_resistance_ohm = 1000000.0 / area_mm2;
        pd.dark_current_na = 1000.0 * area_mm2;
        pd.dark_current_tempco = 8.0;
        pd.breakdown_voltage = 30.0;
        pd.noise_equivalent_power = 1e-16 * sqrt(area_mm2);
        pd.specific_detectivity = 100000000000000.0 * sqrt(area_mm2);
        pd.rise_time_ns = 1.0 * sqrt(area_mm2);
        pd.package_capacitance_pf = 1.0;
        break;
    case PHOTODIODE_QUAD:
        snprintf(pd.model_name, sizeof(pd.model_name), "Quad-%.0fmm2", area_mm2);
        pd.responsivity_a_per_w = 0.55;
        pd.quantum_efficiency = 0.78;
        pd.peak_wavelength_nm = 950.0;
        pd.spectral_range_low_nm = 400.0;
        pd.spectral_range_high_nm = 1100.0;
        pd.junction_capacitance_pf = 8.0 * area_mm2;
        pd.shunt_resistance_ohm = 500000000.0 / area_mm2;
        pd.dark_current_na = 0.05 * area_mm2;
        pd.dark_current_tempco = 10.0;
        pd.breakdown_voltage = 50.0;
        pd.noise_equivalent_power = 2e-14 * sqrt(area_mm2);
        pd.specific_detectivity = 4000000000000.0 * sqrt(area_mm2);
        pd.rise_time_ns = 10.0 * sqrt(area_mm2);
        pd.package_capacitance_pf = 1.0;
        break;
    case PHOTODIODE_CUSTOM:
    default:
        snprintf(pd.model_name, sizeof(pd.model_name), "Custom-PD");
        pd.responsivity_a_per_w = 0.50; pd.quantum_efficiency = 0.70;
        pd.peak_wavelength_nm = 850.0;
        pd.spectral_range_low_nm = 400.0; pd.spectral_range_high_nm = 1100.0;
        pd.junction_capacitance_pf = 10.0; pd.shunt_resistance_ohm = 1.0e9;
        pd.dark_current_na = 0.1; pd.dark_current_tempco = 10.0;
        pd.breakdown_voltage = 50.0; pd.noise_equivalent_power = 1.0e-14;
        pd.specific_detectivity = 5.0e12; pd.rise_time_ns = 5.0;
        pd.package_capacitance_pf = 0.5;
        break;
    }

    if (bias_v > 0.001) {
        double vbi = (type == PHOTODIODE_INGAAS_PIN ||
                      type == PHOTODIODE_INGAAS_APD) ? 0.75 : 0.65;
        double factor = sqrt(1.0 + bias_v / vbi);
        if (factor > 0.01) pd.junction_capacitance_pf /= factor;
    }
    return pd;
}
/* ==== L1: Area Scaling ==== */

photodiode_model_t photodiode_scale_area(const photodiode_model_t *pd, double area_mm2)
{
    photodiode_model_t scaled;
    if (!pd || area_mm2 <= 0.0) { memset(&scaled, 0, sizeof(scaled)); return scaled; }
    scaled = *pd;
    double ratio = area_mm2 / pd->active_area_mm2;
    scaled.active_area_mm2 = area_mm2;
    scaled.junction_capacitance_pf = pd->junction_capacitance_pf * ratio;
    scaled.shunt_resistance_ohm = pd->shunt_resistance_ohm / ratio;
    scaled.dark_current_na = pd->dark_current_na * ratio;
    scaled.noise_equivalent_power = pd->noise_equivalent_power * sqrt(ratio);
    scaled.specific_detectivity = pd->specific_detectivity;
    scaled.rise_time_ns = pd->rise_time_ns * sqrt(ratio);
    snprintf(scaled.model_name, sizeof(scaled.model_name), "%s-s%.1f", pd->model_name, area_mm2);
    return scaled;
}

/* ==== L3: Junction Capacitance vs Bias ==== */

double photodiode_cj_at_bias(const photodiode_model_t *pd, double vr)
{
    if (!pd || vr < 0.0) return 0.0;
    double vbi;
    switch (pd->type) {
    case PHOTODIODE_INGAAS_PIN: case PHOTODIODE_INGAAS_APD: vbi = 0.75; break;
    case PHOTODIODE_GE_PIN: vbi = 0.35; break;
    default: vbi = 0.65; break;
    }
    if (vr < 0.001) return pd->junction_capacitance_pf;
    return pd->junction_capacitance_pf / sqrt(1.0 + vr / vbi);
}

/* ==== L1: Op-Amp Parameter Database ==== */

opamp_params_t opamp_params_init(const char *part_number)
{
    opamp_params_t opa;
    memset(&opa, 0, sizeof(opa));
    if (!part_number) return opa;
    strncpy(opa.part_number, part_number, sizeof(opa.part_number) - 1);
    if (strcmp(part_number, "OPA657") == 0) {
        opa.gain_bandwidth_mhz = 1600.0; opa.unity_gain_stable = 1.0;
        opa.open_loop_gain_db = 70.0; opa.input_voltage_noise_nv = 4.8;
        opa.input_current_noise_fa = 1.3;
        opa.input_capacitance_cm_pf = 0.7; opa.input_capacitance_diff_pf = 4.5;
        opa.input_resistance_ohm = 1000000000000.0; opa.slew_rate_v_per_us = 700.0;
        opa.output_voltage_swing_v = 7.0;
        opa.supply_voltage_min = 8.0; opa.supply_voltage_max = 12.0;
        opa.supply_current_ma = 14.0; opa.corner_freq_1f_hz = 100.0;
        opa.output_impedance_ohm = 0.01;
    }
    if (strcmp(part_number, "OPA656") == 0) {
        opa.gain_bandwidth_mhz = 500.0; opa.unity_gain_stable = 1.0;
        opa.open_loop_gain_db = 65.0; opa.input_voltage_noise_nv = 7.0;
        opa.input_current_noise_fa = 1.2;
        opa.input_capacitance_cm_pf = 0.7; opa.input_capacitance_diff_pf = 2.8;
        opa.input_resistance_ohm = 1000000000000.0; opa.slew_rate_v_per_us = 290.0;
        opa.output_voltage_swing_v = 6.0;
        opa.supply_voltage_min = 8.0; opa.supply_voltage_max = 12.0;
        opa.supply_current_ma = 14.0; opa.corner_freq_1f_hz = 150.0;
        opa.output_impedance_ohm = 0.01;
    }
    if (strcmp(part_number, "LTC6268") == 0) {
        opa.gain_bandwidth_mhz = 500.0; opa.unity_gain_stable = 1.0;
        opa.open_loop_gain_db = 90.0; opa.input_voltage_noise_nv = 4.3;
        opa.input_current_noise_fa = 0.0055;
        opa.input_capacitance_cm_pf = 0.45; opa.input_capacitance_diff_pf = 0.55;
        opa.input_resistance_ohm = 100000000000000.0; opa.slew_rate_v_per_us = 400.0;
        opa.output_voltage_swing_v = 4.8;
        opa.supply_voltage_min = 3.1; opa.supply_voltage_max = 5.25;
        opa.supply_current_ma = 16.5; opa.corner_freq_1f_hz = 2000.0;
        opa.output_impedance_ohm = 0.1;
    }
    if (strcmp(part_number, "ADA4817") == 0) {
        opa.gain_bandwidth_mhz = 1050.0; opa.unity_gain_stable = 1.0;
        opa.open_loop_gain_db = 65.0; opa.input_voltage_noise_nv = 4.0;
        opa.input_current_noise_fa = 2.5;
        opa.input_capacitance_cm_pf = 1.3; opa.input_capacitance_diff_pf = 1.5;
        opa.input_resistance_ohm = 1000000000000.0; opa.slew_rate_v_per_us = 870.0;
        opa.output_voltage_swing_v = 7.0;
        opa.supply_voltage_min = 5.0; opa.supply_voltage_max = 10.0;
        opa.supply_current_ma = 19.0; opa.corner_freq_1f_hz = 500.0;
        opa.output_impedance_ohm = 0.01;
    }
    if (strcmp(part_number, "OPA847") == 0) {
        opa.gain_bandwidth_mhz = 3900.0; opa.unity_gain_stable = 0.0;
        opa.open_loop_gain_db = 60.0; opa.input_voltage_noise_nv = 0.85;
        opa.input_current_noise_fa = 2500.0;
        opa.input_capacitance_cm_pf = 1.2; opa.input_capacitance_diff_pf = 2.5;
        opa.input_resistance_ohm = 300000.0; opa.slew_rate_v_per_us = 950.0;
        opa.output_voltage_swing_v = 6.0;
        opa.supply_voltage_min = 9.0; opa.supply_voltage_max = 12.0;
        opa.supply_current_ma = 18.1; opa.corner_freq_1f_hz = 60.0;
        opa.output_impedance_ohm = 0.01;
    }
    if (strcmp(part_number, "OPA380") == 0) {
        opa.gain_bandwidth_mhz = 90.0; opa.unity_gain_stable = 1.0;
        opa.open_loop_gain_db = 130.0; opa.input_voltage_noise_nv = 5.8;
        opa.input_current_noise_fa = 0.01;
        opa.input_capacitance_cm_pf = 6.5; opa.input_capacitance_diff_pf = 8.0;
        opa.input_resistance_ohm = 10000000000000.0; opa.slew_rate_v_per_us = 80.0;
        opa.output_voltage_swing_v = 4.9;
        opa.supply_voltage_min = 2.7; opa.supply_voltage_max = 5.5;
        opa.supply_current_ma = 7.5; opa.corner_freq_1f_hz = 500.0;
        opa.output_impedance_ohm = 0.5;
    }
    if (strcmp(part_number, "LMP7721") == 0) {
        opa.gain_bandwidth_mhz = 17.0; opa.unity_gain_stable = 1.0;
        opa.open_loop_gain_db = 120.0; opa.input_voltage_noise_nv = 6.5;
        opa.input_current_noise_fa = 0.01;
        opa.input_capacitance_cm_pf = 10.0; opa.input_capacitance_diff_pf = 12.0;
        opa.input_resistance_ohm = 100000000000000.0; opa.slew_rate_v_per_us = 12.8;
        opa.output_voltage_swing_v = 4.9;
        opa.supply_voltage_min = 1.8; opa.supply_voltage_max = 5.5;
        opa.supply_current_ma = 1.3; opa.corner_freq_1f_hz = 10.0;
        opa.output_impedance_ohm = 5.0;
    }
    if (strcmp(part_number, "AD8015") == 0) {
        opa.gain_bandwidth_mhz = 240.0; opa.unity_gain_stable = 1.0;
        opa.open_loop_gain_db = 55.0; opa.input_voltage_noise_nv = 3.0;
        opa.input_current_noise_fa = 18000.0;
        opa.input_capacitance_cm_pf = 1.5; opa.input_capacitance_diff_pf = 1.0;
        opa.input_resistance_ohm = 50.0; opa.slew_rate_v_per_us = 2500.0;
        opa.output_voltage_swing_v = 0.8;
        opa.supply_voltage_min = 4.5; opa.supply_voltage_max = 11.0;
        opa.supply_current_ma = 25.0; opa.corner_freq_1f_hz = 0.0;
        opa.output_impedance_ohm = 50.0;
    }
    if (strcmp(part_number, "THS4631") == 0) {
        opa.gain_bandwidth_mhz = 325.0; opa.unity_gain_stable = 1.0;
        opa.open_loop_gain_db = 80.0; opa.input_voltage_noise_nv = 7.0;
        opa.input_current_noise_fa = 2.0;
        opa.input_capacitance_cm_pf = 1.2; opa.input_capacitance_diff_pf = 3.4;
        opa.input_resistance_ohm = 1000000000.0; opa.slew_rate_v_per_us = 1000.0;
        opa.output_voltage_swing_v = 24.0;
        opa.supply_voltage_min = 10.0; opa.supply_voltage_max = 30.0;
        opa.supply_current_ma = 11.5; opa.corner_freq_1f_hz = 200.0;
        opa.output_impedance_ohm = 0.01;
    }
    if (strcmp(part_number, "OPA827") == 0) {
        opa.gain_bandwidth_mhz = 22.0; opa.unity_gain_stable = 1.0;
        opa.open_loop_gain_db = 120.0; opa.input_voltage_noise_nv = 4.0;
        opa.input_current_noise_fa = 2.2;
        opa.input_capacitance_cm_pf = 3.0; opa.input_capacitance_diff_pf = 9.0;
        opa.input_resistance_ohm = 10000000000000.0; opa.slew_rate_v_per_us = 28.0;
        opa.output_voltage_swing_v = 26.0;
        opa.supply_voltage_min = 8.0; opa.supply_voltage_max = 36.0;
        opa.supply_current_ma = 4.8; opa.corner_freq_1f_hz = 1.0;
        opa.output_impedance_ohm = 1.0;
    }
    return opa;
}
/* ==== L2: Total Input Capacitance ==== */

double tia_input_capacitance(const photodiode_model_t *pd,
                              const opamp_params_t *opa, double stray)
{
    if (!pd || !opa) return 1.0;
    double cin = pd->junction_capacitance_pf + pd->package_capacitance_pf +
                 opa->input_capacitance_cm_pf + opa->input_capacitance_diff_pf + stray;
    return (cin > 0.01) ? cin : 1.0;
}

/* ==== L5: Feedback Compensation Capacitance Design ==== */

double tia_compensation_capacitance(const photodiode_model_t *pd,
                                     const opamp_params_t *opa,
                                     double rf, double target_pm)
{
    if (!pd || !opa || rf <= 0.0 || target_pm <= 0.0) return 0.5;
    double cin = tia_input_capacitance(pd, opa, 0.5);
    double gbw = opa->gain_bandwidth_mhz * 1.0e6;
    double two_pi = 2.0 * M_PI;
    double pm_rad = target_pm * M_PI / 180.0;
    double tan_arg = M_PI / 2.0 - pm_rad;
    if (tan_arg <= 0.0 || target_pm >= 90.0) {
        return sqrt(cin * 1.0e-12 / (two_pi * rf * gbw)) * 1.0e12;
    }
    double f_p = 1.0 / (two_pi * rf * cin * 1.0e-12);
    double tan_val = tan(tan_arg);
    if (tan_val < 1.0e-6) tan_val = 1.0e-6;
    double f_cross = f_p * tan_val;
    if (f_cross > gbw) f_cross = gbw * 0.8;
    double f_z = (f_cross * f_cross) / gbw;
    if (f_z < 1.0) f_z = 1.0;
    double cf = 1.0 / (two_pi * rf * f_z) * 1.0e12;
    if (cf < 0.01) cf = 0.01;
    if (cf > 100.0) cf = 100.0;
    return cf;
}

/* ==== L5: Basic TIA Design ==== */

tia_design_t tia_design_basic(const photodiode_model_t *pd,
                               const opamp_params_t *opa,
                               double gain_target, double bw_target_hz)
{
    tia_design_t design;
    memset(&design, 0, sizeof(design));
    if (!pd || !opa || gain_target <= 0.0) return design;
    design.photodiode = *pd;
    design.opamp = *opa;
    design.topology = TIA_TOPOLOGY_BASIC;
    design.bias = pd->bias_mode;
    design.rf_ohm = gain_target;
    design.total_input_capacitance_pf = tia_input_capacitance(pd, opa, 0.5);
    design.cf_pf = tia_compensation_capacitance(pd, opa, gain_target, 65.0);
    double two_pi = 2.0 * M_PI;
    double cin = design.total_input_capacitance_pf * 1.0e-12;
    double gbw = opa->gain_bandwidth_mhz * 1.0e6;
    double rf = design.rf_ohm;
    double cf = design.cf_pf * 1.0e-12;
    double f_3db_est = sqrt(gbw / (two_pi * rf * cin));
    if (f_3db_est > gbw * 0.5) f_3db_est = gbw * 0.5;
    double f_cf = 1.0 / (two_pi * rf * cf);
    if (f_cf < f_3db_est) f_3db_est = f_cf;
    if (bw_target_hz > 0.0) {
        if (bw_target_hz > f_3db_est * 1.2) f_3db_est *= 1.2;
        else f_3db_est = bw_target_hz;
    }
    design.bandwidth_3db_hz = f_3db_est;
    design.bandwidth_3db_mhz = f_3db_est / 1.0e6;
    design.transimpedance_gain_ohm = rf;
    design.transimpedance_gain_db = 20.0 * log10(rf);
    design.signal_bandwidth_product = rf * f_3db_est;
    double pm_est = 90.0 - atan(f_3db_est * two_pi * rf * cin) * 180.0 / M_PI;
    if (pm_est < 0.0) pm_est = 0.0;
    if (pm_est > 90.0) pm_est = 90.0;
    design.phase_margin_deg = pm_est;
    design.gain_margin_db = (pm_est > 30.0) ? 10.0 : 3.0;
    design.rise_time_ns = 0.35 / f_3db_est * 1.0e9;
    design.overshoot_percent = (pm_est < 50.0) ? (50.0 - pm_est) : 0.0;
    design.settling_time_ns = design.rise_time_ns * 3.0;
    design.group_delay_ns = 1.0 / (two_pi * f_3db_est) * 1.0e9;
    design.input_resistance_ohm = rf / pow(10.0, opa->open_loop_gain_db / 20.0);
    double johnson_i = sqrt(4.0 * BOLTZMANN_CONSTANT * TEMPERATURE_STANDARD / rf);
    double shot_i = sqrt(2.0 * ELECTRON_CHARGE * pd->dark_current_na * 1.0e-9);
    double en_noise = opa->input_voltage_noise_nv * 1.0e-9 * two_pi * f_3db_est * cin;
    double in_noise = opa->input_current_noise_fa * 1.0e-15;
    double total_i = sqrt(johnson_i*johnson_i + shot_i*shot_i + en_noise*en_noise + in_noise*in_noise);
    design.total_input_noise_pa = total_i * sqrt(f_3db_est) * 1.0e12;
    design.total_output_noise_mv = design.total_input_noise_pa * rf * 1.0e-6;
    design.input_noise_density_pa = total_i * 1.0e12;
    design.nepo_w_per_sqrt_hz = design.input_noise_density_pa * 1.0e-12 / pd->responsivity_a_per_w;
    double q_ber12 = 7.0;
    double p_sens_w = q_ber12 * design.total_input_noise_pa * 1.0e-12 / pd->responsivity_a_per_w;
    design.sensitivity_dbm = 10.0 * log10(p_sens_w * 1000.0);
    design.dc_output_offset_v = (pd->dark_current_na * 1.0e-9 + opa->input_current_noise_fa * 1.0e-15) * rf;
    design.dc_output_drift_uv_per_c = fabs(design.dc_output_offset_v) * 0.01 * 1.0e6;
    double swing = opa->output_voltage_swing_v;
    design.max_photocurrent_ua = (swing - fabs(design.dc_output_offset_v)) / rf * 1.0e6;
    if (design.max_photocurrent_ua < 0.0) design.max_photocurrent_ua = 0.0;
    design.saturation_power_dbm = 10.0 * log10(design.max_photocurrent_ua * 1.0e-6 / pd->responsivity_a_per_w * 1000.0);
    return design;
}

/* ==== L2: 3dB Bandwidth ==== */

double tia_bandwidth_3db(const tia_design_t *design)
{
    if (!design || design->rf_ohm <= 0.0) return 0.0;
    double two_pi = 2.0 * M_PI;
    double cin = design->total_input_capacitance_pf * 1.0e-12;
    double gbw = design->opamp.gain_bandwidth_mhz * 1.0e6;
    double rf = design->rf_ohm;
    double cf = design->cf_pf * 1.0e-12;
    double f_gbw = sqrt(gbw / (two_pi * rf * cin));
    double f_cf = 1.0 / (two_pi * rf * cf);
    double f_3db = (f_gbw < f_cf) ? f_gbw : f_cf;
    if (design->phase_margin_deg < 60.0 && design->phase_margin_deg > 0.0) {
        double zeta = sin(design->phase_margin_deg * M_PI / 360.0);
        if (zeta < 0.707) {
            double z2 = zeta * zeta;
            f_3db *= sqrt(sqrt(4.0*z2*z2 - 4.0*z2 + 2.0) + 1.0 - 2.0*z2);
        }
    }
    return f_3db;
}


/* ==== L2: Gain at Frequency ==== */

double tia_gain_at_frequency(const tia_design_t *design, double freq_hz)
{
    if (!design || design->rf_ohm <= 0.0 || freq_hz < 0.0) return 0.0;
    double f_3db = design->bandwidth_3db_hz;
    if (f_3db <= 0.0) return design->rf_ohm;
    if (freq_hz > 10.0 * f_3db)
        return design->rf_ohm * f_3db / freq_hz;
    return design->rf_ohm / sqrt(1.0 + (freq_hz/f_3db)*(freq_hz/f_3db));
}

/* ==== L5: Frequency Response Computation ==== */

tia_freq_response_t tia_compute_frequency_response(const tia_design_t *design,
                                                    double freq_start,
                                                    double freq_stop,
                                                    size_t points)
{
    tia_freq_response_t resp;
    memset(&resp, 0, sizeof(resp));
    if (!design || points < 2 || freq_start <= 0.0 || freq_stop <= freq_start)
        return resp;
    resp.num_points = points;
    resp.freq_hz = (double*)safe_malloc(points * sizeof(double));
    resp.magnitude_db = (double*)safe_malloc(points * sizeof(double));
    resp.phase_deg = (double*)safe_malloc(points * sizeof(double));
    resp.group_delay_ns = (double*)safe_malloc(points * sizeof(double));

    double rf = design->rf_ohm;
    double two_pi = 2.0 * M_PI;
    double zeta = sin(design->phase_margin_deg * M_PI / 360.0);
    if (zeta < 0.1) zeta = 0.1;
    double omega_n = two_pi * design->bandwidth_3db_hz;
    if (omega_n < two_pi * freq_start) omega_n = two_pi * freq_start * 10.0;

    double log_start = log10(freq_start);
    double log_stop  = log10(freq_stop);
    double delta = (log_stop - log_start) / (double)(points - 1);

    double f_3db = 0.0, peak_db = 0.0, f_peak = freq_start, phase_at_3db = -45.0;

    for (size_t i = 0; i < points; i++) {
        double f = pow(10.0, log_start + delta * i);
        resp.freq_hz[i] = f;
        double w = two_pi * f;
        double den_re = omega_n * omega_n - w * w;
        double den_im = 2.0 * zeta * omega_n * w;
        double den_mag = sqrt(den_re * den_re + den_im * den_im);
        double mag = (den_mag > 1.0e-30) ? rf * omega_n * omega_n / den_mag : rf;
        if (mag > rf * 5.0) mag = rf * 5.0;
        resp.magnitude_db[i] = 20.0 * log10(mag);
        resp.phase_deg[i] = -atan2(den_im, den_re) * 180.0 / M_PI;
        double dc_db = 20.0 * log10(rf);
        if (f_3db == 0.0 && resp.magnitude_db[i] < dc_db - 3.0) {
            f_3db = f; phase_at_3db = resp.phase_deg[i];
        }
        if (resp.magnitude_db[i] > peak_db) {
            peak_db = resp.magnitude_db[i]; f_peak = f;
        }
    }

    for (size_t i = 1; i < points - 1; i++) {
        double dw = two_pi * (resp.freq_hz[i + 1] - resp.freq_hz[i - 1]);
        double dphi_rad = (resp.phase_deg[i + 1] - resp.phase_deg[i - 1])
                          * M_PI / 180.0;
        if (dw > 0.0) {
            resp.group_delay_ns[i] = -dphi_rad / dw * 1.0e9;
        }
    }
    if (points >= 2) {
        resp.group_delay_ns[0] = resp.group_delay_ns[1];
        resp.group_delay_ns[points - 1] = resp.group_delay_ns[points - 2];
    }

    resp.f_3db_hz = (f_3db > 0.0) ? f_3db : design->bandwidth_3db_hz;
    resp.f_peak_hz = f_peak;
    resp.peaking_db = peak_db - 20.0 * log10(rf);
    if (resp.peaking_db < 0.0) resp.peaking_db = 0.0;
    resp.phase_at_3db_deg = phase_at_3db;
    return resp;
}

/* ==== L5: Bode Analysis ==== */

tia_bode_data_t tia_compute_bode(const tia_design_t *design,
                                  double freq_start, double freq_stop,
                                  size_t points)
{
    tia_bode_data_t bode;
    memset(&bode, 0, sizeof(bode));
    if (!design || points < 2 || freq_start <= 0.0 || freq_stop <= freq_start)
        return bode;

    bode.num_points = points;
    bode.freq_hz = (double*)safe_malloc(points * sizeof(double));
    bode.aol_db = (double*)safe_malloc(points * sizeof(double));
    bode.aol_phase_deg = (double*)safe_malloc(points * sizeof(double));
    bode.beta_db = (double*)safe_malloc(points * sizeof(double));
    bode.loop_gain_db = (double*)safe_malloc(points * sizeof(double));
    bode.loop_phase_deg = (double*)safe_malloc(points * sizeof(double));

    double gbw = design->opamp.gain_bandwidth_mhz * 1.0e6;
    double aol_dc = design->opamp.open_loop_gain_db;
    double two_pi = 2.0 * M_PI;
    double rf = design->rf_ohm;
    double cin = design->total_input_capacitance_pf * 1.0e-12;
    double cf = design->cf_pf * 1.0e-12;
    double aol_linear = pow(10.0, aol_dc / 20.0);
    double omega_p1 = gbw * two_pi / aol_linear;

    double log_start = log10(freq_start);
    double log_stop  = log10(freq_stop);
    double delta = (log_stop - log_start) / (double)(points - 1);

    double f_cross = 0.0, pm_found = 0.0, gm_found = 0.0;

    for (size_t i = 0; i < points; i++) {
        double f = pow(10.0, log_start + delta * i);
        bode.freq_hz[i] = f;
        double w = two_pi * f;

        double aol_mag = aol_linear / sqrt(1.0 + (w / omega_p1) * (w / omega_p1));
        bode.aol_db[i] = 20.0 * log10(aol_mag);
        bode.aol_phase_deg[i] = -90.0 - atan(w / omega_p1) * 180.0 / M_PI;

        double w_rf_cf = w * rf * cf;
        double w_rf_cin_cf = w * rf * (cin + cf);
        double beta_num = sqrt(1.0 + w_rf_cf * w_rf_cf);
        double beta_den = sqrt(1.0 + w_rf_cin_cf * w_rf_cin_cf);
        double beta_mag = (beta_den > 0.0) ? beta_num / beta_den : 0.0;
        bode.beta_db[i] = 20.0 * log10(beta_mag);
        if (bode.beta_db[i] > 0.0) bode.beta_db[i] = 0.0;

        double loop_mag = aol_mag * beta_mag;
        bode.loop_gain_db[i] = 20.0 * log10(loop_mag);

        double beta_phase = atan(w_rf_cf) - atan(w_rf_cin_cf);
        double loop_phase = bode.aol_phase_deg[i] + beta_phase * 180.0 / M_PI;
        bode.loop_phase_deg[i] = loop_phase;

        if (f_cross == 0.0 && i > 0 && bode.loop_gain_db[i] <= 0.0) {
            f_cross = f; pm_found = 180.0 + loop_phase;
        }
        if (gm_found == 0.0 && i > 0 && loop_phase <= -180.0) {
            gm_found = -bode.loop_gain_db[i];
        }
    }

    bode.phase_margin_deg = (pm_found > 0.0) ? pm_found : design->phase_margin_deg;
    bode.gain_margin_db = (gm_found > 0.0) ? gm_found : design->gain_margin_db;
    bode.crossover_freq_hz = (f_cross > 0.0) ? f_cross : design->bandwidth_3db_hz;
    return bode;
}


/* ==== L6: Step Response ==== */

tia_step_response_t tia_compute_step_response(const tia_design_t *design,
                                               double i_step_ua,
                                               double duration_ns, size_t points)
{
    tia_step_response_t step;
    memset(&step, 0, sizeof(step));
    if (!design || i_step_ua <= 0.0 || duration_ns <= 0.0 || points < 2) return step;

    step.num_points = points;
    step.time_ns = (double*)safe_malloc(points * sizeof(double));
    step.output_v = (double*)safe_malloc(points * sizeof(double));

    double rf = design->rf_ohm;
    double v_final = i_step_ua * 1.0e-6 * rf;
    step.final_value_v = v_final;

    double zeta = sin(design->phase_margin_deg * M_PI / 360.0);
    if (zeta < 0.1) zeta = 0.1;
    double omega_n = 2.0 * M_PI * design->bandwidth_3db_hz;
    if (omega_n < 1.0) omega_n = 1.0;

    double dt = duration_ns / (double)(points - 1);
    double t_10 = 0.0, t_90 = 0.0;
    int found_10 = 0, found_90 = 0;
    double v_max = 0.0;

    for (size_t i = 0; i < points; i++) {
        double t_sec = i * dt * 1.0e-9;
        step.time_ns[i] = t_sec * 1.0e9;
        double sigma = zeta * omega_n * t_sec;
        double wd = omega_n * sqrt(1.0 - zeta * zeta);
        double exp_term = (sigma > 50.0) ? 0.0 : exp(-sigma);
        double v;
        if (zeta < 0.999) {
            v = v_final * (1.0 - (1.0 / sqrt(1.0 - zeta * zeta)) *
                                 exp_term * sin(wd * t_sec + acos(zeta)));
        } else {
            double tau = 1.0 / omega_n;
            v = v_final * (1.0 - exp(-t_sec / tau) * (1.0 + t_sec / tau));
        }
        if (v < 0.0) v = 0.0;
        if (v > v_final * 1.5) v = v_final * 1.5;
        step.output_v[i] = v;
        if (v > v_max) v_max = v;
        if (!found_10 && v >= 0.10 * v_final) { t_10 = t_sec * 1.0e9; found_10 = 1; }
        if (!found_90 && v >= 0.90 * v_final) { t_90 = t_sec * 1.0e9; found_90 = 1; }
    }

    step.rise_time_10_90_ns = t_90 - t_10;
    step.fall_time_90_10_ns = step.rise_time_10_90_ns;
    step.overshoot_pct = (v_max > v_final) ? (v_max - v_final) / v_final * 100.0 : 0.0;
    step.undershoot_pct = 0.0;

    double settle_time = duration_ns;
    for (size_t i = points; i > 0; i--) {
        size_t idx = i - 1;
        if (fabs(step.output_v[idx] - v_final) / v_final > 0.01) {
            settle_time = step.time_ns[idx]; break;
        }
    }
    step.settling_time_1pct_ns = settle_time;

    double t_50 = 0.0;
    for (size_t i = 0; i < points; i++) {
        if (step.output_v[i] >= 0.5 * v_final) { t_50 = step.time_ns[i]; break; }
    }
    step.propagation_delay_ns = t_50;

    double max_dvdt = 0.0;
    for (size_t i = 1; i < points; i++) {
        double dvdt = fabs(step.output_v[i] - step.output_v[i-1]) /
                      ((step.time_ns[i] - step.time_ns[i-1]) * 1.0e-3);
        if (dvdt > max_dvdt) max_dvdt = dvdt;
    }
    step.slew_rate_v_per_us = max_dvdt;
    return step;
}


/* ==== L7: Optical Link Budget ==== */

optical_link_budget_t tia_link_budget(const tia_design_t *design,
                                       double optical_power, double wavelength_nm)
{
    optical_link_budget_t link;
    memset(&link, 0, sizeof(link));
    if (!design) return link;
    link.optical_power_w = optical_power;
    link.optical_power_dbm = (optical_power > 1.0e-30) ?
        10.0 * log10(optical_power * 1000.0) : -120.0;
    link.responsivity_effective = design->photodiode.responsivity_a_per_w;
    if (wavelength_nm > 0.0 && design->photodiode.peak_wavelength_nm > 0.0) {
        double ratio = wavelength_nm / design->photodiode.peak_wavelength_nm;
        if (ratio > 0.5 && ratio < 1.5) link.responsivity_effective *= ratio;
    }
    link.photocurrent_ua = optical_power * link.responsivity_effective * 1.0e6;
    link.output_voltage_v = link.photocurrent_ua * 1.0e-6 * design->transimpedance_gain_ohm;
    double noise_rms = design->total_input_noise_pa * 1.0e-12;
    double signal_rms = link.photocurrent_ua * 1.0e-6;
    link.snr_db = (noise_rms > 0.0) ? 20.0 * log10(signal_rms / noise_rms) : 100.0;
    double q_factor = signal_rms / (noise_rms * 2.0);
    link.ber_estimate = (q_factor > 0.0) ? 0.5 * erfc(q_factor / sqrt(2.0)) : 0.5;
    if (link.ber_estimate < 1.0e-30) link.ber_estimate = 1.0e-30;
    link.link_margin_db = 20.0 * log10(q_factor / 7.03);
    link.extinction_ratio_db = 10.0;
    return link;
}

/* ==== L5: Receiver Sensitivity ==== */

double tia_sensitivity(const tia_design_t *design, double target_ber, double bitrate)
{
    if (!design || target_ber <= 0.0 || bitrate <= 0.0) return -60.0;
    double q;
    if (target_ber >= 1.0e-3)      q = 3.09;
    else if (target_ber >= 1.0e-6) q = 4.75;
    else if (target_ber >= 1.0e-9) q = 6.0;
    else if (target_ber >= 1.0e-12) q = 7.03;
    else                           q = 7.5;
    double bw_noise = 0.7 * bitrate;
    double inoise_rms = design->input_noise_density_pa * 1.0e-12 * sqrt(bw_noise);
    double required_current = q * inoise_rms;
    double required_power_w = required_current / design->photodiode.responsivity_a_per_w;
    return 10.0 * log10(required_power_w * 1000.0);
}

/* ==== L5: Performance Summary ==== */

tia_performance_summary_t tia_performance_summary(const tia_design_t *design)
{
    tia_performance_summary_t s;
    memset(&s, 0, sizeof(s));
    if (!design) return s;
    s.transimpedance_dbohm = design->transimpedance_gain_db;
    s.bandwidth_mhz = design->bandwidth_3db_mhz;
    s.input_noise_density = design->input_noise_density_pa;
    s.integrated_noise_na = design->total_input_noise_pa * 1.0e-3;
    s.sensitivity_dbm = design->sensitivity_dbm;
    s.dynamic_range_db = (design->total_input_noise_pa > 0.0) ?
        20.0 * log10(design->max_photocurrent_ua / (design->total_input_noise_pa * 1.0e-3)) : 120.0;
    s.power_consumption_mw = design->opamp.supply_current_ma * design->opamp.supply_voltage_max;
    s.figure_of_merit = design->transimpedance_gain_ohm * design->bandwidth_3db_hz /
                         (design->input_noise_density_pa * 1.0e-12 *
                          sqrt(s.power_consumption_mw * 1.0e-3));
    if (design->opamp.gain_bandwidth_mhz > 1000.0) s.cost_estimate_usd = 12.0;
    else if (design->opamp.gain_bandwidth_mhz > 100.0) s.cost_estimate_usd = 6.0;
    else s.cost_estimate_usd = 3.0;
    return s;
}

/* ==== Memory Management ==== */

void tia_freq_response_free(tia_freq_response_t *resp) {
    if (!resp) return;
    free(resp->freq_hz); free(resp->magnitude_db);
    free(resp->phase_deg); free(resp->group_delay_ns);
    memset(resp, 0, sizeof(*resp));
}

void tia_bode_data_free(tia_bode_data_t *bode) {
    if (!bode) return;
    free(bode->freq_hz); free(bode->aol_db); free(bode->aol_phase_deg);
    free(bode->beta_db); free(bode->loop_gain_db); free(bode->loop_phase_deg);
    memset(bode, 0, sizeof(*bode));
}

void tia_step_response_free(tia_step_response_t *step) {
    if (!step) return;
    free(step->time_ns); free(step->output_v);
    memset(step, 0, sizeof(*step));
}

/* ==== Utility Functions ==== */

double tia_snr_compute(double i_photo_ua, double i_noise_pa, double bandwidth_hz)
{
    if (i_noise_pa <= 0.0 || bandwidth_hz <= 0.0) return 200.0;
    double signal_rms = i_photo_ua * 1.0e-6;
    double noise_rms = i_noise_pa * 1.0e-12 * sqrt(bandwidth_hz);
    if (noise_rms <= 0.0) return 200.0;
    return 20.0 * log10(signal_rms / noise_rms);
}

double tia_required_photocurrent(double target_snr_db, double i_noise_pa)
{
    double noise_a = i_noise_pa * 1.0e-12;
    double snr_linear = pow(10.0, target_snr_db / 20.0);
    return noise_a * snr_linear * 1.0e6;
}

double tia_optical_power_from_current(double i_photo_ua, double responsivity)
{
    if (responsivity <= 0.0) return 0.0;
    return (i_photo_ua * 1.0e-6) / responsivity;
}
