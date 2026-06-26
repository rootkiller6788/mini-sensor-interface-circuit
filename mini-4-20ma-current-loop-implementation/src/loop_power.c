/**
 * @file loop_power.c
 * @brief Loop power analysis: compliance, cable modeling, intrinsic safety.
 * Knowledge: L2 (power budget), L4 (KVL compliance), L7 (intrinsic safety).
 */
#include "current_loop.h"
#include "loop_power.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void loop_power_analyze(const current_loop_t *loop, loop_power_analysis_t *ana)
{
    if (!loop || !ana) return;
    memset(ana, 0, sizeof(*ana));
    ana->supply_voltage = loop->supply_voltage;
    ana->total_load_resistance = loop->cable_resistance + loop->shunt_resistance
                               + loop->barrier_resistance;
    double i_a = loop->loop_current_mA / 1000.0;
    ana->voltage_drop_cable = i_a * loop->cable_resistance;
    ana->voltage_drop_shunt = i_a * loop->shunt_resistance;
    ana->voltage_drop_barrier = i_a * loop->barrier_resistance;
    ana->voltage_drop_transmitter = loop->transmitter_min_voltage;
    ana->remaining_compliance = loop->supply_voltage
        - ana->voltage_drop_cable - ana->voltage_drop_shunt
        - ana->voltage_drop_barrier - ana->voltage_drop_transmitter;
    ana->min_required_voltage = loop->transmitter_min_voltage
        + 0.020 * ana->total_load_resistance;
    double v_avail = loop->supply_voltage - loop->transmitter_min_voltage;
    ana->max_possible_current_mA = (ana->total_load_resistance > 0.0)
        ? (v_avail / ana->total_load_resistance) * 1000.0 : 100.0;
    ana->power_available_mW = loop->supply_voltage * i_a * 1000.0;
    ana->power_consumed_mW = i_a * i_a * ana->total_load_resistance * 1000.0;
    if (ana->power_available_mW > 0.0)
        ana->efficiency_percent = (i_a * i_a * loop->shunt_resistance * 1000.0)
                                / ana->power_available_mW * 100.0;
    ana->is_compliant = (ana->remaining_compliance >= 0.0);
}

bool cable_init_by_awg(loop_cable_t *cable, double awg, double len)
{
    if (!cable || awg < 10.0 || awg > 40.0) return false;
    cable->awg = awg;
    cable->length_meters = len;
    double dia_mm = 0.127 * pow(92.0, (36.0 - awg) / 39.0);
    cable->cross_section_mm2 = M_PI * dia_mm * dia_mm / 4.0;
    double rho_copper = 1.68e-8;
    cable->resistance_per_km = rho_copper / (cable->cross_section_mm2 * 1e-6) / 1000.0 * 1000.0;
    cable->total_resistance = 2.0 * cable->resistance_per_km * len / 1000.0;
    return true;
}

double cable_total_resistance(const loop_cable_t *cable)
{
    if (!cable) return 0.0;
    return 2.0 * cable->resistance_per_km * cable->length_meters / 1000.0;
}

double cable_voltage_drop(const loop_cable_t *cable, double current_mA)
{
    if (!cable) return 0.0;
    return (current_mA / 1000.0) * cable_total_resistance(cable);
}

double cable_max_length_for_loop(const current_loop_t *loop,
    double r_per_m, double margin_v)
{
    if (!loop || r_per_m <= 0.0) return 0.0;
    double i_max_a = 0.020;
    double v_avail = loop->supply_voltage - loop->transmitter_min_voltage
        - i_max_a * (loop->shunt_resistance + loop->barrier_resistance) - margin_v;
    if (v_avail <= 0.0) return 0.0;
    return v_avail / (2.0 * i_max_a * r_per_m);
}

double loop_power_efficiency(const current_loop_t *loop)
{
    if (!loop || loop->supply_voltage <= 0.0) return 0.0;
    double i_a = loop->loop_current_mA / 1000.0;
    double p_receiver = i_a * i_a * loop->shunt_resistance;
    double p_supply = loop->supply_voltage * i_a;
    if (p_supply <= 0.0) return 0.0;
    return (p_receiver / p_supply) * 100.0;
}

double loop_power_max_current(const current_loop_t *loop)
{
    if (!loop || loop->total_resistance <= 0.0) return 0.0;
    double v_avail = loop->supply_voltage - loop->transmitter_min_voltage;
    return (v_avail / loop->total_resistance) * 1000.0;
}

bool is_barrier_verify(const is_barrier_t *barrier, double v_supply)
{
    if (!barrier) return false;
    if (v_supply > barrier->zener_voltage) return false;
    double i_sc = v_supply / barrier->series_resistance;
    if (i_sc > barrier->fuse_current) return false;
    if (barrier->max_output_current < 20.0) return false;
    return barrier->is_certified;
}

void is_barrier_init_standard(is_barrier_t *barrier, char gas_group)
{
    if (!barrier) return;
    memset(barrier, 0, sizeof(*barrier));
    barrier->zener_voltage = 28.0;
    barrier->series_resistance = 300.0;
    barrier->max_power_rating = 1.0;
    barrier->fuse_current = 0.05;
    barrier->max_output_voltage = 28.0;
    barrier->max_output_current = 93.0;
    barrier->gas_group = gas_group;
    barrier->is_certified = true;
}

double is_energy_capacitive(double cap_nF, double voltage)
{
    return 0.5 * cap_nF * 1e-9 * voltage * voltage * 1000.0;
}

double is_energy_inductive(double ind_mH, double current_mA)
{
    double i_a = current_mA / 1000.0;
    return 0.5 * ind_mH * 1e-3 * i_a * i_a * 1000.0;
}

double is_minimum_ignition_energy(char gas_group)
{
    switch (gas_group) {
        case 'A': case 'a': return 0.25;
        case 'B': case 'b': return 0.07;
        case 'C': case 'c': return 0.017;
        case 'I': case 'i': return 0.28;  /* Methane */
        default: return 0.0;
    }
}
