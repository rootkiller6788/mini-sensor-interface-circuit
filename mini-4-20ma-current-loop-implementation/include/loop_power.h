/**
 * @file loop_power.h
 * @brief Loop power management, compliance analysis, and intrinsic safety.
 *
 * Power is the fundamental constraint in 4-20mA loop design, especially
 * for 2-wire (loop-powered) transmitters. This module covers power budget
 * analysis, voltage compliance, cable loss, and intrinsic safety barriers.
 *
 * Reference: IEC 60079-11 (Intrinsic Safety), ISA-50.1
 * Knowledge: L2 loop power, L4 Kirchhoff compliance, L7 intrinsic safety
 */

#ifndef LOOP_POWER_H
#define LOOP_POWER_H

#include "current_loop.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Detailed loop power analysis result (L2).
 */
typedef struct {
    double supply_voltage;
    double min_required_voltage;
    double total_load_resistance;
    double voltage_drop_cable;
    double voltage_drop_shunt;
    double voltage_drop_barrier;
    double voltage_drop_transmitter;
    double remaining_compliance;
    double max_possible_current_mA;
    double power_available_mW;
    double power_consumed_mW;
    double efficiency_percent;
    bool   is_compliant;
} loop_power_analysis_t;

/**
 * @brief Cable parameters for loop wiring (L2).
 *
 * Wire gauge and length determine cable resistance which directly
 * impacts loop compliance. Common industrial wiring:
 *   AWG 18: 20.9 ohm/km per conductor
 *   AWG 20: 33.3 ohm/km per conductor
 *   AWG 22: 53.0 ohm/km per conductor
 *   AWG 24: 84.2 ohm/km per conductor
 */
typedef struct {
    double awg;
    double cross_section_mm2;
    double resistance_per_km;
    double length_meters;
    double total_resistance;
} loop_cable_t;

/**
 * @brief Intrinsic safety barrier parameters (L7).
 *
 * Zener barrier or galvanic isolator parameters per IEC 60079-11.
 */
typedef struct {
    double zener_voltage;
    double series_resistance;
    double max_power_rating;
    double fuse_current;
    double max_output_voltage;
    double max_output_current;
    char   gas_group;
    bool   is_certified;
} is_barrier_t;

/**
 * @brief Perform complete loop power analysis (L2/L4).
 *
 * Computes all voltage drops around the loop, determines compliance,
 * and calculates maximum possible loop current.
 *
 * @param loop             Populated loop descriptor
 * @param analysis         Output analysis structure
 */
void loop_power_analyze(const current_loop_t *loop,
                         loop_power_analysis_t *analysis);

/**
 * @brief Initialize cable parameters by AWG (L2).
 *
 * Sets resistance_per_km based on standard copper wire tables.
 *
 * @param cable         Cable structure to initialize
 * @param awg           American Wire Gauge (18-30)
 * @param length_meters Cable length in meters
 * @return              true if valid AWG
 */
bool cable_init_by_awg(loop_cable_t *cable, double awg, double length_meters);

/**
 * @brief Compute total cable resistance from length and gauge (L2).
 *
 * R_total = 2 * length * resistance_per_km / 1000
 * Factor 2 for round-trip (outgoing + return).
 *
 * @param cable Cable parameters
 * @return      Total loop cable resistance (ohm)
 */
double cable_total_resistance(const loop_cable_t *cable);

/**
 * @brief Compute voltage drop in cable at given loop current (L2).
 *
 * V_drop = I_loop * R_cable_total
 *
 * @param cable       Cable parameters
 * @param current_mA  Loop current (mA)
 * @return            Voltage drop (V)
 */
double cable_voltage_drop(const loop_cable_t *cable, double current_mA);

/**
 * @brief Compute maximum cable length for a given loop budget (L2/L4).
 *
 * L_max = (V_compliance - V_margin) / (2 * I_max * r_per_meter)
 *
 * Where V_compliance = V_supply - V_transmitter_min - I_max * R_shunt
 * and r_per_meter is the resistance per meter per conductor.
 *
 * @param loop                    Loop descriptor
 * @param r_per_meter_per_conductor Resistance per meter per conductor (ohm/m)
 * @param margin_v                Desired voltage margin at max current (V)
 * @return                        Max cable length (m)
 */
double cable_max_length_for_loop(const current_loop_t *loop,
                                  double r_per_meter_per_conductor,
                                  double margin_v);

/**
 * @brief Compute loop efficiency (L2).
 *
 * eta = P_receiver / P_supply * 100%
 * P_receiver = I^2 * R_shunt (useful signal power at receiver)
 * P_supply = V_supply * I_loop (total power from supply)
 *
 * Typical efficiency: 10-30%. The rest is lost in cable, transmitter
 * voltage drop, and barrier.
 *
 * @param loop Loop descriptor
 * @return     Efficiency percentage
 */
double loop_power_efficiency(const current_loop_t *loop);

/**
 * @brief Compute maximum available loop current given compliance (L4).
 *
 * I_max = (V_supply - V_transmitter_min) / R_total
 *
 * If I_max < 20mA, the loop cannot reach full scale.
 *
 * @param loop Loop descriptor
 * @return     Maximum possible loop current (mA)
 */
double loop_power_max_current(const current_loop_t *loop);

/**
 * @brief Verify that a Zener barrier satisfies IS requirements (L7).
 *
 * Per IEC 60079-11, the barrier must limit voltage and current
 * below the ignition limits for the target gas group.
 *
 * @param barrier    IS barrier parameters
 * @param v_supply   Supply voltage
 * @return           true if barrier provides adequate protection
 */
bool is_barrier_verify(const is_barrier_t *barrier, double v_supply);

/**
 * @brief Initialize a standard IS Zener barrier (L7).
 *
 * Typical values: 28V Zener, 300 ohm series, 50mA fuse,
 * suitable for Gas Group IIC (hydrogen).
 *
 * @param barrier    Barrier structure to initialize
 * @param gas_group  Target gas group
 */
void is_barrier_init_standard(is_barrier_t *barrier, char gas_group);

/**
 * @brief Compute energy stored in connected capacitance (L7).
 *
 * E_c = 0.5 * C * V^2
 * Must not exceed the Minimum Ignition Energy (MIE) of the gas.
 * MIE values: Hydrogen 0.017 mJ, Ethylene 0.07 mJ, Propane 0.25 mJ
 *
 * @param capacitance_nF Total capacitance (nF)
 * @param voltage        Applied voltage (V)
 * @return               Stored energy (mJ)
 */
double is_energy_capacitive(double capacitance_nF, double voltage);

/**
 * @brief Compute energy stored in connected inductance (L7).
 *
 * E_l = 0.5 * L * I^2
 *
 * @param inductance_mH Total inductance (mH)
 * @param current_mA    Loop current (mA)
 * @return              Stored energy (mJ)
 */
double is_energy_inductive(double inductance_mH, double current_mA);

/**
 * @brief Compute minimum ignition energy for a gas group (L7).
 *
 * Reference values per IEC 60079-11:
 *   I (methane):   0.28 mJ
 *   IIA (propane): 0.25 mJ
 *   IIB (ethylene):0.07 mJ
 *   IIC (hydrogen):0.017 mJ
 *
 * @param gas_group Gas group character
 * @return          MIE in mJ (0 if unknown)
 */
double is_minimum_ignition_energy(char gas_group);

#ifdef __cplusplus
}
#endif

#endif /* LOOP_POWER_H */