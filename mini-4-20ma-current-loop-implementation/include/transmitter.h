/**
 * @file transmitter.h
 * @brief 4-20mA transmitter (TX) implementation API.
 *
 * Covers transmitter design from sensor input to current output,
 * including 2-wire/3-wire/4-wire topologies, DAC driver circuits,
 * and the Howland current pump topology.
 *
 * Reference: Sedra & Smith (2020), Ch.2 (Op-Amps), Ch.9 (Output Stages)
 * Knowledge: L1 transmitter types, L2 compliance, L5 DAC/PWM, L6 transmitter design
 */

#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include "current_loop.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Transmitter configuration and state (L1/L6).
 *
 * Models a complete 4-20mA transmitter including sensor input conditioning,
 * DAC/PWM output stage, and loop power management for 2-wire devices.
 */
typedef struct {
    current_loop_topology_t topology;
    double sensor_input_min;
    double sensor_input_max;
    double sensor_input_current;
    double loop_output_mA;
    double dac_reference_voltage;
    uint8_t dac_bits;
    uint32_t dac_code;
    double pwm_frequency_hz;
    double pwm_duty_cycle;
    double filter_cutoff_hz;
    double min_operating_voltage;
    double quiescent_current_mA;
    double max_output_compliance;
    bool   reverse_polarity_protected;
    bool   overcurrent_protected;
    bool   output_enabled;
} current_loop_transmitter_t;

/**
 * @brief Initialize a transmitter with default parameters (L1).
 *
 * @param tx        Pointer to transmitter structure
 * @param topology  Wiring topology
 */
void transmitter_init(current_loop_transmitter_t *tx,
                      current_loop_topology_t topology);

/**
 * @brief Compute required DAC code for a target loop current (L5).
 *
 * For a typical XTR116/AD421 transmitter IC:
 *   dac_code = (I_target_mA / 16.0) * (2^bits - 1)
 * The internal reference scales 4-20mA to 0-(2^bits-1).
 *
 * @param tx          Transmitter configuration
 * @param target_mA   Desired loop current (mA)
 * @return            DAC code
 */
uint32_t transmitter_compute_dac_code(const current_loop_transmitter_t *tx,
                                       double target_mA);

/**
 * @brief Compute PWM duty cycle for a target loop current using
 *        PWM-to-current conversion (L5).
 *
 * Many low-cost transmitters use a filtered PWM output instead of
 * a precision DAC. The RC-filtered PWM average voltage drives
 * a V-I converter (Howland current pump or XTR11x).
 *
 * duty = target_mA / 20.0   (0-100% for 0-20mA range)
 * With live-zero: duty = 0.20 at 4mA, duty = 1.0 at 20mA
 *
 * @param tx         Transmitter configuration
 * @param target_mA  Target loop current
 * @return           PWM duty cycle (0.0 to 1.0)
 */
double transmitter_compute_pwm_duty(const current_loop_transmitter_t *tx,
                                     double target_mA);

/**
 * @brief Design the PWM reconstruction filter (L5/L6).
 *
 * For a PWM-to-current transmitter, the RC low-pass filter must:
 * 1. Attenuate the PWM carrier frequency sufficiently (>40dB)
 * 2. Have acceptable settling time for step changes
 *
 * Given carrier f_pwm and desired attenuation A_dB:
 *   f_c = f_pwm / 10^(A_dB/20)  (first-order approximation)
 *   f_c = 1 / (2*pi*R*C)
 *
 * This function computes the minimum capacitance for given R.
 *
 * @param pwm_freq_hz    PWM carrier frequency
 * @param attenuation_db Desired attenuation at carrier (typ 40dB)
 * @param r_ohms         Filter resistor value
 * @return               Minimum filter capacitance (farads)
 */
double transmitter_pwm_filter_design(double pwm_freq_hz,
                                      double attenuation_db,
                                      double r_ohms);

/**
 * @brief Compute the maximum load resistance a transmitter can drive (L2).
 *
 * R_load_max = (V_supply - V_transmitter_min) / I_max
 *
 * This is the compliance limit: the transmitter can only push 20mA
 * through a load up to R_load_max before saturating.
 *
 * @param tx              Transmitter configuration
 * @param supply_voltage  Available supply voltage
 * @return                Maximum load resistance (ohm)
 */
double transmitter_max_load_resistance(const current_loop_transmitter_t *tx,
                                        double supply_voltage);

/**
 * @brief Compute minimum supply voltage for a given load (L2).
 *
 * V_supply_min = V_tx_min + I_max * R_load
 *
 * @param tx          Transmitter configuration
 * @param load_ohms   Total load resistance
 * @return            Minimum supply voltage
 */
double transmitter_min_supply_for_load(const current_loop_transmitter_t *tx,
                                        double load_ohms);

/**
 * @brief Compute the Howland current pump resistor values (L6).
 *
 * The Howland current pump is a classic op-amp circuit that converts
 * a voltage input to a precision current output. For 4-20mA:
 *
 * R1 = R2 = R3 = R4 for unity gain, or:
 * I_out = V_in * (1/R_sense) * (R3/R2)   when R1/R2 = R3/R4
 *
 * Constraint for proper operation:
 *   R1 * R4 = R2 * R3  (balanced bridge condition)
 *
 * This function verifies the Howland balance condition.
 *
 * @param r1, r2, r3, r4  Bridge resistor values (ohm)
 * @param r_sense          Sense resistor (ohm)
 * @param v_in             Input voltage
 * @return                 Expected output current (A)
 */
double transmitter_howland_current(double r1, double r2, double r3, double r4,
                                    double r_sense, double v_in);

/**
 * @brief Verify Howland pump balance condition (L6).
 *
 * Balanced when R1 * R4 == R2 * R3 (within tolerance).
 * Output impedance of a balanced Howland pump approaches infinity
 * (ideally), making it an excellent current source.
 *
 * @param r1, r2, r3, r4  Bridge resistor values (ohm)
 * @param tolerance        Relative tolerance (e.g. 0.01 for 1%)
 * @return                 true if balanced
 */
bool transmitter_howland_is_balanced(double r1, double r2,
                                      double r3, double r4,
                                      double tolerance);

/**
 * @brief Compute the 2-wire transmitter power budget (L2).
 *
 * In a 2-wire (loop-powered) transmitter, all operating power comes from
 * the loop current itself. At 4mA, the minimum available power is:
 *   P_min = (V_supply - V_transmitter_min) * 0.004
 *
 * This severely constrains the electronics design. Ultra-low-power
 * op-amps, MCUs, and ADCs are required.
 *
 * @param v_supply           Supply voltage
 * @param v_transmitter_min  Minimum transmitter operating voltage
 * @param i_quiescent        Quiescent current consumption (A)
 * @return                   Available power at quiescent current (W)
 */
double transmitter_two_wire_power_budget(double v_supply,
                                          double v_transmitter_min,
                                          double i_quiescent);

/**
 * @brief Configure a transmitter for range scaling (L6).
 *
 * Zero and span adjustment: two trim potentiometers (or digital equivalents)
 * set the 4mA point (zero) and 20mA point (span).
 *
 * @param tx            Transmitter to configure
 * @param zero_offset   Zero adjustment offset (mA)
 * @param span_gain     Span adjustment gain
 */
void transmitter_configure_range(current_loop_transmitter_t *tx,
                                  double zero_offset, double span_gain);

/**
 * @brief Compute the transmitter output error (L3).
 *
 * Error sources:
 * - DAC quantization: +/- 0.5 LSB
 * - Reference drift: ppm/degC * delta_T
 * - Output stage nonlinearity
 * - Thermal EMF in connectors
 *
 * @param ideal_mA     Ideal output current
 * @param actual_mA    Measured output current
 * @return             Error as percentage of span
 */
double transmitter_output_error_percent(double ideal_mA, double actual_mA);

#ifdef __cplusplus
}
#endif

#endif /* TRANSMITTER_H */