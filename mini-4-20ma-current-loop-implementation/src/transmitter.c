/**
 * @file transmitter.c
 * @brief 4-20mA transmitter implementation: DAC/PWM driver, Howland pump,
 *        range configuration, power budget for 2-wire/3-wire/4-wire.
 * Knowledge: L1 (TX types), L2 (compliance), L5 (DAC/PWM), L6 (TX design).
 */
#include "current_loop.h"
#include "transmitter.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void transmitter_init(current_loop_transmitter_t *tx, current_loop_topology_t topo)
{
    if (!tx) return;
    memset(tx, 0, sizeof(*tx));
    tx->topology = topo;
    tx->sensor_input_min = 0.0;
    tx->sensor_input_max = 100.0;
    tx->dac_reference_voltage = 5.0;
    tx->dac_bits = 16;
    tx->pwm_frequency_hz = 10000.0;
    tx->filter_cutoff_hz = 10.0;
    tx->min_operating_voltage = 8.0;
    tx->quiescent_current_mA = 3.5;
    tx->max_output_compliance = 40.0;
    tx->reverse_polarity_protected = true;
    tx->overcurrent_protected = true;
    tx->output_enabled = false;
}

uint32_t transmitter_compute_dac_code(const current_loop_transmitter_t *tx, double target_mA)
{
    if (!tx || tx->dac_bits == 0) return 0;
    if (target_mA < 4.0) target_mA = 4.0;
    if (target_mA > 20.0) target_mA = 20.0;
    uint32_t max_code = (1u << tx->dac_bits) - 1;
    double fraction = (target_mA - 4.0) / 16.0;
    return (uint32_t)(fraction * max_code + 0.5);
}

double transmitter_compute_pwm_duty(const current_loop_transmitter_t *tx, double target_mA)
{
    if (!tx) return 0.0;
    if (target_mA < 0.0) target_mA = 0.0;
    if (target_mA > 20.0) target_mA = 20.0;
    return target_mA / 20.0;
}

double transmitter_pwm_filter_design(double pwm_freq, double atten_db, double r_ohms)
{
    if (pwm_freq <= 0.0 || atten_db <= 0.0 || r_ohms <= 0.0) return 0.0;
    double fc = pwm_freq / pow(10.0, atten_db / 20.0);
    return 1.0 / (2.0 * M_PI * r_ohms * fc);
}

double transmitter_max_load_resistance(const current_loop_transmitter_t *tx, double v_supply)
{
    if (!tx) return 0.0;
    double v_avail = v_supply - tx->min_operating_voltage;
    if (v_avail <= 0.0) return 0.0;
    return v_avail / 0.020;
}

double transmitter_min_supply_for_load(const current_loop_transmitter_t *tx, double load_ohms)
{
    if (!tx) return 0.0;
    return tx->min_operating_voltage + 0.020 * load_ohms;
}

double transmitter_howland_current(double r1, double r2, double r3, double r4,
                                    double r_sense, double v_in)
{
    (void)r1; (void)r4;  /* r1,r4 used in balance condition; r2,r3 set gain */
    if (r_sense <= 0.0 || r2 <= 0.0) return 0.0;
    double gain = r3 / r2;
    return (v_in * gain) / r_sense;
}

bool transmitter_howland_is_balanced(double r1, double r2, double r3, double r4,
                                      double tolerance)
{
    double left = r1 * r4;
    double right = r2 * r3;
    if (right <= 0.0) return false;
    double error = fabs((left - right) / right);
    return error <= tolerance;
}

double transmitter_two_wire_power_budget(double v_supply, double v_tx_min, double i_quiescent)
{
    double v_avail = v_supply - v_tx_min;
    if (v_avail < 0.0) return 0.0;
    return v_avail * i_quiescent;
}

void transmitter_configure_range(current_loop_transmitter_t *tx,
                                  double zero_offset, double span_gain)
{
    if (!tx) return;
    tx->sensor_input_min += zero_offset;
    tx->sensor_input_max = tx->sensor_input_min
                         + (tx->sensor_input_max - tx->sensor_input_min) * span_gain;
}

double transmitter_output_error_percent(double ideal_mA, double actual_mA)
{
    if (ideal_mA == 0.0) return 0.0;
    return fabs((actual_mA - ideal_mA) / 16.0) * 100.0;
}
