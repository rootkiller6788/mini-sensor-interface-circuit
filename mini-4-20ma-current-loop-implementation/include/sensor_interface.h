/**
 * @file sensor_interface.h
 * @brief Sensor-to-4-20mA interface circuits.
 *
 * Covers the complete signal chain from physical sensor element
 * (thermocouple, RTD, strain gauge bridge, etc.) to conditioned
 * voltage input for the 4-20mA transmitter.
 *
 * Reference: Kester (1999), Practical Design Techniques for Sensor
 *            Signal Conditioning, Analog Devices
 * Knowledge: L1 sensor types, L2 signal conditioning, L5 linearization
 */

#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include "current_loop.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L1: SENSOR TYPE DEFINITIONS
 *===========================================================================*/

/**
 * @brief Supported sensor types for 4-20mA transmitters.
 */
typedef enum {
    SENSOR_TYPE_RTD_PT100       = 0,
    SENSOR_TYPE_RTD_PT1000      = 1,
    SENSOR_TYPE_THERMOCOUPLE_K  = 2,
    SENSOR_TYPE_THERMOCOUPLE_J  = 3,
    SENSOR_TYPE_THERMOCOUPLE_T  = 4,
    SENSOR_TYPE_STRAIN_GAUGE    = 5,
    SENSOR_TYPE_LOAD_CELL       = 6,
    SENSOR_TYPE_PRESSURE_BRIDGE = 7,
    SENSOR_TYPE_VOLTAGE_0_5V    = 8,
    SENSOR_TYPE_VOLTAGE_0_10V   = 9,
    SENSOR_TYPE_CURRENT_4_20mA  = 10,
    SENSOR_TYPE_POTENTIOMETER   = 11,
    SENSOR_TYPE_PH_PROBE        = 12,
    SENSOR_TYPE_CONDUCTIVITY    = 13,
    SENSOR_TYPE_DISSOLVED_O2    = 14,
} sensor_type_t;

/**
 * @brief Sensor excitation configuration (L2).
 *
 * Many sensors require excitation: RTDs need constant current,
 * strain gauges need bridge voltage, pH probes need ultra-high
 * impedance buffer.
 */
typedef struct {
    double excitation_voltage;
    double excitation_current_mA;
    bool   excitation_enabled;
    bool   ratiometric;
} sensor_excitation_t;

/**
 * @brief Sensor conditioning configuration (L1/L2).
 */
typedef struct {
    sensor_type_t type;
    double min_input;
    double max_input;
    double min_output;
    double max_output;
    double gain;
    double offset;
    double filter_cutoff_hz;
    bool   use_cjc;
    double cjc_temperature_c;
    sensor_excitation_t excitation;
} sensor_config_t;

/*===========================================================================
 * SENSOR TRANSFER FUNCTIONS (L2/L3)
 *===========================================================================*/

/**
 * @brief PT100 RTD resistance to temperature (IEC 60751) (L3).
 *
 * Callendar-Van Dusen equation:
 *   For T >= 0 degC: R(T) = R0*(1 + A*T + B*T^2)
 *   For T <  0 degC: R(T) = R0*(1 + A*T + B*T^2 + C*(T-100)*T^3)
 *
 * Where R0 = 100 ohm (PT100) or 1000 ohm (PT1000)
 * A = 3.9083e-3, B = -5.775e-7, C = -4.183e-12
 *
 * @param temperature_c Temperature in degC
 * @param r0            Resistance at 0 degC (100 or 1000)
 * @return              RTD resistance (ohm)
 */
double sensor_rtd_resistance(double temperature_c, double r0);

/**
 * @brief PT100 resistance to temperature via inverse C-V-D equation (L5).
 *
 * Iterative Newton-Raphson inversion of the Callendar-Van Dusen equation.
 * For PT100: T_n+1 = T_n - (R(T_n) - R_measured) / (dR/dT(T_n))
 *
 * @param resistance_ohm Measured RTD resistance
 * @param r0             R0 value (100 or 1000)
 * @return               Temperature in degC
 */
double sensor_rtd_to_temperature(double resistance_ohm, double r0);

/**
 * @brief Type K thermocouple voltage to temperature (L3).
 *
 * ITS-90 polynomial for Type K (NiCr-NiAl):
 *   T = c0 + c1*V + c2*V^2 + c3*V^3 + ... + c9*V^9
 *
 * Range: -200 to 1372 degC, output: -5.891 to 54.886 mV
 *
 * @param voltage_mv Thermocouple EMF (mV), cold-junction compensated
 * @return           Temperature in degC
 */
double sensor_tc_k_voltage_to_temp(double voltage_mv);

/**
 * @brief Type K thermocouple temperature to voltage (L3).
 *
 * Inverse ITS-90 polynomial for Type K.
 *
 * @param temperature_c Temperature in degC
 * @return              EMF in mV
 */
double sensor_tc_k_temp_to_voltage(double temperature_c);

/**
 * @brief Cold junction compensation for thermocouples (L2/L6).
 *
 * The thermocouple measures temperature DIFFERENCE between hot and
 * cold junctions. Cold junction temperature must be known (measured
 * by a thermistor, RTD, or IC sensor at the terminal block).
 *
 * V_compensated = V_measured + V_CJC(T_cold)
 * T_hot = sensor_tc_k_voltage_to_temp(V_compensated)
 *
 * @param voltage_measured_mv Measured TC voltage (mV)
 * @param cold_junction_temp_c Cold junction temperature (degC)
 * @param tc_type             Thermocouple type
 * @return                    Hot junction temperature (degC)
 */
double sensor_cjc_compensate(double voltage_measured_mv,
                              double cold_junction_temp_c,
                              sensor_type_t tc_type);

/**
 * @brief Strain gauge bridge output to microstrain (L3).
 *
 * For a quarter-bridge configuration:
 *   epsilon = -4 * V_out / (GF * V_excitation)
 *
 * For a half-bridge:
 *   epsilon = -2 * V_out / (GF * V_excitation)
 *
 * For a full-bridge:
 *   epsilon = -V_out / (GF * V_excitation)
 *
 * Where GF = gauge factor (typically 2.0 for metal foil).
 *
 * @param v_out           Bridge differential output voltage (V)
 * @param v_excitation    Bridge excitation voltage (V)
 * @param gauge_factor    Gauge factor (unitless, typically 2.0)
 * @param bridge_config   1=quarter, 2=half, 4=full
 * @return                Microstrain (ue)
 */
double sensor_strain_to_microstrain(double v_out, double v_excitation,
                                     double gauge_factor, int bridge_config);

/**
 * @brief 3-wire RTD lead wire compensation (L6).
 *
 * In 3-wire configuration, two matched current sources eliminate
 * lead resistance errors:
 *   R_rtd = (V_lead1 - V_lead2) / I_excitation
 *
 * Error without compensation: delta_R = 2 * R_lead
 * Error with 3-wire: delta_R ~= (R_lead1 - R_lead2) << R_lead
 *
 * @param v_measured1     Voltage at first sense lead (V)
 * @param v_measured2     Voltage at second sense lead (V)
 * @param i_excitation    Excitation current (A)
 * @param lead_r_est      Estimated lead resistance for error calc (ohm)
 * @param error_out       Output: residual lead error (ohm)
 * @return                Compensated RTD resistance (ohm)
 */
double sensor_3wire_rtd_compensate(double v_measured1, double v_measured2,
                                    double i_excitation, double lead_r_est,
                                    double *error_out);

/**
 * @brief Compute instrumentation amplifier gain for sensor scaling (L2/L6).
 *
 * Given sensor output range and desired ADC input range:
 *   gain = V_adc_span / V_sensor_span
 *
 * Common 3-op-amp instrumentation amplifier:
 *   gain = 1 + 2*R1/R_gain   (where R_gain sets the gain)
 *
 * @param v_sensor_min     Minimum sensor output (V)
 * @param v_sensor_max     Maximum sensor output (V)
 * @param v_adc_min        Minimum ADC input (V)
 * @param v_adc_max        Maximum ADC input (V)
 * @return                 Required amplifier gain
 */
double sensor_compute_inst_amp_gain(double v_sensor_min, double v_sensor_max,
                                     double v_adc_min, double v_adc_max);

/**
 * @brief Compute excitation voltage for ratiometric measurement (L2).
 *
 * In ratiometric systems, the ADC reference is derived from the same
 * source as the sensor excitation, canceling excitation drift.
 *
 * Ratiometric error cancellation factor:
 *   error_cancelled = 1 - (V_ref_drift / V_excitation_drift)
 *
 * @param excitation_drift_percent Excitation voltage drift (%)
 * @param reference_drift_percent  Reference voltage drift (%)
 * @return                         Residual error after ratiometric cancellation (%)
 */
double sensor_ratiometric_error(double excitation_drift_percent,
                                 double reference_drift_percent);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_INTERFACE_H */