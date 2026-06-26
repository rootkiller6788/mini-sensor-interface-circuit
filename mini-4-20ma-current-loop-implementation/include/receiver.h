/**
 * @file receiver.h
 * @brief 4-20mA receiver (RX) / PLC analog input API.
 *
 * Covers shunt resistor selection, ADC interfacing, signal conditioning,
 * filtering, and PLC/SCADA integration for 4-20mA loop receivers.
 *
 * Reference: Kester (2004), Analog-Digital Conversion, Analog Devices
 * Knowledge: L1 receiver, L2 shunt selection, L3/L5 ADC/Filtering, L7 PLC
 */

#ifndef RECEIVER_H
#define RECEIVER_H

#include "current_loop.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Receiver / PLC analog input configuration (L1/L2).
 *
 * Models the complete receiver chain: shunt resistor -> signal conditioning
 * -> ADC -> digital filtering -> engineering units conversion.
 */
typedef struct {
    double shunt_resistance;
    double shunt_power_rating_w;
    double adc_reference_voltage;
    uint8_t adc_bits;
    uint32_t adc_raw_value;
    double adc_voltage_input;
    double measured_current_mA;
    double filtered_current_mA;
    double iir_alpha;
    size_t moving_avg_window;
    double engineering_value;
    double engineering_min;
    double engineering_max;
    bool   filter_enabled;
    bool   burnout_detection;
    double burnout_threshold_mA;
} current_loop_receiver_t;

/**
 * @brief Initialize a receiver with standard 250-ohm shunt (L1).
 *
 * Standard configuration: 250 ohm shunt, 5V reference, 12-bit ADC.
 *
 * @param rx Pointer to receiver structure
 */
void receiver_init_standard(current_loop_receiver_t *rx);

/**
 * @brief Compute optimal shunt resistance for a given voltage range (L2).
 *
 * The shunt resistor converts loop current to voltage for ADC measurement.
 * Design trade-offs:
 * - Higher R_shunt -> larger signal swing, better SNR
 * - Higher R_shunt -> more loop voltage drop, less compliance
 * - Standard values: 250 ohm (1-5V), 100 ohm (0.4-2V), 500 ohm (2-10V)
 *
 * R_optimal = V_adc_max / I_loop_max = V_adc_max / 0.020
 *
 * Power rating requirement: P >= I_max^2 * R = 0.008 * R (at 20mA)
 *
 * @param adc_v_max  Maximum ADC input voltage (V)
 * @return           Optimal shunt resistance (ohm)
 */
double receiver_optimal_shunt(double adc_v_max);

/**
 * @brief Compute shunt power dissipation at max current (L2).
 *
 * P = (0.020)^2 * R_shunt = 0.0004 * R_shunt
 * For 250 ohm: P = 0.1W -> 1/4W resistor is adequate (derated to 50%)
 * For 500 ohm: P = 0.2W -> use 1/2W resistor
 *
 * @param r_shunt Shunt resistance (ohm)
 * @return        Power dissipation at 20mA (W)
 */
double receiver_shunt_power_at_max(double r_shunt);

/**
 * @brief Convert ADC reading to loop current (L5).
 *
 * For standard 250 ohm shunt with 5V ADC reference:
 *   V_adc = adc_code * V_ref / (2^bits - 1)
 *   I_loop = V_adc / R_shunt  (in A)
 *   I_loop_mA = I_loop * 1000
 *
 * Combined: I_loop_mA = (adc_code * V_ref * 1000) / ( (2^bits - 1) * R_shunt )
 *
 * @param rx       Receiver configuration
 * @param adc_code Raw ADC reading
 * @return         Loop current in mA
 */
double receiver_adc_to_current(const current_loop_receiver_t *rx,
                                uint32_t adc_code);

/**
 * @brief Compute ADC resolution in mA per LSB (L5).
 *
 * resolution_mA = (V_ref / R_shunt * 1000) / (2^bits - 1)
 *
 * Examples:
 *   - 12-bit, 250 ohm, 5V: 20mA / 4095 = 4.88 uA/LSB
 *   - 16-bit, 250 ohm, 5V: 20mA / 65535 = 0.305 uA/LSB
 *   - 24-bit, 250 ohm, 5V: 20mA / 16777215 = 1.19 nA/LSB (noise-limited)
 *
 * @param rx Receiver configuration
 * @return   Resolution in mA/LSB
 */
double receiver_adc_resolution(const current_loop_receiver_t *rx);

/**
 * @brief Detect burnout (open loop) condition from ADC readings (L6).
 *
 * Burnout detection: if loop current drops below a threshold
 * (typically < 3.6mA per NAMUR NE43), flag as open circuit.
 *
 * Uses a voting scheme: N consecutive readings below threshold
 * are required to avoid false trips on transient noise.
 *
 * @param rx            Receiver configuration
 * @param recent_readings Array of recent current readings (mA)
 * @param n_readings    Number of readings
 * @param votes_needed  Consecutive votes needed (typically 3-5)
 * @return              true if burnout detected
 */
bool receiver_detect_burnout(const current_loop_receiver_t *rx,
                              const double *recent_readings,
                              size_t n_readings,
                              size_t votes_needed);

/**
 * @brief Apply input oversampling and averaging for resolution enhancement (L5).
 *
 * By sampling at N times the Nyquist rate and averaging, the effective
 * resolution improves by 0.5 * log2(N) bits (for white noise).
 *
 * ENOB_effective = ENOB_native + 0.5 * log2(oversample_ratio)
 *
 * @param rx               Receiver configuration
 * @param oversample_ratio Number of samples to average
 * @return                 Effective resolution improvement (additional bits)
 */
double receiver_oversampling_improvement(const current_loop_receiver_t *rx,
                                          size_t oversample_ratio);

/**
 * @brief Convert loop current to engineering units (L7).
 *
 * EU = EU_min + (I_meas - 4.0) / 16.0 * (EU_max - EU_min)
 *
 * Example applications:
 * - 4-20mA pressure transmitter, 0-100 psi: 12mA -> 50 psi
 * - 4-20mA temperature transmitter, -50 to 150 degC: 12mA -> 50 degC
 * - 4-20mA level transmitter, 0-10 m: 12mA -> 5 m
 *
 * @param rx          Receiver configuration (EU range fields)
 * @param current_mA  Measured loop current (mA)
 * @return            Process value in engineering units
 */
double receiver_current_to_engineering(const current_loop_receiver_t *rx,
                                        double current_mA);

/**
 * @brief Apply a second-order Butterworth low-pass filter (L5).
 *
 * Second-order IIR filter for improved noise rejection vs. first-order.
 * Difference equation (Direct Form I):
 *   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *
 * @param x_n      Current input sample
 * @param x_n1     Input sample delayed by 1
 * @param x_n2     Input sample delayed by 2 (updated in-place)
 * @param y_n1     Output delayed by 1 (updated in-place)
 * @param y_n2     Output delayed by 2 (updated in-place)
 * @param b0,b1,b2 Feed-forward coefficients
 * @param a1,a2    Feed-back coefficients
 * @return         Filtered output y[n]
 */
double receiver_butterworth_lp2(double x_n, double *x_n1, double *x_n2,
                                 double *y_n1, double *y_n2,
                                 double b0, double b1, double b2,
                                 double a1, double a2);

/**
 * @brief Compute Butterworth LPF coefficients for given specs (L5).
 *
 * Designs a 2nd-order Butterworth low-pass filter using bilinear transform.
 * The filter has maximally flat passband and -3dB at f_cutoff.
 *
 * @param f_cutoff_hz Cutoff frequency (Hz)
 * @param f_sample_hz Sampling frequency (Hz)
 * @param b0,b1,b2    Output feed-forward coefficients
 * @param a1,a2       Output feed-back coefficients
 */
void receiver_butterworth_design(double f_cutoff_hz, double f_sample_hz,
                                  double *b0, double *b1, double *b2,
                                  double *a1, double *a2);

/**
 * @brief Implement sample-and-hold model for ADC aperture error (L6).
 *
 * For a sinusoidal input at frequency f, the maximum aperture error is:
 *   dV_max = 2*pi*f * V_peak * t_aperture
 *
 * @param signal_freq_hz  Input signal frequency (Hz)
 * @param signal_amplitude_v Signal amplitude (V)
 * @param aperture_time_s ADC aperture time (s)
 * @return                Maximum aperture error voltage (V)
 */
double receiver_aperture_error(double signal_freq_hz,
                                double signal_amplitude_v,
                                double aperture_time_s);

#ifdef __cplusplus
}
#endif

#endif /* RECEIVER_H */