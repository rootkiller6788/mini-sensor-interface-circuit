/**
 * @file hart_protocol.h
 * @brief HART (Highway Addressable Remote Transducer) Protocol Implementation.
 *
 * HART is a hybrid analog+digital protocol that superimposes FSK
 * (Frequency Shift Keying) digital communication on the 4-20mA
 * analog signal without disturbing the analog measurement.
 *
 * Physical Layer: Bell 202 FSK at 1200 bps
 *   - 1200 Hz = logical "1" (mark)
 *   - 2200 Hz = logical "0" (space)
 *   - Amplitude: +/- 0.5 mA peak, average = 0 (no DC shift)
 *
 * Reference: HCF SPEC-99, HCF SPEC-127 (HART 7)
 * Knowledge: L8 HART protocol, L5 FSK modulation, L6 industrial protocol
 */

#ifndef HART_PROTOCOL_H
#define HART_PROTOCOL_H

#include "current_loop.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief HART device variable descriptor (L8).
 *
 * Each HART device can report up to 4 dynamic variables (PV, SV, TV, QV)
 * plus the loop current. Variables have units codes per HART specification.
 */
typedef struct {
    float primary_variable;
    float secondary_variable;
    float tertiary_variable;
    float quaternary_variable;
    float loop_current_mA;
    uint8_t pv_unit_code;
    uint8_t sv_unit_code;
    uint8_t tv_unit_code;
    uint8_t qv_unit_code;
} hart_device_variables_t;

/**
 * @brief HART device identity information (L8).
 *
 * Response to Command 0 (Read Unique Identifier).
 */
typedef struct {
    uint8_t manufacturer_id;
    uint8_t device_type;
    uint8_t device_id[3];
    uint8_t hart_revision;
    uint8_t device_revision;
    uint8_t software_revision;
    uint8_t preambles_required;
    uint8_t flags;
} hart_device_info_t;

/**
 * @brief HART modem state machine states (L8).
 */
typedef enum {
    HART_STATE_IDLE,
    HART_STATE_PREAMBLE,
    HART_STATE_DELIMITER,
    HART_STATE_ADDRESS,
    HART_STATE_COMMAND,
    HART_STATE_BYTE_COUNT,
    HART_STATE_DATA,
    HART_STATE_CHECKSUM,
    HART_STATE_FRAME_COMPLETE,
    HART_STATE_ERROR
} hart_state_t;

/**
 * @brief FSK modulator parameters for HART transmission (L8).
 *
 * The modulator generates a sine wave that is superimposed on the
 * DC loop current. The AC component must have zero mean to avoid
 * disturbing the analog signal.
 */
typedef struct {
    double mark_frequency_hz;
    double space_frequency_hz;
    double amplitude_mA;
    double sample_rate_hz;
    double phase_radians;
} hart_modulator_t;

/**
 * @brief Initialize HART modulator with standard Bell 202 parameters (L8).
 */
void hart_modulator_init(hart_modulator_t *mod);

/**
 * @brief Generate the next FSK sample for transmission (L5/L8).
 *
 * Continuous-phase FSK modulation:
 *   s(t) = A * sin(2*pi*f(t)*t + phi)
 * Where f(t) = f_mark for bit=1, f_space for bit=0
 *
 * Phase continuity is maintained across bit transitions to minimize
 * spectral splatter.
 *
 * @param mod      Modulator state (phase updated in-place)
 * @param bit      1 for mark (1200 Hz), 0 for space (2200 Hz)
 * @param t_sample Current sample time (s)
 * @return         Modulated amplitude (mA)
 */
double hart_modulator_sample(hart_modulator_t *mod, uint8_t bit,
                              double t_sample);

/**
 * @brief FSK demodulator — detect bit from bandpass filter outputs (L5/L8).
 *
 * Uses two bandpass filters centered at mark and space frequencies.
 * The bit decision is based on which filter has greater energy.
 *
 * @param mark_energy   Energy in mark (1200 Hz) filter output
 * @param space_energy  Energy in space (2200 Hz) filter output
 * @return              1 if mark energy > space energy, 0 otherwise
 */
uint8_t hart_demodulate_bit(double mark_energy, double space_energy);

/**
 * @brief Build a HART Command 0 (Read Unique Identifier) request frame (L8).
 *
 * This is typically the first command sent by a master to discover
 * connected devices in multi-drop mode.
 *
 * @param frame   Output frame to populate
 * @param address Polling address (0 for short frame)
 */
void hart_build_command_0(hart_frame_t *frame, uint8_t address);

/**
 * @brief Build a HART Command 3 (Read Dynamic Variables) request frame (L8).
 *
 * Returns PV, loop current, and optionally SV, TV, QV.
 *
 * @param frame   Output frame to populate
 * @param address Device address
 */
void hart_build_command_3(hart_frame_t *frame, uint8_t address);

/**
 * @brief Parse a HART Command 3 response into device variables (L8).
 *
 * @param frame Received response frame (must be validated first)
 * @param vars  Output: parsed device variables
 * @return      true if parsing succeeded
 */
bool hart_parse_command_3_response(const hart_frame_t *frame,
                                    hart_device_variables_t *vars);

/**
 * @brief Parse a HART Command 0 response into device info (L8).
 *
 * @param frame Received response frame
 * @param info  Output: parsed device information
 * @return      true if parsing succeeded
 */
bool hart_parse_command_0_response(const hart_frame_t *frame,
                                    hart_device_info_t *info);

/**
 * @brief Compute HART frame preamble count for a given device (L8).
 *
 * The number of preamble bytes required depends on the device's
 * wake-up time. Most devices need 5-20 preamble bytes.
 *
 * @param preambles_required Per device info
 * @return                   Recommended preamble count
 */
uint8_t hart_recommended_preambles(uint8_t preambles_required);

/**
 * @brief Detect HART signal presence on a 4-20mA loop (L8).
 *
 * HART signal has zero DC component but introduces ~1 mA p-p AC ripple.
 * Detection: bandpass filter 500-3000 Hz, check energy > threshold.
 *
 * @param ac_samples   AC-coupled current samples (mA)
 * @param n            Number of samples
 * @param sample_rate  Sample rate (Hz)
 * @return             true if HART signal detected
 */
bool hart_detect_signal(const double *ac_samples, size_t n,
                         double sample_rate);

/**
 * @brief Compute HART message turnaround time (L8).
 *
 * After transmitting a message, the master must wait for the slave
 * response. Typical turnaround: 256 ms (per HART specification).
 *
 * @param data_length  Number of data bytes in response
 * @return             Turnaround time in milliseconds
 */
double hart_turnaround_time_ms(size_t data_length);

/**
 * @brief Enumerate HART device in burst mode simulation (L8).
 *
 * In burst mode, a HART device continuously broadcasts its PV without
 * being polled. This function simulates the burst timing.
 *
 * @param burst_period_ms Desired burst period (ms)
 * @return                Actual burst period based on baud constraints (ms)
 */
double hart_burst_period(double burst_period_ms);

#ifdef __cplusplus
}
#endif

#endif /* HART_PROTOCOL_H */