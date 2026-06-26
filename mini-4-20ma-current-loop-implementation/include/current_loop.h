/**
 * @file current_loop.h
 * @brief Core definitions, data types, and fundamental theorems for 4-20mA current loop.
 *
 * 4-20mA current loop is the dominant analog signaling standard in industrial
 * process control (ISA-50.1, IEC 60381-1). This header defines the complete
 * type system for modeling, designing, and analyzing current loop circuits.
 *
 * Reference: ISA-50.1, IEC 60381-1, NAMUR NE43
 * Textbook: Sedra & Smith (2020), Microelectronic Circuits, 8th Ed
 *
 * Knowledge Coverage:
 * - L1: Current loop span/zero/live-zero, compliance voltage, loop resistance
 * - L2: 2-wire/3-wire/4-wire topologies, loop power budget
 * - L3: Transfer function models, linearity error propagation
 * - L4: Kirchhoff's voltage/current laws applied to loops
 */

#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CURRENT_LOOP_MIN_mA         (4.0)
#define CURRENT_LOOP_MAX_mA         (20.0)
#define CURRENT_LOOP_SPAN_mA        (16.0)
#define CURRENT_LOOP_ZERO_mA        (4.0)
#define CURRENT_LOOP_NAMUR_LOW_mA   (3.6)
#define CURRENT_LOOP_NAMUR_HIGH_mA  (21.0)
#define CURRENT_LOOP_OVERRANGE_mA   (20.5)

typedef enum {
    CURRENT_LOOP_TOPOLOGY_TWO_WIRE   = 2,
    CURRENT_LOOP_TOPOLOGY_THREE_WIRE = 3,
    CURRENT_LOOP_TOPOLOGY_FOUR_WIRE  = 4
} current_loop_topology_t;

typedef enum {
    LOOP_STATE_OFF        = 0,
    LOOP_STATE_INIT       = 1,
    LOOP_STATE_NORMAL     = 2,
    LOOP_STATE_OVERRANGE  = 3,
    LOOP_STATE_UNDERRANGE = 4,
    LOOP_STATE_OPEN       = 5,
    LOOP_STATE_SHORT      = 6,
    LOOP_STATE_NAMUR_FAIL = 7
} current_loop_state_t;

typedef struct {
    double supply_voltage;
    double supply_tolerance_percent;
    double loop_current_mA;
    double loop_current_target_mA;
    double shunt_resistance;
    double cable_resistance;
    double barrier_resistance;
    double transmitter_min_voltage;
    double transmitter_quiescent_mA;
    current_loop_topology_t topology;
    double total_resistance;
    double compliance_voltage;
    double voltage_margin;
    double power_delivered_mW;
    double loop_efficiency_percent;
    current_loop_state_t state;
} current_loop_t;

typedef struct {
    double process_min;
    double process_max;
    double current_min_mA;
    double current_max_mA;
    double offset_mA;
    double gain;
    bool   is_inverted;
    bool   is_linear;
} current_loop_transfer_t;

typedef struct {
    double low_alarm_mA;
    double low_saturation_mA;
    double high_saturation_mA;
    double high_alarm_mA;
    double short_circuit_mA;
} namur_ne43_levels_t;

typedef struct {
    double supply_voltage;
    double loop_current_mA;
    double total_available_mW;
    double transmitter_consumed_mW;
    double sensor_excitation_mW;
    double adc_power_mW;
    double mcu_power_mW;
    double display_power_mW;
    double hart_modem_power_mW;
    double margin_mW;
    double total_consumed_mW;
    bool   is_sustainable;
} loop_power_budget_t;

typedef struct {
    double sensor_error_percent;
    double transmitter_error_percent;
    double adc_quantization_error_percent;
    double dac_quantization_error_percent;
    double reference_error_percent;
    double temperature_drift_percent;
    double nonlinearity_error_percent;
    double cable_leakage_error_percent;
    double rss_total_percent;
    double worst_case_total_percent;
} loop_error_budget_t;

#define HART_PREAMBLE_MIN      5
#define HART_PREAMBLE_MAX      20
#define HART_FRAME_MAX_BYTES   37
#define HART_FSK_MARK_HZ       1200.0
#define HART_FSK_SPACE_HZ      2200.0
#define HART_BAUD              1200

typedef enum {
    HART_CMD_READ_PV           = 1,
    HART_CMD_READ_CURRENT      = 2,
    HART_CMD_READ_PV_CURRENT   = 3,
    HART_CMD_WRITE_DAC         = 40,
    HART_CMD_READ_DEVICE_INFO  = 0,
    HART_CMD_READ_TAG          = 13,
    HART_CMD_WRITE_TAG         = 18
} hart_command_t;

typedef struct {
    uint8_t preamble_count;
    uint8_t delimiter;
    uint8_t address[5];
    uint8_t address_length;
    hart_command_t command;
    uint8_t byte_count;
    uint8_t data[HART_FRAME_MAX_BYTES];
    uint8_t data_length;
    uint8_t checksum;
} hart_frame_t;

void current_loop_kvl_solve(current_loop_t *loop);
double current_loop_max_cable_length(const current_loop_t *loop, double cable_r_per_meter);
double current_loop_min_supply_voltage(const current_loop_t *loop);
double current_loop_from_shunt_voltage(double v_shunt, double r_shunt);
double current_loop_to_shunt_voltage(double current_mA, double r_shunt);
double current_loop_process_to_current(const current_loop_transfer_t *tf, double process_value);
double current_loop_current_to_process(const current_loop_transfer_t *tf, double current_mA);
double current_loop_piecewise_linearize(double x, const double *breakpoints_x, const double *breakpoints_y, size_t n_breakpoints);
current_loop_state_t current_loop_classify_state(double current_mA);

#define LOOP_DIAG_OK                0x00
#define LOOP_DIAG_OPEN_CIRCUIT      0x01
#define LOOP_DIAG_SHORT_CIRCUIT     0x02
#define LOOP_DIAG_LOW_CURRENT       0x04
#define LOOP_DIAG_HIGH_CURRENT      0x08
#define LOOP_DIAG_NOISE_EXCESSIVE   0x10
#define LOOP_DIAG_SUPPLY_LOW        0x20
#define LOOP_DIAG_CALIBRATION_DRIFT 0x40
#define LOOP_DIAG_GROUND_FAULT      0x80

uint8_t current_loop_diagnose(const current_loop_t *loop, double measured_current_mA, double measured_supply_voltage, double noise_stddev_mA);
double current_loop_noise_rms(const double *samples, size_t n, double *mean_out);
double current_loop_enob(double noise_rms_mA);
double current_loop_snr_db(double noise_rms_mA);
double current_loop_rc_step_response(double t_seconds, double i_initial_mA, double i_final_mA, double r_ohms, double c_farads);
double current_loop_rc_cutoff_frequency(double r_ohms, double c_farads);
double current_loop_rc_settling_time(double r_ohms, double c_farads, double percent);
double current_loop_isolation_voltage(double supply_voltage, double safety_factor);
bool current_loop_verify_intrinsic_safety(double voltage, double current_mA, double capacitance_nF, double inductance_mH, char gas_group);
double current_loop_iir_filter(double current_sample, double previous_filtered, double alpha);
double current_loop_iir_alpha(double cutoff_hz, double sample_hz);
double current_loop_moving_average(double new_sample, double *ring_buffer, size_t window_size, size_t *sample_index);
void current_loop_median_filter(double *samples, size_t n, size_t window_size);
void current_loop_init_standard_24v(current_loop_t *loop);
bool current_loop_check_compliance(const current_loop_t *loop, double target_mA);
double current_loop_compute_accuracy(const loop_error_budget_t *error_budget, bool use_rss);
double current_loop_shunt_power(double current_mA, double r_shunt);
double current_loop_cable_voltage_drop(double current_mA, double r_cable);
void current_loop_power_budget_solve(loop_power_budget_t *budget);
void namur_ne43_init_default(namur_ne43_levels_t *levels);
double current_loop_adc_to_current(uint32_t adc_code, uint8_t adc_bits, double v_ref, double r_shunt);
uint32_t current_loop_current_to_dac(double current_mA, uint8_t dac_bits);
uint8_t hart_compute_checksum(const hart_frame_t *frame);
bool hart_validate_frame(const hart_frame_t *frame);
float hart_parse_float(const uint8_t bytes[4]);
void hart_encode_float(float value, uint8_t bytes[4]);
void current_loop_two_point_calibration(double ref_low_mA, double ref_high_mA, double measured_low_mA, double measured_high_mA, double *offset_out, double *gain_out);
double current_loop_apply_calibration(double raw_mA, double offset, double gain);
double current_loop_polynomial_eval(double x, const double *coeffs, size_t degree);
double current_loop_temp_compensation(double raw_current_mA, double temperature_c, double ref_temp_c, double alpha_per_c);
double current_loop_to_percent(double current_mA);
double current_loop_from_percent(double percent);
int current_loop_infer_topology_from_startup(const double *startup_samples, size_t n, double sample_period_s);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_LOOP_H */