#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../include/current_loop.h"
#include "../include/transmitter.h"
#include "../include/receiver.h"
#include "../include/loop_power.h"
#include "../include/hart_protocol.h"
#include "../include/sensor_interface.h"
#include "../include/calibration.h"
#include "../include/loop_diagnostics.h"

static int passed = 0, total = 0;

static void check_true(int cond, const char *msg) {
    total++;
    if (cond) { passed++; printf("  PASS: %s\n", msg); }
    else printf("  FAIL: %s\n", msg);
}

static void check_eq(double a, double b, const char *msg) {
    total++;
    if (fabs(a - b) < 0.001) { passed++; printf("  PASS: %s\n", msg); }
    else printf("  FAIL: %s (%.4f vs %.4f)\n", msg, a, b);
}

int main(void) {
    printf("=== 4-20mA Current Loop Test Suite ===\n\n");

    /* L1: Ohm's Law */
    check_eq(current_loop_from_shunt_voltage(1.0, 250.0), 4.0, "1V/250ohm = 4mA");
    check_eq(current_loop_to_shunt_voltage(12.0, 250.0), 3.0, "12mA -> 3V");

    /* L4: KVL */
    current_loop_t loop;
    current_loop_init_standard_24v(&loop);
    loop.loop_current_mA = 12.0;
    current_loop_kvl_solve(&loop);
    check_true(loop.total_resistance > 0.0, "KVL: total resistance");
    check_true(loop.voltage_margin > 0.0, "KVL: positive margin");

    /* L3: Transfer function */
    current_loop_transfer_t tf = {0.0, 100.0, 4.0, 20.0, 0.0, 1.0, false, true};
    check_eq(current_loop_process_to_current(&tf, 50.0), 12.0, "TF: 50% -> 12mA");
    check_eq(current_loop_current_to_process(&tf, 12.0), 50.0, "TF: 12mA -> 50%");

    /* L6: State classification */
    check_true(current_loop_classify_state(0.0) == LOOP_STATE_OPEN, "State: 0mA=OPEN");
    check_true(current_loop_classify_state(12.0) == LOOP_STATE_NORMAL, "State: 12mA=NORMAL");
    check_true(current_loop_classify_state(3.5) == LOOP_STATE_NAMUR_FAIL, "State: 3.5mA=NAMUR");

    /* L3: Noise */
    double samples[] = {12.0, 12.1, 11.9, 12.0, 12.05, 11.95, 12.0, 12.0};
    double mean;
    double rms = current_loop_noise_rms(samples, 8, &mean);
    check_true(rms > 0.0 && rms < 1.0, "Noise: RMS in range");

    /* L3: ENOB */
    check_true(current_loop_enob(0.001) > 12.0, "ENOB > 12 bits");
    check_true(current_loop_snr_db(0.001) > 80.0, "SNR > 80 dB");

    /* L6: RC filter */
    double resp = current_loop_rc_step_response(0.0025, 4.0, 20.0, 250.0, 10e-6);
    check_true(resp > 12.0 && resp < 16.0, "RC step response");

    /* L5: IIR filter */
    double out = current_loop_iir_filter(20.0, 4.0, 0.1);
    check_true(out > 4.0 && out < 20.0, "IIR filter");

    /* L2/L7: Intrinsic safety */
    check_true(current_loop_verify_intrinsic_safety(12.0, 20.0, 10.0, 1.0, 'C'), "IS: safe params pass");
    check_true(!current_loop_verify_intrinsic_safety(20.0, 20.0, 10.0, 1.0, 'C'), "IS: unsafe params fail");

    /* L6: Calibration */
    double off, gain;
    current_loop_two_point_calibration(4.0, 20.0, 3.8, 20.2, &off, &gain);
    double cal = current_loop_apply_calibration(12.0, off, gain);
    check_true(fabs(cal - 12.0) < 1.0, "Calibration corrects error");

    /* L5: Polynomial (Horner) */
    double coeffs[] = {1.0, 2.0, 3.0};
    check_eq(current_loop_polynomial_eval(2.0, coeffs, 2), 17.0, "Poly: 1+2x+3x^2 at 2");

    /* L7: Percent */
    check_eq(current_loop_to_percent(12.0), 50.0, "Percent: 12mA=50%");
    check_eq(current_loop_from_percent(50.0), 12.0, "Percent: 50%=12mA");

    /* L8: HART checksum */
    hart_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.preamble_count = 5;
    frame.delimiter = 0x02;
    frame.address[0] = 0x80;
    frame.address_length = 1;
    frame.command = HART_CMD_READ_PV;
    frame.checksum = hart_compute_checksum(&frame);
    check_true(hart_validate_frame(&frame), "HART checksum");

    /* L5: ADC/DAC */
    double i_adc = current_loop_adc_to_current(2047, 12, 5.0, 250.0);
    check_true(i_adc > 8.0 && i_adc < 14.0, "ADC mid-scale");
    uint32_t dac = current_loop_current_to_dac(12.0, 16);
    check_true(dac > 30000 && dac < 35000, "DAC mid-scale");

    /* L2: Power budget */
    loop_power_budget_t budget;
    memset(&budget, 0, sizeof(budget));
    budget.supply_voltage = 24.0;
    budget.loop_current_mA = 4.0;
    budget.transmitter_consumed_mW = 10.0;
    current_loop_power_budget_solve(&budget);
    check_true(budget.is_sustainable, "Power budget sustainable");

    /* L5: Sensor RTD */
    check_eq(sensor_rtd_resistance(0.0, 100.0), 100.0, "RTD: 0C = 100 ohm");
    double r100 = sensor_rtd_resistance(100.0, 100.0);
    check_true(r100 > 138.0 && r100 < 139.0, "RTD: 100C ~ 138.5 ohm");

    /* L5: Thermocouple */
    double v_tc = sensor_tc_k_temp_to_voltage(100.0);
    check_true(v_tc > 3.5 && v_tc < 4.5, "TC K: 100C ~ 4.1mV");
    double t_tc = sensor_tc_k_voltage_to_temp(v_tc);
    check_true(fabs(t_tc - 100.0) < 5.0, "TC K inverse");

    /* L6: Linear regression */
    double x[] = {1.0, 2.0, 3.0, 4.0, 5.0};
    double y[] = {2.0, 4.0, 6.0, 8.0, 10.0};
    double slope, inter, r2;
    calibration_linear_regression(x, y, 5, &slope, &inter, &r2);
    check_eq(slope, 2.0, "OLS: slope=2");
    check_true(r2 > 0.99, "OLS: R^2~1");

    /* L7: NE107 */
    current_loop_t l2;
    current_loop_init_standard_24v(&l2);
    loop_error_budget_t eb;
    memset(&eb, 0, sizeof(eb));
    eb.rss_total_percent = 0.05;
    check_true(loop_diag_ne107_status(&l2, &eb, 0.1) == NE107_STATUS_OK, "NE107 OK");

    /* L7: Health score */
    loop_health_trend_t trend;
    memset(&trend, 0, sizeof(trend));
    trend.insulation_resistance_mohm = 100.0;
    check_true(loop_diag_health_score(&l2, &trend) > 80.0, "Health score");

    printf("\n=== %d/%d tests passed ===\n", passed, total);
    return (passed == total) ? 0 : 1;
}
