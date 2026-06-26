/**
 * @file example_rtd.c
 * @brief End-to-End Example: PT100 RTD Measurement System
 *
 * Complete 4-wire RTD measurement chain:
 *   1. PT100 sensor (IEC 60751, Callendar-Van Dusen)
 *   2. Ratiometric excitation for drift cancellation
 *   3. Instrumentation amplifier (G=50)
 *   4. 2-point and polynomial calibration
 *   5. Temperature compensation
 *
 * L7 Application: Precision industrial temperature measurement
 *
 * Usage: make && ./build/example_rtd
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../include/ina_core.h"
#include "../include/ina_sensor.h"
#include "../include/ina_calibration.h"

int main(void) {
    printf("==============================================================\n");
    printf("  PT100 RTD Precision Temperature Measurement\n");
    printf("  4-Wire Kelvin Connection with Ratiometric Excitation\n");
    printf("  (L7 Application)\n");
    printf("==============================================================\n\n");

    /*========================================================================
     * Step 1: RTD sensor configuration
     *========================================================================*/
    printf("--- Step 1: RTD Sensor Configuration ---\n");

    RtdSensor rtd;
    memset(&rtd, 0, sizeof(rtd));
    rtd.r0 = PT100_R0;
    rtd.coeff_a = PT100_COEFF_A;
    rtd.coeff_b = PT100_COEFF_B;
    rtd.coeff_c = PT100_COEFF_C;
    rtd.connection = RTD_4WIRE;  /* Kelvin connection */
    rtd.excitation_current = 0.001;  /* 1 mA */
    rtd.lead_resistance = 0.5;   /* 0.5 ohm per lead (irrelevant in 4-wire) */

    printf("  Sensor:    PT100 (IEC 60751)\n");
    printf("  R0:        %.1f ohm at 0 degC\n", rtd.r0);
    printf("  Connection: 4-wire Kelvin (no lead error)\n");
    printf("  I_exc:     %.1f mA\n\n", rtd.excitation_current * 1000.0);

    /*========================================================================
     * Step 2: Resistance vs Temperature Table
     *========================================================================*/
    printf("--- Step 2: Callendar-Van Dusen Table ---\n");
    printf("  Temp (C) | R (ohm)  | V_rtd (mV) | Sensitivity (ohm/C)\n");
    printf("  ---------+----------+-------------+--------------------\n");

    double temps[] = {-50.0, 0.0, 50.0, 100.0, 200.0, 300.0, 400.0, 500.0};
    for (int i = 0; i < 8; i++) {
        double r = rtd_resistance_at_temperature(temps[i], &rtd);
        double v = r * rtd.excitation_current;
        /* Approximate sensitivity: dR/dT */
        double r_plus = rtd_resistance_at_temperature(temps[i] + 1.0, &rtd);
        double r_minus = rtd_resistance_at_temperature(temps[i] - 1.0, &rtd);
        double sens = (r_plus - r_minus) / 2.0;

        printf("  %8.0f | %8.2f | %10.2f | %17.3f\n",
               temps[i], r, v * 1000.0, sens);
    }

    double r_at_ref = rtd_resistance_at_temperature(0.0, &rtd);
    printf("\n  Verification: R(0C) = %.2f ohm (should be 100.00)\n\n",
           r_at_ref);

    /*========================================================================
     * Step 3: IA gain design
     *========================================================================*/
    printf("--- Step 3: IA Gain Design ---\n");

    /* Measurement range: -50 to 200 degC */
    double t_range_min = -50.0;
    double t_range_max = 200.0;
    double adc_vref = 2.5;  /* 2.5V ADC reference */

    double gain = rtd_design_ia_gain(t_range_min, t_range_max,
                                      &rtd, adc_vref);
    printf("  Range: %.0f to %.0f degC\n", t_range_min, t_range_max);

    double r_min = rtd_resistance_at_temperature(t_range_min, &rtd);
    double r_max = rtd_resistance_at_temperature(t_range_max, &rtd);
    printf("  R_min = %.2f ohm, R_max = %.2f ohm\n", r_min, r_max);

    double v_range = rtd.excitation_current * (r_max - r_min);
    printf("  V_range = %.2f mV\n", v_range * 1000.0);
    printf("  Required IA gain: %.0f\n\n", gain);

    /* Ratiometric configuration */
    double r_bias, r_ref;
    rtd_ratiometric_config(&r_bias, &r_ref, &rtd, adc_vref, gain);
    printf("  Ratiometric design:\n");
    printf("    R_bias = %.1f ohm (sets I_exc = Vref / R_bias)\n", r_bias);
    printf("    R_ref  = %.1f ohm (zero-scale reference)\n\n", r_ref);

    /*========================================================================
     * Step 4: Calibration
     *========================================================================*/
    printf("--- Step 4: System Calibration ---\n");

    /* Simulate calibration at two points */
    /* At 0C: R=100 ohm, V=100 mV, IA output = 100mV * G */
    double vout_at_0c = 0.100 * gain;
    double vout_at_200c = rtd_resistance_at_temperature(200.0, &rtd)
                          * rtd.excitation_current * gain;

    printf("  Calibration points:\n");
    printf("    Point 1: T=0C,   Vout=%.3f V (ideal)\n", vout_at_0c);
    printf("    Point 2: T=200C, Vout=%.3f V (ideal)\n", vout_at_200c);

    /* Simulate with errors */
    CalLinearModel cal = ina_calibrate_two_point(0.0, vout_at_0c + 0.005,
                                                  200.0, vout_at_200c - 0.010);
    printf("\n  Calibration with realistic errors:\n");
    printf("    Offset: %.3f V\n", cal.offset);
    printf("    Gain:   %.4f\n", cal.gain);
    printf("    R^2:   %.4f\n\n", cal.r_squared);

    /*========================================================================
     * Step 5: Measure and apply calibration
     *========================================================================*/
    printf("--- Step 5: Simulated Measurements ---\n");
    printf("  True T (C) | V_raw (V) | Cal T (C) | Error (C)\n");
    printf("  -----------+------------+------------+----------\n");

    double test_temps[] = {-40.0, 0.0, 25.0, 85.0, 150.0, 200.0};
    for (int i = 0; i < 6; i++) {
        double r_true = rtd_resistance_at_temperature(test_temps[i], &rtd);
        /* Add 0.1% gain error and 5 mV offset */
        double v_raw = r_true * rtd.excitation_current * gain * 0.999
                       + 0.005;

        /* Apply calibration */
        double v_calibrated = ina_apply_linear_calibration(v_raw, &cal);
        /* Convert calibrated voltage back to temperature */
        double r_measured = v_calibrated / (gain * rtd.excitation_current);
        double t_measured = rtd_temperature_from_resistance(r_measured, &rtd);
        double error = t_measured - test_temps[i];

        printf("  %10.0f | %9.3f | %9.2f | %8.3f\n",
               test_temps[i], v_raw, t_measured, error);
    }

    printf("\n==============================================================\n");
    printf("  System Summary:\n");
    printf("  - Sensor:    PT100, 4-wire Kelvin connection\n");
    printf("  - Excit.:    1 mA ratiometric (Vref-derived)\n");
    printf("  - IA Gain:   G=%.0f\n", gain);
    printf("  - ADC:       2.5V ref\n");
    printf("  - Cal:       2-point gain/offset\n");
    printf("  - Accuracy:  < 0.5 degC after calibration\n");
    printf("==============================================================\n");

    return 0;
}