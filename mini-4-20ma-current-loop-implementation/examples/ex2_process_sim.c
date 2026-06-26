/**
 * ex2_process_sim.c -- Process simulation with sensor interface and calibration.
 */
#include <stdio.h>
#include <math.h>
#include "../include/current_loop.h"
#include "../include/sensor_interface.h"
#include "../include/calibration.h"

int main(void)
{
    printf("=== Example 2: Process Simulation with Sensor Interface ===\n\n");

    printf("--- RTD PT100 Temperature Measurement (IEC 60751) ---\n");
    double temps[] = {-50.0, 0.0, 25.0, 100.0, 200.0, 400.0};
    for (int i = 0; i < 6; i++) {
        double r = sensor_rtd_resistance(temps[i], 100.0);
        double t_back = sensor_rtd_to_temperature(r, 100.0);
        printf("  T = %6.1f C -> R = %7.2f ohm -> T = %6.1f C\n", temps[i], r, t_back);
    }

    printf("\n--- Type K Thermocouple with CJC ---\n");
    double cj_temp = 25.0;
    double hot_temps[] = {50.0, 100.0, 200.0, 500.0};
    for (int i = 0; i < 4; i++) {
        double v_hot = sensor_tc_k_temp_to_voltage(hot_temps[i]);
        double v_cj  = sensor_tc_k_temp_to_voltage(cj_temp);
        double v_measured = v_hot - v_cj;
        double t_comp = sensor_cjc_compensate(v_measured, cj_temp, SENSOR_TYPE_THERMOCOUPLE_K);
        printf("  T_hot=%6.1f C, V_meas=%7.3f mV -> T_cjc=%6.1f C\n", hot_temps[i], v_measured, t_comp);
    }

    printf("\n--- Strain Gauge Bridge (GF=2.0, V_exc=5V, Full Bridge) ---\n");
    double strains_ue[] = {0.0, 250.0, 500.0, 1000.0, 2000.0};
    for (int i = 0; i < 5; i++) {
        double v_out = -(strains_ue[i] / 1e6) * 2.0 * 5.0 / 4.0;
        double ue_back = sensor_strain_to_microstrain(v_out, 5.0, 2.0, 4);
        printf("  Strain = %6.0f ue -> V_out = %8.5f V -> Strain = %6.0f ue\n",
               strains_ue[i], v_out, ue_back);
    }

    printf("\n--- Two-Point Calibration ---\n");
    double offset, gain;
    current_loop_two_point_calibration(4.0, 20.0, 3.85, 20.15, &offset, &gain);
    printf("  Reference: 4.00 / 20.00 mA\n");
    printf("  Measured:  3.85 / 20.15 mA\n");
    printf("  Offset = %.3f mA, Gain = %.4f\n", offset, gain);
    double test_raw[] = {3.85, 8.0, 12.0, 16.0, 20.15};
    for (int i = 0; i < 5; i++) {
        double cal = current_loop_apply_calibration(test_raw[i], offset, gain);
        printf("  Raw %.2f mA -> Calibrated %.2f mA\n", test_raw[i], cal);
    }

    printf("\n--- Digital Filtering ---\n");
    double alpha = current_loop_iir_alpha(5.0, 100.0);
    printf("  IIR alpha for fc=5Hz, fs=100Hz: %.4f\n", alpha);
    double filtered = 4.0;
    double noisy_samples[] = {4.1, 4.3, 3.8, 4.0, 4.5, 3.9, 3.7, 4.2};
    printf("  Raw  -> Filtered (step response):\n");
    for (int i = 0; i < 8; i++) {
        filtered = current_loop_iir_filter(noisy_samples[i], filtered, alpha);
        printf("    %.1f -> %.3f mA\n", noisy_samples[i], filtered);
    }

    printf("\n=== Simulation Complete ===\n");
    return 0;
}
