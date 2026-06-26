/**
 * @file    ex4_industrial_pid.c
 * @brief   Example 4: Industrial PID temperature controller with Type K feedback
 *
 * Demonstrates a complete industrial temperature control loop:
 *   1. Simulated plant (first-order thermal system with delay)
 *   2. Type K thermocouple measurement with CJC
 *   3. PID controller with anti-windup
 *   4. Kalman filter for measurement noise reduction
 *   5. Temperature setpoint tracking
 *
 * Knowledge: L7 - process control application, L8 - Kalman filtering
 * Reference: Astron & Hagglund, Ziegler-Nichols tuning
 */

#include "thermocouple_cjc_rtd.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Simulated thermal plant: first-order + dead time */
static double plant_simulate(double heater_power, double ambient, double *plant_temp,
                              double time_constant, double dt) {
    /* dT/dt = (heater_power * efficiency - (T - ambient) / thermal_R) / thermal_C
     * Simplified: T_new = T + dt/tau * (P * gain - (T - ambient)) */
    double tau = time_constant;
    double gain = 5.0; /* C per % power */
    double t_target = ambient + heater_power * gain;
    *plant_temp += (dt / tau) * (t_target - *plant_temp);
    return *plant_temp;
}

int main(void) {
    tc_measurement_config_t config;
    tc_measurement_t meas;
    double setpoint = 150.0;        /* Target: 150C */
    double plant_temp = 25.0;        /* Initial: ambient */
    double ambient = 25.0;
    double dt = 0.1;                 /* 100 ms control loop */
    double time_constant = 10.0;     /* Plant time constant */
    double heater_power = 0.0;
    double pid_integral = 0.0;
    double pid_prev_error = 0.0;
    double pid_limits[2] = { 0.0, 100.0 }; /* 0-100% heater */
    double kalman_T = 25.0, kalman_P = 1.0;
    size_t step, n_steps = 200;
    tc_error_t err;

    printf("=== Example 4: Industrial PID Temperature Controller ===\n");
    printf("Setpoint: %.1f C, Ambient: %.1f C\n", setpoint, ambient);
    printf("Plant time constant: %.1f s, Control period: %.3f s\n\n",
           time_constant, dt);

    /* Initialize measurement for Type K */
    err = tc_measurement_init(&config, TC_TYPE_K, WIRE_4_WIRE);
    if (err != TC_OK) {
        printf("ERROR: %s\n", tc_error_string(err));
        return 1;
    }
    config.cjc.cj_temperature = ambient;

    /* PID tuning (Ziegler-Nichols for temperature loop) */
    double Kp = 8.0, Ki = 0.15, Kd = 5.0;

    printf("%-6s %-10s %-10s %-10s %-10s %-10s\n",
           "Step", "T_plant(C)", "T_meas(C)", "T_filt(C)", "Heater(%)", "Error(C)");
    printf("-------------------------------------------------------------------\n");

    for (step = 0; step < n_steps; step++) {
        /* 1. Measure temperature (simulate ADC reading from plant) */
        {
            double emf_true, emf_raw, v_input;
            uint32_t adc_code;

            tc_temp_to_emf(TC_TYPE_K, plant_temp, &emf_true);
            /* Simulate measurement noise (~0.5C std dev) */
            emf_true += 0.020 * ((double)rand() / RAND_MAX - 0.5);
            emf_raw = emf_true; /* Assuming CJC already compensated */

            v_input = emf_raw * 1e-3;
            adc_code = (uint32_t)(v_input / config.adc_vref * 65536.0);
            tc_measure_temperature(&config, adc_code, &meas);
        }

        /* 2. Kalman filter for noise reduction */
        double T_filtered = tc_kalman_track_temperature(
            meas.temperature, dt, 0.05, 0.3, &kalman_T, &kalman_P);

        /* 3. PID control */
        heater_power = tc_pid_control(setpoint, T_filtered, dt,
                                       Kp, Ki, Kd,
                                       &pid_integral, &pid_prev_error,
                                       pid_limits);

        /* 4. Apply to plant */
        plant_simulate(heater_power, ambient, &plant_temp, time_constant, dt);

        /* Print every 20 steps */
        if (step % 20 == 0) {
            printf("%-6zu %-10.2f %-10.2f %-10.2f %-10.1f %-10.2f\n",
                   step, plant_temp, meas.temperature, T_filtered,
                   heater_power, setpoint - plant_temp);
        }
    }

    /* Steady-state results */
    printf("\n--- Steady State (final 50 steps) ---\n");
    printf("  Plant temperature:  %.2f C\n", plant_temp);
    printf("  Setpoint error:     %.3f C\n", setpoint - plant_temp);
    printf("  Heater power:       %.1f %%\n", heater_power);
    printf("  Kalman uncertainty: %.4f C\n", sqrt(kalman_P));

    printf("\n=== PID Control Complete ===\n");
    return 0;
}
