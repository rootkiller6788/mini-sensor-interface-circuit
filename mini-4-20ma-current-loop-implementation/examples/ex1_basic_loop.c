/**
 * ex1_basic_loop.c -- Basic 4-20mA loop analysis and simulation.
 */
#include <stdio.h>
#include <math.h>
#include "../include/current_loop.h"

int main(void)
{
    printf("=== Example 1: Basic 4-20mA Loop Analysis ===\n\n");

    current_loop_t loop;
    current_loop_init_standard_24v(&loop);
    printf("Standard 24V loop initialized.\n");
    printf("  V_supply = %.1f V, R_shunt = %.0f ohm, R_cable = %.1f ohm\n",
           loop.supply_voltage, loop.shunt_resistance, loop.cable_resistance);

    loop.loop_current_mA = 12.0;
    current_loop_kvl_solve(&loop);
    printf("\n--- KVL Analysis at 12.0 mA ---\n");
    printf("  Total resistance: %.2f ohm\n", loop.total_resistance);
    printf("  Compliance voltage: %.2f V\n", loop.compliance_voltage);
    printf("  Voltage margin at 20mA: %.2f V\n", loop.voltage_margin);
    printf("  Power at receiver: %.2f mW\n", loop.power_delivered_mW);
    printf("  Loop efficiency: %.1f%%\n", loop.loop_efficiency_percent);

    double r_per_m = 0.033;
    double max_len = current_loop_max_cable_length(&loop, r_per_m);
    printf("\n--- Cable Analysis (AWG 20, %.3f ohm/m) ---\n", r_per_m);
    printf("  Maximum cable length: %.0f m (one way)\n", max_len);

    current_loop_transfer_t tf = {0.0, 10.0, 4.0, 20.0, 0.0, 1.0, false, true};
    printf("\n--- Transfer Function: 0-10 bar Pressure TX ---\n");
    double pressures[] = {0.0, 2.5, 5.0, 7.5, 10.0};
    for (int i = 0; i < 5; i++) {
        double i_out = current_loop_process_to_current(&tf, pressures[i]);
        double pv_back = current_loop_current_to_process(&tf, i_out);
        printf("  %.1f bar -> %.2f mA -> %.1f bar\n", pressures[i], i_out, pv_back);
    }

    printf("\n--- NAMUR NE43 State Classification ---\n");
    double test_currents[] = {0.0, 2.0, 3.5, 4.0, 12.0, 20.0, 21.0, 22.5, 25.0};
    const char *state_names[] = {"OFF","INIT","NORMAL","OVERRANGE","UNDERRANGE",
                                  "OPEN","SHORT","NAMUR_FAIL"};
    for (int i = 0; i < 9; i++) {
        current_loop_state_t st = current_loop_classify_state(test_currents[i]);
        printf("  %.1f mA -> %s\n", test_currents[i], state_names[st]);
    }

    printf("\n=== Analysis Complete ===\n");
    return 0;
}
