/**
 * @file demo.c
 * @brief Interactive demonstration of 4-20mA current loop concepts.
 */
#include <stdio.h>
#include "../include/current_loop.h"

int main(void) {
    printf("=== 4-20mA Current Loop Demo ===\n\n");

    current_loop_t loop;
    current_loop_init_standard_24v(&loop);

    printf("Standard 24V industrial loop parameters:\n");
    printf("  Supply: %.1f V\n", loop.supply_voltage);
    printf("  Shunt:  %.0f ohm (1-5V signal)\n", loop.shunt_resistance);
    printf("  Cable:  %.1f ohm\n", loop.cable_resistance);
    printf("  TX min: %.1f V\n", loop.transmitter_min_voltage);

    printf("\nLoop current sweep (4-20mA):\n");
    printf("  Current | Shunt V | Compliance | State\n");
    printf("  --------|---------|------------|------\n");
    for (double ma = 4.0; ma <= 20.0; ma += 2.0) {
        loop.loop_current_mA = ma;
        current_loop_kvl_solve(&loop);
        double sv = current_loop_to_shunt_voltage(ma, loop.shunt_resistance);
        const char *state_names[] = {"OFF","INIT","NORMAL","OVERRANGE",
            "UNDERRANGE","OPEN","SHORT","NAMUR_FAIL"};
        printf("  %6.1f | %7.3f | %10.2f | %s\n",
               ma, sv, loop.voltage_margin, state_names[loop.state]);
    }

    printf("\n=== Demo Complete ===\n");
    return 0;
}
