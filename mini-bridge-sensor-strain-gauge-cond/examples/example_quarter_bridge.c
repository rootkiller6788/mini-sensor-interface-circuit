/**
 * example_quarter_bridge.c - Quarter bridge strain measurement demo
 * Knowledge: L6 Canonical Problem - quarter bridge strain measurement
 * Course: MIT 6.002, Berkeley EE16A, TU Munich EI0430
 */
#include <stdio.h>
#include <math.h>
#include "bridge_core.h"

int main(void) {
    printf("=== Quarter Bridge Strain Measurement ===\n\n");
    bridge_sensor_t sensor;
    bridge_sensor_init(&sensor);
    sensor.config = BRIDGE_QUARTER;
    sensor.bridge.v_excitation = 2.5;
    sensor.gauge[0].nominal_resistance = 120.0;
    sensor.gauge[0].gauge_factor = 2.08;
    bridge_state_init(&sensor.bridge, 120.0, 2.5, BRIDGE_QUARTER);

    printf("Configuration: Quarter bridge, 120 Ohm, GF=2.08, Vexc=2.5V\n");
    printf("Sensitivity: %.4f mV/V at 1000 ue\n\n",
           bridge_sensitivity_mv_per_v(BRIDGE_QUARTER, 2.08, 1000.0));

    printf("Strain(ue) | Vout(mV)  | Recovered(ue) | NL(%%)\n");
    double strains[] = {0, 100, 250, 500, 750, 1000, 1500, 2000};
    int i;
    for (i = 0; i < 8; i++) {
        double eps = strains[i];
        sensor.bridge.r1 = gauge_resistance_from_strain(120.0, 2.08, eps);
        double vout = bridge_output_voltage(&sensor.bridge);
        double eps_r = bridge_output_to_strain(vout, 2.5, 2.08, BRIDGE_QUARTER);
        double nl = bridge_nonlinearity_error(eps, 2.08, BRIDGE_QUARTER);
        printf("%9.0f | %9.4f | %13.1f | %6.3f\n", eps, vout*1000, eps_r, nl);
        sensor.bridge.r1 = 120.0;
    }
    printf("\nKey: Q-bridge output ~0.52mV/100ue at 2.5V. NL > 0.5%% above 1000ue.\n");
    return 0;
}
