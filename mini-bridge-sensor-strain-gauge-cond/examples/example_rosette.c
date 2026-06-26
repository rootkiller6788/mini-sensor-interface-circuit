/**
 * example_rosette.c - 2D strain rosette analysis
 * Knowledge: L6 Canonical Problem - experimental stress analysis
 * Course: Michigan EECS 411, THU Experimental Mechanics
 */
#include <stdio.h>
#include <math.h>
#include "bridge_core.h"
#include "strain_gauge_physics.h"

int main(void) {
    printf("=== 2D Strain Rosette Analysis ===\n\n");

    rosette_data_t rect = {ROSETTE_RECTANGULAR, 1200.0, 350.0, -400.0};
    rosette_resolve_strain(&rect);
    printf("Rectangular (0/45/90): e0=%.0f e45=%.0f e90=%.0f ue\n",
           rect.ea, rect.eb, rect.ec);
    printf("  eps_x=%.1f eps_y=%.1f gamma_xy=%.1f\n",
           rect.resolved.epsilon_x, rect.resolved.epsilon_y,
           rect.resolved.gamma_xy);
    printf("  Principal: e1=%.1f e2=%.1f theta=%.1f deg\n",
           rect.resolved.epsilon_1, rect.resolved.epsilon_2,
           rect.resolved.angle_deg);

    stress_state_t stress;
    hookes_law_plane_stress(&rect.resolved, 200.0, 0.29, &stress);
    printf("  Stress (steel): sig1=%.1f sig2=%.1f MPa, VM=%.1f MPa\n\n",
           stress.sigma_1, stress.sigma_2, stress.sigma_von_mises);

    rosette_data_t delta = {ROSETTE_DELTA, 800.0, 250.0, -150.0};
    rosette_resolve_strain(&delta);
    printf("Delta (0/60/120): e0=%.0f e60=%.0f e120=%.0f ue\n",
           delta.ea, delta.eb, delta.ec);
    printf("  eps_x=%.1f eps_y=%.1f gamma_xy=%.1f\n",
           delta.resolved.epsilon_x, delta.resolved.epsilon_y,
           delta.resolved.gamma_xy);

    double center, radius;
    strain_gauge_mohr_circle(&delta.resolved, &center, &radius);
    printf("  Mohr: center=%.1f radius=%.1f ue\n", center, radius);
    printf("\n[PASS] Rosette analysis complete.\n");
    return 0;
}
