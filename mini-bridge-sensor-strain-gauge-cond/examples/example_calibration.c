/**
 * example_calibration.c - Bridge sensor calibration workflow
 * Knowledge: L5 Algorithm - multi-point calibration
 * Course: TU Munich EI0430, THU Sensor Technology
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "bridge_core.h"
#include "bridge_calibration.h"

int main(void) {
    printf("=== Bridge Sensor Calibration Workflow ===\n\n");

    strain_gauge_t gauge;
    strain_gauge_init(&gauge, 350.0, 2.05, "constantan");
    double eps_shunt = calibration_shunt_strain(&gauge, 350000.0, 1);
    printf("Step 1: Shunt cal (350kOhm) -> simulates %.1f ue\n", eps_shunt);

    double r_shunt = calibration_shunt_resistor(&gauge, -500.0, 1);
    printf("  For -500 ue: need R_shunt = %.0f Ohm\n\n", r_shunt);

    cal_data_point_t pts[7];
    int i;
    for (i = 0; i < 7; i++) {
        double f = i * 100.0;
        double v = 10.0 * 0.002 * f / 500.0 + (rand()%100-50)*1e-6;
        cal_data_point_init(&pts[i], f, v, 10.0, 25.0);
    }

    calibration_result_t result;
    calibration_linear_fit(pts, 7, &result);
    printf("Step 2: Linear fit: slope=%.6f V/kg, offset=%.6f V, R2=%.6f\n",
           result.slope, result.offset, result.r_squared);

    double coeffs[8] = {0};
    calibration_polynomial_fit(pts, 7, 2, coeffs);
    printf("Step 3: Poly fit: eps=%.6f+%.6f*V+%.6f*V^2\n",
           coeffs[0], coeffs[1], coeffs[2]);

    int best;
    double cv = calibration_cross_validate(pts, 7, 3, 3, &best);
    printf("Step 4: Cross-val: best order=%d, CV RMS error=%.6f\n", best, cv);

    double u = calibration_uncertainty(0.000005, 10, 0.01, 2.0);
    printf("Step 5: Uncertainty (k=2): %.3f ue\n", u);

    printf("\n[PASS] Calibration workflow complete.\n");
    return 0;
}
