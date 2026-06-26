/**
 * @file    ex2_rtd_4wire.c
 * @brief   Example 2: Precision 4-wire RTD measurement with self-heating analysis
 *
 * Demonstrates:
 *   1. 4-wire (Kelvin) Pt100 measurement
 *   2. Self-heating correction for different excitation currents
 *   3. Ratio-metric measurement technique
 *   4. Comparison of IEC, US Industrial, and Old US alpha standards
 *
 * Knowledge: L6 - 4-wire RTD measurement, self-heating analysis
 * Reference: IEC 60751, Callendar-Van Dusen equation
 */

#include "thermocouple_cjc_rtd.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    rtd_cvd_coeffs_t coeffs_iec, coeffs_us, coeffs_oldus;
    rtd_self_heating_t sh;
    rtd_measurement_t result;
    double test_temps[] = { -50.0, 0.0, 25.0, 100.0, 200.0, 400.0, 600.0 };
    size_t i, n = 7;
    tc_error_t err;

    printf("=== Example 2: Precision 4-Wire RTD Measurement ===\n\n");

    /* Load coefficient standards */
    tc_rtd_get_coeffs(RTD_TYPE_PT100, RTD_ALPHA_IEC_385, &coeffs_iec);
    tc_rtd_get_coeffs(RTD_TYPE_PT100, RTD_ALPHA_US_392, &coeffs_us);
    tc_rtd_get_coeffs(RTD_TYPE_PT100, RTD_ALPHA_US_390, &coeffs_oldus);

    printf("RTD Coefficients:\n");
    printf("  IEC 60751:  R0=%.1f, A=%.6e, B=%.6e, alpha=%.6f\n",
           coeffs_iec.r0, coeffs_iec.a, coeffs_iec.b, coeffs_iec.alpha);
    printf("  US Ind'l:   R0=%.1f, A=%.6e, B=%.6e, alpha=%.6f\n",
           coeffs_us.r0, coeffs_us.a, coeffs_us.b, coeffs_us.alpha);
    printf("  Old US:     R0=%.1f, A=%.6e, B=%.6e, alpha=%.6f\n\n",
           coeffs_oldus.r0, coeffs_oldus.a, coeffs_oldus.b, coeffs_oldus.alpha);

    /* Self-heating model: ceramic Pt100 in still air */
    sh.dissipation_constant = 75.0;  /* K/W in still air */
    sh.max_current = 0.005;          /* 5 mA max */
    sh.max_power = 0.005;            /* 5 mW max */
    sh.medium = 0;                   /* Air */
    sh.medium_conductivity = 0.026;  /* W/(m*K) for air */
    sh.time_constant = 4.0;          /* seconds */

    printf("Self-Heating Model: dissipation=%.1f K/W (still air)\n\n", sh.dissipation_constant);

    /* Demonstrate 4-wire measurement at different temperatures */
    printf("%-10s %-14s %-14s %-12s %-12s\n",
           "T(C)", "R_IEC(ohm)", "R_US(ohm)", "T_IEC(C)", "SH_rise(C)");
    printf("-----------------------------------------------------------------\n");

    for (i = 0; i < n; i++) {
        double r_iec, r_us, v_sense, i_excite = 1.0e-3; /* 1 mA */
        double delta_t;

        tc_rtd_temp_to_r(&coeffs_iec, test_temps[i], &r_iec);
        tc_rtd_temp_to_r(&coeffs_us, test_temps[i], &r_us);

        /* Simulate 4-wire measurement */
        v_sense = i_excite * r_iec;
        err = tc_rtd_4wire_measurement(v_sense, i_excite, &coeffs_iec, &result);

        /* Compute self-heating */
        tc_rtd_self_heating(r_iec, i_excite, &sh, &delta_t);

        printf("%-10.1f %-14.4f %-14.4f %-12.3f %-12.6f\n",
               test_temps[i], r_iec, r_us,
               result.temperature, delta_t);
    }

    /* Ratio-metric measurement demonstration */
    printf("\nRatio-metric measurement (R_ref=100 ohm, T=100C):\n");
    {
        double r_rtd, v_rtd, v_ref, r_ref = 100.0, i_ex = 1e-3;
        tc_rtd_temp_to_r(&coeffs_iec, 100.0, &r_rtd);
        v_rtd = i_ex * r_rtd;
        v_ref = i_ex * r_ref;

        err = tc_rtd_ratiometric(v_rtd, v_ref, r_ref, &coeffs_iec, &result);
        printf("  V_rtd=%.6fV, V_ref=%.6fV, R_rtd=%.4f ohm, T=%.3fC\n",
               v_rtd, v_ref, result.resistance, result.temperature);
    }

    printf("\n=== RTD Measurement Complete ===\n");
    return 0;
}
