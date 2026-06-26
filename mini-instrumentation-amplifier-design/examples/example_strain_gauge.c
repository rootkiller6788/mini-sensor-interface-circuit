/**
 * @file example_strain_gauge.c
 * @brief End-to-End Example: Strain Gauge Measurement System
 *
 * A complete strain gauge measurement chain using an IA:
 *   1. Quarter-bridge strain gauge sensor (350 ohm, GF=2.0)
 *   2. 3-op-amp instrumentation amplifier (AD620-like, G=500)
 *   3. Anti-aliasing filter
 *   4. ADC interface with error budget analysis
 *
 * This example demonstrates the full L6 Canonical Problem
 * of designing a precision strain measurement system.
 *
 * Usage: make && ./build/example_strain_gauge
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../include/ina_core.h"
#include "../include/ina_topology.h"
#include "../include/ina_sensor.h"
#include "../include/ina_filter.h"

int main(void) {
    printf("==============================================================\n");
    printf("  Strain Gauge Measurement System Design\n");
    printf("  End-to-End Example (L6 Canonical Problem)\n");
    printf("==============================================================\n\n");

    /*========================================================================
     * Step 1: Define the sensor
     *========================================================================*/
    printf("--- Step 1: Sensor Definition ---\n");

    StrainGauge gauge;
    memset(&gauge, 0, sizeof(gauge));
    gauge.gauge_factor = 2.0;           /* Metal foil strain gauge */
    gauge.nominal_resistance = 350.0;   /* Standard 350 ohm gauge */
    gauge.youngs_modulus_gpa = 200.0;   /* Steel */
    gauge.poisson_ratio = 0.3;
    gauge.max_strain_ue = 2000.0;       /* +/- 2000 microstrain range */
    gauge.excitation_voltage = 5.0;     /* 5V bridge excitation */

    printf("  Gauge Factor:       %.1f\n", gauge.gauge_factor);
    printf("  Nominal Resistance: %.0f ohm\n", gauge.nominal_resistance);
    printf("  Excitation:         %.1f V\n", gauge.excitation_voltage);
    printf("  Max Strain:         %.0f ue\n\n", gauge.max_strain_ue);

    /*========================================================================
     * Step 2: Compute bridge output for full-scale strain
     *========================================================================*/
    printf("--- Step 2: Bridge Output Analysis ---\n");

    double v_bridge_fs = strain_to_bridge_output(
        gauge.max_strain_ue, &gauge, BRIDGE_QUARTER);
    printf("  Full-scale bridge output: %.3f mV\n", v_bridge_fs * 1000.0);

    /* Expected: 5 * 2 * 0.002 / 4 = 5 mV */
    double v_bridge_expected = 5.0 * 2.0 * 0.002 / 4.0;
    printf("  Expected (analytical):   %.3f mV\n",
           v_bridge_expected * 1000.0);

    /* Bridge output impedance */
    BridgeSensor bs;
    memset(&bs, 0, sizeof(bs));
    bs.type = BRIDGE_QUARTER;
    bs.excitation_voltage = 5.0;
    bs.nominal_resistance = 350.0;
    bs.sensor_resistance = 350.0;

    double rth = bridge_output_impedance(&bs);
    printf("  Bridge output impedance: %.0f ohm\n", rth);

    /* Johnson noise of the bridge */
    const double k_boltzmann = 1.380649e-23;
    double vn_bridge = sqrt(4.0 * k_boltzmann * 298.0 * rth * 1000.0);
    printf("  Bridge thermal noise (BW=1kHz): %.1f nVrms\n\n",
           vn_bridge * 1e9);

    /*========================================================================
     * Step 3: Design instrumentation amplifier
     *========================================================================*/
    printf("--- Step 3: IA Design (AD620-like) ---\n");

    /* Target: 5 mV full-scale bridge output -> 2.5 V ADC input */
    double adc_fs = 2.5;
    double gain_needed = adc_fs / v_bridge_fs;
    printf("  Required gain: %.1f\n", gain_needed);

    /* AD620: R_internal = 24.7k */
    double r_internal = 24700.0;
    double rg = ina_calculate_rg(gain_needed, r_internal);
    printf("  Rg required:    %.1f ohm\n", rg);

    /* Nearest standard E96 resistor */
    double rg_std = ina_nearest_standard_resistor(rg, 96);
    double gain_actual = ina_calculate_gain_from_rg(rg_std, r_internal);
    printf("  Rg (E96 std):   %.1f ohm\n", rg_std);
    printf("  Actual gain:    %.2f\n", gain_actual);
    printf("  Gain error:     %.2f%%\n\n",
           (gain_actual - gain_needed) / gain_needed * 100.0);

    /*========================================================================
     * Step 4: CMRR and error budget
     *========================================================================*/
    printf("--- Step 4: CMRR and Error Budget ---\n");

    /* CMRR analysis */
    double cmrr_db = ina_3opamp_cmrr(r_internal, rg_std, 0.1, 90.0);
    printf("  Estimated CMRR: %.1f dB\n", cmrr_db);

    /* Complete error budget */
    InaParameters params;
    memset(&params, 0, sizeof(params));
    params.gain = gain_actual;
    params.gain_error_percent = fabs(gain_actual - gain_needed)
                                / gain_needed * 100.0;
    params.vos_uv = 50.0;
    params.vos_drift_nv_per_C = 600.0;
    params.cmrr_db = cmrr_db;
    params.psrr_plus_db = 100.0;
    params.en_rti_at_gain1_nv_rms = 80.0;
    params.gain_nonlinearity_ppm = 10.0;
    params.output_swing_max = 4.5;
    params.output_swing_min = 0.5;

    InaErrorBudget budget = ina_compute_error_budget(&params,
        2.5, 0.05, 35.0, 350.0);

    printf("\n  Error Budget (RTI):\n");
    printf("    Offset error:        %.1f uV\n", budget.vos_error_uv);
    printf("    Gain error:          %.1f uV\n", budget.gain_error_uv);
    printf("    CMRR error:          %.1f uV\n", budget.cmrr_error_uv);
    printf("    PSRR error:          %.1f uV\n", budget.psrr_error_uv);
    printf("    Noise error (RMS):   %.1f uV\n", budget.noise_error_uv_rms);
    printf("    Nonlinearity:        %.1f uV\n", budget.nonlinearity_error_uv);
    printf("    Ib*Rs offset:        %.1f uV\n", budget.ib_offset_error_uv);
    printf("    ----------------------------------------\n");
    printf("    Total RSS:           %.1f uV\n", budget.total_error_rss_uv);
    printf("    Total Worst-Case:    %.1f uV\n", budget.total_error_worst_uv);
    printf("    Effective Resolution: %.1f bits\n\n",
           budget.effective_resolution_bits);

    /*========================================================================
     * Step 5: Simulate measurements at different strains
     *========================================================================*/
    printf("--- Step 5: Simulated Measurements ---\n");
    printf("  Strain (ue) | Vbridge (mV) | Vout (V) | Stress (MPa)\n");
    printf("  ------------+--------------+----------+-------------\n");

    double strains[] = {0.0, 250.0, 500.0, 1000.0, 1500.0, 2000.0};
    for (int i = 0; i < 6; i++) {
        double vb = strain_to_bridge_output(strains[i], &gauge,
                                            BRIDGE_QUARTER);
        double vout = vb * gain_actual;
        double stress = strain_to_stress_mpa(strains[i],
                                             gauge.youngs_modulus_gpa);
        printf("  %10.0f | %11.3f | %8.3f | %11.1f\n",
               strains[i], vb * 1000.0, vout, stress);
    }

    printf("\n==============================================================\n");
    printf("  Design Summary:\n");
    printf("  - IA Topology: 3-op-amp (AD620-like)\n");
    printf("  - Gain: %.2f (Rg = %.1f ohm)\n", gain_actual, rg_std);
    printf("  - Total RSS Error: %.1f uV RTI\n", budget.total_error_rss_uv);
    printf("  - Effective Resolution: %.1f bits\n",
           budget.effective_resolution_bits);
    printf("==============================================================\n");

    return 0;
}