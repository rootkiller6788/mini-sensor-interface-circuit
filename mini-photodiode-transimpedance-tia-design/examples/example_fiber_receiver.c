/* example_fiber_receiver.c - Fiber Optic Receiver TIA Design */
#include "tia_core.h"
#include "tia_advanced.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    printf("=== Fiber Optic Receiver TIA Design Example ===\n\n");

    /* 10 Gbps fiber receiver at 1550 nm */
    photodiode_model_t pd = photodiode_model_init(PHOTODIODE_INGAAS_PIN, 0.04, 3.0);
    opamp_params_t opa = opamp_params_init("OPA657");

    printf("Photodiode: %s (%.0fum dia), R=%.2f A/W\n",
           pd.model_name, sqrt(pd.active_area_mm2)*1000.0, pd.responsivity_a_per_w);
    printf("OpAmp:       %s, GBWP=%.0f MHz\n", opa.part_number, opa.gain_bandwidth_mhz);

    double target_gain = 1000.0;
    double target_bw_mhz = 8000.0;

    tia_design_t design = tia_design_basic(&pd, &opa, target_gain, target_bw_mhz * 1.0e6);

    printf("\n--- Design Results ---\n");
    printf("R_f        = %.0f ohm\n", design.rf_ohm);
    printf("C_f        = %.3f pF\n", design.cf_pf);
    printf("C_in       = %.1f pF\n", design.total_input_capacitance_pf);
    printf("Bandwidth  = %.1f MHz\n", design.bandwidth_3db_mhz);
    printf("Gain       = %.0f ohm (%.1f dB)\n", design.transimpedance_gain_ohm, design.transimpedance_gain_db);
    printf("Phase Margin = %.1f deg\n", design.phase_margin_deg);
    printf("Input noise  = %.1f pA/rtHz\n", design.input_noise_density_pa);

    fiber_receiver_spec_t spec;
    spec.data_rate_gbps = 10.0;
    spec.wavelength_nm = 1550.0;
    spec.fiber_length_km = 80.0;
    spec.fiber_attenuation_db_per_km = 0.2;
    spec.connector_loss_db = 2.0;
    spec.transmitter_power_dbm = 0.0;
    spec.required_ber = 1.0e-12;
    spec.dispersion_penalty_db = 2.0;
    snprintf(spec.modulation_format, 16, "NRZ-OOK");
    spec.extinction_ratio_db = 10.0;

    fiber_receiver_perf_t perf = tia_fiber_receiver_analyze(&design, &spec);

    printf("\n--- Link Budget ---\n");
    printf("Tx Power    = %.1f dBm\n", spec.transmitter_power_dbm);
    printf("Fiber Loss  = %.1f dB\n", spec.fiber_length_km * spec.fiber_attenuation_db_per_km);
    printf("Rx Power    = %.1f dBm\n", perf.received_power_dbm);
    printf("Sensitivity = %.1f dBm (BER=1e-12)\n", perf.sensitivity_dbm);
    printf("Link Margin = %.1f dB\n", perf.link_margin_db);
    printf("Expected BER= %.1e\n", perf.estimated_ber);
    printf("Eye Opening = %.1f%%\n", perf.eye_opening_percent);

    printf("\n--- Power Budget ---\n");
    if (perf.link_margin_db > 3.0) {
        printf("LINK OK - %.1f dB margin\n", perf.link_margin_db);
    } else if (perf.link_margin_db > 0.0) {
        printf("LINK MARGINAL - only %.1f dB margin\n", perf.link_margin_db);
    } else {
        printf("LINK FAIL - %.1f dB below sensitivity\n", -perf.link_margin_db);
    }

    return 0;
}
