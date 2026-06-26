/* example_spectrophotometer.c - Spectrophotometer Photodiode Frontend */
#include "tia_core.h"
#include "tia_advanced.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    printf("=== Spectrophotometer Photodiode Frontend Example ===\n\n");

    /* Visible spectrophotometer: 400-700nm, Si photodiode */
    photodiode_model_t pd = photodiode_model_init(PHOTODIODE_SI_PIN, 5.0, 0.0);
    opamp_params_t opa = opamp_params_init("OPA380");

    printf("Photodiode: %s, area=%.0f mm2, R=%.2f A/W\n",
           pd.model_name, pd.active_area_mm2, pd.responsivity_a_per_w);
    printf("OpAmp:      %s, GBWP=%.0f MHz, CMOS precision\n", opa.part_number, opa.gain_bandwidth_mhz);

    /* Low-noise, high-gain for weak optical signals */
    double target_gain = 1.0e6;
    tia_design_t design = tia_design_basic(&pd, &opa, target_gain, 0.0);

    printf("\n--- Low-Noise TIA Design ---\n");
    printf("R_f        = %.0f kOhm\n", design.rf_ohm / 1000.0);
    printf("C_f        = %.3f pF\n", design.cf_pf);
    printf("Bandwidth  = %.1f kHz\n", design.bandwidth_3db_hz / 1000.0);
    printf("Noise      = %.2f pA/rtHz\n", design.input_noise_density_pa);
    printf("Sensitivity= %.1f dBm\n", design.sensitivity_dbm);

    spectrometer_spec_t spec;
    spec.wavelength_start_nm = 400.0;
    spec.wavelength_stop_nm = 700.0;
    spec.wavelength_step_nm = 5.0;
    spec.optical_power_nw = 10.0;
    spec.integration_time_ms = 100.0;
    spec.dark_subtraction_enabled = 1.0;

    spectrometer_measurement_t meas = tia_spectrometer_measure(&design, &spec);

    printf("\n--- Spectrophotometer Scan ---\n");
    printf("Wavelength range: %.0f - %.0f nm\n", spec.wavelength_start_nm, spec.wavelength_stop_nm);
    printf("Step: %.1f nm, Points: %zu\n", spec.wavelength_step_nm, meas.num_points);
    printf("SNR range: %.1f - %.1f dB\n", meas.snr_min_db, meas.snr_max_db);
    printf("Stray light: %.2f%%\n", meas.stray_light_percent);

    printf("\nSample data points:\n");
    printf("  WL(nm)  I_photo(uA)  Transmittance  Absorbance\n");
    for (size_t i = 0; i < meas.num_points; i += (meas.num_points / 8 + 1)) {
        printf("  %6.1f  %10.4f  %13.4f  %10.4f\n",
               meas.wavelength_nm[i], meas.photocurrent_ua[i],
               meas.transmittance[i], meas.absorbance[i]);
    }

    printf("\n--- Design Assessment ---\n");
    double nepo = design.total_input_noise_pa * 1.0e-12 / pd.responsivity_a_per_w;
    printf("System NEP = %.1e W/rtHz\n", nepo);
    printf("Min detectable power (1s) = %.1e W\n", nepo * sqrt(1.0));

    if (design.bandwidth_3db_hz > 100.0) {
        printf("Bandwidth sufficient for scanning applications.\n");
    } else {
        printf("Bandwidth limited; suitable for slow-scan or fixed-wavelength.\n");
    }

    spectrometer_measurement_free(&meas);
    return 0;
}
