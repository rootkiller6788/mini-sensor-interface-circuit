/* example_lidar_receiver.c - LIDAR ToF Receiver TIA Design */
#include "tia_core.h"
#include "tia_advanced.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    printf("=== LIDAR Time-of-Flight Receiver TIA Example ===\n\n");

    /* Automotive LIDAR: 905nm, 5ns pulses, 100m range */
    photodiode_model_t pd = photodiode_model_init(PHOTODIODE_SI_APD, 0.5, 150.0);
    opamp_params_t opa = opamp_params_init("ADA4817");

    printf("Photodiode: %s (APD, M~50), R=%.1f A/W\n",
           pd.model_name, pd.responsivity_a_per_w);
    printf("OpAmp:      %s, GBWP=%.0f MHz\n", opa.part_number, opa.gain_bandwidth_mhz);

    double target_gain = 2000.0;
    tia_design_t design = tia_design_basic(&pd, &opa, target_gain, 0.0);

    printf("\n--- TIA Design ---\n");
    printf("R_f        = %.0f ohm\n", design.rf_ohm);
    printf("Bandwidth  = %.1f MHz\n", design.bandwidth_3db_mhz);
    printf("Noise      = %.1f pA/rtHz\n", design.input_noise_density_pa);

    lidar_receiver_spec_t spec;
    spec.laser_wavelength_nm = 905.0;
    spec.pulse_energy_nj = 100.0;
    spec.pulse_width_ns = 5.0;
    spec.pulse_repetition_khz = 100.0;
    spec.target_range_m = 100.0;
    spec.target_reflectivity = 0.1;
    spec.aperture_diameter_mm = 25.0;
    spec.atmospheric_attenuation_db_per_km = 0.1;

    lidar_receiver_perf_t perf = tia_lidar_receiver_analyze(&design, &spec);

    printf("\n--- LIDAR Link Analysis ---\n");
    printf("Pulse Energy     = %.0f nJ\n", spec.pulse_energy_nj);
    printf("Pulse Width      = %.1f ns\n", spec.pulse_width_ns);
    printf("Target Range     = %.1f m\n", spec.target_range_m);
    printf("Peak Photocurrent= %.1f uA\n", perf.peak_photocurrent_ua);
    printf("Received Energy  = %.1f fJ\n", perf.received_pulse_energy_fj);
    printf("SNR              = %.1f dB\n", perf.snr_db);
    printf("Range Resolution = %.1f mm\n", perf.range_resolution_mm);
    printf("Max Range        = %.1f m\n", perf.max_detectable_range_m);
    printf("Detection Prob   = %.2f\n", perf.detection_probability);

    printf("\n--- Detection Assessment ---\n");
    if (perf.snr_db > 15.0) {
        printf("STRONG DETECTION: SNR=%.1f dB, reliable ranging\n", perf.snr_db);
    } else if (perf.snr_db > 10.0) {
        printf("ADEQUATE DETECTION: SNR=%.1f dB\n", perf.snr_db);
    } else {
        printf("WEAK DETECTION: SNR=%.1f dB, consider increasing pulse energy\n", perf.snr_db);
    }

    return 0;
}
