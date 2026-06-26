/**
 * @file example_isolated_adc.c
 * @brief L7: Isolated 24-bit ADC with signal integrity analysis
 */
#include "digital_isolator.h"
#include "isolator_signal_integrity.h"
#include "isolated_adc_interface.h"
#include <stdio.h>
#include <math.h>

int main(void) {
    printf("=== Isolated ADC Signal Integrity Example (L7) ===

");

    digital_isolator_t iso;
    digital_isolator_init(&iso, ISOL_TECH_MAGNETIC, ISOL_CLASS_REINFORCED, 4);
    printf("Isolator: %s (magnetic, ADuM1401-class)
", iso.part_number);
    printf("  Data rate: %.0f Mbps
", iso.barrier.data_rate_mbps);
    printf("  Propagation delay: %.1f ns
", iso.barrier.propagation_delay_ns);
    printf("  CMTI: %.0f kV/us
", iso.barrier.cmti_kv_us);

    /* Signal integrity analyzer setup */
    signal_integrity_analyzer_t sia;
    double bit_period_ps = 10000.0; /* 100 Mbps */
    double sample_period_ps = 100.0;
    si_analyzer_init(&sia, bit_period_ps, sample_period_ps, 100000);

    /* Generate pseudo-waveform (simulated) */
    double voltages[500];
    double times_ns[500];
    for (int i = 0; i < 500; i++) {
        times_ns[i] = i * sample_period_ps / 1000.0;
        double phase = fmod(times_ns[i], bit_period_ps / 500.0);
        voltages[i] = 3.0 * (phase < (bit_period_ps / 1000.0) ? 1.0 : 0.0);
    }
    si_analyzer_feed_waveform(&sia, voltages, times_ns, 500);

    /* Compute eye diagram */
    si_analyzer_compute_eye(&sia);
    printf("
Eye Diagram Analysis:
");
    printf("  Eye height: %.3f V
", sia.eye.eye_height_v);
    printf("  Q-factor: %.2f
", sia.eye.q_factor);
    printf("  SNR: %.1f dB
", sia.eye.snr_db);

    /* BER estimation */
    double q = si_q_factor_from_eye(&sia);
    double ber = si_ber_from_q_factor(q);
    printf("  Estimated BER: %.2e
", ber);

    /* Jitter decomposition */
    si_analyzer_decompose_jitter(&sia);
    printf("
Jitter Analysis:
");
    printf("  RJ RMS: %.2f ps
", sia.jitter.random_jitter_rms_ps);
    printf("  DJ PP:  %.2f ps
", sia.jitter.deterministic_jitter_pp_ps);
    printf("  TJ PP:  %.2f ps
", sia.jitter.total_jitter_pp_ps);

    /* BER bathtub */
    si_analyzer_extrapolate_ber(&sia, 1e-12);
    printf("
Bathtub at BER=1e-12:
");
    printf("  Eye opening: %.3f UI
", sia.bathtub.eye_opening_at_1e12);

    /* Common-mode rejection */
    double cmrr = si_common_mode_rejection(1000.0, 60.0, 100.0);
    printf("
CM rejection at 60 Hz, 1kV: %.1f dB
", cmrr);

    si_analyzer_destroy(&sia);
    digital_isolator_destroy(&iso);
    printf("
=== Example complete ===
");
    return 0;
}