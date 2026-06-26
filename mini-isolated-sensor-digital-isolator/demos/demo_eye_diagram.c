/**
 * @file demo_eye_diagram.c
 * @brief L8: Advanced eye diagram demo with jitter decomposition
 *
 * Demonstrates advanced signal integrity analysis for isolated
 * data links including bathtub curve generation and BER extrapolation.
 */
#include "digital_isolator.h"
#include "isolator_signal_integrity.h"
#include <stdio.h>
#include <math.h>

int main(void) {
    printf("=== Eye Diagram & Jitter Demo (L8) ===

");

    /* Setup analyzer */
    signal_integrity_analyzer_t sia;
    si_analyzer_init(&sia, 10000.0, 50.0, 100000);

    /* Generate simulated waveform with Gaussian jitter */
    printf("Generating waveform with 2 ps RMS random jitter...
");
    double v[2000], t_ns[2000];
    for (int i = 0; i < 2000; i++) {
        t_ns[i] = i * 0.05;
        double jitter = 0.002 * ((i * 12345 + 6789) % 1000) / 500.0 - 0.001;
        double ideal = fmod(t_ns[i] + jitter, 10.0);
        v[i] = (ideal < 5.0) ? 3.3 : 0.0;
    }
    si_analyzer_feed_waveform(&sia, v, t_ns, 2000);

    /* Compute eye diagram */
    si_analyzer_compute_eye(&sia);
    printf("
Eye Diagram Results:
");
    printf("  Height:  %.3f V
", sia.eye.eye_height_v);
    printf("  Width:   %.3f UI
", sia.eye.eye_width_ui);
    printf("  Q:       %.2f
", sia.eye.q_factor);
    printf("  SNR:     %.1f dB
", sia.eye.snr_db);

    /* Decompose jitter */
    si_analyzer_decompose_jitter(&sia);
    printf("
Jitter Decomposition (Dual-Dirac Model):
");
    printf("  RJ (sigma): %.3f ps RMS
", sia.jitter.random_jitter_rms_ps);
    printf("  DJ (delta): %.3f ps PP
", sia.jitter.deterministic_jitter_pp_ps);
    printf("  DDJ:        %.3f ps PP
", sia.jitter.data_dependent_jitter_pp_ps);
    printf("  PJ:         %.3f ps PP
", sia.jitter.periodic_jitter_pp_ps);

    /* BER bathtub at different levels */
    printf("
BER Bathtub Curve:
");
    si_analyzer_extrapolate_ber(&sia, 1e-9);
    printf("  BER=1e-9:  %.3f UI opening
", sia.bathtub.eye_opening_at_1e12);
    si_analyzer_extrapolate_ber(&sia, 1e-12);
    printf("  BER=1e-12: %.3f UI opening
", sia.bathtub.eye_opening_at_1e12);
    si_analyzer_extrapolate_ber(&sia, 1e-15);
    printf("  BER=1e-15: %.3f UI opening
", sia.bathtub.eye_opening_at_1e15);

    /* Jitter transfer function */
    printf("
PLL Jitter Transfer Function:
");
    double freq[] = {1e3, 10e3, 100e3, 1e6, 10e6};
    for (int i = 0; i < 5; i++) {
        double jtf = si_jitter_transfer_function(freq[i], 1e6, 0.707);
        printf("  %.0f kHz: JTF = %.4f
", freq[i]/1000.0, jtf);
    }

    si_analyzer_destroy(&sia);
    printf("
=== Demo complete ===
");
    return 0;
}