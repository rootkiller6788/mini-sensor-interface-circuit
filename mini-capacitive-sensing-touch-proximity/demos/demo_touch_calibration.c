/**
 * @file demo_touch_calibration.c
 * @brief L8: Touch calibration and sensitivity demo with visualization
 *
 * Interactive-style demo that walks through the touch calibration process,
 * showing baseline convergence, noise floor measurement, and adaptive
 * threshold adjustment. Simulates manufacturing test station.
 *
 * Advanced topic: Statistical process control (Cpk) for production testing.
 * Application: Toyota touch panel quality assurance, iPhone assembly line.
 */
#include "cap_sense_core.h"
#include "cap_touch_detection.h"
#include "cap_sensor_geometry.h"
#include "cap_noise_immunity.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

int main(void)
{
    printf("========================================\n");
    printf("  Capacitive Touch Calibration Demo\n");
    printf("  Production Test Station Simulation\n");
    printf("========================================\n\n");

    /* Phase 1: Hardware check */
    printf("--- Phase 1: Hardware Integrity Check ---\n");
    cap_sensor_system_t sys;
    cap_sensor_system_init(&sys, 4, 3.3, 10e-9);

    /* Configure 4 buttons with different sizes */
    double areas[] = {7.85e-5, 1.26e-4, 1.77e-4, 3.14e-4}; /* 10-20mm dia */
    const char *names[] = {"BTN1", "BTN2", "BTN3", "PROX"};

    for (int i = 0; i < 4; i++) {
        cap_channel_configure(&sys.channels[i],
            CAP_METHOD_CHARGE_TRANSFER, CAP_SENSE_SELF,
            ELEC_DISC, areas[i], 2.0e-3, 7.5, 100e-15);
    }

    /* Parasitic check */
    for (int i = 0; i < 4; i++) {
        double c_par = sys.channels[i].self_meas.c_baseline_f;
        printf("  %s: area=%.0fmm2  C_par=%.2fpF",
               names[i], areas[i]*1e6, c_par*1e12);
        if (c_par > 50e-12) {
            printf("  WARNING: High parasitic (>50pF)\n");
        } else {
            printf("  OK\n");
        }
    }

    /* Phase 2: Noise floor measurement */
    printf("\n--- Phase 2: Noise Floor Measurement ---\n");
    cap_noise_profile_t profiles[4];
    double noise_samples[200];

    for (int ch = 0; ch < 4; ch++) {
        /* Simulate noise sampling (1% of baseline as RMS) */
        double c_base = sys.channels[ch].self_meas.c_baseline_f;
        for (int i = 0; i < 200; i++) {
            noise_samples[i] = c_base * (1.0 + (sin(i*0.3) * 0.01));
        }
        cap_noise_profile_t *p = &profiles[ch];
        cap_noise_profile_init(p);
        cap_analyze_noise(p, noise_samples, 200, 100.0);

        printf("  %s: RMS=%.2ffF  p-p=%.2ffF  SNR=%.1fdB (for 100fF touch)\n",
               names[ch], p->noise_rms_f*1e15, p->noise_peak_peak_f*1e15,
               20.0*log10(100e-15 / (p->noise_rms_f > 0 ? p->noise_rms_f : 1e-30)));
    }

    /* Phase 3: Sensitivity calibration */
    printf("\n--- Phase 3: Sensitivity Calibration ---\n");
    printf("  Simulating metal finger (12mm dia) on each button...\n");

    double finger_results[4];
    for (int ch = 0; ch < 4; ch++) {
        double c_ideal = cap_finger_electrode_c(1.13e-4, 2.0e-3, 7.5, 1.2);
        double c_actual = c_ideal * (0.8 + (double)(ch) * 0.1);
        finger_results[ch] = c_actual;
        printf("  %s: deltaC=%.1ffF  threshold=%.1ffF  margin=%.1f%%\n",
               names[ch], c_actual*1e15, c_actual*0.3*1e15,
               (c_actual - c_actual*0.3)/c_actual*100.0);
    }

    /* Phase 4: Manufacturing Cpk */
    printf("\n--- Phase 4: Capability Analysis (Cpk) ---\n");
    double delta_min = finger_results[0];
    double delta_max = delta_min;
    for (int i = 1; i < 4; i++) {
        if (finger_results[i] < delta_min) delta_min = finger_results[i];
        if (finger_results[i] > delta_max) delta_max = finger_results[i];
    }
    double delta_mean = (delta_min + delta_max) / 2.0;
    double sigma_process = (delta_max - delta_min) / 6.0; /* Range/6 estimate */
    double usl = delta_mean * 1.5; /* Upper spec: 50% above mean */
    double lsl = profiles[0].noise_rms_f * 10.0; /* Lower spec: 10x noise */

    double cpk = fmin(
        (delta_mean - lsl) / (3.0 * sigma_process),
        (usl - delta_mean) / (3.0 * sigma_process)
    );

    printf("  deltaC mean: %.1f fF\n", delta_mean*1e15);
    printf("  deltaC sigma: %.1f fF\n", sigma_process*1e15);
    printf("  Spec range: [%.1f, %.1f] fF\n", lsl*1e15, usl*1e15);
    printf("  Cpk: %.2f %s\n", cpk, cpk >= 1.33 ? "PASS (>= 1.33)" : "FAIL (< 1.33)");

    /* Phase 5: Guard ring design for PROX channel */
    printf("\n--- Phase 5: Guard Ring Optimization ---\n");
    cap_guard_ring_design_t grd;
    memset(&grd, 0, sizeof(grd));
    grd.electrode_width_m = sqrt(areas[3]);
    grd.guard_ring_width_m = 1.5e-3;
    grd.guard_gap_m = 0.5e-3;
    grd.pcb_thickness_m = 1.6e-3;
    grd.pcb_epsilon_r = 4.5;

    cap_guard_ring_design_analyze(&grd);
    printf("  PROX electrode: %.1fmm sq\n", grd.electrode_width_m*1000);
    printf("  C_par unguarded: %.2fpF\n", grd.c_parasitic_unguarded_f*1e12);
    printf("  C_par guarded: %.2fpF\n", grd.c_parasitic_guarded_f*1e12);
    printf("  Reduction ratio: %.1fx\n", grd.reduction_ratio);

    printf("\n========================================\n");
    printf("  Calibration Complete\n");
    printf("========================================\n");

    cap_sensor_system_destroy(&sys);
    return 0;
}
