/**
 * @file example_touch_button.c
 * @brief L6: Single capacitive touch button with LED indicator
 *
 * Demonstrates a complete touch detection pipeline:
 * 1. System initialization (self-cap, charge transfer)
 * 2. Baseline calibration (rapid initial settling)
 * 3. Continuous scanning with adaptive threshold
 * 4. Touch state machine with debounce
 * 5. LED toggle on touch/release
 *
 * This is the canonical "hello world" of capacitive sensing:
 * a single button that toggles an LED when touched.
 *
 * Application: Consumer electronics (iPhone home button concept),
 * industrial control panels, medical device interfaces (NHS).
 */

#include "cap_sense_core.h"
#include "cap_touch_detection.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(void)
{
    printf("=== Capacitive Touch Button Demo ===\n");
    printf("Simulating a single touch button with LED toggle.\n\n");

    /* L1: Initialize system with 1 channel */
    cap_sensor_system_t sys;
    if (cap_sensor_system_init(&sys, 1, 3.3, 10e-9) != 0) {
        printf("ERROR: System init failed\n");
        return 1;
    }

    /* L2: Configure the button channel */
    /* 10mm diameter disc, 1.5mm glass overlay, 100fF threshold */
    cap_channel_configure(&sys.channels[0],
        CAP_METHOD_CHARGE_TRANSFER,
        CAP_SENSE_SELF,
        ELEC_DISC,
        7.85e-5,    /* pi*(5mm)^2 = 78.5 mm^2 */
        1.5e-3,     /* 1.5mm glass */
        7.5,        /* glass eps_r */
        100.0e-15); /* 100 fF threshold */

    /* L5: Set up baseline tracker and detection config */
    cap_baseline_tracker_config_t bl_cfg;
    cap_baseline_tracker_config_init(&bl_cfg);

    cap_touch_detect_config_t td_cfg;
    cap_touch_detect_config_init(&td_cfg);
    td_cfg.threshold_sigma_mult = 5.0;  /* K=5, P_false~1e-7 */
    td_cfg.debounce_samples = 3;
    td_cfg.release_debounce = 2;

    /* L2: Simulate initial calibration scan */
    printf("Calibrating baseline...\n");
    double baseline_est = sys.channels[0].self_meas.c_baseline_f;
    printf("  Estimated baseline: %.3f pF\n", baseline_est * 1e12);

    /* L4: Report fundamental noise limit */
    double vn = cap_ktc_noise_voltage(300.0, baseline_est);
    printf("  kT/C noise floor: %.2f uVrms\n", vn * 1e6);

    double min_dc = cap_min_resolvable_delta_c(baseline_est, 300.0, 3.3, 5.0, 100);
    printf("  Min resolvable deltaC: %.3f fF (SNR=5, N=100)\n", min_dc * 1e15);
    printf("\n");

    /* L6: Simulate a touch event sequence */
    printf("Simulating touch sequence...\n");
    printf("Format: [time_ms] event\n\n");

    /* Simulated raw counts (baseline ~10000, touch ~10100 for 1% deltaC) */
    uint32_t raw_values[] = {
        10000, 10002, 9998, 10001,  /* idle, ~1% noise */
        10050, 10080, 10100,        /* approaching finger */
        10120, 10125, 10130,        /* touch detected */
        10130, 10128, 10131,        /* hold */
        10130, 10100, 10050,        /* releasing */
        10010, 10002, 10000         /* idle again */
    };
    int n_scans = sizeof(raw_values) / sizeof(raw_values[0]);

    uint32_t baseline = 10000;
    cap_sensor_channel_t *chan = &sys.channels[0];
    bool led_state = false;
    double var_est = 100.0;  /* variance estimate initial ~10^2 */
    double mean_est = 0.0;

    for (int i = 0; i < n_scans; i++) {
        uint32_t raw = raw_values[i];
        uint32_t t_ms = (uint32_t)(i * 10);  /* 10 ms per scan */

        /* L5: Update baseline with asymmetric EMA */
        bool touch_active = (chan->state == TOUCH_ACTIVE ||
                             chan->state == TOUCH_HOLD);
        baseline = cap_baseline_update_ema(baseline, raw, &bl_cfg, touch_active);

        /* L5: Compute delta */
        int32_t delta_count = cap_compute_delta(raw, baseline, CAP_SENSE_SELF);
        double delta_c = cap_delta_count_to_farad(delta_count, 10e-12, 16, 1.0);

        /* L5: Update noise estimate */
        cap_update_noise_estimate(&var_est, &mean_est, delta_c, 0.02, 0.01);
        double noise_rms = sqrt(var_est > 0.0 ? var_est : 1e-30);

        /* L5: Adaptive threshold (used by state machine internally) */
        double threshold = cap_adaptive_threshold(noise_rms,
            td_cfg.threshold_sigma_mult,
            td_cfg.min_threshold_f,
            td_cfg.max_threshold_f);
        (void)threshold;  /* Referenced by config, consumed by state machine */

        /* L6: Run touch state machine */
        cap_touch_event_t event;
        memset(&event, 0, sizeof(event));
        touch_state_t state = cap_touch_state_machine(chan, delta_c, &td_cfg,
                                                       &event, t_ms);

        /* Report state changes */
        if (state == TOUCH_DETECT && !led_state) {
            led_state = true;
            printf("[%4u ms] TOUCH DETECTED -> LED ON  (deltaC=%.1f fF, SNR=%.1f dB)\n",
                   t_ms, delta_c * 1e15,
                   20.0 * log10(delta_c / (noise_rms > 0 ? noise_rms : 1e-30)));
        } else if (state == TOUCH_RELEASE && led_state) {
            led_state = false;
            printf("[%4u ms] TOUCH RELEASED -> LED OFF  (deltaC=%.1f fF)\n",
                   t_ms, delta_c * 1e15);
        } else if (state == TOUCH_ACTIVE && (i == 0 || raw_values[i-1] < 10100)) {
            printf("[%4u ms] TOUCH ACTIVE             (deltaC=%.1f fF)\n",
                   t_ms, delta_c * 1e15);
        } else if (state == TOUCH_HOLD) {
            double pressure = cap_estimate_touch_pressure(delta_c, 500e-15, 100e-15);
            printf("[%4u ms] TOUCH HOLD  (pressure=%.2f)  (deltaC=%.1f fF)\n",
                   t_ms, pressure, delta_c * 1e15);
        }
    }

    /* L4: Report SNR for the touch event */
    double snr_touch = cap_snr_limit_db(200e-15, 3.3, baseline_est, 300.0, 3);
    printf("\nTouch SNR (3-sample avg): %.1f dB\n", snr_touch);

    printf("\n=== Demo complete ===\n");

    cap_sensor_system_destroy(&sys);
    return 0;
}
