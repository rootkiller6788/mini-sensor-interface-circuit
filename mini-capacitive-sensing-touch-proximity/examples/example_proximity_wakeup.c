/**
 * @file example_proximity_wakeup.c
 * @brief L7: Proximity wake-up for battery-powered devices
 *
 * Implements a proximity wake-up system similar to smartphone "raise to wake"
 * or laptop "presence detection". The sensor operates in low-power mode,
 * scanning slowly until a body is detected, then waking the system.
 *
 * Application: Smartphone display wake-up, laptop lid-close detection,
 * automotive keyless entry approach detection (Toyota Smart Key concept).
 */

#include "cap_sense_core.h"
#include "cap_proximity_sense.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

int main(void)
{
    printf("=== Proximity Wake-Up Demo ===\n");
    printf("Low-power capacitive proximity for battery devices.\n\n");

    /* L1: Initialize system with a large proximity electrode */
    cap_sensor_system_t sys;
    if (cap_sensor_system_init(&sys, 1, 5.0, 1e-9) != 0) {
        printf("ERROR: System init failed\n");
        return 1;
    }

    /* Large electrode for long range: 20cm x 10cm = 0.02 m^2 */
    cap_channel_configure(&sys.channels[0],
        CAP_METHOD_SIGMA_DELTA,
        CAP_SENSE_SELF,
        ELEC_RECT,
        0.02,       /* 20x10 cm */
        3.0e-3,     /* 3mm plastic housing */
        3.5,        /* ABS plastic epsilon_r */
        5.0e-15);   /* 5 fF proximity threshold */

    /* L2: Configure proximity sensing */
    cap_proximity_config_t prox_cfg;
    cap_proximity_config_init(&prox_cfg, 0.02, 0.30);  /* 30cm max range */

    /* L3: Set up range estimation model */
    cap_range_model_t range_model;
    range_model.c0_f = 2.0e-12;   /* ~2pF at contact */
    range_model.r0_m = 3.0e-3;    /* 3mm overlay = reference distance */
    range_model.exponent_n = 2.5; /* Intermediate field */

    cap_proximity_state_t prox_state;
    cap_proximity_state_reset(&prox_state);

    /* L4: Report fundamental range limit */
    printf("Electrode area: %.0f cm^2\n", 0.02 * 10000.0);
    printf("Max range: %.0f cm\n", 0.30 * 100.0);
    printf("Resolution needed at 30cm: ");
    double dc_at_max = cap_delta_c_at_range(0.30, &range_model);
    printf("%.3f fF\n", dc_at_max * 1e15);

    double min_area = cap_min_electrode_area_for_range(0.30, 5e-15, 5.0, 3.5);
    printf("Min electrode area for 30cm: %.0f cm^2\n", min_area * 10000.0);
    printf("\n");

    /* L6: Simulate approach sequence */
    printf("Simulating body approach from 50cm to contact:\n");
    printf("Distance(cm) | deltaC(fF) | Zone      | Action\n");
    printf("-------------|------------|-----------|-------\n");

    struct {
        double distance_cm;
        const char *expected_zone;
    } approach[] = {
        {50.0, "FAR"},
        {30.0, "MEDIUM"},
        {20.0, "MEDIUM"},
        {10.0, "NEAR"},
        {5.0,  "NEAR"},
        {2.0,  "CONTACT"},
        {0.5,  "CONTACT"}
    };

    double smoothed_dc = 0.0;
    bool system_awake = false;

    for (int i = 0; i < 7; i++) {
        double r_m = approach[i].distance_cm / 100.0;
        double dc_true = cap_delta_c_at_range(r_m, &range_model);

        /* Add some noise */
        double dc_meas = dc_true + (rand() % 100 - 50) * 0.01e-15;

        /* L5: Smooth the measurement */
        cap_proximity_smooth_delta(&smoothed_dc, dc_meas, 0.2, 0.1);

        /* L6: Determine zone */
        proximity_zone_t zone = cap_determine_proximity_zone(
            smoothed_dc, &prox_cfg, &prox_state, (uint32_t)(1000 + i * 200));

        /* Range estimate (for debug/display) */
        double r_est = cap_estimate_range_power_law(smoothed_dc,
            &range_model, 0.50);
        (void)r_est;  /* Used implicitly via zone classification */

        /* Action based on zone */
        const char *action = "Sleep";
        if (zone >= PROX_ZONE_NEAR && !system_awake) {
            system_awake = true;
            action = "WAKE UP!";
        } else if (zone >= PROX_ZONE_CONTACT && system_awake) {
            action = "Touch ready";
        } else if (zone <= PROX_ZONE_MEDIUM && system_awake && i > 4) {
            /* Not applicable here */
        }

        printf("  %5.1f cm   |  %7.3f fF | %-9s | %s\n",
               approach[i].distance_cm, smoothed_dc * 1e15,
               (zone == PROX_ZONE_FAR ? "FAR" :
                zone == PROX_ZONE_MEDIUM ? "MEDIUM" :
                zone == PROX_ZONE_NEAR ? "NEAR" : "CONTACT"),
               action);
    }

    printf("\nSystem %s\n", system_awake ? "AWAKE" : "SLEEPING");
    printf("=== Demo complete ===\n");

    cap_sensor_system_destroy(&sys);
    return 0;
}
