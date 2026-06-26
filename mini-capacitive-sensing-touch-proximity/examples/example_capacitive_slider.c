/**
 * @file example_capacitive_slider.c
 * @brief L7: Capacitive slider for volume control / dimmer
 *
 * Demonstrates a linear capacitive slider with position interpolation
 * and gesture detection. Maps finger position to a 0-100% output value.
 *
 * Application: Audio volume slider (smartphone, car audio - Toyota),
 * LED dimmer control, thermostat temperature adjustment,
 * industrial HMI position input.
 */

#include "cap_sense_core.h"
#include "cap_sensor_geometry.h"
#include "cap_touch_detection.h"
#include "cap_gesture_recognition.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

int main(void)
{
    printf("=== Capacitive Slider Demo ===\n");
    printf("8-segment linear slider with gesture detection.\n\n");

    /* L1: Initialize slider geometry */
    cap_slider_geometry_t slider;
    memset(&slider, 0, sizeof(slider));
    slider.num_segments = 8;
    slider.segment_width_m = 5.0e-3;   /* 5mm per segment */
    slider.segment_length_m = 20.0e-3; /* 20mm tall */
    slider.inter_segment_gap_m = 0.5e-3;
    slider.total_length_m = 8 * (5.0e-3 + 0.5e-3);  /* 44mm total */

    printf("Slider: %d segments, %.0f mm total length\n",
           8, slider.total_length_m * 1000.0);

    /* L5: Initialize gesture recognizer for tap/swipe */
    cap_gesture_recognizer_t recog;
    cap_gesture_recognizer_init(&recog);

    /* L6: Simulate finger sliding left to right */
    printf("\nSimulating finger slide left-to-right:\n");
    printf("Position(mm) | Output%% | Gesture\n");
    printf("-------------|---------|--------\n");

    /* Simulate 20 touch positions moving across slider */
    for (int step = 0; step < 20; step++) {
        /* Finger position moves from 2mm to 42mm */
        double finger_pos_mm = 2.0 + (double)step * 2.0;

        /* Compute which segments are affected (Gaussian profile) */
        memset(slider.delta_c_per_seg, 0, sizeof(slider.delta_c_per_seg));
        for (int s = 0; s < 8; s++) {
            double seg_center = (double)s * (5.0 + 0.5) + 2.5;  /* mm */
            double dist = finger_pos_mm - seg_center;
            /* Gaussian touch profile: sigma ~ 4mm */
            double delta_c = 200.0 * exp(-dist * dist / (2.0 * 16.0));
            if (delta_c < 1.0) delta_c = 0.0;
            slider.delta_c_per_seg[s] = delta_c;
        }

        /* L6: Interpolate position */
        double pos_mm = cap_slider_interpolate_position(&slider) * 1000.0;  /* to mm */
        double output_pct = (pos_mm / slider.total_length_m / 1000.0) * 100.0;
        if (output_pct < 0.0) output_pct = 0.0;
        if (output_pct > 100.0) output_pct = 100.0;

        /* L6: Feed to gesture recognizer */
        cap_touch_point_t pt;
        memset(&pt, 0, sizeof(pt));
        pt.x_mm = pos_mm;
        pt.y_mm = 0.0;
        pt.pressure = 0.5;
        pt.timestamp_ms = (uint32_t)(step * 50);

        cap_gesture_result_t gesture_result;
        bool recognized = cap_gesture_feed_point(&recog, &pt,
                                                  (uint32_t)(step * 50),
                                                  &gesture_result);

        const char *gesture_name = "";
        if (recognized) {
            gesture_name = cap_gesture_type_name(gesture_result.gesture);
        }

        printf("  %5.1f mm   |  %5.1f%%  | %s\n",
               pos_mm, output_pct, gesture_name);
    }

    /* Release */
    cap_gesture_result_t final_result;
    cap_gesture_feed_point(&recog, NULL, 1000, &final_result);
    printf("  RELEASE    |          | %s\n",
           cap_gesture_type_name(final_result.gesture));

    /* L3: Report crosstalk estimate between adjacent segments */
    double xtalk = cap_crosstalk_estimate(
        slider.segment_length_m,
        slider.segment_width_m,
        slider.inter_segment_gap_m,
        4.5);
    printf("\nAdjacent segment crosstalk: %.3f pF\n", xtalk * 1e12);
    /* SNR from crosstalk: if deltaC=200fF per segment and xtalk=5pF,
     * the crosstalk signal is ~200fF*(5/50) = 20fF on neighbor, still OK */

    printf("\n=== Demo complete ===\n");
    return 0;
}
