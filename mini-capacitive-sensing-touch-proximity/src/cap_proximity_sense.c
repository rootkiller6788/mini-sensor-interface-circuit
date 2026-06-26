/**
 * @file cap_proximity_sense.c
 * @brief Proximity Sensing: Range Estimation, Zone Detection, SAR
 *
 * Proximity sensing detects a human body before physical contact occurs.
 * The key physical mechanism is the same as touch sensing (capacitive
 * coupling of human body), but with much smaller capacitance changes
 * (aF to low fF range) due to 1/r^n field decay.
 *
 * Knowledge Coverage:
 *   L1: proximity zones, detection range, approach speed
 *   L2: self-cap proximity, range vs electrode size tradeoff
 *   L3: dipole approximation, inverse power law, image charge method
 *   L4: Coulomb inverse-square law, method of images
 *   L5: range estimation, zone classification, adaptive scan rate
 *   L6: SAR-compliant proximity sensor, display wake-up, approach detection
 *
 * The electric field from an electrode decays with distance depending on
 * the ratio of distance to electrode size:
 * - Near field (r < electrode_radius): E(r) approximately constant
 * - Intermediate (r ~ electrode_radius): E(r) ~ 1/r^2 to 1/r^3
 * - Far field (r >> electrode_radius): E(r) ~ 1/r^3 (dipole)
 *
 * For a body that couples to earth ground, the effective field at large
 * distances transitions to 1/r^2 (monopole), increasing range.
 *
 * Ref: Baxter (1997) "Capacitive Sensors" Ch. 6
 *      IEEE C95.1-2019 SAR limits for RF exposure
 */

#include "cap_proximity_sense.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ==========================================================================
 * L1-L2: CONFIGURATION
 * ========================================================================== */

/**
 * cap_proximity_config_init
 *
 * Initializes proximity sensing configuration with size-dependent defaults.
 *
 * Detection range for self-capacitance scales approximately as:
 *   range_max ≈ 0.5 * sqrt(electrode_area)
 *
 * For a 10 × 10 cm electrode: range_max ≈ 0.5 * 0.1 = 5 cm
 * For a 20 × 20 cm electrode: range_max ≈ 0.5 * 0.2 = 10 cm
 *
 * The thresholds are set based on the estimated deltaC at each range
 * using the inverse power law with n=2.5 (intermediate field).
 *
 * For an electrode with C_contact ≈ 1 pF at touch:
 *   At 1 cm: deltaC ≈ 1pF * (1mm/1cm)^2.5 ≈ 1pF * 0.00316 ≈ 3.16 fF
 *   At 5 cm: deltaC ≈ 1pF * (1mm/5cm)^2.5 ≈ 1pF * 0.000178 ≈ 0.178 fF
 *   At 10 cm: deltaC ≈ 1pF * (1mm/10cm)^2.5 ≈ 1pF * 0.0000316 ≈ 0.032 fF
 *
 * For reliable detection at 5 cm, we need resolution ~0.1 fF, which
 * requires a sigma-delta CDC with OSR >= 256 and good shielding.
 */
void cap_proximity_config_init(cap_proximity_config_t *cfg,
                               double electrode_area, double max_range)
{
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));

    cfg->electrode_area_m2 = electrode_area;
    cfg->electrode_diameter_m = 2.0 * sqrt(electrode_area / M_PI);
    cfg->max_range_m = max_range;

    /* Compute zone thresholds using power-law model */
    /* Estimate C0 (deltaC at contact ≈ d_overlay):
     * C0 ≈ eps0 * eps_r_glass * A_electrode / d_overlay */
    double d_overlay = 0.001; /* 1 mm glass */
    double eps_r_glass = 7.5;
    double c0_est = CAP_EPSILON0 * eps_r_glass * electrode_area / d_overlay;
    /* Empirical derating (finger covers only part of electrode, non-ideal coupling) */
    c0_est *= 0.3;

    /* Compute deltaC at various ranges using n=2.5 */
    double n = 2.5;
    double r0 = d_overlay;

    /* FAR->MEDIUM: at max_range * 0.5 */
    double r_medium = max_range * 0.5;
    if (r_medium < r0) r_medium = r0 * 2.0;
    cfg->medium_threshold_f = c0_est * pow(r0 / r_medium, n);
    if (cfg->medium_threshold_f < 1.0e-15) cfg->medium_threshold_f = 1.0e-15;

    /* MEDIUM->NEAR: at ~5 cm or max_range * 0.15 */
    double r_near = max_range * 0.15;
    if (r_near < r0 * 10.0) r_near = r0 * 10.0;
    cfg->near_threshold_f = c0_est * pow(r0 / r_near, n);
    if (cfg->near_threshold_f < 10.0e-15) cfg->near_threshold_f = 10.0e-15;

    /* NEAR->CONTACT: at ~1 cm or 10*r0 */
    double r_contact = r0 * 10.0;
    cfg->far_threshold_f = c0_est * pow(r0 / r_contact, n) * 0.5;
    if (cfg->far_threshold_f < 0.5e-15) cfg->far_threshold_f = 0.5e-15;

    cfg->scan_period_slow_ms = 100.0;  /* 10 Hz in FAR zone */
    cfg->scan_period_fast_ms = 10.0;   /* 100 Hz in NEAR zone */
    cfg->debounce_zone_samples = 3;
    cfg->sar_enabled = false;
    cfg->sar_range_m = max_range * 0.3; /* SAR trigger at 30% of max range */
}

/* ==========================================================================
 * L3: RANGE ESTIMATION — INVERSE POWER LAW
 * ========================================================================== */

/**
 * cap_estimate_range_power_law
 *
 * Estimates the distance to an approaching body using the inverse power law:
 *
 *   deltaC = C0 * (r0 / r)^n
 *   → r = r0 * (C0 / deltaC)^(1/n)
 *
 * The exponent n depends on the field regime:
 *   n ≈ 1: Very near field (electrode looks like parallel plate, rare)
 *   n ≈ 2: Intermediate (monopole-like, body couples well to ground)
 *   n ≈ 3: Far field (dipole-like, electrode-image pair)
 *
 * In practice, n is between 2.0 and 3.0. We fit n empirically during
 * calibration by measuring deltaC at two known distances.
 *
 * The estimate degrades at very small deltaC (noise-dominated) and
 * very large deltaC (near contact, where the model breaks down).
 * We clamp to [0, max_range].
 *
 * @param delta_c_f    Measured delta capacitance [F]
 * @param model        Range model (C0, r0, n)
 * @param max_range_m  Maximum reportable range [m]
 * @return Estimated range [m]
 */
double cap_estimate_range_power_law(double delta_c_f,
                                    const cap_range_model_t *model,
                                    double max_range_m)
{
    if (!model || delta_c_f <= 0.0 || model->c0_f <= 0.0 ||
        model->r0_m <= 0.0 || model->exponent_n <= 0.0) {
        return max_range_m;
    }

    /* Signal too small to distinguish from noise → assume at max range */
    if (delta_c_f < model->c0_f * 1e-6) {
        return max_range_m;
    }

    /* Signal larger than contact reference → at or closer than r0 */
    if (delta_c_f >= model->c0_f) {
        return model->r0_m * 0.5; /* Closer than reference, likely touching */
    }

    double ratio = model->c0_f / delta_c_f;
    double r = model->r0_m * pow(ratio, 1.0 / model->exponent_n);

    if (r > max_range_m) r = max_range_m;
    if (r < 0.0) r = 0.0;
    return r;
}

/**
 * cap_delta_c_at_range
 *
 * Forward model: predicts the deltaC at a given distance.
 *
 *   deltaC = C0 * (r0 / r)^n
 *
 * Used for threshold calculation and expected signal verification.
 */
double cap_delta_c_at_range(double r_m, const cap_range_model_t *model)
{
    if (!model || r_m <= 0.0 || model->c0_f <= 0.0 ||
        model->r0_m <= 0.0 || model->exponent_n <= 0.0) {
        return 0.0;
    }

    if (r_m < model->r0_m) {
        return model->c0_f; /* Clamp at contact value */
    }

    return model->c0_f * pow(model->r0_m / r_m, model->exponent_n);
}

/* ==========================================================================
 * L6: PROXIMITY ZONE DETECTION
 * ========================================================================== */

/**
 * cap_determine_proximity_zone
 *
 * Classifies the current deltaC into a proximity zone with debounce.
 *
 * Zone thresholds (ascending deltaC):
 *   FAR:     deltaC < far_threshold        → body not detected
 *   MEDIUM:  far <= deltaC < medium       → body somewhere in range
 *   NEAR:    medium <= deltaC < near       → body close, prepare
 *   CONTACT: deltaC >= near               → body nearly touching
 *
 * Zone changes are debounced: the signal must stay in the new zone
 * for debounce_zone_samples consecutive measurements before the
 * transition is recognized. This prevents flicker between zones.
 *
 * The scan rate can be adapted: slow (10 Hz) in FAR, fast (100+ Hz)
 * in NEAR/CONTACT for responsive touch detection.
 */
proximity_zone_t cap_determine_proximity_zone(double delta_c_f,
                                              const cap_proximity_config_t *cfg,
                                              cap_proximity_state_t *state,
                                              uint32_t now_ms)
{
    if (!cfg || !state) return PROX_ZONE_FAR;

    /* Determine raw zone from thresholds */
    proximity_zone_t raw_zone;
    if (delta_c_f >= cfg->near_threshold_f) {
        raw_zone = PROX_ZONE_CONTACT;
    } else if (delta_c_f >= cfg->medium_threshold_f) {
        raw_zone = PROX_ZONE_NEAR;
    } else if (delta_c_f >= cfg->far_threshold_f) {
        raw_zone = PROX_ZONE_MEDIUM;
    } else {
        raw_zone = PROX_ZONE_FAR;
    }

    /* Debounce: count consecutive samples in new zone */
    if (raw_zone != state->current_zone) {
        if (state->far_zone_samples == 0 && state->near_zone_samples == 0) {
            /* Start tracking new zone */
            if (raw_zone >= PROX_ZONE_NEAR) {
                state->near_zone_samples = 1;
            } else {
                state->far_zone_samples = 1;
            }
        }

        /* Increment appropriate counter */
        if (raw_zone >= PROX_ZONE_MEDIUM) {
            state->near_zone_samples++;
            state->far_zone_samples = 0;
        } else {
            state->far_zone_samples++;
            state->near_zone_samples = 0;
        }

        /* Check if debounce threshold reached */
        uint32_t *counter = (raw_zone >= PROX_ZONE_MEDIUM) ?
                            &state->near_zone_samples : &state->far_zone_samples;

        if (*counter >= cfg->debounce_zone_samples) {
            state->prev_zone = state->current_zone;
            state->current_zone = raw_zone;
            state->zone_entry_time_ms = now_ms;
            *counter = 0;
        }
    } else {
        /* Reset counters when staying in current zone */
        state->far_zone_samples = 0;
        state->near_zone_samples = 0;
    }

    /* SAR trigger logic */
    if (cfg->sar_enabled) {
        if (state->current_zone <= PROX_ZONE_MEDIUM) {
            /* Body is within SAR trigger range → reduce RF power */
            state->sar_triggered = true;
        } else if (state->current_zone == PROX_ZONE_FAR) {
            state->sar_triggered = false;
        }
    }

    return state->current_zone;
}

/**
 * cap_proximity_smooth_delta
 *
 * Applies asymmetric low-pass filtering to deltaC for stable range estimation.
 *
 * When a body approaches, deltaC increases. We want fast response to
 * detect the approach quickly. When the body recedes, deltaC decreases.
 * We filter more heavily to prevent the range estimate from jumping
 * around due to body movement variability.
 *
 * Asymmetric smoothing:
 *   if deltaC increasing: alpha = alpha_approach (faster, typ 0.1-0.3)
 *   if deltaC decreasing: alpha = alpha_recede (slower, typ 0.05-0.1)
 *
 * This provides hysteresis in the range estimate, preventing flicker
 * when the body is at a zone boundary.
 */
void cap_proximity_smooth_delta(double *smoothed, double new_delta_c,
                                double alpha_approach, double alpha_recede)
{
    if (!smoothed) return;

    double alpha;
    if (new_delta_c > *smoothed) {
        alpha = alpha_approach;
    } else {
        alpha = alpha_recede;
    }

    /* Clamp alpha to [0,1] */
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;

    *smoothed = alpha * new_delta_c + (1.0 - alpha) * (*smoothed);
}

/* ==========================================================================
 * L5: APPROACH SPEED ESTIMATION
 * ========================================================================== */

/**
 * cap_estimate_approach_speed
 *
 * Estimates the speed of an approaching body from the rate of change
 * of delta capacitance.
 *
 * Using the inverse power law: deltaC = C0 * (r0/r)^n
 * Taking the derivative with respect to time:
 *
 *   d(deltaC)/dt = -n * C0 * r0^n * r^(-n-1) * dr/dt
 *   dr/dt = -d(deltaC)/dt * r^(n+1) / (n * C0 * r0^n)
 *
 * Or: dr/dt = -(d(deltaC)/dt) * r / (n * deltaC)
 *
 * Approximating d(deltaC)/dt with finite difference:
 *   v_approach = -((C2 - C1)/dt) * r_est / (n * C_avg)
 *
 * Positive v_approach = body approaching (range decreasing).
 *
 * @param delta_c_f      Current deltaC [F]
 * @param delta_c_prev_f Previous deltaC [F]
 * @param dt_s           Time between samples [s]
 * @param model          Range model (for exponent n)
 * @return Approach speed [m/s], positive = approaching
 */
double cap_estimate_approach_speed(double delta_c_f, double delta_c_prev_f,
                                   double dt_s, const cap_range_model_t *model)
{
    if (!model || dt_s <= 0.0 || model->exponent_n <= 0.0) return 0.0;

    double dc_dt = (delta_c_f - delta_c_prev_f) / dt_s;
    double c_avg = (delta_c_f + delta_c_prev_f) / 2.0;

    if (c_avg <= 0.0) return 0.0;

    /* Estimate current range */
    double r_est = cap_estimate_range_power_law(c_avg, model, 1.0);

    /* v = -dr/dt = (dc/dt) * r / (n * C)   (note sign: dc/dt > 0 when approaching) */
    double v = dc_dt * r_est / (model->exponent_n * c_avg);

    return v;
}

/* ==========================================================================
 * L3: DIPOLE MODEL
 * ========================================================================== */

/**
 * cap_compute_dipole_model
 *
 * Models the sensor electrode and its ground-plane image as an electric
 * dipole. This is the simplest model that captures the 1/r^3 far-field
 * decay characteristic of capacitive proximity sensors.
 *
 * Dipole moment: p = Q * d = C * V * d
 * where d is the electrode-to-image separation (approx 2 * pcb_thickness
 * for electrodes with ground plane on bottom layer).
 *
 * Field along the axis at distance r:
 *   E(r) = (2 * p) / (4 * pi * eps0 * (r^2 + d^2)^(3/2))
 *
 * For r >> d: E(r) ≈ 2p / (4*pi*eps0 * r^3) → 1/r^3 decay.
 *
 * @param model      Dipole model to populate
 * @param radius_m   Electrode effective radius [m]
 * @param voltage_v  Electrode voltage [V]
 * @param c_self_f   Electrode self-capacitance [F]
 */
void cap_compute_dipole_model(cap_dipole_model_t *model, double radius_m,
                              double voltage_v, double c_self_f)
{
    if (!model) return;
    memset(model, 0, sizeof(*model));

    model->electrode_radius_m = radius_m;
    /* Image distance: ~2x the distance to ground plane.
     * For a 1.6 mm PCB, d ≈ 2 * 1.6e-3 = 3.2e-3 m */
    model->image_distance_m = 2.0 * 0.0016; /* Assume standard 1.6mm PCB */

    /* Charge on electrode: Q = C * V */
    model->charge_c = c_self_f * voltage_v;

    /* Dipole moment: p = Q * d */
    model->dipole_moment_cm = model->charge_c * model->image_distance_m;

    /* Field at 1m (reference): */
    double r = 1.0;
    double d = model->image_distance_m;
    double denominator = 4.0 * M_PI * CAP_EPSILON0 * pow(r * r + d * d, 1.5);
    if (denominator > 0.0) {
        model->field_at_1m_v_m = 2.0 * model->dipole_moment_cm / denominator;
    }
}

/**
 * cap_min_electrode_area_for_range
 *
 * Computes the minimum electrode area needed to achieve a desired
 * detection range with a given deltaC threshold.
 *
 * From the dipole field model, the deltaC at range r is approximately:
 *
 *   deltaC(r) ≈ k * eps0 * eps_r_overlay * A_electrode * V_exc / (r^3 * V_ref)
 *
 * where k is a geometric factor (empirically ~0.5-2.0 for disc electrodes).
 *
 * Solving for area:
 *   A_min = deltaC_threshold * r^3 * V_ref / (k * eps0 * eps_r * V_exc)
 *
 * This is an approximate model; actual range depends on:
 * - Body coupling to earth ground (better ground → longer range)
 * - Environmental noise floor
 * - Electrode shape (disc vs rectangle)
 * - Ground plane configuration
 *
 * Example: r=0.1m, deltaC_thresh=1fF, V_exc=3.3V, eps_overlay=7.5:
 *   A_min ≈ 1e-15 * 0.001 * 3.3 / (1.0 * 8.85e-12 * 7.5 * 3.3)
 *         ≈ 3.3e-18 / 2.19e-10 ≈ 1.5e-8 m² ≈ 0.15 cm²
 *
 * In practice, this is optimistic; 10x larger area is recommended.
 */
double cap_min_electrode_area_for_range(double desired_range_m,
                                        double threshold_f,
                                        double v_exc, double overlay_er)
{
    if (desired_range_m <= 0.0 || threshold_f <= 0.0 ||
        v_exc <= 0.0 || overlay_er < 1.0) {
        return 0.0;
    }

    double k = 1.5; /* Geometric factor for disc electrodes */
    double r3 = desired_range_m * desired_range_m * desired_range_m;

    double numerator = threshold_f * r3;
    double denominator = k * CAP_EPSILON0 * overlay_er * v_exc;

    if (denominator <= 0.0) return 0.0;

    double area = numerator / denominator;

    /* Apply safety factor 5x for real-world conditions */
    return area * 5.0;
}

/**
 * cap_proximity_state_reset
 *
 * Resets proximity state to initial values (FAR zone, zero speed).
 */
void cap_proximity_state_reset(cap_proximity_state_t *state)
{
    if (!state) return;
    memset(state, 0, sizeof(*state));
    state->current_zone = PROX_ZONE_FAR;
    state->prev_zone = PROX_ZONE_FAR;
    state->estimated_range_m = 1.0; /* Assume far away */
    state->approach_speed_m_s = 0.0;
    state->time_to_contact_s = INFINITY;
}
