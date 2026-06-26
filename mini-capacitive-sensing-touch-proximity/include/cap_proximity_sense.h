/**
 * @file cap_proximity_sense.h
 * @brief Proximity Sensing: Range Estimation, Electrode Sizing, SAR Compliance
 *
 * Proximity sensing detects a human body at distances of 1-30 cm before
 * physical contact. This enables: display wake-up, SAR power reduction,
 * presence detection, and touchless gesture interfaces.
 *
 * Knowledge Coverage:
 *   L1: proximity range, SAR, detection zone, approach speed
 *   L2: self-cap proximity principle, 1/r sensitivity rolloff, range vs size
 *   L3: dipole approximation, image charge method, field line computation
 *   L4: Coulomb inverse-square, method of images, Gauss flux theorem
 *   L5: range estimation via 1/r^3 model, adaptive proximity threshold
 *   L6: proximity wake-up, SAR-compliant sensor, approach speed detection
 *
 * The electric field from a sensor electrode decays approximately as 1/r^3
 * for distances comparable to electrode size (dipole-like), transitioning
 * to 1/r^2 at larger distances (monopole regime when body couples to ground).
 *
 * Ref: Baxter (1997) Ch.6 "Proximity Sensing"
 *      IEEE C95.1-2019 SAR limits
 *      IEC 62209 SAR measurement procedures
 */

#ifndef CAP_PROXIMITY_SENSE_H
#define CAP_PROXIMITY_SENSE_H

#include "cap_sense_core.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * L1: PROXIMITY DEFINITIONS
 * ========================================================================== */

/** Proximity range zones.
 *
 *  FAR:     Beyond useful range (>30 cm), sensor idle or slow scan.
 *  MEDIUM:  Body detected within ~30 cm, prepare for possible touch.
 *           Scan rate increases, touch detection armed.
 *  NEAR:    Body within ~5 cm, imminent touch. Fast scan, display wake.
 *  CONTACT: Touching or nearly touching (<1 cm gap).
 */
typedef enum {
    PROX_ZONE_FAR,
    PROX_ZONE_MEDIUM,
    PROX_ZONE_NEAR,
    PROX_ZONE_CONTACT
} proximity_zone_t;

/** Proximity sensing configuration.
 *
 *  The key design tradeoff: larger electrode = longer range but more
 *  parasitic capacitance and slower settling. Range scales approximately
 *  with the square root of electrode area.
 *
 *  Rule of thumb: sensing range ≈ 0.5 * sqrt(electrode_area) for self-cap,
 *  and range ≈ 0.25 * electrode_diameter for mutual-cap.
 *
 *  Example: 10×10 cm electrode → self-cap range ~5 cm, mutual range ~2.5 cm.
 */
typedef struct {
    double  electrode_area_m2;       /**< Sensing electrode area [m2] */
    double  electrode_diameter_m;    /**< Equivalent diameter [m] */
    double  far_threshold_f;         /**< deltaC for FAR->MEDIUM transition [F] */
    double  medium_threshold_f;      /**< deltaC for MEDIUM->NEAR transition [F] */
    double  near_threshold_f;        /**< deltaC for NEAR->CONTACT transition [F] */
    double  max_range_m;             /**< Maximum detection range [m] */
    double  scan_period_slow_ms;     /**< Scan period in FAR zone [ms] */
    double  scan_period_fast_ms;     /**< Scan period in NEAR zone [ms] */
    uint8_t debounce_zone_samples;   /**< Zone-change debounce count */
    bool    sar_enabled;             /**< SAR proximity for RF power reduction */
    double  sar_range_m;             /**< SAR trigger distance [m] */
} cap_proximity_config_t;

/** Proximity measurement state.
 *
 *  Tracks the current zone, smoothed range estimate, and approach
 *  velocity for predictive touch detection.
 */
typedef struct {
    proximity_zone_t current_zone;   /**< Current proximity zone */
    proximity_zone_t prev_zone;      /**< Previous zone (for edge detection) */
    double   estimated_range_m;      /**< Estimated body distance [m] */
    double   smoothed_delta_c_f;     /**< Low-pass filtered deltaC [F] */
    double   approach_speed_m_s;     /**< Estimated approach speed [m/s] */
    double   time_to_contact_s;      /**< Predicted time to contact [s] */
    uint32_t zone_entry_time_ms;     /**< Time of last zone change [ms] */
    uint32_t far_zone_samples;       /**< Consecutive FAR samples */
    uint32_t near_zone_samples;      /**< Consecutive NEAR samples */
    bool     sar_triggered;          /**< SAR power reduction active */
    double   max_delta_c_observed_f; /**< Peak deltaC in current approach */
} cap_proximity_state_t;

/* ==========================================================================
 * L3: RANGE ESTIMATION MODELS
 * ========================================================================== */

/** Dipole approximation for proximity field.
 *
 *  Models the sensor electrode and its image in the ground plane as
 *  an electric dipole. For a circular electrode of radius a, the field
 *  at distance r along the axis is:
 *
 *  E(r) = (2 * Q * d) / (4*pi*eps0 * (r^2 + d^2)^(3/2))
 *
 *  where d is the electrode-to-image separation.
 */
typedef struct {
    double electrode_radius_m;   /**< Electrode radius [m] */
    double image_distance_m;     /**< Electrode-to-image distance [m] */
    double charge_c;             /**< Net charge on electrode [C] */
    double field_at_1m_v_m;      /**< Field strength at 1m [V/m] */
    double dipole_moment_cm;     /**< Dipole moment p = Q*d [C*m] */
} cap_dipole_model_t;

/** Inverse power-law range estimator.
 *
 *  deltaC = C0 * (r0/r)^n
 *
 *  where n ≈ 2 (far field, monopole) to n ≈ 3 (near field, dipole).
 *  C0 is the deltaC at reference distance r0 (typically contact).
 *  Typical C0 for a 10 mm dia electrode: 0.5-2 pF at contact.
 */
typedef struct {
    double c0_f;                 /**< deltaC at contact [F] */
    double r0_m;                 /**< Reference distance [m] (typ overlay thickness) */
    double exponent_n;           /**< Power-law exponent (2.0-3.0) */
    double range_at_threshold_m; /**< Range where deltaC = threshold [m] */
} cap_range_model_t;

/* ==========================================================================
 * L4-L5: API
 * ========================================================================== */

/** Initialize proximity configuration.
 *
 *  @param cfg           Config to initialize
 *  @param electrode_area Electrode area [m2]
 *  @param max_range     Desired max detection range [m]
 */
void cap_proximity_config_init(cap_proximity_config_t *cfg,
                               double electrode_area, double max_range);

/** Estimate range from delta capacitance using inverse power law.
 *
 *  range = r0 * (C0 / deltaC)^(1/n)
 *
 *  Clamped to [0, max_range].
 *
 *  @param delta_c_f    Measured deltaC [F]
 *  @param model        Range model parameters
 *  @param max_range_m  Clamp maximum [m]
 *  @return Estimated range [m]
 */
double cap_estimate_range_power_law(double delta_c_f,
                                    const cap_range_model_t *model,
                                    double max_range_m);

/** Compute the deltaC expected at a given range.
 *
 *  deltaC = C0 * (r0 / r)^n
 *
 *  @param r_m          Range [m]
 *  @param model        Range model parameters
 *  @return Expected deltaC [F]
 */
double cap_delta_c_at_range(double r_m, const cap_range_model_t *model);

/** Determine proximity zone from deltaC.
 *
 *  @param delta_c_f   Current deltaC [F]
 *  @param cfg         Proximity config with zone thresholds
 *  @param state       Proximity state (updated with debounce)
 *  @param now_ms      Current time [ms]
 *  @return Current proximity zone
 */
proximity_zone_t cap_determine_proximity_zone(double delta_c_f,
                                              const cap_proximity_config_t *cfg,
                                              cap_proximity_state_t *state,
                                              uint32_t now_ms);

/** Smooth deltaC with low-pass filter for stable range estimation.
 *
 *  y[n] = alpha * x[n] + (1-alpha) * y[n-1]
 *
 *  Separate alpha values for approaching (faster response) vs receding
 *  (slower, to avoid flicker).
 *
 *  @param smoothed      Current smoothed value (updated in-place)
 *  @param new_delta_c   New raw deltaC
 *  @param alpha_approach Smoothing factor when delta increasing
 *  @param alpha_recede   Smoothing factor when delta decreasing
 */
void cap_proximity_smooth_delta(double *smoothed, double new_delta_c,
                                double alpha_approach, double alpha_recede);

/** Estimate approach speed from deltaC rate of change.
 *
 *  v_approach = dr/dt = dr/dC * dC/dt
 *
 *  Using the inverse power law: dr/dC = -r0 * C0^(1/n) / (n * deltaC^((n+1)/n))
 *
 *  @param delta_c_f      Current deltaC [F]
 *  @param delta_c_prev_f  Previous deltaC [F]
 *  @param dt_s           Time between samples [s]
 *  @param model          Range model
 *  @return Approach speed [m/s], positive = approaching
 */
double cap_estimate_approach_speed(double delta_c_f, double delta_c_prev_f,
                                   double dt_s, const cap_range_model_t *model);

/** Compute dipole model parameters from electrode geometry.
 *
 *  @param model       Dipole model to populate
 *  @param radius_m    Electrode radius [m]
 *  @param voltage_v   Electrode voltage [V]
 *  @param c_self_f    Electrode self-capacitance [F]
 */
void cap_compute_dipole_model(cap_dipole_model_t *model, double radius_m,
                              double voltage_v, double c_self_f);

/** Compute minimum electrode area for desired range.
 *
 *  From the dipole approximation, the field at range r scales with
 *  electrode area A: E(r) proportional to A * V / r^3.
 *
 *  Minimum A = (threshold_deltaC * r^3) / (k * V * eps0 * eps_r)
 *
 *  where k is a geometric factor (~1.0 for disc electrodes).
 *
 *  @param desired_range_m  Target detection range [m]
 *  @param threshold_f      Minimum detectable deltaC [F]
 *  @param v_exc            Excitation voltage [V]
 *  @param overlay_er       Overlay permittivity
 *  @return Minimum electrode area [m2]
 */
double cap_min_electrode_area_for_range(double desired_range_m,
                                        double threshold_f,
                                        double v_exc, double overlay_er);

/** Reset proximity state.
 *
 *  @param state  State to reset
 */
void cap_proximity_state_reset(cap_proximity_state_t *state);

#endif /* CAP_PROXIMITY_SENSE_H */
