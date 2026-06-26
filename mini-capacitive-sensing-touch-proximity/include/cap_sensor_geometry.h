/**
 * @file cap_sensor_geometry.h
 * @brief Electrode Geometry Models, Fringe Fields, Guard Rings
 *
 * The electrode geometry determines the electric field distribution,
 * which dictates sensitivity, range, and immunity to parasitic coupling.
 * Proper geometry design can improve SNR by 6-20 dB over naive layouts.
 *
 * Knowledge Coverage:
 *   L1: electrode shape types, fill ratio, hatch pattern, ground clearance
 *   L2: fringe field, guard ring principle, driven shield, ground-hatch
 *   L3: conformal mapping (Schwarz-Christoffel), elliptic integrals, BEM
 *   L4: Laplace equation, uniqueness theorem, method of images
 *   L5: optimal gap calculation, guard ring sizing, crosstalk minimization
 *   L6: interdigitated mutual-cap design, slider/wheel segmentation
 *
 * Ref: Baxter (1997) Ch.3-4 "Electrode Geometry and Fields"
 *      Haberman "Applied Partial Differential Equations" Ch.2 (Laplace)
 *      Weber "Electromagnetic Fields" (conformal mapping for CPW)
 */

#ifndef CAP_SENSOR_GEOMETRY_H
#define CAP_SENSOR_GEOMETRY_H

#include "cap_sense_core.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * L3: CONFORMAL MAPPING FOR COPLANAR ELECTRODES
 * ========================================================================== */

/** Coplanar strip model using conformal mapping.
 *
 *  Two coplanar electrodes of width w, length L, separation g.
 *  The mutual capacitance per unit length is:
 *
 *  C'/epsilon0 = (1+eps_r_sub)/2 * K(k')/K(k)
 *
 *  where k = g/(g+2w), k' = sqrt(1-k^2), K is complete elliptic integral
 *  of the first kind.
 *
 *  For thin substrates: effective eps_r_eff = (1+eps_r_sub)/2
 *  For thick substrates: eps_r_eff approaches eps_r_sub
 */
typedef struct {
    double electrode_width_m;     /**< Width w [m] */
    double gap_m;                 /**< Gap g [m] */
    double electrode_length_m;    /**< Length L [m] */
    double substrate_thickness_m; /**< Substrate thickness h [m] */
    double substrate_epsilon_r;   /**< Substrate permittivity */
    double eps_r_eff;             /**< Effective permittivity */
    double k_modulus;             /**< Elliptic modulus k */
    double k_complement;          /**< Complementary modulus k_prime */
    double elliptic_ratio;        /**< K(k')/K(k) */
    double c_per_unit_length_fm;  /**< C' [F/m] */
    double c_total_f;             /**< C_total [F] */
} cap_coplanar_strip_model_t;

/** Complete elliptic integral K(k) approximation.
 *
 *  Uses the arithmetic-geometric mean (AGM) method, which converges
 *  quadratically in ~5-8 iterations to machine precision.
 *
 *  K(k) = pi/2 / AGM(1, k')
 *
 *  Ref: Abramowitz & Stegun §17.6
 */
typedef struct {
    double modulus_k;            /**< Input modulus k [0,1) */
    double agm_result;           /**< AGM(1, k') result */
    double integral_k;           /**< K(k) */
    double integral_k_prime;     /**< K(k') */
    uint8_t iterations;          /**< AGM iterations to convergence */
} cap_elliptic_integral_t;

/** Fringe field correction model.
 *
 *  Real electrodes have fringe fields that increase the total capacitance
 *  beyond the parallel-plate value. The fringe factor F depends on the
 *  aspect ratio (perimeter/thickness) and dielectric environment.
 *
 *  For a rectangular electrode of width W, length L, at distance d from
 *  ground plane:
 *
 *  C_total = C_parallel * (1 + F_fringe)
 *  F_fringe ≈ (2d/pi*W) * ln(1 + pi*W/d) + (perimeter/d) * ...
 *
 *  Simplified Palmer's formula for square electrode:
 *  F_fringe ≈ 1 + (2d/pi*a) * (1 + ln(pi*a/d))
 *  where a = sqrt(area) is the characteristic dimension.
 */
typedef struct {
    double parallel_plate_c_f;   /**< C_pp = eps0*eps_r*A/d [F] */
    double electrode_area_m2;    /**< Electrode area [m2] */
    double electrode_perimeter_m;/**< Electrode perimeter [m] */
    double characteristic_dim_m; /**< sqrt(area) [m] */
    double gap_to_ground_m;      /**< Distance to ground plane [m] */
    double dielectric_constant;  /**< Between electrode and ground */
    double fringe_correction;    /**< Palmer fringe factor (>=1) */
    double total_c_with_fringe_f;/**< C_total = C_pp * fringe [F] */
} cap_fringe_field_model_t;

/* ==========================================================================
 * L2: GUARD RING DESIGN
 * ========================================================================== */

/** Guard ring design parameters.
 *
 *  A guard ring is a conducting ring around the sensor electrode, driven
 *  to the same potential as the sensor. This eliminates the E-field between
 *  sensor and guard, making the parasitic capacitance C_par_guarded ≈ 0.
 *
 *  The guard ring must:
 *  1. Be driven by a unity-gain buffer with BW >> f_excitation
 *  2. Have low output impedance across the frequency range
 *  3. Be wide enough to shield effectively (> 3x gap is ideal)
 *
 *  Parasitic reduction ratio = C_par_original / C_par_guarded
 *  Typically 10x-100x depending on guard ring width and buffer quality.
 */
typedef struct {
    double  electrode_width_m;    /**< Sensor electrode dimension [m] */
    double  guard_ring_width_m;   /**< Guard ring conductor width [m] */
    double  guard_gap_m;          /**< Gap between electrode and guard [m] */
    double  pcb_thickness_m;      /**< PCB thickness [m] */
    double  pcb_epsilon_r;        /**< PCB dielectric constant */
    double  c_parasitic_unguarded_f;  /**< Parasitic C without guard [F] */
    double  c_parasitic_guarded_f;    /**< Parasitic C with ideal guard [F] */
    double  reduction_ratio;      /**< C_unguarded / C_guarded */
    double  buffer_gain;          /**< Guard buffer gain (ideally 1.0) */
    double  buffer_bw_hz;         /**< Guard buffer bandwidth [Hz] */
    double  residual_error_pct;   /**< Residual parasitic % due to non-ideal buffer */
} cap_guard_ring_design_t;

/* ==========================================================================
 * L5: INTERDIGITATED MUTUAL CAPACITANCE
 * ========================================================================== */

/** Interdigitated electrode design for mutual capacitance.
 *
 *  Interdigitated fingers maximize the TX-RX coupling perimeter. The
 *  mutual capacitance is proportional to the total coupling length:
 *
 *  C_mutual ≈ N_fingers * L_finger * C'_unit
 *
 *  where C'_unit depends on finger width, gap, and substrate.
 *
 *  Finger width w and gap g follow: w ≈ g for maximum sensitivity.
 *  Minimum w and g are limited by PCB manufacturing (typ 100 um for standard,
 *  50 um for advanced).
 *
 *  Number of fingers N: more fingers = more C_mutual but also more
 *  parasitic. Practical N: 5-50 depending on available area.
 */
typedef struct {
    uint16_t num_fingers_tx;      /**< Number of TX fingers */
    uint16_t num_fingers_rx;      /**< Number of RX fingers */
    double   finger_width_m;      /**< Individual finger width [m] */
    double   finger_length_m;     /**< Overlap (coupling) length [m] */
    double   finger_gap_m;        /**< Gap between adjacent TX/RX fingers [m] */
    double   total_coupling_length_m; /**< Total TX-RX edge length [m] */
    double   c_mutual_estimated_f; /**< Estimated C_mutual [F] */
    double   c_parasitic_tx_f;     /**< TX parasitic to ground [F] */
    double   c_parasitic_rx_f;     /**< RX parasitic to ground [F] */
    double   sensitivity_delta_c_over_c; /**< deltaC/C_mutual for touch [%] */
} cap_interdigitated_design_t;

/* ==========================================================================
 * L5: SLIDER AND WHEEL GEOMETRY
 * ========================================================================== */

/** Linear slider geometry.
 *
 *  A slider uses N adjacent electrode segments. Touch position is
 *  interpolated from the capacitance changes on adjacent segments:
 *
 *  position = sum(seg_i * deltaC_i) / sum(deltaC_i)
 *
 *  Segmentation: N segments of width W_seg, gap g between segments.
 *  Total slider length = N * (W_seg + g).
 *  Resolution = total_length / (SNR_linear * N) typically 0.1-1 mm.
 */
typedef struct {
    uint8_t  num_segments;        /**< Number of slider segments */
    double   segment_width_m;     /**< Each segment width [m] */
    double   segment_length_m;    /**< Segment height [m] */
    double   inter_segment_gap_m; /**< Gap between segments [m] */
    double   total_length_m;      /**< Overall slider length [m] */
    double   delta_c_per_seg[16]; /**< deltaC for each segment [F] (max 16 seg) */
    double   interpolated_pos_m;  /**< Interpolated finger position [m] */
    double   position_resolution_m; /**< Estimated position resolution [m] */
} cap_slider_geometry_t;

/** Rotary wheel geometry.
 *
 *  A wheel uses N angular segments. Touch position is interpolated
 *  similarly to slider, but wrapped modulo 360 degrees.
 *
 *  Angle = atan2(sum(deltaC_i * sin(theta_i)), sum(deltaC_i * cos(theta_i)))
 *
 *  Typical: 3-8 segments, angular resolution 1-5 degrees.
 */
typedef struct {
    uint8_t  num_segments;        /**< Number of angular segments */
    double   inner_radius_m;      /**< Wheel inner radius [m] */
    double   outer_radius_m;      /**< Wheel outer radius [m] */
    double   inter_segment_gap_m; /**< Gap between segments [m] */
    double   segment_angle_deg;   /**< Angular span per segment [deg] */
    double   delta_c_per_seg[12]; /**< deltaC per segment (max 12) */
    double   interpolated_angle_deg; /**< Interpolated angle [deg] */
    double   angle_resolution_deg;   /**< Estimated resolution [deg] */
} cap_wheel_geometry_t;

/* ==========================================================================
 * L5-L6: API
 * ========================================================================== */

/** Compute elliptic integral K(k) using AGM method.
 *
 *  K(k) = pi/2 / AGM(1, sqrt(1-k^2))
 *
 *  For 0 <= k < 1. Converges in ~6 iterations for k < 0.999.
 *  For k close to 1, use K(k) = ln(4/sqrt(1-k^2)) approximation.
 *
 *  @param k  Modulus [0, 1)
 *  @return K(k)
 *  Ref: Abramowitz & Stegun 17.6
 */
double cap_elliptic_k(double k);

/** Compute coplanar strip capacitance using conformal mapping.
 *
 *  @param model  Coplanar strip model (populated with geometry, outputs C)
 */
void cap_coplanar_strip_capacitance(cap_coplanar_strip_model_t *model);

/** Compute fringe field correction using Palmer's formula.
 *
 *  @param model  Fringe model (inputs: area, perimeter, gap, eps_r; outputs: fringe and C_total)
 */
void cap_fringe_field_correction(cap_fringe_field_model_t *model);

/** Design guard ring and compute parasitic reduction.
 *
 *  @param design  Guard ring design (inputs: geometry, buffer specs; outputs: reduction ratio)
 */
void cap_guard_ring_design_analyze(cap_guard_ring_design_t *design);

/** Design interdigitated electrodes for target mutual capacitance.
 *
 *  @param design    Design struct (inputs: area constraint, PCB specs; outputs: geometry)
 *  @param max_width Maximum allowed electrode width [m]
 *  @param max_length Maximum allowed electrode length [m]
 *  @param min_gap   Minimum manufacturable gap [m]
 */
void cap_interdigitated_design(cap_interdigitated_design_t *design,
                               double max_width, double max_length,
                               double min_gap);

/** Interpolate touch position on a linear slider.
 *
 *  Uses centroid interpolation: pos = sum(i * deltaC_i) / sum(deltaC_i)
 *  Scaled to physical units [m].
 *
 *  @param slider  Slider geometry with measured deltaC
 *  @return Interpolated position [m]
 */
double cap_slider_interpolate_position(cap_slider_geometry_t *slider);

/** Interpolate touch angle on a rotary wheel.
 *
 *  Uses vector sum: angle = atan2(sum(sin_i * dC_i), sum(cos_i * dC_i))
 *
 *  @param wheel  Wheel geometry with measured deltaC
 *  @return Interpolated angle [deg] (0-360)
 */
double cap_wheel_interpolate_angle(cap_wheel_geometry_t *wheel);

/** Compute optimal electrode-to-ground gap for minimum parasitic coupling.
 *
 *  There is a tradeoff: larger gap reduces parasitic C but increases
 *  susceptibility to external interference. The optimum balances these.
 *
 *  @param electrode_area    Electrode area [m2]
 *  @param pcb_thickness     PCB thickness [m]
 *  @param epsilon_r         PCB dielectric constant
 *  @return Optimal ground gap [m]
 */
double cap_optimal_ground_gap(double electrode_area, double pcb_thickness,
                              double epsilon_r);

/** Estimate crosstalk between two adjacent electrodes.
 *
 *  Crosstalk C_xtalk causes false touches on adjacent channels.
 *  For rectangular electrodes: C_xtalk ≈ eps0 * eps_r_eff * L * w / g
 *
 *  @param adjacent_length   Adjacent edge length [m]
 *  @param electrode_width   Electrode width perpendicular to gap [m]
 *  @param gap               Gap between electrodes [m]
 *  @param eps_r_eff         Effective dielectric constant
 *  @return Crosstalk capacitance [F]
 */
double cap_crosstalk_estimate(double adjacent_length, double electrode_width,
                              double gap, double eps_r_eff);

/** Design hatched ground plane fill pattern.
 *
 *  @param hatch_pitch_m     Center-to-center hatch spacing [m]
 *  @param hatch_width_m     Conductor width within hatch [m]
 *  @return Fill ratio [0-1]
 */
double cap_hatch_fill_ratio(double hatch_pitch_m, double hatch_width_m);

#endif /* CAP_SENSOR_GEOMETRY_H */
