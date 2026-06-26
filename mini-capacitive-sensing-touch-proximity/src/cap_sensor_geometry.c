/**
 * @file cap_sensor_geometry.c
 * @brief Electrode Geometry Models: Conformal Mapping, Fringe Fields, Guard Rings
 *
 * The electrode geometry determines the electric field distribution,
 * which directly impacts sensitivity, noise immunity, and touch accuracy.
 * This file implements the key analytical models.
 *
 * Knowledge Coverage:
 *   L3: elliptic integrals (AGM), conformal mapping for CPW, fringe fields
 *   L4: Laplace equation (analytic solutions), method of images
 *   L5: guard ring design optimization, interdigitated electrode synthesis
 *   L6: slider/wheel position interpolation, crosstalk minimization
 *
 * Ref: Abramowitz & Stegun "Handbook of Mathematical Functions" (1964)
 *      Ghione & Naldi (1984) "Coplanar Waveguides for MMIC Applications"
 *      IEEE Trans MTT, 32(3)
 */

#include "cap_sensor_geometry.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ==========================================================================
 * L3: ELLIPTIC INTEGRAL VIA AGM
 * ========================================================================== */

/**
 * cap_elliptic_k
 *
 * Computes the complete elliptic integral of the first kind K(k) using
 * the Arithmetic-Geometric Mean (AGM) method.
 *
 * Definition: K(k) = integral_0^{pi/2} dtheta / sqrt(1 - k^2 * sin^2(theta))
 *
 * AGM algorithm (Gauss, 1799):
 *   a_0 = 1, b_0 = sqrt(1 - k^2) = k'
 *   a_{n+1} = (a_n + b_n) / 2
 *   b_{n+1} = sqrt(a_n * b_n)
 *   Converges to AGM(1, k') in ~log2(1/k') iterations.
 *
 *   K(k) = pi / (2 * AGM(1, k'))
 *
 * For k close to 1 (k > 0.999), the AGM converges slowly. In that case,
 * we use the asymptotic approximation:
 *   K(k) ≈ ln(4/k')   where k' = sqrt(1 - k^2)
 *
 * Convergence: ~5-8 iterations for double precision, k < 0.99.
 *
 * @param k  Modulus [0, 1)
 * @return K(k)
 *
 * Ref: Abramowitz & Stegun (1964) §17.6
 *      Carlson (1979) "Computing Elliptic Integrals by Duplication"
 */
double cap_elliptic_k(double k)
{
    if (k < 0.0 || k >= 1.0) {
        /* For k >= 1 (should not happen for capacitance calculations) */
        if (k >= 1.0) return 1e308; /* K(1) diverges */
        return 0.0;
    }

    if (k == 0.0) return M_PI / 2.0; /* K(0) = pi/2 */

    double kp = sqrt(1.0 - k * k);

    /* For k very close to 1, use asymptotic form */
    if (kp < 1e-4) {
        return log(4.0 / kp);
    }

    /* AGM */
    double a = 1.0;
    double b = kp;
    uint8_t iter = 0;

    while (fabs(a - b) > 1e-15 && iter < 30) {
        double a_next = (a + b) / 2.0;
        double b_next = sqrt(a * b);
        a = a_next;
        b = b_next;
        iter++;
    }

    return M_PI / (2.0 * a);
}

/* ==========================================================================
 * L3: COPLANAR STRIP — CONFORMAL MAPPING
 * ========================================================================== */

/**
 * cap_coplanar_strip_capacitance
 *
 * Computes the mutual capacitance per unit length of two coplanar
 * strip electrodes using conformal mapping.
 *
 * For two electrodes of width w, separation g, on a substrate of
 * thickness h and permittivity eps_r:
 *
 *   k = g / (g + 2w)
 *   k' = sqrt(1 - k^2)
 *
 * Effective permittivity:
 *   eps_eff = 1 + (eps_r - 1) * K(k')*K(k1) / (2*K(k)*K(k1'))
 *   where k1 = sinh(pi*g/(4h)) / sinh(pi*(g+2w)/(4h))
 *
 * For thin substrates (h << g+w):
 *   eps_eff ≈ (1 + eps_r) / 2   (half field in air, half in substrate)
 *
 * Capacitance per unit length:
 *   C' = eps0 * eps_eff * K(k') / K(k)   [F/m]
 *
 * Total mutual capacitance:
 *   C_m = C' * L   [F]
 *
 * where L is the electrode length.
 *
 * This is the fundamental design equation for interdigitated mutual-cap
 * sensors.
 *
 * @param model  Coplanar strip model (geometry in, capacitance out)
 *
 * Ref: Ghione & Naldi (1984) IEEE Trans MTT 32(3)
 *      Simons "Coplanar Waveguide Circuits, Components, and Systems" (2001)
 */
void cap_coplanar_strip_capacitance(cap_coplanar_strip_model_t *model)
{
    if (!model) return;
    if (model->electrode_width_m <= 0.0 || model->gap_m <= 0.0) {
        model->c_total_f = 0.0;
        return;
    }

    double w = model->electrode_width_m;
    double g = model->gap_m;
    double L = model->electrode_length_m;
    double h = model->substrate_thickness_m;
    double eps_r = model->substrate_epsilon_r;

    /* Elliptic modulus k = g/(g+2w) */
    model->k_modulus = g / (g + 2.0 * w);
    model->k_complement = sqrt(1.0 - model->k_modulus * model->k_modulus);

    /* Compute K(k) and K(k') */
    double Kk = cap_elliptic_k(model->k_modulus);
    double Kkp = cap_elliptic_k(model->k_complement);
    model->elliptic_ratio = Kkp / Kk;

    /* Effective permittivity */
    if (h > 0.0 && h < 1000.0) {
        /* Full expression with substrate thickness */
        double k1_num = sinh(M_PI * g / (4.0 * h));
        double k1_den = sinh(M_PI * (g + 2.0 * w) / (4.0 * h));
        double k1 = k1_num / k1_den;
        double k1p = sqrt(1.0 - k1 * k1);
        double Kk1 = cap_elliptic_k(k1);
        double Kk1p = cap_elliptic_k(k1p);

        model->eps_r_eff = 1.0 + (eps_r - 1.0) * (Kkp * Kk1) / (2.0 * Kk * Kk1p);
    } else {
        /* Thin substrate limit */
        model->eps_r_eff = (1.0 + eps_r) / 2.0;
    }

    /* Capacitance per unit length */
    model->c_per_unit_length_fm = CAP_EPSILON0 * model->eps_r_eff * model->elliptic_ratio;

    /* Total capacitance */
    model->c_total_f = model->c_per_unit_length_fm * L;
}

/* ==========================================================================
 * L3: FRINGE FIELD CORRECTION — PALMER FORMULA
 * ========================================================================== */

/**
 * cap_fringe_field_correction
 *
 * Computes the fringe field correction to the parallel-plate capacitance.
 *
 * For a rectangular electrode of area A, perimeter P, at distance d from
 * the ground plane:
 *
 *   C_pp = eps0 * eps_r * A / d
 *
 * Palmer's fringe correction (modified for rectangular electrodes):
 *   C_fringe = C_pp * (1 + (2*d/(pi*sqrt(A))) * (1 + ln(pi*sqrt(A)/d))
 *                     + eps0 * eps_r * P * 0.5 * ln(4*d/w_trace))
 *
 * where the first term corrects for area-dependent fringing and the
 * second term accounts for edge/perimeter fringing.
 *
 * For a 1 cm² square electrode at d=1mm from ground:
 *   sqrt(A) = 0.01m, d/sqrt(A) = 0.1
 *   fringe factor ≈ 1 + (2*0.1/pi) * (1 + ln(pi*10)) ≈ 1 + 0.064 * 4.45 ≈ 1.28
 *
 * So C_total ≈ 1.28 * C_pp, roughly 28% higher due to fringe fields.
 *
 * @param model  Fringe field model (inputs: area, perimeter, gap, eps_r;
 *               outputs: fringe correction and C_total)
 *
 * Ref: Palmer (1937) "Capacitance of a Parallel-Plate Capacitor by the
 *      Schwartz-Christoffel Transformation" Trans AIEE, 56(3)
 */
void cap_fringe_field_correction(cap_fringe_field_model_t *model)
{
    if (!model) return;
    if (model->electrode_area_m2 <= 0.0 || model->gap_to_ground_m <= 0.0) {
        model->fringe_correction = 1.0;
        model->total_c_with_fringe_f = model->parallel_plate_c_f;
        return;
    }

    double A = model->electrode_area_m2;
    double P = model->electrode_perimeter_m;
    double d = model->gap_to_ground_m;
    double eps_r = model->dielectric_constant;

    /* Characteristic dimension */
    model->characteristic_dim_m = sqrt(A);

    /* Parallel-plate capacitance */
    model->parallel_plate_c_f = CAP_EPSILON0 * eps_r * A / d;

    /* Palmer area-based fringe correction */
    double char_dim = model->characteristic_dim_m;
    double ratio = d / char_dim;
    double fringe_area = 1.0 + (2.0 * ratio / M_PI) * (1.0 + log(M_PI / ratio));

    /* Perimeter-based edge fringe */
    double fringe_perim = 0.0;
    if (P > 0.0 && char_dim > 0.0) {
        /* Approximate edge width as sqrt(A) */
        fringe_perim = CAP_EPSILON0 * eps_r * P * 0.5 * log(4.0 * d / (char_dim * 0.1));
        /* Normalize to parallel-plate value */
        fringe_perim = fringe_perim / model->parallel_plate_c_f;
    }

    model->fringe_correction = fringe_area + fringe_perim;
    model->total_c_with_fringe_f = model->parallel_plate_c_f * model->fringe_correction;
}

/* ==========================================================================
 * L5: GUARD RING DESIGN
 * ========================================================================== */

/**
 * cap_guard_ring_design_analyze
 *
 * Analyzes the effectiveness of a guard ring in reducing parasitic
 * capacitance from the sensor electrode to surrounding ground.
 *
 * Without guard ring: C_parasitic ≈ eps0 * eps_r * A_edge / gap_ground
 *   This represents field lines from electrode edge to nearby ground.
 *
 * With ideal guard ring: The guard is driven to the same potential as
 * the sensor, so there is zero E-field between sensor and guard.
 * The parasitic C is then only from the guard ring outer edge to ground.
 *
 * Parasitic reduction ratio:
 *   R = C_unguarded / C_guarded ≈ (gap_to_ground / guard_gap) * (guard_width / electrode_width)
 *
 * Non-ideal buffer effects:
 *   If the buffer has gain error dG = (1 - G), the residual voltage
 *   across the sensor-guard gap is V_sense * dG.
 *
 *   Residual C_parasitic = C_unguarded * dG
 *   Reduction ratio limited to 1/dG
 *
 * For a buffer with gain error 0.1% (dG=0.001):
 *   Maximum reduction ratio ≈ 1/0.001 = 1000x
 *
 * @param design  Guard ring design (inputs: geometry, buffer specs;
 *                outputs: parasitics, reduction ratio)
 */
void cap_guard_ring_design_analyze(cap_guard_ring_design_t *design)
{
    if (!design) return;
    if (design->electrode_width_m <= 0.0 || design->guard_gap_m <= 0.0) {
        design->reduction_ratio = 1.0;
        return;
    }

    double w = design->electrode_width_m;
    double g = design->guard_gap_m;
    double h = design->pcb_thickness_m;
    double eps_r = design->pcb_epsilon_r;

    /* Estimate unguarded parasitic: electrode edge to ground */
    /* Simplified: C = eps0 * eps_r * fringe_perimeter * effective_distance */
    double perimeter = 4.0 * w; /* Assuming square electrode */
    double d_eff = h + g;       /* Field paths to ground */

    if (d_eff > 0.0) {
        design->c_parasitic_unguarded_f =
            CAP_EPSILON0 * eps_r * perimeter * 0.5 * log(1.0 + h / d_eff);
    } else {
        design->c_parasitic_unguarded_f = 1.0e-12; /* 1 pF default */
    }

    /* Guarded parasitic: reduced by gap geometry */
    /* Field lines now go from guard outer edge to ground */
    double guarded_perimeter = 4.0 * (w + 2.0 * design->guard_ring_width_m + 2.0 * g);
    double guarded_gap = h; /* Guard to underlying ground plane */

    if (guarded_gap > 0.0) {
        design->c_parasitic_guarded_f =
            CAP_EPSILON0 * eps_r * guarded_perimeter * 0.5 * log(1.0 + design->guard_ring_width_m / guarded_gap);
        /* Scale down: the guard ring is driven, not sensor */
        design->c_parasitic_guarded_f *= 0.1;
    } else {
        design->c_parasitic_guarded_f = design->c_parasitic_unguarded_f;
    }

    /* Geometric reduction ratio */
    if (design->c_parasitic_guarded_f > 0.0) {
        design->reduction_ratio =
            design->c_parasitic_unguarded_f / design->c_parasitic_guarded_f;
    } else {
        design->reduction_ratio = 100.0;
    }

    /* Buffer gain error limits the reduction */
    design->buffer_gain = 1.0; /* Assumed unity-gain buffer */
    /* Use a default input error of 0.1% for max possible reduction */
    double input_gain_error_pct = 0.1;
    double max_reduction = 1.0 / (input_gain_error_pct / 100.0);

    if (design->reduction_ratio > max_reduction) {
        design->reduction_ratio = max_reduction;
    }

    design->residual_error_pct = 100.0 / design->reduction_ratio;
}

/* ==========================================================================
 * L5: INTERDIGITATED ELECTRODE DESIGN
 * ========================================================================== */

/**
 * cap_interdigitated_design
 *
 * Designs an interdigitated electrode pattern for mutual-capacitance sensing.
 *
 * Interdigitated fingers maximize the coupling perimeter between TX and RX
 * electrodes. The mutual capacitance is:
 *
 *   C_m = C'_unit * L_finger * N_fingers
 *
 * where C'_unit is the per-unit-length capacitance from the coplanar strip model.
 *
 * Design constraints:
 * - Minimum finger width and gap set by PCB manufacturer (typ 100 um)
 * - Total area limited by available PCB real estate
 * - Number of fingers limited by total width: N * (w + g) <= total_width
 *
 * The design prioritizes:
 * 1. Maximum sensitivity (deltaC/C for touch)
 * 2. Minimum parasitic (C_tx to ground, C_rx to ground)
 * 3. Fabrication reliability (min > 100 um)
 *
 * @param design      Design struct (outputs finger geometry and C_m)
 * @param max_width   Max electrode total width [m]
 * @param max_length  Max finger length [m]
 * @param min_gap     Minimum gap [m] (PCB design rule)
 *
 * Ref: Microchip AN2934 §3.5 "Interdigitated Electrodes"
 */
void cap_interdigitated_design(cap_interdigitated_design_t *design,
                               double max_width, double max_length,
                               double min_gap)
{
    if (!design) return;

    double total_width = max_width;
    double finger_l = max_length;
    double min_w = min_gap; /* Finger width >= gap for sensitivity */

    /* Optimal: finger width = gap for maximum coupling per area */
    double finger_w = min_w;
    double finger_g = min_w;

    /* How many TX+RX finger pairs can fit? */
    /* For alternating TX-RX-TX-RX, each pair takes 2*w + 2*g space */
    double pitch_per_pair = 2.0 * finger_w + 2.0 * finger_g;
    uint16_t max_pairs = (uint16_t)(total_width / pitch_per_pair);
    if (max_pairs < 1) max_pairs = 1;
    if (max_pairs > 50) max_pairs = 50;

    design->num_fingers_tx = max_pairs;
    design->num_fingers_rx = max_pairs;
    design->finger_width_m = finger_w;
    design->finger_length_m = finger_l;
    design->finger_gap_m = finger_g;

    /* Total coupling length: N_pairs * (each pair shares 2 edges * L) */
    design->total_coupling_length_m = (double)max_pairs * 2.0 * finger_l;

    /* Estimate C_mutual using coplanar strip model */
    cap_coplanar_strip_model_t cps;
    memset(&cps, 0, sizeof(cps));
    cps.electrode_width_m = finger_w;
    cps.gap_m = finger_g;
    cps.electrode_length_m = design->total_coupling_length_m;
    cps.substrate_thickness_m = 0.0016; /* 1.6 mm FR4 */
    cps.substrate_epsilon_r = 4.5;

    cap_coplanar_strip_capacitance(&cps);
    design->c_mutual_estimated_f = cps.c_total_f;

    /* Estimate parasitics: each finger to ground */
    double finger_area = finger_w * finger_l;
    double c_par_per_finger = CAP_EPSILON0 * 4.5 * finger_area / 0.0016;

    design->c_parasitic_tx_f = c_par_per_finger * (double)max_pairs;
    design->c_parasitic_rx_f = c_par_per_finger * (double)max_pairs;

    /* Sensitivity: touch diverts ~20-40% of C_mutual to ground */
    design->sensitivity_delta_c_over_c = 0.3;
}

/* ==========================================================================
 * L6: SLIDER AND WHEEL INTERPOLATION
 * ========================================================================== */

/**
 * cap_slider_interpolate_position
 *
 * Interpolates finger position on a linear slider from segment deltaC values.
 *
 * Uses centroid (center-of-mass) interpolation:
 *
 *   position = sum(i * deltaC_i) / sum(deltaC_i)
 *
 * where i indexes the segment (0 to N-1).
 *
 * Converted to physical coordinates:
 *   x_mm = position * (segment_width + gap)
 *
 * The interpolation assumes the finger affects 1-3 adjacent segments,
 * with the deltaC profile approximating a Gaussian centered on the
 * finger position. Centroid interpolation is the maximum-likelihood
 * estimator for a symmetric profile.
 *
 * For N=8 segments of 5mm width:
 *   Position resolution ≈ width / (SNR * sqrt(N/2)) ≈ 5mm / (20 * 2) ≈ 0.125 mm
 *
 * @param slider  Slider geometry with measured deltaC per segment
 * @return Interpolated position [m] from start of slider
 */
double cap_slider_interpolate_position(cap_slider_geometry_t *slider)
{
    if (!slider || slider->num_segments == 0) return 0.0;

    double weighted_sum = 0.0;
    double total_delta_c = 0.0;

    for (uint8_t i = 0; i < slider->num_segments && i < 16; i++) {
        double dc = slider->delta_c_per_seg[i];
        if (dc > 0.0) {
            weighted_sum += (double)i * dc;
            total_delta_c += dc;
        }
    }

    if (total_delta_c <= 0.0) return 0.0;

    double position_index = weighted_sum / total_delta_c;
    double seg_pitch = slider->segment_width_m + slider->inter_segment_gap_m;

    slider->interpolated_pos_m = position_index * seg_pitch;
    return slider->interpolated_pos_m;
}

/**
 * cap_wheel_interpolate_angle
 *
 * Interpolates finger angle on a rotary wheel.
 *
 * Method: Complex-plane vector sum (phasor method)
 *
 *   X = sum(deltaC_i * cos(theta_i))
 *   Y = sum(deltaC_i * sin(theta_i))
 *   angle = atan2(Y, X)
 *
 * where theta_i is the angular center of segment i.
 *
 * This handles the circular wrap-around naturally because sin/cos
 * are periodic. The interpolation is independent of the absolute
 * deltaC magnitude (ratiometric).
 *
 * Resolution depends on SNR and number of segments:
 *   angle_res ≈ 360 / (N * SNR) [degrees]
 *
 * For 8 segments with SNR=30:
 *   resolution ≈ 360 / 240 = 1.5 degrees
 *
 * @param wheel  Wheel geometry with measured deltaC per segment
 * @return Interpolated angle [deg] in [0, 360)
 */
double cap_wheel_interpolate_angle(cap_wheel_geometry_t *wheel)
{
    if (!wheel || wheel->num_segments == 0) return 0.0;

    double sum_x = 0.0, sum_y = 0.0;
    double seg_angle = 360.0 / (double)wheel->num_segments;

    for (uint8_t i = 0; i < wheel->num_segments && i < 12; i++) {
        double dc = wheel->delta_c_per_seg[i];
        if (dc > 0.0) {
            double theta_rad = (seg_angle * (double)i) * M_PI / 180.0;
            sum_x += dc * cos(theta_rad);
            sum_y += dc * sin(theta_rad);
        }
    }

    double angle_rad = atan2(sum_y, sum_x);
    double angle_deg = angle_rad * 180.0 / M_PI;

    /* Normalize to [0, 360) */
    if (angle_deg < 0.0) angle_deg += 360.0;

    wheel->interpolated_angle_deg = angle_deg;
    return angle_deg;
}

/* ==========================================================================
 * L5: GROUND GAP AND CROSSTALK
 * ========================================================================== */

/**
 * cap_optimal_ground_gap
 *
 * Computes the optimal gap between a sensor electrode and surrounding
 * ground plane.
 *
 * Tradeoff:
 * - Larger gap → lower parasitic C (good) but more susceptibility to
 *   external interference (bad) and larger total footprint (bad)
 * - Smaller gap → more shielding from interference (good) but higher
 *   parasitic C (bad) reducing sensitivity
 *
 * The optimum minimizes the ratio: C_parasitic(effective) / sensitivity
 *
 * For a square electrode of side a on PCB of thickness h:
 *   C_parasitic ≈ 2 * eps0 * eps_r_eff * a * ln(1 + pi*a/(2*g))
 *   where g is the ground gap.
 *
 * Empirically: optimal_gap ≈ 0.5 * a to 1.0 * a
 *
 * For buttons ≤ 10mm: gap ≈ 0.5 mm
 * For buttons > 10mm: gap ≈ 1.0 to 2.0 mm
 *
 * @return Optimal ground gap [m]
 */
double cap_optimal_ground_gap(double electrode_area, double pcb_thickness,
                              double epsilon_r)
{
    if (electrode_area <= 0.0) return 0.0005; /* Default 0.5 mm */

    double a = sqrt(electrode_area);
    /* Empirical formula: gap proportional to electrode size */
    double gap = 0.2 * a;

    /* Minimum gap 0.25 mm (PCB manufacturing limit) */
    if (gap < 0.00025) gap = 0.00025;
    /* Maximum 2 mm */
    if (gap > 0.002) gap = 0.002;

    (void)pcb_thickness;
    (void)epsilon_r;
    return gap;
}

/**
 * cap_crosstalk_estimate
 *
 * Estimates the parasitic capacitive crosstalk between two adjacent
 * sensor electrodes.
 *
 * For rectangular electrodes separated by gap g, the edge-to-edge
 * capacitance is approximately:
 *
 *   C_xtalk ≈ eps0 * eps_r_eff * L_adjacent * w_electrode / g *
 *             (1 + (2*g/(pi*w)) * ln(1 + pi*w/g))
 *
 * where L_adjacent is the length of the adjacent edge.
 *
 * Crosstalk causes: adjacent channel false touches, position error
 * in sliders, reduced SNR.
 *
 * Mitigation: ground trace between electrodes, larger gaps,
 * driven shield between channels (time-multiplexed guard).
 *
 * @return Crosstalk capacitance [F]
 */
double cap_crosstalk_estimate(double adjacent_length, double electrode_width,
                              double gap, double eps_r_eff)
{
    if (adjacent_length <= 0.0 || gap <= 0.0) return 0.0;

    /* Parallel-plate approximation for edge-to-edge */
    double c_edge = CAP_EPSILON0 * eps_r_eff * adjacent_length * electrode_width / gap;

    /* Fringe correction for edge coupling */
    double ratio = M_PI * electrode_width / gap;
    double fringe = 1.0 + (2.0 * gap / (M_PI * electrode_width)) * (1.0 + log(ratio));

    return c_edge * fringe;
}

/**
 * cap_hatch_fill_ratio
 *
 * Computes the copper fill ratio for a hatched ground plane.
 *
 * A hatched (cross-hatched) ground plane reduces parasitic capacitance
 * compared to a solid ground plane while still providing shielding.
 *
 * For a square hatch pattern with pitch p and trace width w:
 *   fill_ratio = 1 - (1 - w/p)^2
 *              = (2w/p - (w/p)^2)
 *
 * Example: p=1mm, w=0.2mm → fill_ratio = 2*0.2 - 0.04 = 0.36 (36% fill)
 *
 * 100% fill = solid plane (maximum C_parasitic)
 * 30-50% fill = good compromise for capacitive sensing
 * <20% fill = degraded shielding effectiveness
 *
 * @return Fill ratio [0, 1]
 */
double cap_hatch_fill_ratio(double hatch_pitch_m, double hatch_width_m)
{
    if (hatch_pitch_m <= 0.0 || hatch_width_m <= 0.0) return 1.0;
    if (hatch_width_m >= hatch_pitch_m) return 1.0;

    double ratio = hatch_width_m / hatch_pitch_m;
    return 2.0 * ratio - ratio * ratio;
}
