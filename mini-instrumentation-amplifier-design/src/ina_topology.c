/**
 * @file ina_topology.c
 * @brief IA Topology Analysis Implementation
 *
 * Implements the three-op-amp, two-op-amp, current-mode, and
 * indirect-current-feedback IA topologies.
 *
 * Each topology represents different trade-offs in CMRR, noise,
 * bandwidth, power consumption, and cost.
 *
 * Reference:
 *   Kitchin & Counts, "A Designer's Guide to Instrumentation
 *     Amplifiers" (Analog Devices, 3rd Ed., 2006)
 *   Analog Devices AN-671: "Simplifying INA Design with
 *     the AD8221/AD8226"
 */
#include "ina_topology.h"
#include <math.h>
#include <float.h>

/*===========================================================================
 * 3-Op-Amp IA Implementation
 *
 * This is the classic, most widely used IA topology.
 *
 * Stage 1 (Pre-amplifier):
 *   Two non-inverting amplifiers (A1, A2) with shared gain resistor Rg.
 *   This stage provides:
 *     - High differential gain with unity common-mode gain
 *     - High input impedance (op-amp non-inverting inputs)
 *     - Gain adjustment via single resistor Rg
 *
 * Stage 2 (Difference amplifier):
 *   Unity-gain difference amplifier that rejects the common-mode
 *   voltage from Stage 1 and produces a single-ended output.
 *   The CMRR of the overall IA is dominated by resistor matching
 *   in this stage.
 *
 * Key advantage: The gain-setting resistor Rg does not need to
 * be matched to any other resistor. Only the four resistors in
 * Stage 2 need matching.
 *===========================================================================*/

double ina_3opamp_gain(double r_feedback, double r_gain)
{
    /**
     * Compute differential gain of 3-op-amp IA.
     *
     * G = 1 + 2*Rf / Rg
     *
     * Derivation (L3: Mathematical Structure):
     *
     * Stage 1 analysis:
     * - A1 non-inverting input: V1
     * - A2 non-inverting input: V2
     * - Virtual short at inverting inputs:
     *   A1? = V1, A2? = V2
     * - Voltage across Rg: V_Rg = V1 - V2 = V_dm
     * - Current through Rg: I_Rg = V_dm / Rg
     * - Same current flows through both Rf resistors (op-amp inputs
     *   draw negligible current)
     * - Output of A1: Vo1 = V1 + I_Rg * Rf = V1 + V_dm * Rf/Rg
     * - Output of A2: Vo2 = V2 - I_Rg * Rf = V2 - V_dm * Rf/Rg
     * - Stage 1 differential output:
     *     Vo1 - Vo2 = (V1 - V2) * (1 + 2*Rf/Rg)
     *               = V_dm * (1 + 2*Rf/Rg)
     * - Stage 1 common-mode output:
     *     (Vo1 + Vo2)/2 = (V1 + V2)/2 = V_cm  (unity CM gain)
     *
     * Stage 2 (unity-gain difference amplifier):
     * - Vout = Vo1 - Vo2 = V_dm * (1 + 2*Rf/Rg)
     * - Total gain: G = 1 + 2*Rf/Rg
     *
     * Reference: Sedra & Smith, ?8.6
     */
    if (r_gain <= 0.0) {
        return INFINITY;  /* No Rg ? max gain */
    }
    return 1.0 + 2.0 * r_feedback / r_gain;
}

double ina_3opamp_cmrr(double r_feedback, double r_gain,
                       double resistor_mismatch_percent,
                       double opamp_cmrr_db)
{
    /**
     * Compute total CMRR of a 3-op-amp IA.
     *
     * The CMRR is primarily limited by:
     *   1. Resistor matching in the Stage 2 difference amplifier
     *   2. Individual op-amp CMRR (less significant at high gains)
     *
     * For the difference amplifier with mismatch delta:
     *   CMRR_diffamp = (1 + R2/R1) / (4 * delta)
     *
     * For unity-gain diff amp (R2=R1): CMRR ? 1 / (2*delta)
     *
     * The Stage 1 gain improves overall CMRR because it amplifies
     * the differential signal before it encounters the Stage 2
     * mismatch errors:
     *   CMRR_total = G_stage1 * CMRR_diffamp
     *
     * More precisely, contributions add in quadrature:
     *   1/CMRR_total^2 = 1/(G*CMRR_opamp)^2 + 1/CMRR_res^2
     *
     * where G = 1 + 2*Rf/Rg is the total gain.
     *
     * The key insight (L2 Core Concept):
     *   CMRR improves with higher gain because the Stage 1
     *   pre-amplifies the differential signal relative to
     *   the common-mode errors introduced in Stage 2.
     *
     * This is why IA CMRR is specified at specific gains
     * (G=1, G=10, G=100, G=1000) - it gets better at higher gains.
     *
     * Reference: Pall?s-Areny & Webster, "Common Mode Rejection
     *   Ratio in Differential Amplifiers" (IEEE TIM, 1991)
     */
    double gain = ina_3opamp_gain(r_feedback, r_gain);

    /* Stage 2 diff amp is usually unity gain (R2/R1 = 1) */
    double r2_over_r1 = 1.0;
    double delta = resistor_mismatch_percent / 100.0;

    double cmrr_res_linear = ina_cmrr_from_resistor_mismatch(r2_over_r1, delta);

    /* Op-amp CMRR contribution reduced by gain */
    double cmrr_opamp_linear = pow(10.0, opamp_cmrr_db / 20.0);
    double cmrr_opamp_effective = cmrr_opamp_linear * gain;

    /* Total CMRR (linear) */
    double cmrr_total_linear = ina_cmrr_total(cmrr_res_linear,
                                               cmrr_opamp_effective,
                                               INFINITY);
    return 20.0 * log10(cmrr_total_linear);
}

double ina_3opamp_bandwidth(double r_feedback, double r_gain,
                            double opamp_gbw)
{
    /**
     * Compute -3dB bandwidth of a 3-op-amp IA.
     *
     * The bandwidth is limited by the dominant pole of the op-amps
     * in a gain-bandwidth product (GBW) relationship.
     *
     * Stage 1: Each non-inverting amplifier has noise gain:
     *   NG = 1 + Rf / (Rg/2)  (because Rg is split between two op-amps)
     *
     * For large gains: NG ? G/2
     *
     * BW_stage1 ? GBW / NG = 2*GBW / G
     *
     * Stage 2: Unity-gain diff amp:
     *   BW_stage2 ? GBW (if compensated for unity gain)
     *
     * Total bandwidth is limited by Stage 1 for G > 2.
     *
     * More precisely, for cascaded stages with BW1 and BW2:
     *   BW_total ? 1 / sqrt(1/BW1^2 + 1/BW2^2)
     *
     * Reference: Sedra & Smith, ?2.8
     */
    if (opamp_gbw <= 0.0) return 0.0;

    double gain = ina_3opamp_gain(r_feedback, r_gain);
    if (gain < 1.0) gain = 1.0;

    /* Stage 1 noise gain */
    double ng_stage1 = 1.0 + r_feedback / (r_gain / 2.0);
    if (ng_stage1 < 1.0) ng_stage1 = 1.0;
    double bw_stage1 = opamp_gbw / ng_stage1;

    /* Stage 2 unity gain bandwidth */
    double bw_stage2 = opamp_gbw;

    /* Cascaded bandwidth */
    double bw_total = 1.0 / sqrt(1.0/(bw_stage1*bw_stage1)
                                + 1.0/(bw_stage2*bw_stage2));

    return bw_total;
}

int ina_3opamp_check_cm_range(double v_cm, double v_dm,
                              double r_feedback, double r_gain,
                              double supply_voltage,
                              double opamp_cm_min, double opamp_cm_max)
{
    /**
     * Check if input signals stay within the IA's common-mode range.
     *
     * This is a critical L2 concept: The "diamond plot" or
     * "input common-mode range vs output voltage" characteristic.
     *
     * For a 3-op-amp IA, the Stage 1 outputs are:
     *   Vo1 = V_cm + V_dm/2 * (1 + 2*Rf/Rg)
     *   Vo2 = V_cm - V_dm/2 * (1 + 2*Rf/Rg)
     *
     * These must stay within the op-amp output range (typically
     * supply rails minus 0.1-1V saturation).
     *
     * The input common-mode at the op-amp inputs is just V_cm,
     * which must be within the op-amp input CM range.
     *
     * When either Vo1 or Vo2 saturates, the IA output is
     * no longer valid. This creates the "diamond plot" constraint:
     * higher differential signals reduce the allowable common-mode
     * range, and vice versa.
     *
     * Returns: 0 = OK, -1 = below range, +1 = above range
     */
    double gain = ina_3opamp_gain(r_feedback, r_gain);
    double vo1 = v_cm + v_dm / 2.0 * gain;
    double vo2 = v_cm - v_dm / 2.0 * gain;

    /* Check input CM range */
    if (v_cm < opamp_cm_min || v_cm > opamp_cm_max) {
        return (v_cm < opamp_cm_min) ? -1 : 1;
    }

    /* Check stage 1 outputs (must be within supply rails) */
    double v_sat = 0.2;  /* typical saturation margin */
    if (vo1 > supply_voltage - v_sat || vo2 > supply_voltage - v_sat) {
        return 1;  /* Above range */
    }
    if (vo1 < -supply_voltage + v_sat || vo2 < -supply_voltage + v_sat) {
        return -1; /* Below range */
    }

    return 0;  /* Within range */
}

double ina_3opamp_output_offset(double vos1_uv, double vos2_uv,
                                double vos3_uv, double r_feedback,
                                double r_gain)
{
    /**
     * Compute output offset of 3-op-amp IA including all op-amp offsets.
     *
     * Each op-amp has its own input offset voltage:
     *   A1: Vos1
     *   A2: Vos2
     *   A3: Vos3 (difference amplifier)
     *
     * The output offset contribution from each:
     *   A1: Vos1_appears at output = Vos1 * (1 + Rf/Rg) ? Vos1 * G/2
     *   A2: Vos2_appears at output = Vos2 * (1 + Rf/Rg) ? Vos2 * G/2
     *   A3: Vos3 appears directly at output (Stage 2 gain = 1)
     *
     * Comprehensive output offset (RTI equivalent):
     *   Vos_RTI = (Vos1 - Vos2)/2 + Vos3/G
     *
     * Note the subtraction of Vos1 and Vos2: if A1 and A2 have
     * matched offsets (common in monolithic implementations),
     * the Stage 1 offset contribution cancels to first order.
     *
     * This cancellation is a key advantage of the 3-op-amp topology
     * over the 2-op-amp topology.
     *
     * Using worst-case (same sign on Vos1 and Vos2):
     *   Vout_offset_max = fabs(Vos1+Vos2)*(1+Rf/Rg) + fabs(Vos3)
     */
    double gain_stage1 = ina_3opamp_gain(r_feedback, r_gain);
    double gain_per_opamp = (gain_stage1 + 1.0) / 2.0;

    /* Output offset from A1, A2, A3 */
    double offset_a1 = vos1_uv * gain_per_opamp;
    double offset_a2 = vos2_uv * gain_per_opamp;
    double offset_a3 = vos3_uv;  /* Stage 2 is unity gain */

    /* RSS combination (uncorrelated) */
    return sqrt(offset_a1*offset_a1 + offset_a2*offset_a2
                + offset_a3*offset_a3);
}

/*===========================================================================
 * 2-Op-Amp IA Implementation
 *
 * Uses only two op-amps, saving power and cost.
 *
 * Architecture:
 *   A1: Non-inverting amplifier with feedback through R1 and R2
 *   A2: Inverting amplifier stage
 *   Rg: Gain-setting resistor between A1 output and A2?
 *
 * Gain: G = 1 + R1/R2 + 2*R1/Rg
 *
 * Disadvantage: Asymmetric signal path ? common-mode signal sees
 * different transfer functions through the two paths, degrading CMRR.
 *
 * Input impedance is high only at A1's non-inverting input.
 * A2's input is virtual ground, presenting low impedance.
 *===========================================================================*/

double ina_2opamp_gain(double r1, double r2, double r_gain)
{
    /**
     * Compute gain of 2-op-amp IA.
     *
     * G = 1 + R1/R2 + 2*R1/Rg
     *
     * Derivation (L3: Mathematical Structure):
     *
     * Let V1 = non-inverting input, V2 = inverting input (A2? via R2)
     *
     * A1 non-inverting, feedback via R1 and Rg:
     *   Vo1 = V1 * (1 + R1/(Rg || R2))
     *
     * More directly (KCL at A2?):
     *   (Vo1 - V?)/Rg + (V2 - V?)/R2 = V?/R1
     *   where V? = A2? = A2? = 0 (virtual ground)
     *
     *   Vo1/Rg + V2/R2 = -Vo/R1  (Vo = A2 output = -V?*Aol)
     *
     * For the ideal case:
     *   Vout = -V2*(R1/R2) - Vo1*(R1/Rg)
     *
     * Substituting Vo1 and solving:
     *   G = Vout/(V1-V2) = 1 + R1/R2 + 2*R1/Rg
     *
     * Reference: Sedra & Smith, Problem 2.83
     */
    if (r_gain <= 0.0) return INFINITY;
    if (r2 <= 0.0) return INFINITY;
    return 1.0 + r1/r2 + 2.0 * r1 / r_gain;
}

double ina_2opamp_cmrr(double r1, double r2, double r_gain,
                       double resistor_mismatch,
                       double opamp_cmrr_db)
{
    /**
     * Compute CMRR of 2-op-amp IA.
     *
     * The 2-op-amp IA has intrinsically lower CMRR than the
     * 3-op-amp IA because the common-mode signal travels through
     * different paths with different gains to the two op-amp inputs.
     *
     * For perfectly matched resistors:
     *   CMRR_2opamp ? G * CMRR_opamp / 2
     *
     * This is approximately half the CMRR of a 3-op-amp IA
     * at the same gain. In practice, the asymmetry makes CMRR
     * 20-30 dB worse than the 3-op-amp topology.
     *
     * The CMRR is also more sensitive to resistor matching
     * because all three resistors (R1, R2, Rg) affect the
     * common-mode transfer function.
     */
    double gain = ina_2opamp_gain(r1, r2, r_gain);
    double opamp_cmrr_linear = pow(10.0, opamp_cmrr_db / 20.0);

    /* Resistor mismatch effect (approximate) */
    double delta = resistor_mismatch / 100.0;
    double cmrr_res_linear = (1.0 + r1/r2) / (2.0 * delta);

    /* Total */
    double cmrr_total_linear = ina_cmrr_total(cmrr_res_linear,
                                               opamp_cmrr_linear * gain,
                                               INFINITY);
    return 20.0 * log10(cmrr_total_linear);
}

double ina_2opamp_bandwidth(double r1, double r2, double r_gain,
                            double opamp_gbw)
{
    /**
     * Compute bandwidth of 2-op-amp IA.
     *
     * A1 noise gain: NG1 = 1 + R1/(Rg||R2)
     * A2 noise gain: NG2 = 1 + R1/R2 + R1/Rg
     *
     * The bandwidth is limited by the higher noise gain stage.
     * For typical values, A2 has higher noise gain.
     *
     * BW ? GBW / NG2
     */
    if (opamp_gbw <= 0.0) return 0.0;
    if (r2 <= 0.0 || r_gain <= 0.0) return 0.0;

    double r_parallel = (r_gain * r2) / (r_gain + r2);
    double ng1 = 1.0 + r1 / r_parallel;
    double ng2 = 1.0 + r1/r2 + r1/r_gain;

    double ng_max = (ng1 > ng2) ? ng1 : ng2;
    if (ng_max <= 0.0) return opamp_gbw;
    return opamp_gbw / ng_max;
}

/*===========================================================================
 * Current-Mode IA Implementation
 *
 * Current-mode IAs convert the differential input voltage to a
 * proportional current, amplify the current, and convert back to
 * voltage. This architecture naturally achieves higher bandwidth
 * because current-mode circuits have fewer high-impedance nodes.
 *===========================================================================*/

double ina_current_mode_gain(double r_feedback, double r_gain)
{
    /**
     * Gain of current-mode IA.
     *
     * Vout = (V1 - V2) * R_feedback / R_gain
     * G = R_feedback / R_gain
     *
     * Unlike the 3-op-amp IA, gain is directly proportional to
     * the resistor ratio (no "+1" term).
     *
     * This simpler relationship means gain can be set precisely
     * by resistor ratios, similar to a difference amplifier
     * but with the high input impedance of an IA.
     */
    if (r_gain <= 0.0) return INFINITY;
    return r_feedback / r_gain;
}

double ina_current_mode_bandwidth(double gm, double rout,
                                  double c_parasitic)
{
    /**
     * Compute bandwidth of current-mode IA.
     *
     * The bandwidth is determined by the dominant pole at the
     * high-impedance output node:
     *   BW = 1 / (2*pi * rout * c_parasitic)
     *
     * The transconductance gm sets the gain but does not directly
     * limit bandwidth in a well-designed current-mode circuit.
     *
     * For a given gain G = gm * rout:
     *   BW = gm / (2*pi * G * c_parasitic)
     *
     * This shows that current-mode IAs can achieve constant
     * gain-bandwidth product, similar to voltage-mode op-amps
     * but typically with higher bandwidth for the same gain
     * because parasitic capacitances are smaller.
     */
    (void)gm; if (rout <= 0.0 || c_parasitic <= 0.0) return 0.0;
    return 1.0 / (2.0 * M_PI * rout * c_parasitic);
}

double ina_current_mode_cmrr(double gm, double gm_mismatch_percent)
{
    /**
     * CMRR of current-mode IA.
     *
     * CMRR is primarily limited by transconductance mismatch
     * between the two input stages.
     *
     * CMRR ? 1 / (?gm/gm) = 100 / mismatch_percent
     *
     * For 1% gm matching: CMRR ? 40 dB
     * For 0.1% gm matching: CMRR ? 60 dB
     *
     * Monolithic implementations achieve 0.01-0.1% matching
     * through careful layout and trimming.
     */
    double delta = gm_mismatch_percent / 100.0;
    (void)gm; if (delta <= 0.0) return INFINITY;
    return 20.0 * log10(1.0 / delta);
}

/*===========================================================================
 * Indirect Current Feedback IA Implementation
 *
 * The ICF architecture (AD8221, AD8226, AD8429) separates the
 * differential and common-mode signal paths using transconductance
 * amplifiers, eliminating the need for precision resistor matching
 * in a difference amplifier.
 *===========================================================================*/

double ina_icf_gain(double gm_input, double gm_feedback,
                    double r_gain, double r_output)
{
    /**
     * Gain of ICF (Indirect Current Feedback) IA.
     *
     * In the ICF architecture:
     *   1. Input gm stages convert V_diff to current: I_in = gm_input * V_diff
     *   2. Feedback gm stages sense output: I_fb = gm_feedback * V_out * beta
     *   3. At balance: I_in = I_fb ? V_out = (gm_input/gm_feedback) * V_diff/beta
     *   4. With beta = Rg/(Rg + 2*R_internal):
     *      G = (gm_input/gm_feedback) * (1 + 2*R_internal/Rg)
     *      G ? 1 + 2*R_internal/Rg  (if gm_input = gm_feedback)
     *
     * The key advantage is that the output is taken from a
     * current summing node, not a voltage difference amplifier.
     * This eliminates the resistor matching requirement for CMRR.
     */
    if (gm_feedback <= 0.0 || r_gain <= 0.0) return 0.0;
    double __attribute__((unused)) beta = 1.0 / ina_icf_gain(gm_input, gm_feedback, r_gain, 1.0);
    /* Simplified: assume gm ratios = 1 */
    double g = 1.0 + 2.0 * r_output / r_gain;
    return g;
}

double ina_icf_cmrr(double gm_input, double gm_feedback,
                    double gm_mismatch)
{
    /**
     * CMRR of ICF IA.
     *
     * The ICF topology achieves high CMRR without precision resistors
     * because the common-mode signal is rejected by the gm stages
     * themselves through active feedback, not by passive resistor matching.
     *
     * CMRR ? (gm_input / gm_mismatch) / (1 + loop_gain)
     *
     * The high loop gain at DC provides excellent low-frequency CMRR
     * (>120 dB is achievable).
     *
     * CMRR rolls off at 20 dB/decade above the CMRR pole.
     */
    (void)gm_feedback; double mismatch_ratio = gm_mismatch / gm_input;
    if (mismatch_ratio <= 0.0) return INFINITY;
    return 20.0 * log10(1.0 / mismatch_ratio);
}

double ina_icf_loop_gain(double gm_input, double gm_feedback,
                          double r_gain, double beta)
{
    /**
     * Compute loop gain of ICF IA.
     *
     * Loop gain determines the accuracy of the closed-loop transfer
     * function and the effectiveness of distortion suppression.
     *
     * T = gm_feedback * r_equiv * beta
     *
     * where r_equiv is the equivalent impedance at the
     * current summing node.
     *
     * Higher loop gain ? better linearity, higher CMRR,
     * lower output impedance.
     */
    (void)gm_input; double r_equiv = r_gain;  /* simplified */
    return gm_feedback * r_equiv * beta;
}

/*===========================================================================
 * L6: Topology Selection (Canonical Design Problem)
 *===========================================================================*/

InaTopology ina_select_topology(double required_cmrr_db,
                                double required_bandwidth_khz,
                                double required_gain,
                                double power_budget_mw,
                                double cost_budget)
{
    (void)required_gain;
    /**
     * Canonical L6 problem: select the optimal IA topology.
     *
     * Decision matrix:
     *
     *                     CMRR     BW       Power   Cost   Complexity
     *   3-op-amp          ?????    ???     ???     ??     ??
     *   2-op-amp          ???      ???     ????    ????   ?
     *   Current-mode      ????     ?????   ???     ???    ???
     *   Flying-cap        ????     ??      ??      ??     ?????
     *   ICF               ?????    ????    ???     ??     ????
     *
     * Selection algorithm (L5):
     *   1. Filter by hard requirements (CMRR, BW, Gain)
     *   2. Score remaining options by soft requirements (power, cost)
     *   3. Return best match
     *
     * This embodies real-world IA selection engineering judgment.
     */
    /* Score structure: [topology_index] = score */
    double scores[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

    /* 3-op-amp: best all-around */
    if (required_cmrr_db <= 130.0 && required_bandwidth_khz <= 1000.0) {
        scores[INA_TOPOLOGY_3OPAMP] = 90.0;
    }

    /* 2-op-amp: low power/cost, moderate performance */
    if (required_cmrr_db <= 90.0 && required_bandwidth_khz <= 500.0) {
        scores[INA_TOPOLOGY_2OPAMP] = 80.0;
        /* Bonus for low power/cost */
        if (power_budget_mw < 10.0 || cost_budget < 2.0) {
            scores[INA_TOPOLOGY_2OPAMP] += 10.0;
        }
    }

    /* Current-mode: best bandwidth */
    if (required_cmrr_db <= 110.0 && required_bandwidth_khz <= 10000.0) {
        scores[INA_TOPOLOGY_CURRENT_MODE] = 85.0;
        if (required_bandwidth_khz > 1000.0) {
            scores[INA_TOPOLOGY_CURRENT_MODE] += 10.0;
        }
    }

    /* Flying-cap: specialty, very low drift */
    if (required_cmrr_db <= 120.0) {
        scores[INA_TOPOLOGY_FLYING_CAP] = 50.0;
    }

    /* ICF: best CMRR without precision resistors */
    if (required_cmrr_db <= 140.0 && required_bandwidth_khz <= 2000.0) {
        scores[INA_TOPOLOGY_INDIRECT_CURRENT] = 95.0;
    }

    /* Find highest score */
    InaTopology best = INA_TOPOLOGY_3OPAMP;
    double best_score = -1.0;
    for (int i = 0; i < 5; i++) {
        if (scores[i] > best_score) {
            best_score = scores[i];
            best = (InaTopology)i;
        }
    }

    return best;
}

double ina_topology_figure_of_merit(double cmrr_db, double bandwidth_khz,
                                    double power_mw, double cost_factor)
{
    /**
     * Figure of Merit (FOM) for IA topology comparison.
     *
     * FOM = CMRR_dB * BW_kHz / (Power_mW * Cost_factor)
     *
     * Higher FOM = better performance per unit cost and power.
     *
     * Example comparison:
     *   3-op-amp: 120 dB * 500 kHz / (5 mW * 1.5) = 8000
     *   2-op-amp:  85 dB * 300 kHz / (3 mW * 1.0) = 8500
     *
     * The 2-op-amp wins on efficiency despite lower raw performance.
     * This is the engineering trade-off captured by FOM.
     */
    if (power_mw <= 0.0 || cost_factor <= 0.0) return INFINITY;
    return cmrr_db * bandwidth_khz / (power_mw * cost_factor);
}

double ina_max_resistor_mismatch_for_cmrr(double target_cmrr_db,
                                          double r_feedback,
                                          double r_gain)
{
    /**
     * Maximum allowable resistor mismatch for target CMRR.
     *
     * For 3-op-amp IA with unity-gain Stage 2:
     *   CMRR_target = -20*log10(4*delta / (1 + 2*Rf/Rg))
     *
     * Solving for delta:
     *   delta_max = (1 + 2*Rf/Rg) / (4 * 10^(CMRR_target/20))
     *
     * This tells us what resistor tolerance grade we need.
     *
     * Example:
     *   G=100, target CMRR=100dB:
     *   delta_max = 100 / (4 * 10^5) = 2.5e-4 = 0.025%
     *   ? need 0.01% resistors (C8 or better grade)
     *
     *   G=10, target CMRR=80dB:
     *   delta_max = 10 / (4 * 10^4) = 2.5e-4 = 0.025%
     *   ? also need 0.01% for Stage 2 resistors
     *
     * Practical insight: Stage 2 resistor matching dominates CMRR
     * at all gains. High gain helps, but the Stage 2 matching is
     * always the limiting factor in practice.
     */
    double gain = ina_3opamp_gain(r_feedback, r_gain);
    double cmrr_linear = pow(10.0, target_cmrr_db / 20.0);
    if (cmrr_linear <= 0.0) return 1.0;
    return gain / (4.0 * cmrr_linear);
}
