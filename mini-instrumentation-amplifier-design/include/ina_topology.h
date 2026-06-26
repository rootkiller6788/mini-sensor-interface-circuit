/**
 * @file ina_topology.h
 * @brief Instrumentation Amplifier Topology Analysis
 *
 * Covers L2 Core Concepts + L5 Algorithms for different IA topologies.
 * Each topology represents a different trade-off in CMRR, noise, power,
 * and input range.
 *
 * Reference:
 *   Kitchin & Counts, "A Designer's Guide to Instrumentation Amplifiers"
 *     (Analog Devices, 3rd Ed., 2006)
 *   Sedra & Smith, "Microelectronic Circuits" (2020), Ch. 8
 *
 * @course-alignment
 *   MIT 6.002: Differential amplifier analysis (2-op-amp, 3-op-amp)
 *   Berkeley EE140 Linear IC: Op-amp topologies and feedback
 *   Stanford EE315: Analog CMOS IC design - IA architectures
 *   TU Munich: High-Frequency Engineering - Current-mode circuits
 */
#ifndef INA_TOPOLOGY_H
#define INA_TOPOLOGY_H

#include "ina_core.h"

/*===========================================================================
 * Three-Op-Amp Instrumentation Amplifier (Classic Topology)
 *
 * Circuit:
 *   Stage 1: Two non-inverting amplifiers with shared Rg
 *     - A1 buffers V1+, A2 buffers V1-
 *     - Rg between A1 and A2 inverting inputs sets gain
 *     - Each op-amp has feedback resistor Rf
 *   Stage 2: Unity-gain difference amplifier
 *     - Converts differential to single-ended
 *     - Rejects common-mode from Stage 1
 *
 * Gain equation: G = 1 + 2*Rf/Rg
 *
 * CMRR: Limited primarily by resistor matching in Stage 2.
 *   CMRR_total ~= (1 + 2*Rf/Rg) / (4 * delta_R/R)
 *   for delta_R = fractional mismatch in difference amplifier resistors.
 *===========================================================================*/

/**
 * @brief 3-op-amp IA configuration parameters
 */
typedef struct {
    double r_feedback;        /**< Feedback resistor Rf (ohm) in Stage 1 */
    double r_gain;            /**< Gain-setting resistor Rg (ohm) */
    double r_diff_amp[4];     /**< Stage 2 difference amplifier resistors:
                                   r_diff_amp[0]=R1, [1]=R2, [2]=R3, [3]=R4
                                   where Vout = (R3/R1)*(R1+R2)/(R3+R4)*V2
                                                - (R2/R1)*V1 */
    double opamp_gbw;         /**< Individual op-amp GBW (Hz) */
    double opamp_cmrr_db;     /**< Individual op-amp CMRR (dB) */
    double opamp_aol_db;      /**< Individual op-amp open-loop gain (dB) */
} InaTopology3Opamp;

/**
 * @brief Compute gain of 3-op-amp IA
 *
 * G = 1 + 2*Rf/Rg
 *
 * Derivation (L3 Mathematical Structure):
 *   Stage 1: A1, A2 form symmetric non-inverting amplifiers.
 *   Voltage across Rg = V1 - V2 = V_dm.
 *   Current through Rg: I_Rg = V_dm / Rg.
 *   This same current flows through both Rf resistors.
 *   V_stage1_diff = (2*Rf + Rg) * V_dm / Rg = (1 + 2*Rf/Rg) * V_dm.
 *   Stage 2: Difference amplifier with gain = 1.
 *   Total gain: G = V_stage1_diff / V_dm = 1 + 2*Rf/Rg.
 */
double ina_3opamp_gain(double r_feedback, double r_gain);

/**
 * @brief Compute CMRR of 3-op-amp IA
 *
 * The total CMRR depends on:
 *   1. Gain of Stage 1: higher gain ? better CMRR
 *   2. Resistor matching in Stage 2
 *   3. Individual op-amp CMRR
 *
 * CMRR_total(dB) = -20*log10( sqrt( 1/(CMRR_res?) + 1/(G*CMRR_opamp)? ) )
 *
 * where:
 *   CMRR_res = (1+G_stage1) / (4*delta_R)
 *   G = total gain
 *   delta_R = fractional mismatch
 */
double ina_3opamp_cmrr(double r_feedback, double r_gain,
                       double resistor_mismatch_percent,
                       double opamp_cmrr_db);

/**
 * @brief Analyze 3-op-amp IA bandwidth
 *
 * The closed-loop bandwidth for each stage:
 *   Stage 1: BW = GBW_opamp / (1 + Rf/(Rg/2))
 *            = GBW_opamp / G_stage1 (approx, for large G)
 *   Stage 2: BW = GBW_opamp / 1 = GBW_opamp (unity gain)
 *
 * Total bandwidth is limited by the slower of the two stages,
 * typically Stage 1 at high gains.
 *
 * @param r_feedback Stage 1 feedback resistor (ohm)
 * @param r_gain Gain-setting resistor (ohm)
 * @param opamp_gbw Individual op-amp GBW (Hz)
 * @return Estimated -3dB bandwidth (Hz)
 */
double ina_3opamp_bandwidth(double r_feedback, double r_gain,
                            double opamp_gbw);

/**
 * @brief Analyze input common-mode range of 3-op-amp IA
 *
 * The input common-mode range is limited by:
 *   1. Op-amp input common-mode range
 *   2. Output swing of A1, A2 Stage 1
 *   3. Supply voltage
 *
 * Vcm_max = min(Vsupply - Vout_A1, V_opamp_cm_max)
 * Vout_A1 = Vcm + Vdm/2 * G_stage1
 *
 * For large differential signals or high gains, Stage 1 outputs
 * can saturate even with small common-mode voltages.
 *
 * @return 0 if input is within range, -1 if below, +1 if above
 */
int ina_3opamp_check_cm_range(double v_cm, double v_dm,
                              double r_feedback, double r_gain,
                              double supply_voltage,
                              double opamp_cm_min, double opamp_cm_max);

/**
 * @brief Compute output offset voltage of 3-op-amp IA
 *
 * Vout_offset = Vos1 * G_stage1 - Vos2 * G_stage1 + Vos3
 *
 * where Vos1, Vos2 are offsets of Stage 1 op-amps and Vos3 is
 * the offset of the Stage 2 difference amplifier.
 */
double ina_3opamp_output_offset(double vos1_uv, double vos2_uv,
                                double vos3_uv, double r_feedback,
                                double r_gain);

/*===========================================================================
 * Two-Op-Amp Instrumentation Amplifier
 *
 * Circuit:
 *   Uses only 2 op-amps (no difference amplifier stage).
 *   Advantage: Lower power, lower cost.
 *   Disadvantage: Lower CMRR, asymmetric signal path ? more distortion.
 *
 * Gain: G = 1 + R1/R2 + 2*R1/Rg
 *
 * CMRR: Limited by resistor matching and asymmetry.
 *   For perfectly matched resistors, CMRR ~= G * CMRR_opamp.
 *   In practice, ~20-30 dB worse than 3-op-amp topology.
 *
 * Reference: Kitchin & Counts, ADI Application Note AN-671
 *===========================================================================*/

typedef struct {
    double r1;                /**< Feedback resistor R1 (ohm) */
    double r2;                /**< Second resistor R2 (ohm) */
    double r_gain;            /**< Gain-setting resistor Rg (ohm) */
    double opamp_gbw;         /**< Op-amp GBW (Hz) */
    double opamp_cmrr_db;     /**< Op-amp CMRR (dB) */
} InaTopology2Opamp;

double ina_2opamp_gain(double r1, double r2, double r_gain);

double ina_2opamp_cmrr(double r1, double r2, double r_gain,
                       double resistor_mismatch,
                       double opamp_cmrr_db);

double ina_2opamp_bandwidth(double r1, double r2, double r_gain,
                            double opamp_gbw);

/*===========================================================================
 * Current-Mode Instrumentation Amplifier
 *
 * Principle:
 *   Uses current mirrors and translinear circuits instead of
 *   voltage-mode op-amps. Input voltage difference is converted
 *   to current, amplified, and converted back to voltage.
 *
 * Advantages:
 *   - Wider bandwidth (current-mode circuits have fewer high-impedance nodes)
 *   - Better high-frequency CMRR
 *   - Can operate at lower supply voltages
 *
 * Example: AD8221 uses an indirect current feedback architecture.
 *
 * Transfer function: Vout = (V1 - V2) * (R_feedback / R_gain) + Vref
 *===========================================================================*/

typedef struct {
    double r_gain;            /**< Transconductance-setting resistor (ohm) */
    double r_feedback;        /**< I-to-V conversion resistor (ohm) */
    double gm;                /**< Input stage transconductance (S) */
    double rout;              /**< Output stage resistance (ohm) */
} InaTopologyCurrentMode;

double ina_current_mode_gain(double r_feedback, double r_gain);

double ina_current_mode_bandwidth(double gm, double rout,
                                  double c_parasitic);

double ina_current_mode_cmrr(double gm, double gm_mismatch_percent);

/*===========================================================================
 * Indirect Current Feedback (ICF) Topology
 *
 * Used in modern precision IAs (AD8221, AD8226, AD8429).
 *
 * The ICF architecture separates the differential signal path from
 * the common-mode feedback, achieving high CMRR without requiring
 * precision resistor matching in a difference amplifier.
 *
 * Key insight: Instead of converting CM from Stage 1 outputs via
 * a resistor network (which requires matching), the ICF uses
 * transconductance amplifiers to sense and subtract the CM component
 * through active feedback.
 *
 * Reference: Brokaw, "An Improved Monolithic Instrumentation Amplifier"
 *   (U.S. Patent 4,714,894, 1987)
 *===========================================================================*/

typedef struct {
    double gm_input;          /**< Input stage transconductance (S) */
    double gm_feedback;       /**< Feedback stage transconductance (S) */
    double r_gain;            /**< Gain resistor (ohm) */
    double r_output;          /**< Output resistor (ohm) */
    double beta;              /**< Feedback factor */
} InaTopologyIndirectCurrent;

double ina_icf_gain(double gm_input, double gm_feedback,
                    double r_gain, double r_output);

double ina_icf_cmrr(double gm_input, double gm_feedback,
                    double gm_mismatch);

double ina_icf_loop_gain(double gm_input, double gm_feedback,
                          double r_gain, double beta);

/*===========================================================================
 * L6: Canonical Problem - Topology Selection
 *===========================================================================*/

/**
 * @brief Select optimal IA topology for given specifications
 *
 * This is a canonical design problem (L6): Given a set of requirements
 * (gain, bandwidth, CMRR, power, cost), select the best IA topology.
 *
 * Selection criteria (weighted scoring):
 *   - High CMRR (>100 dB) ? 3-op-amp or ICF
 *   - Low power ? 2-op-amp
 *   - High bandwidth (>1 MHz) ? Current-mode or ICF
 *   - Low cost ? 2-op-amp or 3-op-amp with low-grade resistors
 *
 * @return Recommended topology
 */
InaTopology ina_select_topology(double required_cmrr_db,
                                double required_bandwidth_khz,
                                double required_gain,
                                double power_budget_mw,
                                double cost_budget);

/**
 * @brief Compare two topologies quantitatively
 *
 * Returns a figure-of-merit:
 *   FOM = CMRR(dB) * BW(kHz) / (Power(mW) * Cost_factor)
 */
double ina_topology_figure_of_merit(double cmrr_db, double bandwidth_khz,
                                    double power_mw, double cost_factor);

/**
 * @brief Determine if resistor matching is adequate for target CMRR
 *
 * For a 3-op-amp IA:
 *   CMRR_target <= -20*log10(4*delta_R / (1 + 2*Rf/Rg))
 *
 * Solving for delta_R:
 *   delta_R_max = (1 + 2*Rf/Rg) / (4 * 10^(CMRR_target/20))
 */
double ina_max_resistor_mismatch_for_cmrr(double target_cmrr_db,
                                          double r_feedback,
                                          double r_gain);

#endif /* INA_TOPOLOGY_H */
