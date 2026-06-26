/**
 * @file    tia_stability.h
 * @brief   TIA Stability & Compensation ? L2 Concepts + L5 Methods
 *
 * @details Stability analysis and compensation techniques for
 *          transimpedance amplifiers. Covers phase margin analysis,
 *          pole-zero placement, feedback compensation strategies,
 *          and stability criteria (Nyquist, Bode, Routh-Hurwitz).
 *
 * Knowledge Mapping:
 *   L1 - Definitions:
 *     - Phase margin (PM) and gain margin (GM)
 *     - Crossover frequency f_c (unity loop gain)
 *     - Pole frequency f_p and zero frequency f_z
 *     - Damping factor zeta and natural frequency omega_n
 *     - Quality factor Q of the closed-loop response
 *   L2 - Core Concepts:
 *     - Negative feedback and the Barkhausen criterion
 *     - Loop gain A_OL * beta (return ratio)
 *     - Noise gain vs signal gain
 *     - Compensation: lead, lag, lead-lag networks
 *     - Pole splitting and Miller effect
 *   L3 - Mathematical Structures:
 *     - Transfer functions in s-domain (Laplace)
 *     - Root locus analysis for feedback systems
 *     - Bode plot construction and interpretation
 *     - Nyquist stability criterion
 *   L4 - Fundamental Laws:
 *     - Bode stability criterion: PM > 0 for stable closed loop
 *     - Nyquist stability criterion: N = Z - P
 *     - Routh-Hurwitz stability criterion
 *     - Gain-bandwidth product constancy for compensated op-amps
 *   L5 - Algorithms/Methods:
 *     - Phase margin calculation from loop gain
 *     - Compensation capacitor design (C_f selection)
 *     - Pole-zero cancellation technique
 *     - In-the-loop compensation (op-amp decompensated type)
 *
 * References:
 *   - Gray, Hurst, Lewis, Meyer, "Analysis & Design of Analog ICs" (5th ed, 2009)
 *   - Sedra & Smith (2020), Ch.10-11
 *   - Graeme (1996), Ch.3
 *   - Bode, "Network Analysis and Feedback Amplifier Design" (1945)
 */

#ifndef TIA_STABILITY_H
#define TIA_STABILITY_H

#include "tia_core.h"

/* ??? L1: Stability Analysis Types ????????????????????????????????????????? */

/**
 * @brief Stability criterion enumeration.
 */
typedef enum {
    STABILITY_STABLE           = 0,
    STABILITY_MARGINALLY_STABLE = 1,
    STABILITY_UNSTABLE         = 2,
    STABILITY_CONDITIONALLY_STABLE = 3
} stability_status_t;

/**
 * @brief Compensation network topology.
 */
typedef enum {
    COMPENSATION_SIMPLE_CF    = 0,
    COMPENSATION_RC_LEAD      = 1,
    COMPENSATION_RC_LAG       = 2,
    COMPENSATION_TYPE_II      = 3,
    COMPENSATION_TYPE_III     = 4,
    COMPENSATION_FEEDFORWARD  = 5,
    COMPENSATION_MILLER       = 6
} compensation_type_t;

/**
 * @brief Pole-zero pair representation (s-domain).
 *
 *        Poles:  denominator roots of T(s), response decays as exp(-p*t)
 *        Zeros:  numerator roots of T(s), counteracts pole effects
 */
typedef struct {
    double freq_hz;
    double q_factor;
    int    is_pole;
    char   origin[32];
} pole_zero_t;

/**
 * @brief Collection of poles and zeros for transfer function analysis.
 */
typedef struct {
    size_t num_poles;
    size_t num_zeros;
    size_t max_pz;
    pole_zero_t *poles;
    pole_zero_t *zeros;
    double dc_gain_db;
} pole_zero_map_t;

/* ??? L1: Compensation Network Parameters ?????????????????????????????????? */

/**
 * @brief Compensation network design parameters.
 */
typedef struct {
    compensation_type_t type;
    double cf_pf;
    double rf_ohm;
    double r_comp_ohm;
    double c_comp_pf;
    double r_comp2_ohm;
    double c_comp2_pf;
    double phase_boost_deg;
    double freq_zero_hz;
    double freq_pole_hz;
} compensation_network_t;

/* ??? L2: Loop Gain Analysis ??????????????????????????????????????????????? */

/**
 * @brief Loop gain analysis results.
 *
 *        Loop gain T(s) = A_OL(s) * beta(s)
 *        where beta(s) = V_- / V_out is the feedback factor.
 *
 *        For TIA: beta(s) = Z_in(s) / (Z_in(s) + Z_f(s))
 *        where Z_in = 1/(j*omega*C_in), Z_f = R_f || (1/(j*omega*C_f))
 */
typedef struct {
    double loop_gain_dc_db;
    double crossover_freq_hz;
    double phase_at_crossover_deg;
    double phase_margin_deg;
    double gain_margin_db;
    double freq_neg180_hz;
    double loop_gain_at_neg180_db;
    stability_status_t status;
    double damping_factor;
    double natural_freq_hz;
    double quality_factor;
    double closed_loop_peaking_db;
} loop_gain_analysis_t;

/* ??? L2: Phase Margin Sweep ??????????????????????????????????????????????? */

/**
 * @brief Phase margin vs C_f sweep data.
 */
typedef struct {
    size_t num_points;
    double *cf_values;
    double *phase_margin;
    double *bandwidth;
    double *peaking;
    double *damping_factor;
    double cf_critical;
    double cf_optimal;
    double pm_optimal;
} phase_margin_sweep_t;

/* ??? L3: Root Locus Data ?????????????????????????????????????????????????? */

/**
 * @brief Root locus analysis for closed-loop poles.
 */
typedef struct {
    size_t num_points;
    double *gain_values;
    double *real_part;
    double *imag_part;
    double *damping_ratio;
    double *natural_freq;
    double gain_for_critical_damping;
    size_t num_branches;
} root_locus_data_t;

/* ??? L4: Nyquist Analysis ????????????????????????????????????????????????? */

/**
 * @brief Nyquist plot data.
 */
typedef struct {
    size_t num_points;
    double *real_part;
    double *imag_part;
    double *freq_hz;
    int    encirclements;
    int    open_loop_rhp_poles;
    int    closed_loop_rhp_poles;
    stability_status_t nyquist_status;
} nyquist_data_t;

/* ??? Function Declarations ? Stability ???????????????????????????????????? */

/**
 * @brief  Compute loop gain analysis for TIA design.
 * @param  design  TIA design
 * @return         Complete loop gain analysis
 *
 * @note   Calculates:
 *         - Crossover frequency from intersection of AOL and 1/beta
 *         - Phase margin at crossover
 *         - Gain margin at -180 degree frequency
 *         - Closed-loop damping factor and peaking
 */
loop_gain_analysis_t  tia_loop_gain_analyze(const tia_design_t *design);

/**
 * @brief  Compute phase margin from loop gain parameters.
 * @param  design  TIA design
 * @return         Phase margin in degrees
 */
double  tia_phase_margin(const tia_design_t *design);

/**
 * @brief  Compute gain margin from loop gain parameters.
 * @param  design  TIA design
 * @return         Gain margin in dB
 */
double  tia_gain_margin(const tia_design_t *design);

/**
 * @brief  Compute the open-loop transfer function value at a specific frequency.
 * @param  design   TIA design
 * @param  freq_hz  Frequency (Hz)
 * @param  gain_db  [out] Open-loop gain magnitude at freq (dB)
 * @param  phase    [out] Open-loop phase at freq (degrees)
 */
void  tia_open_loop_at_freq(const tia_design_t *design, double freq_hz,
                             double *gain_db, double *phase);

/**
 * @brief  Compute the feedback factor beta(j*omega).
 * @param  design   TIA design
 * @param  freq_hz  Frequency (Hz)
 * @return          Complex beta value (magnitude = feedback attenuation)
 *
 * @note   beta(s) = s*C_f*R_f / (1 + s*(C_in+C_f)*R_f)
 *         At DC: beta(0) = 0 (integrator behavior)
 *         At HF:  beta(inf) = C_f / (C_in + C_f)
 */
double  tia_feedback_factor(const tia_design_t *design, double freq_hz);

/**
 * @brief  Design a compensation network for target phase margin.
 * @param  design      Base TIA design (without compensation)
 * @param  target_pm   Target phase margin (degrees, typically 45-70)
 * @param  type        Compensation type
 * @return             Designed compensation network
 */
compensation_network_t  tia_design_compensation(const tia_design_t *design,
                                                 double target_pm,
                                                 compensation_type_t type);

/**
 * @brief  Sweep C_f to find optimal compensation for stability.
 * @param  design     Base TIA design
 * @param  cf_min_pf  Minimum C_f to sweep (pF)
 * @param  cf_max_pf  Maximum C_f to sweep (pF)
 * @param  steps      Number of sweep steps
 * @return            Phase margin vs C_f sweep data
 */
phase_margin_sweep_t  tia_phase_margin_sweep(const tia_design_t *design,
                                              double cf_min_pf, double cf_max_pf,
                                              size_t steps);

/**
 * @brief  Generate pole-zero map for TIA transfer function.
 * @param  design  TIA design
 * @return         Pole-zero map
 */
pole_zero_map_t  tia_pole_zero_map(const tia_design_t *design);

/**
 * @brief  Perform root locus analysis with varying A_OL.
 * @param  design     TIA design
 * @param  gain_min   Minimum dc gain for locus (linear)
 * @param  gain_max   Maximum dc gain for locus (linear)
 * @param  steps      Number of gain steps
 * @return            Root locus data
 */
root_locus_data_t  tia_root_locus(const tia_design_t *design,
                                   double gain_min, double gain_max,
                                   size_t steps);

/**
 * @brief  Generate Nyquist plot data for stability verification.
 * @param  design      TIA design
 * @param  freq_start  Start frequency (Hz)
 * @param  freq_stop   Stop frequency (Hz)
 * @param  points      Number of frequency points
 * @return             Nyquist analysis data
 */
nyquist_data_t  tia_nyquist_analysis(const tia_design_t *design,
                                      double freq_start, double freq_stop,
                                      size_t points);

/**
 * @brief  Check Routh-Hurwitz stability criterion for the TIA.
 * @param  design  TIA design
 * @return         1 if all Routh array elements are positive, 0 otherwise
 *
 * @note   For a second-order system denominator s^2 + 2*zeta*omega_n*s + omega_n^2:
 *         Routh array: [1, omega_n^2; 2*zeta*omega_n, 0; omega_n^2, 0]
 *         Stable if all first-column elements positive: zeta > 0, omega_n^2 > 0
 */
int  tia_routh_hurwitz_stable(const tia_design_t *design);

/**
 * @brief  Compute closed-loop damping factor from phase margin.
 * @param  phase_margin_deg  Phase margin (degrees)
 * @return                   Damping factor zeta
 *
 * @note   Approximate relation: zeta ? PM/100 for PM < 70 degrees
 *         More accurate: zeta = -ln(OS/100) / sqrt(pi^2 + ln(OS/100)^2)
 */
double  tia_damping_from_pm(double phase_margin_deg);

/**
 * @brief  Estimate closed-loop peaking from phase margin.
 * @param  phase_margin_deg  Phase margin (degrees)
 * @return                   Peaking in dB
 *
 * @note   PM=45? -> ~2.3 dB peaking
 *         PM=60? -> ~0.3 dB peaking
 *         PM=75? -> ~0.01 dB peaking (essentially flat)
 */
double  tia_peaking_from_pm(double phase_margin_deg);

/* ??? Memory Management ???????????????????????????????????????????????????? */

void  phase_margin_sweep_free(phase_margin_sweep_t *sweep);
void  pole_zero_map_free(pole_zero_map_t *pzm);
void  root_locus_data_free(root_locus_data_t *rl);
void  nyquist_data_free(nyquist_data_t *ny);

#endif /* TIA_STABILITY_H */
