/**
 * @file    tia_noise.h
 * @brief   TIA Noise Analysis ? L1 Definitions + L4 Fundamental Laws
 *
 * @details Comprehensive noise model for transimpedance amplifier
 *          design. Covers all major noise sources: Johnson (thermal)
 *          noise, shot noise, op-amp voltage and current noise,
 *          1/f flicker noise, and kT/C noise.
 *
 * Knowledge Mapping:
 *   L1 - Definitions:
 *     - Noise spectral density S(f) for each source
 *     - Input-referred noise current i_n_in
 *     - Total integrated output noise v_n_rms
 *     - Noise figure NF and noise factor F
 *     - Signal-to-noise ratio SNR
 *     - Noise Equivalent Power NEP_system
 *   L4 - Fundamental Laws:
 *     - Johnson-Nyquist: v_n^2 = 4 k_B T R Delta_f (Nyquist 1928)
 *     - Shot noise: i_n^2 = 2 q I_DC Delta_f (Schottky 1918)
 *     - kT/C noise: v_n^2 = k_B T / C (sampled systems)
 *     - Friis noise formula for cascaded stages
 *     - Input-referred noise summation: i_total = sqrt(sum i_n_i^2)
 *   L5 - Algorithms/Methods:
 *     - Noise integration over frequency (analytical + numerical)
 *     - Noise corner frequency identification
 *     - Dominant noise source identification
 *     - Optimal R_f selection for minimum total noise
 *
 * References:
 *   - Motchenbacher & Connelly, "Low-Noise Electronic System Design" (1993)
 *   - Graeme, "Photodiode Amplifiers" (1996), Ch.4-5
 *   - Hobbs (2011), Ch.18
 *   - Nyquist, "Thermal Agitation of Electric Charge in Conductors" (1928)
 *   - Schottky, "Uber spontane Stromschwankungen" (1918)
 */

#ifndef TIA_NOISE_H
#define TIA_NOISE_H

#include "tia_core.h"

/* ??? L1: Noise Source Type Enumeration ???????????????????????????????????? */

/**
 * @brief Enumerated noise sources for identification and budgeting.
 */
typedef enum {
    NOISE_SOURCE_JOHNSON_RF     = 0,
    NOISE_SOURCE_JOHNSON_RSH    = 1,
    NOISE_SOURCE_SHOT_DARK      = 2,
    NOISE_SOURCE_SHOT_SIGNAL    = 3,
    NOISE_SOURCE_OPAMP_EN       = 4,
    NOISE_SOURCE_OPAMP_IN       = 5,
    NOISE_SOURCE_OPAMP_EN_CIN   = 6,
    NOISE_SOURCE_FLICKER        = 7,
    NOISE_SOURCE_KTC            = 8,
    NOISE_SOURCE_COUNT          = 9
} noise_source_type_t;

/**
 * @brief Single noise contribution with spectral density.
 */
typedef struct {
    noise_source_type_t source_type;
    char   name[32];
    double spectral_density_at_1khz;
    double spectral_density_at_freq;
    double test_freq_hz;
    double integrated_noise_uv;
    double percent_of_total;
    int    is_white;
    double corner_freq_hz;
} noise_contribution_t;

/* ??? L1: Complete TIA Noise Model ????????????????????????????????????????? */

/**
 * @brief Complete TIA noise analysis results.
 *
 *        Total output noise:  V_n_out = sqrt(integral(|Z_T(f)|^2 * S_out(f) df))
 *        Total input noise:   I_n_in  = V_n_out / Z_T(0) = V_n_out / R_f
 *        System NEP:          NEP_sys = I_n_in_total / R
 *
 *        Where S_out(f) is the output-referred noise PSD summing all sources.
 */
typedef struct {
    /* ?? Individual noise contributions ?? */
    noise_contribution_t contributions[NOISE_SOURCE_COUNT];

    /* ?? Spectral densities at reference frequency ?? */
    double johnson_rf_density;         /**< sqrt(4kT/R_f) in V/sqrt(Hz) referred to output */
    double johnson_rsh_density;        /**< R_f/R_sh noise gain contribution */
    double shot_dark_density;          /**< sqrt(2q*I_dark) in A/sqrt(Hz) */
    double shot_signal_density;        /**< sqrt(2q*I_signal) in A/sqrt(Hz) */
    double opamp_en_density;           /**< e_n * (1 + C_in/C_f) noise gain */
    double opamp_in_density;           /**< i_n * R_f contribution */
    double opamp_en_cin_density;       /**< e_n * 2*pi*f*C_in enhancement */
    double flicker_noise_density;      /**< 1/f noise contribution */
    double ktc_noise_density;          /**< sqrt(kT/C) contribution */

    /* ?? Integrated noise ?? */
    double total_output_noise_uv;       /**< Total integrated output noise (muV_rms) */
    double total_input_noise_pa;        /**< Total input-referred noise (pA_rms) */
    double total_output_noise_mvpp;     /**< Peak-to-peak output noise (mV_pp, 6-sigma) */

    /* ?? SNR and sensitivity ?? */
    double nepo_system_w_per_sqrt_hz;   /**< System NEP at reference wavelength */
    double system_noise_figure_db;       /**< Noise figure relative to Johnson-only limit */
    double dominant_noise_source;        /**< Index of largest noise contribution */

    /* ?? Noise bandwidth ?? */
    double noise_bandwidth_hz;           /**< Equivalent noise bandwidth */
    double noise_corner_hz;              /**< 1/f noise corner of total system */
} tia_noise_model_t;

/* ??? L1: Noise Spectral Density Functions ????????????????????????????????? */

/**
 * @brief Noise PSD as a function of frequency.
 */
typedef struct {
    size_t num_points;
    double *freq_hz;
    double *output_psd_v2_per_hz;       /**< Total output noise PSD (V^2/Hz) */
    double *input_psd_a2_per_hz;        /**< Total input-referred PSD (A^2/Hz) */
    double *noise_gain;                  /**< Noise gain magnitude vs frequency */
    double *contributions[NOISE_SOURCE_COUNT]; /**< Individual PSDs */
} tia_noise_spectrum_t;

/* ??? L1: Noise Optimization Parameters ???????????????????????????????????? */

/**
 * @brief Parameters for TIA noise optimization.
 */
typedef struct {
    double rf_min_ohm;                   /**< Lower bound for R_f sweep */
    double rf_max_ohm;                   /**< Upper bound for R_f sweep */
    double rf_step_factor;               /**< Geometric step factor (>1.0) */
    double target_bandwidth_hz;          /**< Bandwidth constraint */
    double max_noise_pa;                 /**< Maximum allowed input noise (pA_rms) */
} tia_noise_optimization_t;

/**
 * @brief Result of noise optimization sweep.
 */
typedef struct {
    size_t num_points;
    double *rf_values;                   /**< R_f sweep values */
    double *total_noise_pa;              /**< Total noise vs R_f */
    double *bandwidth_mhz;               /**< Bandwidth vs R_f */
    double *johnson_noise_pa;            /**< Johnson contribution */
    double *shot_noise_pa;               /**< Shot contribution */
    double *opamp_noise_pa;              /**< Op-amp contribution */
    double optimal_rf;                   /**< R_f for minimum total noise */
    double optimal_noise_pa;             /**< Minimum achievable noise */
    double optimal_bw_mhz;               /**< Bandwidth at optimal point */
} tia_noise_sweep_t;

/* ??? Function Declarations ? Noise Analysis ??????????????????????????????? */

/**
 * @brief  Compute Johnson (thermal) noise voltage spectral density.
 * @param  resistance_ohm  Resistance value (ohm)
 * @param  temperature_k   Temperature in Kelvin (typically 298.15)
 * @return                 Voltage noise density in V/sqrt(Hz)
 *
 * @note   v_n = sqrt(4 * k_B * T * R)
 *         The noise is white (constant PSD) up to THz frequencies.
 *
 * Reference: Nyquist (1928), Johnson (1928)
 */
double  noise_johnson_voltage(double resistance_ohm, double temperature_k);

/**
 * @brief  Compute Johnson noise current spectral density.
 * @param  resistance_ohm  Resistance value
 * @param  temperature_k   Temperature in Kelvin
 * @return                 Current noise density in A/sqrt(Hz)
 *
 * @note   i_n = sqrt(4 * k_B * T / R)
 *         = v_n / R by Norton equivalence
 */
double  noise_johnson_current(double resistance_ohm, double temperature_k);

/**
 * @brief  Compute shot noise current spectral density.
 * @param  dc_current_a    DC current in amperes
 * @return                 Shot noise density in A/sqrt(Hz)
 *
 * @note   i_n = sqrt(2 * q * I_DC)
 *         Shot noise is white up to f_T of the junction.
 *
 * Reference: Schottky (1918)
 */
double  noise_shot(double dc_current_a);

/**
 * @brief  Compute kT/C noise (sampled/ switched-capacitor noise).
 * @param  capacitance_f   Capacitance in farads
 * @param  temperature_k   Temperature in Kelvin
 * @return                 RMS noise voltage in V
 *
 * @note   v_n_rms = sqrt(k_B * T / C)
 *         This is the total integrated noise, not spectral density!
 */
double  noise_ktc_rms(double capacitance_f, double temperature_k);

/**
 * @brief  Compute complete TIA noise model for a given design.
 * @param  design         TIA design to analyze
 * @param  i_signal_ua    DC signal photocurrent (muA), used for shot noise
 * @param  freq_low       Lower integration limit (Hz)
 * @param  freq_high      Upper integration limit (Hz)
 * @return                Complete noise model with all contributions
 *
 * @note   Integrates noise PSD numerically over [freq_low, freq_high].
 *         Uses analytical expressions for white noise sources.
 *
 * Complexity: O(n_points) for numerical integration
 */
tia_noise_model_t  tia_noise_analyze(const tia_design_t *design,
                                      double i_signal_ua,
                                      double freq_low, double freq_high);

/**
 * @brief  Compute noise spectral density over frequency.
 * @param  design      TIA design
 * @param  i_signal_ua Signal photocurrent (muA)
 * @param  freq_start  Start frequency (Hz)
 * @param  freq_stop   Stop frequency (Hz)
 * @param  points      Number of frequency points
 * @return             Noise spectral density data
 */
tia_noise_spectrum_t  tia_noise_spectrum(const tia_design_t *design,
                                          double i_signal_ua,
                                          double freq_start, double freq_stop,
                                          size_t points);

/**
 * @brief  Compute noise gain of the TIA at a given frequency.
 * @param  design  TIA design
 * @param  freq_hz Frequency (Hz)
 * @return         Noise gain magnitude |1/beta(f)|
 *
 * @note   Noise gain = 1 + Z_in/(R_f||C_f) where Z_in = 1/(j*omega*C_in)
 *         At high freq: NG ~ 1 + C_in/C_f, which amplifies e_n
 */
double  tia_noise_gain(const tia_design_t *design, double freq_hz);

/**
 * @brief  Compute input-referred noise current from noise model.
 * @param  model   Noise model
 * @param  bw_hz   Bandwidth of interest (Hz)
 * @return         Input-referred noise current (A_rms)
 */
double  tia_input_referred_noise(const tia_noise_model_t *model, double bw_hz);

/**
 * @brief  Perform noise optimization sweep over R_f values.
 * @param  design_base  Base TIA design (R_f will be varied)
 * @param  opt          Optimization parameters
 * @return              Sweep results with optimal R_f identified
 */
tia_noise_sweep_t  tia_noise_optimize_rf(const tia_design_t *design_base,
                                          const tia_noise_optimization_t *opt);

/**
 * @brief  Identify the dominant noise source in the TIA.
 * @param  model   Noise model
 * @return         Index of dominant source in contributions[] array
 */
int  tia_noise_dominant_source(const tia_noise_model_t *model);

/**
 * @brief  Compute the output SNR given signal and noise.
 * @param  model       Noise model
 * @param  i_signal_ua Signal photocurrent (muA)
 * @param  bw_hz       Bandwidth (Hz)
 * @return             SNR in dB
 */
double  tia_output_snr(const tia_noise_model_t *model,
                        double i_signal_ua, double bw_hz);

/**
 * @brief  Estimate noise figure of TIA compared to Johnson-only limit.
 * @param  model   Noise model
 * @return         Noise figure in dB
 *
 * @note   NF = 10*log10(total_output_noise^2 / johnson_rf_noise^2)
 *         Compares actual noise to ideal R_f Johnson noise floor.
 */
double  tia_noise_figure(const tia_noise_model_t *model);

/**
 * @brief  Compute optimal R_f for minimum noise at given bandwidth.
 * @param  pd            Photodiode model
 * @param  opa           Op-amp parameters
 * @param  bw_target_hz  Target bandwidth (Hz)
 * @return               Optimal R_f in ohms
 *
 * @note   Optimal R_f balances Johnson noise (decreases with R_f)
 *         against op-amp e_n*C_in noise (increases with BW/R_f tradeoff).
 *
 *         For bandwidth-constrained design:
 *         R_f_opt = 1 / (2*pi * C_in * f_3dB_target)
 */
double  tia_optimal_rf_for_noise(const photodiode_model_t *pd,
                                  const opamp_params_t *opa,
                                  double bw_target_hz);

/**
 * @brief  Free noise spectrum data.
 */
void  tia_noise_spectrum_free(tia_noise_spectrum_t *ns);

/**
 * @brief  Free noise sweep data.
 */
void  tia_noise_sweep_free(tia_noise_sweep_t *sweep);

#endif /* TIA_NOISE_H */
