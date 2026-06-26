/**
 * @file ina_core.c
 * @brief Core Instrumentation Amplifier Implementation
 *
 * Implements fundamental calculations for L1-L5:
 *   L1: SNR, ENOB, dynamic range, offset, noise definitions
 *   L2: Signal decomposition, CMRR computation
 *   L3: Noise integration, transfer function analysis
 *   L4: Superposition, CMRR from gains, resistor mismatch analysis
 *   L5: Error budget computation (algorithm), gain resistor selection
 *
 * Reference:
 *   Sedra & Smith, "Microelectronic Circuits" (2020)
 *   Motchenbacher & Connelly, "Low-Noise Electronic System Design" (1993)
 *   IEEE Std 1241-2010: ADC Terminology and Test Methods
 */
#include "ina_core.h"
#include <math.h>
#include <string.h>

/*===========================================================================
 * L4: Signal Decomposition (Superposition Principle)
 *===========================================================================*/

void ina_decompose_signal(double v_plus, double v_minus,
                          double *v_dm, double *v_cm)
{
    /**
     * Applies superposition principle (L4 Fundamental Law).
     *
     * Any two voltages can be expressed as:
     *   V_dm = V1 - V2          (differential-mode)
     *   V_cm = (V1 + V2) / 2    (common-mode)
     *
     * The inverse is:
     *   V1 = V_cm + V_dm/2
     *   V2 = V_cm - V_dm/2
     *
     * This decomposition is fundamental to all differential amplifier
     * analysis because real amplifiers respond differently to V_dm
     * and V_cm (Ad >> Acm).
     *
     * Reference: Sedra & Smith ?2.4.1
     */
    if (v_dm) *v_dm = v_plus - v_minus;
    if (v_cm) *v_cm = (v_plus + v_minus) / 2.0;
}

void ina_recompose_signal(double v_dm, double v_cm,
                          double *v_plus, double *v_minus)
{
    /**
     * Inverse decomposition:
     *   V+ = V_cm + V_dm/2
     *   V- = V_cm - V_dm/2
     */
    if (v_plus)  *v_plus  = v_cm + v_dm / 2.0;
    if (v_minus) *v_minus = v_cm - v_dm / 2.0;
}

/*===========================================================================
 * L4: CMRR Computation
 *===========================================================================*/

double ina_cmrr_from_gains(double ad, double acm)
{
    /**
     * CMRR (Common-Mode Rejection Ratio) is the ratio of
     * differential-mode gain to common-mode gain.
     *
     * CMRR_dB = 20 * log10(|Ad/Acm|)
     *
     * For an ideal IA, Acm ? 0 so CMRR ? ?.
     * Real IAs achieve 80-130 dB CMRR depending on gain.
     *
     * Protection against division by zero (ideal amplifier case):
     *   Returns INFINITY for Acm = 0.
     *
     * Reference: Sedra & Smith, Eq. 2.15
     */
    if (fabs(acm) < 1e-30) {
        return INFINITY;
    }
    return 20.0 * log10(fabs(ad / acm));
}

double ina_cmrr_from_resistor_mismatch(double r2_over_r1, double delta_r)
{
    /**
     * CMRR limited by resistor matching in a difference amplifier.
     *
     * For a difference amplifier with resistors R1, R2:
     *   Ideal gain: Vo = (R2/R1) * (V2 - V1)    when R1=R3, R2=R4
     *
     * With mismatch delta (?R/R):
     *   CMRR ? (1 + R2/R1) / (4 * delta)
     *
     * Derivation (L3 Mathematical Structure):
     *   Let R3 = R1(1+?1), R4 = R2(1+?2)
     *   The common-mode gain becomes: Acm ? 4*? / (1+R2/R1) * Ad
     *   Therefore CMRR = Ad/Acm = (1+R2/R1)/(4*|?|)
     *
     * For a unity-gain diff amp (R2/R1 = 1) with 0.1% resistors:
     *   CMRR = 2 / (4 * 0.001) = 500 = 54 dB
     *
     * This shows why IA CMRR degrades severely with poor matching.
     *
     * Reference: Kitchin & Counts, "A Designer's Guide to IAs" (ADI, 2006)
     */
    if (fabs(delta_r) < 1e-30) {
        return INFINITY;
    }
    return (1.0 + r2_over_r1) / (4.0 * fabs(delta_r));
}

double ina_cmrr_total(double cmrr_resistor, double cmrr_opamp,
                      double cmrr_layout)
{
    /**
     * Total CMRR from uncorrelated contributors.
     *
     * Individual CMRR contributions add in quadrature because
     * the error voltages they produce are uncorrelated:
     *
     *   1/CMRR_total^2 = 1/CMRR_res^2 + 1/CMRR_opamp^2 + 1/CMRR_layout^2
     *
     * This is mathematically analogous to parallel resistors or
     * RSS error summation.
     *
     * All inputs are linear ratios (not dB). Convert from dB first:
     *   CMRR_linear = 10^(CMRR_dB / 20)
     */
    double res_lin = cmrr_resistor;
    double opa_lin = cmrr_opamp;
    double lay_lin = cmrr_layout;

    double inv_sq = 0.0;
    if (res_lin > 0.0) inv_sq += 1.0 / (res_lin * res_lin);
    if (opa_lin > 0.0) inv_sq += 1.0 / (opa_lin * opa_lin);
    if (lay_lin > 0.0) inv_sq += 1.0 / (lay_lin * lay_lin);

    if (inv_sq <= 0.0) return INFINITY;
    return 1.0 / sqrt(inv_sq);
}

double ina_cmrr_vs_frequency(double cmrr_dc, double frequency,
                             double f_cmrr_pole)
{
    /**
     * CMRR degrades with frequency due to finite op-amp bandwidth.
     *
     * CMRR(f) = CMRR_DC / sqrt(1 + (f / f_cmrr_pole)^2)
     *
     * The CMRR pole (f_cmrr_pole) is typically at:
     *   f_cmrr_pole ? GBW / G   for 3-op-amp IA
     *
     * This is because the common-mode gain has a different
     * frequency response than the differential gain.
     *
     * In dB: CMRR(f)_dB = CMRR_DC_dB - 20*log10(sqrt(1 + (f/fp)^2))
     */
    if (f_cmrr_pole <= 0.0) return cmrr_dc;
    double ratio = frequency / f_cmrr_pole;
    return cmrr_dc / sqrt(1.0 + ratio * ratio);
}

/*===========================================================================
 * L1/L2: Offset Voltage and Drift
 *===========================================================================*/

double ina_offset_at_temperature(double vos_25c, double tc_vos_nv_per_c,
                                 double temperature_c,
                                 double tc2_vos_nv_per_c2)
{
    /**
     * Offset voltage varies with temperature.
     *
     * Linear model (most common):
     *   Vos(T) = Vos_25C + TC_Vos * (T - 25)
     *
     * Quadratic model (for precision applications):
     *   Vos(T) = Vos_25C + TC1*(T-25) + TC2*(T-25)^2
     *
     * where:
     *   Vos_25C in ?V, TC in nV/?C, TC2 in nV/?C^2
     *   Result in ?V
     *
     * Example: AD620 has Vos_max = 50?V, TC_vos = 0.6?V/?C max
     *   At 85?C: Vos = 50 + 0.6*(85-25) = 50 + 36 = 86 ?V
     *
     * Reference: Sedra & Smith ?2.6
     */
    double dt = temperature_c - 25.0;
    double drift = tc_vos_nv_per_c * 1e-3 * dt;  /* nV/?C ? ?V */
    double drift2 = tc2_vos_nv_per_c2 * 1e-3 * dt * dt;
    return vos_25c + drift + drift2;
}

double ina_offset_drift_over_range(double drift_nv_per_c,
                                   double delta_t_c)
{
    /**
     * Convert drift rate to total offset change over a given ?T.
     *
     * ?Vos = drift_rate * ?T
     *
     * Drift is specified in nV/?C, output in ?V.
     * This provides an intuitive error magnitude for the
     * expected operating temperature range.
     */
    return drift_nv_per_c * 1e-3 * delta_t_c;
}

/*===========================================================================
 * L3: Noise Calculations
 *===========================================================================*/

double ina_rms_noise(double en_noise_density, double f_low, double f_high,
                     double f_corner)
{
    /**
     * Compute total RMS noise by integrating the noise spectral density.
     *
     * The total noise voltage spectral density at the IA input is:
     *   en_total(f)^2 = en_white^2 * (1 + fc/f)
     *
     * where fc is the 1/f corner frequency.
     *
     * Total RMS noise over bandwidth [f_low, f_high]:
     *   Vn_rms^2 = ?[f_low, f_high] en_total(f)^2 df
     *
     * For white noise only (f_corner = 0):
     *   Vn_rms^2 = en_white^2 * (f_high - f_low)
     *   Vn_rms = en_white * sqrt(f_high - f_low)
     *
     * With 1/f noise:
     *   Vn_rms^2 = en_white^2 * [(f_high - f_low) + fc * ln(f_high/f_low)]
     *
     * This integral is the mathematical foundation of noise budgeting.
     * The en_noise_density parameter should be in nV/?Hz,
     * frequencies in Hz, output in V RMS.
     *
     * Reference: Motchenbacher & Connelly (1993), Ch. 4
     */
    if (f_high <= f_low) return 0.0;

    /* en is in nV/?Hz, convert to V/?Hz for calculation */
    double en_v = en_noise_density * 1e-9;

    double white_term = f_high - f_low;
    double flicker_term = 0.0;

    if (f_corner > 0.0 && f_low > 0.0) {
        flicker_term = f_corner * log(f_high / f_low);
    }

    double variance = en_v * en_v * (white_term + flicker_term);
    if (variance < 0.0) return 0.0;
    return sqrt(variance);
}

double ina_rms_to_peak_to_peak(double vn_rms, double crest_factor)
{
    /**
     * Convert RMS noise to peak-to-peak using crest factor.
     *
     * Crest Factor (CF) relates peak-to-peak to RMS for Gaussian noise:
     *   Vpp = CF * Vrms
     *
     * Common crest factors:
     *   CF = 3.3 ? 90% confidence (all samples within ?1.65?)
     *   CF = 4.0 ? 95.4% confidence (?2?)
     *   CF = 6.0 ? 99.73% confidence (?3?)
     *   CF = 6.6 ? 99.9% confidence (?3.3?)
     *   CF = 8.0 ? 99.99% confidence (?4?)
     *
     * For oscilloscope display, CF = 6 to 8 is commonly used.
     * For specifying guaranteed noise-free resolution, CF = 6.6.
     */
    return vn_rms * crest_factor;
}

double ina_noise_figure(double en_nv_per_sqrt_hz, double source_resistance,
                        double temperature_k, double bandwidth_hz)
{
    /**
     * Compute Noise Figure (NF) of the IA.
     *
     * NF compares the total output noise to the noise that would
     * be present if only the source resistance contributed noise.
     *
     * NF_dB = 10*log10( Total_Noise_Power / Source_Noise_Power )
     *
     * Source thermal noise (Johnson-Nyquist):
     *   Vn_source^2 = 4 * k_B * T * Rs * BW
     *
     * Total noise at IA output:
     *   Vn_total^2 = Vn_source^2 + Vn_ia_input^2
     *   where Vn_ia_input = en * sqrt(BW)
     *
     * NF = 10*log10(1 + (en^2 * BW) / (4kTRs * BW))
     *    = 10*log10(1 + en^2 / (4kTRs))
     *
     * k_B = Boltzmann constant = 1.380649e-23 J/K
     *
     * Reference: Friis, "Noise Figures of Radio Receivers" (1944)
     */
    (void)bandwidth_hz; const double k_boltzmann = 1.380649e-23;
    double en_v = en_nv_per_sqrt_hz * 1e-9;

    /* Johnson noise voltage spectral density */
    double source_noise_v2_per_hz = 4.0 * k_boltzmann * temperature_k
                                    * source_resistance;

    /* IA input noise voltage spectral density squared */
    double ia_noise_v2_per_hz = en_v * en_v;

    if (source_noise_v2_per_hz <= 0.0) return INFINITY;

    double ratio = ia_noise_v2_per_hz / source_noise_v2_per_hz;
    return 10.0 * log10(1.0 + ratio);
}

/*===========================================================================
 * L1: SNR, ENOB, Dynamic Range
 *===========================================================================*/

double ina_snr_db(double vsignal_rms, double vnoise_rms)
{
    /**
     * Signal-to-Noise Ratio (SNR).
     *
     * SNR_dB = 20 * log10(V_signal_rms / V_noise_rms)
     *
     * For sinusoidal signals:
     *   V_rms = V_peak / sqrt(2)
     *   SNR_dB = 20*log10(Vp/(sqrt(2)*Vn_rms))
     *
     * Full-scale sine wave in ADC:
     *   Vp = Vref/2 (for single-ended)
     *   SNR_max_dB = 6.02*N + 1.76 (ideal N-bit ADC)
     *
     * Reference: IEEE Std 1241-2010 ?4.5
     */
    if (vnoise_rms <= 0.0) return INFINITY;
    return 20.0 * log10(vsignal_rms / vnoise_rms);
}

double ina_enob_from_snr(double snr_db)
{
    /**
     * Effective Number of Bits (ENOB).
     *
     * ENOB = (SNR_dB - 1.76) / 6.02
     *
     * Derivation:
     *   Ideal N-bit ADC SNR: SNR = 6.02*N + 1.76 dB
     *   Solving for N: N = (SNR - 1.76) / 6.02
     *
     * The 1.76 dB comes from the quantization noise of an ideal ADC:
     *   SNR_q = 20*log10(2^N * sqrt(1.5))
     *         = 6.02*N + 1.76
     *
     * ENOB represents the effective resolution considering all
     * noise sources (thermal, quantization, jitter, INL, DNL).
     * An ideal 16-bit ADC might achieve only 14.5 ENOB in practice.
     *
     * Reference: IEEE Std 1241-2010, Annex A
     */
    return (snr_db - 1.76) / 6.02;
}

double ina_dynamic_range_db(double v_full_scale, double v_noise_floor)
{
    /**
     * Dynamic Range (DR).
     *
     * DR_dB = 20 * log10(V_full_scale / V_noise_floor)
     *
     * Dynamic range differs from SNR:
     *   - DR: maximum signal / minimum detectable signal
     *   - SNR: signal / noise at a given signal level
     *
     * For an IA, V_noise_floor is typically the RTI noise
     * integrated over the measurement bandwidth.
     *
     * In audio systems, DR is often > SNR because the maximum
     * signal exceeds the nominal operating level.
     */
    if (v_noise_floor <= 0.0) return INFINITY;
    return 20.0 * log10(v_full_scale / v_noise_floor);
}

double ina_minimum_detectable_signal(double noise_floor_v, double snr_min)
{
    /**
     * Minimum Detectable Signal (MDS).
     *
     * MDS = V_noise_floor * 10^(SNR_min/20)
     *
     * SNR_min depends on the detection criterion:
     *   - SNR_min = 0 dB for 50% detection probability
     *   - SNR_min = 12-15 dB for reliable detection (>90%)
     *
     * In radar: MDS = kTB * NF * SNR_min
     * In sensors: MDS = noise_density * sqrt(BW) * SNR_min_linear
     *
     * This is the fundamental sensitivity limit of the measurement system.
     */
    double snr_linear = pow(10.0, snr_min / 20.0);
    return noise_floor_v * snr_linear;
}

/*===========================================================================
 * L5: Error Budget Analysis
 *===========================================================================*/

InaErrorBudget ina_compute_error_budget(const InaParameters *params,
                                         double vcm,
                                         double vsupply_deviation,
                                         double temperature,
                                         double rs_source)
{
    /**
     * Complete DC error budget computation (L5 Algorithm).
     *
     * This is the systematic methodology for determining total
     * measurement uncertainty in an IA-based system.
     *
     * Error sources (all RTI, in ?V):
     *
     * 1. Offset voltage: Vos_uv(T)
     *    = vos_uv + vos_drift_nv_per_C * (T - 25) * 1e-3
     *
     * 2. Gain error (contribution RTI):
     *    = V_full_scale * gain_error_percent / 100
     *
     * 3. CMRR error:
     *    = Vcm / 10^(CMRR_dB/20) * 1e6  (in ?V)
     *
     * 4. PSRR error:
     *    = ?Vsupply / 10^(PSRR_dB/20) * 1e6
     *
     * 5. Noise error (RMS, converted from noise density):
     *    = en_rti * sqrt(BW)  (already RTI at G=1)
     *
     * 6. Nonlinearity error:
     *    = V_full_scale * nonlinearity_ppm / 1e6
     *
     * 7. Ib * Rs offset:
     *    = Ios * Rs (from input offset current and source imbalance)
     *
     * Total RSS:
     *    e_total_rss = sqrt(? ei^2) (uncorrelated errors)
     *
     * Total worst-case:
     *    e_total_wc = ? |ei|
     *
     * Effective resolution:
     *    N_eff = log2(V_fs / (Gain * e_total_rss))
     *
     * Reference: Kester, "Data Conversion Handbook" (ADI, 2005), Ch. 2
     */
    InaErrorBudget budget;
    memset(&budget, 0, sizeof(budget));

    if (!params) return budget;

    /* 1. Offset voltage at temperature (?V) */
    double dt = temperature - 25.0;
    budget.vos_error_uv = params->vos_uv
                         + params->vos_drift_nv_per_C * 1e-3 * dt;
    if (budget.vos_error_uv < 0.0) budget.vos_error_uv = -budget.vos_error_uv;

    /* 2. Gain error RTI */
    /* Vout_fs = params->gain * Vin_fs; assume Vin_fs = Vout_fs_max / gain */
    /* Use a nominal full-scale input of output_swing_max / gain */
    double vin_fs = 0.0;
    if (params->gain > 0.0) {
        vin_fs = (params->output_swing_max - params->output_swing_min)
                 / params->gain;
    }
    budget.gain_error_uv = vin_fs * fabs(params->gain_error_percent) / 100.0 * 1e6;

    /* 3. CMRR error: error = Vcm / CMRR_linear */
    double cmrr_linear = pow(10.0, params->cmrr_db / 20.0);
    if (cmrr_linear > 0.0) {
        budget.cmrr_error_uv = fabs(vcm) / cmrr_linear * 1e6;
    }

    /* 4. PSRR error */
    double psrr_linear = pow(10.0, params->psrr_plus_db / 20.0);
    if (psrr_linear > 0.0) {
        budget.psrr_error_uv = fabs(vsupply_deviation)
                               / psrr_linear * 1e6;
    }

    /* 5. Noise error (RTI RMS) */
    budget.noise_error_uv_rms = params->en_rti_at_gain1_nv_rms * 1e-3;

    /* 6. Nonlinearity */
    budget.nonlinearity_error_uv = vin_fs
                                   * params->gain_nonlinearity_ppm / 1e6 * 1e6;

    /* 7. Ib * Rs offset (assume Ios = 10% of Ib typical) */
    /* Ib is not directly in params, use a rough estimate */
    /* A typical precision IA has Ib ~ 1 nA to 10 nA */
    /* The in_pa_per_sqrt_hz is current noise, not Ib */
    /* Approximate Ib from common-mode input impedance:
       Ib ? (2*Vsupply/2) / Zin_cm  (but this is very rough) */
    /* Use 1 nA typical, with Rs imbalance of 1% */
    double ib_estimate = 1.0e-9;  /* 1 nA typical for precision IA */
    double rs_imbalance = rs_source * 0.01;  /* 1% mismatch */
    budget.ib_offset_error_uv = ib_estimate * rs_imbalance * 1e6;

    /* Compute totals */
    double sum_sq = budget.vos_error_uv * budget.vos_error_uv
                  + budget.gain_error_uv * budget.gain_error_uv
                  + budget.cmrr_error_uv * budget.cmrr_error_uv
                  + budget.psrr_error_uv * budget.psrr_error_uv
                  + budget.noise_error_uv_rms * budget.noise_error_uv_rms
                  + budget.nonlinearity_error_uv * budget.nonlinearity_error_uv
                  + budget.ib_offset_error_uv * budget.ib_offset_error_uv;

    budget.total_error_rss_uv = sqrt(sum_sq);

    budget.total_error_worst_uv = budget.vos_error_uv
                                + budget.gain_error_uv
                                + budget.cmrr_error_uv
                                + budget.psrr_error_uv
                                + budget.noise_error_uv_rms
                                + budget.nonlinearity_error_uv
                                + budget.ib_offset_error_uv;

    /* Effective resolution */
    double error_v = budget.total_error_rss_uv * 1e-6;
    double fs_v = params->output_swing_max - params->output_swing_min;
    if (error_v > 0.0 && fs_v > 0.0) {
        budget.effective_resolution_bits = log2(fs_v / error_v);
    }

    return budget;
}

/*===========================================================================
 * L6: Gain Setting Resistor Calculation
 *===========================================================================*/

double ina_calculate_rg(double desired_gain, double r_internal)
{
    /**
     * Compute Rg for a 3-op-amp IA.
     *
     * The gain equation for a 3-op-amp IA:
     *   G = 1 + 2*R_internal / Rg
     *
     * Therefore:
     *   Rg = 2*R_internal / (G - 1)
     *
     * For G = 1 (no gain), Rg ? ? (open circuit).
     * For G > 1, Rg decreases as G increases.
     *
     * Example (AD620: R_internal = 24.7 k?):
     *   G = 10:   Rg = 2*24.7k / 9 = 5.489 k?
     *   G = 100:  Rg = 2*24.7k / 99 = 499 ?
     *   G = 1000: Rg = 2*24.7k / 999 = 49.5 ?
     */
    if (desired_gain <= 1.0) {
        return INFINITY;  /* Open circuit for unity gain */
    }
    return 2.0 * r_internal / (desired_gain - 1.0);
}

double ina_calculate_gain_from_rg(double rg, double r_internal)
{
    /**
     * Compute actual gain given Rg value.
     *   G = 1 + 2*R_internal / Rg
     */
    if (rg <= 0.0) {
        return INFINITY;  /* Short circuit ? infinite gain (theoretical) */
    }
    return 1.0 + 2.0 * r_internal / rg;
}

double ina_nearest_standard_resistor(double target_r, int series)
{
    /**
     * Find nearest standard E-series resistor value.
     *
     * E96 series (1% tolerance): 96 values per decade
     * E192 series (0.5%): 192 values per decade
     *
     * For E96: multiplier = 10^(1/96) ? 1.0243
     * Each value: R = 10^(i/96) for i = 0..95, rounded to 3 sig figs
     *
     * Standard E96 values (first 24 shown, pattern repeats per decade):
     *   100, 102, 105, 107, 110, 113, 115, 118, 121, 124, 127,
     *   130, 133, 137, 140, 143, 147, 150, 154, 158, 162, 165,
     *   169, 174, 178, 182, 187, 191, 196, 200, 205, 210, 215,
     *   ... 976
     *
     * Algorithm:
     *   1. Determine decade: m = floor(log10(target_R))
     *   2. Normalize: R_norm = target_R / 10^m  (value in [1, 10))
     *   3. Find index: i = round(log10(R_norm) * 96)
     *   4. Standard value = round(10^(i/96)) * 10^m
     */
    if (target_r <= 0.0 || series <= 0) return 0.0;

    int n = (series == 192) ? 192 : 96;
    double decade = floor(log10(target_r));
    double normalized = target_r / pow(10.0, decade);

    /* Clamp to [1, 10) */
    if (normalized < 1.0)  { normalized = 1.0;  decade -= 1.0; }
    if (normalized >= 10.0) { normalized = 10.0; decade += 1.0; normalized = 1.0; }

    /* Find nearest E-series index */
    double idx = log10(normalized) * (double)n;
    int idx_int = (int)round(idx);
    if (idx_int < 0) idx_int = 0;
    if (idx_int >= n) idx_int = n - 1;

    double std_value = round(pow(10.0, (double)idx_int / (double)n) * 100.0)
                       / 100.0;
    return std_value * pow(10.0, decade);
}

double ina_gain_error_from_rg_tolerance(double rg_nominal,
                                         double rg_tolerance_percent,
                                         double r_internal)
{
    /**
     * Gain error caused by Rg tolerance.
     *
     * G_nominal = 1 + 2*R_internal / Rg_nominal
     * G_actual_min = 1 + 2*R_internal / (Rg_nominal * (1 + tol/100))
     * G_actual_max = 1 + 2*R_internal / (Rg_nominal * (1 - tol/100))
     *
     * Gain error (%) = (G_actual - G_nominal) / G_nominal * 100
     *
     * Since Rg appears in the denominator, 1% Rg tolerance does NOT
     * produce 1% gain error. The error depends on the gain setting.
     *
     * For G >> 1:
     *   G ? 2*R_internal / Rg
     *   ?G/G ? -?Rg/Rg  (gain error ? -Rg tolerance)
     *
     * For G ? 2-10: gain error is less than Rg tolerance
     * For G ? 100-1000: gain error ? Rg tolerance
     */
    if (rg_nominal <= 0.0) return 0.0;

    double g_nominal = ina_calculate_gain_from_rg(rg_nominal, r_internal);
    double rg_max = rg_nominal * (1.0 + rg_tolerance_percent / 100.0);
    double rg_min = rg_nominal * (1.0 - rg_tolerance_percent / 100.0);
    if (rg_min <= 0.0) rg_min = rg_nominal * 0.001;

    double g_max = ina_calculate_gain_from_rg(rg_min, r_internal);
    double g_min = ina_calculate_gain_from_rg(rg_max, r_internal);

    double error_max = fabs(g_max - g_nominal) / g_nominal * 100.0;
    double error_min = fabs(g_nominal - g_min) / g_nominal * 100.0;

    return (error_max > error_min) ? error_max : error_min;
}

/*===========================================================================
 * L5: Input Bias Current Return Path
 *===========================================================================*/

double ina_max_source_resistance(double ib_amps, double allowed_error_uv)
{
    /**
     * Maximum source resistance for acceptable Ib-induced error.
     *
     * Rule (L2 Core Concept):
     *   BOTH IA inputs MUST have a DC path to ground for Ib.
     *
     * The voltage developed across source resistance:
     *   V_error = Ib * Rs
     *
     * For allowed error V_err_max:
     *   Rs_max = V_err_max / Ib
     *
     * Example: Ib = 1 nA, allowed error = 10 ?V
     *   Rs_max = 10e-6 / 1e-9 = 10,000 ?
     *
     * This is why thermocouple and high-impedance sensors need
     * careful attention to bias current paths.
     */
    if (ib_amps <= 0.0) return INFINITY;
    return (allowed_error_uv * 1e-6) / ib_amps;
}

double ina_ib_offset(double ib_plus, double ib_minus,
                     double rs_plus, double rs_minus)
{
    /**
     * Offset voltage caused by input bias current.
     *
     * The input bias current flows through the source resistance,
     * creating a voltage drop:
     *   V_ib_plus = Ib+ * Rs+
     *   V_ib_minus = Ib- * Rs-
     *
     * The differential error is:
     *   V_os_ib = Ib+ * Rs+ - Ib- * Rs-
     *
     * This can be expressed as:
     *   V_os_ib = Ib * ?Rs + Ios * Rs_avg
     *
     * where:
     *   Ib = (Ib+ + Ib-)/2  (input bias current)
     *   Ios = |Ib+ - Ib-|    (input offset current)
     *   ?Rs = |Rs+ - Rs-|
     *   Rs_avg = (Rs+ + Rs-)/2
     *
     * For matched source resistances (Rs+ = Rs- = Rs):
     *   V_os_ib = Ios * Rs
     *
     * This shows why matching source impedances is critical
     * for precision IA applications.
     */
    return (ib_plus * rs_plus) - (ib_minus * rs_minus);
}
