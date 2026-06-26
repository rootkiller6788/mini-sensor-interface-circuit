/**
 * @file cap_measurement_circuit.c
 * @brief Capacitance Measurement Circuit Models
 *
 * Implements analytical models for the six main capacitance measurement
 * topologies. Each model computes the key performance metrics:
 * resolution, conversion time, sensitivity to parasitics, and noise.
 *
 * Knowledge Coverage:
 *   L1-L2: charge transfer, sigma-delta CDC, relaxation osc, dual-slope, AC bridge
 *   L3: transfer functions, noise shaping, oversampling theory
 *   L4: kT/C noise in each topology, charge injection analysis
 *   L5: auto-ranging, optimal measurement parameter selection
 *   L6: CDC comparison methodology
 *
 * Ref: O'Dowd et al. (2011) "Capacitive Sensor Interfaces" Ch.3-5
 *      AD7745/AD7746 datasheet (24-bit CDC)
 *      TI FDC1004/FDC2214 datasheets
 */

#include "cap_measurement_circuit.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ==========================================================================
 * L2: CHARGE TRANSFER
 * ========================================================================== */

void cap_charge_transfer_init(cap_charge_transfer_circuit_t *ct,
                              double c_sense, double c_int,
                              double v_dd, double v_ref)
{
    if (!ct) return;
    memset(ct, 0, sizeof(*ct));
    ct->c_sense_f = c_sense;
    ct->c_int_f = c_int;
    ct->v_dd = v_dd;
    ct->v_ref = v_ref;
    ct->v_int_initial = 0.0;
    ct->switch_resistance_ohm = 100.0; /* Typical CMOS switch */
}

/**
 * cap_charge_transfer_count
 *
 * Computes the number of charge transfers needed to charge C_int from
 * V_int_initial to V_ref.
 *
 * Each transfer: C_s is charged to V_dd, then discharged into C_int.
 *
 * Charge per transfer: delta_Q = C_s * V_dd (assuming ideal switches)
 * Voltage increment per transfer: delta_V = delta_Q / C_int
 *
 * For non-ideal switches with finite on-resistance R_on:
 * The transfer is an RC settling: V_Cint(t) = V_final * (1 - e^{-t/tau})
 * where tau = R_on * C_s (for charging) or R_on * C_s || C_int (for sharing).
 *
 * Number of transfers: N_ideal = C_int * (V_ref - V_init) / (C_s * V_dd)
 *
 * With switch losses (~5%): N_actual ≈ N_ideal / 0.95
 *
 * For C_int=10nF, V_ref=1.65V, V_init=0, C_s=10pF, V_dd=3.3V:
 *   N ≈ 10e-9 * 1.65 / (10e-12 * 3.3) ≈ 500 transfers
 *
 * At 500 kHz transfer rate: measurement time ≈ 1 ms.
 */
uint32_t cap_charge_transfer_count(const cap_charge_transfer_circuit_t *ct)
{
    if (!ct) return 0;
    if (ct->c_sense_f <= 0.0 || ct->c_int_f <= 0.0 || ct->v_dd <= 0.0) return 0;

    double delta_q = ct->c_sense_f * ct->v_dd;
    double delta_v = delta_q / ct->c_int_f;

    if (delta_v <= 0.0) return 0;

    double voltage_to_go = ct->v_ref - ct->v_int_initial;
    if (voltage_to_go <= 0.0) return 0;

    uint32_t transfers = (uint32_t)ceil(voltage_to_go / delta_v);

    /* Add 5% margin for switch losses */
    transfers = (uint32_t)((double)transfers / 0.95);

    return transfers;
}

double cap_charge_transfer_to_capacitance(const cap_charge_transfer_circuit_t *ct,
                                          uint32_t transfer_count)
{
    if (!ct || transfer_count == 0 || ct->c_int_f <= 0.0 || ct->v_dd <= 0.0) {
        return 0.0;
    }

    /* C_s = C_int * (V_ref - V_init) / (N * V_dd) */
    double dv = ct->v_ref - ct->v_int_initial;
    return (ct->c_int_f * dv) / ((double)transfer_count * ct->v_dd);
}

/* ==========================================================================
 * L5: SIGMA-DELTA CDC
 * ========================================================================== */

void cap_sigma_delta_cdc_init(cap_sigma_delta_cdc_t *cdc,
                              double c_ref, double v_exc,
                              uint32_t osr, uint32_t order)
{
    if (!cdc) return;
    memset(cdc, 0, sizeof(*cdc));
    cdc->c_ref_f = c_ref;
    cdc->v_exc = v_exc;
    cdc->v_ref = v_exc; /* Ratiometric */
    cdc->osr = osr;
    cdc->modulator_order = (order == 2) ? 2 : 1;
    cdc->bitstream_length = osr;

    /* Estimate ENOB */
    cdc->enob = cap_sigma_delta_enob(osr, order);
}

/**
 * cap_sigma_delta_convert
 *
 * Simulates a sigma-delta capacitance measurement.
 *
 * First-order SDM: The feedback is controlled by comparator output.
 * When output=1: C_ref is subtracted from the integrator.
 * When output=0: nothing subtracted.
 *
 * After N cycles, the bitstream density (fraction of 1s) is:
 *   density = C_sense / C_ref
 *
 * Therefore: C_sense_measured = density * C_ref
 *
 * Input-referred noise is added to the measurement, scaled by the
 * quantization noise:
 *   sigma_C = C_ref / (sqrt(12) * sqrt(OSR)) + sigma_noise_input
 *
 * For OSR=256, first-order: quantization noise ≈ C_ref/(3.46*16) ≈ C_ref/55.4
 * If C_ref=10pF: sigma_quant ≈ 0.18 pF
 * Averaging 100 measurements reduces this by sqrt(100)=10x → 18 fF
 *
 * A second-order modulator improves noise shaping:
 *   sigma_C_2nd ≈ C_ref * pi^2 / (sqrt(5) * OSR^(5/2))
 *
 * @param cdc       CDC model (updated with results)
 * @param c_sense   True sense capacitance [F]
 * @param noise_rms Input-referred noise RMS [F]
 */
void cap_sigma_delta_convert(cap_sigma_delta_cdc_t *cdc,
                             double c_sense, double noise_rms)
{
    if (!cdc || cdc->c_ref_f <= 0.0) return;

    /* Ideal bitstream density = C_sense / C_ref */
    double ideal_density = c_sense / cdc->c_ref_f;
    if (ideal_density < 0.0) ideal_density = 0.0;
    if (ideal_density > 1.0) ideal_density = 1.0;

    /* Add quantization noise:
     * For first-order SDM, in-band quantization noise:
     * sigma_C = C_ref * pi / (sqrt(3) * OSR^(3/2)) */
    double sigma_quant;
    if (cdc->modulator_order == 2) {
        sigma_quant = cdc->c_ref_f * M_PI * M_PI / (sqrt(5.0) * pow((double)cdc->osr, 2.5));
    } else {
        sigma_quant = cdc->c_ref_f * M_PI / (sqrt(3.0) * pow((double)cdc->osr, 1.5));
    }

    /* Combined noise */
    double sigma_total = sqrt(sigma_quant * sigma_quant + noise_rms * noise_rms);

    /* Measured density with noise */
    /* Using simplified model: add Gaussian noise to density */
    double sigma_density = sigma_total / cdc->c_ref_f;
    cdc->bitstream_density = ideal_density + sigma_density;

    /* Clamp */
    if (cdc->bitstream_density < 0.0) cdc->bitstream_density = 0.0;
    if (cdc->bitstream_density > 1.0) cdc->bitstream_density = 1.0;

    cdc->c_measured_f = cdc->bitstream_density * cdc->c_ref_f;
    cdc->quantization_noise_f = sigma_total;

    /* SNR = 20 * log10(C_measured / sigma_total) */
    if (sigma_total > 0.0 && cdc->c_measured_f > 0.0) {
        cdc->snr_db = 20.0 * log10(cdc->c_measured_f / sigma_total);
    }
}

/**
 * cap_sigma_delta_enob
 *
 * Computes the Effective Number of Bits for a sigma-delta CDC.
 *
 * For an L-th order modulator with oversampling ratio OSR:
 *
 *   SQNR = 6.02*N + 1.76 + (20*L+10)*log10(OSR) - 10*log10(pi^(2L)/(2L+1))
 *
 * where N is the quantizer bits (1 for a comparator).
 *
 * For first-order (L=1) single-bit (N=1):
 *   SQNR ≈ 7.78 + 30*log10(OSR) - 10*log10(pi^2/3) ≈ 7.78 + 30*log10(OSR) - 5.17
 *        ≈ 2.61 + 30*log10(OSR)
 *
 *   OSR=64:  SQNR ≈ 2.61 + 30*1.806 ≈ 56.8 dB → ENOB ≈ (56.8-1.76)/6.02 ≈ 9.1
 *   OSR=256: SQNR ≈ 2.61 + 30*2.408 ≈ 74.9 dB → ENOB ≈ 12.1
 *   OSR=1024:SQNR ≈ 2.61 + 30*3.010 ≈ 92.9 dB → ENOB ≈ 15.1
 *
 * For second-order (L=2):
 *   SQNR ≈ 7.78 + 50*log10(OSR) - 10*log10(pi^4/5) ≈ 7.78 + 50*log10(OSR) - 12.9
 *        ≈ -5.12 + 50*log10(OSR)
 *
 *   OSR=64:  SQNR ≈ -5.12 + 50*1.806 ≈ 85.2 dB → ENOB ≈ 13.9
 *   OSR=256: SQNR ≈ -5.12 + 50*2.408 ≈ 115.3 dB → ENOB ≈ 18.9
 *
 * Ref: Schreier & Temes "Understanding Delta-Sigma Data Converters" (2005)
 *      §4.4 "SQNR of Delta-Sigma Modulators"
 */
double cap_sigma_delta_enob(uint32_t osr, uint32_t order)
{
    if (osr == 0) return 0.0;

    double sqnr_db;
    double n_bits = 1.0; /* Single-bit quantizer (comparator) */

    if (order == 2) {
        /* Second-order: SQNR ≈ 6.02*N + 1.76 + 50*log10(OSR) - 10*log10(pi^4/5) */
        sqnr_db = 6.02 * n_bits + 1.76 + 50.0 * log10((double)osr)
                  - 10.0 * log10(pow(M_PI, 4.0) / 5.0);
    } else {
        /* First-order: SQNR ≈ 6.02*N + 1.76 + 30*log10(OSR) - 10*log10(pi^2/3) */
        sqnr_db = 6.02 * n_bits + 1.76 + 30.0 * log10((double)osr)
                  - 10.0 * log10(M_PI * M_PI / 3.0);
    }

    /* ENOB = (SQNR - 1.76) / 6.02 */
    double enob = (sqnr_db - 1.76) / 6.02;
    if (enob < 0.0) enob = 0.0;

    return enob;
}

/* ==========================================================================
 * L2: RELAXATION OSCILLATOR
 * ========================================================================== */

void cap_relaxation_osc_init(cap_relaxation_osc_t *osc,
                             double c_sense, double r_timing,
                             double v_dd, double v_th_h, double v_th_l)
{
    if (!osc) return;
    memset(osc, 0, sizeof(*osc));
    osc->c_sense_f = c_sense;
    osc->r_timing_ohm = r_timing;
    osc->v_dd = v_dd;
    osc->v_th_high = v_th_h;
    osc->v_th_low = v_th_l;

    /* k-factor from comparator thresholds */
    if (v_th_h > v_th_l && v_dd > 0.0) {
        osc->k_factor = log((v_dd - v_th_l) / (v_dd - v_th_h));
    } else {
        osc->k_factor = log(3.0); /* Default for thresholds at 1/3 and 2/3 VDD */
    }
}

/**
 * cap_relaxation_osc_frequency
 *
 * Computes the oscillation frequency of a relaxation oscillator.
 *
 * For an RC oscillator with comparator thresholds V_th_high and V_th_low:
 *
 * Charging: V_C(t) = V_dd * (1 - e^{-t/RC}) + V_th_low * e^{-t/RC}
 * Time to reach V_th_high:
 *   t_charge = RC * ln((V_dd - V_th_low) / (V_dd - V_th_high))
 *
 * Discharging similarly. For symmetric thresholds:
 *   T_period = 2 * RC * ln((V_dd - V_th_low)/(V_dd - V_th_high))
 *
 * With V_th_l=V_dd/3, V_th_h=2*V_dd/3:
 *   T_period = 2 * RC * ln(2/3 / 1/3) = 2 * RC * ln(2) ≈ 1.386 * RC
 *
 * For R=100kOhm, C=10pF:
 *   T ≈ 1.386 * 1e5 * 1e-11 = 1.386 us → f ≈ 721 kHz
 *
 * Capacitance sensitivity:
 *   df/dC ≈ -f/C → For 1 fF change on 10 pF: df ≈ -721k * 0.0001 ≈ -72 Hz
 *
 * @param osc  Oscillator model
 * @return Frequency [Hz]
 */
double cap_relaxation_osc_frequency(const cap_relaxation_osc_t *osc)
{
    if (!osc) return 0.0;
    if (osc->c_sense_f <= 0.0 || osc->r_timing_ohm <= 0.0) return 0.0;

    double rc = osc->r_timing_ohm * osc->c_sense_f;
    double period = 2.0 * rc * osc->k_factor;

    if (period <= 0.0) return 0.0;
    return 1.0 / period;
}

/* ==========================================================================
 * L2: DUAL-SLOPE INTEGRATION
 * ========================================================================== */

void cap_dual_slope_init(cap_dual_slope_circuit_t *ds,
                         double c_sense, double v_ref,
                         double i_ref, double t_charge, double t_clk)
{
    if (!ds) return;
    memset(ds, 0, sizeof(*ds));
    ds->c_sense_f = c_sense;
    ds->v_ref = v_ref;
    ds->i_ref_a = i_ref;
    ds->t_charge_s = t_charge;
    ds->clock_period_s = t_clk;

    /* Discharge time: T_discharge = C_sense * V_ref / I_ref */
    ds->t_discharge_s = c_sense * v_ref / i_ref;
    ds->discharge_ticks = (uint32_t)(ds->t_discharge_s / t_clk);

    /* Resolution: minimum detectable deltaC = I_ref * T_clk / V_ref */
    ds->resolution_f = i_ref * t_clk / v_ref;

    /* Full-scale capacitance */
    double max_ticks = (t_charge * 10.0) / t_clk; /* Allow 10x charge time */
    ds->max_c_f = max_ticks * ds->resolution_f;
}

double cap_dual_slope_to_capacitance(const cap_dual_slope_circuit_t *ds)
{
    if (!ds) return 0.0;
    if (ds->v_ref <= 0.0 || ds->clock_period_s <= 0.0) return 0.0;

    /* C = I_ref * T_discharge / V_ref = I_ref * N_ticks * T_clk / V_ref */
    return ds->i_ref_a * (double)ds->discharge_ticks * ds->clock_period_s / ds->v_ref;
}

/* ==========================================================================
 * L2: AC BRIDGE
 * ========================================================================== */

void cap_ac_bridge_init(cap_ac_bridge_circuit_t *bridge,
                        double c_nom, double v_exc, double f_exc,
                        double tia_gain)
{
    if (!bridge) return;
    memset(bridge, 0, sizeof(*bridge));
    bridge->c_nominal_f = c_nom;
    bridge->v_exc = v_exc;
    bridge->f_exc_hz = f_exc;
    bridge->omega = 2.0 * M_PI * f_exc;
    bridge->tia_gain_v_a = tia_gain;
    bridge->sensitivity_v_per_ff = bridge->omega * v_exc * tia_gain * 0.5 * 1e-15;
}

/**
 * cap_ac_bridge_output
 *
 * Computes the TIA output voltage for a given capacitance imbalance.
 *
 * For a half-bridge with two nominally equal capacitors C, excited by V_exc:
 *
 *   i_imbalance = V_exc * j*omega * deltaC / 2   (assuming C_nom >> deltaC)
 *
 * TIA output: V_out = i_imbalance * R_feedback = V_exc * omega * deltaC * R_f / 2
 *
 * For V_exc=3.3V, f=100kHz, deltaC=1fF, R_f=100kOhm:
 *   V_out = 3.3 * 2*pi*1e5 * 1e-15 * 1e5 / 2 ≈ 0.104 uV → very small!
 *   Need R_f=1MOhm and gain=100 to get 10.4 mV/fF
 *
 * This illustrates why AC bridge measurement requires high-gain, low-noise
 * front-end amplifiers and is generally limited to fF resolution.
 *
 * @param bridge   AC bridge model
 * @param delta_c  Capacitance imbalance [F]
 * @return Output voltage [V]
 */
double cap_ac_bridge_output(const cap_ac_bridge_circuit_t *bridge,
                            double delta_c)
{
    if (!bridge || bridge->omega <= 0.0) return 0.0;

    /* I_imbalance = V * omega * delta_C (approximate, half-bridge) */
    double i_imbalance = bridge->v_exc * bridge->omega * delta_c * 0.5;

    /* V_out = I * TIA_gain */
    return i_imbalance * bridge->tia_gain_v_a;
}

/* ==========================================================================
 * L6: METHOD COMPARISON
 * ========================================================================== */

/**
 * cap_compare_measurement_methods
 *
 * Multi-criteria comparison of capacitance measurement methods.
 *
 * Scoring weights (typical for IoT/sensor applications):
 *   Resolution:  40% (how small a deltaC can be detected)
 *   Speed:       30% (conversion time)
 *   Complexity:  20% (BOM cost, PCB area, power)
 *   Immunity:    10% (noise rejection without extra filtering)
 *
 * Each criterion is normalized to [0,1] and weighted.
 * Score > 0 means method A is preferred; < 0 means method B.
 *
 * Typical rankings for general-purpose touch sensing:
 *   1. Charge transfer — best balance (simple, good resolution)
 *   2. Sigma-delta CDC — highest resolution at cost of complexity
 *   3. Relaxation oscillator — simplest, moderate performance
 *   4. AC bridge — best noise immunity, high BOM cost
 *   5. Dual-slope — slow but very precise (lab use)
 *   6. Resonant shift — highest sensitivity, needs inductor
 *
 * @return Score [-100, 100]
 */
double cap_compare_measurement_methods(cap_measurement_method_t method_a,
                                       cap_measurement_method_t method_b,
                                       double res_a, double time_a,
                                       double res_b, double time_b)
{
    /* Method base scores (0-10) on complexity and noise immunity */
    static const double complexity_score[] = {
        8.0,  /* CHARGE_TRANSFER: simple, just switches + comparator */
        4.0,  /* SIGMA_DELTA: needs modulator + digital filter */
        9.0,  /* RELAXATION_OSC: simplest, R + C + comparator */
        7.0,  /* DUAL_SLOPE: needs integrator + current source */
        5.0,  /* AC_BRIDGE: needs excitation + TIA + demodulator */
        3.0   /* RESONANT_SHIFT: needs inductor, precision frequency ref */
    };
    static const double immunity_score[] = {
        6.0,  /* CHARGE_TRANSFER: moderate */
        8.0,  /* SIGMA_DELTA: good (oversampling + noise shaping) */
        4.0,  /* RELAXATION_OSC: poor (sensitive to supply noise) */
        9.0,  /* DUAL_SLOPE: excellent (integrating, rejects periodic) */
        7.0,  /* AC_BRIDGE: good (synchronous detection) */
        5.0   /* RESONANT_SHIFT: moderate */
    };

    int ia = (int)method_a;
    int ib = (int)method_b;
    if (ia < 0 || ia >= CAP_METHOD_COUNT || ib < 0 || ib >= CAP_METHOD_COUNT) {
        return 0.0;
    }

    /* Resolution score: smaller is better → inverse normalized */
    double res_norm_a = (res_a > 0.0) ? 1.0e-15 / res_a : 1.0;
    double res_norm_b = (res_b > 0.0) ? 1.0e-15 / res_b : 1.0;

    /* Speed score: faster is better → inverse normalized */
    double speed_norm_a = (time_a > 0.0) ? 0.001 / time_a : 1.0; /* Reference 1ms */
    double speed_norm_b = (time_b > 0.0) ? 0.001 / time_b : 1.0;

    double score = 0.0;
    score += 40.0 * (res_norm_a - res_norm_b);
    score += 30.0 * (speed_norm_a - speed_norm_b);
    score += 20.0 * (complexity_score[ia] - complexity_score[ib]) / 10.0;
    score += 10.0 * (immunity_score[ia] - immunity_score[ib]) / 10.0;

    return score;
}
