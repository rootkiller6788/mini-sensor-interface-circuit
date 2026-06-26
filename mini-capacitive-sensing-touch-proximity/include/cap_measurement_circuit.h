/**
 * @file cap_measurement_circuit.h
 * @brief Capacitance Measurement Circuit Topologies and Transfer Functions
 *
 * Converting capacitance (fF-pF range) to a digital value requires careful
 * analog front-end design. This file models the six main measurement
 * topologies and their noise/resolution/speed tradeoffs.
 *
 * Knowledge Coverage:
 *   L1: resolution (fF/LSB), conversion time, input range, sampling rate
 *   L2: charge transfer, SigmaDelta CDC, relaxation oscillator, dual-slope
 *   L3: charge conservation, transfer function, noise shaping, oversampling
 *   L4: kT/C noise, charge injection analysis, settling time limits
 *   L5: auto-ranging, dual-slope integration timing, CDC decimation
 *   L6: complete capacitance-to-digital converter (CDC)
 *
 * Ref: TI SNOA927, ADI AN-1370 "Capacitance-to-Digital Converters"
 *      O'Dowd "Capacitive Sensor Interfaces" (Springer 2011)
 *      AD7745/AD7746 datasheets (24-bit CDC)
 */

#ifndef CAP_MEASUREMENT_CIRCUIT_H
#define CAP_MEASUREMENT_CIRCUIT_H

#include "cap_sense_core.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * L1: MEASUREMENT CIRCUIT TYPES
 * ========================================================================== */

/** Charge transfer measurement circuit model.
 *
 *  Principle: A sense capacitor C_s is repeatedly charged to V_dd and
 *  discharged into a larger integrating capacitor C_int. The number of
 *  charge transfers needed to reach a threshold voltage V_ref is:
 *
 *  N = (C_int * V_ref) / (C_s * V_dd)  (with compensation for switch loss)
 *
 *  Resolution is set by C_int and V_ref step size. A single transfer
 *  moves delta_V_int = V_dd * C_s / C_int.
 *
 *  For C_s = 10 pF, C_int = 10 nF, V_dd = 3.3 V:
 *  delta_V_int = 3.3 * 10p/10n = 3.3 mV per transfer.
 *  To reach V_ref = 1.65 V: N = 500 transfers.
 *
 *  Pros: simple, digital output (count), no ADC needed.
 *  Cons: slow for high resolution, C_int leakage causes drift.
 */
typedef struct {
    double   c_sense_f;          /**< Sense capacitor [F] */
    double   c_int_f;            /**< Integrating capacitor [F] */
    double   v_dd;               /**< Supply voltage [V] */
    double   v_ref;              /**< Comparator threshold [V] */
    double   v_int_initial;      /**< Initial integrator voltage [V] */
    double   charge_per_transfer_c; /**< Q per transfer = C_s * V_dd [C] */
    double   delta_v_per_transfer;  /**< V step per transfer = V_dd*C_s/C_int */
    uint32_t transfers_to_threshold;/**< N to reach V_ref */
    double   resolution_f;       /**< Capacitance resolution [F] */
    double   measurement_time_s; /**< Total conversion time [s] */
    double   switch_resistance_ohm; /**< Switch on-resistance [Ohm] */
    double   settling_time_s;    /**< RC settling time per transfer [s] */
} cap_charge_transfer_circuit_t;

/** Sigma-Delta Capacitance-to-Digital Converter (CDC) model.
 *
 *  Places the sense capacitor in the feedback path of a sigma-delta
 *  modulator. The output bitstream density is proportional to C_sense:
 *
 *  Density = C_sense / C_ref   (first-order SDM)
 *
 *  Benefits of sigma-delta:
 *  1. Noise shaping: quantization noise pushed to high frequencies
 *  2. Oversampling: each 4x OSR gives ~1.5 bits (first order)
 *  3. High resolution: 16-24 bits achievable with moderate OSR
 *  4. Inherently digital output
 *
 *  For first-order SDM with OSR = N:
 *    SNR = 6.02*N_bits + 1.76 - 5.17 + 30*log10(OSR)  [dB]
 *
 *  Example: OSR=256, first-order -> SNR ~ 6.02+1.76-5.17+30*log10(256)
 *           ≈ 2.61 + 72.25 = 74.86 dB → ~12.1 ENOB equivalent
 */
typedef struct {
    double   c_sense_f;          /**< Sense capacitor [F] */
    double   c_ref_f;            /**< Reference capacitor [F] */
    double   v_exc;              /**< Excitation voltage [V] */
    double   v_ref;              /**< Reference voltage [V] */
    uint32_t osr;                /**< Oversampling ratio */
    uint32_t modulator_order;    /**< SDM order (1 or 2) */
    uint32_t bitstream_length;   /**< Total bitstream samples */
    double   bitstream_density;  /**< Fraction of 1s = C_sense/C_ref */
    double   c_measured_f;       /**< C_sense = bitstream_density * C_ref [F] */
    double   quantization_noise_f; /**< RMS quantization noise in C [F] */
    double   enob;               /**< Effective number of bits */
    double   snr_db;             /**< SNR [dB] */
    uint32_t conversion_time_s;  /**< Total conversion time [s] */
} cap_sigma_delta_cdc_t;

/** Relaxation oscillator measurement model.
 *
 *  The sense capacitor C_s is part of an RC relaxation oscillator.
 *  Oscillation period: T = k * R * C_s where k depends on comparator
 *  thresholds.
 *
 *  Delta-C is measured as a frequency shift:
 *  f = 1/T = 1/(k*R*C_s)
 *  delta_f ≈ -f0 * deltaC / C_s0  (first-order approximation)
 *
 *  Pros: simple (comparator + resistor + counter), continuous measurement.
 *  Cons: sensitive to temperature (R tempco, comparator delay tempco),
 *        limited resolution without precision timing.
 */
typedef struct {
    double   c_sense_f;          /**< Sense capacitor [F] */
    double   r_timing_ohm;       /**< Timing resistor [Ohm] */
    double   v_th_high;          /**< Comparator high threshold [V] */
    double   v_th_low;           /**< Comparator low threshold [V] */
    double   v_dd;               /**< Supply voltage [V] */
    double   k_factor;           /**< Geometric factor from thresholds */
    double   period_s;           /**< Oscillation period [s] */
    double   frequency_hz;       /**< Oscillation frequency [Hz] */
    double   delta_f_per_ff;     /**< Frequency sensitivity [Hz/fF] */
    uint32_t gate_time_s;        /**< Frequency counting gate time [s] */
    uint32_t count;              /**< Oscillation count in gate time */
    double   resolution_f;       /**< Capacitance resolution [F] */
} cap_relaxation_osc_t;

/** Dual-slope integration measurement model.
 *
 *  Phase 1 (charge): Charge C_sense to V_ref for fixed time T_charge.
 *  Phase 2 (discharge): Discharge through known current I_ref, measure
 *  time to reach zero. T_discharge = C_sense * V_ref / I_ref.
 *
 *  deltaC is measured as: delta_T = deltaC * V_ref / I_ref
 *
 *  Benefits: ratiometric (eliminates V_ref and clock errors if same
 *  reference used for both phases), good noise rejection (integration).
 */
typedef struct {
    double   c_sense_f;          /**< Sense capacitor [F] */
    double   v_ref;              /**< Reference voltage [V] */
    double   i_ref_a;            /**< Discharge current [A] */
    double   t_charge_s;         /**< Phase 1 charge time [s] */
    double   t_discharge_s;      /**< Phase 2 discharge time [s] */
    double   clock_period_s;     /**< Timer clock period [s] */
    uint32_t discharge_ticks;    /**< Timer counts in discharge phase */
    double   resolution_f;       /**< C resolution = I_ref * T_clk / V_ref [F] */
    double   max_c_f;            /**< Full-scale capacitance [F] */
} cap_dual_slope_circuit_t;

/** AC bridge measurement model.
 *
 *  A balanced capacitive bridge with excitation V_exc at frequency f_exc.
 *  Capacitance imbalance deltaC produces a current:
 *
 *  i_imbalance = V_exc * j*omega * deltaC / 2 (half-bridge)
 *
 *  This current is amplified by a transimpedance amplifier (TIA) and
 *  synchronously demodulated.
 *
 *  SNR improves with excitation voltage and frequency, limited by
 *  amplifier noise and parasitic coupling.
 */
typedef struct {
    double   c_nominal_f;        /**< Nominal bridge capacitor [F] */
    double   v_exc;              /**< Excitation voltage amplitude [V] */
    double   f_exc_hz;           /**< Excitation frequency [Hz] */
    double   omega;              /**< Angular frequency = 2*pi*f [rad/s] */
    double   tia_gain_v_a;       /**< Transimpedance gain [V/A] */
    double   imbalance_current_a; /**< Bridge imbalance current [A] */
    double   output_voltage_v;   /**< TIA output voltage [V] */
    double   sensitivity_v_per_ff; /**< Output sensitivity [V/fF] */
    double   noise_density_v_sqrt_hz; /**< TIA input noise [V/sqrt(Hz)] */
    double   bw_hz;              /**< Measurement bandwidth [Hz] */
    double   snr_db;             /**< SNR [dB] */
} cap_ac_bridge_circuit_t;

/* ==========================================================================
 * L2-L5: API
 * ========================================================================== */

/** Initialize charge transfer circuit model.
 *
 *  @param ct       Circuit model to initialize
 *  @param c_sense  Sense capacitance [F]
 *  @param c_int    Integrating capacitor [F]
 *  @param v_dd     Supply voltage [V]
 *  @param v_ref    Comparator threshold [V]
 */
void cap_charge_transfer_init(cap_charge_transfer_circuit_t *ct,
                              double c_sense, double c_int,
                              double v_dd, double v_ref);

/** Compute number of charge transfers to reach threshold.
 *
 *  N = ceil(C_int * V_ref / (C_sense * V_dd))
 *
 *  @param ct  Initialized charge transfer circuit
 *  @return Number of transfers needed
 */
uint32_t cap_charge_transfer_count(const cap_charge_transfer_circuit_t *ct);

/** Convert transfer count back to capacitance.
 *
 *  C_sense = C_int * V_ref / (N * V_dd)
 *
 *  @param ct             Circuit (uses C_int, V_ref, V_dd)
 *  @param transfer_count Measured transfer count
 *  @return Estimated C_sense [F]
 */
double cap_charge_transfer_to_capacitance(const cap_charge_transfer_circuit_t *ct,
                                          uint32_t transfer_count);

/** Initialize sigma-delta CDC model.
 *
 *  @param cdc      CDC model to initialize
 *  @param c_ref    Reference capacitor [F]
 *  @param v_exc    Excitation voltage [V]
 *  @param osr      Oversampling ratio
 *  @param order    Modulator order (1 or 2)
 */
void cap_sigma_delta_cdc_init(cap_sigma_delta_cdc_t *cdc,
                              double c_ref, double v_exc,
                              uint32_t osr, uint32_t order);

/** Simulate one sigma-delta bitstream and compute C_sense.
 *
 *  @param cdc          CDC model (updated with result)
 *  @param c_sense      Sense capacitance to measure [F]
 *  @param noise_rms    Input-referred noise [F]
 */
void cap_sigma_delta_convert(cap_sigma_delta_cdc_t *cdc,
                             double c_sense, double noise_rms);

/** Compute ENOB from sigma-delta parameters.
 *
 *  ENOB = (SNR_dB - 1.76) / 6.02
 *  where SNR for L-th order SDM:
 *    SNR = 6.02*N + 1.76 + (20*L+10)*log10(OSR) - 10*log10(pi^(2L)/(2L+1))
 *
 *  @param osr    Oversampling ratio
 *  @param order  Modulator order
 *  @return ENOB [bits]
 */
double cap_sigma_delta_enob(uint32_t osr, uint32_t order);

/** Initialize relaxation oscillator model.
 *
 *  @param osc     Oscillator model
 *  @param c_sense Sense capacitance [F]
 *  @param r_timing Timing resistor [Ohm]
 *  @param v_dd    Supply voltage [V]
 *  @param v_th_h  Comparator high threshold [V]
 *  @param v_th_l  Comparator low threshold [V]
 */
void cap_relaxation_osc_init(cap_relaxation_osc_t *osc,
                             double c_sense, double r_timing,
                             double v_dd, double v_th_h, double v_th_l);

/** Compute frequency from relaxation oscillator parameters.
 *
 *  @param osc  Oscillator model
 *  @return Frequency [Hz]
 */
double cap_relaxation_osc_frequency(const cap_relaxation_osc_t *osc);

/** Initialize dual-slope measurement circuit.
 *
 *  @param ds     Dual-slope model
 *  @param c_sense Sense capacitance [F]
 *  @param v_ref  Reference voltage [V]
 *  @param i_ref  Discharge current [A]
 *  @param t_charge Charge time [s]
 *  @param t_clk  Timer clock period [s]
 */
void cap_dual_slope_init(cap_dual_slope_circuit_t *ds,
                         double c_sense, double v_ref, double i_ref,
                         double t_charge, double t_clk);

/** Convert discharge time to capacitance.
 *
 *  @param ds  Dual-slope model
 *  @return Measured capacitance [F]
 */
double cap_dual_slope_to_capacitance(const cap_dual_slope_circuit_t *ds);

/** Initialize AC bridge measurement model.
 *
 *  @param bridge   AC bridge model
 *  @param c_nom    Nominal capacitor value [F]
 *  @param v_exc    Excitation amplitude [V]
 *  @param f_exc    Excitation frequency [Hz]
 *  @param tia_gain Transimpedance gain [V/A]
 */
void cap_ac_bridge_init(cap_ac_bridge_circuit_t *bridge,
                        double c_nom, double v_exc, double f_exc,
                        double tia_gain);

/** Compute bridge output voltage for a given delta-C.
 *
 *  @param bridge  AC bridge model
 *  @param delta_c Capacitance imbalance [F]
 *  @return Output voltage [V]
 */
double cap_ac_bridge_output(const cap_ac_bridge_circuit_t *bridge,
                            double delta_c);

/** Compare key metrics of two measurement methods.
 *
 *  Returns a cost function score (0-100) combining:
 *  resolution (40%), speed (30%), complexity (20%), noise immunity (10%).
 *
 *  @param method_a  First method
 *  @param method_b  Second method
 *  @param res_a     Resolution of method a [F]
 *  @param time_a    Conversion time of method a [s]
 *  @param res_b     Resolution of method b [F]
 *  @param time_b    Conversion time of method b [s]
 *  @return Score [-100, 100], positive = method_a better
 */
double cap_compare_measurement_methods(cap_measurement_method_t method_a,
                                       cap_measurement_method_t method_b,
                                       double res_a, double time_a,
                                       double res_b, double time_b);

#endif /* CAP_MEASUREMENT_CIRCUIT_H */
