/**
 * bridge_excitation.h — Bridge Excitation and Power Supply Design
 *
 * Knowledge Coverage:
 *   L2: Voltage vs. current excitation, ratiometric measurement
 *   L3: Excitation stability analysis, noise contribution
 *   L4: Ohm's law in bridge context, Johnson-Nyquist noise
 *   L5: Precision voltage reference design, current source design
 *
 * Reference:
 *   - Kester, "Sensor Signal Conditioning", Analog Devices, Ch.4
 *   - Horowitz & Hill, "The Art of Electronics", 3rd ed., Ch.8
 *   - Baker, "A Designer's Guide to Instrumentation Amplifiers", 3rd ed.
 *
 * Course: MIT 6.002, Berkeley EE105, TU Munich EI0430
 */

#ifndef BRIDGE_EXCITATION_H
#define BRIDGE_EXCITATION_H

#include "bridge_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * L2: Excitation Mode Analysis
 * ======================================================================== */

/**
 * Excitation source descriptor.
 *
 * The excitation quality directly limits measurement accuracy:
 * - Voltage excitation: output proportional to Vexc (1% Vexc error = 1% strain error)
 * - Current excitation: output proportional to Iexc, independent of lead R
 * - Ratiometric: uses same reference for ADC and bridge, cancels drift
 */
typedef struct {
    excitation_mode_t mode;
    double nominal_voltage;        /* Setpoint for voltage excitation [V] */
    double nominal_current;        /* Setpoint for current excitation [A] */
    double voltage_accuracy_percent; /* Voltage accuracy [%] */
    double current_accuracy_percent; /* Current accuracy [%] */
    double tempco_ppm_per_c;       /* Temperature coefficient [ppm/K] */
    double noise_uv_rms;           /* Output noise (0.1-10 Hz) [uVrms] */
    double line_regulation_ppm;    /* Line regulation [ppm/V] */
    double load_regulation_ppm;    /* Load regulation [ppm/mA] */
    double output_impedance_ohm;   /* Source output impedance [Ohm] */
    double max_compliance_v;       /* Max compliance voltage [V] */
} excitation_source_t;

/**
 * Compute bridge output with voltage excitation.
 * Accounts for source impedance loading effect.
 *
 * Vout = Vexc * [R3/(R1+R3) - R4/(R2+R4)]
 * where Vexc is the actual voltage at bridge terminals,
 * accounting for source impedance Zsrc:
 *   Vexc_actual = Vsrc * Zbridge / (Zbridge + Zsrc)
 *
 * @param bridge     Bridge state
 * @param source     Excitation source parameters
 * @return           Actual bridge output voltage [V]
 *
 * Complexity: O(1).
 */
double excitation_voltage_output(const bridge_state_t *bridge,
                                 const excitation_source_t *source);

/**
 * Compute bridge output with current excitation.
 * Current excitation eliminates lead resistance effects because
 * the sense leads carry negligible current.
 *
 * For current Iexc through the bridge:
 *   Vout = Iexc * [R3*R1/(R1+R3) - R4*R2/(R2+R4)] / 2
 *
 * This is the difference between the parallel combinations of
 * each half-bridge, scaled by the excitation current.
 *
 * @param bridge     Bridge state
 * @param source     Excitation source (current mode)
 * @return           Bridge output voltage [V]
 *
 * Complexity: O(1).
 */
double excitation_current_output(const bridge_state_t *bridge,
                                 const excitation_source_t *source);

/**
 * Compute ratiometric transfer function.
 * When ADC reference = bridge excitation (or proportional),
 * the digitized value is independent of excitation drift:
 *
 *   ADC_code = (Vout / Vexc) * (2^N - 1) * gain
 *
 * Ratiometric operation eliminates excitation as an error source,
 * leaving only bridge ratio and amplifier errors.
 *
 * @param vout        Bridge output [V]
 * @param vref        ADC reference voltage [V] = k * Vexc
 * @param adc_bits    ADC resolution [bits]
 * @param gain        Amplifier gain [V/V]
 * @return            ADC output code [LSB]
 *
 * Complexity: O(1).
 */
double excitation_ratiometric_code(double vout, double vref,
                                   int adc_bits, double gain);

/* ========================================================================
 * L3: Excitation Stability and Error Analysis
 * ======================================================================== */

/**
 * Compute excitation-induced strain measurement error.
 *
 * For voltage excitation, any error in Vexc directly maps to strain:
 *   eps_error = (dVexc / Vexc) * (4 / GF)  [quarter bridge]
 *
 * Example: GF=2, Vexc=5V, dVexc=1mV (0.02%)
 *   eps_error = 0.0002 * 4/2 = 0.0004 = 400 microstrain
 * This is a LARGE error for precision measurement!
 * This is why ratiometric or high-stability excitation is essential.
 *
 * @param vex_nominal    Nominal excitation [V]
 * @param vex_actual     Actual excitation [V]
 * @param gf             Gauge factor
 * @param config         Bridge configuration
 * @return               Strain measurement error [microstrain]
 *
 * Complexity: O(1).
 */
double excitation_strain_error(double vex_nominal, double vex_actual,
                               double gf, bridge_config_t config);

/**
 * Compute Johnson-Nyquist noise of bridge resistance.
 *
 * Johnson-Nyquist theorem (1928):
 *   Vn_rms = sqrt(4 * kB * T * R * BW)
 *
 * where:
 *   kB = Boltzmann constant = 1.380649e-23 J/K
 *   T  = absolute temperature [K]
 *   R  = resistance [Ohm]
 *   BW = noise bandwidth [Hz]
 *
 * Example: R=350 Ohm, T=300K, BW=100 Hz
 *   Vn_rms = sqrt(4*1.38e-23*300*350*100)
 *          = sqrt(5.80e-16) = 24 nVrms
 *
 * This is the fundamental noise limit. Real circuits add more noise.
 *
 * @param resistance    Bridge output resistance [Ohm]
 * @param temperature_k Absolute temperature [K]
 * @param bandwidth_hz  Noise bandwidth [Hz]
 * @return              RMS noise voltage [Vrms]
 *
 * Complexity: O(1).
 * Reference: Nyquist, "Thermal Agitation of Electric Charge", Phys Rev 1928
 */
double excitation_johnson_noise(double resistance, double temperature_k,
                                double bandwidth_hz);

/**
 * Compute total noise at bridge output.
 *
 * Total noise = sqrt(Johnson^2 + source_noise^2 + amp_noise^2)
 *
 * Combining uncorrelated noise sources by root-sum-square.
 *
 * @param johnson_noise   Johnson noise [Vrms]
 * @param source_noise    Excitation source noise [Vrms]
 * @param amp_noise_rti   Amplifier input-referred noise [Vrms]
 * @return                Total RMS noise [Vrms]
 *
 * Complexity: O(1).
 */
double excitation_total_noise(double johnson_noise, double source_noise,
                              double amp_noise_rti);

/**
 * Compute optimal excitation voltage for minimum self-heating error.
 *
 * Trade-off:
 * - Higher Vexc → higher SNR (signal grows linearly with Vexc)
 * - Higher Vexc → higher power dissipation (P grows quadratically)
 * - Self-heating causes apparent strain via gauge TCR
 *
 * Optimal: Vexc_opt = sqrt(P_max * R_bridge)
 * where P_max is limited by acceptable self-heating temperature rise.
 *
 * For a 350 Ohm gauge, P_max ~ 5 mW per gauge (10 degC rise on steel):
 *   Vexc_opt = sqrt(0.005 * 350) = 1.32 V (single gauge)
 *
 * For full bridge (4 gauges, 350 Ohm each): P_distributed across 4 gauges
 *   Vexc_opt = sqrt(0.020 * 350) = 2.65 V
 *
 * @param r_bridge      Bridge total resistance [Ohm]
 * @param max_power_w   Maximum allowable total power [W]
 * @return              Optimal excitation voltage [V]
 *
 * Complexity: O(1).
 */
double excitation_optimal_voltage(double r_bridge, double max_power_w);

/* ========================================================================
 * L4: Precision Excitation Circuit Design
 * ======================================================================== */

/**
 * Voltage reference parameters for precision excitation.
 *
 * Key specifications for bridge excitation:
 * - Initial accuracy: determines factory calibration requirement
 * - Temperature drift: dominates measurement drift over temperature
 * - Noise: limits resolution (especially 0.1-10 Hz flicker)
 * - Long-term stability: affects recalibration interval
 */
typedef struct {
    double output_voltage;         /* Nominal output [V] */
    double initial_accuracy_percent; /* Initial tolerance [%] */
    double tempco_ppm_per_c;       /* Temperature drift [ppm/K] */
    double noise_uv_pp;            /* 0.1-10 Hz peak-peak noise [uVpp] */
    double long_term_stability_ppm; /* Per 1000 hours [ppm] */
    double line_regulation_ppm_per_v; /* [ppm/V] */
    double load_regulation_ppm_per_ma; /* [ppm/mA] */
    double dropout_voltage_mv;     /* Minimum headroom [mV] */
    double quiescent_current_ua;   /* Supply current [uA] */
} voltage_reference_t;

/**
 * Design a precision voltage excitation source.
 *
 * Computes the components for a buffered precision voltage reference:
 * - Reference IC (e.g., ADR4525, REF5025, MAX6325)
 * - Output buffer (low drift op-amp in unity gain)
 * - Remote sense connections for load regulation
 *
 * The buffer must:
 * 1. Drive the bridge load without gain error
 * 2. Maintain low output impedance at measurement BW
 * 3. Not add significant noise or drift
 *
 * @param ref           Voltage reference specs
 * @param bridge_r_ohm  Bridge input resistance [Ohm]
 * @param max_error_percent  Max allowable excitation error [%]
 * @return              0 if design feasible, -1 if constraints violated
 *
 * Complexity: O(1).
 */
int excitation_design_voltage_source(const voltage_reference_t *ref,
                                     double bridge_r_ohm,
                                     double max_error_percent);

/**
 * Compute excitation current for optimal bridge performance.
 *
 * For current-driven bridges:
 *   Iexc_opt = Vexc_opt / R_bridge
 *
 * Current excitation advantages:
 * - Lead resistance eliminated (sense leads carry ~0 current)
 * - Linear output for single-element variation (quarter bridge)
 * - Constant power dissipation across arms
 *
 * Disadvantages:
 * - Requires precision current source (more complex than voltage ref)
 * - Higher noise (current noise * bridge R adds voltage noise)
 *
 * @param r_bridge      Bridge input resistance [Ohm]
 * @param target_power_w Desired total bridge power [W]
 * @return              Excitation current [A]
 *
 * Complexity: O(1).
 */
double excitation_current_for_power(double r_bridge, double target_power_w);

/**
 * Calculate bridge power supply rejection ratio requirement.
 *
 * PSRR_required = 20 * log10(dVexc / (Vout_min * gain))
 *
 * where dVexc is the worst-case excitation variation and
 * Vout_min is the minimum resolvable output voltage.
 *
 * Example: dVexc=10mV (0.2% of 5V), Vout_min=1uV, gain=100
 *   PSRR_required = 20*log10(0.01/(1e-6*100)) = 20*log10(100) = 40 dB
 *
 * @param vex_nominal     Nominal excitation [V]
 * @param vex_variation   Worst-case variation [V]
 * @param vout_min        Minimum resolvable output [V]
 * @param gain            Amplifier gain [V/V]
 * @return                Required PSRR [dB]
 *
 * Complexity: O(1).
 */
double excitation_psrr_requirement(double vex_nominal, double vex_variation,
                                   double vout_min, double gain);

/**
 * Initialize excitation source with typical defaults.
 */
void excitation_source_init(excitation_source_t *src, excitation_mode_t mode);

/**
 * Initialize voltage reference with standard precision specs.
 */
void voltage_reference_init(voltage_reference_t *ref, double vout);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_EXCITATION_H */
