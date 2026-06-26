/**
 * bridge_excitation.c — Bridge Excitation Implementation
 *
 * Knowledge Coverage:
 *   L2: Voltage vs. current excitation analysis
 *   L3: Excitation stability, noise calculations
 *   L4: Johnson-Nyquist noise, optimal excitation design
 *   L5: Precision voltage reference design
 *
 * Reference:
 *   - Kester, "Sensor Signal Conditioning", Analog Devices, Ch.4
 *   - Horowitz & Hill, "The Art of Electronics", 3rd ed., Ch.8
 *   - Nyquist, "Thermal Agitation of Electric Charge", Phys Rev 1928
 */

#include "bridge_excitation.h"
#include <math.h>
#include <string.h>

/* Boltzmann constant [J/K] — exact per CODATA 2018 */
#define BOLTZMANN_K 1.380649e-23

double excitation_voltage_output(const bridge_state_t *bridge,
                                 const excitation_source_t *source)
{
    /* Bridge output accounting for voltage source impedance.
     *
     * When a voltage source has non-zero output impedance Zsrc,
     * the actual voltage at the bridge terminals is:
     *
     *   V_bridge = V_src * Z_bridge / (Z_bridge + Z_src)
     *
     * where Z_bridge is the bridge input impedance.
     *
     * This is the classic voltage divider loading effect.
     *
     * For a typical precision source (Zsrc < 0.1 Ohm) driving
     * a 350 Ohm bridge, the loading error is:
     *   error = 0.1 / (350 + 0.1) = 0.029% — usually acceptable.
     *
     * But for long cables (Zsrc = cable resistance + source R),
     * the error can become significant if not using 4-wire sensing.
     */

    double z_bridge = bridge_input_impedance(bridge);
    if (z_bridge <= 0.0) return 0.0;

    double z_src = source->output_impedance_ohm;
    double vex_actual = source->nominal_voltage * z_bridge / (z_bridge + z_src);

    /* Use the actual bridge equation with corrected excitation */
    bridge_state_t temp = *bridge;
    temp.v_excitation = vex_actual;
    return bridge_output_voltage(&temp);
}

double excitation_current_output(const bridge_state_t *bridge,
                                 const excitation_source_t *source)
{
    /* Bridge output with current excitation.
     *
     * With constant current Iexc flowing through the bridge:
     *
     * The voltage across the bridge: V_bridge = Iexc * Z_bridge
     *
     * The differential output using superposition:
     *   Vout = Iexc * (R1||R3 - R2||R4) / 2
     *
     * Wait, let me derive this properly.
     *
     * With current excitation, the same current Iexc flows
     * through R1+R3 path and R2+R4 path in parallel.
     *
     * By current division:
     *   I_left  = Iexc * (R2+R4) / (R1+R2+R3+R4)
     *   I_right = Iexc * (R1+R3) / (R1+R2+R3+R4)
     *
     * The voltage at the left output node (between R1 and R3):
     *   V_left  = I_left * R3 = Iexc * R3*(R2+R4)/(R1+R2+R3+R4)
     *
     * The voltage at the right output node (between R2 and R4):
     *   V_right = I_right * R4 = Iexc * R4*(R1+R3)/(R1+R2+R3+R4)
     *
     * Differential output:
     *   Vout = V_left - V_right
     *        = Iexc * [R3*(R2+R4) - R4*(R1+R3)] / (R1+R2+R3+R4)
     *
     * For a balanced bridge (all R=R0):
     *   Vout = Iexc * [R0*(2R0) - R0*(2R0)] / (4R0) = 0 ✓
     *
     * For R1 = R0*(1+eps*GF), others = R0 (quarter bridge):
     *   Let's compute numerically rather than algebraically.
     *   The key advantage: current excitation eliminates lead
     *   resistance effects because sense leads carry ~0 current.
     */

    double r_total = bridge->r1 + bridge->r2 + bridge->r3 + bridge->r4;
    if (r_total <= 0.0) return 0.0;

    double i_left  = source->nominal_current * (bridge->r2 + bridge->r4) / r_total;
    double i_right = source->nominal_current * (bridge->r1 + bridge->r3) / r_total;

    double v_left  = i_left * bridge->r3;
    double v_right = i_right * bridge->r4;

    return v_left - v_right;
}

double excitation_ratiometric_code(double vout, double vref,
                                   int adc_bits, double gain)
{
    /* Ratiometric ADC conversion.
     *
     * When the ADC reference voltage is derived from (or proportional to)
     * the bridge excitation, the digitized reading becomes independent
     * of excitation drift:
     *
     *   ADC_code = (Vout / Vref) * (2^N - 1) * Gain
     *
     * Ratiometric operation:
     * - Bridge: Vout = Vexc * f(epsilon)  [f is the bridge function]
     * - ADC:    code = (Vout_gained / Vref) * (2^N - 1)
     *           where Vout_gained = Vout * Gain
     *           and Vref = k * Vexc
     *
     * - code = (Vexc * f(eps) * Gain) / (k * Vexc) * (2^N - 1)
     *        = f(eps) * Gain/k * (2^N - 1)
     *
     * Vexc cancels! The reading depends only on strain and fixed
     * parameters.
     *
     * This is the standard approach for load cells and pressure
     * sensors. The ADC reference is often the same supply that
     * powers the bridge (possibly buffered).
     *
     * Common practice: use a 5V supply for both bridge and ADC ref.
     * The ADC reads the ratio Vout/Vexc directly.
     */

    if (vref <= 0.0 || adc_bits <= 0) return 0.0;

    double full_scale_codes = (1 << adc_bits) - 1;
    double ratio = (vout * gain) / vref;

    /* Clamp to valid range */
    if (ratio > 0.9999) ratio = 0.9999;
    if (ratio < -0.0001) ratio = -0.0001;

    return ratio * full_scale_codes;
}

double excitation_strain_error(double vex_nominal, double vex_actual,
                               double gf, bridge_config_t config)
{
    /* Strain measurement error caused by excitation variation.
     *
     * For non-ratiometric systems, any error in excitation voltage
     * maps directly to strain measurement error:
     *
     *   eps_error = (dVexc/Vexc) * (N/GF)
     *
     * where N = 4 (quarter), 2 (half), 1 (full).
     *
     * This is why precision bridge measurement requires:
     * - Ratiometric ADC configuration (self-correcting), OR
     * - High-stability excitation (< 10 ppm/K), OR
     * - Regular calibration against a known reference
     */

    if (vex_nominal <= 0.0 || gf <= 0.0) return 0.0;

    double relative_error = (vex_actual - vex_nominal) / vex_nominal;
    double n_factor;

    switch (config) {
        case BRIDGE_QUARTER:   n_factor = 4.0; break;
        case BRIDGE_HALF:      n_factor = 2.0; break;
        case BRIDGE_FULL:      n_factor = 1.0; break;
        default:               n_factor = 4.0; break;
    }

    return relative_error * (n_factor / gf) * 1.0e6;  /* To microstrain */
}

double excitation_johnson_noise(double resistance, double temperature_k,
                                double bandwidth_hz)
{
    /* Johnson-Nyquist thermal noise (Johnson 1928, Nyquist 1928).
     *
     *   Vn_rms = sqrt(4 * kB * T * R * BW)
     *
     * This is the fundamental thermodynamic noise limit for any
     * resistive circuit. It arises from thermal agitation of
     * charge carriers (Brownian motion of electrons).
     *
     * Key properties:
     * - White noise (flat power spectral density up to ~THz)
     * - Proportional to sqrt(T): cooling reduces noise
     * - Proportional to sqrt(R): lower bridge R = lower noise
     *   BUT higher R allows higher Vexc for same self-heating
     *   (the SNR stays approximately constant because both
     *    signal and noise scale with sqrt(R))
     *
     * Example: R=350 Ohm, T=300 K, BW=100 Hz
     *   Vn = sqrt(4 * 1.381e-23 * 300 * 350 * 100)
     *      = sqrt(5.80e-16) = 2.41e-8 V = 24.1 nVrms
     *
     * For a 350 Ohm full bridge at 5V excitation:
     *   Vout_FS = 5 * 2.05 * 0.001 = 10.25 mV (full bridge, 1000 ue)
     *   SNR = 10.25e-3 / 24.1e-9 = 425,000
     *   Equivalent resolution: log2(425000) ≈ 18.7 bits
     *   → Johnson noise alone supports ~18-19 bit resolution!
     *
     * In practice, amplifier noise (10-50 nV/rtHz) and ADC
     * quantization limit achievable resolution to ~14-16 bits.
     */

    if (resistance < 0.0 || temperature_k < 0.0 || bandwidth_hz < 0.0) {
        return 0.0;
    }

    return sqrt(4.0 * BOLTZMANN_K * temperature_k * resistance * bandwidth_hz);
}

double excitation_total_noise(double johnson_noise, double source_noise,
                              double amp_noise_rti)
{
    /* Total RMS noise: root-sum-square of uncorrelated sources.
     *
     * Uncorrelated noise sources add as variances (power), not
     * voltages:
     *
     *   Vn_total = sqrt(Vn_johnson^2 + Vn_source^2 + Vn_amp^2)
     *
     * Correlated noise sources (e.g., power supply ripple at
     * both amplifier inputs) would add linearly, but this is
     * generally rejected by CMRR > 100 dB.
     */

    return sqrt(johnson_noise * johnson_noise +
                source_noise  * source_noise +
                amp_noise_rti * amp_noise_rti);
}

double excitation_optimal_voltage(double r_bridge, double max_power_w)
{
    /* Optimal excitation voltage for minimum self-heating error.
     *
     * The bridge output signal grows linearly with Vexc:
     *   Vout = Vexc * GF/4 * eps  (linear, quarter bridge)
     *
     * But the power dissipated grows quadratically:
     *   P = Vexc^2 / R_bridge
     *
     * Self-heating temperature rise:
     *   dT = P * R_thermal
     *
     * Apparent strain from self-heating:
     *   eps_heating = dT * (alpha_specimen + thermal_output_coeff)
     *
     * The optimal Vexc balances SNR improvement against
     * self-heating error:
     *
     *   d(SNR)/dVexc = d(Signal/Noise)/dVexc
     *
     * For Johnson-noise-limited measurement:
     *   Signal ∝ Vexc
     *   Noise ∝ sqrt(R) (independent of Vexc for fixed T)
     *   → SNR ∝ Vexc (higher is always better for signal)
     *
     * BUT self-heating limits practical Vexc:
     *   P_max determines Vexc_max = sqrt(P_max * R_bridge)
     *
     * @param r_bridge: Bridge input resistance [Ohm]
     * @param max_power_w: Maximum allowable total bridge power [W]
     * @return Optimal excitation voltage [V]
     */

    if (r_bridge <= 0.0 || max_power_w <= 0.0) return 0.0;

    return sqrt(max_power_w * r_bridge);
}

int excitation_design_voltage_source(const voltage_reference_t *ref,
                                     double bridge_r_ohm,
                                     double max_error_percent)
{
    /* Validate voltage reference for bridge excitation.
     *
     * Checks if the reference can drive the bridge load while
     * maintaining the required accuracy.
     *
     * Key checks:
     * 1. Load regulation: Vref droop under bridge load
     * 2. Output current: does Vref/bridge_R exceed ref's max load?
     * 3. Temperature drift: over expected dT range
     *
     * Returns 0 if design is feasible, -1 if constraints violated.
     */

    if (ref == NULL || bridge_r_ohm <= 0.0) return -1;

    /* Check 1: Load current */
    double load_current_ma = (ref->output_voltage / bridge_r_ohm) * 1000.0;

    /* Typical precision refs can source 5-30 mA.
     * Use a conservative 10 mA limit. */
    if (load_current_ma > 10.0) {
        return -1;  /* Would exceed typical reference drive capability */
    }

    /* Check 2: Load regulation error */
    double load_reg_error_percent =
        (ref->load_regulation_ppm_per_ma * load_current_ma) / 10000.0;

    if (load_reg_error_percent > max_error_percent) {
        return -1;
    }

    /* Check 3: Total error (initial + temp + long-term) */
    double temp_error_percent =
        (ref->tempco_ppm_per_c * 50.0) / 10000.0;  /* Assume 50K range */

    double total_error = ref->initial_accuracy_percent +
                         temp_error_percent +
                         (ref->long_term_stability_ppm / 10000.0) * 10.0 +
                         load_reg_error_percent;

    if (total_error > max_error_percent) {
        return -1;
    }

    return 0;
}

double excitation_current_for_power(double r_bridge, double target_power_w)
{
    /* Compute excitation current for a given bridge power dissipation.
     *
     * P = I^2 * R → I = sqrt(P / R)
     *
     * The same power formula applies regardless of excitation mode.
     *
     * For target power of 10 mW in a 350 Ohm bridge:
     *   I = sqrt(0.01/350) = sqrt(2.857e-5) = 5.35 mA
     *
     * This is a modest current that most precision current sources
     * can provide.
     */

    if (r_bridge <= 0.0 || target_power_w <= 0.0) return 0.0;

    return sqrt(target_power_w / r_bridge);
}

double excitation_psrr_requirement(double vex_nominal, double vex_variation,
                                   double vout_min, double gain)
{
    /* Power Supply Rejection Ratio requirement.
     *
     * Excitation voltage variations couple into the measurement
     * as apparent strain. The required PSRR depends on how much
     * excitation variation can be tolerated:
     *
     *   PSRR_required[dB] = 20*log10(Vexc_variation / Vout_min_allowed)
     *
     * where Vout_min_allowed = Vout_min_resolution * gain.
     *
     * If the excitation is regulated with PSRR > required, the
     * excitation-induced error is below the measurement resolution.
     */

    (void)vex_nominal;
    if (vout_min <= 0.0 || gain <= 0.0) return 200.0;

    double signal_min = vout_min * gain;
    double ratio = vex_variation / signal_min;

    return 20.0 * log10(ratio);
}

void excitation_source_init(excitation_source_t *src, excitation_mode_t mode)
{
    /* Initialize excitation source with typical specifications. */

    memset(src, 0, sizeof(*src));

    src->mode = mode;

    switch (mode) {
        case EXCITATION_VOLTAGE:
            src->nominal_voltage         = 5.0;
            src->nominal_current         = 0.0;
            src->voltage_accuracy_percent = 0.05;
            src->tempco_ppm_per_c        = 10.0;
            src->noise_uv_rms            = 2.0;
            src->output_impedance_ohm    = 0.01;
            src->max_compliance_v        = 5.5;
            break;

        case EXCITATION_CURRENT:
            src->nominal_voltage         = 0.0;
            src->nominal_current         = 0.005;  /* 5 mA */
            src->current_accuracy_percent = 0.1;
            src->tempco_ppm_per_c        = 25.0;
            src->noise_uv_rms            = 5.0;
            src->output_impedance_ohm    = 1.0e6;   /* High Z for current source */
            src->max_compliance_v        = 10.0;
            break;

        case EXCITATION_RATIOMETRIC:
            src->nominal_voltage         = 5.0;
            src->nominal_current         = 0.0;
            src->voltage_accuracy_percent = 0.1;
            src->tempco_ppm_per_c        = 50.0;
            src->noise_uv_rms            = 10.0;
            src->output_impedance_ohm    = 0.1;
            src->max_compliance_v        = 5.5;
            break;
    }

    src->line_regulation_ppm = 10.0;
    src->load_regulation_ppm = 5.0;
}

void voltage_reference_init(voltage_reference_t *ref, double vout)
{
    /* Initialize voltage reference with standard precision specs.
     *
     * Representative of high-quality references:
     *   - ADR45xx series (Analog Devices)
     *   - REF50xx series (Texas Instruments)
     *   - MAX63xx series (Maxim)
     */

    memset(ref, 0, sizeof(*ref));

    ref->output_voltage            = vout;
    ref->initial_accuracy_percent  = 0.02;     /* 0.02% = 200 ppm */
    ref->tempco_ppm_per_c          = 2.0;      /* 2 ppm/K — high grade */
    ref->noise_uv_pp               = 2.0;      /* 2 uVpp 0.1-10 Hz */
    ref->long_term_stability_ppm   = 50.0;     /* 50 ppm/1000hr */
    ref->line_regulation_ppm_per_v = 5.0;      /* 5 ppm/V */
    ref->load_regulation_ppm_per_ma = 10.0;    /* 10 ppm/mA */
    ref->dropout_voltage_mv        = 500.0;    /* 500 mV headroom */
    ref->quiescent_current_ua      = 800.0;    /* 800 uA quiescent */
}
