/**
 * @file ina_advanced.c
 * @brief Advanced Instrumentation Amplifier Topics Implementation
 *
 * Implements L8 (Advanced Topics):
 *   - Chopper-stabilized amplifier analysis
 *   - Auto-zero noise folding analysis
 *   - Programmable Gain Amplifier (PGA) design
 *   - Fully differential IA configuration
 *   - Adaptive gain control
 *   - Kalman filter offset tracking
 *
 * And L9 (Research Frontiers):
 *   - MEMS capacitive sensor interface analysis
 *   - Synchronous demodulation (lock-in amplifier)
 *   - MEMS accelerometer SNR limits
 *
 * Reference:
 *   Enz & Temes, Proc. IEEE (1996)
 *   Bakker et al., JSSC (2000)
 *   Wu et al., JSSC (2017)
 *   Yazdi, Ayazi, Najafi, Proc. IEEE (1998)
 */
#include "ina_advanced.h"
#include <math.h>
#include <string.h>
#include <float.h>

/*===========================================================================
 * L8: Chopper-Stabilized IA Analysis
 *===========================================================================*/

void ina_chopper_analyze(double vos_dc,
                          double opamp_gbw,
                          double f_chop,
                          ChopperIaConfig *config)
{
    /**
     * Analyze chopper-stabilized IA performance.
     *
     * The chopper technique (Enz & Temes, 1996) modulates the input
     * signal to a higher frequency using a square-wave carrier,
     * amplifies it where 1/f noise and offset are minimal, then
     * demodulates back to baseband.
     *
     * Key metrics:
     *
     * 1. Residual offset (after chopping):
     *    Vos_res = Vos_dc * (2/?) * (1/(1 + A0(f_chop)))
     *
     *    where A0(f_chop) is the open-loop gain at the chop frequency.
     *    With A0(f_chop) ? GBW/f_chop:
     *    Vos_res ? Vos_dc * (2/?) * (f_chop/GBW)
     *
     *    Example: Vos_dc = 1 mV, f_chop = 10 kHz, GBW = 10 MHz:
     *      Vos_res ? 1e-3 * 0.637 * 1e-3 = 0.637 ?V
     *
     * 2. Output ripple at f_chop:
     *    V_ripple = Vos_dc * (?/2) * (1/(1 + A0(f_chop))) * 2/?
     *    ? Vos_dc (simplified)
     *
     *    The ripple must be filtered by a subsequent LPF.
     *
     * 3. Noise improvement:
     *    The 1/f noise is shifted to f_chop and its harmonics,
     *    leaving only white noise in the baseband.
     *
     *    Thermal noise floor increases slightly due to
     *    aliasing of wideband noise (similar to auto-zero).
     *
     * 4. Charge injection:
     *    The chopper switches inject charge (Q_inj = C_gs * V_clk)
     *    at each transition, causing small voltage steps.
     *    Residual charge = Q_inj * f_chop / gm_input.
     */
    if (!config) return;

    memset(config, 0, sizeof(ChopperIaConfig));
    config->chopper_frequency_hz = f_chop;

    /* Residual offset: Vos_res ? Vos * (2/?) * (f_chop/GBW) */
    if (opamp_gbw > 0.0 && f_chop > 0.0) {
        double a0_at_fchop = opamp_gbw / f_chop;
        config->residual_offset_nv = fabs(vos_dc) * (2.0 / M_PI)
                                     / (1.0 + a0_at_fchop) * 1000.0;
        /* ^ vos_dc in ?V, result in nV */
    }

    /* Ripple estimate */
    config->ripple_uv = fabs(vos_dc) * 0.5;

    /* Noise reduction: 1/f noise at 1 Hz reduced by (f_chop/fc) */
    /* Assuming fc = 1 kHz typical 1/f corner */
    double flicker_corner = 1000.0;
    if (f_chop > flicker_corner) {
        config->noise_reduction_db = 20.0 * log10(f_chop / flicker_corner);
    }

    /* Bandwidth: signal must be << f_chop; typically BW = f_chop / 10 */
    config->bandwidth_hz = f_chop / 10.0;

    /* Settling time: ~5 time constants after chop edge */
    config->settling_time_us = 5.0 / f_chop * 1e6;
}

double ina_chopper_ripple_attenuation(double f_chop, double f_lpf_cutoff,
                                       int filter_order)
{
    /**
     * Compute attenuation of chopper ripple by output LPF.
     *
     * The ripple at f_chop must be attenuated below the
     * noise floor or resolution target.
     *
     * For an n-th order LPF:
     *   Attenuation = 20*n*log10(f_chop / f_lpf)
     *
     * Example: f_chop = 100 kHz, f_lpf = 1 kHz, order = 2:
     *   Atten = 20*2*log10(100000/1000) = 40*2 = 80 dB
     *
     * Ripple specification: typically < 1 LSB of ADC
     */
    if (f_lpf_cutoff <= 0.0 || f_chop <= f_lpf_cutoff) return 0.0;

    double atten_per_order = 20.0 * log10(f_chop / f_lpf_cutoff);
    return atten_per_order * filter_order;
}

void ina_chopper_clock_design(double signal_bw_hz,
                               double opamp_gbw_hz,
                               double flicker_corner_hz,
                               ChopperClock *clock)
{
    /**
     * Design chopper clock parameters.
     *
     * Clock requirements:
     *
     * 1. f_chop >> signal BW:
     *    Typically f_chop = 100 to 1000 * signal_bw
     *    At minimum, f_chop > 2 * signal_bw (Nyquist)
     *
     * 2. f_chop >> flicker corner:
     *    So that the modulated 1/f noise spectrum doesn't
     *    overlap with the baseband signal.
     *    f_chop > 10 * fc_1/f
     *
     * 3. f_chop < GBW / 10:
     *    So the op-amp has adequate open-loop gain at f_chop
     *    to accurately perform the modulation
     *
     * 4. Non-overlap time:
     *    Prevents shoot-through in the chopper switches.
     *    t_non_overlap ? 5-20 ns for CMOS switches.
     *
     * 5. Jitter:
     *    Clock jitter causes imperfect demodulation,
     *    adding noise. jitter_rms < 1/(100 * f_chop)
     */
    if (!clock) return;

    memset(clock, 0, sizeof(ChopperClock));

    /* Choose f_chop: at least 100x signal BW, > 10x flicker corner */
    double f_chop_min = signal_bw_hz * 100.0;
    if (f_chop_min < flicker_corner_hz * 10.0) {
        f_chop_min = flicker_corner_hz * 10.0;
    }

    double f_chop_max = opamp_gbw_hz / 10.0;

    clock->f_chop = f_chop_min;
    if (clock->f_chop > f_chop_max && f_chop_max > 0.0) {
        clock->f_chop = f_chop_max;
    }
    if (clock->f_chop < 1000.0) clock->f_chop = 1000.0;

    /* Nested chopper at ~1/10 of main chop freq */
    clock->f_chop_nested = clock->f_chop / 10.0;

    /* Non-overlap: 10 ns typical */
    clock->non_overlap_time_ns = 10.0;

    /* Duty cycle: 50% ideal */
    clock->duty_cycle = 0.5;

    /* Jitter requirement: < 0.1% of chop period */
    if (clock->f_chop > 0.0) {
        double t_chop = 1.0 / clock->f_chop;
        clock->jitter_ps_rms = t_chop * 0.001 * 1e12;  /* ps */
    }
}

/*===========================================================================
 * L8: Auto-Zero Noise Folding
 *===========================================================================*/

double ina_autozero_noise_fold(double opamp_bandwidth_hz,
                                double auto_zero_freq_hz)
{
    /**
     * Compute noise folding factor in auto-zero amplifiers.
     *
     * This is the FUNDAMENTAL LIMITATION of auto-zero techniques:
     * the wideband noise of the amplifier is undersampled by the
     * auto-zero clock and folds back into the baseband.
     *
     * Noise fold factor = BW_opamp / (2 * f_AZ)
     *
     * Baseband noise density after AZ:
     *   en_AZ = en * sqrt(1 + fold_factor)
     *         ? en * sqrt(fold_factor)   for fold_factor >> 1
     *
     * Example: BW = 10 MHz, f_AZ = 10 kHz
     *   fold = 10e6 / (2 * 10e3) = 500
     *   en_AZ / en = sqrt(500) ? 22.4x increase in noise!
     *
     * Mitigation strategies:
     *   1. Limit amplifier BW before AZ sampling (add pre-filter)
     *   2. Use correlated double sampling (CDS) to cancel kT/C noise
     *   3. Prefer chopper over AZ for low-noise applications
     *
     * Reference: Enz & Temes (1996), ?VI-B
     */
    if (auto_zero_freq_hz <= 0.0) return INFINITY;
    return opamp_bandwidth_hz / (2.0 * auto_zero_freq_hz);
}

int ina_compare_chopper_vs_autozero(double signal_bw_hz,
                                     double noise_requirement_nv_per_sqrt_hz,
                                     double supply_budget_ua,
                                     int is_sampled_system)
{
    /**
     * Compare chopper vs auto-zero techniques for given spec.
     *
     * Decision matrix (L8 engineering design):
     *
     *                    Chopper       Auto-Zero
     *   Noise            ?????         ?? (noise folding)
     *   Offset           ?????         ?????
     *   Drift            ?????         ???? (droop)
     *   Bandwidth        ????          ??? (settling time)
     *   Power            ???           ???? (simpler)
     *   Complexity       ???           ????
     *   Sampled system   ???           ????? (natural fit)
     *   Continuous-time  ?????          ??
     *
     * Recommendation:
     *   - Low noise, continuous-time ? Chopper
     *   - Sampled system, moderate noise ? Auto-Zero
     *   - Ultra-low power ? Auto-Zero (no oscillator needed if
     *     synchronized with ADC sampling)
     *
     * Return: 0 = Chopper preferred, 1 = Auto-Zero preferred
     */
    /* Score calculation */
    double chopper_score = 50.0;
    double az_score = 50.0;

    /* Low noise requirement strongly favors chopper */
    if (noise_requirement_nv_per_sqrt_hz < 10.0) {
        chopper_score += 30.0;
    } else if (noise_requirement_nv_per_sqrt_hz < 50.0) {
        chopper_score += 15.0;
    }

    /* Sampled system favors auto-zero */
    if (is_sampled_system) {
        az_score += 20.0;
    } else {
        chopper_score += 20.0;
    }

    /* Low power favors auto-zero */
    if (supply_budget_ua < 100.0) {
        az_score += 15.0;
    }

    /* High bandwidth favors chopper */
    if (signal_bw_hz > 10000.0) {
        chopper_score += 15.0;
    }

    return (chopper_score >= az_score) ? 0 : 1;
}

/*===========================================================================
 * L8: Programmable Gain Amplifier (PGA)
 *===========================================================================*/

void ina_pga_design_gain_table(double g_min, double g_max,
                                 int num_steps, int logarithmic,
                                 double r_feedback,
                                 PgaGainStep *steps)
{
    /**
     * Design PGA gain step table.
     *
     * Generates the resistor values needed for each gain setting.
     *
     * For logarithmic (exponential) stepping:
     *   step_ratio = (G_max / G_min)^(1/(N-1))
     *   G[i] = G_min * step_ratio^i
     *
     * For linear stepping:
     *   G[i] = G_min + i * (G_max - G_min) / (N-1)
     *
     * The corresponding Rg is:
     *   Rg = 2 * R_feedback / (G - 1)
     *
     * Practical PGA examples:
     *   ADS1015 PGA: G = 1, 2, 4, 8, 16 (binary steps)
     *   AD8250: G = 1, 2, 5, 10
     *   AD8251: G = 1, 2, 4, 8
     *   LTC6915: 14 steps from G=1 to G=4096
     *
     * Implementation note: each step requires a precision resistor
     * and an analog switch to select it. Switch Ron adds to Rg.
     */
    if (!steps || num_steps <= 0) return;

    if (logarithmic && num_steps > 1) {
        double step_ratio = pow(g_max / g_min, 1.0 / (num_steps - 1));
        for (int i = 0; i < num_steps; i++) {
            steps[i].gain_setting = g_min * pow(step_ratio, i);
            steps[i].digital_code = i;
            if (steps[i].gain_setting > 1.0) {
                steps[i].rg_resistance = 2.0 * r_feedback
                                         / (steps[i].gain_setting - 1.0);
            } else {
                steps[i].rg_resistance = INFINITY;
            }
            steps[i].gain_error_percent = 0.0;  /* Calibrated to 0 initially */
        }
    } else {
        for (int i = 0; i < num_steps; i++) {
            if (num_steps > 1) {
                steps[i].gain_setting = g_min
                    + i * (g_max - g_min) / (num_steps - 1);
            } else {
                steps[i].gain_setting = g_min;
            }
            steps[i].digital_code = i;
            if (steps[i].gain_setting > 1.0) {
                steps[i].rg_resistance = 2.0 * r_feedback
                                         / (steps[i].gain_setting - 1.0);
            } else {
                steps[i].rg_resistance = INFINITY;
            }
            steps[i].gain_error_percent = 0.0;
        }
    }
}

double ina_pga_rg_with_switch(double desired_gain, double r_feedback,
                                double switch_ron)
{
    /**
     * Compute effective Rg considering analog switch resistance.
     *
     * The analog switch Ron appears in series with Rg.
     * For a 3-op-amp IA, two switches in the Rg path add 2*Ron.
     *
     * Rg_effective = Rg_nominal + 2 * Ron
     *
     * G_actual = 1 + 2*Rf / (Rg_nominal + 2*Ron)
     *
     * Solving for Rg_nominal to achieve desired_gain:
     *   Rg_nominal = 2*Rf/(G_desired - 1) - 2*Ron
     *
     * Example: AD620 Rf=24.7k, STG3692 switch Ron=5?, G=1000:
     *   Rg_ideal = 2*24.7k/999 = 49.45 ?
     *   Rg_actual = 49.45 + 10 = 59.45 ? (with 2*Ron)
     *   G_actual = 1 + 49.4k/59.45 = 832 (16% error!)
     *
     * This shows why low Ron switches are critical for high gains
     * in PGA designs. For G < 100, the error is manageable.
     */
    if (desired_gain <= 1.0) return INFINITY;

    double rg_ideal = 2.0 * r_feedback / (desired_gain - 1.0);
    return rg_ideal - 2.0 * switch_ron;
}

double ina_pga_switch_gain_error(double rg_nominal, double switch_ron_nominal,
                                   double switch_ron_tc_ppm, double temperature)
{
    /**
     * Compute gain error from switch Ron temperature variation.
     *
     * Switch Ron changes with temperature:
     *   Ron(T) = Ron_25C * (1 + TC * 1e-6 * (T - 25))
     *
     * The gain error is:
     *   ?G/G = -?Ron / (Rg + Ron)
     *
     * For a typical CMOS switch: TC_Ron ? 3000-6000 ppm/?C
     * Over 0-70?C range: Ron can double!
     *
     * Mitigation:
     *   1. Use low TC switches (or compensate with opposite TC in Rg)
     *   2. Make Ron << Rg (but this limits max gain)
     *   3. Use force-sense (Kelvin) connections to remove Ron
     *   4. Place switch in op-amp feedback loop (bootstrapped)
     */
    double dt = temperature - 25.0;
    double ron = switch_ron_nominal * (1.0 + switch_ron_tc_ppm * 1e-6 * dt);
    double rg_total = rg_nominal + 2.0 * ron;
    double rg_total_nominal = rg_nominal + 2.0 * switch_ron_nominal;

    if (rg_total_nominal <= 0.0) return 0.0;
    /* Gain ? 1/Rg_total, so ?G/G = -?Rg_total/Rg_total */
    return -(rg_total - rg_total_nominal) / rg_total_nominal * 100.0;
}

double ina_pga_optimum_rg_for_bandwidth(double required_bandwidth_hz,
                                          double parasitic_capacitance_pf,
                                          double r_feedback)
{
    /**
     * Find optimum Rg for maximum bandwidth in PGA.
     *
     * Trade-off:
     *   - Smaller Rg ? higher gain ? lower bandwidth (GBW/G)
     *   - Larger Rg ? lower gain ? higher bandwidth
     *
     * But Rg also interacts with parasitic capacitance at the
     * Rg pins, creating a pole:
     *   f_pole = 1 / (2? * Rg * C_parasitic)
     *
     * For a given target bandwidth BW and parasitic C:
     *   Rg_max = 1 / (2? * BW * C_parasitic)
     *   G_min = 1 + 2*Rf / Rg_max
     *
     * If G_min > 1, bandwidth is limited by GBW.
     * If G_min ? 1, bandwidth is limited by parasitic pole.
     *
     * Returns optimum Rg value.
     */
    if (required_bandwidth_hz <= 0.0) return r_feedback;
    if (parasitic_capacitance_pf <= 0.0) parasitic_capacitance_pf = 5.0;

    double c = parasitic_capacitance_pf * 1e-12;
    double rg_parasitic_limit = 1.0 / (2.0 * M_PI * required_bandwidth_hz * c);

    /* Don't let Rg exceed the parasitic limit */
    if (rg_parasitic_limit < 100.0) return 100.0;
    return rg_parasitic_limit;
}

/*===========================================================================
 * L8: Fully Differential IA
 *===========================================================================*/

FullyDifferentialIa ina_fully_differential_design(double gain,
                                                    double supply_voltage,
                                                    double vocm_target)
{
    /**
     * Design fully differential IA configuration.
     *
     * Fully differential amplifiers provide:
     *   - Two outputs: Vout+ and Vout- (180? out of phase)
     *   - Differential output: Vout_diff = Vout+ - Vout-
     *   - Common-mode output set by VOCM pin
     *
     * Key advantages:
     *   1. 2x voltage swing for same supply (6 dB SNR improvement)
     *   2. Even-order harmonic cancellation
     *   3. Better PSRR (supply noise is common-mode)
     *   4. Direct drive of differential ADC inputs
     *
     * VOCM (Output Common-Mode) voltage:
     *   Typically set to mid-supply for maximum swing:
     *   VOCM = Vsupply / 2
     *   Or to match ADC reference: VOCM = Vref/2
     *
     * Example: THS4521, AD8138, LTC6362
     *
     * Reference: Karki, TI SLOA054D
     */
    FullyDifferentialIa diff;
    memset(&diff, 0, sizeof(diff));

    diff.differential_gain = gain;
    diff.common_mode_gain = 0.0;       /* Ideally 0 */
    diff.common_mode_to_diff_gain = 0.0; /* Ideally 0 */
    diff.output_cm_voltage = vocm_target;

    /* Differential output swing */
    double v_sat = 0.3;  /* Saturation margin per output */
    double max_single = supply_voltage - 2.0 * v_sat;
    diff.differential_output_swing = 2.0 * max_single;

    /* CMRR estimates */
    diff.output_cmrr_db = 80.0;  /* Typical */
    diff.input_cmrr_db = 90.0;

    return diff;
}

double ina_differential_snr_improvement(double supply_voltage,
                                          double saturation_margin)
{
    /**
     * Compute SNR improvement of differential vs single-ended.
     *
     * Single-ended max swing: Vse_pp = Vsupply - 2*Vsat
     * Differential max swing: Vdiff_pp = 2 * (Vsupply - 2*Vsat)
     *
     * SNR improvement: 20*log10(Vdiff_pp / Vse_pp)
     *                 = 20*log10(2) = 6.02 dB
     *
     * This 6 dB improvement comes "for free" with differential
     * signaling, which is why it's preferred in modern low-voltage
     * (3.3V, 1.8V) systems.
     *
     * Additional benefit: Differential signaling also reduces
     * EMI radiation (the currents in the two lines cancel) and
     * provides common-mode noise rejection.
     */
    double v_se_pp = supply_voltage - 2.0 * saturation_margin;
    double v_diff_pp = 2.0 * v_se_pp;

    if (v_se_pp <= 0.0) return 0.0;
    return 20.0 * log10(v_diff_pp / v_se_pp);
}

/*===========================================================================
 * L9: MEMS Capacitive Sensor Interface
 *===========================================================================*/

double ina_mems_signal_current(const MemsSensor *sensor)
{
    /**
     * Compute signal current from MEMS capacitive sensor.
     *
     * MEMS capacitive sensors (accelerometers, gyroscopes, pressure
     * sensors) produce a capacitance change ?C proportional to the
     * physical quantity being measured.
     *
     * With a carrier voltage excitation:
     *   i_sig = V_carrier * dC/dt
     *         = V_carrier * ? * ?C  (sinusoidal modulation)
     *
     * where ? = 2? * f_carrier.
     *
     * The current is typically:
     *   - Accelerometer: 10-100 fA (femtoamps!) per g
     *   - Gyroscope: 1-10 pA per ?/s
     *   - Pressure: 1-100 pA per kPa
     *
     * These extremely small currents require:
     *   1. Ultra-low input bias current IA (< 1 pA)
     *   2. High input impedance (> 1 T?)
     *   3. Careful guarding and shielding
     *   4. Synchronous demodulation
     *
     * Reference: Boser & Howe (1996), Yazdi et al. (1998)
     */
    if (!sensor) return 0.0;

    double omega = 2.0 * M_PI * sensor->carrier_frequency_hz;
    double delta_c = sensor->capacitance_change_fF * 1e-15;  /* fF ? F */

    /* i = V * ? * ?C */
    return sensor->carrier_amplitude_v * omega * delta_c;
}

void ina_design_sync_demodulator(double carrier_freq_hz,
                                  double signal_bw_hz,
                                  double noise_rejection_db,
                                  SyncDemodulator *demod)
{
    /**
     * Design synchronous demodulator (lock-in amplifier).
     *
     * Synchronous demodulation is the key technique for recovering
     * signals from MEMS capacitive sensors. It multiplies the
     * amplified sensor signal by the carrier reference, shifting
     * the baseband signal to DC (or near DC) while shifting noise
     * and interference away from the signal band.
     *
     * Principle (L9):
     *   Input: V_sig(t) = A * sin(?c*t) * sin(?s*t + ?)
     *   Reference: V_ref(t) = sin(?c*t + ?)
     *   Product: V_mix(t) = A/2 * [cos((?s-?c)t + ?-?)
     *                            - cos((?s+?c)t + ?+?)]
     *   After LPF: V_out(t) = A/2 * cos(?-?)  (DC signal)
     *
     * The LPF bandwidth sets the trade-off:
     *   - Narrower BW ? better noise rejection, slower response
     *   - Wider BW ? faster response, less noise rejection
     *
     * Noise rejection:
     *   Rejection = 10*log10(BW_carrier / BW_signal)
     *   (ratio of pre-demod to post-demod bandwidth)
     *
     * Phase compensation:
     *   The phase shift through the IA and parasitic capacitances
     *   must be compensated to maximize demodulated output.
     *
     * This is the same principle used in:
     *   - Lock-in amplifiers (Stanford Research SR830)
     *   - LVDT signal conditioning
     *   - MEMS Coriolis vibratory gyroscopes
     *   - Quantum measurement (Josephson junctions, SQUIDs)
     */
    if (!demod) return;

    memset(demod, 0, sizeof(SyncDemodulator));

    demod->carrier_frequency = carrier_freq_hz;
    demod->lowpass_cutoff_hz = signal_bw_hz;

    /* Phase shift estimate from IA bandwidth */
    /* Phase = -atan(f_carrier / BW_ia), compensated in reference */
    demod->phase_shift_deg = 0.0;  /* Set manually based on measurement */

    /* Demodulation gain = 0.5 (half amplitude from mixing) */
    demod->demodulation_gain = 0.5;

    /* Noise rejection */
    demod->noise_rejection_db = noise_rejection_db;
}

double ina_mems_accelerometer_snr(const MemsSensor *sensor,
                                   const InaParameters *ia_params,
                                   double bandwidth_hz,
                                   double temperature_k)
{
    /**
     * Estimate SNR for a MEMS capacitive accelerometer
     * with instrumentation amplifier readout.
     *
     * This L9 analysis combines:
     *   1. Mechanical Brownian noise (fundamental limit)
     *   2. Electronic noise from IA
     *   3. Quantization limits
     *
     * 1. Brownian noise (mechanical-thermal):
     *    F_n = sqrt(4 * kB * T * D)  [N/?Hz]
     *    a_n = F_n / m  [m/s^2/?Hz]
     *
     *    where D = damping coefficient = m*?_res/Q
     *
     *    a_n_rms = sqrt(4*kB*T*?_res / (m*Q)) * sqrt(BW)
     *
     * 2. Electronic noise RTI:
     *    Vn_total_rti = en_ia * sqrt(BW_total)
     *
     * 3. Capacitance-to-voltage sensitivity:
     *    S = V_carrier / (d0 * Cf) * epsilon
     *    (depends on specific MEMS geometry)
     *
     * 4. Total SNR:
     *    SNR = a_full_scale / sqrt(a_n_brownian^2 + a_n_electronic^2)
     *
     * Reference: Gabrielson, "Mechanical-Thermal Noise in
     *   Micromachined Acoustic and Vibration Sensors"
     *   (IEEE Trans. Electron Devices, 1993)
     */
    if (!sensor || !ia_params) return 0.0;

    const double k_boltzmann = 1.380649e-23;
    const double mass_estimate = 1e-9;  /* 1 ?g typical MEMS proof mass */

    /* 1. Brownian noise acceleration */
    double omega_res = 2.0 * M_PI * sensor->mechanical_resonance_hz;
    double q = sensor->quality_factor;
    if (q <= 0.0) q = 100.0;

    double a_brownian_density = sqrt(
        4.0 * k_boltzmann * temperature_k * omega_res
        / (mass_estimate * q)
    );
    double a_brownian_rms = a_brownian_density * sqrt(bandwidth_hz);

    /* 2. Electronic noise equivalent acceleration */
    /* Simplified: assuming 1 fF = 1 g sensitivity */
    double sensitivity_ff_per_g = sensor->capacitance_change_fF;
    if (sensitivity_ff_per_g <= 0.0) sensitivity_ff_per_g = 10.0;  /* 10 fF/g */

    double vn_electronic = ia_params->en_nv_per_sqrt_hz * 1e-9
                           * sqrt(bandwidth_hz);
    /* Convert voltage noise to equivalent acceleration */
    /* V_noise = i_noise * Z_feedback */
    /* i_noise = j? * V_carrier * C_noise */
    /* a_noise = C_noise / sensitivity_ff_per_g */
    double c_noise = vn_electronic
                     / (2.0 * M_PI * sensor->carrier_frequency_hz
                        * sensor->carrier_amplitude_v);
    double a_electronic_rms = c_noise / (sensitivity_ff_per_g * 1e-15);

    /* Full-scale acceleration */
    double a_fs = 2.0 * 9.81;  /* ?2g typical */

    /* Total noise and SNR */
    double a_noise_total = sqrt(a_brownian_rms * a_brownian_rms
                                + a_electronic_rms * a_electronic_rms);

    if (a_noise_total <= 0.0) return INFINITY;

    double snr = a_fs / a_noise_total;
    return 20.0 * log10(snr);
}

/*===========================================================================
 * L8: Kalman Filter for Offset Tracking
 *===========================================================================*/

void ina_kalman_offset_init(KalmanCalState *state,
                              double initial_offset,
                              double initial_uncertainty,
                              double process_noise)
{
    /**
     * Initialize Kalman filter for tracking IA offset drift.
     *
     * The Kalman filter provides optimal state estimation for
     * linear systems with Gaussian noise. Applied to offset
     * tracking, it smoothly estimates the true offset while
     * filtering measurement noise.
     *
     * State model:
     *   x[k] = x[k-1] + w[k]    (random walk offset)
     *
     * Measurement model:
     *   z[k] = x[k] + v[k]      (noisy measurement)
     *
     * where:
     *   w ~ N(0, Q)  process noise (offset drift variance)
     *   v ~ N(0, R)  measurement noise
     *
     * The Kalman filter optimally weighs the prediction
     * (previous estimate + expected drift) against the
     * measurement (current reading).
     *
     * Reference: Kalman, "A New Approach to Linear Filtering
     *   and Prediction Problems" (1960)
     *   Gelb, "Applied Optimal Estimation" (1974)
     */
    if (!state) return;

    memset(state, 0, sizeof(KalmanCalState));
    state->state_estimate = initial_offset;
    state->state_variance = initial_uncertainty * initial_uncertainty;
    state->process_noise = process_noise * process_noise;
    state->kalman_gain = 0.0;
}

void ina_kalman_offset_update(KalmanCalState *state,
                               double measurement,
                               double measurement_variance)
{
    /**
     * Kalman filter update step.
     *
     * Predict:
     *   x??[k] = x?[k-1]
     *   P?[k] = P[k-1] + Q
     *
     * Update:
     *   K[k] = P?[k] / (P?[k] + R)
     *   x?[k] = x??[k] + K[k] * (z[k] - x??[k])
     *   P[k] = (1 - K[k]) * P?[k]
     *
     * Interpretation:
     *   - When P? >> R (prediction uncertain, measurement precise):
     *     K ? 1, trust the measurement
     *   - When P? << R (prediction certain, measurement noisy):
     *     K ? 0, trust the prediction
     *
     * This adaptive weighting is what makes the Kalman filter
     * superior to simple averaging for tracking slowly varying
     * parameters like IA offset drift.
     */
    if (!state) return;

    /* Predict step */
    double x_pred = state->state_estimate;
    double p_pred = state->state_variance + state->process_noise;

    /* Update step */
    double r = measurement_variance;
    double k = p_pred / (p_pred + r);  /* Kalman gain */

    state->kalman_gain = k;
    state->state_estimate = x_pred + k * (measurement - x_pred);
    state->state_variance = (1.0 - k) * p_pred;
    state->measurement_noise = measurement_variance;
}

/*===========================================================================
 * L8: Adaptive Gain Control
 *===========================================================================*/

void ina_adaptive_gain_update(AdaptiveGainControl *agc,
                               double signal_reading,
                               double full_scale,
                               const PgaGainStep *gain_table,
                               int num_gains)
{
    /**
     * Adaptive gain control (autoranging) for IA signal chain.
     *
     * Automatically adjusts PGA gain to keep the signal within
     * the optimal range of the ADC, maximizing SNR without clipping.
     *
     * Algorithm (L5):
     *   1. Compute signal amplitude relative to full scale
     *   2. If signal > upper_threshold * FS: reduce gain
     *   3. If signal < lower_threshold * FS: increase gain
     *   4. Apply hysteresis to prevent oscillation
     *   5. Enforce minimum dwell time between gain changes
     *
     * This is a canonical engineering control problem:
     *   - Too frequent gain switching ? distortion at transitions
     *   - Too infrequent switching ? clipping or poor resolution
     *
     * The hysteresis band prevents "gain hunting" when the
     * signal is near a threshold.
     *
     * Reference: Analog Devices AN-1024, "Autoranging ADC Inputs"
     */
    if (!agc || !gain_table || num_gains <= 0) return;

    /* Smooth the signal estimate (first-order IIR) */
    double alpha = 0.1;  /* Smoothing factor */
    agc->current_signal_estimate = alpha * fabs(signal_reading)
                                   + (1.0 - alpha)
                                   * agc->current_signal_estimate;

    double relative_amplitude = agc->current_signal_estimate / full_scale;
    int new_gain_index = agc->current_gain_index;

    /* Upper threshold: signal too large ? reduce gain */
    if (relative_amplitude > agc->upper_threshold_percent / 100.0) {
        if (agc->current_gain_index > 0) {
            new_gain_index = agc->current_gain_index - 1;
        }
    }

    /* Lower threshold: signal too small ? increase gain */
    if (relative_amplitude < agc->lower_threshold_percent / 100.0) {
        if (agc->current_gain_index < num_gains - 1) {
            /* Apply hysteresis: only increase if well below threshold */
            double hysteresis_lower =
                (agc->lower_threshold_percent - agc->hysteresis_percent)
                / 100.0;
            if (relative_amplitude < hysteresis_lower) {
                new_gain_index = agc->current_gain_index + 1;
            }
        }
    }

    agc->current_gain_index = new_gain_index;
}

double ina_adaptive_gain_get(const AdaptiveGainControl *agc,
                              const PgaGainStep *gain_table)
{
    /**
     * Get current gain value from adaptive gain control.
     */
    if (!agc || !gain_table) return 1.0;
    if (agc->current_gain_index < 0) return 1.0;
    return gain_table[agc->current_gain_index].gain_setting;
}
