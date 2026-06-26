/**
 * @file ina_advanced.h
 * @brief Advanced Instrumentation Amplifier Topics
 *
 * Covers L8 (Advanced Topics) and L9 (Research Frontiers):
 *   - Chopper-stabilized amplifiers
 *   - Auto-zero techniques
 *   - Programmable gain amplifiers (PGA)
 *   - Fully differential architectures
 *   - MEMS sensor interfaces
 *   - Time-varying offset compensation
 *   - Stochastic calibration
 *
 * Reference:
 *   Enz & Temes, "Circuit Techniques for Reducing the Effects of Op-Amp
 *     Imperfections" (Proc. IEEE, 1996)
 *   Bakker, Thiele, Huijsing, "A CMOS Nested-Chopper Instrumentation
 *     Amplifier with 100-nV Offset" (JSSC, 2000)
 *   Pertijs & Huijsing, "Precision Temperature Sensors in CMOS Technology"
 *   Wu et al., "A Chopper Current-Feedback IA with 1m? 1/f Noise Corner"
 *
 * @course-alignment
 *   MIT 6.775 CMOS Analog IC Design: Chopper/auto-zero techniques
 *   Stanford EE315 VLSI Data Conversion: PGA architectures
 *   Berkeley EE240 Advanced Analog IC: Precision analog
 *   ETH Zurich 227-0166: Mixed-Signal IC Design
 */
#ifndef INA_ADVANCED_H
#define INA_ADVANCED_H

#include "ina_core.h"
#include "ina_topology.h"
#include "ina_calibration.h"
#include <stdint.h>

/*===========================================================================
 * L8: Chopper-Stabilized Instrumentation Amplifier
 *
 * Chopper stabilization modulates the input signal to a higher frequency
 * (chopper frequency f_chop), amplifies it where 1/f noise is low, then
 * demodulates back to baseband. The offset and 1/f noise are modulated
 * to f_chop and filtered out.
 *
 * Principle:
 *   1. Modulator: multiply input by ?1 at f_chop
 *   2. Amplifier: amplifies modulated signal (offset + signal)
 *   3. Demodulator: multiply by ?1 at f_chop
 *      - Signal: returns to baseband (demodulated)
 *      - Offset: modulated to f_chop
 *   4. Lowpass filter: removes f_chop components
 *
 * Residual offset: Vos_residual = Vos * (1/f_chop * tau)
 *   where tau is the settling time constant.
 *
 * Can achieve < 1 ?V offset and < 10 nV/?C drift.
 *
 * Reference: Enz & Temes (1996), Bakker et al. (2000)
 *===========================================================================*/

/**
 * @brief Chopper-stabilized IA parameters
 */
typedef struct {
    double chopper_frequency_hz;   /**< Chopping frequency (Hz) */
    double modulator_efficiency;   /**< Modulator efficiency (0-1) */
    double residual_offset_nv;     /**< Residual offset after chopping (nV) */
    double ripple_uv;              /**< Output ripple at f_chop (?V) */
    double noise_reduction_db;     /**< 1/f noise reduction at 1 Hz (dB) */
    double charge_injection_fc;    /**< Charge injection error (fC) */
    double settling_time_us;       /**< Settling time after chop edge (?s) */
    double bandwidth_hz;           /**< Usable signal bandwidth (<< f_chop) */
    int    use_nested_chopper;     /**< Nested chopper for ripple reduction */
    int    use_spread_spectrum;    /**< Spread-spectrum clocking */
} ChopperIaConfig;

/**
 * @brief Analyze chopper-stabilized IA performance
 *
 * Computes key metrics for a chopper IA design:
 *   1. Residual offset: Vos_res ? Vos * (2/pi) * (1/(1 + A0))
 *      where A0 is the open-loop gain at f_chop
 *   2. Ripple amplitude: Vripple ? Voffset * (?/2)
 *   3. Noise reduction: reduces 1/f noise by ~ (f_chop/fc)
 *
 * @param vos_dc Input offset without chopping (?V)
 * @param opamp_gbw Individual op-amp GBW (Hz)
 * @param f_chop Chopper frequency (Hz)
 * @param config Chopper configuration (will be populated)
 */
void ina_chopper_analyze(double vos_dc,
                          double opamp_gbw,
                          double f_chop,
                          ChopperIaConfig *config);

/**
 * @brief Compute residual ripple after chopper demodulation
 *
 * The ripple at f_chop and its harmonics must be filtered.
 * For a first-order LPF with cutoff fc:
 *   Ripple_attenuation = 20*log10(sqrt(1 + (f_chop/fc)^2))
 *
 * Nested chopper: uses a second chopper at f_chop2 < f_chop1
 * to modulate residual ripple to f_chop1 ? f_chop2.
 */
double ina_chopper_ripple_attenuation(double f_chop, double f_lpf_cutoff,
                                       int filter_order);

/**
 * @brief Design clock for chopper IA
 *
 * Chopper clock requirements:
 *   - f_chop >> f_signal (typically 100x to 1000x)
 *   - f_chop >> fc_1overf (to move 1/f above signal band)
 *   - f_chop < GBW / 10 (for adequate settling)
 *   - Duty cycle = 50% ? 0.1% (for offset cancellation)
 *
 * Non-overlapping clock phases prevent charge injection errors.
 */
typedef struct {
    double f_chop;              /**< Chopper frequency (Hz) */
    double f_chop_nested;       /**< Nested chopper frequency (Hz) */
    double non_overlap_time_ns; /**< Non-overlap interval (ns) */
    double duty_cycle;          /**< Duty cycle (0.5 = 50%) */
    double jitter_ps_rms;       /**< RMS clock jitter (ps) */
} ChopperClock;

void ina_chopper_clock_design(double signal_bw_hz,
                               double opamp_gbw_hz,
                               double flicker_corner_hz,
                               ChopperClock *clock);

/*===========================================================================
 * L8: Auto-Zero Technique (Sampled Offset Correction)
 *
 * Auto-zero (AZ) amplifiers sample the offset periodically and subtract
 * it from the signal. Unlike chopping, AZ works in the sampled domain.
 *
 * Two phases:
 *   Phase 1 (?1): Inputs shorted, offset stored on capacitor
 *   Phase 2 (?2): Signal amplified, stored offset subtracted
 *
 * AZ reduces offset to: Vos_res ? Vos / (A0 + 1)
 *
 * Noise folding: AZ samples wideband noise onto the holding capacitor,
 * increasing baseband noise by the ratio BW_opamp / f_AZ.
 * This is a key disadvantage vs chopping.
 *
 * Reference: Enz & Temes (1996)
 *===========================================================================*/

typedef struct {
    double auto_zero_frequency_hz; /**< Auto-zero sampling frequency */
    double hold_capacitor_pf;      /**< Offset storage capacitor (pF) */
    double sample_time_us;         /**< Sampling/acquisition time (?s) */
    double noise_fold_factor;      /**< Noise folding ratio */
    double residual_offset_uv;     /**< Residual offset after AZ (?V) */
    double droop_rate_uv_per_s;    /**< Capacitor leakage droop (?V/s) */
    int    correlated_double_sampling; /**< Use CDS for kT/C noise reduction */
} AutoZeroConfig_v2;

/**
 * @brief Analyze auto-zero amplifier noise folding
 *
 * The fundamental limitation of auto-zero: baseband noise increases
 * because wideband noise is aliased by the sampling process.
 *
 * Noise fold factor = BW_opamp / (2 * f_AZ)
 *
 * For f_AZ = 10 kHz and BW_opamp = 10 MHz:
 *   fold factor = 10e6 / (2 * 10e3) = 500
 *
 * This means the baseband noise density increases by sqrt(500) ? 22x.
 *
 * Mitigation: limit amplifier bandwidth before auto-zero sampling.
 */
double ina_autozero_noise_fold(double opamp_bandwidth_hz,
                                double auto_zero_freq_hz);

/**
 * @brief Compare chopper vs auto-zero for given specification
 *
 * Returns a recommendation based on:
 *   - Chopper: better for continuous-time, lower noise folding
 *   - Auto-zero: better for sampled systems, simpler implementation
 *
 * @return 0 = chopper preferred, 1 = auto-zero preferred
 */
int ina_compare_chopper_vs_autozero(double signal_bw_hz,
                                     double noise_requirement_nv_per_sqrt_hz,
                                     double supply_budget_ua,
                                     int is_sampled_system);

/*===========================================================================
 * L8: Programmable Gain Amplifier (PGA) Design
 *
 * PGAs allow digital selection of gain, enabling the same hardware
 * to handle different sensor types and signal ranges.
 *
 * Common PGA architectures:
 *   1. Switched Rg: MUX selects different Rg resistors
 *   2. Switched feedback network: MUX selects gain-setting resistors
 *   3. MDAC-based: Multiplying DAC in feedback path
 *
 * Key specifications:
 *   - Gain range and step size
 *   - Gain accuracy and drift over temperature
 *   - Settling time after gain change
 *   - Switch resistance and leakage effects
 *===========================================================================*/

/** PGA gain step definition */
typedef struct {
    double gain_setting;         /**< Gain value (V/V) */
    double rg_resistance;        /**< Corresponding Rg (ohm) */
    int    digital_code;         /**< Digital select code */
    double gain_error_percent;   /**< Calibrated gain error at this setting */
} PgaGainStep;

/**
 * @brief PGA configuration
 */
typedef struct {
    PgaGainStep *gain_steps;     /**< Array of available gain settings */
    int    num_steps;            /**< Number of gain settings */
    double r_feedback;           /**< Internal feedback resistor (ohm) */
    double switch_on_resistance; /**< Analog switch Ron (ohm) */
    double switch_leakage_na;    /**< Switch leakage current (nA) */
    double switch_charge_injection_pc; /**< Switch charge injection (pC) */
    int    current_step_index;   /**< Currently selected gain step */
} PgaConfig;

/**
 * @brief Design PGA gain table
 *
 * Generates gain steps based on desired range and resolution.
 *
 * For exponential stepping (common for wide dynamic range):
 *   G[i] = G_min * step_ratio^i
 *
 * For linear stepping:
 *   G[i] = G_min + i * (G_max - G_min) / (N - 1)
 *
 * @param g_min Minimum gain
 * @param g_max Maximum gain
 * @param num_steps Number of gain steps
 * @param logarithmic If 1, use exponential steps; if 0, linear
 * @param r_feedback Internal feedback resistor (ohm)
 * @param steps Output array (caller-allocated, num_steps elements)
 */
void ina_pga_design_gain_table(double g_min, double g_max,
                                 int num_steps, int logarithmic,
                                 double r_feedback,
                                 PgaGainStep *steps);

/**
 * @brief Compute Rg needed for a given gain step
 *
 * Rg = 2 * Rf / (G - 1)
 *
 * Considers analog switch Ron:
 *   Rg_effective = Rg + 2 * Ron
 *   G_actual = 1 + 2 * Rf / (Rg + 2 * Ron)
 */
double ina_pga_rg_with_switch(double desired_gain, double r_feedback,
                                double switch_ron);

/**
 * @brief Estimate gain error from switch Ron variation
 *
 * Analog switch Ron varies with temperature and supply:
 *   Ron(T) = Ron_25C * (1 + TC * (T - 25))
 *
 * This causes a gain error:
 *   Err_percent = (Ron(T) - Ron_nominal) / (Rg + Ron_nominal) * 100
 */
double ina_pga_switch_gain_error(double rg_nominal, double switch_ron_nominal,
                                   double switch_ron_tc_ppm, double temperature);

/**
 * @brief Optimize PGA step selection for minimum settling time
 *
 * Larger Rg ? slower settling (higher time constant with parasitic C)
 * Smaller Rg ? faster settling but higher gain
 *
 * Trade-off analysis for L8 advanced design optimization.
 */
double ina_pga_optimum_rg_for_bandwidth(double required_bandwidth_hz,
                                          double parasitic_capacitance_pf,
                                          double r_feedback);

/*===========================================================================
 * L8: Fully Differential Instrumentation Amplifier
 *
 * Fully differential IAs provide differential output, which doubles
 * the signal swing, improves CMRR, and rejects even-order harmonics.
 *
 * Key advantage for modern low-voltage systems:
 *   Differential signal swing = 2x single-ended swing
 *   For 3.3V supply: SE = ~2Vpp, Diff = ~4Vpp differential
 *
 * VOCM pin: sets output common-mode voltage, typically mid-supply
 * for maximum swing or Vref of the ADC for direct interface.
 *
 * Reference: Karki, "Fully Differential Amplifiers" (TI SLOA054D)
 *===========================================================================*/

typedef struct {
    double differential_gain;      /**< Differential gain Vout_diff/Vin_diff */
    double common_mode_gain;       /**< CM-to-CM gain (ideally 0) */
    double common_mode_to_diff_gain; /**< CM-to-differential gain (ideally 0) */
    double output_cm_voltage;      /**< Output common-mode voltage (VOCM) */
    double differential_output_swing; /**< Max differential output swing (Vpp) */
    double output_cmrr_db;         /**< Output CMRR */
    double input_cmrr_db;          /**< Input CMRR */
} FullyDifferentialIa;

/**
 * @brief Design fully differential IA configuration
 *
 * Analyzes the trade-offs:
 *   - Differential output doubles SNR for same supply
 *   - Eliminates need for single-ended conversion
 *   - Drives differential ADC directly
 *   - Requires VOCM generation circuit
 */
FullyDifferentialIa ina_fully_differential_design(double gain,
                                                    double supply_voltage,
                                                    double vocm_target);

/**
 * @brief Compute differential SNR improvement over single-ended
 *
 * For a given supply voltage:
 *   SE swing = Vsupply - 2*V_sat (e.g., 3.3V - 0.6V = 2.7Vpp)
 *   Diff swing = 2 * (Vsuply - 2*V_sat) (e.g., 5.4Vpp diff)
 *   SNR improvement = 20*log10(2) = 6 dB
 */
double ina_differential_snr_improvement(double supply_voltage,
                                          double saturation_margin);

/*===========================================================================
 * L9: Research Frontiers - MEMS Sensor Interfaces
 *
 * MEMS (Micro-Electro-Mechanical Systems) sensors present unique
 * challenges for instrumentation amplifier design:
 *   - Ultra-low signal levels (?V to mV)
 *   - High source impedance (capacitive MEMS: 1-10 pF)
 *   - Parasitic effects (feedthrough, mechanical resonance)
 *   - Need for synchronous demodulation
 *
 * Applications:
 *   - MEMS accelerometers and gyroscopes
 *   - MEMS pressure sensors
 *   - MEMS microphones
 *   - Resonant MEMS sensors (frequency output)
 *
 * Reference:
 *   Yazdi, Ayazi, Najafi, "Micromachined Inertial Sensors" (Proc. IEEE, 1998)
 *   Boser & Howe, "Surface Micromachined Accelerometers" (JSSC, 1996)
 *===========================================================================*/

typedef struct {
    double capacitance_nominal_pf;   /**< Nominal sense capacitance */
    double capacitance_change_fF;    /**< Capacitance change per full-scale */
    double parasitic_capacitance_pf; /**< Total parasitic capacitance */
    double carrier_frequency_hz;     /**< Carrier/modulation frequency */
    double carrier_amplitude_v;      /**< Carrier amplitude */
    double mechanical_resonance_hz;  /**< Mechanical resonant frequency */
    double quality_factor;           /**< Mechanical Q factor */
} MemsSensor;

/**
 * @brief Analyze MEMS capacitive sensor interface
 *
 * For capacitive MEMS sensors, the signal is typically a capacitance
 * change modulated onto a high-frequency carrier.
 *
 * Signal current: i_sig = V_carrier * dC/dt * (angular frequency)
 *
 * Required IA: Very high input impedance (> 1 G? at carrier freq)
 *              Low input capacitance (< 0.5 pF)
 */
double ina_mems_signal_current(const MemsSensor *sensor);

/**
 * @brief Design synchronous demodulator for MEMS interface
 *
 * Synchronous demodulation (lock-in amplifier principle):
 *   1. Amplify modulated signal (AC-coupled, high gain)
 *   2. Multiply by carrier reference
 *   3. Lowpass filter to extract baseband
 *
 * This rejects uncorrelated noise and parasitic feedthrough.
 *
 * L9 concept: This is the same principle used in
 * quantum sensing and ultra-sensitive measurements.
 */
typedef struct {
    double carrier_frequency;      /**< Demodulation frequency (Hz) */
    double phase_shift_deg;        /**< Phase compensation (degrees) */
    double lowpass_cutoff_hz;      /**< Output filter cutoff (Hz) */
    double demodulation_gain;      /**< Effective demodulator gain */
    double noise_rejection_db;     /**< Out-of-band noise rejection (dB) */
} SyncDemodulator;

void ina_design_sync_demodulator(double carrier_freq_hz,
                                  double signal_bw_hz,
                                  double noise_rejection_db,
                                  SyncDemodulator *demod);

/**
 * @brief Estimate SNR for MEMS capacitive accelerometer
 *
 * Covers L9: MEMS + CMOS integration noise limits.
 *
 * Total noise sources:
 *   1. Mechanical Brownian noise: Fn = sqrt(4*kB*T*D)
 *   2. Electronic noise: IA input noise + kT/C noise
 *   3. Quantization noise (if on-chip ADC)
 *
 * This function computes the fundamental SNR limit for a given
 * MEMS sensor + IA system.
 */
double ina_mems_accelerometer_snr(const MemsSensor *sensor,
                                   const InaParameters *ia_params,
                                   double bandwidth_hz,
                                   double temperature_k);

/*===========================================================================
 * L9: Stochastic Calibration and Bayesian Error Estimation
 *
 * Advanced calibration using probabilistic methods for
 * time-varying, uncertain systems.
 *
 * Reference:
 *   Box, Jenkins, Reinsel, "Time Series Analysis" (2015)
 *   Gelman et al., "Bayesian Data Analysis" (2013)
 *===========================================================================*/

typedef struct {
    double state_estimate;         /**< Current best estimate of parameter */
    double state_variance;         /**< Estimate variance */
    double process_noise;          /**< Expected drift variance per step */
    double measurement_noise;      /**< Measurement noise variance */
    double kalman_gain;            /**< Current Kalman gain */
} KalmanCalState;

/**
 * @brief Kalman filter for tracking IA offset drift
 *
 * Models offset as a random walk:
 *   x[k] = x[k-1] + w[k]  (state: offset, process noise w)
 *   z[k] = x[k] + v[k]    (measurement z, measurement noise v)
 *
 * Kalman update:
 *   predict: x?? = x?, P? = P + Q
 *   update:  K = P?/(P? + R), x? = x?? + K*(z - x??), P = (1-K)*P?
 *
 * This is an L9 technique: adaptive, self-calibrating systems.
 */
void ina_kalman_offset_update(KalmanCalState *state,
                               double measurement,
                               double measurement_variance);

/**
 * @brief Initialize Kalman filter for offset tracking
 */
void ina_kalman_offset_init(KalmanCalState *state,
                              double initial_offset,
                              double initial_uncertainty,
                              double process_noise);

/*===========================================================================
 * L8: Time-Varying and Adaptive Gain
 *===========================================================================*/

/**
 * @brief Adaptive gain control for IA signal chain
 *
 * Automatically adjusts PGA gain to keep signal within
 * optimal ADC range (autoranging).
 *
 * Algorithm:
 *   1. Monitor signal amplitude
 *   2. If signal > upper threshold: reduce gain
 *   3. If signal < lower threshold: increase gain
 *   4. Apply hysteresis to prevent oscillation
 */
typedef struct {
    double upper_threshold_percent;  /**< Upper threshold (% of FS) */
    double lower_threshold_percent;  /**< Lower threshold (% of FS) */
    double hysteresis_percent;       /**< Hysteresis band (%) */
    double min_dwell_time_ms;        /**< Minimum time between gain changes */
    int    current_gain_index;       /**< Current gain setting index */
    double current_signal_estimate;  /**< Smoothed signal estimate */
} AdaptiveGainControl;

void ina_adaptive_gain_update(AdaptiveGainControl *agc,
                               double signal_reading,
                               double full_scale,
                               const PgaGainStep *gain_table,
                               int num_gains);

double ina_adaptive_gain_get(const AdaptiveGainControl *agc,
                              const PgaGainStep *gain_table);

#endif /* INA_ADVANCED_H */
