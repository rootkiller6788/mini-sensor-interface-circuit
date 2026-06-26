# Knowledge Graph — mini-photodiode-transimpedance-tia-design

## L1: Definitions
- Photodiode responsivity R (A/W) → `photodiode_model_t.responsivity_a_per_w`
- Quantum efficiency η → `photodiode_model_t.quantum_efficiency`
- Dark current I_dark → `photodiode_model_t.dark_current_na`
- Junction capacitance C_j → `photodiode_model_t.junction_capacitance_pf`
- Shunt resistance R_sh → `photodiode_model_t.shunt_resistance_ohm`
- Noise Equivalent Power NEP → `photodiode_model_t.noise_equivalent_power`
- Specific Detectivity D* → `photodiode_model_t.specific_detectivity`
- Transimpedance gain Z_T = V_out/I_photo → `tia_design_t.transimpedance_gain_ohm`
- 3-dB bandwidth f_3dB → `tia_design_t.bandwidth_3db_hz`
- Phase margin PM → `tia_design_t.phase_margin_deg`
- Gain-bandwidth product GBWP → `opamp_params_t.gain_bandwidth_mhz`
- Input-referred noise current density → `tia_design_t.input_noise_density_pa`
- Receiver sensitivity (dBm) → `tia_design_t.sensitivity_dbm`
- Johnson noise spectral density → `noise_johnson_voltage()`
- Shot noise spectral density → `noise_shot()`

## L2: Core Concepts
- Current-to-voltage conversion (R_f feedback) → `tia_design_basic()`
- Virtual ground at summing junction → input resistance derivation
- Photovoltaic vs photoconductive bias → `photodiode_bias_t`
- Gain-bandwidth tradeoff → `tia_bandwidth_3db()`
- Noise gain peaking → `tia_noise_gain()`
- Feedback pole-zero compensation → `tia_compensation_capacitance()`
- Stability compensation → `tia_design_compensation()`

## L3: Mathematical Structures
- s-domain transfer function → `tia_compute_frequency_response()`
- Bode magnitude/phase analysis → `tia_compute_bode()`
- Complex impedance networks → feedback factor beta(s)
- Root locus analysis → `tia_root_locus()`
- Nyquist stability → `tia_nyquist_analysis()`
- Pole-zero mapping → `tia_pole_zero_map()`
- PN junction electrostatics → `pn_junction_compute()`

## L4: Fundamental Laws
- Johnson-Nyquist theorem: v_n²=4kTR → `noise_johnson_voltage()`
- Schottky shot noise: i_n²=2qI_DC → `noise_shot()`
- kT/C noise: v_n²=kT/C → `noise_ktc_rms()`
- Gain-bandwidth product constancy → TIA bandwidth equation
- Bode stability criterion: PM > 0 for stability → `tia_loop_gain_analyze()`
- Routh-Hurwitz criterion → `tia_routh_hurwitz_stable()`
- Photoelectric effect: E=hc/λ → `photon_energy_ev()`

## L5: Algorithms/Methods
- TIA design methodology (Graeme method) → `tia_design_basic()`
- Compensation capacitor design → `tia_compensation_capacitance()`
- Noise integration over bandwidth → `tia_noise_analyze()`
- Phase margin calculation → `tia_phase_margin()`
- Noise optimization sweep → `tia_noise_optimize_rf()`
- Optimal R_f selection → `tia_optimal_rf_for_noise()`
- Sensitivity calculation (Q-factor) → `tia_sensitivity()`

## L6: Canonical Problems
- High-speed TIA design → `tia_design_high_speed()`
- Low-noise TIA design → `tia_design_low_noise()`
- Wide-dynamic-range TIA → `tia_design_wide_dynamic()`
- Low-power TIA → `tia_design_low_power()`
- Step response analysis → `tia_compute_step_response()`
- Frequency response → `tia_compute_frequency_response()`

## L7: Applications
- Fiber optic receiver (telecom/datacom) → `tia_fiber_receiver_analyze()`
- LIDAR ToF receiver → `tia_lidar_receiver_analyze()`
- Spectrophotometer frontend → `tia_spectrometer_measure()`
- Optical link budget → `tia_link_budget()`

## L8: Advanced Topics
- Bootstrap input for C_j reduction → `tia_bootstrap_design()`
- Differential TIA → `tia_differential_design()`
- Composite amplifier TIA → `tia_composite_design()`
- T-network feedback → `tia_tnetwork_design()`
- Temperature compensation → `tia_temp_compensation_design()`
- CMOS integrated TIA → `tia_cmos_estimate()`
- APD excess noise factor → `apd_excess_noise_factor()`

## L9: Research Frontiers
- Single-photon detection (SiPM modeling)
- Quantum-limited receiver sensitivity
- Integrated CMOS TIA design (documented)
