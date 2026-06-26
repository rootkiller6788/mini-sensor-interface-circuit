# Knowledge Graph — Bridge Sensor / Strain Gauge Conditioning

## L1: Definitions (Complete)

| # | Term | Definition | Implementation |
|---|------|-----------|----------------|
| 1 | Wheatstone Bridge | Differential 4-arm resistive divider; Vout = Vexc·[R3/(R1+R3)-R4/(R2+R4)] | `bridge_state_t` in bridge_core.h |
| 2 | Gauge Factor (GF) | GF = (ΔR/R₀)/ε; dimensionless sensitivity of strain gauge | `strain_gauge_t.gauge_factor` |
| 3 | Strain (ε) | Mechanical deformation ε = ΔL/L; measured in με = 10⁻⁶ m/m | `strain_state_t` |
| 4 | Stress (σ) | Force per unit area; σ = E·ε (Hooke) | `stress_state_t` |
| 5 | Bridge Ratio | R1/R3 = R2/R4 at balance | `bridge_output_voltage()` |
| 6 | Bridge Sensitivity | Rated output in mV/V at full scale | `bridge_sensitivity_t` |
| 7 | Bridge Balance | Zero-offset condition R1·R4 = R2·R3 | `bridge_balance_t` |
| 8 | Rosette | Multi-gauge configuration for 2D strain field | `rosette_data_t` |
| 9 | Excitation | Voltage or current powering the bridge | `excitation_mode_t` |
| 10 | Lead Wire Config | 2/3/4/6-wire connection topologies | `leadwire_config_t` |

## L2: Core Concepts (Complete)

| # | Concept | Description | Implementation |
|---|---------|-------------|----------------|
| 1 | Quarter Bridge | 1 active gauge + 3 completion resistors | `BRIDGE_QUARTER` |
| 2 | Half Bridge | 2 active gauges in adjacent arms | `BRIDGE_HALF` |
| 3 | Full Bridge | All 4 arms active; maximum sensitivity | `BRIDGE_FULL` |
| 4 | Ratiometric | ADC Vref tracks Vexc; self-correcting | `EXCITATION_RATIOMETRIC` |
| 5 | Kelvin Connection | 4-wire: force/sense separate | `LEADWIRE_4WIRE` |
| 6 | Self-Heating | Joule heating changes gauge temperature | `strain_gauge_self_heating()` |
| 7 | Thermal Output | Apparent strain from temperature change | `temperature_apparent_strain()` |
| 8 | STC | Self-temperature-compensated gauge matching | `strain_gauge_select_stc()` |

## L3: Mathematical Structures (Complete)

| # | Structure | Formula | Implementation |
|---|-----------|---------|----------------|
| 1 | Exact Bridge Output | Vout = Vexc·[R3/(R1+R3) - R4/(R2+R4)] | `bridge_output_voltage()` |
| 2 | Linearized Output | Vout ≈ Vexc·GF/4·(ε1-ε2+ε3-ε4) | `bridge_output_linear()` |
| 3 | Inverse Equation | ε = -4r/[GF·(2r+1)] (quarter bridge) | `bridge_output_to_strain()` |
| 4 | Bridge Impedance | Zin = (R1+R3)||(R2+R4); Zout = R1||R3 + R2||R4 | `bridge_input/output_impedance()` |
| 5 | Power Dissipation | P_i = V_i²/R_i for each arm | `bridge_power_dissipation()` |
| 6 | CMRR Requirement | CMRR = 20·log10(Vcm_var/Vout_res) | `bridge_required_cmrr()` |
| 7 | Strain Transformation | ε(θ) = εx·cos²θ + εy·sin²θ + γxy·sinθ·cosθ | `strain_gauge_transform()` |
| 8 | Mohr's Circle | Center=(εx+εy)/2, Radius=√[(Δ/2)²+(γ/2)²] | `strain_gauge_mohr_circle()` |

## L4: Fundamental Laws (Complete)

| # | Law | Formula | Implementation |
|---|-----|---------|----------------|
| 1 | Hooke's Law (1D) | σ = E·ε | `hookes_law_stress()` |
| 2 | Hooke's Law (2D) | σij = Qijkl·εkl (plane stress) | `hookes_law_plane_stress()` |
| 3 | Piezoresistive Effect | ΔR/R₀ = GF·ε | `gauge_resistance_from_strain()` |
| 4 | Johnson-Nyquist Noise | Vn = √(4·kB·T·R·BW) | `excitation_johnson_noise()` |
| 5 | Thermal Expansion | εthermal = (αs-αg)·ΔT + (βg/GF)·ΔT | `temperature_apparent_strain()` |
| 6 | Von Mises Yield | σvm = √(σ₁² - σ₁σ₂ + σ₂²) | `von_mises_stress()` |
| 7 | Gauge Factor Decomposition | GF = 1 + 2ν + π₁₁·E | `gauge_factor_from_material()` |
| 8 | Piezoresistance Tensor | π_l = π₁₁ - 2(π₁₁-π₁₂-π₄₄)(l₁²m₁²+...) | `strain_gauge_piezo_longitudinal()` |

## L5: Algorithms/Methods (Complete)

| # | Algorithm | Description | Implementation |
|---|-----------|-------------|----------------|
| 1 | Shunt Calibration | Simulate strain via parallel resistor | `calibration_shunt_strain()` |
| 2 | Two-Point Calibration | Zero + span linear fit | `calibration_two_point()` |
| 3 | Least-Squares Regression | Linear fit with R² and residuals | `calibration_linear_fit()` |
| 4 | Polynomial Regression | Normal equations + Gaussian elimination | `calibration_polynomial_fit()` |
| 5 | Cross-Validation | K-fold CV for model order selection | `calibration_cross_validate()` |
| 6 | Moving Average Filter | O(1) incremental circular buffer | `conditioning_maf_process()` |
| 7 | Median Filter | Sliding window median | `conditioning_median_filter()` |
| 8 | FIR Filter Design | Kaiser window method | `conditioning_fir_design()` |
| 9 | Decimation | Integer downsampling with anti-alias | `conditioning_decimate()` |
| 10 | Oversampling | OSR = 4^(target_bits - adc_bits) | `conditioning_oversampling_ratio()` |
| 11 | Nonlinearity Correction | Polynomial correction via Horner | `bridge_apply_nl_correction()` |

## L6: Canonical Problems (Complete)

| # | Problem | Description | Implementation |
|---|---------|-------------|----------------|
| 1 | Quarter Bridge Measurement | Strain from single active gauge | `example_quarter_bridge.c` |
| 2 | Load Cell Design | Full bridge on elastic element | `application_loadcell_output()` |
| 3 | Pressure Sensor | MEMS diaphragm with piezoresistors | `application_pressure_read()` |
| 4 | Torque Sensor | ±45° gauges on shaft | `application_torque_strain()` |
| 5 | Rosette Analysis | 2D strain field from 3-gauge rosette | `rosette_resolve_strain()` |
| 6 | Bridge Linearization | Exact inverse + polynomial correction | `bridge_fit_nl_correction()` |

## L7: Applications (Complete)

| # | Application | Domain | Implementation |
|---|-------------|--------|----------------|
| 1 | Industrial Weighing | OIML R60 / NTEP legal-for-trade | `application_weighing_total()` |
| 2 | Automotive MAP Sensor | SAE J1763 manifold pressure | `application_automotive_map()` |
| 3 | Aerospace SHM | Boeing/Airbus fatigue monitoring | `application_aerospace_fatigue()` |
| 4 | Multi-Cell Platform | 4-cell corner-corrected scale | `application_multicell_total()` |

## L8: Advanced Topics (Partial)

| # | Topic | Status | Implementation |
|---|-------|--------|----------------|
| 1 | Wireless Sensor Node | Implemented | `application_wireless_battery_life()` |
| 2 | Edge Processing | Implemented | `application_edge_process()` |
| 3 | MEMS Piezoresistors | Implemented | `strain_gauge_silicon_gf()` |
| 4 | Self-Calibrating Bridges | Documented | gap-report.md |
| 5 | IoT Sensor Networks | Documented | gap-report.md |

## L9: Research Frontiers (Partial, Documented)

| # | Topic | Status |
|---|-------|--------|
| 1 | Flexible Strain Sensors | Documented |
| 2 | Nanomaterial Gauges (CNT, graphene) | Documented |
| 3 | 6G-Enabled Wireless SHM | Documented |
| 4 | Quantum Strain Sensing | Documented |
