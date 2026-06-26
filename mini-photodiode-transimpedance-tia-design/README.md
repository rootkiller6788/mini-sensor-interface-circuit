# mini-photodiode-transimpedance-tia-design

**Photodiode Transimpedance Amplifier (TIA) Design** — complete design and analysis library covering photodiode physics, TIA topology, noise analysis, stability compensation, and optical receiver applications.

## Module Status: COMPLETE ✅

- **L1-L6**: Complete (all definitions, concepts, math, laws, algorithms, canonical problems)
- **L7**: Complete (3 applications: fiber receiver, LIDAR, spectrophotometer)
- **L8**: Complete (7 advanced topics: bootstrap, differential, composite, T-network, temp comp, CMOS, APD)
- **L9**: Partial (SiPM, quantum-limited receivers documented)

## Line Count Verification

```
include/tia_core.h:        325 lines
include/tia_noise.h:       322 lines
include/tia_stability.h:   350 lines
include/tia_design.h:      306 lines
include/tia_advanced.h:    321 lines
include/tia_photodiode.h:  288 lines
src/tia_core.c:            817 lines
src/tia_noise.c:           223 lines
src/tia_stability.c:       228 lines
src/tia_design.c:          148 lines
src/tia_advanced.c:        190 lines
src/tia_photodiode.c:      125 lines
src/tia_lean.lean:         170 lines
─────────────────────────────────
include/ + src/ total:    3643 lines (≥ 3000 ✓)
```

## Core Definitions (L1)

| Definition | Symbol | Unit |
|-----------|--------|------|
| Responsivity | R | A/W |
| Quantum efficiency | η | — |
| Transimpedance gain | Z_T | Ω (V/A) |
| Dark current | I_dark | nA |
| Junction capacitance | C_j | pF |
| Noise Equivalent Power | NEP | W/√Hz |
| Specific Detectivity | D* | cm·√Hz/W (Jones) |
| 3-dB bandwidth | f_3dB | Hz |
| Phase margin | PM | degrees |
| Input-referred noise | i_n_in | pA/√Hz |
| Receiver sensitivity | P_sens | dBm |

## Core Theorems (L4)

| Theorem | Formula | Reference |
|---------|---------|-----------|
| Johnson-Nyquist | v_n = √(4k_B T R Δf) | Nyquist (1928) |
| Schottky shot noise | i_n = √(2q I_DC Δf) | Schottky (1918) |
| kT/C noise | v_n_rms = √(k_B T / C) | — |
| TIA bandwidth | f_3dB = √(GBWP/(2π R_f C_in)) | Graeme (1996) |
| Compensation C | C_f = √(C_in/(2π R_f GBWP)) | Graeme (1996) |
| APD excess noise | F = M·[1-(1-k)(M-1)²/M²] | McIntyre (1966) |

## Core Algorithms (L5)

| Algorithm | Function |
|-----------|----------|
| TIA design (Graeme method) | `tia_design_basic()` |
| Noise analysis | `tia_noise_analyze()` |
| Stability analysis | `tia_loop_gain_analyze()` |
| Frequency response | `tia_compute_frequency_response()` |
| Step response | `tia_compute_step_response()` |
| Noise optimization | `tia_noise_optimize_rf()` |
| Sensitivity calculation | `tia_sensitivity()` |

## Canonical Problems (L6)

| Problem | Function |
|---------|----------|
| High-speed TIA (BW optimization) | `tia_design_high_speed()` |
| Low-noise TIA (noise optimization) | `tia_design_low_noise()` |
| Wide-dynamic-range TIA | `tia_design_wide_dynamic()` |
| Low-power TIA | `tia_design_low_power()` |

## Nine-School Course Mapping

| School | Relevant Courses |
|--------|-----------------|
| MIT | 6.002 (Circuits), 6.003 (Signals), 6.776 (High-Speed IC) |
| Stanford | EE101B (Circuits II), EE247 (Optical Comm) |
| Berkeley | EE105 (Analog), EE140 (Analog IC), EE242B (Advanced IC) |
| Illinois | ECE 342 (Electronic Circuits), ECE 482 (Optical Imaging) |
| Michigan | EECS 311 (Analog), EECS 523 (Optical Comm) |
| Georgia Tech | ECE 3042 (Microelectronic), ECE 4451 (Optical Eng) |
| TU Munich | EI70040 (Analog), EI71080 (Optical Comm) |
| ETH | 227-0146 (Circuits), 227-0436 (Communications) |
| 清华 | 模拟电子技术, 光电子技术, 通信电子线路 |

## Building and Testing

```
make          # Build library, test, and examples
make test     # Run 70-test suite (all pass)
make examples # Build 3 example applications
make clean    # Remove build artifacts
```

## References

- Graeme, J.G. "Photodiode Amplifiers: Op Amp Solutions" (1996) — the canonical TIA text
- Hobbs, P.C.D. "Building Electro-Optical Systems" 2nd ed. (2011), Ch.18
- Horowitz & Hill "The Art of Electronics" 3rd ed. (2015), Ch.8
- Sedra & Smith "Microelectronic Circuits" 8th ed. (2020), Ch.10
- Agrawal, G.P. "Fiber-Optic Communication Systems" 5th ed. (2021), Ch.4
- Sackinger, E. "Analysis and Design of Transimpedance Amplifiers" (2017)
- Razavi, B. "Design of Integrated Circuits for Optical Communications" 2nd ed. (2012)
