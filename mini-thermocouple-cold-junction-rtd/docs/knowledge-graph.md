# Knowledge Graph: Thermocouple CJC + RTD

## L1: Definitions
| Item | C Implementation | Lean Formalization |
|------|-----------------|-------------------|
| Thermocouple types (K,J,T,E,N,R,S,B,C) | `tc_type_t` enum in `thermocouple_cjc_rtd.h` | `TCTypes` inductive type |
| RTD types (Pt100-Pt2000, Ni, Cu) | `rtd_type_t` enum | `RTDTypes` inductive type |
| CJC sensor types | `cjc_sensor_type_t` enum | - |
| Wiring configurations | `rtd_wiring_t` enum | `WiringConfig` inductive type |
| Measurement error codes | `tc_error_t` enum | `SensorError` inductive type |
| ITS-90 polynomial coefficients | `tc_polynomial_t` struct | - |
| CVD coefficients | `rtd_cvd_coeffs_t` struct | `CVDCoeffs` structure |
| CJC configuration | `cjc_config_t` struct | - |
| Measurement system config | `tc_measurement_config_t` struct | - |
| Measurement result | `tc_measurement_t` struct | - |
| RTD measurement result | `rtd_measurement_t` struct | - |

## L2: Core Concepts
| Concept | Implementation |
|---------|---------------|
| Seebeck effect (dE/dT) | `tc_seebeck_coefficient()`, `tc_seebeck_info()` |
| CJC principle | `tc_cjc_compensate_emf()`, `tc_cjc_voltage()` |
| Callendar-Van Dusen equation | `tc_rtd_temp_to_r()`, `tc_rtd_r_to_temp()` |
| 4-wire Kelvin measurement | `tc_rtd_4wire_measurement()` |
| Ratio-metric measurement | `tc_rtd_ratiometric()` |
| Temperature coefficient of resistance | `tc_rtd_compute_alpha()` |
| Self-heating effect | `tc_rtd_self_heating()` |

## L3: Mathematical Structures
| Structure | Implementation |
|-----------|---------------|
| Polynomial (Horner evaluation) | `tc_horner_eval()`, `tc_horner_derivative()` |
| Piecewise linear interpolation | `tc_piecewise_build()`, `tc_piecewise_eval()` |
| Cubic spline interpolation | `tc_cal_spline_interpolate()` |
| Newton-Raphson solver | `tc_newton_inverse()` |
| Quadratic formula inversion | `tc_rtd_r_to_temp()` (T>=0 branch) |
| Johnson-Nyquist noise | `tc_johnson_noise()` |
| ADC quantization noise | `tc_adc_quantization_noise()` |

## L4: Fundamental Laws
| Law | C Verification | Lean Theorem |
|-----|---------------|--------------|
| Seebeck effect: E = integral of S(T)dT | ITS-90 polynomial tables | - |
| Law of Successive Temperatures | `tc_verify_intermediate_metal_law()` | `cjc_correctness` |
| Law of Intermediate Metals | `tc_verify_intermediate_metal_law()` | - |
| Law of Homogeneous Materials | `tc_verify_intermediate_metal_law()` | - |
| Callendar-Van Dusen: R(T) = R0*(1+A*T+B*T^2) | `tc_rtd_temp_to_r()` | `cvd_quadratic_well_posed` |
| IEC 60751 standard coefficients | `tc_rtd_get_coeffs()` | `ValidCVDCoeffs` |
| ITS-90 temperature scale | NIST Monograph 175 polynomials | - |
| GUM uncertainty propagation | `tc_uncertainty_budget()` | - |

## L5: Algorithms/Methods
| Algorithm | Implementation |
|-----------|---------------|
| ITS-90 polynomial conversion | `tc_temp_to_emf()`, `tc_emf_to_temp()` |
| Newton-Raphson refinement | `tc_newton_inverse()` |
| CJC compensation | `tc_cjc_measure()` |
| 3-wire lead compensation | `tc_rtd_3wire_measurement()` |
| 2-wire lead compensation | `tc_rtd_2wire_measurement()` |
| Cubic spline calibration | `tc_cal_spline_interpolate()` |
| Robust linear regression (Huber) | `tc_robust_linear_fit()` |
| IIR low-pass filter | `tc_iir_filter()` |
| Moving average filter | `tc_moving_average()` |
| ADC resolution analysis | `tc_adc_resolution_required()` |
| Open circuit detection | `tc_detect_open_circuit()` |
| Noise budget analysis | `tc_noise_budget()` |
| Seebeck coefficient computation | `tc_seebeck_coefficient()` |
| RTD self-heating correction | `tc_rtd_self_heating()` |
| Alpha coefficient computation | `tc_rtd_compute_alpha()` |

## L6: Canonical Problems
| Problem | Implementation |
|---------|---------------|
| Thermocouple measurement with CJC | `tc_measure_temperature()` |
| 4-wire precision RTD measurement | `tc_rtd_4wire_measurement()` |
| 3-wire industrial RTD measurement | `tc_rtd_3wire_measurement()` |
| 2-wire RTD measurement | `tc_rtd_2wire_measurement()` |
| Ratio-metric RTD measurement | `tc_rtd_ratiometric()` |
| Calibration table management | `tc_cal_table_create()` |
| Linearity error analysis | `tc_linearity_error()` |
| RTD uncertainty estimation | `tc_rtd_uncertainty()` |

## L7: Applications
| Application | Implementation |
|-------------|---------------|
| Industrial PID temperature control | `tc_pid_control()` |
| Complete measurement pipeline | `tc_measurement_process()` |
| Multi-type T/C comparison | `ex3_cjc_pipeline.c` |

## L8: Advanced Topics
| Topic | Implementation |
|-------|---------------|
| Kalman filter temperature tracking | `tc_kalman_track_temperature()` |
| Huber robust regression | `tc_robust_linear_fit()` |
| Cubic spline calibration | `tc_cal_spline_interpolate()` |

## L9: Research Frontiers
| Topic | Status |
|-------|--------|
| NIST ITS-90 traceability | Documented |
| GUM uncertainty framework | Partial implementation |
| Multi-sensor fusion (T/C + RTD) | Architectural notes |
