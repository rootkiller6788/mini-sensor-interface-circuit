# mini-thermocouple-cold-junction-rtd

Thermocouple Cold-Junction Compensation and RTD Sensor Interface Library in C with Lean 4 formalization.

## Module Status: COMPLETE ✅

- **L1-L6**: Complete
- **L7**: Complete (3 applications: PID control, measurement pipeline, multi-type comparison)
- **L8**: Complete (Kalman tracking, robust regression, cubic spline)
- **L9**: Partial (documented: NIST traceability, multi-sensor fusion, quantum standards)

| Level | Status | Score |
|-------|--------|-------|
| L1 Definitions | Complete | 2 |
| L2 Core Concepts | Complete | 2 |
| L3 Math Structures | Complete | 2 |
| L4 Fundamental Laws | Complete | 2 |
| L5 Algorithms/Methods | Complete | 2 |
| L6 Canonical Problems | Complete | 2 |
| L7 Applications | Complete | 2 |
| L8 Advanced Topics | Complete | 2 |
| L9 Research Frontiers | Partial | 1 |
| **Total** | | **17/18** |

**Line Count**: include/ + src/ = 4192 lines (threshold: 3000) ✅

---

## Core Definitions

### Thermocouple Types
| Type | Composition | Range | Sensitivity at 25C |
|------|------------|-------|-------------------|
| K | Chromel-Alumel (Ni-Cr / Ni-Al) | -270 to +1372 C | ~41 uV/C |
| J | Iron-Constantan (Fe / Cu-Ni) | -210 to +1200 C | ~52 uV/C |
| T | Copper-Constantan (Cu / Cu-Ni) | -270 to +400 C | ~41 uV/C |
| E | Chromel-Constantan (Ni-Cr / Cu-Ni) | -270 to +1000 C | ~68 uV/C |
| N | Nicrosil-Nisil (Ni-Cr-Si / Ni-Si-Mg) | -270 to +1300 C | ~27 uV/C |
| R | Pt-13%Rh vs Pt | -50 to +1768 C | ~10 uV/C |
| S | Pt-10%Rh vs Pt | -50 to +1768 C | ~10 uV/C |
| B | Pt-30%Rh vs Pt-6%Rh | 0 to +1820 C | ~3 uV/C |
| C | W-5%Re vs W-26%Re | 0 to +2315 C | ~14 uV/C |

### RTD Types (IEC 60751)
| Type | R0 (ohm) | TCR alpha (/C) | Range |
|------|---------|----------------|-------|
| Pt100 | 100 | 0.00385055 | -200 to +850 C |
| Pt500 | 500 | 0.00385055 | -200 to +850 C |
| Pt1000 | 1000 | 0.00385055 | -200 to +850 C |
| Ni100 | 100 | 0.006180 | -60 to +180 C |
| Cu100 | 100 | 0.004270 | -100 to +260 C |

---

## Core Theorems

### Seebeck Effect
```
dE = (S_A(T) - S_B(T)) * dT
E(T) = integral_{Tref}^{T} (S_A(t) - S_B(t)) dt
```
where S_A, S_B are absolute Seebeck coefficients of the two thermoelements.

### Law of Successive Temperatures (CJC Foundation)
```
E(T1, T3) = E(T1, T2) + E(T2, T3)
```
Applied to CJC: `E(Thot, 0) = E(Thot, Tcj) + E(Tcj, 0)`

### Law of Intermediate Metals
```
E_{A,C}(T) + E_{C,B}(T) = E_{A,B}(T)   when T is uniform
```
Copper traces at the terminal block introduce no error when isothermal.

### Callendar-Van Dusen Equation (IEC 60751)
```
For T >= 0: R(T) = R0 * (1 + A*T + B*T^2)
For T <  0: R(T) = R0 * (1 + A*T + B*T^2 + C*(T-100)*T^3)
```
where A = 3.9083e-3, B = -5.775e-7, C = -4.183e-12 (IEC standard Pt).

### Johnson-Nyquist Thermal Noise
```
Vn_rms = sqrt(4 * kB * T * R * BW)    where kB = 1.380649e-23 J/K
```

### ADC Quantization Noise
```
Vq_rms = Vref / (2^N * sqrt(12))
```

### Self-Heating Temperature Rise
```
Delta_T = I^2 * R * theta     where theta = dissipation constant [K/W]
```

---

## Core Algorithms

| Algorithm | Function | Complexity | Reference |
|-----------|----------|-----------|-----------|
| ITS-90 Forward (T->EMF) | `tc_temp_to_emf()` | O(P) | NIST Monograph 175 |
| ITS-90 Inverse (EMF->T) | `tc_emf_to_temp()` | O(P + I*P) | NIST Monograph 175 |
| Newton-Raphson Inverse | `tc_newton_inverse()` | O(I*P) | Newton (1687) |
| CJC Compensation | `tc_cjc_measure()` | O(P) | Law of Successive Temps |
| Callendar-Van Dusen (T->R) | `tc_rtd_temp_to_r()` | O(1) | IEC 60751 |
| CVD Inverse (R->T) | `tc_rtd_r_to_temp()` | O(I) | Quadratic formula + Newton |
| 4-Wire Kelvin Measurement | `tc_rtd_4wire_measurement()` | O(1) | Lord Kelvin (1861) |
| 3-Wire Lead Compensation | `tc_rtd_3wire_measurement()` | O(1) | Industrial practice |
| Ratio-metric Measurement | `tc_rtd_ratiometric()` | O(1) | ADS1248 app note |
| Cubic Spline Interpolation | `tc_cal_spline_interpolate()` | O(N) + O(log N) | de Boor (1978) |
| Huber Robust Regression | `tc_robust_linear_fit()` | O(N*I) | Huber (1964) |
| Kalman Temperature Tracking | `tc_kalman_track_temperature()` | O(1) | Kalman (1960) |
| PID Temperature Control | `tc_pid_control()` | O(1) | Ziegler-Nichols (1942) |

---

## Classic Problems Solved

| Problem | Example |
|---------|--------|
| T/C measurement with CJC | `ex1_ktype_measurement.c` |
| 4-wire precision RTD | `ex2_rtd_4wire.c` |
| Multi-type CJC comparison | `ex3_cjc_pipeline.c` |
| Industrial PID temperature control | `ex4_industrial_pid.c` |

---

## Nine-School Course Mapping

| School | Course | Covered Topics |
|--------|--------|---------------|
| MIT | 2.671 Measurement & Instrumentation | Sensor characterization, uncertainty |
| Stanford | ME 220 Sensors | Thermocouple physics, CJC |
| Berkeley | EE105 Microelectronic Devices | Seebeck effect, thermal sensors |
| Illinois | ECE 437 Sensors & Instrumentation | T/C, RTD theory, signal conditioning |
| Michigan | EECS 461 Embedded Control | PID control, sensor integration |
| Georgia Tech | ECE 4415 Sensor Interface | CJC design, 4-wire measurement |
| TU Munich | EI 0406 Sensorik | Industrial temp, IEC 60751 |
| ETH Zurich | 227-0455 EM Waves lab | Thermoelectric effects |
| Tsinghua | 030224 Sensor & Detection Tech | T/C/RTD standards |

---

## Build & Test

```bash
make          # Build library and test
make test     # Run test suite (146 tests, 0 failures)
make examples # Build all 4 examples
make clean    # Remove build artifacts
```

## File Structure

```
mini-thermocouple-cold-junction-rtd/
├── include/
│   └── thermocouple_cjc_rtd.h     (645 lines) Header with all types and APIs
├── src/
│   ├── tc_thermocouple.c          (1172 lines) ITS-90 polynomials for 9 T/C types
│   ├── tc_rtd.c                   (633 lines) CVD equation, 2/3/4-wire, ratio-metric
│   ├── tc_cold_junction.c         (332 lines) CJC compensation, uncertainty
│   ├── tc_linearization.c         (559 lines) Piecewise, spline, robust fit
│   ├── tc_conversion.c            (467 lines) Noise, filtering, Kalman, GUM
│   ├── tc_measurement.c           (384 lines) ADC pipeline, PID controller
│   └── tc_formal.lean             Lean 4 formalization
├── tests/
│   └── test_thermocouple.c        (146 tests, 0 fail)
├── examples/
│   ├── ex1_ktype_measurement.c    Type K with CJC
│   ├── ex2_rtd_4wire.c            Precision RTD measurement
│   ├── ex3_cjc_pipeline.c         Multi-type comparison
│   └── ex4_industrial_pid.c       PID temperature control
├── docs/
│   ├── knowledge-graph.md         L1-L9 coverage table
│   ├── coverage-report.md         Per-layer assessment
│   ├── gap-report.md              Missing items and known limitations
│   ├── course-alignment.md        9-school curriculum mapping
│   └── course-tree.md             Prerequisite dependency tree
└── README.md                      This file
```

## References

- Burns, G.W. et al. (1993) *NIST Monograph 175*: Temperature-EMF Reference Functions
- IEC 60751:2008 *Industrial Platinum Resistance Thermometers*
- ASTM E230/E230M-17 *Standard Specification for T/C EMF Tables*
- Callendar, H.L. (1887) *On the Practical Measurement of Temperature*, Phil. Trans. Roy. Soc. A
- Van Dusen, M.S. (1925) *Platinum-Resistance Thermometry*, JACS
- ISO/IEC 98-3:2008 *Guide to the Expression of Uncertainty in Measurement (GUM)*
- Kalman, R.E. (1960) *A New Approach to Linear Filtering and Prediction Problems*
- Huber, P.J. (1964) *Robust Estimation of a Location Parameter*
