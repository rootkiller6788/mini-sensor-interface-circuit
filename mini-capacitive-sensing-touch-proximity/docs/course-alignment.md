# Course Alignment — Capacitive Sensing Touch & Proximity

## MIT

| Course | Topic | Implementation |
|--------|-------|----------------|
| 6.003 Signal Processing | RC circuits, Laplace, frequency response | cap_measurement_circuit.c (relaxation osc, charge transfer) |
| 6.630 EM Waves | Electrostatics, Gauss Law, method of images | cap_sense_core.c (parallel-plate), cap_proximity_sense.c (dipole) |
| 6.002 Circuits | Switched capacitor circuits, charge transfer | cap_measurement_circuit.c |

## Stanford

| Course | Topic | Implementation |
|--------|-------|----------------|
| EE102A Signal Processing | Digital filters, FIR/IIR, noise analysis | cap_noise_immunity.c (median, IIR, moving avg) |
| EE101B Circuits | Op-amp guard drivers, TIA, bridge circuits | cap_measurement_circuit.c (AC bridge) |

## UC Berkeley

| Course | Topic | Implementation |
|--------|-------|----------------|
| EE16A/B Circuits | Capacitive sensing, touch interfaces | cap_sense_core.c, cap_touch_detection.c |
| EE117 EM | Conformal mapping, CPW analysis | cap_sensor_geometry.c (elliptic integrals, coplanar strip) |
| EE123 DSP | Sigma-delta modulation, oversampling | cap_measurement_circuit.c (SDM CDC, ENOB) |

## Illinois (UIUC)

| Course | Topic | Implementation |
|--------|-------|----------------|
| ECE 310 DSP | Filter design, noise estimation | cap_noise_immunity.c |
| ECE 451 EM | Capacitance computation, field modeling | cap_sensor_geometry.c |
| ECE 329 Fields | Laplace/Poisson equations | cap_proximity_sense.c (dipole model) |

## Michigan

| Course | Topic | Implementation |
|--------|-------|----------------|
| EECS 351 DSP | Statistical signal processing, detection | cap_touch_detection.c (Neyman-Pearson) |
| EECS 411 Microwave | CPW, coupled lines | cap_sensor_geometry.c |
| EECS 461 Embedded | Sensor interfacing, calibration | demos/demo_touch_calibration.c |

## Georgia Tech

| Course | Topic | Implementation |
|--------|-------|----------------|
| ECE 4270 DSP | Digital filter implementation | cap_noise_immunity.c |
| ECE 6350 EM | Computational EM, method of images | cap_proximity_sense.c |
| ECE 6601 Comm | Spread spectrum, processing gain | cap_noise_immunity.c (spread spectrum) |

## TU Munich

| Course | Topic | Implementation |
|--------|-------|----------------|
| Signal Processing | Adaptive filters, noise suppression | cap_noise_immunity.c |
| Communications | Frequency hopping, interference avoidance | cap_noise_immunity.c |

## ETH Zurich

| Course | Topic | Implementation |
|--------|-------|----------------|
| 227-0427 Signal Proc | Statistical detection theory | cap_touch_detection.c |
| 227-0455 EM | Exact capacitance solutions | cap_sensor_geometry.c |

## Tsinghua

| Course | Topic | Implementation |
|--------|-------|----------------|
| Signal & Systems | RC response, frequency domain | cap_measurement_circuit.c |
| EM Fields | Electrostatic boundary value problems | cap_sensor_geometry.c |
| DSP | Digital filters, detection algorithms | cap_noise_immunity.c, cap_touch_detection.c |

## Common Core (All 9 Schools)

1. **Parallel-plate capacitance** (C=epsilon0*epsilon_r*A/d) — Gauss Law
2. **RC circuit dynamics** (charge/discharge, time constant)
3. **Digital filtering** (FIR, IIR, median, moving average)
4. **Detection theory** (Neyman-Pearson, threshold, false-alarm rate)
5. **Noise analysis** (kT/C, Johnson-Nyquist, quantization)
6. **Sigma-delta modulation** (noise shaping, oversampling, ENOB)
