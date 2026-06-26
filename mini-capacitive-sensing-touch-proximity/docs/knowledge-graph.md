# Knowledge Graph — Capacitive Sensing Touch & Proximity

## L1: Definitions (Complete)

| Term | Symbol | Definition | Unit |
|------|--------|------------|------|
| Self-capacitance | C_self | Capacitance from electrode to earth ground | F |
| Mutual capacitance | C_mutual | Capacitance between TX and RX electrodes | F |
| Baseline capacitance | C_b | Quiescent capacitance when untouched | F |
| Delta capacitance | DeltaC | Change due to touch/proximity | F |
| Sensitivity | S | DeltaC / C_baseline | % or ppm |
| Resolution | DeltaC_min | Minimum detectable capacitance change | F |
| SNR | SNR | 20*log10(DeltaC / sigma_noise) | dB |
| Touch threshold | Th_touch | K * sigma_noise (Neyman-Pearson) | F |
| Scan rate | f_scan | Electrodes measured per second | Hz |
| Parasitic capacitance | C_par | Electrode-to-ground stray capacitance | F |
| Guard ring | - | Driven shield to reduce C_par | - |
| Shield electrode | - | Active driven guard plane | - |
| Human body model | HBM | C_body=150pF, R_body=1500Ohm (IEC 61340) | - |
| Electrode | - | Conductor that forms sensing element | - |
| Overlay | - | Dielectric panel covering electrodes | m, epsilon_r |

## L2: Core Concepts (Complete)

- Charge transfer measurement principle
- Sigma-Delta Capacitance-to-Digital Converter (CDC)
- Relaxation oscillator frequency counting
- Self-capacitance vs mutual-capacitance tradeoff
- Projected capacitance (p-cap) touchscreen
- Baseline tracking with asymmetric EMA
- Asymmetric update: fast recovery, slow attack
- Auto-calibration at power-up
- Differential sensing for common-mode rejection
- Spread-spectrum modulation for EMI reduction
- Frequency hopping for noise avoidance
- Shield driving (active guard)
- Liquid/water tolerance via mutual-cap sign
- Parasitic capacitance cancellation
- Touch pressure estimation via contact area
- Glove touch detection (reduced amplitude, slower rise)
- Water film detection (mutual C increase vs decrease)
- Touch proximity approach detection
- Gesture vocabulary (tap, swipe, etc.)
- Interdigitated electrode design

## L3: Mathematical Structures (Complete)

- Parallel-plate model: C = epsilon0 * epsilon_r * A / d
- Coplanar strip model: C' = epsilon0 * eps_r_eff * K(k')/K(k)
- Complete elliptic integral K(k) via AGM algorithm
- Fringe field correction (Palmer formula)
- Inverse power law for proximity: DeltaC = C0 * (r0/r)^n
- Dipole field approximation for proximity
- Exponential moving average: y[n] = alpha*x[n] + (1-alpha)*y[n-1]
- Running variance: sigma2[n] = alpha*(x-mu)^2 + (1-alpha)*sigma2[n-1]
- Charge transfer model: N = C_int*V_ref/(C_sense*V_dd)
- Sigma-delta noise shaping: NTF = (1-z^{-1})^L
- Sigma-delta SQNR: SNR = 6.02*N + 1.76 + (20L+10)*log10(OSR) - ...
- Centroid interpolation: pos = sum(i*DeltaC_i)/sum(DeltaC_i)
- Phasor angle: theta = atan2(sum(sin_i*C_i), sum(cos_i*C_i))
- kT/C noise: v_n^2 = k_B*T/C

## L4: Fundamental Laws (Complete)

- Gauss's Law: surface_integral(E·dA) = Q/epsilon0
- Coulomb's Law: F = k*Q1*Q2/r^2
- Johnson-Nyquist noise: v^2_n = 4k_B*T*R*B
- kT/C noise: v^2_n = k_B*T/C
- Neyman-Pearson detection criterion: P_false = 1 - Phi(K)
- Parseval theorem (conservation of energy in Fourier domain)
- Charge conservation: sum(Q_i) = 0 in closed system
- Energy in capacitor: U = 1/2 * C * V^2
- Charge: Q = C * V

## L5: Algorithms/Methods (Complete)

- Asymmetric EMA baseline tracking
- Adaptive threshold (Neyman-Pearson CFAR)
- Touch state machine with debounce
- Running noise estimation (exponential window)
- Median filter for impulse noise removal
- IIR lowpass filter (single-pole)
- N-tap moving average (O(1) running sum)
- Spread-spectrum processing gain computation
- Frequency hopping sequence generation
- Synchronous detection (lock-in amplifier)
- $1 unistroke gesture recognizer
- Trajectory resampling to N points
- Slider centroid interpolation
- Wheel phasor angle interpolation
- Charge transfer count computation
- Sigma-delta CDC simulation
- Relaxation oscillator frequency computation
- Dual-slope capacitance conversion
- AC bridge output voltage computation
- Coplanar strip capacitance (conformal mapping)
- Fringe field correction (Palmer)
- Guard ring parasitic reduction analysis
- Interdigitated electrode synthesis
- Crosstalk estimation between electrodes
- Frequency selection avoiding interferers
- Auto-ranging for measurement methods
- SIR computation

## L6: Canonical Problems (Complete)

- Single-button capacitive touch sensor (example_touch_button)
- Multi-touch mutual capacitance touchscreen (example_capacitive_slider)
- Proximity wake-up for battery devices (example_proximity_wakeup)
- Liquid-tolerant touch (water film detection algorithm)
- Glove touch through thick overlay (glove confidence)
- Gesture recognition: tap, swipe, double-tap, long-press
- Manufacturing test with Cpk analysis (demo_touch_calibration)
- Production calibration sequence

## L7: Applications (Complete, 3 examples)

- Smartphone touch button (iPhone concept) — example_touch_button.c
- Automotive touch panel (Toyota Smart Key) — example_proximity_wakeup.c
- Consumer slider control (volume/dimmer) — example_capacitive_slider.c
- Industrial HMI with glove support — glove_touch_confidence in src/
- Medical device touch interface (NHS) — water/liquid detection

## L8: Advanced Topics (Partial+)

- Monte Carlo noise simulation — bench_cdc_throughput.c
- Statistical process control (Cpk) — demo_touch_calibration.c
- Stochastic/adaptive threshold — adaptive_threshold in src/
- Bayesian touch classification (structure in gesture_recognition)
- Time-varying baseline under temperature drift

## L9: Research Frontiers (Partial, documented)

- Sub-fF resolution with MEMS capacitive sensors
- AI/ML-based touch classification (gesture template matching as precursor)
- 3D gesture recognition with electrode arrays
- Self-calibrating sensor arrays
- Quantum capacitance sensing (documented only)
