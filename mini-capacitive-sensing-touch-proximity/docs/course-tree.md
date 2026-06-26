# Course Tree — Prerequisite Dependencies

```
                          [Mathematics Foundation]
                                   |
              +--------------------+--------------------+
              |                    |                    |
        [Calculus III]     [Linear Algebra]    [Probability]
              |                    |                    |
    Vector calculus,         Matrix ops,        Random variables,
    Gauss/Stokes,            eigenvalues        hypothesis testing
    PDE (Laplace)                               
              |                    |                    |
              +--------------------+--------------------+
                                   |
                        [Electromagnetics]
                                   |
              +--------------------+--------------------+
              |                                         |
    [Electrostatics]                          [Circuit Theory]
              |                                         |
    Coulomb, Gauss,                           RC/RL/RLC, Laplace,
    Poisson, method of                        transfer functions,
    images, conformal                         switched-capacitor
    mapping                                         |
              |                                         |
              +--------------------+--------------------+
                                   |
                        [Capacitive Sensing]
                                   |
    +--------+--------+--------+---+---+--------+--------+--------+
    |        |        |        |       |        |        |        |
[Physics] [Circuits] [DSP]   [Stats] [Algos] [Systems] [Mfg]   [UX]
    |        |        |        |       |        |        |        |
Parallel  Charge    Digital  Neyman-  EMA     Touch    Cpk     Gesture
plate     transfer  filters  Pearson  baseline state   analysis recog-
C=eps*A/d CDC       IIR/FIR  CFAR     tracker  machine          nition
kT/C      oscillator Median   detection         debounce
noise     SDM       Spread   SNR
1/r^n     lock-in   spectrum estimation
falloff   amp       hopping
```

## Module Dependency Sequence

```
1. cap_sense_core        (L1-L4, no dependencies)
   |
2. cap_touch_detection   (L2-L6, depends on cap_sense_core)
   |
3. cap_proximity_sense   (L2-L6, depends on cap_sense_core)
   |
4. cap_noise_immunity    (L2-L5, depends on cap_sense_core)
   |
5. cap_sensor_geometry   (L3-L6, depends on cap_sense_core)
   |
6. cap_measurement_circuit (L2-L6, depends on cap_sense_core)
   |
7. cap_gesture_recognition (L5-L6, depends on cap_sense_core)
```

## External Dependencies

- Mathematics: Calculus (derivatives, integrals), Linear Algebra (vector ops), Probability (Gaussian, hypothesis testing)
- Physics: Electromagnetics (Gauss, Coulomb, Poisson, Laplace), Thermodynamics (kT/C noise)
- EE: Circuits (RC, op-amp, switched-capacitor), DSP (digital filters, detection theory), Control (state machines)
- Statistics: Neyman-Pearson, Cpk, Monte Carlo
