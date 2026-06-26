# Knowledge Graph: Instrumentation Amplifier Design

## L1: Definitions (Complete)
- CMRR (Common-Mode Rejection Ratio): CMRR = 20*log10(|Ad/Acm|)
- Differential gain (Ad), Common-mode gain (Acm)
- Input impedance: Zin_diff, Zin_cm
- Offset voltage (Vos) and drift (TC_Vos)
- Input bias current (Ib) and offset current (Ios)
- Noise spectral density (en, in), 1/f corner frequency
- Gain-bandwidth product (GBW), Slew rate (SR)
- PSRR (Power Supply Rejection Ratio)
- SNR, ENOB, Dynamic Range, MDS
- Nonlinearity: INL, DNL
- Settling time, Overload recovery
- Crest factor (RMS to peak-to-peak conversion)

## L2: Core Concepts (Complete)
- Differential vs common-mode signal decomposition
- Superposition principle in differential amplifiers
- Virtual short concept in op-amp circuits
- Negative feedback and stability
- Common-mode rejection mechanism (IA vs diff amp)
- Input bias current return path requirement
- Guarding and shielding techniques
- Ratiometric measurement principle
- Bridge excitation and sensing
- Cold junction compensation (Seebeck effect)

## L3: Mathematical Structures (Complete)
- Transfer function analysis H(s) = N(s)/D(s)
- Noise power spectral density integration
- Callendar-Van Dusen equation (RTD)
- NIST ITS-90 thermocouple polynomials
- Wheatstone bridge transfer function
- Sallen-Key filter transfer function
- Gaussian elimination (for polynomial calibration)
- Newton-Raphson iteration (inverse RTD)
- Kalman filter state estimation
- Least squares polynomial fitting (OLS)

## L4: Fundamental Laws (Complete)
- Superposition theorem (signal decomposition)
- Ohm's law (bridge analysis, Ib offset)
- Kirchhoff's laws (KCL at op-amp inputs, KVL in bridge)
- Johnson-Nyquist noise: Vn = sqrt(4kTRB)
- Hooke's law: sigma = E * epsilon
- Seebeck effect: V = S * DeltaT
- Nyquist-Shannon sampling theorem (anti-alias filter)
- Friis formula (noise figure cascade)
- Callendar-Van Dusen law (RTD temperature)
- Thévenin equivalent (bridge output impedance)

## L5: Algorithms/Methods (Complete)
- Error budget analysis (RSS + worst-case)
- Two-point calibration (gain + offset)
- Polynomial least-squares calibration
- Anti-aliasing filter order determination
- Sallen-Key active filter design
- Twin-T notch filter design
- RFI/EMI input filter design
- Noise bandwidth computation (brick-wall factor)
- Auto-zero offset correction
- Cold junction compensation (CJC) algorithm
- Ratiometric RTD configuration
- Gain resistor selection (E-series)
- Temperature compensation (gain and offset)
- Gaussian elimination (numerical linear algebra)
- Kalman filter update (predict + correct)

## L6: Canonical Problems (Complete)
- Strain gauge measurement system design
- Thermocouple temperature measurement with CJC
- PT100 RTD precision temperature measurement
- 3-op-amp IA topology selection and design
- Wheatstone bridge linearity analysis
- Complete IA signal chain filter design
- Factory calibration sequence
- Topology selection for given specifications

## L7: Applications (Complete)
- Industrial strain gauge weigh scale (e.g., Toledo, Mettler)
- Thermocouple furnace control (Type K, 0-500°C)
- RTD precision temperature monitoring (PT100, IEC 60751)
- Automotive pressure sensor (bridge-based, e.g., Toyota, Bosch)
- Medical ECG biopotential amplifier (3-op-amp IA)
- Industrial 4-20mA transmitter (IA front-end)
- Aircraft strain monitoring (Boeing, Airbus)

## L8: Advanced Topics (Complete)
- Chopper-stabilized IA (ultra-low offset/drift)
- Auto-zero technique (sampled offset correction)
- Noise folding analysis (auto-zero limitation)
- Programmable Gain Amplifier (PGA) design
- Fully differential IA architecture
- Indirect Current Feedback (ICF) topology
- Kalman filter for offset drift tracking
- Adaptive gain control (autoranging)
- Chopper clock design and charge injection

## L9: Research Frontiers (Partial)
- MEMS capacitive sensor interface (accelerometer, gyroscope)
- Synchronous demodulation (lock-in amplifier for MEMS)
- MEMS-CMOS co-integrated IA design
- MEMS accelerometer Brownian noise limits
- Nested chopper techniques for ripple reduction
- Spread-spectrum chopper clocking
