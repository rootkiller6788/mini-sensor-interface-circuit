# Knowledge Graph — MEMS Accelerometer / Gyroscope / PCB

## L1: Definitions
- Proof mass (kg), spring constant (N/m), damping coefficient (N·s/m)
- Quality factor Q = √(m·k)/b, damping ratio ζ = b/(2√(m·k))
- Sensitivity: V/g (analog), LSB/g (digital)
- Noise density: μg/√Hz (accel), dps/√Hz (gyro)
- Bandwidth (Hz), dynamic range (dB), full-scale range
- Bias instability (dps/h or mg), Angle Random Walk (dps/√h)
- Allan variance: σ²(τ), cluster analysis
- ENOB, SNR, SFDR (ADC specifications)
- Capacitance sensitivity, modulation frequency

## L2: Core Concepts
- Capacitive sensing: ΔC proportional to displacement
- Differential capacitance measurement (common-mode rejection)
- Coriolis effect: Fc = 2m·v×Ω
- Tuning fork gyroscope: drive/sense mode separation
- Open-loop vs closed-loop (force-rebalance) accelerometer
- Electrostatic pull-in instability (displacement > g0/3)
- Capacitance-to-voltage (C2V) conversion
- Quaternion representation of 3D rotation
- Microstrip and stripline transmission lines
- Power Distribution Network (PDN) design
- Mixed-signal PCB layout (AGND/DGND splitting)

## L3: Mathematical Structures
- Second-order ODE: m·ẍ + b·ẋ + k·x = m·a(t)
- Transfer function: H(s) = ωₙ²/(s² + 2ζωₙs + ωₙ²)
- Frequency response: |H(jω)| and ∠H(jω)
- Coriolis coupling equations
- Quaternion algebra (Hamilton multiplication)
- Rotation matrices SO(3)
- Cross-axis sensitivity matrix (3×3)
- Sigma-delta NTF: |NTF(z)| = |(1-z⁻¹)^L|
- PDN impedance model (RLC networks)

## L4: Fundamental Laws
- Brownian motion noise: Fₙ = √(4·kB·T·b)
- Equipartition theorem applied to MEMS
- Nyquist-Shannon sampling theorem
- Gauss-Markov theorem (least-squares optimality)
- IPC-2141 impedance formulas
- Shannon-Hartley channel capacity

## L5: Algorithms/Methods
- Six-position static calibration (least-squares)
- Linear least-squares via normal equations
- Complementary filter (α-blending)
- Mahony explicit complementary filter (PI correction)
- Madgwick gradient descent filter
- Kalman filter (7-state IMU)
- Allan variance computation
- Noise source identification (ARW/BI/RRW)
- Anti-aliasing filter design (Sallen-Key)
- CRC-8 for SPI communication
- Decimation filter (moving average)

## L6: Canonical Problems
- Tri-axial tilt/inclination measurement
- Free-fall and tap detection
- Angular rate integration → heading
- AHRS heading from magnetometer + quaternion
- Six-position calibration execution
- Sensor register map parsing (BMI160)
- Mixed-signal PCB validation
- PDN impedance profiling

## L7: Applications
- Vibration severity monitoring (ISO 10816)
- Bearing fault detection (crest factor, ISO 13373)
- Pedestrian dead reckoning (step detection + length)
- Bosch BMI160 initialization and data parsing
- ST LSM6DSO initialization
- ADI ADXL355 high-performance accel
- TRIAD attitude determination
- Barometric altitude Kalman filter (Boeing/aircraft)
- Temperature-compensated calibration

## L8: Advanced Topics
- SLERP quaternion interpolation
- Quaternion averaging (Markley method)
- RANSAC robust calibration
- Iterative calibration refinement
- Gradient descent orientation optimization
- Zero-Velocity Update (ZUPT) drift compensation
- Gyro mode matching and electrostatic spring softening
- Vibration rectification error (VRE) modeling
- Sigma-delta noise shaping
- Eye diagram analysis for signal integrity
- I3C interface protocol

## L9: Research Frontiers
- Rate-integrating gyro (RIG) mode (documented only)
- Bulk acoustic wave (BAW) disk gyroscope
- NEMS resonant sensors
- Quantum sensing for inertial measurement
- Machine learning for sensor calibration
- 6G-integrated MEMS sensors
