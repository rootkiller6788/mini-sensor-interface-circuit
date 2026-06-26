# Course Alignment: Instrumentation Amplifier Design

## Nine-University Curriculum Mapping

### MIT
- **6.002 Circuits & Electronics**: Op-amp differential amplifiers, feedback
  → `ina_core.c`: Signal decomposition, CMRR computation
  → `ina_topology.c`: 3-op-amp, 2-op-amp analysis
- **6.003 Signal Processing**: Filter design, frequency response
  → `ina_filter.c`: Anti-alias, notch, Sallen-Key filters
- **6.775 CMOS Analog IC Design**: Chopper/auto-zero techniques
  → `ina_advanced.c`: Chopper analysis, auto-zero noise folding
- **6.123 Bioelectronics**: ECG biopotential amplifiers
  → `ina_sensor.h`: Medical IA applications

### Stanford
- **EE101B Circuits II**: Instrumentation amplifier topologies
  → `ina_topology.c`: Topology comparison, selection algorithm
- **EE102A Signal Processing**: Continuous-time filter design
  → `ina_filter.c`: Noise bandwidth, optimal cutoff
- **EE315 VLSI Data Conversion**: PGA, fully differential architectures
  → `ina_advanced.c`: PGA design, fully differential IA

### UC Berkeley
- **EE105 Analog IC**: Differential pair, CMRR analysis
  → `ina_core.h`: CMRR model, mismatch analysis
- **EE16A/B Circuits**: Bridge sensors, op-amp circuits
  → `ina_sensor.c`: Wheatstone bridge, strain gauge
- **EE123 DSP**: Anti-aliasing, sampling theory
  → `ina_filter.c`: Nyquist aliasing, anti-alias design
- **EE145B Medical Imaging**: Sensor interfaces
  → `ina_sensor.c`: RTD, thermocouple signal conditioning

### Illinois
- **ECE 310 DSP**: Filter design, frequency response
- **ECE 451 EM**: RFI/EMI filter design

### Michigan
- **EECS 351 DSP**: Signal processing for sensors
- **EECS 455 Comm**: Noise analysis, SNR optimization
- **EECS 461 Embedded Control**: Sensor calibration, automotive

### Georgia Tech
- **ECE 4270 DSP**: Sampling and reconstruction
- **ECE 4430 Sensor Systems**: Bridge sensors, RTD, TC
- **ECE 6601 Comm**: Noise figure, SNR analysis

### TU Munich
- **Signal Processing**: Analog filter design
- **Communications**: Noise and interference
- **High-Frequency Engineering**: RFI/EMI considerations

### ETH Zurich
- **227-0427 Signal Processing**: Filter bank design
- **227-0436 Comm**: SNR, ENOB analysis
- **227-0455 EM**: Precision analog design, calibration

### Tsinghua
- **信号与系统**: LTI system analysis, transfer functions
- **通信原理**: Noise in communication systems
- **电磁场**: EMI/EMC considerations
- **数字信号处理**: Anti-aliasing filter design

## Reference Textbooks

| Topic | Textbook | Key Chapters |
|-------|----------|-------------|
| IA Design | Kitchin & Counts, "Designer's Guide to IAs" (ADI, 2006) | All |
| Analog Circuits | Sedra & Smith, "Microelectronic Circuits" (2020) | §2, §8 |
| Sensors | Fraden, "Handbook of Modern Sensors" (2016) | §5-7 |
| Filter Design | Zumbahlen, "Linear Circuit Design Handbook" (2008) | §8 |
| Noise | Motchenbacher & Connelly, "Low-Noise Design" (1993) | §4, §11 |
| Calibration | NIST TN 1297, JCGM 100:2008 (GUM) | All |
| Temperature | NIST ITS-90 Monograph 175 | TC polynomials |
| Data Conversion | Kester, "Data Conversion Handbook" (ADI, 2005) | §2, §5 |
