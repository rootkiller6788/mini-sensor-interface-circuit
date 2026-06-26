# Course Dependency Tree: Instrumentation Amplifier Design

## Prerequisites

### Essential (Must Know First)
```
Basic Circuit Theory (L1)
├── Ohm's Law, KVL, KCL
├── Resistor networks, voltage dividers
├── Thévenin/Norton equivalents
└── Capacitor/inductor basics

Operational Amplifiers (L2)
├── Ideal op-amp model (infinite gain, ∞ Zin, 0 Zout)
├── Virtual short concept
├── Inverting/non-inverting configurations
├── Negative feedback and stability
└── Frequency response (GBW, slew rate)

Differential Amplifiers (L2)
├── Differential pair (BJT/MOS)
├── Common-mode vs differential-mode
├── CMRR definition
└── Active loads and current mirrors
```

### Recommended
```
Basic Signal Processing (L3)
├── Fourier analysis (frequency domain)
├── Laplace transform (s-domain)
├── Transfer functions, poles, zeros
└── Bode plots

Noise Theory (L3)
├── Johnson (thermal) noise
├── Shot noise, 1/f (flicker) noise
├── Noise spectral density
└── Equivalent noise bandwidth

Sensor Fundamentals (L1)
├── Resistive sensors (strain gauge, RTD)
├── Thermoelectric sensors (thermocouple)
├── Capacitive sensors (MEMS)
└── Bridge measurement techniques
```

## This Module (Instrumentation Amplifier Design)

```
INA Core (L1-L4)
├── Signal decomposition
├── CMRR analysis
├── Offset and drift
├── Noise integration
└── Error budget

INA Topology (L6)
├── 3-op-amp IA
├── 2-op-amp IA
├── Current-mode IA
└── Indirect Current Feedback (ICF)

Sensor Interfaces (L7)
├── Wheatstone bridge
├── Strain gauge signal chain
├── RTD (Callendar-Van Dusen)
└── Thermocouple (NIST ITS-90, CJC)

Filtering (L5)
├── Anti-aliasing
├── RFI/EMI
├── Sallen-Key
└── Notch (50/60 Hz)

Calibration (L5-L7)
├── Two-point
├── Polynomial (OLS)
├── Temperature compensation
└── Factory calibration

Advanced (L8-L9)
├── Chopper stabilization
├── Auto-zero
├── PGA design
├── Fully differential IA
├── Kalman offset tracking
└── MEMS sensor interface
```

## Downstream Dependencies

### Modules That Depend on This Knowledge
```
Sensor Measurement Systems
├── mini-thermocouple-cold-junction-rtd
├── mini-bridge-sensor-strain-gauge-cond
├── mini-4-20ma-current-loop-implementation
├── mini-photodiode-transimpedance-tia-design
└── mini-mems-accelerometer-gyro-pcb

Analog Electronics
├── mini-analog-electronics (op-amp circuits)
└── mini-sensor-measurement (sensor signal conditioning)

Embedded Systems
├── mini-mcu-embedded-sys (ADC interfacing)
└── mini-iot-edge-computing (wireless sensor nodes)

Control Systems
└── mini-control-automation (feedback, calibration)
```

## Learning Path
1. Start with `ina_core.h/c` → L1 definitions, L4 fundamental laws
2. Continue to `ina_topology.h/c` → L6 canonical design problems
3. Study `ina_sensor.h/c` → L7 real-world sensor applications
4. Explore `ina_filter.h/c` → L5 filter design algorithms
5. Review `ina_calibration.h/c` → L5-L7 calibration and error analysis
6. Advanced: `ina_advanced.h/c` → L8-L9 chopper, PGA, MEMS
7. Formal: `ina_theorems.lean` → Formal verification of key properties
