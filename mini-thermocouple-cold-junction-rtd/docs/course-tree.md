# Course Tree: Prerequisites for Thermocouple CJC + RTD

## Prerequisite Knowledge Graph

```
Solid State Physics (Seebeck/Peltier effects)
    |
Thermodynamics (temperature, heat transfer)
    |
    +---> Analog Electronics (amplifiers, filters, ADC)
    |         |
    |         +---> Instrumentation Amplifier Design
    |         +---> Analog Filter Design
    |         +---> ADC Theory & Quantization
    |
    +---> Materials Science (thermoelement properties)
    |         |
    |         +---> Metal Alloy Thermoelectric Properties
    |         +---> Platinum Resistivity (Matthiessen's Rule)
    |
    +---> Measurement Theory (uncertainty, calibration)
    |         |
    |         +---> GUM (ISO/IEC 98-3)
    |         +---> Fixed-Point Calibration
    |         +---> Traceability Chains
    |
    +---> Digital Signal Processing
    |         |
    |         +---> Digital Filtering (IIR, MA)
    |         +---> Kalman Filtering
    |
    +---> Control Theory
              |
              +---> PID Control
              +---> Anti-Windup Methods
```

## Module Dependencies (within mini-electronic-info)

```
mini-circuit-analysis/mini-dc-ac-circuit
    |
mini-analog-electronics/mini-analog-ic-design
    |
mini-sensor-measurement/
    |
    +---> mini-thermocouple-cold-junction-rtd  <-- THIS MODULE
    +---> mini-bridge-sensor-strain-gauge-cond
    +---> mini-instrumentation-amplifier-design
    +---> mini-4-20ma-current-loop-implementation
```

## Key Concepts This Module Depends On
1. Ohm's Law (V=IR) - RTD resistance measurement
2. Kirchhoff's Voltage Law - thermocouple loop analysis
3. Polynomial evaluation (Horner's method) - ITS-90 conversion
4. Newton's method - inverse CVD equation
5. Error propagation - GUM uncertainty budget
6. Digital filtering - measurement noise reduction
7. PID control - temperature regulation
