# Course Tree — Prerequisites and Dependencies

## Module Dependencies

```
mini-bridge-sensor-strain-gauge-cond
|
+-- Prerequisites
|   +-- mini-signal-system-theory (signal processing)
|   +-- mini-circuit-analysis (Ohm, Kirchhoff)
|   +-- mini-analog-electronics (op-amps)
|   +-- mini-sensor-measurement (sensor basics)
|
+-- Core Knowledge
|   +-- Wheatstone Bridge Theory (Christie 1833)
|   +-- Strain Gauge Physics (Kelvin 1856, Smith 1954)
|   +-- Signal Conditioning (Kester 1999)
|   +-- Calibration Metrology (ASTM E74, ISO 376)
|   +-- Application Engineering (OIML R60, SAE J1763)
|
+-- Feeds Into
    +-- mini-communication-principle
    +-- mini-control-automation
    +-- mini-navigation-positioning
    +-- mini-iot-edge-computing
```

## Knowledge Dependency Graph

1. Physics: Ohm -> Bridge Analysis, Hooke -> Stress-Strain, Piezoresistance -> GF
2. Bridge Theory: Wheatstone Equation -> Topologies -> Excitation -> Linearity
3. Strain Gauges: Metal Foil -> Semiconductor -> Temperature Effects -> Installation
4. Signal Chain: In-Amp -> Anti-Alias Filter -> ADC -> Digital Filtering
5. Calibration: Shunt -> Multi-Point -> Regression -> Cross-Validation -> GUM
6. Applications: Load Cell -> Pressure -> Torque -> Rosette -> SHM -> IoT
