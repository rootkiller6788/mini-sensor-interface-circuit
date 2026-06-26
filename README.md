# Mini Sensor Interface Circuit

A collection of **from-scratch, zero-dependency C implementations** of sensor interface circuits and signal conditioning techniques for industrial, biomedical, and consumer sensing applications. Each sub-module maps to MIT, Stanford, and Berkeley courses, covering the complete analog front-end design chain from transducer physics to digitized data.

## Sub-Modules

| Sub-Module | Topics | Key Courses |
|------------|--------|-------------|
| [mini-4-20ma-current-loop-implementation](mini-4-20ma-current-loop-implementation/) | 4–20 mA loop compliance, 2/3/4-wire topologies, HART protocol, NAMUR NE43 | MIT 2.171, MIT 6.002 |
| [mini-bridge-sensor-strain-gauge-cond](mini-bridge-sensor-strain-gauge-cond/) | Wheatstone bridge, gauge factor, quarter/half/full bridge, signal conditioning, ADC interfacing | MIT 6.003, Stanford EE102A |
| [mini-capacitive-sensing-touch-proximity](mini-capacitive-sensing-touch-proximity/) | CDC topologies, charge transfer, ΣΔ CDC, kT/C noise, gesture recognition, spread spectrum | Stanford EE315, MIT 6.775 |
| [mini-instrumentation-amplifier-design](mini-instrumentation-amplifier-design/) | 2-/3-op-amp IA topologies, CMRR, offset drift, noise modelling, gain-bandwidth, calibration | MIT 6.002, MIT 6.776 |
| [mini-isolated-sensor-digital-isolator](mini-isolated-sensor-digital-isolator/) | Galvanic isolation, CMTI, capacitive/magnetic/optical barriers, isolated SPI/ADC, IEC 60747-5-5 | MIT 6.012, Stanford EE314A |
| [mini-mems-accelerometer-gyro-pcb](mini-mems-accelerometer-gyro-pcb/) | MEMS accelerometer, Coriolis gyroscope, SPI/I2C interfacing, 6-axis calibration, PCB layout | MIT 6.777, Stanford ME310 |
| [mini-photodiode-transimpedance-tia-design](mini-photodiode-transimpedance-tia-design/) | TIA gain, GBWP, NEP/D*, gain peaking, stability compensation, dark/noise current | MIT 6.776, Stanford EE214B |
| [mini-thermocouple-cold-junction-rtd](mini-thermocouple-cold-junction-rtd/) | Seebeck effect, ITS-90, Callendar–Van Dusen, CJC, 4‑wire RTD, rational polynomial fit | MIT 2.171, Stanford ME220 |

## Design Philosophy

- **Zero external dependencies** — pure C11, only `libm` and standard library
- **Self-contained sub-modules** — each has its own `Makefile`, `include/`, `src/`, `tests/`, `examples/`
- **Theory-to-code mapping** — every header documents knowledge coverage levels (L1–L6) aligned with university coursework
- **Pattern library** — establishes sensor-interface coding conventions reused by all downstream electronic modules

## Building

Each sub-module is standalone. Build with `make`:

```bash
cd mini-4-20ma-current-loop-implementation
make
make test
```

Requires **GCC** (or any C11 compiler) and **GNU Make**.

## Project Structure

```
28. mini-sensor-interface-circuit/
├── mini-4-20ma-current-loop-implementation/   # 4–20 mA loop, HART, ISA-50.1 compliance
├── mini-bridge-sensor-strain-gauge-cond/      # Wheatstone bridge, strain gauge conditioning
├── mini-capacitive-sensing-touch-proximity/   # Capacitance measurement, touch/proximity sensing
├── mini-instrumentation-amplifier-design/     # IA topologies, CMRR optimisation
├── mini-isolated-sensor-digital-isolator/     # Galvanic isolation, isolated ADC interfaces
├── mini-mems-accelerometer-gyro-pcb/          # MEMS accel/gyro interface, IMU calibration
├── mini-photodiode-transimpedance-tia-design/ # TIA design, photodiode noise analysis
├── mini-thermocouple-cold-junction-rtd/       # CJC compensation, RTD linearisation
├── .gitignore
├── README.md
└── README-CN.md
```

## License

MIT
