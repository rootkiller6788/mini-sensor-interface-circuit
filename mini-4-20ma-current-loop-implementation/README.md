# mini-4-20ma-current-loop-implementation

## Module Status: COMPLETE

- L1-L6: Complete
- L7: Complete (3 applications)
- L8: Partial (HART protocol; advanced diagnostics)
- L9: Partial (documented, not implemented)

---

## Nine-Layer Knowledge Coverage

| Level | Name | Status | Key Items |
|-------|------|--------|------------|
| L1 | Definitions | Complete | 4-20mA range, live zero, NAMUR NE43, topologies |
| L2 | Core Concepts | Complete | 2/3/4-wire TX, compliance, power budget, IS |
| L3 | Math Structures | Complete | Transfer fn, RSS errors, ENOB/SNR, RC filters |
| L4 | Fundamental Laws | Complete | KVL, Ohm, compliance theorem, min supply |
| L5 | Algorithms | Complete | IIR, moving avg, median, piecewise, spline, OLS |
| L6 | Canonical Problems | Complete | TX/RX design, burnout, calibration, faults |
| L7 | Applications | Complete | PLC input, temp/pressure TX, NE107, predictive |
| L8 | Advanced Topics | Partial | HART FSK, intrinsic safety IEC 60079-11 |
| L9 | Research Frontiers | Partial | WirelessHART, IIoT noted; not implemented |

## Core Definitions (L1)

- **4-20mA current loop**: Industrial analog signaling standard (ISA-50.1, IEC 60381-1)
- **Live zero (4mA)**: Distinguishes zero signal from open-loop fault (0mA)
- **NAMUR NE43**: Standardized failure signal levels (<=3.6mA or >=21.0mA)
- **Compliance voltage**: V_compliance = V_supply - V_transmitter_min
- **Shunt resistor**: Precision resistor converting loop current to voltage (typ 250 ohm -> 1-5V)

## Core Theorems (L4)

1. **Kirchhoff Voltage Law (KVL)** applied to current loop:
   V_supply = V_tx + I_loop * (R_cable + R_shunt + R_barrier)

2. **Compliance condition**: Loop can drive 20mA iff V_margin >= 0 where
   V_margin = V_supply - V_tx_min - I_max * R_total

3. **Maximum cable length theorem**:
   L_max = (V_supply - V_tx_min - I_max*(R_shunt + R_barrier)) / (2 * I_max * r_per_meter)

4. **ENOB theorem** (from quantization noise theory):
   ENOB = (SNR_dB - 1.76) / 6.02
   SNR_dB = 20 * log10(I_span / I_noise_rms)

## Core Algorithms (L5)

| Algorithm | Complexity | File |
|-----------|------------|------|
| Piecewise linearization (binary search) | O(log N) | current_loop.c |
| IIR low-pass filter | O(1)/sample | current_loop.c |
| Moving average (ring buffer) | O(N)/window | current_loop.c |
| Median filter (impulse rejection) | O(N*W*logW) | current_loop.c |
| Polynomial evaluation (Horner) | O(N) | current_loop.c |
| Ordinary Least Squares regression | O(N) | calibration.c |
| Polynomial fit (Gaussian elim) | O(M^3+N*M^2) | calibration.c |
| Cubic spline interpolation | O(N) init | calibration.c |
| Newton-Raphson (RTD inversion) | O(iter) | sensor_interface.c |
| Butterworth LPF design (bilinear) | O(1) | receiver.c |
| HART FSK modulation | O(1)/sample | hart_protocol.c |
| EWMA trend prediction | O(N) | loop_diagnostics.c |

## Canonical Problems (L6)

1. **2-wire loop-powered TX design** — Howland current pump, power budget
2. **Receiver/PLC analog input** — Shunt selection, ADC, filtering, EU conversion
3. **Burnout detection** — Voting scheme with NAMUR NE43 thresholds
4. **Transmitter calibration** — Two-point zero/span, polynomial linearization
5. **Loop fault diagnosis** — Open/short/ground fault/NAMUR alarm detection
6. **RC filter design** — Settling time vs. noise rejection trade-off

## Nine-School Course Mapping

| School | Key Course | Topics Covered |
|--------|-----------|----------------|
| MIT | 6.002 Circuits | KVL, op-amp circuits, Howland pump |
| Stanford | EE102A Signal Processing | Digital filtering, ADC, sampling |
| Berkeley | EE105 Analog | Instrumentation amplifier, sensor |
| Illinois | ECE 310 DSP | IIR/FIR filters, moving average |
| Michigan | EECS 411 Microwave | Cable modeling, impedance |
| Georgia Tech | ECE 4270 DSP | Oversampling, ENOB, quantization |
| TU Munich | Sensor Systems | RTD, thermocouple, strain gauge |
| ETH | 227-0427 Signal Processing | Butterworth, bilinear transform |
| Tsinghua | Industrial Control | 4-20mA, PLC, NAMUR, HART |

## Building

mkdir -p build
gcc -std=c11 -Wall -Wextra -O2 -Iinclude -c src/calibration.c -o build/calibration.o
gcc -std=c11 -Wall -Wextra -O2 -Iinclude -c src/current_loop.c -o build/current_loop.o
gcc -std=c11 -Wall -Wextra -O2 -Iinclude -c src/hart_protocol.c -o build/hart_protocol.o
gcc -std=c11 -Wall -Wextra -O2 -Iinclude -c src/loop_diagnostics.c -o build/loop_diagnostics.o
gcc -std=c11 -Wall -Wextra -O2 -Iinclude -c src/loop_power.c -o build/loop_power.o
gcc -std=c11 -Wall -Wextra -O2 -Iinclude -c src/loop_power.c -o build/loop_power.o
rm -rf build

## File Layout



## References
- ISA-50.1, IEC 60381-1, NAMUR NE43, NAMUR NE107
- IEC 60079-11 (Intrinsic Safety)
- HCF SPEC-99 (HART Protocol)
- Sedra & Smith (2020): Microelectronic Circuits
