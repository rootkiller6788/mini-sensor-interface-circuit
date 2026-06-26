# Gap Report: Thermocouple CJC + RTD

## Missing Items

### L9: Research Frontiers (Low Priority)
1. **NIST ITS-90 full traceability** - Complete uncertainty chain from measurement to NIST primary standards (documented but not implemented)
2. **Multi-sensor fusion** - Kalman-based fusion of T/C and RTD for improved accuracy
3. **Quantum temperature standards** - Johnson noise thermometry, Coulomb blockade thermometry

### Known Limitations
1. **High-range inverse polynomials** - Type K (20.644-54.886 mV range) and some other high-range inverse polynomials use mV input units while most use uV; rescaling needed for full-range Newton refinement
2. **Type C (W-Re) coefficients** - Limited to single forward/inverse range; multi-range fit desirable for sub-zero measurements
3. **Thermistor CJC** - CJC sensor type enum has THERMISTOR entry but Steinhart-Hart equation not implemented

### Completed Since Last Audit
- All L1-L6 items implemented
- L7: 3 application examples
- L8: Kalman filter, robust regression, spline interpolation
- 146 tests passing, 0 failing
- include/ + src/ = 4192 lines (above 3000 threshold)
