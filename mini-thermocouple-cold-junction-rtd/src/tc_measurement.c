/**
 * @file    tc_measurement.c
 * @brief   Complete thermocouple measurement system: ADC code processing,
 *          voltage conversion, CJC integration, filtering, PID control.
 *
 * Knowledge Coverage:
 *   L5: ADC code to voltage conversion, PGA gain handling
 *   L6: Complete measurement pipeline: ADC -> voltage -> CJC -> temperature
 *   L6: End-to-end measurement function tc_measure_temperature()
 *   L7: PID temperature controller for industrial process control
 *
 * Reference:
 *   Analog Devices AN-1155: Thermocouple Signal Conditioning Using the AD8495
 *   TI Application Report SBAA274: A Basic Guide to Thermocouple Measurements
 *   Astrom, K.J. & Hagglund, T. (1995) "PID Controllers: Theory, Design, Tuning"
 *   Ziegler, J.G. & Nichols, N.B. (1942) "Optimum Settings for Automatic
 *     Controllers" Trans. ASME, vol. 64, pp. 759-768
 */

#include "thermocouple_cjc_rtd.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* =========================================================================
 * L1: Measurement System Initialization
 * ========================================================================= */

/**
 * @brief tc_measurement_init: Initialize measurement configuration
 *
 * Sets up default values for a complete thermocouple measurement system.
 * User should adjust ADC parameters, CJC settings, and filtering based
 * on their specific hardware.
 *
 * Default configuration:
 *   - 16-bit ADC with 2.5V reference (typical delta-sigma ADC)
 *   - PGA gain of 32x for thermocouple signals
 *   - IIR filter alpha = 0.95 (moderate filtering)
 *   - Burnout detection enabled
 *   - Calibration at unity gain, zero offset
 */
tc_error_t tc_measurement_init(tc_measurement_config_t *config,
                                tc_type_t tc_type, rtd_wiring_t wiring) {
    if (!config) return TC_ERR_NULL_POINTER;
    if (tc_type >= TC_COUNT) return TC_ERR_INVALID_TYPE;

    memset(config, 0, sizeof(*config));

    /* Thermocouple settings */
    config->tc_type = tc_type;
    config->wiring = wiring;

    /* ADC defaults (typical 16-bit sigma-delta for thermocouple) */
    config->adc_vref = 2.5;      /* 2.5V reference */
    config->adc_bits = 16;       /* 16-bit */
    config->adc_vmin = 0.0;      /* Single-ended input */
    config->adc_vmax = 2.5;
    config->pga_gain = 32.0;     /* 32x PGA for uV-level signals */
    config->pga_enabled = 1;

    /* CJC defaults - external RTD at terminal block */
    config->cjc.sensor_type = CJC_SENSOR_RTD;
    config->cjc.cj_temperature = 25.0;
    config->cjc.cj_temperature_uncertainty = 0.1;
    config->cjc.cj_resistance = 100.0;
    config->cjc.cj_adc_bits = 16;
    config->cjc.cj_vref = 2.5;
    config->cjc.cj_excitation = 1.0e-3; /* 1 mA for Pt100 */
    config->cjc.pcb_gradient = 0.1;     /* 0.1C across PCB */
    config->cjc.pcb_drift_rate = 0.01;  /* 0.01C/min */
    config->cjc.auto_calibrate = 1;
    config->cjc.calibration_interval = 3600; /* Every hour */

    /* Filter defaults */
    config->filter_alpha = 0.95;
    config->filter_window = 16;
    config->filter_cutoff_hz = 10.0;
    config->sample_rate_hz = 100.0;

    /* Protection defaults */
    config->open_tc_threshold = 0.05; /* 50 mV from rail = open */
    config->burnout_current = 100e-9; /* 100 nA burnout current */
    config->burnout_enabled = 1;

    /* Calibration defaults */
    config->gain_cal = 1.0;
    config->offset_cal = 0.0;
    config->cal_interval_days = 365; /* Annual recalibration */

    return TC_OK;
}

/* =========================================================================
 * L6: ADC Code to Voltage Conversion
 * ========================================================================= */

/**
 * @brief Internal helper: Convert raw ADC code to voltage
 *
 * For a standard single-ended ADC:
 *   V = (ADC_code / 2^N) * Vref
 *
 * For differential ADC with bipolar mode:
 *   V = (ADC_code / 2^(N-1) - 1) * Vref/2  (when MSB is sign bit)
 *
 * This implementation uses the simple unipolar formula since
 * thermocouple ADC inputs are typically single-ended or
 * pseudo-differential.
 *
 * PGA gain is applied: V_input = V_adc / PGA_gain
 *
 * Complexity: O(1).
 */
static double tc_adc_code_to_voltage(uint32_t adc_code, double vref,
                                      size_t bits, double pga_gain,
                                      int pga_enabled) {
    double v_adc;
    double max_code;

    if (bits == 0 || bits > 32) return 0.0;
    max_code = (double)((size_t)1 << bits);

    v_adc = ((double)adc_code / max_code) * vref;

    if (pga_enabled && pga_gain > 1.0) {
        v_adc = v_adc / pga_gain;
    }

    return v_adc;
}

/* =========================================================================
 * L6: Measurement Pipeline - ADC Code Input
 * ========================================================================= */

/**
 * @brief tc_measurement_process: Process ADC reading through full pipeline
 *
 * Pipeline stages:
 *   1. Convert ADC code to input voltage
 *   2. Apply gain and offset calibration
 *   3. Detect open circuit condition
 *   4. Convert voltage to mV (thermocouple EMF)
 *   5. Apply CJC compensation
 *   6. Convert compensated EMF to temperature
 *   7. Compute measurement resolution
 *   8. Estimate uncertainties
 *
 * @param config    Measurement system configuration
 * @param adc_code  Raw ADC reading [counts, 0 to 2^N-1]
 * @param result     Output: complete measurement with all diagnostics
 * @return TC_OK on success
 */
tc_error_t tc_measurement_process(const tc_measurement_config_t *config,
                                   uint32_t adc_code, tc_measurement_t *result) {
    double voltage, emf_mv;
    tc_error_t err;

    if (!config || !result) return TC_ERR_NULL_POINTER;

    memset(result, 0, sizeof(*result));
    result->sample_count = 1;

    /* Stage 1: ADC code to voltage */
    voltage = tc_adc_code_to_voltage(adc_code, config->adc_vref,
                                      config->adc_bits,
                                      config->pga_gain,
                                      config->pga_enabled);

    /* Stage 2: Apply calibration */
    voltage = voltage * config->gain_cal + config->offset_cal;

    /* Stage 3: Open circuit detection */
    {
        int is_open;
        err = tc_detect_open_circuit(voltage, config, &is_open);
        if (err == TC_OK && is_open) {
            result->error = TC_ERR_OPEN_CIRCUIT;
            return TC_ERR_OPEN_CIRCUIT;
        }
    }

    /* Stage 4: Voltage to EMF (mV) */
    emf_mv = voltage * 1000.0; /* V -> mV */
    result->emf_raw = emf_mv;

    /* Stage 5 & 6: CJC compensation and temperature conversion */
    err = tc_cjc_measure(config->tc_type, emf_mv,
                          config->cjc.cj_temperature,
                          &result->temperature);
    if (err != TC_OK) {
        result->error = err;
        return err;
    }

    /* Fill result fields */
    result->emf_compensated = emf_mv + 0.0; /* CJC already baked in */
    result->cj_temperature = config->cjc.cj_temperature;

    /* Stage 7: Resolution (temperature equivalent of 1 LSB) */
    {
        double q = config->adc_vref / (double)((size_t)1 << config->adc_bits);
        double v_lsb = q / (config->pga_enabled ? config->pga_gain : 1.0);
        double seebeck;
        if (tc_seebeck_coefficient(config->tc_type, result->temperature,
                                    &seebeck) == TC_OK && fabs(seebeck) > 1e-15) {
            result->resolution = (v_lsb * 1000.0) / fabs(seebeck);
        } else {
            result->resolution = (v_lsb * 1000.0) / 40.0; /* 40 uV/C default */
        }
    }

    /* Stage 8: Uncertainty budget */
    tc_uncertainty_budget(result, config);

    result->error = TC_OK;
    return TC_OK;
}

/* =========================================================================
 * L6: Measurement Pipeline - Direct Voltage Input
 * ========================================================================= */

/**
 * @brief tc_measurement_process_voltage: Process voltage directly
 *
 * Bypasses the ADC code conversion stage for cases where the
 * voltage is already known (e.g., from a precision DMM or
 * external ADC).
 *
 * This is useful for:
 *   - System simulation and testing
 *   - Post-processing logged voltage data
 *   - Calibration system integration
 */
tc_error_t tc_measurement_process_voltage(
    const tc_measurement_config_t *config,
    double voltage, double cj_temp,
    tc_measurement_t *result) {
    double emf_mv;
    tc_error_t err;

    if (!config || !result) return TC_ERR_NULL_POINTER;

    memset(result, 0, sizeof(*result));
    result->sample_count = 1;

    /* Apply calibration */
    voltage = voltage * config->gain_cal + config->offset_cal;

    emf_mv = voltage * 1000.0;
    result->emf_raw = emf_mv;

    /* CJC compensation and conversion */
    err = tc_cjc_measure(config->tc_type, emf_mv, cj_temp,
                          &result->temperature);
    if (err != TC_OK) {
        result->error = err;
        return err;
    }

    result->cj_temperature = cj_temp;
    tc_uncertainty_budget(result, config);

    return TC_OK;
}

/* =========================================================================
 * L6: End-to-End Measurement (Primary API)
 * ========================================================================= */

/**
 * @brief tc_measure_temperature: Complete thermocouple temperature measurement
 *
 * This is the primary application-level function. Given a raw ADC reading
 * and system configuration, it returns a fully compensated temperature
 * measurement with uncertainty analysis.
 *
 * This single function encapsulates the entire measurement chain:
 *   ADC code -> Voltage -> EMF -> CJC compensation -> Temperature -> Uncertainty
 *
 * It handles:
 *   - Input validation (NULL checks, range validation)
 *   - Open circuit detection with burnout current
 *   - PGA gain compensation
 *   - Gain/offset calibration
 *   - Cold-junction compensation
 *   - ITS-90 polynomial conversion
 *   - GUM uncertainty budget
 *
 * Complexity: O(P + U) where P = max polynomial degree, U = uncertainty calc O(1).
 */
tc_error_t tc_measure_temperature(const tc_measurement_config_t *config,
                                   uint32_t adc_code,
                                   tc_measurement_t *result) {
    return tc_measurement_process(config, adc_code, result);
}

/* =========================================================================
 * L7: PID Temperature Controller
 * ========================================================================= */

/**
 * @brief tc_pid_control: Digital PID controller for temperature regulation
 *
 * Implements the standard parallel-form PID algorithm:
 *
 *   u(t) = Kp * e(t) + Ki * integral(e) + Kd * de/dt
 *
 * where:
 *   e(t) = setpoint - measured  (error signal)
 *
 * Discretized form (backward Euler for integral, backward difference for derivative):
 *
 *   integral[k] = integral[k-1] + e[k] * dt
 *   derivative = (e[k] - e[k-1]) / dt
 *   u[k] = Kp * e[k] + Ki * integral[k] + Kd * derivative
 *
 * Anti-windup: output clamping to [output_min, output_max].
 * When output saturates, the integral term is frozen (conditional
 * integration) to prevent integral windup.
 *
 * Ziegler-Nichols tuning method (for temperature loops):
 *   1. Set Ki=0, Kd=0, increase Kp until sustained oscillation (Ku).
 *   2. Measure oscillation period Pu.
 *   3. Set: Kp=0.6*Ku, Ki=2*Kp/Pu, Kd=Kp*Pu/8
 *
 * Typical temperature loop tuning:
 *   - Slow response (large thermal mass): Kp=2-10, Ki=0.02-0.1, Kd=5-20
 *   - Fast response (small heater): Kp=10-50, Ki=0.1-0.5, Kd=0-5
 *
 * @param setpoint       Desired temperature [C]
 * @param measured       Current measured temperature [C]
 * @param dt             Time step [seconds]
 * @param Kp             Proportional gain
 * @param Ki             Integral gain
 * @param Kd             Derivative gain
 * @param integral       Integral accumulator (in/out, static across calls)
 * @param prev_error     Previous error value (in/out)
 * @param output_limits  [min, max] output limits (e.g., [0, 100] for %)
 * @return Controller output (clamped to output_limits)
 */
double tc_pid_control(double setpoint, double measured, double dt,
                       double Kp, double Ki, double Kd,
                       double *integral, double *prev_error,
                       const double output_limits[2]) {
    double error, derivative, output;

    if (!integral || !prev_error || !output_limits) return 0.0;
    if (dt <= 0.0) dt = 1.0;

    /* Error computation */
    error = setpoint - measured;

    /* Proportional term */
    output = Kp * error;

    /* Integral term with anti-windup */
    *integral += error * dt;
    output += Ki * (*integral);

    /* Derivative term (derivative on error, not measurement,
     * to allow setpoint changes to drive derivative action) */
    derivative = (error - *prev_error) / dt;
    output += Kd * derivative;

    /* Store error for next derivative calculation */
    *prev_error = error;

    /* Output clamping with anti-windup */
    if (output > output_limits[1]) {
        output = output_limits[1];
        /* Back-calculate integral to prevent windup */
        *integral -= error * dt;
    }
    if (output < output_limits[0]) {
        output = output_limits[0];
        *integral -= error * dt;
    }

    return output;
}
