/**
 * @file sensor_interface.c
 * @brief Sensor-to-4-20mA interface circuits: RTD, thermocouple, strain gauge,
 *        instrumentation amplifier, ratiometric measurement, CJC.
 * Knowledge: L1 (sensor types), L2 (signal conditioning), L3 (transfer functions),
 *            L5 (linearization), L6 (sensor interface design).
 */
#include "current_loop.h"
#include "sensor_interface.h"
#include <math.h>
#include <string.h>

#define PT100_A  3.9083e-3
#define PT100_B -5.775e-7
#define PT100_C -4.183e-12

double sensor_rtd_resistance(double temperature_c, double r0)
{
    if (temperature_c >= 0.0)
        return r0 * (1.0 + PT100_A * temperature_c + PT100_B * temperature_c * temperature_c);
    else
        return r0 * (1.0 + PT100_A * temperature_c + PT100_B * temperature_c * temperature_c
                     + PT100_C * (temperature_c - 100.0) * temperature_c * temperature_c * temperature_c);
}

double sensor_rtd_to_temperature(double resistance_ohm, double r0)
{
    if (r0 <= 0.0) return 0.0;
    double T = (resistance_ohm / r0 - 1.0) / PT100_A;
    for (int iter = 0; iter < 20; iter++) {
        double R_calc = sensor_rtd_resistance(T, r0);
        double dRdT;
        if (T >= 0.0)
            dRdT = r0 * (PT100_A + 2.0 * PT100_B * T);
        else {
            double T2 = T * T;
            dRdT = r0 * (PT100_A + 2.0 * PT100_B * T
                + PT100_C * (4.0 * T * T2 - 300.0 * T2));
        }
        if (fabs(dRdT) < 1e-12) break;
        double delta = (R_calc - resistance_ohm) / dRdT;
        T -= delta;
        if (fabs(delta) < 0.001) break;
    }
    return T;
}

/* Type K thermocouple ITS-90 coefficients (0 to 1372 degC) */
static const double TC_K_COEFFS[] = {
    0.0, 2.508355e-2, 7.860106e-8, -2.503131e-10,
    8.315270e-14, -1.228034e-17, 9.804036e-22, -4.413030e-26,
    1.057734e-30, -1.052755e-35
};
#define TC_K_COEFFS_N 10

double sensor_tc_k_voltage_to_temp(double voltage_mv)
{
    /* ITS-90 coefficients expect voltage in microvolts */
    double voltage_uv = voltage_mv * 1000.0;
    double T = 0.0;
    double v_pow = 1.0;
    for (int i = 0; i < TC_K_COEFFS_N; i++) {
        T += TC_K_COEFFS[i] * v_pow;
        v_pow *= voltage_uv;
    }
    return T;
}

/* Type K inverse ITS-90 coefficients (0 to 500 degC) */
static const double TC_K_INV_COEFFS[] = {
    0.0, 3.94754331391e-2, 2.74652511347e-5, -1.65654067161e-7,
    6.44114635488e-10, -1.49017027087e-12, 2.02339632904e-15,
    -1.48851544267e-18, 4.59636267964e-22
};
#define TC_K_INV_N 9

double sensor_tc_k_temp_to_voltage(double temperature_c)
{
    if (temperature_c < 0.0) return 0.0;
    double v = 0.0;
    double t_pow = 1.0;
    for (int i = 0; i < TC_K_INV_N; i++) {
        v += TC_K_INV_COEFFS[i] * t_pow;
        t_pow *= temperature_c;
    }
    return v;
}

double sensor_cjc_compensate(double voltage_measured_mv, double cj_temp_c, sensor_type_t tc_type)
{
    double v_cjc = 0.0;
    if (tc_type == SENSOR_TYPE_THERMOCOUPLE_K)
        v_cjc = sensor_tc_k_temp_to_voltage(cj_temp_c);
    else if (tc_type == SENSOR_TYPE_THERMOCOUPLE_J) {
        /* Simplified Type J CJC: ~50 uV/degC near 25 degC */
        v_cjc = cj_temp_c * 0.050;
    } else if (tc_type == SENSOR_TYPE_THERMOCOUPLE_T) {
        v_cjc = cj_temp_c * 0.040;
    }
    double v_total = voltage_measured_mv + v_cjc;
    if (tc_type == SENSOR_TYPE_THERMOCOUPLE_K)
        return sensor_tc_k_voltage_to_temp(v_total);
    return v_total / 0.040;
}

double sensor_strain_to_microstrain(double v_out, double v_exc, double gf, int bridge)
{
    if (v_exc <= 0.0 || gf <= 0.0) return 0.0;
    double ratio = v_out / v_exc;
    double factor;
    switch (bridge) {
        case 1: factor = -4.0; break;
        case 2: factor = -2.0; break;
        case 4: factor = -1.0; break;
        default: return 0.0;
    }
    return factor * ratio / gf * 1e6;
}

double sensor_3wire_rtd_compensate(double v1, double v2, double i_exc,
    double lead_r_est, double *error_out)
{
    if (i_exc <= 0.0) { if (error_out) *error_out = 0.0; return 0.0; }
    double r_lead1 = v1 / i_exc;
    double r_lead2 = v2 / i_exc;
    double r_rtd = r_lead1 - r_lead2;
    if (error_out)
        *error_out = fabs(r_lead2 - lead_r_est);
    return r_rtd;
}

double sensor_compute_inst_amp_gain(double vs_min, double vs_max, double va_min, double va_max)
{
    double v_sensor_span = vs_max - vs_min;
    double v_adc_span = va_max - va_min;
    if (v_sensor_span <= 0.0) return 1.0;
    return v_adc_span / v_sensor_span;
}

double sensor_ratiometric_error(double exc_drift_pct, double ref_drift_pct)
{
    double residual = exc_drift_pct - ref_drift_pct;
    return fabs(residual);
}
