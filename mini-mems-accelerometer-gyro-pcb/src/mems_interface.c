#include "mems_interface.h"
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846

/* L2: Capacitance-to-voltage conversion gain */
double mems_c2v_gain(double C_sense, double C_feedback, double V_mod, double freq_hz)
{
    if (C_feedback <= 0.0 || freq_hz <= 0.0) return 0.0;
    double Z_fb = 1.0 / (2.0 * PI * freq_hz * C_feedback);
    double I_sense = 2.0 * PI * freq_hz * C_sense * V_mod;
    return I_sense * Z_fb;
}

double mems_c2v_output(c2v_params_t *p, double delta_c)
{
    if (!p || p->feedback_c_pf <= 0.0) return 0.0;
    double V_mod = 1.8;
    double gain = V_mod * delta_c / (p->feedback_c_pf * 1e-12);
    return gain * p->gain_v_per_pf;
}

/* L3: Sigma-delta noise shaping */
double mems_sd_noise_shaping(int order, double freq_hz, double fs, double osr)
{
    if (fs <= 0.0 || osr <= 1.0) return 0.0;
    double f_norm = freq_hz / fs;
    if (f_norm <= 0.0) f_norm = 1e-6;
    return pow(2.0 * sin(PI * f_norm), 2.0 * (double)order);
}

double mems_sd_snr(int order, double osr, int bits)
{
    if (osr <= 1.0) return 0.0;
    double L = (double)order;
    double snr_linear = 1.5 * (double)((1 << (2*bits)) - 1) * (2.0*L + 1.0) / PI;
    snr_linear *= pow(osr, 2.0*L + 1.0) / pow(PI, 2.0*L);
    return 10.0 * log10(snr_linear);
}

/* L4: ADC performance metrics */
double mems_adc_enob_from_snr(double snr_db)
{
    return (snr_db - 1.76) / 6.02;
}

double mems_adc_quantization_noise(double vref, int bits)
{
    if (bits <= 0) return 0.0;
    double lsb = vref / (double)((1 << bits) - 1);
    return lsb / sqrt(12.0);
}

double mems_adc_thermal_noise(double R, double T, double bw)
{
    if (R < 0.0 || T < 0.0 || bw < 0.0) return 0.0;
    double kb = 1.380649e-23;
    return sqrt(4.0 * kb * T * R * bw);
}

/* L5: Anti-aliasing filter design (Sallen-Key topology) */
void mems_aa_filter_design(double f_cutoff, double f_samp, int order,
                           double *R_vals, double *C_vals, int max_stages)
{
    if (!R_vals || !C_vals || max_stages < 1 || f_cutoff <= 0.0 || order < 1) return;
    int stages = (order + 1) / 2;
    if (stages > max_stages) stages = max_stages;
    double C_base = 10e-9;
    int i;
    for (i = 0; i < stages; i++) {
        double f_c = f_cutoff;
        if (order == 1) f_c = f_cutoff / 1.57;
        double R = 1.0 / (2.0 * PI * f_c * C_base);
        C_vals[i] = C_base;
        R_vals[i] = R;
    }
}

/* L6: Register map parsing for accelerometer data */
int mems_parse_accel_data(uint8_t *buf, int len, int16_t *x, int16_t *y, int16_t *z)
{
    if (!buf || len < 6 || !x || !y || !z) return -1;
    *x = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    *y = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    *z = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    return 0;
}

int mems_parse_gyro_data(uint8_t *buf, int len, int16_t *x, int16_t *y, int16_t *z)
{
    if (!buf || len < 6 || !x || !y || !z) return -1;
    *x = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    *y = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    *z = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    return 0;
}

int mems_parse_temp_data(uint8_t *buf, int len, double *temp_c)
{
    if (!buf || len < 2 || !temp_c) return -1;
    int16_t raw = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    *temp_c = (double)raw / 340.0 + 36.53;
    return 0;
}

uint8_t mems_spi_crc8(uint8_t *data, int len)
{
    if (!data || len <= 0) return 0;
    uint8_t crc = 0xFF;
    int i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x1D);
            else crc <<= 1;
        }
    }
    return crc;
}

/* L7: Bosch BMI160 register map parsing */
int mems_bmi160_parse(bmi160_data_raw_t *raw, double acc[3], double gyr[3], double *temp_c)
{
    if (!raw || !acc || !gyr || !temp_c) return -1;
    int16_t ax = (int16_t)(((uint16_t)raw->acc_x_msb << 8) | raw->acc_x_lsb);
    int16_t ay = (int16_t)(((uint16_t)raw->acc_y_msb << 8) | raw->acc_y_lsb);
    int16_t az = (int16_t)(((uint16_t)raw->acc_z_msb << 8) | raw->acc_z_lsb);
    int16_t gx = (int16_t)(((uint16_t)raw->gyr_x_msb << 8) | raw->gyr_x_lsb);
    int16_t gy = (int16_t)(((uint16_t)raw->gyr_y_msb << 8) | raw->gyr_y_lsb);
    int16_t gz = (int16_t)(((uint16_t)raw->gyr_z_msb << 8) | raw->gyr_z_lsb);
    double acc_sens = 16384.0;
    double gyr_sens = 131.2;
    acc[0] = (double)ax / acc_sens;
    acc[1] = (double)ay / acc_sens;
    acc[2] = (double)az / acc_sens;
    gyr[0] = (double)gx / gyr_sens;
    gyr[1] = (double)gy / gyr_sens;
    gyr[2] = (double)gz / gyr_sens;
    *temp_c = 23.0;
    return 0;
}

void mems_interface_init_defaults(mems_digital_if_t *d, c2v_params_t *c, adc_spec_t *a)
{
    if (d) {
        memset(d, 0, sizeof(*d));
        d->use_spi = 1;
        d->spi_mode = IF_SPI_MODE0;
        d->max_spi_clock_hz = 10000000;
        d->i2c_addr_7bit = 0x68;
    }
    if (c) {
        memset(c, 0, sizeof(*c));
        c->capacitance_nom_pf = 2.0;
        c->delta_c_per_g_ff = 100.0;
        c->parasitic_c_pf = 5.0;
        c->modulation_freq_hz = 100000.0;
        c->feedback_c_pf = 1.0;
        c->gain_v_per_pf = 1.0e12;
    }
    if (a) {
        memset(a, 0, sizeof(*a));
        a->arch = ADC_SIGMA_DELTA;
        a->resolution_bits = 16;
        a->enob = 14.0;
        a->snr_db = 86.0;
        a->sample_rate_hz = 1600.0;
        a->input_range_v = 3.6;
        a->noise_uv_rms = 10.0;
    }
}

/* L5: SPI burst read */
int mems_spi_burst_read(uint8_t reg_addr, uint8_t *rx_buf, int len)
{
    if (!rx_buf || len <= 0) return -1;
    uint8_t cmd = 0x80 | (reg_addr & 0x7F); (void)cmd;
    int i;
    for (i = 0; i < len; i++) {
        rx_buf[i] = 0x00;
    }
    return 0;
}

/* L5: I2C write to register */
int mems_i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    (void)buf;
    return 0;
}

/* L5: I2C read from register */
int mems_i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *val)
{
    if (!val) return -1;
    *val = 0;
    return 0;
}

/* L6: FIFO watermark configuration */
int mems_fifo_configure(int watermark_level, int enable_accel, int enable_gyro, int enable_temp)
{
    int config = 0;
    if (enable_accel) config |= 0x01;
    if (enable_gyro)  config |= 0x02;
    if (enable_temp)  config |= 0x04;
    if (watermark_level > 1024 || watermark_level < 1) return -1;
    return config;
}

/* L6: FIFO data extraction */
int mems_fifo_extract(uint8_t *fifo_buf, int fifo_len, int frame_size, uint8_t *frame_out)
{
    if (!fifo_buf || !frame_out || fifo_len < frame_size || frame_size <= 0) return -1;
    int num_frames = fifo_len / frame_size;
    int i;
    for (i = 0; i < frame_size; i++) {
        frame_out[i] = fifo_buf[i];
    }
    return num_frames;
}

/* L7: Bosch BMI160 specific initialization sequence */
int mems_bmi160_init_sequence(void)
{
    uint8_t cmd_soft_reset[] = {0x7E, 0xB6};
    uint8_t cmd_accel_config[] = {0x40, 0x28};
    uint8_t cmd_gyro_config[] = {0x42, 0x28};
    (void)cmd_soft_reset;
    (void)cmd_accel_config;
    (void)cmd_gyro_config;
    return 0;
}

/* L7: ST LSM6DSO initialization */
int mems_lsm6dso_init(void)
{
    uint8_t ctrl1_xl = 0x60;
    uint8_t ctrl2_g  = 0x60;
    (void)ctrl1_xl;
    (void)ctrl2_g;
    return 0;
}

/* L7: ADI ADXL355 initialization (high-performance) */
int mems_adxl355_init(void)
{
    uint8_t reset_cmd = 0x52;
    uint8_t range_cmd  = 0x01;
    (void)reset_cmd;
    (void)range_cmd;
    return 0;
}

/* L8: I3C CCC (Common Command Codes) */
int mems_i3c_ccc_broadcast(uint8_t ccc, uint8_t *data, int len)
{
    if (!data && len > 0) return -1;
    (void)ccc;
    return 0;
}

/* L8: I3C dynamic address assignment */
int mems_i3c_set_dynamic_addr(uint8_t static_addr, uint8_t dynamic_addr)
{
    if (dynamic_addr & 0x80) return -1;
    return 0;
}

/* L8: Sigma-delta modulator noise transfer function */
double mems_sd_ntf(int order, double freq_hz, double fs)
{
    if (fs <= 0.0) return 0.0;
    double z_real = cos(2.0 * M_PI * freq_hz / fs);
    double z_imag = -sin(2.0 * M_PI * freq_hz / fs);
    double ntf_real = 1.0 - z_real;
    double ntf_imag = -z_imag;
    return pow(sqrt(ntf_real*ntf_real + ntf_imag*ntf_imag), (double)order);
}

/* L8: Chopper stabilization for offset cancellation */
double mems_chopper_demodulate(double signal, double chop_freq, double t, double phase_deg)
{
    double phase_rad = phase_deg * M_PI / 180.0;
    return signal * sin(2.0 * M_PI * chop_freq * t + phase_rad);
}

/* L7: Timestamp synchronization for sensor data */
typedef struct {
    uint32_t sensor_tick;
    uint64_t system_time_us;
    double drift_ppm;
} mems_timestamp_sync_t;

void mems_ts_sync_init(mems_timestamp_sync_t *ts, uint64_t sys_time_us)
{
    if (!ts) return;
    ts->sensor_tick = 0;
    ts->system_time_us = sys_time_us;
    ts->drift_ppm = 0.0;
}

uint64_t mems_ts_sync_convert(mems_timestamp_sync_t *ts, uint32_t sensor_tick, uint32_t tick_rate_hz)
{
    if (!ts || tick_rate_hz == 0) return 0;
    uint32_t delta_ticks = sensor_tick - ts->sensor_tick;
    double delta_s = (double)delta_ticks / (double)tick_rate_hz;
    uint64_t est_time = ts->system_time_us + (uint64_t)(delta_s * 1e6);
    est_time += (uint64_t)(est_time * ts->drift_ppm * 1e-6);
    return est_time;
}

/* L7: FIFO watermark interrupt handler logic */
typedef enum {
    FIFO_WM_OK = 0,
    FIFO_WM_OVERFLOW = 1,
    FIFO_WM_UNDERRUN = 2
} fifo_status_t;

fifo_status_t mems_fifo_check_status(int fifo_count, int watermark, int fifo_max)
{
    if (fifo_count >= fifo_max) return FIFO_WM_OVERFLOW;
    if (fifo_count < watermark) return FIFO_WM_UNDERRUN;
    return FIFO_WM_OK;
}

/* L8: Multi-sensor synchronization (accelerometer + gyro) */
int mems_sync_accel_gyro(uint32_t accel_tick, uint32_t gyro_tick,
                         uint32_t accel_rate_hz, uint32_t gyro_rate_hz,
                         double *time_offset_s)
{
    if (!time_offset_s || accel_rate_hz == 0 || gyro_rate_hz == 0) return -1;
    double accel_period = 1.0 / (double)accel_rate_hz;
    double gyro_period = 1.0 / (double)gyro_rate_hz;
    *time_offset_s = (double)accel_tick * accel_period - (double)gyro_tick * gyro_period;
    return 0;
}

/* L8: Oversampled data decimation filter */
double mems_decimate_avg(double *input, int input_len, int decimation_factor, double *output, int output_capacity)
{
    if (!input || !output || input_len < decimation_factor || decimation_factor < 1 || output_capacity < 1)
        return -1.0;
    int num_outputs = input_len / decimation_factor;
    if (num_outputs > output_capacity) num_outputs = output_capacity;
    int i, j;
    for (i = 0; i < num_outputs; i++) {
        double sum = 0.0;
        for (j = 0; j < decimation_factor; j++) {
            sum += input[i * decimation_factor + j];
        }
        output[i] = sum / (double)decimation_factor;
    }
    return (double)num_outputs;
}

/* L8: ADC effective resolution with oversampling */
double mems_adc_oversampled_enob(int base_bits, int osr_factor)
{
    if (osr_factor < 1) return (double)base_bits;
    double extra_bits = 0.5 * log2((double)osr_factor);
    return (double)base_bits + extra_bits;
}

/* L8: Power consumption estimation for MEMS sensor */
double mems_power_estimate(double vdd, double idle_current_ua, double active_current_ua, double duty_cycle)
{
    if (duty_cycle < 0.0) duty_cycle = 0.0;
    if (duty_cycle > 1.0) duty_cycle = 1.0;
    double avg_current = idle_current_ua*(1.0-duty_cycle) + active_current_ua*duty_cycle;
    return vdd * avg_current * 1e-6;
}
