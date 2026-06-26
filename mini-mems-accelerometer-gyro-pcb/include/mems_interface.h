#ifndef MEMS_INTERFACE_H
#define MEMS_INTERFACE_H
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef enum {
    IF_SPI_MODE0 = 0, IF_SPI_MODE1 = 1, IF_SPI_MODE2 = 2, IF_SPI_MODE3 = 3
} spi_mode_t;

typedef enum {
    IF_I2C_STANDARD = 100, IF_I2C_FAST = 400,
    IF_I2C_FAST_PLUS = 1000, IF_I2C_HIGH_SPEED = 3400
} i2c_speed_t;

typedef enum {
    IF_I3C_SDR = 0, IF_I3C_DDR = 1, IF_I3C_HDR_DDR = 2
} i3c_mode_t;

typedef enum {
    ADC_SAR = 0, ADC_SIGMA_DELTA = 1, ADC_PIPELINE = 2
} adc_arch_t;

typedef struct {
    adc_arch_t arch;
    int resolution_bits;
    double enob;
    double snr_db;
    double sample_rate_hz;
    double input_range_v;
    double noise_uv_rms;
    double inl_lsb;
    double dnl_lsb;
    double sfdr_db;
} adc_spec_t;

typedef struct {
    spi_mode_t spi_mode;
    i2c_speed_t i2c_speed;
    i3c_mode_t i3c_mode;
    uint32_t max_spi_clock_hz;
    uint8_t i2c_addr_7bit;
    int use_i3c;
    int use_spi;
    int use_i2c;
} mems_digital_if_t;

typedef struct {
    double capacitance_nom_pf;
    double delta_c_per_g_ff;
    double parasitic_c_pf;
    double modulation_freq_hz;
    double feedback_c_pf;
    double gain_v_per_pf;
} c2v_params_t;

/* L2: Capacitance-to-voltage conversion */
double mems_c2v_gain(double C_sense, double C_feedback, double V_mod, double freq_hz);
double mems_c2v_output(c2v_params_t *p, double delta_c);

/* L3: Sigma-delta modulation */
double mems_sd_noise_shaping(int order, double freq_hz, double fs, double osr);
double mems_sd_snr(int order, double osr, int bits);

/* L4: ADC performance */
double mems_adc_enob_from_snr(double snr_db);
double mems_adc_quantization_noise(double vref, int bits);
double mems_adc_thermal_noise(double R, double T, double bw);

/* L5: Anti-aliasing filter design */
void mems_aa_filter_design(double f_cutoff, double f_samp, int order,
                           double *R_vals, double *C_vals, int max_stages);

/* L6: Register map parsing */
int mems_parse_accel_data(uint8_t *buf, int len, int16_t *x, int16_t *y, int16_t *z);
int mems_parse_gyro_data(uint8_t *buf, int len, int16_t *x, int16_t *y, int16_t *z);
int mems_parse_temp_data(uint8_t *buf, int len, double *temp_c);
uint8_t mems_spi_crc8(uint8_t *data, int len);

/* L7: Bosch BMI160 register map */
typedef struct {
    uint8_t acc_x_lsb, acc_x_msb;
    uint8_t acc_y_lsb, acc_y_msb;
    uint8_t acc_z_lsb, acc_z_msb;
    uint8_t gyr_x_lsb, gyr_x_msb;
    uint8_t gyr_y_lsb, gyr_y_msb;
    uint8_t gyr_z_lsb, gyr_z_msb;
} bmi160_data_raw_t;
int mems_bmi160_parse(bmi160_data_raw_t *raw, double acc[3], double gyr[3], double *temp_c);

void mems_interface_init_defaults(mems_digital_if_t *d, c2v_params_t *c, adc_spec_t *a);

#endif
