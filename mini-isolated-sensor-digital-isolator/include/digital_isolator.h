/**
 * @file digital_isolator.h
 * @brief Core Digital Isolator Definitions and API
 *
 * Digital isolation provides galvanic separation between sensor-side and
 * controller-side circuits, essential for safety, noise immunity, and
 * ground loop elimination in industrial sensor interfaces.
 *
 * Knowledge Coverage:
 *   L1 Definitions:  isolation voltage, working voltage, CMTI, propagation delay,
 *                    pulse width distortion, data rate, creepage, clearance
 *   L2 Concepts:     galvanic isolation, capacitive coupling barrier, magnetic
 *                    coupling, optical isolation, common mode transients
 *   L3 Math:         transfer functions across isolation barrier, coupled
 *                    transmission line models
 *   L4 Laws:         Paschen's law for breakdown, IEC 60747-5-5, VDE 0884-10
 *   L5 Algorithms:   OOK encoding/decoding, Manchester encoding, CRC verification
 *   L6 Problems:     Isolated SPI/I2C/RS-485, isolated ADC data acquisition
 *
 * References:
 *   - IEC 60747-5-5 Semiconductor devices - Isolators
 *   - VDE 0884-10 Magnetic and capacitive couplers
 *   - TI "Digital Isolator Design Guide" (SLLA284)
 *   - Analog Devices "iCoupler Technology" (AN-0971)
 */

#ifndef DIGITAL_ISOLATOR_H
#define DIGITAL_ISOLATOR_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ==========================================================================
 * L1: CORE DEFINITIONS
 * ========================================================================== */

typedef enum {
    ISOL_TECH_CAPACITIVE,
    ISOL_TECH_MAGNETIC,
    ISOL_TECH_OPTICAL,
    ISOL_TECH_GIANT_MAGNETORESISTIVE,
    ISOL_TECH_RF_MODULATED,
    ISOL_TECH_COUNT
} isolator_technology_t;

typedef enum {
    ISOL_CLASS_BASIC,
    ISOL_CLASS_SUPPLEMENTARY,
    ISOL_CLASS_DOUBLE,
    ISOL_CLASS_REINFORCED
} isolation_class_t;

typedef enum {
    SURGE_CAT_I,
    SURGE_CAT_II,
    SURGE_CAT_III,
    SURGE_CAT_IV
} surge_category_t;

typedef enum {
    POLLUTION_DEG_1,
    POLLUTION_DEG_2,
    POLLUTION_DEG_3,
    POLLUTION_DEG_4
} pollution_degree_t;

typedef struct {
    double  viso_peak_kv;
    double  viso_rms_kv;
    double  viso_surge_kv;
    double  working_voltage_rms;
    double  working_voltage_dc;
    double  creepage_mm;
    double  clearance_mm;
    double  dti_um;
    isolation_class_t  iso_class;
    surge_category_t   surge_cat;
    pollution_degree_t pollution;
    double  propagation_delay_ns;
    double  max_prop_delay_ns;
    double  pulse_width_distortion_ns;
    double  channel_skew_ns;
    double  part_part_skew_ns;
    double  data_rate_mbps;
    double  cmti_kv_us;
    double  max_jitter_ns;
    double  supply_voltage_v;
    double  supply_current_per_ch_ma;
    double  idle_current_ma;
} isolator_barrier_params_t;

typedef struct {
    uint8_t channel_index;
    bool    is_forward;
    bool    has_enable;
    bool    default_state;
    double  input_threshold_v;
    double  output_voh;
    double  output_vol;
} isolator_channel_config_t;

/* L2: Barrier models */
typedef struct {
    double barrier_capacitance_ff;
    double parasitic_side1_ff;
    double parasitic_side2_ff;
    double dielectric_thickness_um;
    double dielectric_constant;
    double carrier_freq_mhz;
    double electrode_area_um2;
} capacitive_barrier_t;

typedef struct {
    double primary_inductance_nh;
    double secondary_inductance_nh;
    double mutual_inductance_nh;
    double coupling_coefficient;
    double coil_resistance_ohm;
    double insulation_thickness_um;
    double coil_turns_primary;
    double coil_turns_secondary;
    double pulse_width_ns;
} magnetic_barrier_t;

typedef struct {
    double led_forward_voltage_v;
    double led_forward_current_ma;
    double current_transfer_ratio;
    double ctr_temp_coeff_pct_per_c;
    double led_degradation_rate;
    double photodiode_responsivity;
    double max_data_rate_mbps;
} optical_barrier_t;

/* L3: Math structures */
typedef struct {
    double freq_hz;
    double magnitude_db;
    double phase_deg;
    double group_delay_ns;
} transfer_point_t;

typedef struct {
    double freq_hz;
    double s11_mag_db;
    double s11_phase_deg;
    double s12_mag_db;
    double s12_phase_deg;
    double s21_mag_db;
    double s21_phase_deg;
    double s22_mag_db;
    double s22_phase_deg;
} isolation_s_params_t;

/* L4: Paschen's Law */
typedef struct {
    double gas_constant_A;
    double gas_constant_B;
    double secondary_emission;
    double gas_pressure_pa;
    double pd_product_pa_m;
    double breakdown_voltage_v;
} paschen_params_t;

/* L5: Encoding algorithms */
typedef struct {
    double  carrier_freq_hz;
    double  sample_rate_hz;
    uint32_t cycles_per_bit;
    uint8_t bit_history;
    double  envelope_threshold;
    double  carrier_amplitude;
} ook_encoder_t;

typedef struct {
    double  bit_period_ns;
    uint8_t current_state;
    uint8_t encoding_polarity;
    uint32_t total_bits_encoded;
    double  dc_balance;
} manchester_encoder_t;

/* L6: Complete device */
typedef struct {
    char        part_number[32];
    isolator_technology_t technology;
    isolator_barrier_params_t barrier;
    uint8_t     num_channels;
    isolator_channel_config_t *channels;
    union {
        capacitive_barrier_t capacitive;
        magnetic_barrier_t   magnetic;
        optical_barrier_t    optical;
    } barrier_model;
    double      ambient_temp_c;
    double      junction_temp_c;
    uint32_t    operating_hours;
    bool        side1_powered;
    bool        side2_powered;
} digital_isolator_t;

/* API declarations */
int digital_isolator_init(digital_isolator_t *isolator,
                          isolator_technology_t tech,
                          isolation_class_t iso_class,
                          uint8_t num_ch);

int digital_isolator_config_channel(digital_isolator_t *isolator,
                                    uint8_t ch_idx,
                                    bool is_forward,
                                    bool default_st);

double paschen_breakdown_voltage(double gap_distance_m, double pressure_pa);

double minimum_creepage_distance(double working_voltage_rms,
                                 pollution_degree_t pollution,
                                 uint8_t material_group);

double minimum_clearance(double surge_voltage_kv, bool homogeneous_field);

double cmti_limited_data_rate(const digital_isolator_t *isolator);

double estimate_junction_temperature(const digital_isolator_t *isolator,
                                     double total_power_mw,
                                     double theta_ja);

double arrhenius_acceleration_factor(double use_temp_c,
                                     double stress_temp_c,
                                     double activation_energy_ev);

bool is_reinforced_isolation(const digital_isolator_t *isolator);

double isolator_power_dissipation(const digital_isolator_t *isolator);

void digital_isolator_destroy(digital_isolator_t *isolator);

#endif /* DIGITAL_ISOLATOR_H */
