/**
 * @file ina_sensor.h
 * @brief Sensor Bridge Interface for Instrumentation Amplifiers
 *
 * Covers L6 Canonical Problems + L7 Applications for sensor interfaces.
 * Instrumentation amplifiers are inherently sensor interface devices -
 * the classic use case is amplifying small differential signals from
 * bridge sensors, thermocouples, RTDs, and other transducers.
 *
 * Reference:
 *   Fraden, "Handbook of Modern Sensors" (2016, 5th Ed.)
 *   Wilson, "Sensor Technology Handbook" (2005)
 *   Analog Devices, "Practical Design Techniques for Sensor Signal
 *     Conditioning" (1999)
 *
 * @course-alignment
 *   MIT 6.123 Bioelectronics: ECG/EEG biopotential amplifiers
 *   Berkeley EE145B Medical Imaging: Sensor interfaces
 *   Michigan EECS 461 Embedded Control: Automotive sensors
 *   Georgia Tech ECE 4430: Sensor systems
 */
#ifndef INA_SENSOR_H
#define INA_SENSOR_H

#include "ina_core.h"
#include "ina_topology.h"

/*===========================================================================
 * Wheatstone Bridge Sensor Interface (L6 Canonical Problem)
 *
 * The Wheatstone bridge is the most common resistive sensor configuration.
 * It converts small resistance changes into differential voltage.
 *
 * For a quarter-bridge with sensor Rs = R0(1 + delta):
 *   Vbridge = Vex * [Rs/(Rs+R) - R/(R+R)]
 *           = Vex * delta / (4 + 2*delta)
 *           ~= Vex * delta / 4    (for delta << 1)
 *
 * Common bridge configurations:
 *   - Quarter-bridge: 1 active element + 3 fixed resistors
 *   - Half-bridge: 2 active elements (opposite arms)
 *   - Full-bridge: 4 active elements (all arms)
 *
 * Bridge nonlinearity error for quarter-bridge:
 *   NL = delta/(4+2*delta) - delta/4
 *   NL_percent ~= -50*delta (for small delta)
 *===========================================================================*/

/** Bridge configuration type */
typedef enum {
    BRIDGE_QUARTER = 0,    /**< 1 active element, 3 fixed */
    BRIDGE_HALF = 1,       /**< 2 active elements (adjacent arms) */
    BRIDGE_HALF_OPPOSITE = 2, /**< 2 active elements (opposite arms) */
    BRIDGE_FULL = 3,       /**< 4 active elements */
    BRIDGE_FULL_DIAGONAL = 4 /**< 4 active, diagonal pairs */
} BridgeType;

/**
 * @brief Wheatstone bridge parameters
 */
typedef struct {
    BridgeType type;            /**< Bridge configuration */
    double excitation_voltage;   /**< Excitation voltage Vex (V) */
    double nominal_resistance;   /**< Nominal arm resistance R0 (ohm) */
    double sensor_resistance;    /**< Current sensor resistance (ohm) */
    double sensor_2_resistance;  /**< Second sensor resistance for half-bridge */
    double sensor_3_resistance;  /**< 3rd sensor for full bridge */
    double sensor_4_resistance;  /**< 4th sensor for full bridge */
    double temperature;          /**< Sensor temperature (?C) for compensation */
    double lead_resistance;      /**< Lead wire resistance per wire (ohm) */
} BridgeSensor;

/**
 * @brief Compute bridge differential output voltage
 *
 * For a quarter-bridge:
 *   V_diff = Vex * (Rs/(Rs+R0) - 0.5)
 *          = Vex * delta / (4 + 2*delta)
 *
 * For a half-bridge (Rs1 and Rs2 in adjacent arms, same sense):
 *   V_diff = Vex * (delta1 - delta2) / 4   (approx)
 *
 * For a full-bridge (all arms active, push-pull):
 *   V_diff = Vex * delta    (approx, twice the sensitivity)
 *
 * @param bridge Bridge configuration and values
 * @return Differential output voltage (V)
 */
double bridge_differential_voltage(const BridgeSensor *bridge);

/**
 * @brief Compute bridge sensitivity in mV/V per unit strain
 *
 * For a quarter-bridge with gauge factor GF:
 *   Sensitivity = Vex * GF * epsilon / 4
 *   Normalized: S = GF/4  (mV/V per unit strain)
 *
 * For full bridge: S = GF  (4x more sensitive)
 */
double bridge_sensitivity(const BridgeSensor *bridge, double gauge_factor);

/**
 * @brief Compute bridge output impedance
 *
 * Thevenin equivalent resistance looking into bridge output:
 * For quarter-bridge: Rth = R0 || R0 = R0/2
 * For full bridge: Rth = R0
 *
 * This is critical for noise analysis - the source resistance
 * sets the thermal noise floor (Johnson noise = sqrt(4kTR)).
 */
double bridge_output_impedance(const BridgeSensor *bridge);

/**
 * @brief Compute bridge nonlinearity error
 *
 * Quarter-bridge has inherent nonlinearity ~ -50*delta percent.
 * Half-bridge and full-bridge are linear for small delta.
 *
 * @return Nonlinearity error as fraction of FS (ppm)
 */
double bridge_nonlinearity_error(const BridgeSensor *bridge);

/**
 * @brief Design IA gain for bridge sensor
 *
 * Given bridge output voltage, desired ADC input range, and
 * required resolution, compute the optimal IA gain.
 *
 * G_optimal = V_adc_fs / (V_bridge_max - V_bridge_min)
 *
 * Also considers: noise, CMRR requirements, bandwidth.
 *
 * @return Recommended IA gain (may differ from exact calculated gain
 *         due to practical constraints)
 */
double bridge_design_ia_gain(double bridge_output_max_v,
                              double adc_full_scale_v,
                              double noise_requirement_uv);

/*===========================================================================
 * Strain Gauge Signal Conditioning (L7 Application)
 *
 * Strain gauges are the most common bridge sensor application.
 *
 * Gauge factor GF = (delta_R/R) / epsilon
 *   where epsilon = delta_L/L is mechanical strain.
 *
 * Metal foil gauges: GF ~= 2.0
 * Semiconductor gauges: GF ~= 50-200
 *
 * Stress-strain relationship (Hooke's Law):
 *   sigma = E * epsilon
 *   where E = Young's modulus (e.g., 200 GPa for steel)
 *
 * Full-scale strain: typically 1000-3000 microstrain (??)
 * At GF=2, delta_R/R = GF*epsilon = 0.002 to 0.006 (0.2% to 0.6%)
 * With Vex = 5V, bridge output = 2.5 mV to 7.5 mV for quarter-bridge
 *===========================================================================*/

typedef struct {
    double gauge_factor;         /**< Gauge factor GF */
    double nominal_resistance;   /**< Unstrained resistance (ohm, typically 120/350) */
    double youngs_modulus_gpa;   /**< Material Young's modulus (GPa) */
    double poisson_ratio;        /**< Poisson's ratio for transverse sensitivity */
    double max_strain_ue;        /**< Maximum strain range (microstrain) */
    double excitation_voltage;   /**< Bridge excitation (V) */
    double temperature_coefficient_ppm; /**< Gauge TCR for compensation */
    double self_heating_mw;      /**< Self-heating power at Vex (mW) */
} StrainGauge;

/**
 * @brief Compute resistance change from strain
 *
 * Delta_R = R0 * GF * epsilon
 */
double strain_to_resistance_change(double strain_ue,
                                    const StrainGauge *gauge);

/**
 * @brief Compute bridge output for given strain
 *
 * For quarter-bridge:
 *   Vout = Vex * GF * epsilon / 4
 *
 * For half-bridge (both gauges active, Poisson arrangement):
 *   Vout = Vex * GF * epsilon * (1 + nu) / 2
 */
double strain_to_bridge_output(double strain_ue,
                                const StrainGauge *gauge,
                                BridgeType bridge_config);

/**
 * @brief Compute strain from bridge output voltage
 *
 * Inverse of strain_to_bridge_output with linearity correction.
 */
double bridge_output_to_strain(double v_bridge, double v_excitation,
                                double gauge_factor, BridgeType bridge_config);

/**
 * @brief Compute stress from strain using Hooke's Law
 *
 * sigma_MPa = E_GPa * epsilon * 1e-6
 * (epsilon is in microstrain)
 */
double strain_to_stress_mpa(double strain_ue, double youngs_modulus_gpa);

/*===========================================================================
 * RTD (Resistance Temperature Detector) Interface (L7 Application)
 *
 * RTDs use the temperature dependence of metal resistance.
 *
 * Platinum RTD (PT100): R0 = 100 ohm at 0?C
 *
 * Callendar-Van Dusen equation (IEC 60751):
 *   For T >= 0?C:
 *     R(T) = R0 * (1 + A*T + B*T^2)
 *   For T < 0?C:
 *     R(T) = R0 * (1 + A*T + B*T^2 + C*(T-100)*T^3)
 *
 * Coefficients for platinum:
 *   A = 3.9083e-3 ?C^-1
 *   B = -5.775e-7 ?C^-2
 *   C = -4.183e-12 ?C^-4
 *
 * Connection methods:
 *   2-wire: Simple, includes lead resistance error
 *   3-wire: Cancels equal lead resistances
 *   4-wire: Kelvin connection, eliminates lead effects
 *===========================================================================*/

#define PT100_R0           100.0    /**< PT100 resistance at 0?C */
#define PT1000_R0          1000.0   /**< PT1000 resistance at 0?C */
#define PT100_COEFF_A      3.9083e-3
#define PT100_COEFF_B     -5.775e-7
#define PT100_COEFF_C     -4.183e-12

/** RTD connection method */
typedef enum {
    RTD_2WIRE = 0,    /**< 2-wire: simple but includes lead resistance */
    RTD_3WIRE = 1,    /**< 3-wire: compensates for equal leads */
    RTD_4WIRE = 2     /**< 4-wire Kelvin: eliminates lead effects */
} RtdConnection;

/**
 * @brief RTD sensor parameters
 */
typedef struct {
    double r0;                   /**< Resistance at 0?C (ohm) */
    double coeff_a;              /**< Linear coefficient */
    double coeff_b;              /**< Quadratic coefficient */
    double coeff_c;              /**< Cubic coefficient (for T<0?C) */
    double lead_resistance;      /**< Lead wire resistance (ohm) */
    RtdConnection connection;    /**< Connection method */
    double excitation_current;   /**< Excitation current (A) */
} RtdSensor;

/**
 * @brief Compute RTD resistance at given temperature
 *
 * Callendar-Van Dusen equation per IEC 60751.
 *
 * @param temperature_c Temperature in ?C
 * @param sensor RTD sensor parameters
 * @return Resistance at temperature (ohm)
 */
double rtd_resistance_at_temperature(double temperature_c,
                                      const RtdSensor *sensor);

/**
 * @brief Compute temperature from RTD resistance
 *
 * Inverse Callendar-Van Dusen using iterative Newton-Raphson.
 *
 * @param resistance Measured resistance (ohm)
 * @param sensor RTD parameters
 * @return Temperature (?C)
 */
double rtd_temperature_from_resistance(double resistance,
                                        const RtdSensor *sensor);

/**
 * @brief Compute lead resistance error
 *
 * For 2-wire connection:
 *   Error = 2 * R_lead  (added to measured resistance)
 *   Temperature error ~= 2 * R_lead / (R0 * A)
 *
 * For 3-wire (matched leads): Error ~= 0
 * For 4-wire: Error = 0 (ideally)
 */
double rtd_lead_error_temperature(double lead_resistance,
                                   RtdConnection connection,
                                   const RtdSensor *sensor);

/**
 * @brief Design IA gain for RTD measurement
 *
 * Given RTD resistance range, excitation current, and ADC full-scale:
 *   V_rtd_max = I_exc * R_max
 *   V_rtd_min = I_exc * R_min
 *   G = V_adc_fs / (V_rtd_max - V_rtd_min)
 *
 * Also computes the required CMRR for ratiometric measurement.
 *
 * @return Required IA gain
 */
double rtd_design_ia_gain(double t_min_c, double t_max_c,
                           const RtdSensor *sensor,
                           double adc_reference_v);

/**
 * @brief Configure ratiometric RTD measurement
 *
 * Ratiometric operation uses the same reference for ADC and bridge
 * excitation, cancelling excitation drift errors.
 *
 * This function computes the required circuit values for ratiometric
 * operation with an IA + ADC system.
 */
void rtd_ratiometric_config(double *r_bias, double *r_ref,
                             const RtdSensor *sensor,
                             double adc_vref, double ia_gain);

/*===========================================================================
 * Thermocouple Interface (L7 Application)
 *
 * Thermocouples produce a small Seebeck voltage proportional to
 * the temperature difference between the hot and cold junctions.
 *
 * V_tc = S * (T_hot - T_cold)   (Seebeck effect)
 *
 * Type K (Chromel-Alumel): ~41 ?V/?C, range -200 to 1372?C
 * Type J (Iron-Constantan): ~52 ?V/?C, range -40 to 750?C
 * Type T (Copper-Constantan): ~43 ?V/?C, range -200 to 350?C
 * Type E (Chromel-Constantan): ~68 ?V/?C, range -200 to 900?C
 * Type N (Nicrosil-Nisil): ~39 ?V/?C, range -200 to 1300?C
 *
 * Cold Junction Compensation (CJC):
 *   T_hot = T_cold + V_tc / S(Thot)
 *
 * Since S is temperature-dependent, CJC requires:
 *   1. Measure cold junction temperature
 *   2. Convert to equivalent voltage: V_cj = integral of S from 0 to T_cj
 *   3. Add to measured voltage: V_total = V_measured + V_cj
 *   4. Convert V_total to T_hot using inverse polynomial
 *===========================================================================*/

/** Thermocouple type enumeration */
typedef enum {
    TC_TYPE_B = 0,   /**< Platinum-Rhodium (0 to 1820?C) */
    TC_TYPE_E = 1,   /**< Chromel-Constantan (-200 to 900?C) */
    TC_TYPE_J = 2,   /**< Iron-Constantan (-40 to 750?C) */
    TC_TYPE_K = 3,   /**< Chromel-Alumel (-200 to 1372?C) */
    TC_TYPE_N = 4,   /**< Nicrosil-Nisil (-200 to 1300?C) */
    TC_TYPE_R = 5,   /**< Platinum-Rhodium (0 to 1768?C) */
    TC_TYPE_S = 6,   /**< Platinum-Rhodium (0 to 1768?C) */
    TC_TYPE_T = 7    /**< Copper-Constantan (-200 to 350?C) */
} ThermocoupleType;

/**
 * @brief Thermocouple parameters
 */
typedef struct {
    ThermocoupleType type;                /**< TC type */
    double seebeck_coefficient_uv_per_c;  /**< Nominal Seebeck coefficient */
    double cold_junction_temperature_c;    /**< Cold junction temperature (?C) */
    double measured_voltage_uv;            /**< Measured TC voltage (?V) */
    double open_circuit_detect_threshold;  /**< Burnout detection threshold */
} ThermocoupleSensor;

/**
 * @brief Get nominal Seebeck coefficient for a TC type at given temperature
 *
 * Uses NIST ITS-90 polynomial coefficients.
 *
 * @param type Thermocouple type
 * @param temperature_c Temperature (?C)
 * @return Seebeck coefficient (?V/?C)
 */
double thermocouple_seebeck(ThermocoupleType type, double temperature_c);

/**
 * @brief Convert thermocouple voltage to temperature (NIST ITS-90)
 *
 * Uses the NIST ITS-90 inverse polynomial for each TC type.
 * This is the standard reference function used in industrial
 * temperature measurement.
 *
 * @param type TC type
 * @param voltage_uv Measured TC voltage (?V) with cjc already applied
 * @return Temperature (?C)
 */
double thermocouple_voltage_to_temperature(ThermocoupleType type,
                                            double voltage_uv);

/**
 * @brief Convert temperature to thermocouple voltage
 *
 * Forward NIST ITS-90 polynomial: T ? V
 */
double thermocouple_temperature_to_voltage(ThermocoupleType type,
                                            double temperature_c);

/**
 * @brief Perform Cold Junction Compensation
 *
 * Full CJC algorithm:
 *   1. Measure CJC temperature Tcj
 *   2. V_cj = f(Tcj) (voltage TC would produce at Tcj relative to 0?C)
 *   3. V_total = V_measured + V_cj
 *   4. T_hot = f^{-1}(V_total)
 *
 * @return Hot junction temperature (?C)
 */
double thermocouple_cjc(ThermocoupleType type,
                         double measured_voltage_uv,
                         double cold_junction_temperature_c);

/**
 * @brief Design IA gain for thermocouple measurement
 *
 * TC output is typically tens of ?V per ?C.
 * For 0.1?C resolution with Type K (41 ?V/?C):
 *   Signal per 0.1?C = 4.1 ?V
 *   If ADC LSB = 1 mV (10-bit, 2.048V ref):
 *     G = 1 mV / 4.1 ?V ? 244
 *
 * @return Required IA gain
 */
double thermocouple_design_ia_gain(ThermocoupleType type,
                                    double temperature_range_c,
                                    double resolution_c,
                                    double adc_lsb_v);

/**
 * @brief Check thermocouple open-circuit (burnout) condition
 *
 * Returns 1 if thermocouple appears open (voltage near zero with
 * known temperature gradient).
 */
int thermocouple_burnout_detect(double measured_voltage_uv,
                                 double expected_voltage_uv,
                                 double threshold_uv);

#endif /* INA_SENSOR_H */
