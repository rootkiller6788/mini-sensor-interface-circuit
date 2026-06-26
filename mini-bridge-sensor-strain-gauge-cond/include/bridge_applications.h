/**
 * bridge_applications.h — Bridge Sensor Applications
 *
 * Knowledge Coverage:
 *   L6: Load cell, pressure sensor, torque sensor design
 *   L7: Industrial weighing (ISO 9000), automotive pressure sensing,
 *       structural health monitoring, aerospace load measurement
 *   L8: Wireless strain sensing, self-calibrating bridges,
 *       IoT edge processing for bridge sensors
 *
 * Reference:
 *   - OIML R60 — "Metrological Regulation for Load Cells"
 *   - NTEP Handbook 44 — "Specifications, Tolerances for Weighing Devices"
 *   - SAE J1763 — "Automotive Pressure Sensor Standard"
 *
 * Course: Michigan EECS 411, THU Sensor Technology
 */

#ifndef BRIDGE_APPLICATIONS_H
#define BRIDGE_APPLICATIONS_H

#include "bridge_core.h"
#include "bridge_excitation.h"
#include "bridge_conditioning.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * L6: Canonical Problems — Load Cell Design
 * ======================================================================== */

/**
 * Load cell type classification
 */
typedef enum {
    LOADCELL_SINGLE_POINT,   /* Single point — retail/bench scales */
    LOADCELL_S_BEAM,         /* S-beam — tension/compression */
    LOADCELL_SHEAR_BEAM,     /* Shear beam — industrial platforms */
    LOADCELL_CANISTER,       /* Canister/column — high capacity */
    LOADCELL_PANCAKE,        /* Pancake/low-profile — limited height */
    LOADCELL_BUTTON,         /* Button/compression only — small size */
    LOADCELL_TORQUE           /* Torque sensor configuration */
} loadcell_type_t;

/**
 * Load cell specification (OIML R60 compliance)
 *
 * Accuracy classes per OIML:
 *   Class A (I): Precision — laboratory, analytical balances
 *   Class B (II): Medium precision — commercial scales
 *   Class C (III): Standard — industrial weighing
 *   Class D (IIII): Basic — skip/hopper scales, not legal trade
 */
typedef struct {
    loadcell_type_t type;
    double rated_capacity_kg;       /* Full scale [kg] or [N] */
    double rated_output_mv_per_v;   /* Sensitivity [mV/V] */
    int    max_intervals;           /* nmax — number of verification intervals */
    double min_dead_load_percent;   /* Minimum dead load [% of capacity] */
    double safe_overload_percent;   /* Safe overload without damage [%] */
    double ultimate_overload_percent; /* Breaking overload [%] */
    double combined_error_percent;  /* Max permissible error [%] */
    double creep_percent_30min;     /* Creep [%FS/30min] */
    double temp_effect_zero_percent; /* Zero tempco [%FS/K] */
    double temp_effect_span_percent; /* Span tempco [%FS/K] */
    double excitation_max_v;        /* Max excitation [V] */
    double input_resistance_ohm;    /* Input impedance [Ohm] */
    double output_resistance_ohm;   /* Output impedance [Ohm] */
    double insulation_resistance_mohm; /* Insulation [MOhm at 50VDC] */
} loadcell_spec_t;

/**
 * Compute load cell output for given applied force.
 *
 * For a properly installed load cell:
 *   Vout = Vexc * S * (F / F_rated)  [linear within accuracy class]
 *
 * where S = rated output [mV/V], F = applied force, F_rated = capacity.
 *
 * Example: 100 kg load cell, S=2 mV/V, Vexc=10V, F=75 kg
 *   Vout = 10 * 0.002 * 75/100 = 0.015 V = 15 mV
 *
 * @param loadcell      Load cell specification
 * @param applied_force_kg Applied load [kg]
 * @param vex           Excitation voltage [V]
 * @return              Bridge output voltage [V]
 *
 * Complexity: O(1).
 */
double application_loadcell_output(const loadcell_spec_t *loadcell,
                                   double applied_force_kg, double vex);

/**
 * Compute force from load cell output voltage.
 *
 * F = F_rated * (Vout / (Vexc * S))
 *
 * @param loadcell      Load cell specification
 * @param vout          Measured output [V]
 * @param vex           Excitation voltage [V]
 * @return              Computed force [kg]
 *
 * Complexity: O(1).
 */
double application_loadcell_force(const loadcell_spec_t *loadcell,
                                  double vout, double vex);

/**
 * Compute corner loading error for multi-loadcell platforms.
 *
 * Platform scales with 4 load cells:
 *   F_total = F1 + F2 + F3 + F4
 *
 * Corner error is the variation in indicated weight when the
 * same load is placed at different positions on the platform.
 *
 * Corner adjustment factor for each cell:
 *   K_i = F_center / F_corner_i (computed during calibration)
 *
 * @param individual_forces  Array of 4 corner force readings [kg]
 * @param corner_factors     Array of 4 adjustment coefficients
 * @return                   Total corrected force [kg]
 *
 * Complexity: O(1).
 */
double application_multicell_total(const double individual_forces[4],
                                   const double corner_factors[4]);

/* ========================================================================
 * L6: Pressure Sensor Bridge Design
 * ======================================================================== */

/**
 * Pressure sensor type
 */
typedef enum {
    PRESSURE_ABSOLUTE,   /* Referenced to vacuum */
    PRESSURE_GAUGE,      /* Referenced to atmospheric */
    PRESSURE_DIFFERENTIAL, /* Difference between two ports */
    PRESSURE_SEALED      /* Referenced to fixed sealed reference */
} pressure_type_t;

/**
 * Pressure sensor specification based on MEMS diaphragm bridge
 */
typedef struct {
    pressure_type_t type;
    double pressure_range_kpa;       /* Full scale [kPa] */
    double overpressure_kpa;         /* Max safe overpressure [kPa] */
    double burst_pressure_kpa;       /* Burst/destruction pressure [kPa] */
    double sensitivity_mv_per_v_per_kpa; /* Bridge sensitivity [mV/V/kPa] */
    double nonlinearity_percent;     /* Nonlinearity [%FS] */
    double bridge_resistance_ohm;    /* Bridge resistance [Ohm] */
    double tempco_zero_percent_fs_per_c; /* Zero temperature effect */
    double tempco_span_percent_fs_per_c; /* Span temperature effect */
    double response_time_ms;         /* Step response time [ms] */
    double resonant_frequency_khz;   /* Mechanical resonance [kHz] */
    const char *media_compatibility; /* Compatible fluids/gases */
} pressure_sensor_spec_t;

/**
 * Compute pressure from bridge output.
 *
 * For a MEMS piezoresistive pressure sensor:
 *
 *   P = (Vout / Vexc) * (1 / S) * P_FS + P_offset
 *
 * where S = sensitivity [mV/V/kPa].
 *
 * Temperature compensation:
 *   P_corrected = P + TC_zero*(T-Tref) + TC_span*(T-Tref)*P/P_FS
 *
 * @param sensor        Pressure sensor specification
 * @param vout          Measured output voltage [V]
 * @param vex           Excitation voltage [V]
 * @param temp_c        Current temperature [degC]
 * @param ref_temp_c    Calibration reference temperature [degC]
 * @return              Pressure [kPa]
 *
 * Complexity: O(1).
 */
double application_pressure_read(const pressure_sensor_spec_t *sensor,
                                 double vout, double vex,
                                 double temp_c, double ref_temp_c);

/**
 * Design MEMS pressure sensor diaphragm thickness.
 *
 * For a square diaphragm (edge length a) under uniform pressure P:
 *
 *   sigma_max = 0.308 * P * (a/t)^2 * (for clamped edges)
 *
 * where t = diaphragm thickness.
 *
 * Maximum strain at diaphragm center:
 *   eps_max = 0.138 * P * (a/t)^2 * (1-nu^2) / E
 *
 * For a given max pressure and target strain:
 *   t = a * sqrt(0.138 * P * (1-nu^2) / (E * eps_max))
 *
 * Typical values for silicon:
 *   a = 1-5 mm, t = 10-100 um, P = 0-1000 kPa
 *
 * @param edge_length_um     Diaphragm edge length [um]
 * @param max_pressure_kpa   Full scale pressure [kPa]
 * @param target_strain_ue   Target max strain [microstrain]
 * @param E_gpa              Young's modulus [GPa]
 * @param nu                 Poisson's ratio
 * @return                   Required diaphragm thickness [um]
 *
 * Complexity: O(1).
 * Reference: Bao, "Micro Mechanical Transducers", Elsevier 2000
 */
double application_pressure_diaphragm_thickness(double edge_length_um,
                                                 double max_pressure_kpa,
                                                 double target_strain_ue,
                                                 double E_gpa, double nu);

/* ========================================================================
 * L6: Torque Sensor Bridge Design
 * ======================================================================== */

/**
 * Torque sensor specification — typically uses full bridge with
 * gauges at +/- 45 degrees on a circular shaft
 */
typedef struct {
    double torque_range_nm;          /* Full scale torque [N*m] */
    double max_speed_rpm;            /* Maximum rotational speed [RPM] */
    double shaft_diameter_mm;        /* Shaft diameter [mm] */
    double shaft_material_E_gpa;     /* Young's modulus of shaft [GPa] */
    double shaft_material_nu;        /* Poisson's ratio */
    double rated_output_mv_per_v;    /* Bridge sensitivity [mV/V] */
    double bridge_resistance_ohm;    /* Bridge resistance [Ohm] */
    double nonlinearity_percent;     /* Nonlinearity [%FS] */
    double bandwidth_khz;            /* Measurement bandwidth [kHz] */
} torque_sensor_spec_t;

/**
 * Compute shear stress from applied torque on a circular shaft.
 *
 * For a solid circular shaft under torsion:
 *
 *   tau_max = T * r / J = T / (pi * d^3 / 16)
 *
 * where:
 *   T = applied torque [N*m]
 *   r = shaft radius [m]
 *   d = shaft diameter [m]
 *   J = polar moment of inertia = pi * d^4 / 32 [m^4]
 *
 * The principal stresses on the shaft surface at 45 degrees:
 *   sigma_1 = tau, sigma_2 = -tau
 *
 * The engineering shear strain:
 *   gamma = tau / G = 2*(1+nu)*tau / E
 *
 * Strain at +/- 45 degrees (gauge orientation):
 *   eps_45 = gamma/2 = (1+nu)*tau / E = T * (1+nu) / (E * pi * d^3 / 16)
 *
 * @param torque_nm      Applied torque [N*m]
 * @param diameter_mm    Shaft diameter [mm]
 * @return               Surface shear stress [MPa]
 *
 * Complexity: O(1).
 */
double application_torque_shear_stress(double torque_nm, double diameter_mm);

/**
 * Compute strain at torque gauge location (+/-45 degrees).
 *
 * @param torque_nm      Applied torque [N*m]
 * @param diameter_mm    Shaft diameter [mm]
 * @param E_gpa          Young's modulus [GPa]
 * @param nu             Poisson's ratio
 * @return               Strain at gauge [microstrain]
 *
 * Complexity: O(1).
 */
double application_torque_strain(double torque_nm, double diameter_mm,
                                 double E_gpa, double nu);

/**
 * Design torque sensor shaft diameter for target strain.
 *
 * Given desired full-scale strain for good SNR:
 *   d = cbrt(T * 16 * (1+nu) / (pi * E * eps_target))
 *
 * Trade-off:
 *   Larger d → lower strain (worse SNR) but higher strength
 *   Smaller d → higher strain (better SNR) but lower overload margin
 *
 * @param max_torque_nm   Full-scale torque [N*m]
 * @param target_strain_ue Target strain at full scale [microstrain]
 * @param E_gpa           Young's modulus [GPa]
 * @param nu              Poisson's ratio
 * @return                Recommended shaft diameter [mm]
 *
 * Complexity: O(1).
 */
double application_torque_shaft_diameter(double max_torque_nm,
                                         double target_strain_ue,
                                         double E_gpa, double nu);

/* ========================================================================
 * L7: Applications — Industrial and Aerospace
 * ======================================================================== */

/**
 * Industrial weighing system configuration (ISO 9000/NTEP)
 *
 * Multi-cell platform scale design parameters.
 */
typedef struct {
    int    n_cells;                 /* Number of load cells (typ 4, 6, 8) */
    loadcell_spec_t cells[8];       /* Individual cell specs */
    double corner_factors[8];       /* Corner adjustment coefficients */
    double platform_mass_kg;        /* Dead load (platform) [kg] */
    double max_capacity_kg;         /* Net capacity [kg] */
    double resolution_kg;           /* Display resolution [kg] */
    int    legal_for_trade;         /* NTEP/OIML certified */
    double zero_tracking_kg;        /* Automatic zero tracking range [kg] */
    double tare_max_percent;        /* Max tare [% of capacity] */
    double motion_detection_kg;     /* Motion detection threshold [kg] */
} weighing_system_t;

/**
 * Compute total weight from multi-cell readings.
 *
 * @param system       Weighing system configuration
 * @param readings     Individual cell readings [kg]
 * @param n_readings   Number of readings (must equal n_cells)
 * @return             Total weight [kg]
 *
 * Complexity: O(n_cells).
 */
double application_weighing_total(const weighing_system_t *system,
                                  const double *readings, int n_readings);

/**
 * Automotive pressure sensor (SAE J1763) — manifold air pressure
 *
 * Key requirements:
 * - Operating temperature: -40 to +125 degC
 * - Accuracy: +/- 1% FS over full temperature range
 * - Response time: < 1 ms
 * - Media compatibility: gasoline, diesel, exhaust gas
 * - EMC: ISO 11452, ISO 7637 (automotive transients)
 */
typedef struct {
    pressure_sensor_spec_t sensor;
    double supply_voltage_v;         /* 5V typical automotive */
    double adc_reference_v;          /* ADC reference (typ ratiometric) */
    int    adc_bits;                 /* ADC resolution (10-12 bit typical) */
    double clamp_voltage_min_v;      /* Output clamp for diagnostics */
    double clamp_voltage_max_v;
    double pullup_resistor_ohm;      /* Diagnostic pull-up */
    double pulldown_resistor_ohm;    /* Diagnostic pull-down */
} automotive_pressure_t;

/**
 * Compute manifold air pressure with diagnostic checks.
 *
 * Returns pressure and additionally checks for:
 * - Open circuit (Vout near 0 or Vcc)
 * - Short to ground (Vout < 2% Vcc)
 * - Short to battery (Vout > 98% Vcc)
 * - Out-of-range (P > overpressure limit)
 *
 * @param sensor        Automotive pressure sensor config
 * @param adc_reading   Raw ADC reading [counts]
 * @param temp_c        Temperature [degC]
 * @param fault_code    Output: fault code (0=ok, 1=open, 2=short_GND, 3=short_Vbat, 4=overpressure)
 * @return              Pressure [kPa] (NaN if fault)
 *
 * Complexity: O(1).
 */
double application_automotive_map(const automotive_pressure_t *sensor,
                                  int adc_reading, double temp_c,
                                  int *fault_code);

/**
 * Aerospace structural load monitoring (Boeing/Airbus approach)
 *
 * Strain gauges bonded to critical airframe structures for:
 * - Fatigue life tracking (safe-life design)
 * - Load spectrum monitoring
 * - Overload detection and recording
 * - Structural health monitoring (SHM)
 *
 * Key parameters:
 * - Long-term stability: < 0.1% drift/year
 * - Temperature range: -54 to +71 degC (MIL-810)
 * - Vibration: 20 g RMS random (DO-160)
 * - EMI: MIL-STD-461 (high-intensity radiated fields)
 */
typedef struct {
    bridge_sensor_t sensor_installation;
    double fatigue_notch_factor;     /* K_t for stress concentration */
    double max_recorded_strain_ue;   /* Peak recorded strain */
    double cumulative_damage;        /* Miner's rule damage sum */
    int    n_cycles_recorded;        /* Total recorded load cycles */
    double install_date;             /* Installation date (Julian) */
    double last_cal_date;            /* Last calibration date */
} aerospace_shm_t;

/**
 * Update fatigue damage using rainflow-counted strain cycles.
 *
 * Miner's linear damage accumulation rule:
 *   D = sum(n_i / N_i)
 * where n_i = cycles at strain level i,
 *       N_i = cycles to failure at strain level i.
 *
 * Failure predicted when D >= 1.0.
 *
 * S-N curve (Basquin's law):
 *   N_f = N_ref * (eps_ref / eps)^m
 * where m = fatigue exponent (~3-10 depending on material)
 *
 * @param shm           SHM installation record
 * @param strain_amp_ue Strain amplitude this cycle [microstrain]
 * @param n_cycles      Number of cycles at this level
 * @param fatigue_exp_m Fatigue exponent m (Basquin's law)
 * @param ref_strain_ue Reference strain for S-N curve [microstrain]
 * @param ref_cycles    Reference cycles N_ref at ref_strain
 *
 * Complexity: O(1).
 */
void application_aerospace_fatigue(aerospace_shm_t *shm,
                                   double strain_amp_ue, int n_cycles,
                                   double fatigue_exp_m,
                                   double ref_strain_ue, double ref_cycles);

/* ========================================================================
 * L8: Advanced Topics — Wireless and IoT Bridge Sensors
 * ======================================================================== */

/**
 * Wireless strain sensor node configuration
 *
 * Combines bridge sensor, signal conditioning, microcontroller,
 * and wireless transceiver (BLE, LoRa, or ZigBee) in one node.
 */
typedef struct {
    bridge_sensor_t sensor;
    signal_chain_t signal_chain;
    double transmit_interval_s;      /* Reporting interval [s] */
    double battery_capacity_mah;     /* Battery capacity [mAh] */
    double sleep_current_ua;         /* Sleep mode current [uA] */
    double active_current_ma;        /* Active mode current [mA] */
    double measurement_time_ms;      /* Measurement + processing [ms] */
    double tx_time_ms;               /* Transmission time [ms] */
    double tx_current_ma;            /* Transmit current [mA] */
    int    data_points_per_tx;       /* Samples per transmission */
} wireless_sensor_node_t;

/**
 * Estimate battery life for wireless sensor node.
 *
 * Average current:
 *   I_avg = (I_sleep*T_sleep + I_active*T_meas + I_tx*T_tx) / T_interval
 *
 * Battery life:
 *   Life_hours = Capacity_mAh / I_avg_mA
 *
 * Example: 2000 mAh battery, 1 minute transmit interval
 *   I_sleep=5uA, I_active=2mA for 10ms, I_tx=15mA for 5ms
 *   I_avg = (5uA*59985ms + 2mA*10ms + 15mA*5ms)/60000ms
 *         = (0.3 + 0.02 + 0.075) / 60 = 0.00658 mA
 *   Life = 2000/0.00658 = 303,839 hours ≈ 34.7 years
 *   (In practice, limited by battery self-discharge ~ 5-10 years)
 *
 * @param node          Wireless node configuration
 * @return              Estimated battery life [hours]
 *
 * Complexity: O(1).
 */
double application_wireless_battery_life(const wireless_sensor_node_t *node);

/**
 * Edge processing: on-sensor digital filtering and event detection.
 *
 * Instead of streaming all raw data, the wireless sensor node
 * performs local processing:
 * 1. Moving average filtering (noise reduction)
 * 2. Threshold-based event detection (wake-up)
 * 3. Delta-sigma data compression (transmit only changes)
 * 4. Rainflow counting for fatigue (histogram transmission)
 *
 * This reduces wireless data by 10-100x, dramatically extending
 * battery life for long-term structural monitoring.
 *
 * @param raw_sample     Latest raw ADC sample [microstrain]
 * @param prev_reported  Last reported value [microstrain]
 * @param threshold_ue   Reporting threshold [microstrain]
 * @param should_report  Output: 1 if this sample triggers report
 * @return               Processed sample value [microstrain]
 *
 * Complexity: O(1).
 */
double application_edge_process(double raw_sample, double prev_reported,
                                double threshold_ue, int *should_report);

/* ========================================================================
 * Initialization utilities
 * ======================================================================== */

void loadcell_spec_init(loadcell_spec_t *lc, loadcell_type_t type);
void pressure_sensor_spec_init(pressure_sensor_spec_t *ps, pressure_type_t type);
void torque_sensor_spec_init(torque_sensor_spec_t *ts);
void weighing_system_init(weighing_system_t *ws);
void automotive_pressure_init(automotive_pressure_t *ap);
void aerospace_shm_init(aerospace_shm_t *shm);
void wireless_sensor_node_init(wireless_sensor_node_t *node);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_APPLICATIONS_H */
