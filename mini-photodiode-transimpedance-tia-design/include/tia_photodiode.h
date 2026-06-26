/**
 * @file    tia_photodiode.h
 * @brief   Photodiode Physics & Characterization ? L1+L3+L4
 *
 * @details Photodiode semiconductor physics, optical-to-electrical
 *          conversion, and device characterization for TIA frontend
 *          design.
 *
 * Knowledge Mapping:
 *   L1 - Definitions:
 *     - Absorption coefficient alpha(lambda) for Si/Ge/InGaAs
 *     - Quantum efficiency eta = (I_photo/q) / (P_opt/h*nu)
 *     - Responsivity R = eta * q * lambda / (h * c)
 *     - Depletion width W_dep = sqrt(2*epsilon*V_bi/q*N_d)
 *     - Diffusion length L_n, L_p
 *     - Transit time and RC time constant
 *   L3 - Mathematical Structures:
 *     - Beer-Lambert law for optical absorption
 *     - Poisson equation for junction electrostatics
 *     - Continuity equations for carrier transport
 *   L4 - Fundamental Laws:
 *     - Photoelectric effect: E_photon = h*nu >= E_gap
 *     - Planck relation: E = h*c/lambda
 *     - Einstein relation: D/mu = kT/q
 *     - Shockley-Ramo theorem for induced current
 *
 * References:
 *   - Sze & Ng, "Physics of Semiconductor Devices" (3rd ed, 2007)
 *   - Hamamatsu, "Photodiode Technical Information" (SD-12)
 *   - OSI Optoelectronics, "Photodiode Characteristics"
 */

#ifndef TIA_PHOTODIODE_H
#define TIA_PHOTODIODE_H

#include "tia_core.h"

/* ??? L1: Material Properties ?????????????????????????????????????????????? */

/**
 * @brief Semiconductor material for photodetection.
 */
typedef enum {
    MATERIAL_SILICON    = 0,
    MATERIAL_INGAAS     = 1,
    MATERIAL_GERMANIUM  = 2,
    MATERIAL_INSB       = 3,
    MATERIAL_HGCDTE     = 4,
    MATERIAL_GAN        = 5
} semiconductor_material_t;

/**
 * @brief Material parameters for photodiode physics.
 */
typedef struct {
    semiconductor_material_t material;
    double bandgap_energy_ev;
    double intrinsic_carrier_ni;
    double electron_mobility;
    double hole_mobility;
    double relative_permittivity;
    double electron_diffusion_length_um;
    double hole_diffusion_length_um;
    double surface_recombination_velocity;
    double absorption_coefficient_cm;
    double refractive_index;
} semiconductor_params_t;

/* ??? L1: Optical Absorption ??????????????????????????????????????????????? */

/**
 * @brief Absorption spectrum for a semiconductor material.
 */
typedef struct {
    size_t num_points;
    double *wavelength_nm;
    double *absorption_coefficient;
    double *quantum_efficiency_ideal;
    double *responsivity_ideal;
    double cutoff_wavelength_nm;
} absorption_spectrum_t;

/* ??? L3: Junction Physics ????????????????????????????????????????????????? */

/**
 * @brief PN junction electrostatics model.
 */
typedef struct {
    double built_in_potential_v;
    double depletion_width_um;
    double zero_bias_capacitance_pf;
    double junction_area_mm2;
    double doping_na_cm3;
    double doping_nd_cm3;
    double grading_coefficient;
    double electric_field_max_v_per_cm;
} pn_junction_model_t;

/* ??? L2: Photodiode Bandwidth ????????????????????????????????????????????? */

/**
 * @brief Photodiode bandwidth limiting mechanisms.
 */
typedef struct {
    double transit_time_limited_bw_ghz;
    double rc_time_limited_bw_ghz;
    double diffusion_limited_bw_ghz;
    double total_bandwidth_ghz;
    double dominant_mechanism;
    double carrier_transit_time_ps;
    double collection_efficiency;
} photodiode_bandwidth_t;

/* ??? L2: APD Model ???????????????????????????????????????????????????????? */

/**
 * @brief Avalanche photodiode model.
 */
typedef struct {
    double multiplication_factor_m;
    double breakdown_voltage_v;
    double k_factor;
    double excess_noise_factor_f;
    double ionization_coefficient_ratio;
    double optimal_gain;
    double dark_current_multiplied_na;
} apd_model_t;

/* ??? Function Declarations ? Photodiode Physics ??????????????????????????? */

/**
 * @brief  Compute photon energy from wavelength.
 * @param  wavelength_nm  Wavelength in nanometers
 * @return                Photon energy in electron-volts (eV)
 *
 * @note   E = h*c/lambda
 *        h = 4.135667696e-15 eV*s
 *        c = 2.99792458e8 m/s
 *        E(eV) ? 1239.84 / lambda(nm)
 */
double  photon_energy_ev(double wavelength_nm);

/**
 * @brief  Compute ideal responsivity for 100% quantum efficiency.
 * @param  wavelength_nm  Wavelength (nm)
 * @return                Responsivity in A/W
 *
 * @note   R_ideal = q*lambda/(h*c) ? lambda(nm) / 1239.84
 *         850nm -> 0.686 A/W, 1550nm -> 1.25 A/W
 */
double  responsivity_ideal(double wavelength_nm);

/**
 * @brief  Compute quantum efficiency from responsivity.
 * @param  responsivity_a_per_w  Measured responsivity
 * @param  wavelength_nm          Wavelength (nm)
 * @return                        Quantum efficiency (0-1)
 *
 * @note   eta = R * h*c / (q*lambda) = R * 1239.84 / lambda(nm)
 */
double  quantum_efficiency_from_r(double responsivity_a_per_w,
                                   double wavelength_nm);

/**
 * @brief  Compute absorption coefficient for Si at given wavelength.
 * @param  wavelength_nm  Wavelength (nm) in range 400-1100
 * @return                Absorption coefficient (cm^-1)
 *
 * @note   Empirical fit to Silicon absorption data from Green & Keevers (1995).
 *         alpha increases sharply below 400nm (direct gap) and above 1000nm (band tail).
 */
double  silicon_absorption(double wavelength_nm);

/**
 * @brief  Compute absorption coefficient for InGaAs at given wavelength.
 * @param  wavelength_nm  Wavelength (nm) in range 900-1700
 * @return                Absorption coefficient (cm^-1)
 */
double  ingaas_absorption(double wavelength_nm);

/**
 * @brief  Compute penetration depth (1/alpha) in semiconductor.
 * @param  wavelength_nm  Wavelength (nm)
 * @param  material       Semiconductor material
 * @return                Penetration depth (um)
 *
 * @note   delta = 1/alpha ? the depth at which intensity drops to 1/e
 *         Critical for photodiode design: depletion region must be
 *         deeper than penetration depth for high efficiency.
 */
double  penetration_depth(double wavelength_nm, semiconductor_material_t material);

/**
 * @brief  Compute semiconductor material parameters.
 * @param  material  Material type
 * @return           Material parameters at 300K
 */
semiconductor_params_t  semiconductor_params_get(semiconductor_material_t material);

/**
 * @brief  Compute PN junction parameters from doping and geometry.
 * @param  material      Semiconductor material
 * @param  na_cm3        Acceptor concentration (cm^-3)
 * @param  nd_cm3        Donor concentration (cm^-3)
 * @param  area_mm2      Junction area (mm^2)
 * @param  reverse_bias  Reverse bias voltage (V)
 * @return               Junction electrostatic model
 */
pn_junction_model_t  pn_junction_compute(semiconductor_material_t material,
                                           double na_cm3, double nd_cm3,
                                           double area_mm2, double reverse_bias);

/**
 * @brief  Compute junction capacitance model from bias and doping.
 * @param  junction     Junction model
 * @param  bias_voltage Applied reverse bias (V)
 * @return              Junction capacitance (pF)
 */
double  junction_capacitance(const pn_junction_model_t *junction,
                              double bias_voltage);

/**
 * @brief  Compute photodiode bandwidth limitations.
 * @param  pd          Photodiode model
 * @param  load_ohm    Load resistance (typically R_f or 50 ohm)
 * @return             Bandwidth analysis
 */
photodiode_bandwidth_t  photodiode_bandwidth_analyze(const photodiode_model_t *pd,
                                                       double load_ohm);

/**
 * @brief  Compute Avalanche photodiode model.
 * @param  pd         Base photodiode (PIN)
 * @param  m_gain     Multiplication factor (1 for PIN)
 * @param  k_ion      Ionization coefficient ratio k = alpha_n/alpha_p
 * @return            APD model
 */
apd_model_t  apd_model_compute(const photodiode_model_t *pd,
                                double m_gain, double k_ion);

/**
 * @brief  Compute excess noise factor for APD.
 * @param  m_gain  Multiplication factor
 * @param  k       Ionization coefficient ratio
 * @return         Excess noise factor F(M)
 *
 * @note   McIntyre formula: F = M * [1 - (1-k)*(M-1)^2/M^2]
 *         For electron injection. k=0 -> F=2 (ideal)
 *         For Si: k?0.02-0.05, InGaAs: k?0.5-0.7
 */
double  apd_excess_noise_factor(double m_gain, double k);

/**
 * @brief  Compute absorption spectrum for a given material.
 * @param  material   Semiconductor material
 * @param   wl_start   Start wavelength (nm)
 * @param   wl_stop    Stop wavelength (nm)
 * @param   points     Number of points
 * @return             Absorption spectrum
 */
absorption_spectrum_t  absorption_spectrum_compute(semiconductor_material_t material,
                                                     double wl_start, double wl_stop,
                                                     size_t points);

/**
 * @brief  Compute the cutoff wavelength for a semiconductor.
 * @param  material  Semiconductor material
 * @return           Cutoff wavelength (nm), lambda_c = h*c/E_gap
 */
double  cutoff_wavelength(semiconductor_material_t material);

/**
 * @brief  Estimate photodiode response time from device parameters.
 * @param  pd  Photodiode model
 * @return     10-90% rise time (ns)
 *
 * @note   t_rise ? 2.2 * (t_transit + t_RC)
 *         t_transit = W_dep / v_sat (~10 ps/um for Si at high field)
 *         t_RC = 2.2 * R_load * (C_j + C_stray)
 */
double  photodiode_rise_time_estimate(const photodiode_model_t *pd);

/**
 * @brief  Free absorption spectrum data.
 */
void  absorption_spectrum_free(absorption_spectrum_t *as);

#endif /* TIA_PHOTODIODE_H */
