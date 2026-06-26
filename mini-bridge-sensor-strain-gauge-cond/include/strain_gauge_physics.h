/**
 * strain_gauge_physics.h — Strain Gauge Physics and Material Science
 *
 * Knowledge Coverage:
 *   L1: Piezoresistive effect, gauge factor decomposition,
 *       strain sensitivity, transverse sensitivity
 *   L2: Self-temperature-compensation (STC), creep, hysteresis
 *   L3: 2D strain transformation, Mohr's circle
 *   L4: Piezoresistance tensor for cubic crystals (Smith 1954)
 *   L5: Gauge bonding stress analysis, residual stress
 *
 * Reference:
 *   - Smith, "Piezoresistance Effect in Ge and Si", Phys Rev 1954
 *   - Bridgman, "The Physics of High Pressure", 1931
 *   - Thurston, "Use of Semiconductors as Strain Gauges", 1953
 *   - Dally & Riley, "Experimental Stress Analysis", 4th ed.
 *
 * Course: MIT 6.630, Berkeley EE117, Georgia Tech ECE 6350
 */

#ifndef STRAIN_GAUGE_PHYSICS_H
#define STRAIN_GAUGE_PHYSICS_H

#include "bridge_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * L1: Piezoresistive Fundamentals
 * ======================================================================== */

/**
 * Decompose gauge factor into dimensional and resistivity components.
 *
 * The total gauge factor is the sum of:
 *   GF = (1 + 2*nu) + (dRho/Rho) / epsilon
 *      = GF_dimensional + GF_resistivity
 *
 * where:
 *   GF_dimensional = 1 + 2*nu (from geometry change)
 *   GF_resistivity = pi_11 * E (from resistivity change under stress)
 *
 * For constantan: GF=2.05, GF_dim=1.66, GF_rho=0.39
 * For silicon (n-type, <100>): GF=-132, GF_dim=1.56, GF_rho=-133.6
 *   → resistivity effect dominates and is NEGATIVE
 *
 * @param nu               Poisson's ratio
 * @param piezo_coeff      Piezoresistive coefficient [1/GPa]
 * @param E_gpa            Young's modulus [GPa]
 * @param gf_dimensional   Output: dimensional contribution
 * @param gf_resistivity   Output: resistivity contribution
 *
 * Complexity: O(1).
 */
void strain_gauge_gf_decompose(double nu, double piezo_coeff, double E_gpa,
                               double *gf_dimensional,
                               double *gf_resistivity);

/**
 * Piezoresistive coefficients for cubic crystals.
 *
 * In cubic crystals (Si, Ge, diamond structure), the piezoresistive
 * effect is described by three independent coefficients:
 *   pi_11 — longitudinal piezoresistance (stress ∥ current ∥ <100>)
 *   pi_12 — transverse piezoresistance (stress ⊥ current, both <100>)
 *   pi_44 — shear piezoresistance
 *
 * For arbitrary crystal direction, the effective piezoresistive
 * coefficient is:
 *
 *   pi_l = pi_11 - 2*(pi_11 - pi_12 - pi_44)*(l1^2*m1^2 + m1^2*n1^2 + n1^2*l1^2)
 *
 * where (l1,m1,n1) are direction cosines of current flow relative
 * to crystal axes.
 *
 * Silicon (n-type, 11.7 ohm-cm, Smith 1954):
 *   pi_11 = -102.2e-11 Pa^-1
 *   pi_12 = +53.4e-11 Pa^-1
 *   pi_44 = -13.6e-11 Pa^-1
 *
 * Silicon (p-type, 7.8 ohm-cm, Smith 1954):
 *   pi_11 =  +6.6e-11 Pa^-1
 *   pi_12 =  -1.1e-11 Pa^-1
 *   pi_44 = +138.1e-11 Pa^-1
 *
 * @param pi_11, pi_12, pi_44  Fundamental piezoresistive coefficients [1/Pa]
 * @param l1, m1, n1           Current direction cosines in crystal frame
 * @return                     Effective longitudinal piezoresistive coeff [1/Pa]
 *
 * Complexity: O(1).
 * Reference: Smith, Phys Rev 94(1), 1954 — the seminal paper
 */
double strain_gauge_piezo_longitudinal(double pi_11, double pi_12, double pi_44,
                                       double l1, double m1, double n1);

/**
 * Compute effective gauge factor for arbitrary crystal orientation.
 *
 * GF(hkl) = 1 + 2*nu + pi_l(hkl) * E
 *
 * This explains why semiconductor strain gauges are highly
 * anisotropic — the gauge factor depends strongly on the
 * crystal orientation of the current path.
 *
 * For maximum sensitivity in n-type Si: align current with <100>
 * For maximum sensitivity in p-type Si: align current with <111>
 *
 * @param pi_11, pi_12, pi_44  Piezoresistive coefficients [1/Pa]
 * @param l1, m1, n1           Direction cosines
 * @param nu                   Poisson's ratio
 * @param E_gpa                Young's modulus [GPa]
 * @return                     Effective gauge factor
 *
 * Complexity: O(1).
 */
double strain_gauge_gf_anisotropic(double pi_11, double pi_12, double pi_44,
                                   double l1, double m1, double n1,
                                   double nu, double E_gpa);

/**
 * Standard material database entry.
 * Provides pre-measured properties for common engineering materials.
 */
typedef struct {
    gauge_material_t material;
    const char *gauge_alloy;         /* Recommended gauge alloy */
    const char *stc_code;            /* Self-temp-compensation code */
    double adhesive_type;            /* Recommended adhesive index */
    double surface_prep_roughness_ra; /* Recommended surface finish [um Ra] */
} material_database_entry_t;

/**
 * Look up material properties from built-in database.
 *
 * @param name     Material name (e.g., "steel-1018", "aluminum-6061")
 * @param entry    Output: material database entry
 * @return         0 if found, -1 if not in database
 *
 * Complexity: O(log N) via binary search on sorted database.
 */
int strain_gauge_material_lookup(const char *name,
                                 material_database_entry_t *entry);

/* ========================================================================
 * L2: Temperature Compensation and Self-Heating
 * ======================================================================== */

/**
 * Design self-temperature-compensation (STC) for given specimen.
 *
 * STC gauge selection matches the gauge's apparent strain curve
 * to the specimen's thermal expansion, minimizing thermal output.
 *
 * The STC number indicates the CTE (in ppm/F) that the gauge is
 * compensated for:
 *   STC-03: alpha = 3.3 ppm/F (5.9 ppm/K) — quartz, invar
 *   STC-06: alpha = 6.1 ppm/F (11.0 ppm/K) — carbon steel
 *   STC-09: alpha = 9.4 ppm/F (16.9 ppm/K) — stainless steel
 *   STC-13: alpha = 12.8 ppm/F (23.0 ppm/K) — aluminum
 *   STC-00: alpha = 1.0 ppm/F — no compensation
 *
 * The residual thermal output after STC selection is typically
 * < 1 ue/K for metallic gauges on matched materials.
 *
 * @param specimen_cte_ppm_per_c  Specimen CTE [ppm/K]
 * @param temperature_range_c     Operating temperature range [K]
 * @return                        Recommended STC number
 *
 * Complexity: O(1).
 */
double strain_gauge_select_stc(double specimen_cte_ppm_per_c,
                               double temperature_range_c);

/**
 * Compute self-heating temperature rise in a strain gauge.
 *
 * When excitation current flows through the gauge, Joule heating
 * raises the gauge temperature above ambient:
 *
 *   dT = P_gauge / (k * A / L) = P_gauge * R_thermal
 *
 * where:
 *   P_gauge = power dissipated [W]
 *   k = thermal conductivity of specimen [W/(m*K)]
 *   A = gauge area [m^2]
 *   L = characteristic length [m]
 *   R_thermal = thermal resistance [K/W]
 *
 * Typical R_thermal values:
 *   - Steel specimen: 10-50 K/W (good heat sink)
 *   - Aluminum specimen: 5-20 K/W (excellent heat sink)
 *   - Plastic specimen: 200-1000 K/W (poor heat sink → severe heating)
 *
 * The apparent strain from self-heating:
 *   eps_heating = dT * (alpha_specimen + thermal_output_coeff)
 *
 * @param power_w             Gauge power dissipation [W]
 * @param thermal_conductivity Specimen thermal conductivity [W/(m*K)]
 * @param gauge_area_m2       Gauge grid area [m^2]
 * @param backing_thickness_m Gauge backing thickness [m]
 * @return                    Temperature rise [K]
 *
 * Complexity: O(1).
 */
double strain_gauge_self_heating(double power_w,
                                 double thermal_conductivity,
                                 double gauge_area_m2,
                                 double backing_thickness_m);

/**
 * Compute creep error for metallic strain gauges.
 *
 * Creep is the time-dependent change in gauge output under
 * constant strain. It is caused by viscoelastic relaxation
 * in the gauge backing and adhesive.
 *
 * Empirical model (logarithmic creep):
 *   eps_creep(t) = eps_0 * C * log10(1 + t/t0)
 *
 * where:
 *   C = creep coefficient (typically 0.0005-0.005 for epoxy)
 *   t = time under load [hours]
 *   t0 = reference time (typically 1 hour)
 *
 * Creep is specified as %FS change over a standard period
 * (typically 30 minutes or 1 hour at room temperature).
 *
 * @param strain_ue          Applied strain [microstrain]
 * @param creep_coefficient  Material creep coefficient [dimensionless]
 * @param time_hours         Time under continuous load [hours]
 * @return                   Creep strain [microstrain]
 *
 * Complexity: O(1).
 */
double strain_gauge_creep(double strain_ue, double creep_coefficient,
                          double time_hours);

/**
 * Compute hysteresis error for strain gauge installation.
 *
 * Hysteresis is the difference between loading and unloading
 * output at the same strain level. It arises from:
 * 1. Adhesive viscoelasticity
 * 2. Gauge backing plasticity
 * 3. Micro-slip at bond interface
 *
 * Typical hysteresis for properly installed gauges: < 0.05% FS.
 * Can reach 0.5% FS for poor installations or high strain levels.
 *
 * @param max_strain_ue      Peak strain during cycle [microstrain]
 * @param hysteresis_factor  Installation quality factor (0.001-0.02)
 * @param num_cycles         Number of load cycles
 * @return                   Hysteresis error [microstrain]
 *
 * Complexity: O(1).
 */
double strain_gauge_hysteresis(double max_strain_ue,
                               double hysteresis_factor,
                               int num_cycles);

/* ========================================================================
 * L3: 2D Strain Analysis — Mohr's Circle and Transformations
 * ======================================================================== */

/**
 * Strain transformation under coordinate rotation.
 *
 * Given strain components (eps_x, eps_y, gamma_xy) in the x-y frame,
 * compute strains in a frame rotated by theta degrees:
 *
 *   eps_x' = (eps_x+eps_y)/2 + (eps_x-eps_y)/2 * cos(2theta) + gamma_xy/2 * sin(2theta)
 *   eps_y' = (eps_x+eps_y)/2 - (eps_x-eps_y)/2 * cos(2theta) - gamma_xy/2 * sin(2theta)
 *   gamma_xy'/2 = -(eps_x-eps_y)/2 * sin(2theta) + gamma_xy/2 * cos(2theta)
 *
 * Mohr's circle for strain: the locus of (eps_n, gamma_n/2) for all
 * orientations is a circle with:
 *   Center: (eps_x+eps_y)/2
 *   Radius: sqrt[((eps_x-eps_y)/2)^2 + (gamma_xy/2)^2] = gamma_max/2
 *
 * @param src       Source strain state in x-y frame
 * @param theta_deg Rotation angle [degrees]
 * @param dest      Transformed strain state in x'-y' frame
 *
 * Complexity: O(1).
 * Reference: Timoshenko & Goodier, "Theory of Elasticity", Ch.2
 */
void strain_gauge_transform(const strain_state_t *src, double theta_deg,
                            strain_state_t *dest);

/**
 * Compute Mohr's circle parameters for strain state.
 *
 * Returns:
 *   center = (eps_x + eps_y) / 2
 *   radius = sqrt[((eps_x-eps_y)/2)^2 + (gamma_xy/2)^2]
 *   eps_1 = center + radius  (major principal strain)
 *   eps_2 = center - radius  (minor principal strain)
 *   theta_p = 0.5 * atan2(gamma_xy, eps_x-eps_y) * 180/PI (principal angle)
 *   gamma_max = 2 * radius  (maximum engineering shear strain)
 *
 * @param strain            Input strain state
 * @param center            Output: center of Mohr's circle [microstrain]
 * @param radius            Output: radius of Mohr's circle [microstrain]
 *
 * Complexity: O(1).
 */
void strain_gauge_mohr_circle(const strain_state_t *strain,
                              double *center, double *radius);

/* ========================================================================
 * L4: Semiconductor Strain Gauge Physics (Advanced)
 * ======================================================================== */

/**
 * Compute the full piezoresistance tensor for silicon.
 *
 * For cubic crystals, the piezoresistance tensor has only 3
 * independent components (pi_11, pi_12, pi_44) when expressed
 * in the <100> crystal coordinate system.
 *
 * The gauge factor in an arbitrary direction is:
 *   GF = 1 + 2*nu + E * [pi_11*l1^2 + pi_12*(m1^2+n1^2)
 *       - 2*(pi_11-pi_12-pi_44)*(l1^2*m1^2 + m1^2*n1^2 + n1^2*l1^2)]
 *
 * @param doping_type      'n' or 'p' for n-type or p-type silicon
 * @param resistivity_ohm_cm  Resistivity [ohm*cm]
 * @param pi_11_out        Output: pi_11 [1/Pa]
 * @param pi_12_out        Output: pi_12 [1/Pa]
 * @param pi_44_out        Output: pi_44 [1/Pa]
 *
 * Complexity: O(1).
 * Reference: Kanda, "Piezoresistance Effect of Silicon",
 *            Sensors and Actuators A, 28 (1991) 83-91
 */
void strain_gauge_silicon_piezo_coefficients(char doping_type,
                                             double resistivity_ohm_cm,
                                             double *pi_11_out,
                                             double *pi_12_out,
                                             double *pi_44_out);

/**
 * Compute gauge factor for diffused silicon piezoresistor.
 *
 * Diffused (or ion-implanted) piezoresistors in silicon are the
 * basis for MEMS pressure sensors and accelerometers. Unlike
 * bonded metal foil gauges, they are fabricated directly in the
 * silicon substrate for monolithic integration.
 *
 * Key advantages:
 * - GF 20-100× larger than metal foil
 * - No adhesive hysteresis or creep
 * - Direct integration with CMOS circuitry
 * - Smaller size (can be < 10 um)
 *
 * Key disadvantages:
 * - Strong temperature dependence (TCR ~ 1000-3000 ppm/K)
 * - Nonlinear (requires correction above ~100 ue)
 * - Leakage current at elevated temperature
 * - Fabrication requires semiconductor processing
 *
 * @param doping_conc_cm3    Doping concentration [atoms/cm^3]
 * @param temperature_k      Operating temperature [K]
 * @param orientation        Crystal orientation (0=<100>, 1=<110>, 2=<111>)
 * @return                   Approximate gauge factor
 *
 * Complexity: O(1).
 */
double strain_gauge_silicon_gf(double doping_conc_cm3,
                               double temperature_k,
                               int orientation);

/**
 * Compute temperature coefficient of resistivity for doped silicon.
 *
 * The TCR of silicon is strongly doping-dependent:
 * - Lightly doped (10^14): TCR ~ 7000 ppm/K (like intrinsic)
 * - Heavily doped (10^20): TCR ~ 500 ppm/K (metallic behavior)
 *
 * This is why MEMS piezoresistors use heavy doping —
 * to reduce temperature sensitivity at the cost of lower GF.
 *
 * @param doping_conc_cm3  Doping concentration [atoms/cm^3]
 * @param temperature_k    Temperature [K]
 * @return                 TCR [ppm/K]
 *
 * Complexity: O(1).
 */
double strain_gauge_silicon_tcr(double doping_conc_cm3, double temperature_k);

/* ========================================================================
 * Gauge installation analysis
 * ======================================================================== */

/**
 * Compute stress concentration factor for gauge location.
 *
 * Strain gauges measure average strain over their active length.
 * Near stress concentrations (holes, fillets, notches), the
 * local strain can be much higher than nominal.
 *
 * The measured strain = K_t * epsilon_nominal
 * where K_t is the theoretical stress concentration factor.
 *
 * For a circular hole in an infinite plate under uniaxial tension:
 *   K_t = 3.0 (the classic Kirsch solution, 1898)
 *
 * For a U-notch in a flat bar:
 *   K_t = 1 + 2*sqrt(D/r) where D=depth, r=notch radius
 *
 * @param geometry_type    0=hole, 1=notch, 2=fillet, 3=groove
 * @param dim1, dim2       Geometry-specific dimensions
 * @return                 Stress concentration factor K_t
 *
 * Complexity: O(1).
 */
double strain_gauge_stress_concentration(int geometry_type,
                                         double dim1, double dim2);

/**
 * Calculate gauge misalignment error.
 *
 * If a strain gauge is installed at angle delta from the intended
 * measurement axis, the measured strain is:
 *
 *   eps_measured = eps_x * cos^2(delta) + eps_y * sin^2(delta)
 *                 + gamma_xy * sin(delta) * cos(delta)
 *
 * For uniaxial stress with principal strain eps_1 along x and
 * -nu*eps_1 along y (Poisson contraction):
 *
 *   eps_measured = eps_1 * [cos^2(delta) - nu * sin^2(delta)]
 *
 * Misalignment error:
 *   error(%) = (eps_measured - eps_1) / eps_1 * 100
 *
 * Example: delta=2 deg, nu=0.3, uniaxial:
 *   eps_meas/eps_1 = cos^2(2) - 0.3*sin^2(2) = 0.9994 - 0.0004 = 0.9990
 *   Error = -0.1% (surprisingly tolerant to small misalignment!)
 *
 * @param eps_principal  Principal strain magnitude [microstrain]
 * @param nu             Poisson's ratio
 * @param delta_deg      Misalignment angle [degrees]
 * @return               Relative error [dimensionless, -1 to 1]
 *
 * Complexity: O(1).
 */
double strain_gauge_misalignment_error(double eps_principal, double nu,
                                       double delta_deg);

#ifdef __cplusplus
}
#endif

#endif /* STRAIN_GAUGE_PHYSICS_H */
