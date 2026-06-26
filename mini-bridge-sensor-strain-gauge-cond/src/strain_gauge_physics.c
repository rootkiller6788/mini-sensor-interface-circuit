/**
 * strain_gauge_physics.c — Strain Gauge Physics Implementation
 *
 * Knowledge Coverage:
 *   L1: Piezoresistive effect decomposition, gauge factor
 *   L2: Self-temperature-compensation, self-heating, creep, hysteresis
 *   L3: Strain transformation (Mohr's circle), coordinate rotation
 *   L4: Piezoresistance tensor for cubic crystals (Smith 1954)
 *   L5: Semiconductor strain gauge analysis
 *
 * Reference:
 *   - Smith, "Piezoresistance Effect in Ge and Si", Phys Rev 94(1), 1954
 *   - Kanda, "Piezoresistance Effect of Silicon", Sens Act A 28, 1991
 *   - Dally & Riley, "Experimental Stress Analysis", 4th ed.
 *   - Timoshenko & Goodier, "Theory of Elasticity", 3rd ed.
 */

#include "strain_gauge_physics.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ========================================================================
 * L1: Piezoresistive Fundamentals
 * ======================================================================== */

void strain_gauge_gf_decompose(double nu, double piezo_coeff, double E_gpa,
                               double *gf_dimensional,
                               double *gf_resistivity)
{
    /* Decompose gauge factor into dimensional and resistivity parts.
     *
     * GF = GF_dimensional + GF_resistivity
     *    = (1 + 2*nu) + (pi_11 * E)
     *
     * For metallic gauges: GF_dimensional dominates (~80%)
     * For semiconductor gauges: GF_resistivity dominates (>99%)
     *
     * This decomposition reveals WHY semiconductor gauges have
     * such large gauge factors: the piezoresistive effect in
     * silicon is orders of magnitude larger than in metals.
     */

    *gf_dimensional = 1.0 + 2.0 * nu;
    *gf_resistivity = piezo_coeff * E_gpa;
}

double strain_gauge_piezo_longitudinal(double pi_11, double pi_12, double pi_44,
                                       double l1, double m1, double n1)
{
    /* Effective longitudinal piezoresistive coefficient for
     * arbitrary crystal direction in cubic crystals.
     *
     * The piezoresistance tensor for cubic symmetry (m3m point group)
     * has only 3 independent components: pi_11, pi_12, pi_44.
     *
     * For a current flowing in direction (l1, m1, n1) relative to
     * the <100> crystal axes, the longitudinal piezoresistive
     * coefficient is:
     *
     *   pi_l = pi_11 - 2*(pi_11 - pi_12 - pi_44) *
     *          (l1^2*m1^2 + m1^2*n1^2 + n1^2*l1^2)
     *
     * The direction cosines satisfy: l1^2 + m1^2 + n1^2 = 1.
     *
     * For <100> direction (l1=1, m1=n1=0):
     *   pi_l = pi_11 - 2*(pi_11-pi_12-pi_44)*0 = pi_11 ✓
     *
     * For <110> direction (l1=m1=1/sqrt(2), n1=0):
     *   l1^2*m1^2 + m1^2*n1^2 + n1^2*l1^2 = 1/4 + 0 + 0 = 1/4
     *   pi_l = pi_11 - 2*(pi_11-pi_12-pi_44)*(1/4)
     *        = pi_11 - (pi_11-pi_12-pi_44)/2
     *        = (pi_11 + pi_12 + pi_44)/2
     *
     * For <111> direction (l1=m1=n1=1/sqrt(3)):
     *   l1^2*m1^2 + ... = 3*(1/9) = 1/3
     *   pi_l = pi_11 - 2*(pi_11-pi_12-pi_44)*(1/3)
     *        = pi_11 - 2/3*(pi_11-pi_12-pi_44)
     *        = (pi_11 + 2*pi_12 + 2*pi_44)/3
     *
     * For n-type Si: pi_11=-102, pi_12=+53, pi_44=-14 (all ×10^-11 Pa^-1)
     *   <100>: pi_l = -102
     *   <110>: pi_l = (-102 + 53 - 14)/2 = -31.5
     *   <111>: pi_l = (-102 + 106 - 28)/3 = -8
     *   → Maximum sensitivity along <100> for n-type Si
     *
     * For p-type Si: pi_11=+6.6, pi_12=-1.1, pi_44=+138
     *   <100>: pi_l = 6.6  (very small!)
     *   <110>: pi_l = (6.6 - 1.1 + 138)/2 = 71.8
     *   <111>: pi_l = (6.6 - 2.2 + 276)/3 = 93.5
     *   → Maximum sensitivity along <111> for p-type Si
     *
     * This anisotropy is critical for MEMS sensor design —
     * piezoresistors must be aligned with the crystal axes
     * for maximum sensitivity.
     *
     * Reference: Smith, Phys Rev 94(1), 1954, Table I
     */

    double l1_2 = l1 * l1;
    double m1_2 = m1 * m1;
    double n1_2 = n1 * n1;

    double anisotropy = l1_2 * m1_2 + m1_2 * n1_2 + n1_2 * l1_2;

    return pi_11 - 2.0 * (pi_11 - pi_12 - pi_44) * anisotropy;
}

double strain_gauge_gf_anisotropic(double pi_11, double pi_12, double pi_44,
                                   double l1, double m1, double n1,
                                   double nu, double E_gpa)
{
    /* Effective gauge factor for arbitrary crystal orientation.
     *
     * GF = 1 + 2*nu + pi_l(hkl) * E
     *
     * where pi_l is the longitudinal piezoresistive coefficient
     * for the given crystal direction.
     */

    double pi_l = strain_gauge_piezo_longitudinal(pi_11, pi_12, pi_44,
                                                   l1, m1, n1);
    return gauge_factor_from_material(nu, pi_l, E_gpa);
}

/* ========================================================================
 * L2: Temperature Compensation and Self-Heating
 * ======================================================================== */

double strain_gauge_select_stc(double specimen_cte_ppm_per_c,
                               double temperature_range_c)
{
    /* Select self-temperature-compensation (STC) gauge.
     *
     * STC gauges are manufactured with a specific thermal output
     * curve that cancels the differential thermal expansion of
     * the specimen material over a specified temperature range.
     *
     * The STC number is derived from the CTE in ppm/degF:
     *   STC_number = alpha_ppm_per_F (approx)
     *
     * Conversion: alpha[ppm/F] = alpha[ppm/K] * 5/9
     *
     * Common STC codes:
     *   STC-00: alpha ≈ 1.0 ppm/F (1.8 ppm/K) — silica, invar
     *   STC-03: alpha ≈ 3.3 ppm/F (5.9 ppm/K) — titanium, quartz
     *   STC-06: alpha ≈ 6.1 ppm/F (11.0 ppm/K) — carbon steel
     *   STC-09: alpha ≈ 9.4 ppm/F (16.9 ppm/K) — stainless steel
     *   STC-13: alpha ≈ 12.8 ppm/F (23.0 ppm/K) — aluminum
     *
     * The STC matching is typically within +/- 1.8 ppm/K,
     * giving residual thermal output < 2 ue/K.
     */

    double cte_ppm_per_f = specimen_cte_ppm_per_c * (5.0 / 9.0);

    /* Available STC numbers */
    double stc_table[] = {1.0, 3.3, 6.1, 9.4, 12.8, 0.0};

    /* Find closest STC number */
    double best_stc = 6.1;  /* Default: steel */
    double best_diff = fabs(cte_ppm_per_f - 6.1);

    int i;
    for (i = 0; stc_table[i] > 0.0; i++) {
        double diff = fabs(cte_ppm_per_f - stc_table[i]);
        if (diff < best_diff) {
            best_diff = diff;
            best_stc = stc_table[i];
        }
    }

    (void)temperature_range_c;  /* Reserved for future use */
    return best_stc;
}

double strain_gauge_self_heating(double power_w,
                                 double thermal_conductivity,
                                 double gauge_area_m2,
                                 double backing_thickness_m)
{
    /* Self-heating temperature rise in a strain gauge.
     *
     * Heat is generated in the gauge grid (Joule heating) and
     * must be conducted away through the backing and adhesive
     * into the specimen. The temperature rise depends on the
     * thermal resistance of this path:
     *
     *   dT = P * R_thermal
     *
     * where R_thermal = backing_thickness / (k * A)
     *
     * k = thermal conductivity of the backing + adhesive system
     * A = gauge grid area
     *
     * For a 350 Ohm gauge dissipating 5 mW:
     *   With polyimide backing (k≈0.12 W/mK), t=0.05mm, A=18mm^2:
     *   R_thermal = 0.05e-3 / (0.12 * 18e-6) = 23.1 K/W
     *   dT = 0.005 * 23.1 = 0.12 K (negligible on steel)
     *
     * With plastic specimen (k≈0.2 W/mK):
     *   R_thermal = 0.05e-3 / (0.2 * 18e-6) = 13.9 K/W
     *   dT = 0.005 * 13.9 = 0.07 K (still small for 5 mW)
     *
     * BUT: the gauge backing + adhesive dominate R_thermal,
     * not the specimen. So even on plastic, self-heating can
     * be moderate at low power.
     *
     * The apparent strain from self-heating:
     *   eps_heat = dT * (CTE_specimen + thermal_output_coefficient)
     *
     * For 0.1 K rise: eps_heat ≈ 0.1 * (11 + 2) = 1.3 ue (steel).
     * For precision measurement, keep dT < 0.1 K.
     */

    if (thermal_conductivity <= 0.0 || gauge_area_m2 <= 0.0) return 100.0;

    double r_thermal = backing_thickness_m /
                       (thermal_conductivity * gauge_area_m2);

    return power_w * r_thermal;
}

double strain_gauge_creep(double strain_ue, double creep_coefficient,
                          double time_hours)
{
    /* Creep error — time-dependent change in output under constant load.
     *
     * Creep is caused by viscoelastic relaxation in the gauge backing
     * polymer and the adhesive. It follows a logarithmic time dependence
     * characteristic of polymer viscoelasticity:
     *
     *   eps_creep(t) = eps_0 * C * log10(1 + t/t0)
     *
     * where:
     *   C = creep coefficient (0.0005-0.005 for epoxy/polyimide)
     *   t = time under sustained load [hours]
     *   t0 = reference time (1 hour)
     *
     * After 1 hour:  eps_creep = eps_0 * C * log10(2) ≈ 0.30*C*eps_0
     * After 10 hours: eps_creep = eps_0 * C * log10(11) ≈ 1.04*C*eps_0
     * After 100 hours: eps_creep = eps_0 * C * log10(101) ≈ 2.00*C*eps_0
     *
     * For C=0.001 (good installation), eps_0=1000 ue:
     *   After 100 hours: creep = 1000 * 0.001 * 2 = 2 ue (0.2% of reading)
     *
     * Creep is specified as a percentage of full scale over a
     * standard time period (typically 30 minutes at room temp).
     */

    if (time_hours < 0.0) return 0.0;

    double log_term = log10(1.0 + time_hours);  /* t0 = 1 hour */
    return strain_ue * creep_coefficient * log_term;
}

double strain_gauge_hysteresis(double max_strain_ue,
                               double hysteresis_factor,
                               int num_cycles)
{
    /* Hysteresis error in strain gauge installation.
     *
     * Hysteresis is the difference between loading and unloading
     * output at the same strain level. It decreases with repeated
     * cycling (cyclic stabilization):
     *
     *   hysteresis = H0 * exp(-n / tau) + H_inf
     *
     * where n = number of cycles, tau = stabilization cycles,
     * H0 = initial hysteresis, H_inf = asymptotic hysteresis.
     *
     * Simplified model used here:
     *   hysteresis = hysteresis_factor * max_strain * (1 + exp(-n/10))
     *
     * After ~10-20 cycles, hysteresis stabilizes at a low value.
     * This is why load cells are "exercised" before calibration:
     * mechanically cycling reduces hysteresis to its stable floor.
     */

    double decay = exp(-(double)num_cycles / 10.0);
    return max_strain_ue * hysteresis_factor * (1.0 + decay);
}

/* ========================================================================
 * L3: 2D Strain Analysis — Mohr's Circle
 * ======================================================================== */

void strain_gauge_transform(const strain_state_t *src, double theta_deg,
                            strain_state_t *dest)
{
    /* Strain transformation under coordinate rotation.
     *
     * Given strain components in x-y frame, compute strains in
     * x'-y' frame rotated by theta counterclockwise.
     *
     * Strain transformation equations (tensor transformation):
     *
     * eps_x' = (eps_x+eps_y)/2 + (eps_x-eps_y)/2 * cos(2theta)
     *          + (gamma_xy/2) * sin(2theta)
     *
     * eps_y' = (eps_x+eps_y)/2 - (eps_x-eps_y)/2 * cos(2theta)
     *          - (gamma_xy/2) * sin(2theta)
     *
     * gamma_xy'/2 = -(eps_x-eps_y)/2 * sin(2theta)
     *               + (gamma_xy/2) * cos(2theta)
     *
     * This is Mohr's circle for strain: the locus of (eps_n, gamma_n/2)
     * for all orientations traces a circle in strain space.
     *
     * Invariant: eps_x + eps_y = eps_x' + eps_y' (trace invariant)
     *
     * Reference: Timoshenko & Goodier, "Theory of Elasticity", 3rd ed., Ch.2
     */

    if (src == NULL || dest == NULL) return;

    double theta_rad = theta_deg * M_PI / 180.0;
    double c = cos(2.0 * theta_rad);
    double s = sin(2.0 * theta_rad);

    double avg = (src->epsilon_x + src->epsilon_y) / 2.0;
    double diff = (src->epsilon_x - src->epsilon_y) / 2.0;
    double half_gamma = src->gamma_xy / 2.0;

    dest->epsilon_x = avg + diff * c + half_gamma * s;
    dest->epsilon_y = avg - diff * c - half_gamma * s;
    dest->gamma_xy  = 2.0 * (-diff * s + half_gamma * c);

    /* Principal strains (invariant under rotation) */
    dest->epsilon_1 = src->epsilon_1;
    dest->epsilon_2 = src->epsilon_2;

    /* Updated principal angle (rotate by -theta) */
    dest->angle_deg = src->angle_deg - theta_deg;

    /* Von Mises strain is invariant under rotation */
    dest->epsilon_von_mises = src->epsilon_von_mises;
}

void strain_gauge_mohr_circle(const strain_state_t *strain,
                              double *center, double *radius)
{
    /* Compute Mohr's circle parameters.
     *
     * Center = (eps_x + eps_y) / 2   [mean normal strain]
     * Radius = sqrt[((eps_x-eps_y)/2)^2 + (gamma_xy/2)^2]
     *        = (eps_1 - eps_2) / 2    [maximum shear strain/2]
     *
     * Mohr's circle relates normal strain eps_n and shear strain
     * gamma_n/2 on any plane through the point:
     *
     *   (eps_n - center)^2 + (gamma_n/2)^2 = radius^2
     *
     * This is the equation of a circle in (eps, gamma/2) space.
     * Every possible strain state on any plane through the point
     * lies on this circle.
     *
     * Named after Otto Mohr (1835-1918), German civil engineer
     * who developed the graphical method for stress analysis.
     */

    if (strain == NULL || center == NULL || radius == NULL) return;

    *center = (strain->epsilon_x + strain->epsilon_y) / 2.0;

    double diff_half = (strain->epsilon_x - strain->epsilon_y) / 2.0;
    double shear_half = strain->gamma_xy / 2.0;

    *radius = sqrt(diff_half * diff_half + shear_half * shear_half);
}

/* ========================================================================
 * L4: Semiconductor Strain Gauge Physics
 * ======================================================================== */

void strain_gauge_silicon_piezo_coefficients(char doping_type,
                                             double resistivity_ohm_cm,
                                             double *pi_11_out,
                                             double *pi_12_out,
                                             double *pi_44_out)
{
    /* Piezoresistive coefficients for silicon.
     *
     * Based on Smith (1954) and Kanda (1991) experimental data.
     * The coefficients depend on doping type and concentration.
     *
     * Units: 10^-11 Pa^-1 (the standard unit in the literature).
     *
     * Higher doping → lower piezoresistive coefficients (lower GF)
     * BUT also → lower temperature coefficient (better stability).
     * This is the fundamental GF vs. stability trade-off.
     *
     * Data from Kanda (1991) for room temperature:
     */

    double pi_11, pi_12, pi_44;
    double log_rho = log10(resistivity_ohm_cm);

    if (doping_type == 'n') {
        /* n-type silicon */
        /* Fit to Kanda's data for n-Si */
        pi_11 = -102.2 * (1.0 - 0.3 * (log_rho - 1.0));  /* ~ -102 at 10 ohm-cm */
        pi_12 =  53.4  * (1.0 - 0.2 * (log_rho - 1.0));  /* ~ +53 at 10 ohm-cm */
        pi_44 = -13.6  * (1.0 - 0.1 * (log_rho - 1.0));  /* ~ -14 at 10 ohm-cm */

        /* Clamp to reasonable range */
        if (pi_11 > -10.0) pi_11 = -10.0;
        if (pi_11 < -150.0) pi_11 = -150.0;
    } else {
        /* p-type silicon */
        pi_11 =   6.6  * (1.0 - 0.4 * (log_rho - 1.0));   /* ~ +7 at 10 ohm-cm */
        pi_12 =  -1.1  * (1.0 - 0.3 * (log_rho - 1.0));   /* ~ -1 at 10 ohm-cm */
        pi_44 = 138.1  * (1.0 - 0.5 * (log_rho - 1.0));   /* ~ +138 at 10 ohm-cm */

        if (pi_44 < 10.0) pi_44 = 10.0;
        if (pi_44 > 200.0) pi_44 = 200.0;
    }

    *pi_11_out = pi_11 * 1.0e-11;
    *pi_12_out = pi_12 * 1.0e-11;
    *pi_44_out = pi_44 * 1.0e-11;
}

double strain_gauge_silicon_gf(double doping_conc_cm3,
                               double temperature_k,
                               int orientation)
{
    /* Approximate gauge factor for diffused silicon piezoresistor.
     *
     * The GF depends on:
     * 1. Doping concentration (higher doping → lower GF, less T-dependent)
     * 2. Temperature (GF decreases with increasing T)
     * 3. Crystal orientation (as discussed in piezo_longitudinal)
     *
     * Empirical fit for p-type <110> orientation (most common for MEMS):
     *
     * GF ≈ GF_0 * (300/T)^a * (1 - b*log10(N_doping))
     *
     * where GF_0 ≈ 120, a ≈ 0.5-1.0, b ≈ 0.1-0.2
     *
     * For N_doping = 10^18 cm^-3, T = 300K, <110> p-type:
     *   GF ≈ 120 * (1)^0.8 * (1 - 0.15*18) = not right...
     *
     * Let's use a simpler model from Kanda's data.
     *
     * Reference doping: 10^18 cm^-3 p-type <110> → GF ≈ 70-90
     * Reference doping: 10^17 cm^-3 p-type <110> → GF ≈ 100-130
     * Reference doping: 10^20 cm^-3 p-type <110> → GF ≈ 20-40
     */

    double log_n = log10(doping_conc_cm3);
    double gf_ref;

    /* Orientation-dependent reference GF at 300K, 10^18 cm^-3 */
    switch (orientation) {
        case 0: /* <100> */
            gf_ref = 10.0;   /* p-type <100> has low GF */
            break;
        case 1: /* <110> */
            gf_ref = 75.0;   /* p-type <110> — most common */
            break;
        case 2: /* <111> */
            gf_ref = 95.0;   /* p-type <111> — highest GF */
            break;
        default:
            gf_ref = 75.0;
            break;
    }

    /* Doping dependence: GF ∝ 1/log(N) */
    double doping_factor = 18.0 / log_n;  /* Normalized at 10^18 */

    /* Temperature dependence: GF ∝ (300/T)^0.8 */
    double temp_factor = pow(300.0 / temperature_k, 0.8);

    return gf_ref * doping_factor * temp_factor;
}

double strain_gauge_silicon_tcr(double doping_conc_cm3, double temperature_k)
{
    /* Temperature coefficient of resistivity for doped silicon.
     *
     * TCR transitions from positive (lightly doped, semiconductor-like)
     * to negative (heavily doped, metallic-like) at high doping.
     *
     * Approximate model:
     *   TCR ≈ TCR_intrinsic * exp(-N/10^19) + TCR_metallic * (1 - exp(-N/10^19))
     *
     * where TCR_intrinsic ≈ 7000 ppm/K, TCR_metallic ≈ -1000 ppm/K.
     *
     * This is why MEMS piezoresistors are heavily doped:
     * the lower GF is acceptable for the dramatic improvement
     * in temperature stability.
     */

    double n_norm = doping_conc_cm3 / 1.0e19;
    double exp_term = exp(-n_norm);

    double tcr_intrinsic = 7000.0;  /* Lightly doped silicon */
    double tcr_metallic   = -500.0;  /* Heavily doped (degenerate) */

    /* Blend between regimes */
    double tcr = tcr_intrinsic * exp_term + tcr_metallic * (1.0 - exp_term);

    /* Temperature scaling: TCR decreases at higher T */
    double temp_factor = 300.0 / temperature_k;

    return tcr * temp_factor;
}

/* ========================================================================
 * Material database and stress concentration
 * ======================================================================== */

int strain_gauge_material_lookup(const char *name,
                                 material_database_entry_t *entry)
{
    /* Built-in material properties database.
     *
     * Provides key mechanical and thermal properties for common
     * engineering materials used with strain gauges.
     *
     * Returns 0 if found, -1 if not in database.
     */

    if (name == NULL || entry == NULL) return -1;

    memset(entry, 0, sizeof(*entry));

    if (strstr(name, "steel") || strstr(name, "Steel")) {
        entry->material.name                = "Carbon Steel (1018)";
        entry->material.youngs_modulus_gpa  = 200.0;
        entry->material.poisson_ratio       = 0.29;
        entry->material.thermal_expansion_ppm = 11.7;
        entry->material.thermal_conductivity = 51.9;
        entry->material.density_g_cm3       = 7.85;
        entry->gauge_alloy                  = "constantan";
        entry->stc_code                     = "STC-06";
        return 0;
    }

    if (strstr(name, "aluminum") || strstr(name, "Aluminum") || strstr(name, "aluminium")) {
        entry->material.name                = "Aluminum (6061-T6)";
        entry->material.youngs_modulus_gpa  = 69.0;
        entry->material.poisson_ratio       = 0.33;
        entry->material.thermal_expansion_ppm = 23.6;
        entry->material.thermal_conductivity = 167.0;
        entry->material.density_g_cm3       = 2.70;
        entry->gauge_alloy                  = "constantan";
        entry->stc_code                     = "STC-13";
        return 0;
    }

    if (strstr(name, "stainless") || strstr(name, "Stainless")) {
        entry->material.name                = "Stainless Steel (304)";
        entry->material.youngs_modulus_gpa  = 193.0;
        entry->material.poisson_ratio       = 0.29;
        entry->material.thermal_expansion_ppm = 17.3;
        entry->material.thermal_conductivity = 16.2;
        entry->material.density_g_cm3       = 8.00;
        entry->gauge_alloy                  = "karma";
        entry->stc_code                     = "STC-09";
        return 0;
    }

    if (strstr(name, "titanium") || strstr(name, "Titanium")) {
        entry->material.name                = "Titanium (Ti-6Al-4V)";
        entry->material.youngs_modulus_gpa  = 114.0;
        entry->material.poisson_ratio       = 0.34;
        entry->material.thermal_expansion_ppm = 8.6;
        entry->material.thermal_conductivity = 6.7;
        entry->material.density_g_cm3       = 4.43;
        entry->gauge_alloy                  = "karma";
        entry->stc_code                     = "STC-03";
        return 0;
    }

    return -1;  /* Material not found */
}

double strain_gauge_stress_concentration(int geometry_type,
                                         double dim1, double dim2)
{
    /* Stress concentration factor K_t for common geometries.
     *
     * The stress concentration factor is the ratio of maximum
     * local stress to nominal (far-field) stress:
     *
     *   K_t = sigma_max / sigma_nominal
     *
     * Strain gauges measure AVERAGE strain over their active length.
     * If placed near a stress concentration, the measured strain
     * will be higher than nominal.
     *
     * This function estimates K_t for common geometries based on
     * standard reference data (Peterson's Stress Concentration Factors).
     */

    switch (geometry_type) {
        case 0: /* Circular hole in infinite plate under uniaxial tension */
            /* Kirsch (1898) solution: K_t = 3.0 for isotropic plate */
            (void)dim1; (void)dim2;
            return 3.0;

        case 1: /* U-notch in semi-infinite plate */
            /* K_t ≈ 1 + 2*sqrt(depth/radius) */
            {
                double depth = dim1;
                double radius = dim2;
                if (radius <= 0.0) return 100.0;  /* Sharp crack → infinite */
                return 1.0 + 2.0 * sqrt(depth / radius);
            }

        case 2: /* Shoulder fillet in stepped shaft under tension */
            /* K_t ≈ 1 + 0.5*sqrt(D/d - 1) / (r/d) */
            {
                double D = dim1;  /* Large diameter */
                double d_val = dim2; /* Small diameter */
                /* Use approximate: K_t depends on D/d and r/d ratios */
                if (d_val <= 0.0) return 10.0;
                return 1.0 + 0.5 * sqrt(D / d_val - 1.0) / 0.1;
            }

        case 3: /* V-groove */
            /* K_t ~ 2.0-5.0 depending on notch angle and depth */
            (void)dim1; (void)dim2;
            return 3.0;

        default:
            return 1.0;  /* No concentration */
    }
}

double strain_gauge_misalignment_error(double eps_principal, double nu,
                                       double delta_deg)
{
    /* Strain gauge misalignment error.
     *
     * If a gauge is installed at angle delta from the intended
     * measurement direction, the measured strain along the gauge
     * axis is:
     *
     *   eps_meas = eps_1 * cos^2(delta) + eps_2 * sin^2(delta)
     *
     * For uniaxial stress (eps_2 = -nu*eps_1):
     *   eps_meas = eps_1 * [cos^2(delta) - nu*sin^2(delta)]
     *
     * Relative error:
     *   error = (eps_meas - eps_1) / eps_1
     *         = cos^2(delta) - nu*sin^2(delta) - 1
     *         = cos^2(delta) - nu*sin^2(delta) - (cos^2+sin^2)
     *         = -sin^2(delta) - nu*sin^2(delta)
     *         = -(1+nu)*sin^2(delta)
     *
     * For small delta: sin(delta) ≈ delta [rad]
     *   error ≈ -(1+nu)*delta^2
     *
     * Example: delta=2 deg (=0.035 rad), nu=0.3
     *   error ≈ -(1.3)*(0.035)^2 = -0.0016 = -0.16%
     *
     * This shows that strain gauges are surprisingly tolerant
     * to small misalignment errors (cosine error is second-order).
     *
     * For delta=10 deg: sin^2(10)=0.030, error = -(1.3)*0.03 = -3.9%
     * Now it matters. Keep alignment within 2-3 degrees.
     */

    (void)eps_principal;
    double delta_rad = delta_deg * M_PI / 180.0;
    double sin_delta = sin(delta_rad);

    return -(1.0 + nu) * sin_delta * sin_delta;
}
