#include "mems_pcb_design.h"
#include <stdlib.h>
#include <string.h>

#define C0 299792458.0

void pcb_trace_params_init(pcb_trace_params_t *p, pcb_layer_count_t layers)
{
    if (!p) return;
    memset(p, 0, sizeof(*p));
    p->layers = layers;
    p->trace_width_mm = 0.15;
    p->trace_spacing_mm = 0.15;
    p->copper_thickness_um = 35.0;
    p->dielectric_thickness_mm = 0.2;
    p->dielectric_constant = 4.3;
    p->characteristic_impedance_ohm = 50.0;
    p->propagation_delay_ps_per_mm = 5.73;
}

/* L2: Microstrip impedance (IPC-2141 formula) */
double pcb_microstrip_z0(double w, double h, double er)
{
    if (h <= 0.0 || w <= 0.0) return 0.0;
    double ratio = w / h;
    if (ratio < 1.0) {
        double eff_er = (er + 1.0) / 2.0 + (er - 1.0) / 2.0 * (1.0 / sqrt(1.0 + 12.0 / ratio) + 0.04 * (1.0 - ratio) * (1.0 - ratio));
        return (60.0 / sqrt(eff_er)) * log(8.0 / ratio + ratio / 4.0);
    } else {
        double eff_er = (er + 1.0) / 2.0 + (er - 1.0) / 2.0 / sqrt(1.0 + 12.0 / ratio);
        return (120.0 * M_PI) / (sqrt(eff_er) * (ratio + 1.393 + 0.667 * log(ratio + 1.444)));
    }
}

/* L2: Stripline impedance */
double pcb_stripline_z0(double w, double h, double er)
{
    if (h <= 0.0 || er <= 0.0) return 0.0;
    double ratio = w / h;
    if (ratio < 0.35) {
        return (60.0 / sqrt(er)) * log(4.0 * h / (M_PI * w));
    } else {
        double term = ratio + 0.441 + (er - 1.0) / (2.0 * er) * (0.445 + w / (M_PI * h));
        return (94.15 / sqrt(er)) / term;
    }
}

/* L2: Differential microstrip impedance */
double pcb_diff_microstrip_z0(double w, double s, double h, double er)
{
    if (h <= 0.0) return 0.0;
    double z0_single = pcb_microstrip_z0(w, h, er);
    if (z0_single <= 0.0) return 0.0;
    double coupling = exp(-0.96 * s / h);
    return 2.0 * z0_single * (1.0 - 0.48 * coupling);
}

/* L2: Differential stripline impedance */
double pcb_diff_stripline_z0(double w, double s, double h, double er)
{
    if (h <= 0.0) return 0.0;
    double z0_single = pcb_stripline_z0(w, h, er);
    if (z0_single <= 0.0) return 0.0;
    double coupling = exp(-2.9 * s / h);
    return 2.0 * z0_single * (1.0 - 0.347 * coupling);
}

/* L3: Propagation delay */
double pcb_propagation_delay(double length_mm, double er)
{
    if (er <= 0.0) return 0.0;
    double v = C0 / sqrt(er);
    return length_mm * 1e-3 / v * 1e12;
}

/* L3: Critical length for transmission line effects */
double pcb_critical_length(double tr_rise_ps, double er)
{
    if (er <= 0.0 || tr_rise_ps <= 0.0) return 0.0;
    double v = C0 / sqrt(er) * 1e-3;
    return v * tr_rise_ps * 0.5;
}

/* L4: Decoupling capacitor impedance */
double pcb_decoupling_cap_z(double f, double C, double ESL, double ESR)
{
    if (f <= 0.0) return HUGE_VAL;
    double w = 2.0 * M_PI * f;
    double Xc = -1.0 / (w * C);
    double Xl = w * ESL;
    double R = ESR;
    double X = Xc + Xl;
    return sqrt(R*R + X*X);
}

/* L4: PDN target impedance */
double pcb_pdn_target_impedance(double Vdd, double ripple_pct, double I_max)
{
    if (I_max <= 0.0) return 0.0;
    double V_ripple = Vdd * ripple_pct / 100.0;
    return V_ripple / I_max;
}

/* L4: PDN impedance profile analysis */
int pcb_pdn_impedance_profile(pdn_design_t *d, pdn_analysis_t *out, int n_freqs)
{
    if (!d || !out || n_freqs < 2) return -1;
    memset(out, 0, sizeof(*out));
    out->target_impedance_ohm = d->target_impedance_ohm;
    out->frequency_range_hz[0] = 1000.0;
    out->frequency_range_hz[1] = 1e9;
    double max_z = 0.0;
    int i;
    for (i = 0; i < d->num_decoupling_caps && i < 10; i++) {
        double C = d->cap_values_uf[i] * 1e-6;
        double esr = d->cap_esr_mohm[i] * 1e-3;
        double esl = d->cap_esl_nh[i] * 1e-9;
        double fres = 1.0 / (2.0 * M_PI * sqrt(esl * C));
        if (esr > max_z) max_z = esr;
        out->resonance_freqs_hz[i] = fres;
        out->num_resonances++;
    }
    out->achieved_impedance_ohm = max_z;
    return 0;
}

/* L5: Keepout zone for crystal/MEMS */
double pcb_keepout_zone_mm(double freq_hz, int is_crystal)
{
    if (freq_hz <= 0.0) return 1.0;
    double lambda = C0 / freq_hz * 1000.0;
    double factor = is_crystal ? 0.1 : 0.05;
    return lambda * factor;
}

/* L5: Via inductance (approximate formula) */
double pcb_via_inductance_nh(double h_mm, double d_mm)
{
    if (d_mm <= 0.0) return 0.0;
    double h = h_mm * 1e-3;
    double r = d_mm * 0.5e-3;
    return 5.08 * h * (log(4.0 * h / r) + 1.0);
}

/* L5: Via capacitance */
double pcb_via_capacitance_pf(double d1_mm, double d2_mm, double h_mm, double er)
{
    if (d2_mm <= d1_mm || d2_mm <= 0.0 || er <= 0.0) return 0.0;
    double e0 = 8.854e-12;
    return 1.41 * er * d1_mm * h_mm / (d2_mm - d1_mm) * e0 * 1e12;
}

/* L6: Mixed-signal split plane analysis */
int pcb_split_plane_check(double agnd_area, double dgnd_area,
                          double isolation_gap_mm, double max_freq_hz)
{
    if (agnd_area <= 0.0 || dgnd_area <= 0.0 || isolation_gap_mm <= 0.0) return 0;
    double ratio = agnd_area / dgnd_area;
    if (ratio < 0.2 || ratio > 5.0) return 0;
    double lambda = C0 / max_freq_hz * 1000.0;
    double min_gap = lambda / 20.0;
    if (isolation_gap_mm < min_gap) return 0;
    return 1;
}

/* L6: Guard ring width */
double pcb_guard_ring_width(double freq_hz, double er)
{
    if (freq_hz <= 0.0 || er <= 0.0) return 0.5;
    double lambda = C0 / (freq_hz * sqrt(er)) * 1000.0;
    return lambda / 4.0;
}

/* L6: Stitching via spacing */
double pcb_stitching_via_spacing(double lambda_min)
{
    if (lambda_min <= 0.0) return 1.0;
    return lambda_min / 8.0;
}

/* L7: Far-field EMI emission (dipole model) */
double pcb_far_field_emission(double f, double I, double L, double d)
{
    if (f <= 0.0 || d <= 0.0) return 0.0;
    double lambda = C0 / f;
    double k = 2.0 * M_PI / lambda;
    double E = (1.0 / (4.0 * M_PI * 8.854e-12)) * (k * I * L) / d;
    return E;
}

/* L7: Near-field crosstalk estimate */
double pcb_near_field_crosstalk(double H_sep, double w, double s, double tr)
{
    if (H_sep <= 0.0 || w <= 0.0 || tr <= 0.0) return 0.0;
    double K = 1.0 / (1.0 + (s / H_sep) * (s / H_sep));
    double Lm = 5.08 * 1e-9 * H_sep * 1e-3;
    double Cm = 8.854e-12 * 4.3 * w * 1e-3 / H_sep * 1e-3;
    return K * (Lm + Cm * 50.0 * 50.0) / tr * 1e12;
}

/* L5: Thermal resistance of copper trace */
double pcb_trace_thermal_resistance(double length_mm, double width_mm, double thickness_um)
{
    if (width_mm <= 0.0 || thickness_um <= 0.0) return 0.0;
    double area = width_mm * 1e-3 * length_mm * 1e-3;
    double R_thermal = 1.0 / (398.0 * area * thickness_um * 1e-6 / (length_mm * 1e-3));
    return R_thermal;
}

/* L5: Current carrying capacity (IPC-2221) */
double pcb_trace_current_capacity(double width_mm, double thickness_um, double temp_rise_c)
{
    if (width_mm <= 0.0 || thickness_um <= 0.0) return 0.0;
    double area_mil2 = width_mm * 39.37 * thickness_um * 0.03937;
    if (temp_rise_c <= 0.0) temp_rise_c = 10.0;
    double k = 0.048, b = 0.44, c = 0.725;
    return k * pow(temp_rise_c, b) * pow(area_mil2, c);
}

/* L6: Impedance discontinuity reflection coefficient */
double pcb_reflection_coefficient(double Z0, double ZL)
{
    if (Z0 + ZL == 0.0) return 0.0;
    return (ZL - Z0) / (ZL + Z0);
}

/* L6: Return loss from reflection coefficient */
double pcb_return_loss_db(double reflection_coeff)
{
    double abs_rc = fabs(reflection_coeff);
    if (abs_rc >= 1.0) return 0.0;
    return -20.0 * log10(abs_rc);
}

/* L6: Insertion loss from S-parameters */
double pcb_insertion_loss_db(double s21_magnitude)
{
    if (s21_magnitude <= 0.0) return HUGE_VAL;
    return -20.0 * log10(s21_magnitude);
}

/* L7: PDN impedance at frequency with multiple capacitors */
double pcb_pdn_multi_cap_z(double f, pdn_design_t *d)
{
    if (!d || f <= 0.0) return 0.0;
    double Z_total = 0.0;
    int i;
    for (i = 0; i < d->num_decoupling_caps && i < 10; i++) {
        double C = d->cap_values_uf[i] * 1e-6;
        double esr = d->cap_esr_mohm[i] * 1e-3;
        double esl = d->cap_esl_nh[i] * 1e-9;
        double Zc = pcb_decoupling_cap_z(f, C, esl, esr);
        if (Zc > 0.0) Z_total += 1.0 / Zc;
    }
    if (Z_total <= 0.0) return HUGE_VAL;
    return 1.0 / Z_total;
}

/* L7: Ground bounce estimation */
double pcb_ground_bounce_mv(double L_nh, double di_ma, double dt_ns)
{
    if (dt_ns <= 0.0) return 0.0;
    return L_nh * (di_ma * 1e-3) / (dt_ns * 1e-9) * 1e3;
}

/* L7: Power plane resonance frequency */
double pcb_plane_resonance(double L_mm, double W_mm, double er, int m, int n)
{
    if (L_mm <= 0.0 || W_mm <= 0.0 || er <= 0.0) return 0.0;
    double c = 3.0e8;
    return (c / (2.0 * sqrt(er))) * sqrt(pow((double)m/L_mm, 2.0) + pow((double)n/W_mm, 2.0)) * 1e3;
}

/* L8: Differential pair skew management */
double pcb_diff_skew_ps(double length_diff_mm, double er)
{
    if (er <= 0.0) return 0.0;
    return length_diff_mm / (3.0e8 / sqrt(er)) * 1e12;
}

/* L8: Eye diagram opening estimate */
double pcb_eye_opening(double bit_period_ps, double jitter_ps, double noise_mv, double signal_swing_mv)
{
    if (bit_period_ps <= 0.0 || signal_swing_mv <= 0.0) return 0.0;
    double ui_jitter = jitter_ps / bit_period_ps;
    double noise_frac = noise_mv / signal_swing_mv;
    return (1.0 - ui_jitter) * (1.0 - noise_frac) * 100.0;
}

/* L7: PCB stackup impedance budget */
typedef struct {
    double layer_impedances[8];
    double target_impedance;
    int num_layers;
} pcb_impedance_budget_t;

int pcb_impedance_budget_check(pcb_impedance_budget_t *budget, double tolerance_pct)
{
    if (!budget || budget->num_layers < 1) return -1;
    int i;
    for (i = 0; i < budget->num_layers && i < 8; i++) {
        double error = fabs(budget->layer_impedances[i] - budget->target_impedance) / budget->target_impedance * 100.0;
        if (error > tolerance_pct) return 0;
    }
    return 1;
}

/* L7: FR-4 dielectric constant vs frequency */
double pcb_fr4_er_vs_freq(double freq_hz)
{
    if (freq_hz <= 0.0) return 4.3;
    double log_f = log10(freq_hz);
    return 4.3 - 0.05 * (log_f - 6.0);
}

/* L7: Trace resistance at temperature */
double pcb_trace_resistance(double R0_ohm, double temp_c, double temp_coeff_ppm)
{
    double dT = temp_c - 25.0;
    return R0_ohm * (1.0 + temp_coeff_ppm * 1e-6 * dT);
}

/* L8: Differential pair intra-pair skew budget */
double pcb_diff_skew_budget(double data_rate_gbps, double max_skew_ui)
{
    if (data_rate_gbps <= 0.0) return 0.0;
    double bit_period_ps = 1.0 / data_rate_gbps * 1000.0;
    return bit_period_ps * max_skew_ui;
}

/* L8: Via stub resonance frequency */
double pcb_via_stub_resonance(double stub_length_mm, double er)
{
    if (stub_length_mm <= 0.0 || er <= 0.0) return 0.0;
    double c = 3.0e8;
    return c / (4.0 * stub_length_mm * 1e-3 * sqrt(er));
}

/* L8: Mixed-signal isolation analysis */
typedef struct {
    double digital_noise_uv;
    double analog_sensitivity_uv;
    double isolation_impedance_ohm;
    double frequency_hz;
} mixed_signal_isolation_t;

double mixed_signal_coupling_noise(mixed_signal_isolation_t *msi)
{
    if (!msi || msi->isolation_impedance_ohm <= 0.0) return 0.0;
    return msi->digital_noise_uv / msi->isolation_impedance_ohm * 377.0;
}

/* L8: Substrate coupling noise estimate */
double pcb_substrate_coupling(double aggressor_voltage_v, double distance_mm, double substrate_resistivity_ohm_cm)
{
    if (distance_mm <= 0.0) return 0.0;
    double coupling_factor = substrate_resistivity_ohm_cm / (distance_mm * 10.0);
    return aggressor_voltage_v * coupling_factor;
}

/* L7: BGA escape routing feasibility */
typedef struct {
    double bga_pitch_mm;
    double pad_diameter_mm;
    double min_trace_width_mm;
    double min_spacing_mm;
    int num_rows;
} bga_escape_params_t;

int bga_escape_feasible(bga_escape_params_t *p, int *channels_per_layer)
{
    if (!p || !channels_per_layer || p->bga_pitch_mm <= 0.0) return 0;
    double available = p->bga_pitch_mm - p->pad_diameter_mm;
    double per_channel = p->min_trace_width_mm + p->min_spacing_mm;
    if (per_channel <= 0.0) return 0;
    *channels_per_layer = (int)(available / per_channel);
    return (*channels_per_layer > 0) ? 1 : 0;
}

/* L8: Signal integrity eye diagram simulation (simple) */
typedef struct {
    double bit_period_ps;
    double rise_time_ps;
    double fall_time_ps;
    double voltage_swing_v;
    double jitter_rms_ps;
    double noise_rms_v;
} eye_diagram_params_t;

double eye_diagram_height(eye_diagram_params_t *ep)
{
    if (!ep || ep->voltage_swing_v <= 0.0) return 0.0;
    double noise_margin = ep->voltage_swing_v - 6.0*ep->noise_rms_v;
    return noise_margin / ep->voltage_swing_v * 100.0;
}

double eye_diagram_width(eye_diagram_params_t *ep)
{
    if (!ep || ep->bit_period_ps <= 0.0) return 0.0;
    double jitter_margin = ep->bit_period_ps - 2.0*ep->rise_time_ps - 2.0*ep->jitter_rms_ps;
    return jitter_margin / ep->bit_period_ps * 100.0;
}
