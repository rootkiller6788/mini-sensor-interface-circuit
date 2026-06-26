#ifndef MEMS_PCB_DESIGN_H
#define MEMS_PCB_DESIGN_H
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef enum {
    PCB_LAYER_2 = 2, PCB_LAYER_4 = 4, PCB_LAYER_6 = 6, PCB_LAYER_8 = 8
} pcb_layer_count_t;

typedef enum {
    STACKUP_SIG_GND_PWR_SIG = 0,
    STACKUP_SIG_GND_SIG_PWR = 1,
    STACKUP_SIG_GND_SIG_PWR_GND_SIG = 2
} pcb_stackup_t;

typedef struct {
    double trace_width_mm;
    double trace_spacing_mm;
    double copper_thickness_um;
    double dielectric_thickness_mm;
    double dielectric_constant;
    double characteristic_impedance_ohm;
    double propagation_delay_ps_per_mm;
    double attenuation_db_per_cm;
    pcb_layer_count_t layers;
    pcb_stackup_t stackup;
} pcb_trace_params_t;

typedef struct {
    double target_impedance_ohm;
    double max_ripple_mv;
    double transient_current_ma;
    double vrm_bandwidth_hz;
    int num_decoupling_caps;
    double cap_values_uf[10];
    double cap_esr_mohm[10];
    double cap_esl_nh[10];
} pdn_design_t;

typedef struct {
    double target_impedance_ohm;
    double frequency_range_hz[2];
    double achieved_impedance_ohm;
    double resonance_freqs_hz[10];
    int num_resonances;
} pdn_analysis_t;

double pcb_microstrip_z0(double w, double h, double er);
double pcb_stripline_z0(double w, double h, double er);
double pcb_diff_microstrip_z0(double w, double s, double h, double er);
double pcb_diff_stripline_z0(double w, double s, double h, double er);
double pcb_propagation_delay(double length_mm, double er);
double pcb_critical_length(double tr_rise_ps, double er);
double pcb_decoupling_cap_z(double f, double C, double ESL, double ESR);
double pcb_pdn_target_impedance(double Vdd, double ripple_pct, double I_max);
int pcb_pdn_impedance_profile(pdn_design_t *d, pdn_analysis_t *out, int n_freqs);
double pcb_keepout_zone_mm(double freq_hz, int is_crystal);
double pcb_via_inductance_nh(double h_mm, double d_mm);
double pcb_via_capacitance_pf(double d1_mm, double d2_mm, double h_mm, double er);
int pcb_split_plane_check(double agnd_area, double dgnd_area, double isolation_gap_mm, double max_freq_hz);
double pcb_guard_ring_width(double freq_hz, double er);
double pcb_stitching_via_spacing(double lambda_min);
double pcb_far_field_emission(double f, double I, double L, double d);
double pcb_near_field_crosstalk(double H_sep, double w, double s, double tr);
void pcb_trace_params_init(pcb_trace_params_t *p, pcb_layer_count_t layers);

#endif
