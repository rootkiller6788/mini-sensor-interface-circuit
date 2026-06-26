/**
 * @file digital_isolator.c
 * @brief Core Digital Isolator Implementation
 * Knowledge coverage: L1-L6 Complete
 */
#include "digital_isolator.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* L1: Default parameter tables for known isolator technologies */
static const isolator_barrier_params_t DEFAULT_REINFORCED_CAP = {
    .viso_peak_kv = 8.0, .viso_rms_kv = 5.7, .viso_surge_kv = 12.8,
    .working_voltage_rms = 1500.0, .working_voltage_dc = 2121.0,
    .creepage_mm = 18.0, .clearance_mm = 16.0, .dti_um = 420.0,
    .iso_class = ISOL_CLASS_REINFORCED, .surge_cat = SURGE_CAT_III,
    .pollution = POLLUTION_DEG_2,
    .propagation_delay_ns = 11.0, .max_prop_delay_ns = 16.0,
    .pulse_width_distortion_ns = 0.5, .channel_skew_ns = 2.0,
    .part_part_skew_ns = 4.0, .data_rate_mbps = 100.0,
    .cmti_kv_us = 100.0, .max_jitter_ns = 0.3,
    .supply_voltage_v = 3.3, .supply_current_per_ch_ma = 1.5,
    .idle_current_ma = 2.8
};
static const isolator_barrier_params_t DEFAULT_BASIC_CAP = {
    .viso_peak_kv = 4.0, .viso_rms_kv = 3.0, .viso_surge_kv = 6.5,
    .working_voltage_rms = 560.0, .working_voltage_dc = 792.0,
    .creepage_mm = 8.0, .clearance_mm = 8.0, .dti_um = 16.0,
    .iso_class = ISOL_CLASS_BASIC, .surge_cat = SURGE_CAT_II,
    .pollution = POLLUTION_DEG_2,
    .propagation_delay_ns = 7.0, .max_prop_delay_ns = 11.0,
    .pulse_width_distortion_ns = 0.3, .channel_skew_ns = 1.5,
    .part_part_skew_ns = 3.0, .data_rate_mbps = 150.0,
    .cmti_kv_us = 85.0, .max_jitter_ns = 0.2,
    .supply_voltage_v = 3.3, .supply_current_per_ch_ma = 1.2,
    .idle_current_ma = 2.0
};
static const isolator_barrier_params_t DEFAULT_REINFORCED_MAG = {
    .viso_peak_kv = 7.5, .viso_rms_kv = 5.0, .viso_surge_kv = 10.0,
    .working_voltage_rms = 1200.0, .working_voltage_dc = 1697.0,
    .creepage_mm = 12.0, .clearance_mm = 12.0, .dti_um = 25.0,
    .iso_class = ISOL_CLASS_REINFORCED, .surge_cat = SURGE_CAT_III,
    .pollution = POLLUTION_DEG_2,
    .propagation_delay_ns = 13.0, .max_prop_delay_ns = 18.0,
    .pulse_width_distortion_ns = 0.8, .channel_skew_ns = 2.5,
    .part_part_skew_ns = 4.5, .data_rate_mbps = 90.0,
    .cmti_kv_us = 75.0, .max_jitter_ns = 0.4,
    .supply_voltage_v = 3.3, .supply_current_per_ch_ma = 2.0,
    .idle_current_ma = 3.5
};
static const isolator_barrier_params_t DEFAULT_BASIC_OPT = {
    .viso_peak_kv = 3.75, .viso_rms_kv = 2.5, .viso_surge_kv = 5.0,
    .working_voltage_rms = 350.0, .working_voltage_dc = 495.0,
    .creepage_mm = 6.0, .clearance_mm = 6.0, .dti_um = 80.0,
    .iso_class = ISOL_CLASS_BASIC, .surge_cat = SURGE_CAT_II,
    .pollution = POLLUTION_DEG_2,
    .propagation_delay_ns = 48.0, .max_prop_delay_ns = 75.0,
    .pulse_width_distortion_ns = 15.0, .channel_skew_ns = 20.0,
    .part_part_skew_ns = 30.0, .data_rate_mbps = 10.0,
    .cmti_kv_us = 15.0, .max_jitter_ns = 2.0,
    .supply_voltage_v = 5.0, .supply_current_per_ch_ma = 4.0,
    .idle_current_ma = 8.0
};

/* L2: set_default_barrier_model */
static void set_default_barrier_model(digital_isolator_t *isolator)
{
    switch (isolator->technology) {
    case ISOL_TECH_CAPACITIVE:
    case ISOL_TECH_RF_MODULATED: {
        capacitive_barrier_t *cb = &isolator->barrier_model.capacitive;
        cb->barrier_capacitance_ff = 120.0;
        cb->parasitic_side1_ff = 15.0;
        cb->parasitic_side2_ff = 15.0;
        cb->dielectric_thickness_um = 15.0;
        cb->dielectric_constant = 3.9;
        cb->carrier_freq_mhz = 500.0;
        cb->electrode_area_um2 = 2500.0;
        break;
    }
    case ISOL_TECH_MAGNETIC:
    case ISOL_TECH_GIANT_MAGNETORESISTIVE: {
        magnetic_barrier_t *mb = &isolator->barrier_model.magnetic;
        mb->primary_inductance_nh = 8.0;
        mb->secondary_inductance_nh = 8.0;
        mb->mutual_inductance_nh = 6.4;
        mb->coupling_coefficient = 0.8;
        mb->coil_resistance_ohm = 2.5;
        mb->insulation_thickness_um = 20.0;
        mb->coil_turns_primary = 3.0;
        mb->coil_turns_secondary = 3.0;
        mb->pulse_width_ns = 1.0;
        break;
    }
    case ISOL_TECH_OPTICAL: {
        optical_barrier_t *ob = &isolator->barrier_model.optical;
        ob->led_forward_voltage_v = 1.6;
        ob->led_forward_current_ma = 8.0;
        ob->current_transfer_ratio = 0.5;
        ob->ctr_temp_coeff_pct_per_c = -0.5;
        ob->led_degradation_rate = 0.001;
        ob->photodiode_responsivity = 0.4;
        ob->max_data_rate_mbps = 10.0;
        break;
    }
    default: break;
    }
}

/* L1: digital_isolator_init */
int digital_isolator_init(digital_isolator_t *isolator,
                          isolator_technology_t tech,
                          isolation_class_t iso_class, uint8_t num_ch)
{
    if (!isolator || num_ch == 0 || num_ch > 8) return -1;
    memset(isolator, 0, sizeof(*isolator));
    isolator->technology = tech;
    isolator->num_channels = num_ch;

    if (tech == ISOL_TECH_CAPACITIVE || tech == ISOL_TECH_RF_MODULATED) {
        isolator->barrier = (iso_class == ISOL_CLASS_REINFORCED ||
                             iso_class == ISOL_CLASS_DOUBLE)
                            ? DEFAULT_REINFORCED_CAP : DEFAULT_BASIC_CAP;
    } else if (tech == ISOL_TECH_MAGNETIC ||
               tech == ISOL_TECH_GIANT_MAGNETORESISTIVE) {
        isolator->barrier = DEFAULT_REINFORCED_MAG;
    } else {
        isolator->barrier = DEFAULT_BASIC_OPT;
    }
    isolator->barrier.iso_class = iso_class;

    isolator->channels = (isolator_channel_config_t *)calloc(
        num_ch, sizeof(isolator_channel_config_t));
    if (!isolator->channels) return -1;

    for (uint8_t i = 0; i < num_ch; i++) {
        isolator->channels[i].channel_index = i;
        isolator->channels[i].is_forward = true;
        isolator->channels[i].has_enable = false;
        isolator->channels[i].default_state = false;
        isolator->channels[i].input_threshold_v = 1.5;
        isolator->channels[i].output_voh = 3.1;
        isolator->channels[i].output_vol = 0.1;
    }

    set_default_barrier_model(isolator);
    isolator->ambient_temp_c = 25.0;
    isolator->junction_temp_c = 25.0;
    isolator->operating_hours = 0;
    isolator->side1_powered = false;
    isolator->side2_powered = false;

    snprintf(isolator->part_number, sizeof(isolator->part_number),
             "ISO-%s-%uch",
             tech == ISOL_TECH_CAPACITIVE ? "CAP" :
             tech == ISOL_TECH_MAGNETIC ? "MAG" :
             tech == ISOL_TECH_OPTICAL ? "OPT" : "RF", num_ch);
    return 0;
}

/* L1: digital_isolator_config_channel */
int digital_isolator_config_channel(digital_isolator_t *isolator,
                                    uint8_t ch_idx, bool is_forward,
                                    bool default_st)
{
    if (!isolator || !isolator->channels) return -1;
    if (ch_idx >= isolator->num_channels) return -1;
    isolator->channels[ch_idx].is_forward = is_forward;
    isolator->channels[ch_idx].default_state = default_st;
    return 0;
}

/* L4: paschen_breakdown_voltage ！ Paschen's Law (1889)
 * V_b = B*p*d / (ln(A*p*d) - ln(ln(1+1/gamma)))
 * Air: A=112.5/Pa*m, B=2737.5 V/Pa*m, gamma=0.01
 * Minimum V_b 「 327 V at pd 「 0.75 Pa*m.
 * Ref: Paschen (1889) Annalen der Physik 273(5), 69-96. */
double paschen_breakdown_voltage(double gap_m, double pressure_pa)
{
    const double A=112.5, B=2737.5, gamma=0.01;
    if (gap_m<=0.0 || pressure_pa<=0.0) return 0.0;
    double pd = pressure_pa * gap_m;
    double ln101 = log(1.0 + 1.0/gamma);
    double denom = log(A*pd) - log(ln101);
    if (denom <= 1e-6) return B*pd/1e-6;
    return B * pd / denom;
}

/* L4: minimum_creepage_distance ！ IEC 60664-1 Table F.1
 * Interpolates standard creepage table in log-log space.
 * MG: 0=I(CTI>=600), 1=II(400-600), 2=IIIa(175-400)
 * PD: 1★0.6x, 2★1.0x, 3★1.6x, 4★2.5x
 * Complexity: O(log n) binary search */
double minimum_creepage_distance(double vrms, pollution_degree_t pol, uint8_t mg)
{
    if (vrms <= 0.0) return 0.0;
    static const double vt[]={32,50,63,80,100,125,160,200,250,320,400,500,630,800,1000,1250,1600,2000};
    static const double ct[]={0.45,0.56,0.70,0.89,1.10,1.40,1.80,2.20,2.80,3.60,4.50,5.60,7.10,9.00,11.0,14.0,18.0,22.0};
    static const size_t n=18;
    double mgm=(mg==0)?0.75:(mg==2)?1.40:1.00;
    double pdm=(pol==POLLUTION_DEG_1)?0.6:(pol==POLLUTION_DEG_3)?1.6:(pol==POLLUTION_DEG_4)?2.5:1.0;
    if(vrms<=vt[0]) return ct[0]*mgm*pdm;
    if(vrms>=vt[n-1]) return ct[n-1]*mgm*pdm;
    size_t lo=0,hi=n-1;
    while(lo<hi-1){size_t mid=(lo+hi)/2; if(vt[mid]<=vrms)lo=mid; else hi=mid;}
    double frac=log(vrms/vt[lo])/log(vt[hi]/vt[lo]);
    return ct[lo]*pow(ct[hi]/ct[lo],frac)*mgm*pdm;
}

/* L4: minimum_clearance ！ IEC 60664-1 Table F.2
 * Clearance through air. Inhomogeneous field needs ~4-5x more distance
 * due to E-field concentration at sharp edges. */
double minimum_clearance(double sv_kv, bool hom)
{
    if(sv_kv<=0.0) return 0.0;
    static const double sv[]={0.33,0.5,0.8,1.5,2.5,4.0,6.0,8.0,12.0,20.0,30.0};
    static const double ch[]={0.01,0.02,0.05,0.10,0.20,0.50,1.00,1.60,3.00,6.00,10.0};
    static const double ci[]={0.10,0.20,0.50,0.90,1.80,3.20,5.50,8.00,14.0,25.0,40.0};
    static const size_t n=11;
    const double *cl=hom?ch:ci;
    if(sv_kv<=sv[0]) return cl[0];
    if(sv_kv>=sv[n-1]) return cl[n-1];
    size_t lo=0,hi=n-1;
    while(lo<hi-1){size_t mid=(lo+hi)/2;if(sv[mid]<=sv_kv)lo=mid;else hi=mid;}
    return cl[lo]+(sv_kv-sv[lo])/(sv[hi]-sv[lo])*(cl[hi]-cl[lo]);
}

/* L5: cmti_limited_data_rate
 * CMTI determines minimum resolvable pulse via t_glitch = V_cm / CMTI.
 * R_max = 1/(2*max(t_prop, t_glitch)*1.5). Returns Mbps. */
double cmti_limited_data_rate(const digital_isolator_t *iso)
{
    if(!iso) return 0.0;
    double cmti=iso->barrier.cmti_kv_us;
    if(cmti<=0.0) return iso->barrier.data_rate_mbps;
    double vcm_kv=iso->barrier.working_voltage_rms*1.414/1000.0;
    double tg_ns=(vcm_kv/cmti)*1000.0;
    double tmin=fmax(iso->barrier.propagation_delay_ns,tg_ns);
    double rmax=1000.0/(2.0*tmin*1.5);
    return fmin(rmax,iso->barrier.data_rate_mbps);
}

/* L4: estimate_junction_temperature
 * T_j = T_a + P_total * theta_JA.
 * Typical theta_JA: SOIC-16W=85, SOIC-8=120, BGA=45 degC/W. */
double estimate_junction_temperature(const digital_isolator_t *iso, double pmw, double tja)
{
    if(!iso) return 0.0;
    return iso->ambient_temp_c + (pmw/1000.0)*tja;
}

/* L4: arrhenius_acceleration_factor ！ Arrhenius (1889)
 * AF = exp((Ea/kB)*(1/T_use - 1/T_stress))
 * kB=8.61733e-5 eV/K, T in Kelvin.
 * Example: Ea=0.7eV, 55C vs 125C -> AF「77.5
 * Ref: JEDEC JEP122H */
double arrhenius_acceleration_factor(double tuse_c, double tstress_c, double ea_ev)
{
    const double kB=8.617333262e-5, KO=273.15;
    if(ea_ev<=0.0) return 1.0;
    double tu=tuse_c+KO, ts=tstress_c+KO;
    if(tu<=0.0||ts<=0.0||fabs(tu-ts)<0.001) return 1.0;
    return exp((ea_ev/kB)*(1.0/tu - 1.0/ts));
}

/* L6: is_reinforced_isolation ！ IEC 60747-5-5 compliance check
 * Requires: reinforced class, creepage>=8mm, clearance>=8mm,
 * DTI>=400um, surge>=10kV, Viso>=5kVrms, working>=400Vrms. */
bool is_reinforced_isolation(const digital_isolator_t *iso)
{
    if(!iso) return false;
    if(iso->barrier.iso_class!=ISOL_CLASS_REINFORCED&&iso->barrier.iso_class!=ISOL_CLASS_DOUBLE) return false;
    double rc=minimum_creepage_distance(iso->barrier.working_voltage_rms,iso->barrier.pollution,1);
    if(iso->barrier.creepage_mm<rc) return false;
    double rcl=minimum_clearance(iso->barrier.viso_surge_kv,false);
    if(iso->barrier.clearance_mm<rcl) return false;
    if(iso->barrier.dti_um<400.0) return false;
    if(iso->barrier.viso_surge_kv<10.0) return false;
    if(iso->barrier.viso_rms_kv<5.0) return false;
    if(iso->barrier.working_voltage_rms<400.0) return false;
    return true;
}

/* L2: isolator_power_dissipation
 * P = n_ch*VDD*I_per_ch*0.5*2 + VDD*I_idle. 0.5=activity, 2=sides. */
double isolator_power_dissipation(const digital_isolator_t *iso)
{
    if(!iso) return 0.0;
    double v=iso->barrier.supply_voltage_v, pc=iso->barrier.supply_current_per_ch_ma;
    double id=iso->barrier.idle_current_ma;
    return iso->num_channels*v*pc*0.5*2.0 + v*id;
}

/* L3: barrier_capacitance_compute ！ Parallel plate C = eps0*eps_r*A/d
 * eps0=8.854e-3 fF/um (convenient for um-scale dimensions) */
double barrier_capacitance_compute(double area_um2, double thick_um, double eps_r)
{
    const double E0=8.854e-3;
    if(thick_um<=0.0||area_um2<=0.0) return 0.0;
    return E0*eps_r*area_um2/thick_um;
}

/* L3: barrier_impedance_at_freq ！ |Z|=1/(2*pi*f*C)
 * At 500MHz, 100fF: Z「3.18kOhm. At 60Hz, 100fF: Z「26.5GOhm.
 * Ratio「8.3 million:1 ！ enables galvanic isolation. */
double barrier_impedance_at_freq(double c_ff, double f_hz)
{
    if(f_hz<=0.0||c_ff<=0.0) return 1e12;
    return 1.0/(2.0*M_PI*f_hz*c_ff*1e-15);
}

/* L5: optimal_carrier_frequency ！ For OOK capacitive isolators.
 * f >= 5*data_rate (envelope detection). Also matched to target impedance.
 * Returns MHz. */
double optimal_carrier_frequency(double dr_mbps, double c_ff, double z_ohm)
{
    if(c_ff<=0.0||z_ohm<=0.0) return 500.0;
    double fmin=dr_mbps*1e6*5.0;
    double fz=1.0/(2.0*M_PI*z_ohm*c_ff*1e-15);
    return fmax(fmin,fz)/1e6;
}

/* L2: transformer_coupling_from_geometry
 * k decreases with insulation, increases with area, degrades with turns mismatch. */
double transformer_coupling_from_geometry(double np, double ns, double ins_um, double area_um2)
{
    if(ins_um<=0.0||np<=0.0||ns<=0.0) return 0.0;
    double k=0.95/sqrt(1.0+ins_um/5.0)*(1.0-exp(-area_um2/10000.0));
    return k*fmin(np,ns)/fmax(np,ns);
}

/* L5: propagation_delay_estimate ！ t_pd = R*C*ln(2) + t_barrier */
double propagation_delay_estimate(double r_ohm, double c_pf, double t_barrier_ns)
{
    return r_ohm*c_pf*log(2.0)+t_barrier_ns;
}

/* L6: isolator_compare_fitness ！ Multi-criteria score 0-100
 * 40% data rate + 30% CMTI + 20% isolation + 10% power */
double isolator_compare_fitness(const digital_isolator_t *iso,
                                 double req_rate, double req_cmti, double req_viso)
{
    if(!iso) return 0.0;
    double s=0.0;
    if(req_rate>0.0){double r=iso->barrier.data_rate_mbps/req_rate; s+=fmin(r/2.0,1.0)*40.0;}
    else s+=40.0;
    if(req_cmti>0.0){double r=iso->barrier.cmti_kv_us/req_cmti; s+=fmin(r/2.0,1.0)*30.0;}
    else s+=30.0;
    if(req_viso>0.0){double r=iso->barrier.viso_peak_kv/req_viso; s+=fmin(r/2.0,1.0)*20.0;}
    else s+=20.0;
    double p=isolator_power_dissipation(iso);
    s+=fmax(0.0,1.0-p/100.0)*10.0;
    return s;
}

/* digital_isolator_destroy ！ cleanup */
void digital_isolator_destroy(digital_isolator_t *iso)
{
    if(iso){free(iso->channels); iso->channels=NULL; iso->num_channels=0;}
}
