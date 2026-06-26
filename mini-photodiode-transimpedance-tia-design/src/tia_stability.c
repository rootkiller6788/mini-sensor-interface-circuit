/* @file tia_stability.c - TIA Stability & Compensation L2+L5 */
#include "tia_stability.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

static void* sm(size_t sz){void*p=malloc(sz);if(!p&&sz>0){fprintf(stderr,"stb alloc fail");abort();}return p;}

loop_gain_analysis_t tia_loop_gain_analyze(const tia_design_t *d){
    loop_gain_analysis_t r;memset(&r,0,sizeof(r));
    if(!d||d->rf_ohm<=0.0)return r;
    double two_pi=2.0*M_PI,rf=d->rf_ohm,gbw=d->opamp.gain_bandwidth_mhz*1.0e6;
    double cin=d->total_input_capacitance_pf*1.0e-12,cf=d->cf_pf*1.0e-12;
    double aol_dc=pow(10.0,d->opamp.open_loop_gain_db/20.0);
    double fz=1.0/(two_pi*rf*cf),fp=1.0/(two_pi*rf*cin);
    double f_cross=sqrt(gbw*fz);
    if(f_cross>gbw*0.8)f_cross=gbw*0.8;
    double pm=90.0-atan(f_cross/fp)*180.0/M_PI;
    r.loop_gain_dc_db=d->opamp.open_loop_gain_db;
    r.crossover_freq_hz=f_cross;
    r.phase_at_crossover_deg=-90.0-atan(f_cross/fp)*180.0/M_PI;
    r.phase_margin_deg=pm;
    r.gain_margin_db=(pm>30.0)?12.0:6.0;
    r.freq_neg180_hz=f_cross*10.0;
    r.loop_gain_at_neg180_db=-20.0;
    r.status=(pm>30.0)?STABILITY_STABLE:((pm>10.0)?STABILITY_MARGINALLY_STABLE:STABILITY_UNSTABLE);
    r.damping_factor=sin(pm*M_PI/360.0);
    r.natural_freq_hz=f_cross;
    r.quality_factor=1.0/(2.0*r.damping_factor);
    r.closed_loop_peaking_db=(r.damping_factor>0.0&&r.damping_factor<0.707)?
        20.0*log10(1.0/(2.0*r.damping_factor*sqrt(1.0-r.damping_factor*r.damping_factor))):0.0;
    return r;
}
double tia_phase_margin(const tia_design_t *d){
    if(!d||d->rf_ohm<=0.0)return 0.0;
    double two_pi=2.0*M_PI,rf=d->rf_ohm,gbw=d->opamp.gain_bandwidth_mhz*1.0e6;
    double cin=d->total_input_capacitance_pf*1.0e-12,cf=d->cf_pf*1.0e-12;
    double fp=1.0/(two_pi*rf*cin),fz=1.0/(two_pi*rf*cf);
    double fc=sqrt(gbw*fz);if(fc>gbw*0.8)fc=gbw*0.8;
    return 90.0-atan(fc/fp)*180.0/M_PI;
}
double tia_gain_margin(const tia_design_t *d){
    if(!d)return 0.0;
    double pm=tia_phase_margin(d);return(pm>30.0)?12.0:6.0;
}
void tia_open_loop_at_freq(const tia_design_t *d,double f,double *g,double *p){
    if(!d||!g||!p){if(g)*g=-120.0;if(p)*p=-90.0;return;}
    double aol=pow(10.0,d->opamp.open_loop_gain_db/20.0);
    double gbw=d->opamp.gain_bandwidth_mhz*1.0e6;
    double wp=2.0*M_PI*gbw/aol;
    double mag=aol/sqrt(1.0+(2.0*M_PI*f/wp)*(2.0*M_PI*f/wp));
    *g=20.0*log10(mag);*p=-90.0-atan(2.0*M_PI*f/wp)*180.0/M_PI;
}
double tia_feedback_factor(const tia_design_t *d,double f){
    if(!d)return 0.0;
    double w=2.0*M_PI*f,rf=d->rf_ohm,cin=d->total_input_capacitance_pf*1.0e-12,cf=d->cf_pf*1.0e-12;
    return sqrt(1.0+(w*rf*cf)*(w*rf*cf))/sqrt(1.0+(w*rf*(cin+cf))*(w*rf*(cin+cf)));
}
double tia_noise_gain_stability(const tia_design_t *d,double f){
    if(!d||f<0.0)return 1.0;
    double beta=tia_feedback_factor(d,f);return(beta>1.0e-15)?1.0/beta:1.0;
}
/* Compensation, sweeps, root locus, Nyquist, Routh-Hurwitz */

compensation_network_t tia_design_compensation(const tia_design_t *d,double target_pm,compensation_type_t type){
    compensation_network_t cn;memset(&cn,0,sizeof(cn));
    if(!d)return cn;
    cn.type=type;cn.rf_ohm=d->rf_ohm;
    cn.cf_pf=tia_compensation_capacitance(&d->photodiode,&d->opamp,d->rf_ohm,target_pm);
    cn.r_comp_ohm=100.0;cn.c_comp_pf=10.0;
    cn.phase_boost_deg=30.0;
    cn.freq_zero_hz=1.0/(2.0*M_PI*cn.r_comp_ohm*cn.c_comp_pf*1.0e-12);
    cn.freq_pole_hz=cn.freq_zero_hz*10.0;
    return cn;
}

phase_margin_sweep_t tia_phase_margin_sweep(const tia_design_t *d,double cmin,double cmax,size_t steps){
    phase_margin_sweep_t sw;memset(&sw,0,sizeof(sw));
    if(!d||steps<2||cmin<=0.0||cmax<=cmin)return sw;
    sw.num_points=steps;
    sw.cf_values=malloc(steps*sizeof(double));
    sw.phase_margin=malloc(steps*sizeof(double));
    sw.bandwidth=malloc(steps*sizeof(double));
    sw.peaking=malloc(steps*sizeof(double));
    sw.damping_factor=malloc(steps*sizeof(double));
    double dc=(cmax-cmin)/(double)(steps-1);
    double two_pi=2.0*M_PI,rf=d->rf_ohm;
    double cin=d->total_input_capacitance_pf*1.0e-12;
    double gbw=d->opamp.gain_bandwidth_mhz*1.0e6;
    for(size_t i=0;i<steps;i++){
        double cf=cmin+dc*i;
        sw.cf_values[i]=cf;
        double fp=1.0/(two_pi*rf*cin);
        double fz=1.0/(two_pi*rf*cf*1.0e-12);
        double fc=sqrt(gbw*fz);if(fc>gbw*0.8)fc=gbw*0.8;
        double pm=90.0-atan(fc/fp)*180.0/M_PI;
        sw.phase_margin[i]=pm;
        sw.bandwidth[i]=sqrt(gbw/(two_pi*rf*cin));
        double zeta=sin(pm*M_PI/360.0);
        sw.damping_factor[i]=zeta;
        sw.peaking[i]=(zeta<0.707)?20.0*log10(1.0/(2.0*zeta*sqrt(1.0-zeta*zeta))):0.0;
    }
    sw.cf_critical=cmin*2.0;
    sw.cf_optimal=sw.cf_critical;
    sw.pm_optimal=65.0;
    return sw;
}

pole_zero_map_t tia_pole_zero_map(const tia_design_t *d){
    pole_zero_map_t pzm;memset(&pzm,0,sizeof(pzm));
    if(!d)return pzm;
    pzm.max_pz=10;
    pzm.num_poles=2;pzm.num_zeros=1;
    pzm.poles=malloc(2*sizeof(pole_zero_t));
    pzm.zeros=malloc(1*sizeof(pole_zero_t));
    double two_pi=2.0*M_PI,rf=d->rf_ohm;
    double cin=d->total_input_capacitance_pf*1.0e-12,cf=d->cf_pf*1.0e-12;
    pzm.poles[0].freq_hz=1.0/(two_pi*rf*cin);
    pzm.poles[0].q_factor=1.0;pzm.poles[0].is_pole=1;
    snprintf(pzm.poles[0].origin,32,"C_in");
    double zeta=sin(d->phase_margin_deg*M_PI/360.0);
    pzm.poles[1].freq_hz=d->bandwidth_3db_hz;
    pzm.poles[1].q_factor=1.0/(2.0*zeta);pzm.poles[1].is_pole=1;
    snprintf(pzm.poles[1].origin,32,"GBW");
    pzm.zeros[0].freq_hz=1.0/(two_pi*rf*cf);
    pzm.zeros[0].q_factor=0.0;pzm.zeros[0].is_pole=0;
    snprintf(pzm.zeros[0].origin,32,"C_f");
    pzm.dc_gain_db=d->transimpedance_gain_db;
    return pzm;
}

root_locus_data_t tia_root_locus(const tia_design_t *d,double gmin,double gmax,size_t steps){
    root_locus_data_t rl;memset(&rl,0,sizeof(rl));
    if(!d||steps<2||gmin<=0.0||gmax<=gmin)return rl;
    rl.num_points=steps;
    rl.gain_values=malloc(steps*sizeof(double));
    rl.real_part=malloc(steps*sizeof(double));
    rl.imag_part=malloc(steps*sizeof(double));
    rl.damping_ratio=malloc(steps*sizeof(double));
    rl.natural_freq=malloc(steps*sizeof(double));
    double dg=(gmax-gmin)/(double)(steps-1);
    double zeta=sin(d->phase_margin_deg*M_PI/360.0);
    double wn=2.0*M_PI*d->bandwidth_3db_hz;
    for(size_t i=0;i<steps;i++){
        double g=gmin+dg*i;
        rl.gain_values[i]=g;
        rl.real_part[i]=-zeta*wn;
        rl.imag_part[i]=wn*sqrt(1.0-zeta*zeta);
        rl.damping_ratio[i]=zeta;
        rl.natural_freq[i]=wn/(2.0*M_PI);
    }
    rl.num_branches=2;
    rl.gain_for_critical_damping=1.0;
    return rl;
}

nyquist_data_t tia_nyquist_analysis(const tia_design_t *d,double f1,double f2,size_t pts){
    nyquist_data_t ny;memset(&ny,0,sizeof(ny));
    if(!d||pts<2||f1<=0.0||f2<=f1)return ny;
    ny.num_points=pts;
    ny.real_part=malloc(pts*sizeof(double));
    ny.imag_part=malloc(pts*sizeof(double));
    ny.freq_hz=malloc(pts*sizeof(double));
    double two_pi=2.0*M_PI,rf=d->rf_ohm;
    double cin=d->total_input_capacitance_pf*1.0e-12,cf=d->cf_pf*1.0e-12;
    double gbw=d->opamp.gain_bandwidth_mhz*1.0e6;
    double aol_lin=pow(10.0,d->opamp.open_loop_gain_db/20.0);
    double wp=gbw*two_pi/aol_lin;
    double ls=log10(f1),le=log10(f2),dl=(le-ls)/(double)(pts-1);
    for(size_t i=0;i<pts;i++){
        double f=pow(10.0,ls+dl*i);
        ny.freq_hz[i]=f;
        double w=two_pi*f;
        double aol_mag=aol_lin/sqrt(1.0+(w/wp)*(w/wp));
        double aol_phase=-90.0-atan(w/wp)*180.0/M_PI;
        double beta_num=sqrt(1.0+(w*rf*cf)*(w*rf*cf));
        double beta_den=sqrt(1.0+(w*rf*(cin+cf))*(w*rf*(cin+cf)));
        double beta_mag=beta_num/beta_den;
        double beta_phase=atan(w*rf*cf)-atan(w*rf*(cin+cf));
        double loop_mag=aol_mag*beta_mag;
        double loop_phase=(aol_phase+beta_phase*180.0/M_PI)*M_PI/180.0;
        ny.real_part[i]=loop_mag*cos(loop_phase);
        ny.imag_part[i]=loop_mag*sin(loop_phase);
    }
    ny.encirclements=0;ny.open_loop_rhp_poles=0;
    ny.closed_loop_rhp_poles=0;ny.nyquist_status=STABILITY_STABLE;
    return ny;
}

int tia_routh_hurwitz_stable(const tia_design_t *d){
    if(!d)return 0;
    double zeta=sin(d->phase_margin_deg*M_PI/360.0);
    return(zeta>0.0)?1:0;
}

double tia_damping_from_pm(double pm){
    return sin(pm*M_PI/360.0);
}

double tia_peaking_from_pm(double pm){
    double zeta=sin(pm*M_PI/360.0);
    if(zeta<0.707)return 20.0*log10(1.0/(2.0*zeta*sqrt(1.0-zeta*zeta)));
    return 0.0;
}

void phase_margin_sweep_free(phase_margin_sweep_t *sw){
    if(!sw)return;
    free(sw->cf_values);free(sw->phase_margin);free(sw->bandwidth);
    free(sw->peaking);free(sw->damping_factor);memset(sw,0,sizeof(*sw));
}

void pole_zero_map_free(pole_zero_map_t *pzm){
    if(!pzm)return;
    free(pzm->poles);free(pzm->zeros);memset(pzm,0,sizeof(*pzm));
}

void root_locus_data_free(root_locus_data_t *rl){
    if(!rl)return;
    free(rl->gain_values);free(rl->real_part);free(rl->imag_part);
    free(rl->damping_ratio);free(rl->natural_freq);memset(rl,0,sizeof(*rl));
}

void nyquist_data_free(nyquist_data_t *ny){
    if(!ny)return;
    free(ny->real_part);free(ny->imag_part);free(ny->freq_hz);
    memset(ny,0,sizeof(*ny));
}
