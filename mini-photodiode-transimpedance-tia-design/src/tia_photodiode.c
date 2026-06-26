/* tia_photodiode.c - Photodiode Physics L1+L3+L4 */
#include "tia_photodiode.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

double photon_energy_ev(double wl_nm){return(wl_nm>0.0)?1239.84/wl_nm:0.0;}
double responsivity_ideal(double wl_nm){return(wl_nm>0.0)?wl_nm/1239.84:0.0;}
double quantum_efficiency_from_r(double R,double wl_nm){return(R>0.0&&wl_nm>0.0)?R*1239.84/wl_nm:0.0;}

double silicon_absorption(double wl_nm){
    if(wl_nm<300.0||wl_nm>1200.0)return 0.0;
    double x=wl_nm/1000.0;
    return exp(14.5-8.2*x+1.5*x*x)*1000.0;
}

double ingaas_absorption(double wl_nm){
    if(wl_nm<700.0||wl_nm>1800.0)return 0.0;
    double x=wl_nm-1650.0;
    return 10000.0*exp(-x*x/40000.0);
}

double penetration_depth(double wl_nm,semiconductor_material_t mat){
    double alpha=0.0;
    switch(mat){
    case MATERIAL_SILICON:alpha=silicon_absorption(wl_nm);break;
    case MATERIAL_INGAAS:alpha=ingaas_absorption(wl_nm);break;
    case MATERIAL_GERMANIUM:alpha=ingaas_absorption(wl_nm)*0.7;break;
    default:alpha=silicon_absorption(wl_nm);break;
    }
    return(alpha>100.0)?1.0e4/alpha:100.0;
}

semiconductor_params_t semiconductor_params_get(semiconductor_material_t mat){
    semiconductor_params_t p;memset(&p,0,sizeof(p));
    p.material=mat;
    switch(mat){
    case MATERIAL_SILICON:p.bandgap_energy_ev=1.12;p.intrinsic_carrier_ni=1.5e10;
        p.electron_mobility=1350.0;p.hole_mobility=450.0;
        p.relative_permittivity=11.7;p.refractive_index=3.42;break;
    case MATERIAL_INGAAS:p.bandgap_energy_ev=0.75;p.intrinsic_carrier_ni=6.3e11;
        p.electron_mobility=8000.0;p.hole_mobility=400.0;
        p.relative_permittivity=13.9;p.refractive_index=3.56;break;
    case MATERIAL_GERMANIUM:p.bandgap_energy_ev=0.66;p.intrinsic_carrier_ni=2.5e13;
        p.electron_mobility=3900.0;p.hole_mobility=1900.0;
        p.relative_permittivity=16.0;p.refractive_index=4.0;break;
    default:p.bandgap_energy_ev=1.12;break;
    }return p;}

pn_junction_model_t pn_junction_compute(semiconductor_material_t mat,double na,double nd,double area,double vr){
    pn_junction_model_t j;memset(&j,0,sizeof(j));
    semiconductor_params_t sp=semiconductor_params_get(mat);
    double Vt=0.02585,ni=sp.intrinsic_carrier_ni>0.0?sp.intrinsic_carrier_ni:1.5e10;
    j.built_in_potential_v=Vt*log(na*nd/(ni*ni));
    double eps=sp.relative_permittivity*8.854187817e-14;
    j.depletion_width_um=sqrt(2.0*eps*(j.built_in_potential_v+vr)/(1.602e-19*na*nd/(na+nd)))*1.0e4;
    j.junction_area_mm2=area;
    j.zero_bias_capacitance_pf=eps*area*1.0e-2/j.depletion_width_um*1.0e12;
    j.doping_na_cm3=na;j.doping_nd_cm3=nd;
    j.grading_coefficient=0.5;
    j.electric_field_max_v_per_cm=(j.built_in_potential_v+vr)/j.depletion_width_um*1.0e4;
    return j;}

double junction_capacitance(const pn_junction_model_t *j,double bias){
    if(!j)return 0.0;
    return j->zero_bias_capacitance_pf/pow(1.0+(bias>0.0?bias:0.0)/j->built_in_potential_v,j->grading_coefficient);}

photodiode_bandwidth_t photodiode_bandwidth_analyze(const photodiode_model_t *pd,double load){
    photodiode_bandwidth_t bw;memset(&bw,0,sizeof(bw));
    if(!pd||load<=0.0)return bw;
    double vsat=1.0e7;
    double wdep=sqrt(2.0*11.7*8.854e-14*0.65/(1.602e-19*1.0e15));
    bw.transit_time_limited_bw_ghz=2.4*vsat/(2.0*M_PI*wdep)*1.0e-5;
    double cj=pd->junction_capacitance_pf*1.0e-12;
    bw.rc_time_limited_bw_ghz=1.0/(2.0*M_PI*load*cj)*1.0e-9;
    bw.total_bandwidth_ghz=1.0/(1.0/bw.transit_time_limited_bw_ghz+1.0/bw.rc_time_limited_bw_ghz);
    bw.carrier_transit_time_ps=wdep/vsat*1.0e12;
    bw.collection_efficiency=0.9;
    return bw;}

apd_model_t apd_model_compute(const photodiode_model_t *pd,double M,double k){
    apd_model_t apd;memset(&apd,0,sizeof(apd));
    if(!pd)return apd;
    apd.multiplication_factor_m=M;apd.k_factor=k;
    apd.breakdown_voltage_v=pd->breakdown_voltage;
    apd.excess_noise_factor_f=M*(1.0-(1.0-k)*(M-1.0)*(M-1.0)/(M*M));
    apd.optimal_gain=sqrt(1.0/(k*(2.0-1.0/M)));
    apd.dark_current_multiplied_na=pd->dark_current_na*M;
    return apd;}

double apd_excess_noise_factor(double M,double k){
    if(M<=1.0)return 1.0;
    return M*(1.0-(1.0-k)*(M-1.0)*(M-1.0)/(M*M));}

absorption_spectrum_t absorption_spectrum_compute(semiconductor_material_t mat,double w1,double w2,size_t n){
    absorption_spectrum_t as;memset(&as,0,sizeof(as));
    if(n<2||w1<=0.0||w2<=w1)return as;
    as.num_points=n;
    as.wavelength_nm=malloc(n*sizeof(double));
    as.absorption_coefficient=malloc(n*sizeof(double));
    as.quantum_efficiency_ideal=malloc(n*sizeof(double));
    as.responsivity_ideal=malloc(n*sizeof(double));
    double dw=(w2-w1)/(double)(n-1);
    for(size_t i=0;i<n;i++){
        double wl=w1+dw*i;
        as.wavelength_nm[i]=wl;
        as.absorption_coefficient[i]=(mat==MATERIAL_SILICON)?silicon_absorption(wl):ingaas_absorption(wl);
        as.quantum_efficiency_ideal[i]=0.8;
        as.responsivity_ideal[i]=responsivity_ideal(wl);
    }
    as.cutoff_wavelength_nm=(mat==MATERIAL_SILICON)?1100.0:1700.0;
    return as;}

double cutoff_wavelength(semiconductor_material_t mat){
    semiconductor_params_t sp=semiconductor_params_get(mat);
    return(sp.bandgap_energy_ev>0.0)?1239.84/sp.bandgap_energy_ev:0.0;}

double photodiode_rise_time_estimate(const photodiode_model_t *pd){
    if(!pd)return 0.0;
    double t_transit=0.01;
    double t_rc=2.2*50.0*(pd->junction_capacitance_pf+pd->package_capacitance_pf)*1.0e-12*1.0e9;
    return 2.2*(t_transit+t_rc);}

void absorption_spectrum_free(absorption_spectrum_t *as){
    if(!as)return;
    free(as->wavelength_nm);free(as->absorption_coefficient);
    free(as->quantum_efficiency_ideal);free(as->responsivity_ideal);
    memset(as,0,sizeof(*as));}