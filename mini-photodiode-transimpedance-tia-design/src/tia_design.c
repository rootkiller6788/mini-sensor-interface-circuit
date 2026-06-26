/* tia_design.c - TIA Design Methodology L5+L6+L7 */
#include "tia_design.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

tia_design_result_t tia_design_from_spec(const tia_specification_t *spec,photodiode_type_t pd_type){
    tia_design_result_t r;memset(&r,0,sizeof(r));
    if(!spec){snprintf(r.failure_reason,128,"null spec");return r;}
    photodiode_model_t pd=photodiode_model_init(pd_type,1.0,spec->supply_voltage_v>5.0?5.0:0.0);
    if(pd_type==PHOTODIODE_CUSTOM){
        if(spec->wavelength_nm<1000.0)pd=photodiode_model_init(PHOTODIODE_SI_PIN,1.0,3.0);
        else pd=photodiode_model_init(PHOTODIODE_INGAAS_PIN,1.0,3.0);
    }
    opamp_params_t opa=opamp_params_init("OPA657");
    if(spec->target_bandwidth_mhz<10.0)opa=opamp_params_init("OPA656");
    else if(spec->max_power_mw<50.0)opa=opamp_params_init("LMP7721");
    r.design=tia_design_basic(&pd,&opa,spec->target_gain_v_per_a,spec->target_bandwidth_mhz*1.0e6);
    r.noise=tia_noise_analyze(&r.design,0.0,1.0,r.design.bandwidth_3db_hz);
    r.stability=tia_loop_gain_analyze(&r.design);
    r.freq_resp=tia_compute_frequency_response(&r.design,100.0,1.0e9,200);
    r.step_resp=tia_compute_step_response(&r.design,100.0,1000.0,200);
    r.summary=tia_performance_summary(&r.design);
    r.meets_specification=1;
    if(r.design.bandwidth_3db_mhz<spec->target_bandwidth_mhz*0.9){
        r.meets_specification=0;
        snprintf(r.failure_reason,128,"BW fail: %.1f<%.1f",r.design.bandwidth_3db_mhz,spec->target_bandwidth_mhz);
    }
    return r;
}

tia_design_t tia_design_high_speed(const photodiode_model_t *pd,const opamp_params_t *opa,double bw_target){
    tia_design_t d;memset(&d,0,sizeof(d));
    if(!pd||!opa||bw_target<=0.0)return d;
    double cin=tia_input_capacitance(pd,opa,0.3);
    double gbw=opa->gain_bandwidth_mhz*1.0e6;
    double two_pi=2.0*M_PI;
    double rf_max=1.0/(two_pi*cin*1.0e-12*bw_target*1.0e6);
    double rf=rf_max*0.7;
    if(rf<100.0)rf=100.0;
    return tia_design_basic(pd,opa,rf,bw_target*1.0e6);
}

tia_design_t tia_design_low_noise(const photodiode_model_t *pd,const opamp_params_t *opa,double gain){
    tia_design_t d;memset(&d,0,sizeof(d));
    if(!pd||!opa||gain<=0.0)return d;
    double cin=tia_input_capacitance(pd,opa,0.5);
    double gbw=opa->gain_bandwidth_mhz*1.0e6;
    double two_pi=2.0*M_PI;
    double rf_max=sqrt(gbw/(two_pi*cin*1.0e-12*1.0e3));
    double rf=(gain<rf_max)?gain:rf_max;
    return tia_design_basic(pd,opa,rf,0.0);
}

tia_design_t tia_design_wide_dynamic(const photodiode_model_t *pd,const opamp_params_t *opa,double imin,double imax){
    tia_design_t d;memset(&d,0,sizeof(d));
    if(!pd||!opa||imin<=0.0||imax<=imin)return d;
    double vswing=opa->output_voltage_swing_v*0.8;
    double rf=vswing/(imax*1.0e-6);
    return tia_design_basic(pd,opa,rf,0.0);
}

tia_design_t tia_design_low_power(const photodiode_model_t *pd,const opamp_params_t *opa,double gain,double pmax){
    tia_design_t d;memset(&d,0,sizeof(d));
    if(!pd||!opa||gain<=0.0||pmax<=0.0)return d;
    opamp_params_t op=*opa;
    if(op.supply_current_ma*op.supply_voltage_max>pmax)op=opamp_params_init("LMP7721");
    return tia_design_basic(pd,&op,gain,0.0);
}

tia_design_result_t tia_select_best_design(const tia_design_candidate_t *c,size_t n,const tia_optimization_weights_t *w,size_t *best){
    tia_design_result_t r;memset(&r,0,sizeof(r));
    if(!c||n==0){if(best)*best=0;return r;}
    double bs=-1e30;size_t bi=0;
    for(size_t i=0;i<n;i++){
        double sc=c[i].gain_ohm*w->weight_gain+c[i].bandwidth_mhz*w->weight_bandwidth-
                  c[i].noise_pa_per_sqrt_hz*w->weight_noise-c[i].power_mw*w->weight_power-
                  c[i].cost_usd*w->weight_cost;
        if(sc>bs){bs=sc;bi=i;}
    }
    if(best)*best=bi;
    return r;
}

tia_pareto_set_t tia_pareto_optimize(const tia_design_candidate_t *c,size_t n){
    tia_pareto_set_t ps;memset(&ps,0,sizeof(ps));
    if(!c||n==0)return ps;
    ps.num_designs=n;
    ps.designs=calloc(n,sizeof(tia_design_result_t));
    ps.pareto_front_indices=malloc(n*sizeof(size_t));
    ps.num_pareto=(n>0)?1:0;
    if(n>0)ps.pareto_front_indices[0]=0;
    return ps;
}

int tia_verify_specification(const tia_design_t *d,const tia_specification_t *spec,char *fail){
    if(!d||!spec)return 0;
    if(d->bandwidth_3db_mhz<spec->target_bandwidth_mhz*0.9){if(fail)snprintf(fail,128,"BW fail");return 0;}
    if(d->transimpedance_gain_ohm<spec->target_gain_v_per_a*0.9){if(fail)snprintf(fail,128,"Gain fail");return 0;}
    return 1;
}

const char* tia_recommend_opamp(const photodiode_model_t *pd,double gain,double bw,int ln){
    (void)pd;(void)gain;
    if(bw>100.0e6)return ln?"ADA4817":"OPA657";
    if(bw>10.0e6)return ln?"LTC6268":"OPA656";
    return ln?"OPA827":"OPA380";
}

photodiode_type_t tia_recommend_photodiode(double wl,double bw,int ln){
    (void)ln;
    if(wl<500.0)return PHOTODIODE_SI_PIN;
    if(wl<1000.0)return(bw>100.0e6)?PHOTODIODE_SI_APD:PHOTODIODE_SI_PIN;
    return(bw>1.0e9)?PHOTODIODE_INGAAS_APD:PHOTODIODE_INGAAS_PIN;
}

double tia_figure_of_merit(const tia_design_t *d){
    if(!d||d->bandwidth_3db_hz<=0.0)return 0.0;
    double pwr=d->opamp.supply_current_ma*d->opamp.supply_voltage_max*1.0e-3;
    if(pwr<=0.0)pwr=1.0e-3;
    return d->transimpedance_gain_ohm*d->bandwidth_3db_hz/(d->input_noise_density_pa*1.0e-12*sqrt(pwr));
}

void tia_design_report(const tia_design_result_t *r){
    if(!r)return;
    printf("=== TIA Design Report ===\n");
    printf("PD: %s, OpAmp: %s\n",r->design.photodiode.model_name,r->design.opamp.part_number);
    printf("Gain: %.0f ohm (%.1f dB)\n",r->design.transimpedance_gain_ohm,r->design.transimpedance_gain_db);
    printf("BW:   %.2f MHz\n",r->design.bandwidth_3db_mhz);
    printf("PM:   %.1f deg\n",r->design.phase_margin_deg);
    printf("Noise: %.1f pA/rtHz, %.1f pA_rms\n",r->design.input_noise_density_pa,r->design.total_input_noise_pa);
    printf("Sens: %.1f dBm\n",r->design.sensitivity_dbm);
    printf("Pass: %s\n",r->meets_specification?"YES":"NO");
    if(!r->meets_specification)printf("Reason: %s\n",r->failure_reason);
}

void tia_design_result_free(tia_design_result_t *r){
    if(!r)return;
    tia_freq_response_free(&r->freq_resp);
    tia_step_response_free(&r->step_resp);
}

void tia_pareto_set_free(tia_pareto_set_t *ps){
    if(!ps)return;
    free(ps->designs);free(ps->pareto_front_indices);
    memset(ps,0,sizeof(*ps));
}
