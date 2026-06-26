/* tia_noise.c - TIA Noise Analysis */
#include "tia_noise.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

static void* safe_malloc(size_t sz) {
    void *p = malloc(sz);
    if (!p && sz > 0) { fprintf(stderr, "noise alloc fail"); abort(); }
    return p;
}

double noise_johnson_voltage(double R, double T) {
    if (R <= 0.0 || T <= 0.0) return 0.0;
    return sqrt(4.0 * BOLTZMANN_CONSTANT * T * R);
}

double noise_johnson_current(double R, double T) {
    if (R <= 0.0 || T <= 0.0) return 0.0;
    return sqrt(4.0 * BOLTZMANN_CONSTANT * T / R);
}

double noise_shot(double I_dc) {
    if (I_dc <= 0.0) return 0.0;
    return sqrt(2.0 * ELECTRON_CHARGE * I_dc);
}

double noise_ktc_rms(double C, double T) {
    if (C <= 0.0 || T <= 0.0) return 0.0;
    return sqrt(BOLTZMANN_CONSTANT * T / C);
}

double tia_noise_gain(const tia_design_t *d, double f_hz) {
    if (!d || f_hz < 0.0) return 1.0;
    double w=2.0*M_PI*f_hz, rf=d->rf_ohm;
    double cin=d->total_input_capacitance_pf*1.0e-12, cf=d->cf_pf*1.0e-12;
    double num=sqrt(1.0+(w*rf*(cin+cf))*(w*rf*(cin+cf)));
    double den=sqrt(1.0+(w*rf*cf)*(w*rf*cf));
    return (den>1.0e-15)?num/den:1.0;
}

tia_noise_model_t tia_noise_analyze(const tia_design_t *d, double i_sig_ua, double flo, double fhi) {
    tia_noise_model_t m;
    memset(&m, 0, sizeof(m));
    if (!d || flo <= 0.0 || fhi <= flo) return m;
    double rf = d->rf_ohm, two_pi = 2.0*M_PI;
    double en = d->opamp.input_voltage_noise_nv*1.0e-9;
    double inn = d->opamp.input_current_noise_fa*1.0e-15;
    double idark = d->photodiode.dark_current_na*1.0e-9;
    double isig = i_sig_ua*1.0e-6;
    double fj = (flo+fhi)*0.5, bw = fhi-flo, cin = d->total_input_capacitance_pf*1.0e-12;
    double j_rf = sqrt(4.0*BOLTZMANN_CONSTANT*TEMPERATURE_STANDARD/rf);
    double j_rsh = sqrt(4.0*BOLTZMANN_CONSTANT*TEMPERATURE_STANDARD/d->photodiode.shunt_resistance_ohm);
    double s_dark = sqrt(2.0*ELECTRON_CHARGE*idark);
    double s_sig = sqrt(2.0*ELECTRON_CHARGE*isig);
    double en_cin = en*two_pi*fj*cin;
    double flicker = 0.0;
    if (d->opamp.corner_freq_1f_hz > 0.0 && fj > 0.0)
        flicker = en*sqrt(d->opamp.corner_freq_1f_hz/fj)*two_pi*fj*cin;
    double tot_dens = sqrt(j_rf*j_rf+j_rsh*j_rsh+s_dark*s_dark+s_sig*s_sig+en_cin*en_cin+inn*inn+flicker*flicker);
    double bw_eff = (bw>0.0)?bw:d->bandwidth_3db_hz;
    m.total_input_noise_pa = tot_dens*sqrt(bw_eff)*1.0e12;
    m.total_output_noise_uv = m.total_input_noise_pa*rf*1.0e-6;
    m.total_output_noise_mvpp = m.total_output_noise_uv*6.0*1.0e-3;
    m.nepo_system_w_per_sqrt_hz = tot_dens/d->photodiode.responsivity_a_per_w;
    double jonly = j_rf*j_rf*bw_eff;
    double tpower = tot_dens*tot_dens*bw_eff;
    m.system_noise_figure_db = (jonly>0.0)?10.0*log10(tpower/jonly):50.0;
    m.noise_bandwidth_hz = bw_eff;
    m.noise_corner_hz = d->opamp.corner_freq_1f_hz;
    m.johnson_rf_density = j_rf*rf;
    m.shot_dark_density = s_dark;
    m.shot_signal_density = s_sig;
    m.opamp_en_density = en;
    m.opamp_in_density = inn;
    m.opamp_en_cin_density = en_cin;
    m.flicker_noise_density = flicker;
    double dens[9] = {j_rf,j_rsh,s_dark,s_sig,en,inn,en_cin,flicker,0.0};
    const char *nms[9] = {"J_Rf","J_Rsh","S_Dark","S_Sig","e_n","i_n","enCin","1f","kTC"};
    double mx=0.0, tp=0.0;
    for(int i=0;i<9;i++) {
        snprintf(m.contributions[i].name,32,"%s",nms[i]);
        m.contributions[i].source_type = (noise_source_type_t)i;
        m.contributions[i].spectral_density_at_1khz = dens[i];
        m.contributions[i].integrated_noise_uv = dens[i]*sqrt(bw_eff)*rf*1.0e6;
        m.contributions[i].is_white = (i!=7);
        tp += dens[i]*dens[i];
        if(dens[i]*dens[i]>mx){mx=dens[i]*dens[i];m.dominant_noise_source=i;}
    }
    for(int i=0;i<9;i++) m.contributions[i].percent_of_total = (tp>0.0)?dens[i]*dens[i]/tp*100.0:0.0;
    return m;
}


/* Noise spectrum, SNR, noise figure, dominant source, etc. */

tia_noise_spectrum_t tia_noise_spectrum(const tia_design_t *d,double i_sig_ua,double f1,double f2,size_t pts){
    tia_noise_spectrum_t ns;memset(&ns,0,sizeof(ns));
    if(!d||pts<2||f1<=0.0||f2<=f1)return ns;
    ns.num_points=pts;
    ns.freq_hz=malloc(pts*sizeof(double));
    ns.output_psd_v2_per_hz=malloc(pts*sizeof(double));
    ns.input_psd_a2_per_hz=malloc(pts*sizeof(double));
    ns.noise_gain=malloc(pts*sizeof(double));
    for(int i=0;i<NOISE_SOURCE_COUNT;i++)ns.contributions[i]=malloc(pts*sizeof(double));
    double rf=d->rf_ohm,en=d->opamp.input_voltage_noise_nv*1.0e-9;
    double inn=d->opamp.input_current_noise_fa*1.0e-15;
    double idark=d->photodiode.dark_current_na*1.0e-9;
    double isig=i_sig_ua*1.0e-6,two_pi=2.0*M_PI;
    double ji=noise_johnson_current(rf,TEMPERATURE_STANDARD);
    double sd=noise_shot(idark),ss=noise_shot(isig);
    double ls=log10(f1),le=log10(f2),dl=(le-ls)/(double)(pts-1);
    for(size_t i=0;i<pts;i++){
        double f=pow(10.0,ls+dl*i);
        ns.freq_hz[i]=f;
        double ng=tia_noise_gain(d,f);
        ns.noise_gain[i]=ng;
        double cj=ji*ji,csd=sd*sd,css=ss*ss,ci=inn*inn;
        double cec=en*two_pi*f*d->total_input_capacitance_pf*1.0e-12;
        cec*=cec;
        double cfk=0.0;
        if(d->opamp.corner_freq_1f_hz>0.0&&f>0.0)cfk=(en*ng)*(en*ng)*d->opamp.corner_freq_1f_hz/f;
        double ipsd=cj+csd+css+ci+cec+cfk;
        ns.input_psd_a2_per_hz[i]=ipsd;
        ns.output_psd_v2_per_hz[i]=ipsd*rf*rf*ng*ng;
        ns.contributions[0][i]=cj;ns.contributions[2][i]=csd;
        ns.contributions[3][i]=css;ns.contributions[5][i]=ci;
        ns.contributions[6][i]=cec;ns.contributions[7][i]=cfk;
    }
    return ns;
}

double tia_input_referred_noise(const tia_noise_model_t *m,double bw){
    if(!m||bw<=0.0)return 0.0;
    return m->total_input_noise_pa*1.0e-12;
}

double tia_output_snr(const tia_noise_model_t *m,double i_sig_ua,double bw){
    if(!m||i_sig_ua<=0.0||bw<=0.0)return 0.0;
    double sig=i_sig_ua*1.0e-6;
    double ns=m->total_input_noise_pa*1.0e-12*sqrt(bw);
    return(ns>0.0)?20.0*log10(sig/ns):100.0;
}

double tia_noise_figure(const tia_noise_model_t *m){
    return m?m->system_noise_figure_db:0.0;
}

int tia_noise_dominant_source(const tia_noise_model_t *m){
    return m?(int)m->dominant_noise_source:-1;
}

double tia_optimal_rf_for_noise(const photodiode_model_t *pd,const opamp_params_t *opa,double bw_target){
    if(!pd||!opa||bw_target<=0.0)return 1.0e3;
    double cin=(pd->junction_capacitance_pf+pd->package_capacitance_pf+opa->input_capacitance_cm_pf+opa->input_capacitance_diff_pf+0.5)*1.0e-12;
    double gbw=opa->gain_bandwidth_mhz*1.0e6,two_pi=2.0*M_PI;
    double rf=1.0/(two_pi*cin*bw_target);
    double cf=sqrt(cin/(two_pi*rf*gbw));
    for(int i=0;i<10;i++){
        double fchk=1.0/(two_pi*rf*cf);
        if(fchk>=bw_target)break;
        rf*=0.9;cf=sqrt(cin/(two_pi*rf*gbw));
        if(cf<0.01e-12)cf=0.01e-12;
    }
    return rf;
}

tia_noise_sweep_t tia_noise_optimize_rf(const tia_design_t *base,const tia_noise_optimization_t *opt){
    tia_noise_sweep_t sw;memset(&sw,0,sizeof(sw));
    if(!base||!opt)return sw;
    double rf_min=(opt->rf_min_ohm>0.0)?opt->rf_min_ohm:1.0e3;
    double rf_max=(opt->rf_max_ohm>rf_min)?opt->rf_max_ohm:rf_min*100.0;
    double factor=(opt->rf_step_factor>1.0)?opt->rf_step_factor:1.2;
    size_t steps=0;
    for(double r=rf_min;r<=rf_max;r*=factor)steps++;
    if(steps<2)steps=2;
    sw.num_points=steps;
    sw.rf_values=malloc(steps*sizeof(double));
    sw.total_noise_pa=malloc(steps*sizeof(double));
    sw.bandwidth_mhz=malloc(steps*sizeof(double));
    sw.johnson_noise_pa=malloc(steps*sizeof(double));
    sw.shot_noise_pa=malloc(steps*sizeof(double));
    sw.opamp_noise_pa=malloc(steps*sizeof(double));
    double best_noise=1.0e30,two_pi=2.0*M_PI;
    double cin=base->total_input_capacitance_pf*1.0e-12;
    double gbw=base->opamp.gain_bandwidth_mhz*1.0e6;
    size_t idx=0;
    for(double rf=rf_min;rf<=rf_max&&idx<steps;rf*=factor,idx++){
        double f3db=sqrt(gbw/(two_pi*rf*cin));
        double cf=sqrt(cin/(two_pi*rf*gbw));
        double fcf=1.0/(two_pi*rf*cf);
        if(fcf<f3db)f3db=fcf;
        double ji=sqrt(4.0*BOLTZMANN_CONSTANT*TEMPERATURE_STANDARD/rf)*sqrt(f3db)*1.0e12;
        double si=sqrt(2.0*ELECTRON_CHARGE*base->photodiode.dark_current_na*1.0e-9)*sqrt(f3db)*1.0e12;
        double tot=sqrt(ji*ji+si*si);
        sw.rf_values[idx]=rf;
        sw.total_noise_pa[idx]=tot;
        sw.bandwidth_mhz[idx]=f3db/1.0e6;
        sw.johnson_noise_pa[idx]=ji;
        sw.shot_noise_pa[idx]=si;
        sw.opamp_noise_pa[idx]=0.0;
        if(tot<best_noise){best_noise=tot;sw.optimal_rf=rf;sw.optimal_noise_pa=tot;sw.optimal_bw_mhz=f3db/1.0e6;}
    }
    sw.num_points=idx;
    return sw;
}

void tia_noise_spectrum_free(tia_noise_spectrum_t *ns){
    if(!ns)return;
    free(ns->freq_hz);free(ns->output_psd_v2_per_hz);
    free(ns->input_psd_a2_per_hz);free(ns->noise_gain);
    for(int i=0;i<NOISE_SOURCE_COUNT;i++)free(ns->contributions[i]);
    memset(ns,0,sizeof(*ns));
}

void tia_noise_sweep_free(tia_noise_sweep_t *sw){
    if(!sw)return;
    free(sw->rf_values);free(sw->total_noise_pa);
    free(sw->bandwidth_mhz);free(sw->johnson_noise_pa);
    free(sw->shot_noise_pa);free(sw->opamp_noise_pa);
    memset(sw,0,sizeof(*sw));
}
