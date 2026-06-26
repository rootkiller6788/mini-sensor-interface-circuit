/* tia_advanced.c - Advanced TIA Topics L7+L8+L9 */
#include "tia_advanced.h"
#include "tia_noise.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* L7: Fiber Optic Receiver Analysis */
fiber_receiver_perf_t tia_fiber_receiver_analyze(const tia_design_t *d,const fiber_receiver_spec_t *spec){
    fiber_receiver_perf_t p;memset(&p,0,sizeof(p));
    if(!d||!spec)return p;
    double total_loss=spec->fiber_length_km*spec->fiber_attenuation_db_per_km+spec->connector_loss_db+spec->dispersion_penalty_db;
    p.received_power_dbm=spec->transmitter_power_dbm-total_loss;
    p.sensitivity_dbm=tia_sensitivity(d,spec->required_ber,spec->data_rate_gbps*1.0e9);
    p.link_margin_db=p.received_power_dbm-p.sensitivity_dbm;
    double q=7.0;
    p.estimated_ber=0.5*erfc(q*pow(10.0,p.link_margin_db/20.0)/sqrt(2.0));
    if(p.estimated_ber<1.0e-30)p.estimated_ber=1.0e-30;
    p.eye_opening_percent=(p.link_margin_db>0.0)?100.0:100.0*pow(10.0,p.link_margin_db/10.0);
    p.jitter_ps=10.0;
    p.power_penalty_db=(spec->extinction_ratio_db>0.0)?10.0*log10((spec->extinction_ratio_db+1.0)/(spec->extinction_ratio_db-1.0)):0.0;
    p.maximum_reach_km=(p.link_margin_db>0.0)?spec->fiber_length_km+spec->fiber_length_km*p.link_margin_db/spec->fiber_attenuation_db_per_km:spec->fiber_length_km*0.5;
    return p;
}

/* L7: LIDAR Receiver Analysis */
lidar_receiver_perf_t tia_lidar_receiver_analyze(const tia_design_t *d,const lidar_receiver_spec_t *spec){
    lidar_receiver_perf_t p;memset(&p,0,sizeof(p));
    if(!d||!spec)return p;
    double area=M_PI*spec->aperture_diameter_mm*spec->aperture_diameter_mm*1.0e-6*0.25;
    double r=spec->target_range_m;
    double tau=exp(-spec->atmospheric_attenuation_db_per_km*r*1.0e-3*0.2302585);
    double rcs=spec->target_reflectivity*area;
    double pr=spec->pulse_energy_nj*1.0e-9*tau*tau*rcs/(4.0*M_PI*r*r*r*r*0.5);
    p.received_pulse_energy_fj=pr*1.0e15;
    p.peak_photocurrent_ua=d->photodiode.responsivity_a_per_w*pr/(spec->pulse_width_ns*1.0e-9)*1.0e6;
    p.snr_db=tia_snr_compute(p.peak_photocurrent_ua,d->input_noise_density_pa,1.0/(2.0*spec->pulse_width_ns*1.0e-9));
    p.range_resolution_mm=3.0e8*spec->pulse_width_ns*1.0e-9*0.5*1000.0;
    double nep_w=d->input_noise_density_pa*1.0e-12/d->photodiode.responsivity_a_per_w;
    p.max_detectable_range_m=sqrt(spec->pulse_energy_nj*1.0e-9*tau*tau*rcs/(4.0*M_PI*nep_w*7.0));
    p.false_alarm_rate=1.0e-6;
    p.detection_probability=(p.snr_db>12.0)?0.99:0.5;
    return p;
}

/* L7: Spectrophotometer Measurement */
spectrometer_measurement_t tia_spectrometer_measure(const tia_design_t *d,const spectrometer_spec_t *spec){
    spectrometer_measurement_t m;memset(&m,0,sizeof(m));
    if(!d||!spec||spec->wavelength_step_nm<=0.0||spec->wavelength_stop_nm<=spec->wavelength_start_nm)return m;
    m.num_points=(size_t)((spec->wavelength_stop_nm-spec->wavelength_start_nm)/spec->wavelength_step_nm)+1;
    m.wavelength_nm=malloc(m.num_points*sizeof(double));
    m.absorbance=malloc(m.num_points*sizeof(double));
    m.transmittance=malloc(m.num_points*sizeof(double));
    m.photocurrent_ua=malloc(m.num_points*sizeof(double));
    double R=d->photodiode.responsivity_a_per_w;
    m.snr_min_db=200.0;m.snr_max_db=-200.0;
    double dw=spec->wavelength_step_nm;
    for(size_t i=0;i<m.num_points;i++){
        double wl=spec->wavelength_start_nm+i*dw;
        m.wavelength_nm[i]=wl;
        double R_wl=R*wl/d->photodiode.peak_wavelength_nm;
        m.photocurrent_ua[i]=spec->optical_power_nw*1.0e-9*R_wl*1.0e6;
        m.transmittance[i]=0.9+0.1*sin(wl/100.0);
        m.absorbance[i]=-log10(m.transmittance[i]);
        double snr=tia_snr_compute(m.photocurrent_ua[i],d->input_noise_density_pa,1.0/spec->integration_time_ms*1000.0);
        if(snr<m.snr_min_db)m.snr_min_db=snr;
        if(snr>m.snr_max_db)m.snr_max_db=snr;
    }
    m.stray_light_percent=0.01;
    return m;
}

/* L8: Bootstrap TIA Design */
tia_bootstrap_config_t tia_bootstrap_design(const photodiode_model_t *pd,const opamp_params_t *opa,double bgain){
    tia_bootstrap_config_t bc;memset(&bc,0,sizeof(bc));
    if(!pd||!opa)return bc;
    bc.bootstrap_gain=bgain;
    bc.bootstrap_bandwidth_hz=opa->gain_bandwidth_mhz*1.0e6/bgain;
    bc.effective_cj_reduction_ratio=(bgain<1.0)?(1.0-bgain):0.1;
    bc.bootstrapped_cj_pf=pd->junction_capacitance_pf*bc.effective_cj_reduction_ratio;
    bc.added_noise_pa_per_sqrt_hz=opa->input_voltage_noise_nv*1.0e-9*2.0*M_PI*1.0e6*pd->junction_capacitance_pf*1.0e-12*1.0e12;
    bc.power_overhead_mw=10.0;
    return bc;
}

/* L8: Differential TIA Design */
tia_differential_config_t tia_differential_design(const photodiode_model_t *ps,const photodiode_model_t *pr,const opamp_params_t *opa){
    tia_differential_config_t dc;memset(&dc,0,sizeof(dc));
    if(!ps||!pr||!opa)return dc;
    dc.photodiode_matching_percent=95.0;
    dc.differential_gain_ohm=1.0e4;
    dc.common_mode_gain_ohm=10.0;
    dc.cmrr_db=20.0*log10(dc.differential_gain_ohm/dc.common_mode_gain_ohm);
    dc.common_mode_rejection_db=dc.cmrr_db+20.0*log10(100.0/(100.0-dc.photodiode_matching_percent));
    dc.offset_voltage_uv=100.0;
    dc.offset_drift_nv_per_c=500.0;
    return dc;
}

/* L8: Composite Amplifier TIA Design */
tia_composite_config_t tia_composite_design(const photodiode_model_t *pd,const opamp_params_t *s1,const opamp_params_t *s2,double g,double bw){
    tia_composite_config_t cc;memset(&cc,0,sizeof(cc));
    if(!pd||!s1||!s2)return cc;
    cc.stage1_opamp=*s1;cc.stage2_opamp=*s2;
    cc.stage1_gain=10.0;cc.stage2_gain=g/cc.stage1_gain;
    cc.effective_gbwp_mhz=s1->gain_bandwidth_mhz*s2->gain_bandwidth_mhz/(s1->gain_bandwidth_mhz+s2->gain_bandwidth_mhz);
    cc.interstage_compensation_pf=1.0;
    cc.total_power_mw=s1->supply_current_ma*s1->supply_voltage_max+s2->supply_current_ma*s2->supply_voltage_max;
    return cc;
}

/* L8: T-Network Feedback TIA Design */
tia_tnetwork_config_t tia_tnetwork_design(double target_gain,double max_rf,double noise_budget){
    tia_tnetwork_config_t tc;memset(&tc,0,sizeof(tc));
    if(target_gain<=0.0||max_rf<=0.0)return tc;
    tc.rf_ohm=max_rf;
    tc.r2_ohm=max_rf*10.0;
    tc.r1_ohm=max_rf*10.0/(target_gain/max_rf-1.0);
    tc.gain_enhancement_factor=1.0+tc.r2_ohm/tc.r1_ohm;
    tc.effective_gain_ohm=max_rf*tc.gain_enhancement_factor;
    tc.offset_multiplication=tc.gain_enhancement_factor;
    tc.noise_multiplication=tc.gain_enhancement_factor;
    return tc;
}

/* L8: Dark Current Temperature Dependence */
double tia_dark_current_at_temperature(const photodiode_model_t *pd,double temp_c){
    if(!pd)return 0.0;
    double dT=temp_c-25.0;
    double factor=pow(2.0,dT/pd->dark_current_tempco);
    return pd->dark_current_na*factor;
}

/* L8: Temperature Compensation Design */
tia_temp_compensation_t tia_temp_compensation_design(const photodiode_model_t *pd,double tmin,double tmax){
    tia_temp_compensation_t tc;memset(&tc,0,sizeof(tc));
    if(!pd)return tc;
    tc.temp_coefficient_gain_ppm=100.0;
    tc.temp_coefficient_offset_uv=50.0;
    tc.compensation_method=1.0;
    tc.residual_drift_ppm=20.0;
    tc.temperature_range_min_c=tmin;
    tc.temperature_range_max_c=tmax;
    return tc;
}

/* L8: CMOS TIA Estimation */
cmos_tia_params_t tia_cmos_estimate(double node_nm,double gain_db,double bw_ghz){
    cmos_tia_params_t cp;memset(&cp,0,sizeof(cp));
    cp.technology_node_nm=node_nm;
    cp.supply_voltage_v=(node_nm>45.0)?1.8:1.0;
    cp.transistor_ft_ghz=300.0/28.0*28.0/(node_nm>0.0?node_nm:28.0);
    cp.flicker_noise_coefficient=1.0e-24;
    cp.bandwidth_ghz=bw_ghz;
    cp.transimpedance_gain_dbohm=gain_db;
    cp.input_referred_noise_pa=10.0*bw_ghz;
    cp.power_consumption_mw=20.0*bw_ghz;
    cp.die_area_mm2=0.1+0.05*bw_ghz;
    return cp;
}

/* LIDAR Range Equation */
double tia_lidar_range_equation(double E,double A,double refl,double tau,double Pmin){
    if(Pmin<=0.0)return 0.0;
    return sqrt(E*A*refl*tau*tau/(4.0*M_PI*Pmin));
}

/* Pulse Detection Threshold */
double tia_pulse_detection_threshold(const tia_noise_model_t *noise,double pfa){
    if(!noise)return 0.0;
    double sigma=noise->total_input_noise_pa*1.0e-12;
    double thresh=sigma*4.0;
    if(pfa<1.0e-3)thresh=sigma*5.0;
    if(pfa<1.0e-6)thresh=sigma*6.0;
    return thresh;
}

/* APD Optimal Gain */
double tia_apd_optimal_gain(double k,double nth,double idark,double isig){
    (void)nth;(void)idark;(void)isig;
    if(k<=0.0)return 100.0;
    return sqrt(1.0/k);
}

void spectrometer_measurement_free(spectrometer_measurement_t *m){
    if(!m)return;
    free(m->wavelength_nm);free(m->absorbance);
    free(m->transmittance);free(m->photocurrent_ua);
    memset(m,0,sizeof(*m));
}
