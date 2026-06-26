#include <stdio.h>
#include <math.h>
#include <string.h>
#include "bridge_core.h"
#include "bridge_excitation.h"
#include "bridge_conditioning.h"
#include "strain_gauge_physics.h"
#include "bridge_calibration.h"
#include "bridge_applications.h"

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;
static const double EPS = 1.0e-9;

static void check(int cond, const char* msg) {
    tests_run++;
    if (!cond) { printf("  FAIL: %s\n", msg); tests_failed++; }
    else { printf("  PASS\n"); tests_passed++; }
}

int main(void) {
    printf("=== Bridge Sensor / Strain Gauge Tests ===\n\n");
    bridge_state_t b;
    bridge_state_init(&b, 350.0, 5.0, BRIDGE_FULL);

    printf("[L3] balanced bridge... "); check(fabs(bridge_output_voltage(&b))<EPS,"balanced");
    printf("[L3] impedances... "); check(fabs(bridge_input_impedance(&b)-350)<0.01&&fabs(bridge_output_impedance(&b)-350)<0.01,"impedances");
    printf("[L3] power... "); { bridge_state_t bp; bridge_state_init(&bp,350,10,BRIDGE_FULL); double p[4]; bridge_power_dissipation(&bp,p); double pe=(10.0/700)*(10.0/700)*350; check(fabs(p[0]-pe)<0.001,"power"); }
    printf("[L3] sensitivity... "); check(fabs(bridge_sensitivity_mv_per_v(BRIDGE_FULL,2.0,1000)-2.0)<0.01,"sens");
    printf("[L3] strain from Vout... "); check(fabs(bridge_output_to_strain(-0.00125,5,2,BRIDGE_QUARTER)-500)<0.2,"strain");
    printf("[L3] nonlinearity... "); check(fabs(bridge_nonlinearity_error(1000,2,BRIDGE_QUARTER)-0.1)<0.02,"NL");
    printf("[L4] Hooke law... "); check(fabs(hookes_law_stress(1000,200)-200)<0.01,"Hooke");
    printf("[L4] R from eps... "); check(fabs(gauge_resistance_from_strain(350,2,1000)-350.7)<0.01,"R");
    printf("[L4] eps from R... "); check(fabs(strain_from_gauge_resistance(350.7,350,2)-1000)<0.1,"eps");
    printf("[L4] GF theory... "); check(fabs(gauge_factor_from_material(0.33,2.4e-3,163)-2.05)<0.05,"GF");
    printf("[L4] thermal output... "); check(fabs(temperature_apparent_strain(10,11,15,2,50)-210)<0.1,"thermal");
    printf("[L4] von Mises... "); check(fabs(von_mises_stress(100,0)-100)<0.01,"VM");
    printf("[L4] VM strain... "); { strain_state_t vs={100,-30,0}; check(von_mises_strain(&vs)>0,"VMe"); }
    printf("[L4] transverse... "); check(fabs(transverse_sensitivity_correction(1000,0.01,0.285,0.33)-1000.45)<0.5,"transv");
    printf("[L4] thermal stress... "); check(fabs(thermal_stress_mpa(200,11,15,10)+8)<1,"thermS");
    printf("[L4] plane stress... "); { strain_state_t ss={1000,-300,0}; stress_state_t so; hookes_law_plane_stress(&ss,200,0.3,&so); check(fabs(so.sigma_x-200)<2,"plane"); }
    printf("[Exc] Johnson noise... "); { double v=excitation_johnson_noise(350,300,100); check(v>1e-9&&v<1e-7,"Jn"); }
    printf("[Exc] strain error... "); check(fabs(excitation_strain_error(5,5.005,2,BRIDGE_FULL)-500)<1,"excErr");
    printf("[Exc] ratiometric... "); { double c=excitation_ratiometric_code(0.0025,5,12,1); check(c>=2&&c<3,"ratio"); }
    printf("[Exc] total noise... "); { double v=excitation_total_noise(1e-8,2e-8,3e-8); check(v>0,"noise"); }
    printf("[Sig] required gain... "); check(fabs(conditioning_required_gain(5,5,2,1000,BRIDGE_FULL)-500)<5,"gain");
    printf("[Sig] ENOB... "); { double e=conditioning_enob(1,0.001); check(e>8&&e<11,"ENOB"); }
    printf("[Sig] moving avg... "); { moving_average_filter_t m; conditioning_maf_init(&m,4); conditioning_maf_process(&m,1); conditioning_maf_process(&m,3); conditioning_maf_process(&m,5); double a=conditioning_maf_process(&m,7); check(fabs(a-4)<EPS,"MAF"); a=conditioning_maf_process(&m,9); check(fabs(a-6)<EPS,"MAF2"); conditioning_maf_free(&m); }
    printf("[Sig] OSR... "); check(conditioning_oversampling_ratio(12,16)==256,"OSR");
    printf("[SG] GF decompose... "); { double d,r; strain_gauge_gf_decompose(0.33,2.4e-3,163,&d,&r); check(fabs(d-1.66)<0.01&&fabs(r-0.39)<0.05,"GFdec"); }
    printf("[SG] piezo long... "); { double p=strain_gauge_piezo_longitudinal(-102.2e-11,53.4e-11,-13.6e-11,1,0,0); check(fabs(p+102.2e-11)<1e-20,"pi_l"); }
    printf("[SG] Mohr circle... "); { strain_state_t s={100,-50,0}; double c,r; strain_gauge_mohr_circle(&s,&c,&r); check(fabs(c-25)<EPS&&fabs(r-75)<EPS,"Mohr"); }
    printf("[SG] misalignment... "); check(fabs(strain_gauge_misalignment_error(1000,0.3,5)+0.0099)<0.002,"mis");
    printf("[SG] material db... "); { material_database_entry_t e; check(strain_gauge_material_lookup("steel",&e)==0,"steel"); check(strain_gauge_material_lookup("aluminum",&e)==0,"Al"); check(strain_gauge_material_lookup("gold",&e)==-1,"gold"); }
    printf("[SG] STC select... "); check(strain_gauge_select_stc(11.7,100)>0,"STC");
    printf("[SG] self-heating... "); { double dt=strain_gauge_self_heating(0.005,50,18e-6,0.05e-3); check(dt<1,"heat"); }
    printf("[SG] creep... "); check(strain_gauge_creep(1000,0.001,10)>0,"creep");
    printf("[SG] hysteresis... "); check(strain_gauge_hysteresis(1000,0.005,5)>0,"hyst");
    printf("[SG] Si GF... "); check(strain_gauge_silicon_gf(1e18,300,1)>10,"SiGF");
    printf("[SG] Si TCR... "); check(fabs(strain_gauge_silicon_tcr(1e18,300))<5000,"SiTCR");
    printf("[SG] Kt... "); check(fabs(strain_gauge_stress_concentration(0,1,1)-3.0)<EPS,"Kt");
    printf("[Cal] shunt cal... "); { strain_gauge_t g; strain_gauge_init(&g,350,2,"C"); double e=calibration_shunt_strain(&g,350000,1); check(e<-490&&e>-510,"shunt"); }
    printf("[Cal] shunt R... "); { strain_gauge_t g; strain_gauge_init(&g,350,2,"C"); double r=calibration_shunt_resistor(&g,-500,1); check(r>300000&&r<400000,"shR"); }
    printf("[Cal] two-point... "); { cal_data_point_t z,s; cal_data_point_init(&z,0,0,5,25); cal_data_point_init(&s,100,0.002,5,25); calibration_result_t cr; calibration_two_point(&z,&s,&cr); check(fabs(cr.slope-0.00002)<1e-9,"2pt"); }
    printf("[Cal] linear fit... "); { cal_data_point_t pts[5]; int i; for(i=0;i<5;i++){double f=i*25.0;cal_data_point_init(&pts[i],f,f*0.00002,5,25);} calibration_result_t lr; calibration_linear_fit(pts,5,&lr); check(lr.r_squared>0.999,"lin"); }
    printf("[Cal] poly fit... "); { cal_data_point_t pts[6]; int i; for(i=0;i<6;i++){double f=i*20.0;cal_data_point_init(&pts[i],f,f*0.00002,5,25);} double c[8]={0}; check(calibration_polynomial_fit(pts,6,2,c)==0,"poly"); }
    printf("[Cal] lookup... "); { cal_data_point_t pts[5]; int i; for(i=0;i<5;i++){cal_data_point_init(&pts[i],(double)i*25,0.001*i,5,25);} double v=calibration_lookup(37.5,pts,5); check(v>0,"lkp"); }
    printf("[Cal] uncertainty... "); check(calibration_uncertainty(0.000005,10,0.01,2)>0,"uncert");
    printf("[Cal] temp compensate... "); { calibration_result_t cr; calibration_result_init(&cr); cr.slope=0.001; calibration_temp_compensate(&cr,35,100,0.5); check(fabs(cr.slope-0.001001)<1e-9,"tcomp"); }
    printf("[App] load cell... "); { loadcell_spec_t lc; loadcell_spec_init(&lc,LOADCELL_SHEAR_BEAM); double v=application_loadcell_output(&lc,50,10); double f=application_loadcell_force(&lc,v,10); check(fabs(f-50)<0.1,"LC"); }
    printf("[App] multicell... "); { double f[4]={25,25,25,25}; double cf[4]={1,1,1,1}; check(fabs(application_multicell_total(f,cf)-100)<0.01,"multi"); }
    printf("[App] pressure... "); { pressure_sensor_spec_t ps; pressure_sensor_spec_init(&ps,PRESSURE_ABSOLUTE); double pv=5*0.05e-3*50; double pr=application_pressure_read(&ps,pv,5,25,25); check(fabs(pr-50)<1,"press"); }
    printf("[App] diaphragm... "); { double t=application_pressure_diaphragm_thickness(2000,100,500,169,0.22); check(t>10&&t<200,"diaph"); }
    printf("[App] torque stress... "); { double t=application_torque_shear_stress(100,25); check(t>20&&t<50,"torS"); }
    printf("[App] torque strain... "); { double e=application_torque_strain(100,25,200,0.29); check(e>50&&e<200,"torE"); }
    printf("[App] shaft diameter... "); { double d=application_torque_shaft_diameter(100,500,200,0.29); check(d>10&&d<40,"shaft"); }
    printf("[App] weighing... "); { weighing_system_t ws; weighing_system_init(&ws); double r[4]={100,100,100,100}; double t=application_weighing_total(&ws,r,4); check(t>350,"weigh"); }
    printf("[App] fatigue... "); { aerospace_shm_t shm; aerospace_shm_init(&shm); application_aerospace_fatigue(&shm,500,100,5,1000,1e7); check(shm.n_cycles_recorded==100,"fatigue"); }
    printf("[App] battery life... "); { wireless_sensor_node_t n; wireless_sensor_node_init(&n); check(application_wireless_battery_life(&n)>100,"batt"); }
    printf("[App] edge process... "); { int sr; application_edge_process(50,45,10,&sr); check(sr==0,"ed0"); application_edge_process(60,45,10,&sr); check(sr==1,"ed1"); }
    printf("[Ros] rectangular... "); { rosette_data_t r={ROSETTE_RECTANGULAR,1000,500,-300}; rosette_resolve_strain(&r); check(fabs(r.resolved.epsilon_x-1000)<EPS&&fabs(r.resolved.epsilon_y+300)<EPS,"ros"); }
    printf("[Ros] delta... "); { rosette_data_t r={ROSETTE_DELTA,800,250,-150}; rosette_resolve_strain(&r); check(fabs(r.resolved.epsilon_x-800)<EPS,"rosD"); }
    printf("[Init] all structures... "); { strain_gauge_t g; strain_gauge_init(&g,350,2.05,"C"); bridge_state_t bb; bridge_state_init(&bb,350,5,BRIDGE_FULL); bridge_sensor_t s; bridge_sensor_init(&s); check(g.nominal_resistance==350&&s.config==BRIDGE_FULL,"init"); }
    printf("[Init] excitation src... "); { excitation_source_t es; excitation_source_init(&es,EXCITATION_VOLTAGE); check(es.mode==EXCITATION_VOLTAGE,"exc"); }
    printf("[Init] voltage ref... "); { voltage_reference_t vr; voltage_reference_init(&vr,5.0); check(vr.output_voltage==5.0,"vref"); }
    printf("[NL] fit correction... "); { double va[5]={0,0.0025,0.005,0.0075,0.01}; double sa[5]={0,250,500,750,1000}; nl_correction_poly_t poly; poly.order=2; check(bridge_fit_nl_correction(va,sa,5,&poly)==0,"NLfit"); }
    printf("[NL] apply correction... "); { nl_correction_poly_t poly; poly.order=1; poly.coeffs[0]=0; poly.coeffs[1]=1; check(fabs(bridge_apply_nl_correction(500,&poly)-500)<EPS,"NLapp"); }

    printf("\n========================================\n");
    printf("Results: %d/%d passed, %d failed\n", tests_passed, tests_run, tests_failed);
    return (tests_failed > 0) ? 1 : 0;
}
