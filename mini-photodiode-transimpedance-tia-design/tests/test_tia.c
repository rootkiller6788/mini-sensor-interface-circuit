/* test_tia.c - TIA module test suite with assert-based verification */
#include "tia_core.h"
#include "tia_noise.h"
#include "tia_stability.h"
#include "tia_design.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) do { printf("  TEST: %s ... ", name); } while(0)
#define PASS() do { printf("PASS\n"); tests_passed++; } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); tests_failed++; } while(0)
#define CHECK(cond, msg) do { if(cond) PASS(); else FAIL(msg); } while(0)

int main(void) {
    printf("=== TIA Module Test Suite ===\n\n");

    /* L1: Photodiode model initialization */
    printf("--- L1: Photodiode Models ---\n");
    TEST("Si PIN init");
    photodiode_model_t pd_si = photodiode_model_init(PHOTODIODE_SI_PIN, 1.0, 0.0);
    CHECK(pd_si.responsivity_a_per_w > 0.4 && pd_si.responsivity_a_per_w < 0.7, "responsivity range");
    CHECK(pd_si.quantum_efficiency > 0.7, "QE > 0.7");
    CHECK(pd_si.junction_capacitance_pf > 0.0, "Cj > 0");
    CHECK(pd_si.shunt_resistance_ohm > 1.0e8, "Rsh large");
    CHECK(pd_si.dark_current_na < 1.0, "dark current < 1nA");
    CHECK(pd_si.bias_mode == BIAS_PHOTOVOLTAIC, "zero bias -> photovoltaic");

    TEST("InGaAs PIN init with bias");
    photodiode_model_t pd_ingaas = photodiode_model_init(PHOTODIODE_INGAAS_PIN, 1.0, 5.0);
    CHECK(pd_ingaas.peak_wavelength_nm > 1500.0, "IR wavelength");
    CHECK(pd_ingaas.bias_mode == BIAS_PHOTOCONDUCTIVE, "5V -> photoconductive");
    CHECK(pd_ingaas.junction_capacitance_pf < 10.0, "Cj reduced by bias");

    TEST("Si APD init");
    photodiode_model_t pd_apd = photodiode_model_init(PHOTODIODE_SI_APD, 0.5, 150.0);
    CHECK(pd_apd.responsivity_a_per_w > 1.0, "APD gain > 1");

    TEST("Photodiode area scaling");
    photodiode_model_t pd_scaled = photodiode_scale_area(&pd_si, 4.0);
    CHECK(fabs(pd_scaled.junction_capacitance_pf - pd_si.junction_capacitance_pf * 4.0) < 1.0, "Cj scales with area");
    CHECK(fabs(pd_scaled.dark_current_na - pd_si.dark_current_na * 4.0) < 0.1, "Idark scales with area");
    CHECK(pd_scaled.shunt_resistance_ohm < pd_si.shunt_resistance_ohm, "Rsh inversely scales");

    TEST("Junction capacitance vs bias");
    double cj0 = pd_si.junction_capacitance_pf;
    double cj5 = photodiode_cj_at_bias(&pd_si, 5.0);
    CHECK(cj5 < cj0, "Cj decreases with reverse bias");

    /* L1: Op-amp parameter initialization */
    printf("\n--- L1: Op-Amp Parameters ---\n");
    TEST("OPA657 init");
    opamp_params_t opa657 = opamp_params_init("OPA657");
    CHECK(opa657.gain_bandwidth_mhz > 1000.0, "GBWP > 1GHz");
    CHECK(opa657.unity_gain_stable == 1.0, "unity gain stable");
    CHECK(opa657.input_current_noise_fa < 10.0, "low JFET current noise");

    TEST("LTC6268 init");
    opamp_params_t opa6268 = opamp_params_init("LTC6268");
    CHECK(opa6268.input_current_noise_fa < 0.01, "ultra-low FET i_n");
    CHECK(opa6268.supply_current_ma < 20.0, "moderate power");

    TEST("Unknown op-amp returns zeros");
    opamp_params_t opa_unk = opamp_params_init("UNKNOWN");
    CHECK(opa_unk.gain_bandwidth_mhz < 0.001, "unknown -> zero GBWP");

    /* L2: TIA Design Flow */
    printf("\n--- L2/L5: TIA Design ---\n");
    TEST("Input capacitance");
    double cin = tia_input_capacitance(&pd_si, &opa657, 0.5);
    CHECK(cin > pd_si.junction_capacitance_pf, "Cin includes all contributions");

    TEST("Compensation capacitance");
    double cf = tia_compensation_capacitance(&pd_si, &opa657, 10000.0, 65.0);
    CHECK(cf > 0.001 && cf < 200.0, "Cf in reasonable range");

    TEST("Basic TIA design");
    tia_design_t design = tia_design_basic(&pd_si, &opa657, 10000.0, 0.0);
    CHECK(design.rf_ohm == 10000.0, "Rf set correctly");
    CHECK(design.bandwidth_3db_hz > 1.0e3, "BW > 1 kHz");
    CHECK(design.bandwidth_3db_hz < 500.0e6, "BW < GBW");
    CHECK(design.transimpedance_gain_db > 60.0, "Gain > 60 dB");
    CHECK(design.phase_margin_deg > 30.0, "PM > 30 deg");
    CHECK(design.total_input_noise_pa > 0.0, "noise computed");

    TEST("TIA bandwidth computation");
    double bw = tia_bandwidth_3db(&design);
    CHECK(bw > 0.0, "bandwidth positive");

    TEST("Gain at DC equals Rf");
    double gain_dc = tia_gain_at_frequency(&design, 1.0);
    CHECK(fabs(gain_dc - 10000.0) / 10000.0 < 0.01, "DC gain = Rf");

    TEST("Gain rolls off at 10x BW");
    double gain_hf = tia_gain_at_frequency(&design, design.bandwidth_3db_hz * 10.0);
    CHECK(gain_hf < gain_dc * 0.3, "HF rolloff");

    /* L4: Johnson Noise & Shot Noise */
    printf("\n--- L4: Noise Laws ---\n");
    TEST("Johnson noise voltage at 300K, 10k");
    double vn_j = noise_johnson_voltage(10000.0, 300.0);
    CHECK(vn_j > 1.0e-8 && vn_j < 3.0e-8, "~12.9 nV/rtHz at 10k");

    TEST("Johnson noise current");
    double in_j = noise_johnson_current(10000.0, 300.0);
    CHECK(fabs(in_j - vn_j / 10000.0) < 1.0e-15, "Norton equivalence");

    TEST("Shot noise at 1uA");
    double in_shot = noise_shot(1.0e-6);
    CHECK(in_shot > 1.0e-16 && in_shot < 1.0e-11, "shot noise at 1uA valid");

    TEST("kT/C noise at 1pF");
    double vn_ktc = noise_ktc_rms(1.0e-12, 300.0);
    CHECK(vn_ktc > 50.0e-6 && vn_ktc < 80.0e-6, "~64 uV rms at 1pF");

    /* L5: Noise Model */
    printf("\n--- L5: Noise Model ---\n");
    TEST("Noise gain at DC");
    double ng_dc = tia_noise_gain(&design, 1.0);
    CHECK(fabs(ng_dc - 1.0) < 0.01, "NG(0) = 1");

    TEST("Noise gain at HF");
    double ng_hf = tia_noise_gain(&design, design.bandwidth_3db_hz);
    CHECK(ng_hf >= 1.0, "NG >= 1");

    TEST("Full noise analysis");
    tia_noise_model_t noise = tia_noise_analyze(&design, 1.0, 1.0, design.bandwidth_3db_hz);
    CHECK(noise.total_input_noise_pa > 0.0, "noise computed");
    CHECK(noise.total_output_noise_uv > 0.0, "output noise computed");
    CHECK(noise.system_noise_figure_db >= 0.0, "NF >= 0 dB");

    TEST("Dominant noise source");
    int dom = tia_noise_dominant_source(&noise);
    CHECK(dom >= 0 && dom < NOISE_SOURCE_COUNT, "valid source index");

    /* L5: Stability Analysis */
    printf("\n--- L5: Stability ---\n");
    TEST("Loop gain analysis");
    loop_gain_analysis_t lga = tia_loop_gain_analyze(&design);
    CHECK(lga.phase_margin_deg > 0.0, "PM > 0");
    CHECK(lga.phase_margin_deg > -180.0, "loop gain analysis valid");

    TEST("Phase margin from design");
    double pm = tia_phase_margin(&design);
    CHECK(pm > -10.0 && pm < 100.0, "PM in valid range");

    TEST("Routh-Hurwitz criterion");
    int rh = tia_routh_hurwitz_stable(&design);
    CHECK(rh == 1, "Routh-Hurwitz stable");

    TEST("Damping from PM");
    double zeta = tia_damping_from_pm(design.phase_margin_deg);
    CHECK(zeta > 0.0 && zeta < 1.5, "valid damping");

    /* L5: Frequency Response */
    printf("\n--- L5: Frequency Response ---\n");
    TEST("Frequency response");
    tia_freq_response_t fresp = tia_compute_frequency_response(&design, 1000.0, 1.0e9, 100);
    CHECK(fresp.num_points == 100, "correct points");
    CHECK(fresp.f_3db_hz > 0.0, "3dB point found");
    tia_freq_response_free(&fresp);

    TEST("Bode analysis");
    tia_bode_data_t bode = tia_compute_bode(&design, 1000.0, 1.0e9, 100);
    CHECK(bode.num_points == 100, "Bode points");
    CHECK(bode.phase_margin_deg > 0.0, "PM from Bode");
    tia_bode_data_free(&bode);

    /* L6: Step Response */
    printf("\n--- L6: Step Response ---\n");
    TEST("Step response");
    tia_step_response_t step = tia_compute_step_response(&design, 10.0, 100.0, 100);
    CHECK(step.num_points == 100, "step points");
    CHECK(step.final_value_v > 0.0, "final value");
    CHECK(step.rise_time_10_90_ns >= 0.0, "rise time non-negative");
    tia_step_response_free(&step);

    /* L7: Optical Link */
    printf("\n--- L7: Optical Link Budget ---\n");
    TEST("Link budget");
    optical_link_budget_t link = tia_link_budget(&design, 1.0e-6, 850.0);
    CHECK(link.photocurrent_ua > 0.0, "photocurrent computed");
    CHECK(link.snr_db > 0.0, "SNR computed");

    TEST("Receiver sensitivity");
    double sens = tia_sensitivity(&design, 1.0e-12, 1.0e9);
    CHECK(sens < 0.0, "sensitivity is negative dBm");

    /* L5: Design Methodology */
    printf("\n--- L5: Design Methodology ---\n");
    TEST("High-speed design");
    tia_design_t hs = tia_design_high_speed(&pd_si, &opa657, 100.0);
    CHECK(hs.bandwidth_3db_mhz > 10.0, "HS BW reasonable");

    TEST("Low-noise design");
    tia_design_t ln = tia_design_low_noise(&pd_si, &opa657, 10000.0);
    CHECK(ln.total_input_noise_pa > 0.0, "LN noise computed");

    TEST("Performance summary");
    tia_performance_summary_t summary = tia_performance_summary(&design);
    CHECK(summary.figure_of_merit > 0.0, "FOM computed");
    CHECK(summary.power_consumption_mw > 0.0, "power estimated");

    /* L1: Utility functions */
    printf("\n--- L1: Utilities ---\n");
    TEST("SNR computation");
    double snr = tia_snr_compute(100.0, 1.0, 1.0e6);
    CHECK(snr > 0.0, "SNR positive");

    TEST("Required photocurrent");
    double iph = tia_required_photocurrent(40.0, 1.0);
    CHECK(iph > 0.0, "required current computed");

    TEST("Optical power from current");
    double popt = tia_optical_power_from_current(550.0, 0.55);
    CHECK(popt > 0.0 && popt < 10.0, "opt power from current valid");

    /* Math assertions */
    printf("\n--- L4: Mathematical Assertions ---\n");
    TEST("Parseval-like: noise power sum");
    double total_noise_sq = noise.johnson_rf_density * noise.johnson_rf_density +
                            noise.shot_dark_density * noise.shot_dark_density +
                            noise.opamp_en_density * noise.opamp_en_density;
    CHECK(total_noise_sq > 0.0, "noise powers sum to positive value");

    TEST("Bandwidth relation: BW*Rf product");
    double bw_product = design.bandwidth_3db_hz * design.rf_ohm;
    CHECK(bw_product > 1.0e9, "BW*Rf > 1e9 for OPA657");

    TEST("Gain-BW product consistency");
    CHECK(design.bandwidth_3db_hz < opa657.gain_bandwidth_mhz * 1.0e6 * 0.5,
          "BW < GBW/2");

    TEST("Phase margin vs damping consistency");
    double pm2 = 65.0;
    double zeta2 = tia_damping_from_pm(pm2);
    double peaking2 = tia_peaking_from_pm(pm2);
    CHECK(zeta2 > 0.3 && zeta2 < 1.2, "PM=65 -> zeta near 0.7");
    CHECK(peaking2 < 5.0, "PM=65 -> moderate peaking");

    TEST("Thermal noise temperature scaling");
    double vn_400k = noise_johnson_voltage(10000.0, 400.0);
    double vn_300k = noise_johnson_voltage(10000.0, 300.0);
    CHECK(vn_400k > vn_300k, "noise increases with temperature");

    TEST("Shot noise current scaling");
    double sn_1ua = noise_shot(1.0e-6);
    double sn_100na = noise_shot(100.0e-9);
    CHECK(sn_1ua > sn_100na, "shot noise scales with sqrt(I)");

    /* Summary */
    printf("\n=== Results ===\n");
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
    printf("Total:  %d\n", tests_passed + tests_failed);

    if (tests_failed > 0) {
        printf("\n*** SOME TESTS FAILED ***\n");
        return 1;
    }
    printf("\n*** ALL TESTS PASSED ***\n");
    return 0;
}
