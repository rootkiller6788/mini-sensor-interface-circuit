#include "mems_pcb_design.h"
#include <stdio.h>
#include <math.h>

int main(void)
{
    printf("=== MEMS Sensor PCB Design Analysis ===\n\n");

    /* L2: Microstrip impedance calculation */
    printf("L2 - Transmission Line Impedance (IPC-2141):\n");
    double w = 0.3, h = 0.2, er = 4.3;
    double z0_ms = pcb_microstrip_z0(w, h, er);
    double z0_sl = pcb_stripline_z0(w, h, er);
    printf("  Microstrip: %.1f ohm (w=%.2f, h=%.2f, Er=%.1f)\n", z0_ms, w, h, er);
    printf("  Stripline:  %.1f ohm\n", z0_sl);

    /* L3: Critical length analysis */
    printf("\nL3 - Signal Integrity Critical Length:\n");
    double tr = 100.0;
    double crit = pcb_critical_length(tr, er);
    printf("  Tr=%.0f ps, Er=%.1f -> Critical length: %.1f mm\n", tr, er, crit);

    /* L4: PDN decoupling design */
    printf("\nL4 - Power Distribution Network Design:\n");
    double Z_target = pcb_pdn_target_impedance(3.3, 5.0, 0.1);
    printf("  Target impedance: %.3f ohm (Vdd=3.3V, ripple=5%%, Imax=100mA)\n", Z_target);

    pdn_design_t pdn = {
        .target_impedance_ohm = Z_target,
        .max_ripple_mv = 165.0,
        .transient_current_ma = 100.0,
        .vrm_bandwidth_hz = 100000.0,
        .num_decoupling_caps = 3,
        .cap_values_uf = {10.0, 1.0, 0.1},
        .cap_esr_mohm = {10.0, 50.0, 100.0},
        .cap_esl_nh = {1.0, 0.5, 0.3}
    };

    double Z_1kHz = pcb_decoupling_cap_z(1000.0, 10e-6, 1e-9, 0.01);
    double Z_1MHz = pcb_decoupling_cap_z(1e6, 10e-6, 1e-9, 0.01);
    printf("  Z@1kHz: %.3f ohm, Z@1MHz: %.3f ohm\n", Z_1kHz, Z_1MHz);

    pdn_analysis_t analysis;
    pcb_pdn_impedance_profile(&pdn, &analysis, 100);
    printf("  Achieved Z: %.3f ohm, Resonances: %d\n",
           analysis.achieved_impedance_ohm, analysis.num_resonances);

    /* L5: Via parameters */
    printf("\nL5 - Via Parasitics:\n");
    double L_via = pcb_via_inductance_nh(1.6, 0.3);
    double C_via = pcb_via_capacitance_pf(0.3, 1.0, 1.6, 4.3);
    printf("  Via L: %.2f nH, Via C: %.3f pF\n", L_via, C_via);

    /* L7: EMI estimation */
    printf("\nL7 - EMI and Crosstalk:\n");
    double E_far = pcb_far_field_emission(100e6, 0.01, 0.05, 3.0);
    printf("  Far-field E @100MHz, 3m: %.2e V/m\n", E_far);
    double xtalk = pcb_near_field_crosstalk(0.2, 0.15, 0.15, 100.0);
    printf("  Near-field crosstalk: %.3f mV\n", xtalk);

    printf("\n=== PCB Design Analysis Complete ===\n");
    return 0;
}
