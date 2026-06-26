/**
 * @file example_isolated_rs485.c
 * @brief L7: Isolated RS-485 transceiver for industrial fieldbus
 *
 * Demonstrates digital isolator + RS-485 transceiver for Modbus/Profibus
 * in solar inverter and motor drive applications. Includes reinforced
 * isolation verification and CMTI analysis for noisy industrial environments.
 */
#include "digital_isolator.h"
#include "isolator_channel.h"
#include "cmr_analysis.h"
#include "isolation_amplifier.h"
#include <stdio.h>
#include <math.h>

int main(void) {
    printf("=== Isolated RS-485 Fieldbus Example (L7) ===

");

    /* Setup reinforced digital isolator for RS-485 */
    digital_isolator_t iso;
    digital_isolator_init(&iso, ISOL_TECH_CAPACITIVE, ISOL_CLASS_REINFORCED, 3);
    printf("Isolator: %s
", iso.part_number);
    printf("  Reinforced: %s
", is_reinforced_isolation(&iso) ? "YES" : "NO");
    printf("  Viso = %.1f kV, Surge = %.1f kV
",
           iso.barrier.viso_peak_kv, iso.barrier.viso_surge_kv);

    /* CMTI analysis for motor drive environment */
    double cmti_rate = cmti_limited_data_rate(&iso);
    printf("
CMTI Analysis:
");
    printf("  CMTI: %.0f kV/us
", iso.barrier.cmti_kv_us);
    printf("  CMTI-limited max data rate: %.1f Mbps
", cmti_rate);

    /* Isolation channel for RS-485 TX */
    isolation_channel_t tx_ch;
    isolation_channel_init(&tx_ch, 0, &iso);
    isolation_channel_capacity_compute(&tx_ch, 25e6, 30.0);
    printf("
RS-485 TX Channel:
");
    printf("  Bandwidth: %.1f MHz
", tx_ch.capacity.channel_bandwidth_hz/1e6);
    printf("  Shannon capacity: %.1f Mbps
", tx_ch.capacity.channel_capacity_bps/1e6);
    printf("  Actual rate: %.1f Mbps
", tx_ch.capacity.actual_data_rate_bps/1e6);
    printf("  Margin: %.1f dB
", tx_ch.capacity.margin_db);

    /* Isolation amplifier for analog front-end */
    isolation_amplifier_t amp;
    isoamp_init(&amp, ISOAMP_ARCH_CARRIER_MODULATED, 8.0, 200.0);
    printf("
Isolation Amplifier:
");
    printf("  Gain: %.1f V/V, BW: %.1f kHz
",
           amp.dc.nominal_gain_v_per_v, amp.ac.bandwidth_khz);
    printf("  CMRR @ DC: %.0f dB, @ 1kHz: %.0f dB
",
           amp.isolation.cmrr_at_dc_db, amp.isolation.cmrr_at_1khz_db);
    printf("  IMRR @ 60Hz: %.0f dB
", amp.isolation.imrr_at_60hz_db);

    /* CMR analysis for source impedance imbalance */
    cmr_analyzer_t cmr;
    cmr_analyzer_init(&cmr, &amp);
    cmr_budget_add_contribution(&cmr, CMR_SOURCE_INPUT_IMBALANCE, 80.0, 100.0,
                                 "Input resistor mismatch 0.1%");
    cmr_budget_add_contribution(&cmr, CMR_SOURCE_BARRIER_COUPLING, 90.0, 1000.0,
                                 "Barrier capacitive coupling");
    cmr_budget_compute(&cmr, 80.0);
    printf("
CMRR Budget:
");
    printf("  RSS combined CMRR: %.1f dB
", cmr.budget.rss_combined_cmrr_db);
    printf("  Worst-case CMRR: %.1f dB
", cmr.budget.worst_case_cmrr_db);
    printf("  Margin to target: %.1f dB
", cmr.budget.margin_db);
    printf("  Meets 80 dB target: %s
", cmr.budget.meets_target ? "YES" : "NO");

    /* Thermocouple application */
    double rejected_uv;
    cmr_analyze_isolated_thermocouple(&cmr, 100.0, 5.0, &rejected_uv);
    printf("
Thermocouple isolation:
");
    printf("  5V ground difference -> %.2f uV output error
", rejected_uv);

    cmr_analyzer_destroy(&cmr);
    isoamp_destroy(&amp);
    isolation_channel_destroy(&tx_ch);
    digital_isolator_destroy(&iso);
    printf("
=== Example complete ===
");
    return 0;
}