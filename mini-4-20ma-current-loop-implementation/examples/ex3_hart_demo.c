/**
 * ex3_hart_demo.c -- HART protocol demonstration on 4-20mA loop.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../include/current_loop.h"
#include "../include/hart_protocol.h"

int main(void)
{
    printf("=== Example 3: HART Protocol on 4-20mA Loop ===\n\n");

    printf("--- HART Command 0: Read Unique Identifier ---\n");
    hart_frame_t cmd0;
    hart_build_command_0(&cmd0, 0);
    printf("  Preamble: %d bytes\n", cmd0.preamble_count);
    printf("  Delimiter: 0x%02X\n", cmd0.delimiter);
    printf("  Command: %d\n", cmd0.command);
    printf("  Checksum: 0x%02X\n", cmd0.checksum);
    printf("  Valid: %s\n", hart_validate_frame(&cmd0) ? "YES" : "NO");

    printf("\n--- HART Command 3: Read Dynamic Variables ---\n");
    hart_frame_t resp;
    memset(&resp, 0, sizeof(resp));
    resp.preamble_count = 5;
    resp.delimiter = 0x06;
    resp.address[0] = 0x80;
    resp.address_length = 1;
    resp.command = HART_CMD_READ_PV_CURRENT;
    resp.byte_count = 24;
    resp.data_length = 24;
    hart_encode_float(14.2f, &resp.data[0]);
    hart_encode_float(72.5f, &resp.data[4]);
    resp.data[8] = 32;
    hart_encode_float(35.0f, &resp.data[9]);
    resp.data[13] = 57;
    hart_encode_float(1013.0f, &resp.data[14]);
    resp.data[18] = 6;
    hart_encode_float(0.0f, &resp.data[19]);
    resp.data[23] = 0;
    resp.checksum = hart_compute_checksum(&resp);

    hart_device_variables_t vars;
    if (hart_parse_command_3_response(&resp, &vars)) {
        printf("  PV (Temperature): %.1f degC\n", vars.primary_variable);
        printf("  Loop Current:     %.2f mA\n", vars.loop_current_mA);
        printf("  SV (Humidity):    %.1f %%RH\n", vars.secondary_variable);
        printf("  TV (Pressure):    %.1f mbar\n", vars.tertiary_variable);
        printf("  Frame valid:      %s\n", hart_validate_frame(&resp) ? "YES" : "NO");
    }

    printf("\n--- Bell 202 FSK Modulation ---\n");
    hart_modulator_t mod;
    hart_modulator_init(&mod);
    printf("  Mark  frequency: %.0f Hz (logical 1)\n", mod.mark_frequency_hz);
    printf("  Space frequency: %.0f Hz (logical 0)\n", mod.space_frequency_hz);
    printf("  Amplitude:       %.1f mA peak\n", mod.amplitude_mA);
    printf("  Baud rate:       1200 bps\n");

    uint8_t test_bits[] = {1, 1, 0, 0, 1};
    printf("  Sample modulation (first 5 samples):\n");
    for (int i = 0; i < 5; i++) {
        double t = i / 9600.0;
        double s = hart_modulator_sample(&mod, test_bits[i], t);
        printf("    Bit=%d, t=%.4fms, amplitude=%.3f mA\n", test_bits[i], t*1000, s);
    }

    printf("\n--- HART Burst Mode ---\n");
    double period = hart_burst_period(1000.0);
    printf("  Requested: 1000 ms, Actual: %.0f ms\n", period);
    double turnaround = hart_turnaround_time_ms(20);
    printf("  Turnaround time (20 data bytes): %.0f ms\n", turnaround);

    printf("\n--- HART Signal on 4-20mA Loop ---\n");
    printf("  DC: 4-20 mA (analog measurement)\n");
    printf("  AC: +/-0.5 mA FSK (digital communication)\n");
    printf("  Zero mean AC preserves analog integrity\n");

    printf("\n=== HART Demo Complete ===\n");
    return 0;
}
