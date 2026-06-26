/**
 * @file hart_protocol.c
 * @brief HART (Highway Addressable Remote Transducer) protocol implementation.
 *        FSK modulation/demodulation, frame building/parsing, burst mode.
 * Knowledge: L8 (HART protocol), L5 (FSK modulation), L6 (industrial protocol).
 */
#include "current_loop.h"
#include "hart_protocol.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void hart_modulator_init(hart_modulator_t *mod)
{
    if (!mod) return;
    mod->mark_frequency_hz = 1200.0;
    mod->space_frequency_hz = 2200.0;
    mod->amplitude_mA = 0.5;
    mod->sample_rate_hz = 9600.0;
    mod->phase_radians = 0.0;
}

double hart_modulator_sample(hart_modulator_t *mod, uint8_t bit, double t_sample)
{
    if (!mod) return 0.0;
    (void)t_sample;  /* sample time reserved for time-aware modulation */
    double freq = (bit) ? mod->mark_frequency_hz : mod->space_frequency_hz;
    double phase_step = 2.0 * M_PI * freq / mod->sample_rate_hz;
    mod->phase_radians += phase_step;
    if (mod->phase_radians > 2.0 * M_PI)
        mod->phase_radians -= 2.0 * M_PI;
    return mod->amplitude_mA * sin(mod->phase_radians);
}

uint8_t hart_demodulate_bit(double mark_energy, double space_energy)
{
    return (mark_energy > space_energy) ? 1 : 0;
}

void hart_build_command_0(hart_frame_t *frame, uint8_t address)
{
    if (!frame) return;
    memset(frame, 0, sizeof(*frame));
    frame->preamble_count = 5;
    frame->delimiter = 0x02;
    frame->address[0] = address;
    frame->address_length = 1;
    frame->command = HART_CMD_READ_DEVICE_INFO;
    frame->byte_count = 0;
    frame->data_length = 0;
    frame->checksum = hart_compute_checksum(frame);
}

void hart_build_command_3(hart_frame_t *frame, uint8_t address)
{
    if (!frame) return;
    memset(frame, 0, sizeof(*frame));
    frame->preamble_count = 5;
    frame->delimiter = 0x02;
    frame->address[0] = address;
    frame->address_length = 1;
    frame->command = HART_CMD_READ_PV_CURRENT;
    frame->byte_count = 0;
    frame->data_length = 0;
    frame->checksum = hart_compute_checksum(frame);
}

bool hart_parse_command_3_response(const hart_frame_t *frame, hart_device_variables_t *vars)
{
    if (!frame || !vars || frame->data_length < 24) return false;
    memset(vars, 0, sizeof(*vars));
    vars->loop_current_mA = hart_parse_float(&frame->data[0]);
    vars->primary_variable = hart_parse_float(&frame->data[4]);
    vars->pv_unit_code = frame->data[8];
    vars->secondary_variable = hart_parse_float(&frame->data[9]);
    vars->sv_unit_code = frame->data[13];
    vars->tertiary_variable = hart_parse_float(&frame->data[14]);
    vars->tv_unit_code = frame->data[18];
    vars->quaternary_variable = hart_parse_float(&frame->data[19]);
    vars->qv_unit_code = frame->data[23];
    return true;
}

bool hart_parse_command_0_response(const hart_frame_t *frame, hart_device_info_t *info)
{
    if (!frame || !info || frame->data_length < 9) return false;
    info->manufacturer_id = frame->data[0];
    info->device_type = frame->data[1];
    info->device_id[0] = frame->data[2];
    info->device_id[1] = frame->data[3];
    info->device_id[2] = frame->data[4];
    info->hart_revision = frame->data[5];
    info->device_revision = frame->data[6];
    info->software_revision = frame->data[7];
    info->preambles_required = frame->data[8];
    info->flags = (frame->data_length > 9) ? frame->data[9] : 0;
    return true;
}

uint8_t hart_recommended_preambles(uint8_t preambles_required)
{
    if (preambles_required < HART_PREAMBLE_MIN) return HART_PREAMBLE_MIN;
    if (preambles_required > HART_PREAMBLE_MAX) return HART_PREAMBLE_MAX;
    return preambles_required;
}

bool hart_detect_signal(const double *ac_samples, size_t n, double sample_rate)
{
    if (!ac_samples || n < 100 || sample_rate <= 0.0) return false;
    double energy = 0.0;
    for (size_t i = 0; i < n; i++)
        energy += ac_samples[i] * ac_samples[i];
    energy /= (double)n;
    return energy > 0.001;
}

double hart_turnaround_time_ms(size_t data_length)
{
    double byte_time_ms = (11.0 / 1200.0) * 1000.0;
    return (data_length + 5) * byte_time_ms + 50.0;
}

double hart_burst_period(double burst_period_ms)
{
    if (burst_period_ms < 250.0) return 250.0;
    if (burst_period_ms > 3600000.0) return 3600000.0;
    return burst_period_ms;
}
