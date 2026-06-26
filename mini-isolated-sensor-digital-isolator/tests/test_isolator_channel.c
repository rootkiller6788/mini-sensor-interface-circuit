/** @file test_isolator_channel.c */
#include "isolator_channel.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    printf("=== isolator_channel tests ===\n");
    digital_isolator_t iso;
    digital_isolator_init(&iso, ISOL_TECH_CAPACITIVE, ISOL_CLASS_REINFORCED, 4);
    isolation_channel_t ch;
    int rc = isolation_channel_init(&ch, 0, &iso);
    assert(rc == 0);
    assert(ch.channel_id == 0);
    assert(ch.state == CHAN_STATE_IDLE);
    isolation_channel_set_state(&ch, CHAN_STATE_ACTIVE);
    assert(ch.state == CHAN_STATE_ACTIVE);
    double il = isolation_channel_insertion_loss(&ch, 1e6);
    assert(il < 0.0);
    double delay = isolation_channel_propagation_delay(&ch, 1.0);
    assert(delay > 0.0);
    isolation_channel_capacity_compute(&ch, 100e6, 20.0);
    assert(ch.capacity.channel_capacity_bps > 100e6);
    double ber = isolation_channel_bit_error_rate(&ch, 15.0);
    assert(ber < 1e-7);
    isolation_channel_destroy(&ch);
    digital_isolator_destroy(&iso);
    printf("=== All channel tests passed ===\n");
    return 0;
}
