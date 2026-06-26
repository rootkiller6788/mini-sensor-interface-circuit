/**
 * example_load_cell.c - Industrial weighing system demo
 * Knowledge: L7 Application - ISO 9000 industrial weighing
 * Course: TU Munich EI0430, THU Sensor Technology
 */
#include <stdio.h>
#include "bridge_applications.h"

int main(void) {
    printf("=== 4-Cell Platform Scale ===\n\n");
    weighing_system_t ws;
    weighing_system_init(&ws);
    printf("Cells: %d, Rated output: %.1f mV/V, Capacity: %.0f kg\n",
           ws.n_cells, ws.cells[0].rated_output_mv_per_v, ws.max_capacity_kg);
    double factors[4] = {0.9998, 1.0002, 0.9995, 1.0005};
    printf("Corner factors: %.4f %.4f %.4f %.4f\n\n",
           factors[0], factors[1], factors[2], factors[3]);

    double loads[] = {0, 50, 200, 500, 1000};
    int i;
    for (i = 0; i < 5; i++) {
        double total = loads[i];
        double r[4] = {total/4, total/4, total/4, total/4};
        memcpy(ws.corner_factors, factors, sizeof(factors));
        double m = application_weighing_total(&ws, r, 4);
        printf("Total=%.0f kg -> Measured=%.1f kg\n", total, m);
    }
    printf("\n[PASS] Weighing system demo complete.\n");
    return 0;
}
