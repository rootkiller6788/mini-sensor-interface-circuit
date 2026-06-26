#include "mems_accel.h"
#include "mems_calibration.h"
#include <stdio.h>
#include <math.h>

int main(void)
{
    printf("=== MEMS Accelerometer 6-Position Calibration ===\n\n");

    mems_accel_model_t model;
    mems_accel_model_init(&model, ACCEL_FS_8G, ACCEL_MODE_OPEN_LOOP);

    /* Simulate 6-position calibration with known gravity reference */
    double expected[6][3] = {
        { 1.0,  0.0,  0.0},
        {-1.0,  0.0,  0.0},
        { 0.0,  1.0,  0.0},
        { 0.0, -1.0,  0.0},
        { 0.0,  0.0,  1.0},
        { 0.0,  0.0, -1.0}
    };

    int32_t measured[6][3] = {
        { 17000,    100,    -50},
        {-16500,    -50,     20},
        {    50,  16800,    -30},
        {   -80, -16700,     40},
        {    30,     60,  16900},
        {   -20,    -40, -16800}
    };

    accel_calib_t calib;
    int ret = mems_accel_six_pos_calibrate(expected, measured, 6, &calib);
    printf("Calibration result: %s\n", ret == 0 ? "SUCCESS" : "FAILED");
    printf("  Bias X: %.2f mg, Scale X: %.6f\n", calib.bias_x_mg, calib.scale_x);
    printf("  Bias Y: %.2f mg, Scale Y: %.6f\n", calib.bias_y_mg, calib.scale_y);
    printf("  Bias Z: %.2f mg, Scale Z: %.6f\n", calib.bias_z_mg, calib.scale_z);

    /* Apply calibration to a test sample */
    double cx, cy, cz;
    mems_accel_apply_calib(&calib, 16500, 0, 0, &cx, &cy, &cz);
    printf("\nCalibrated sample (+1g X):\n");
    printf("  X: %.3f g, Y: %.3f g, Z: %.3f g\n", cx/1000.0, cy/1000.0, cz/1000.0);

    printf("\n=== Calibration Demo Complete ===\n");
    return 0;
}
