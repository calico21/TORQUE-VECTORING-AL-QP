#ifndef V1_LUT_DATA_H
#define V1_LUT_DATA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define V1_LUT_PEDAL_STEPS 11
#define V1_LUT_RPM_STEPS   11

/* Accelerator Pedal Travel Axis [0.0 to 1.0] (0% to 100%) */
static const float V1_PEDAL_AXIS[V1_LUT_PEDAL_STEPS] = {
    0.00f, 0.10f, 0.20f, 0.30f, 0.40f, 0.50f, 0.60f, 0.70f, 0.80f, 0.90f, 1.00f
};

/* Motor Speed Axis [RPM] (0 to 20,000 RPM) */
static const float V1_RPM_AXIS[V1_LUT_RPM_STEPS] = {
    0.0f, 2000.0f, 4000.0f, 6000.0f, 8000.0f, 10000.0f, 12000.0f, 14000.0f, 16000.0f, 18000.0f, 20000.0f
};

/* 2D Drive Torque Demand Map [Nm] (Flash-Resident) */
static const float V1_DRIVE_TORQUE_MAP[V1_LUT_PEDAL_STEPS][V1_LUT_RPM_STEPS] = {
    {  0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f},
    {  2.0f,   5.0f,   5.0f,   5.0f,   5.0f,   4.5f,   4.0f,   3.5f,   3.0f,   2.0f,   0.0f},
    {  5.0f,  10.0f,  10.0f,  10.0f,  10.0f,   9.0f,   8.0f,   7.0f,   5.0f,   3.0f,   0.0f},
    {  8.0f,  15.0f,  15.0f,  15.0f,  15.0f,  13.5f,  12.0f,  10.0f,   7.5f,   4.0f,   0.0f},
    { 12.0f,  20.0f,  20.0f,  20.0f,  20.0f,  18.0f,  16.0f,  13.0f,  10.0f,   5.0f,   0.0f},
    { 15.0f,  25.0f,  25.0f,  25.0f,  25.0f,  22.5f,  20.0f,  16.0f,  12.0f,   6.0f,   0.0f},
    { 18.0f,  30.0f,  30.0f,  30.0f,  30.0f,  27.0f,  24.0f,  20.0f,  15.0f,   7.0f,   0.0f},
    { 21.0f,  35.0f,  35.0f,  35.0f,  35.0f,  31.5f,  28.0f,  23.0f,  17.0f,   8.0f,   0.0f},
    { 25.0f,  40.0f,  40.0f,  40.0f,  40.0f,  36.0f,  32.0f,  26.0f,  20.0f,   9.0f,   0.0f},
    { 28.0f,  45.0f,  45.0f,  45.0f,  45.0f,  40.5f,  36.0f,  30.0f,  22.0f,  10.0f,   0.0f},
    { 30.0f,  50.0f,  50.0f,  50.0f,  50.0f,  45.0f,  40.0f,  33.0f,  25.0f,  12.0f,   0.0f}
};

/* Fast Inline Bilinear Interpolation Routine */
static inline float v1_interp2d_drive_torque(float pedal, float rpm) {
    if (pedal <= V1_PEDAL_AXIS[0]) return 0.0f;
    if (pedal >= V1_PEDAL_AXIS[V1_LUT_PEDAL_STEPS - 1]) pedal = V1_PEDAL_AXIS[V1_LUT_PEDAL_STEPS - 1];
    if (rpm <= V1_RPM_AXIS[0]) rpm = V1_RPM_AXIS[0];
    if (rpm >= V1_RPM_AXIS[V1_LUT_RPM_STEPS - 1]) rpm = V1_RPM_AXIS[V1_LUT_RPM_STEPS - 1];

    uint8_t i = 0, j = 0;
    while (i < V1_LUT_PEDAL_STEPS - 2 && V1_PEDAL_AXIS[i + 1] < pedal) i++;
    while (j < V1_LUT_RPM_STEPS - 2 && V1_RPM_AXIS[j + 1] < rpm) j++;

    float x1 = V1_PEDAL_AXIS[i],     x2 = V1_PEDAL_AXIS[i + 1];
    float y1 = V1_RPM_AXIS[j],       y2 = V1_RPM_AXIS[j + 1];

    float t = (pedal - x1) / (x2 - x1);
    float u = (rpm - y1) / (y2 - y1);

    float f00 = V1_DRIVE_TORQUE_MAP[i][j];
    float f10 = V1_DRIVE_TORQUE_MAP[i + 1][j];
    float f01 = V1_DRIVE_TORQUE_MAP[i][j + 1];
    float f11 = V1_DRIVE_TORQUE_MAP[i + 1][j + 1];

    return (1.0f - t) * (1.0f - u) * f00 + t * (1.0f - u) * f10 + (1.0f - t) * u * f01 + t * u * f11;
}

#ifdef __cplusplus
}
#endif

#endif /* V1_LUT_DATA_H */