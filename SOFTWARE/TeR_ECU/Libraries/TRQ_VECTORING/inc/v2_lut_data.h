#ifndef V2_LUT_DATA_H
#define V2_LUT_DATA_H

#include <stdint.h>
#include "v2_math.h"

/* Tabla 2D de Par Base: Pedal APPS [0.0 - 1.0] vs RPM de Motor [0 - 20000] */
static const float V2_PEDAL_AXIS[5] = {0.0f, 0.25f, 0.50f, 0.75f, 1.0f};
static const float V2_RPM_AXIS[6]   = {0.0f, 4000.0f, 8000.0f, 12000.0f, 16000.0f, 20000.0f};

/* Par entregado por rueda [Nm] (hasta 90 Nm por rueda / 180 Nm total en eje) */
static const float V2_DRIVE_TORQUE_MAP[5][6] = {
    { 0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f },  /* 0% pedal */
    {22.5f,  22.5f,  22.5f,  20.0f,  15.0f,  10.0f },  /* 25% pedal */
    {45.0f,  45.0f,  45.0f,  40.0f,  30.0f,  20.0f },  /* 50% pedal */
    {67.5f,  67.5f,  67.5f,  60.0f,  45.0f,  30.0f },  /* 75% pedal */
    {90.0f,  90.0f,  90.0f,  80.0f,  60.0f,  40.0f }   /* 100% pedal */
};

static inline float v2_interp2d_drive_torque(float apps_pct, float motor_rpm) {
    apps_pct  = V2_CLAMP(apps_pct, 0.0f, 1.0f);
    motor_rpm = V2_CLAMP(motor_rpm, 0.0f, 20000.0f);

    int i = 0;
    while (i < 3 && apps_pct >= V2_PEDAL_AXIS[i + 1]) i++;
    float alpha_apps = (apps_pct - V2_PEDAL_AXIS[i]) / (V2_PEDAL_AXIS[i + 1] - V2_PEDAL_AXIS[i]);

    int j = 0;
    while (j < 4 && motor_rpm >= V2_RPM_AXIS[j + 1]) j++;
    float alpha_rpm = (motor_rpm - V2_RPM_AXIS[j]) / (V2_RPM_AXIS[j + 1] - V2_RPM_AXIS[j]);

    float t00 = V2_DRIVE_TORQUE_MAP[i][j];
    float t01 = V2_DRIVE_TORQUE_MAP[i][j + 1];
    float t10 = V2_DRIVE_TORQUE_MAP[i + 1][j];
    float t11 = V2_DRIVE_TORQUE_MAP[i + 1][j + 1];

    float t0 = t00 + alpha_rpm * (t01 - t00);
    float t1 = t10 + alpha_rpm * (t11 - t10);

    return t0 + alpha_apps * (t1 - t0);
}

#endif /* V2_LUT_DATA_H */