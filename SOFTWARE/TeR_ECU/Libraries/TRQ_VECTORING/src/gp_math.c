/*
 * gp_math.c
 */

#include "gp_math.h"

// Recreación de jax.nn.softplus
float gp_softplus(float x) {
    // Si x es muy grande, expf(x) produce overflow. 
    // softplus(x) tiende a x para x > 20
    if (x > 20.0f) {
        return x;
    }
    return logf(1.0f + expf(x));
}

// Recreación de jax.nn.sigmoid
float gp_sigmoid(float x) {
    // Limitar x para evitar underflow/overflow en expf
    float x_clamped = GP_CLAMP(x, -20.0f, 20.0f);
    return 1.0f / (1.0f + expf(-x_clamped));
}

// Para el límite de la elipse de fricción: sqrt(softplus(x))
float gp_softplus_sqrt(float x) {
    float sp = gp_softplus(x);
    // Añadimos un pequeño epsilon para evitar sqrt(0)
    return sqrtf(sp + 1e-6f);
}

// Interpolación bilineal 4x4 extraída de _bilinear_interp_4x4
float gp_bilinear_interp_4x4(const float table[16], float x_norm, float y_norm) {
    // Mapeamos [0, 1] a la cuadrícula 3x3 (índices 0 a 3)
    float gx = GP_CLAMP(x_norm * 3.0f, 0.0f, 2.99999f);
    float gy = GP_CLAMP(y_norm * 3.0f, 0.0f, 2.99999f);

    int x0 = (int)gx;
    int y0 = (int)gy;
    int x1 = GP_CLAMP(x0 + 1, 0, 3);
    int y1 = GP_CLAMP(y0 + 1, 0, 3);

    float fx = gx - (float)x0;
    float fy = gy - (float)y0;

    // table se lee de forma row-major: índice = row * 4 + col
    float c00 = table[x0 * 4 + y0];
    float c10 = table[x1 * 4 + y0];
    float c01 = table[x0 * 4 + y1];
    float c11 = table[x1 * 4 + y1];

    // Fórmula bilineal
    return c00 * (1.0f - fx) * (1.0f - fy) +
           c10 * fx          * (1.0f - fy) +
           c01 * (1.0f - fx) * fy +
           c11 * fx          * fy;
}