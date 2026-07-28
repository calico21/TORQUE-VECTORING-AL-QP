/*
 * gp_math.h
 * Utilidades matemáticas para Torque Vectoring & Traction Control
 * Optimizadas para FPU de 32-bit (STM32F4)
 */

#ifndef GP_MATH_H
#define GP_MATH_H

#include <math.h>

// Macros de conveniencia
#define GP_MAX(a, b) ((a) > (b) ? (a) : (b))
#define GP_MIN(a, b) ((a) < (b) ? (a) : (b))
#define GP_CLAMP(val, min, max) GP_MIN(GP_MAX((val), (min)), (max))

// Constantes físicas
#define GP_GRAVITY 9.81f
#define GP_PI 3.1415926535f

// Funciones de activación C-mooth (Suaves y diferenciables)
float gp_softplus(float x);
float gp_sigmoid(float x);

// Interpolación bilineal 2D (4x4) para mapas de ganancias (Kp, Kd, Ki)
float gp_bilinear_interp_4x4(const float table[16], float x_norm, float y_norm);

// Aproximación rápida para el límite del Friction Ellipse
float gp_softplus_sqrt(float x);

// Smooth ceiling: y = gp_soft_cap(x, cap, beta) satisfies y < cap for ALL x
// (strict, by construction — softplus(z) > 0 everywhere), y -> x for x << cap,
// y -> cap for x >> cap, and is C1-continuous through the boundary. Use this
// instead of `if (x > cap) scale = cap/x;` anywhere the trigger signal is
// noisy near the boundary — the hard branch flips every tick under sensor
// noise, this doesn't.
float gp_soft_cap(float x, float cap, float beta);

#endif // GP_MATH_H