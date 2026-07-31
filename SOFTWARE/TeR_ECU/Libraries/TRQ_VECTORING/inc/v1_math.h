#ifndef V1_MATH_H
#define V1_MATH_H
#include <math.h>

/* Deliberately NOT TeR_UTILS.h — that header pulls stm32f4xx_hal.h + cmsis_os.h
 * transitively, which is exactly the coupling that makes v1_vehicle_dynamics.c
 * unbuildable as a standalone .so for SIL regression. Same three-line idiom
 * gp_math.h already uses. */
#define V1_MAX(a, b) ((a) > (b) ? (a) : (b))
#define V1_MIN(a, b) ((a) < (b) ? (a) : (b))
#define V1_CLAMP(x, lo, hi) V1_MIN(V1_MAX((x), (lo)), (hi))

#endif /* V1_MATH_H */