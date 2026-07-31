#ifndef V2_MATH_H
#define V2_MATH_H

#include <math.h>

#define V2_MAX(a, b) ((a) > (b) ? (a) : (b))
#define V2_MIN(a, b) ((a) < (b) ? (a) : (b))
#define V2_CLAMP(x, lo, hi) V2_MIN(V2_MAX((x), (lo)), (hi))

#endif /* V2_MATH_H */