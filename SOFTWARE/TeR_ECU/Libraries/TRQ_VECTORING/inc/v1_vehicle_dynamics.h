#ifndef V1_VEHICLE_DYNAMICS_H
#define V1_VEHICLE_DYNAMICS_H

#include <stdint.h>
#include <stdbool.h>
#include "TeR_CONSTANTS.h"
#include "TeR_CAN.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Vehicle Physical Geometry (Formula Student Monocoque) */
#define V1_VEHICLE_WB        1.530f   /* Wheelbase L [m] */
#define V1_VEHICLE_TW        1.220f   /* Track Width w [m] */
#define V1_K_UNDERSTEER      0.0025f  /* Understeer Gradient K_u [rad/(m/s^2)] */

/* Controller Parameters Struct */
typedef struct {
    float kp_yaw;                 /* Proportional Gain [Nm / (rad/s)] */
    float ki_yaw;                 /* Integral Gain [Nm / rad] */
    float k_ff;                   /* Feedforward Gain [Nm / rad] */
    float max_yaw_moment_nm;      /* Maximum DYC Yaw Moment [Nm] */
    float pi_windup_limit_nm;     /* Integrator Anti-Windup Limit [Nm] */
    float peak_mu;                /* Peak Tire Friction Coefficient */

    /* Decoupled Traction Control Parameters */
    float max_allowable_slip;     /* Target Slip Ratio Threshold (0.12 = 12%) */
    float slip_cut_gain;          /* Smooth Attenuation Gain */
} v1_params_t;

/* Static Internal State (Zero Dynamic Memory Allocation) */
typedef struct {
    float error_integral_nm;      /* Integral accumulator */
    bool  cut_active_rl;          /* Left wheel slip hysteresis flag */
    bool  cut_active_rr;          /* Right wheel slip hysteresis flag */
    bool  initialized;
} v1_state_t;

/* Public API Functions */
void v1_init_params(v1_params_t *params);
void v1_reset_state(void);

/* Pipeline-Compatible Function Signature Wrappers */
trqMap_t v1_trqVectoring(trq_t limit);
trqMap_t v1_tractionControl(trqMap_t in);

#ifdef __cplusplus
}
#endif

#endif /* V1_VEHICLE_DYNAMICS_H */