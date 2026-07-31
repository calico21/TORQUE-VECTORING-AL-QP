#ifndef GP_NMPC_H
#define GP_NMPC_H

#include "gp_params.h"

#define GP_NMPC_N          5
#define GP_NMPC_STATES     2
#define GP_NMPC_INPUTS     1

#ifndef GP_VEH_MASS
#define GP_VEH_MASS        230.0f
#define GP_VEH_IZ          110.0f
#define GP_VEH_LF          0.800f
#define GP_VEH_LR          0.730f
#define GP_VEH_CF          85000.0f
#define GP_VEH_CR          95000.0f
#endif

typedef struct {
    float x_pred[GP_NMPC_N + 1][GP_NMPC_STATES];
    float A_d[GP_NMPC_STATES][GP_NMPC_STATES]; // this tick's linearization (LTI over horizon)
    float B_d[GP_NMPC_STATES][GP_NMPC_INPUTS];
    float u_warm; // last ACTUALLY APPLIED Mz (post box+rate limit) — caller must write this back
} gp_nmpc_state_t;

void gp_nmpc_init(gp_nmpc_state_t *state);

void gp_nmpc_compute_jacobians(float v_x,
                               float A_c[GP_NMPC_STATES][GP_NMPC_STATES],
                               float B_c[GP_NMPC_STATES][GP_NMPC_INPUTS]);

// dt_ctrl      : REAL control-loop dt (no more hardcoded GP_NMPC_DT drift vs actual call rate)
// mz_max       : dynamic, friction/speed-derived Mz ceiling for THIS tick (caller computes)
// mz_rate_max  : max |Mz_k - Mz_{k-1}| this tick, derived from GP_TV_RATE_LIMIT*dt_ctrl,
//                so the optimizer's own plan already respects the actuator's real slew limit
void gp_nmpc_step(const float states[3],
                   float delta_sw,
                   float r_ref,
                   float dt_ctrl,
                   float mz_max,
                   float mz_rate_max,
                   gp_nmpc_state_t *nmpc_state,
                   float *mz_cmd);

#endif // GP_NMPC_H