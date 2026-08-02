#ifndef GP_NMPC_H
#define GP_NMPC_H

#include "gp_params.h"
#include "gp_vehicle_model.h"

#define GP_NMPC_N          10
#define GP_NMPC_DT         0.010f
#define GP_NMPC_STATES     2
#define GP_NMPC_INPUTS     1

// Control-horizon-1 batch QP: single decision variable (u_0) held over the
// GP_NMPC_N-step prediction. A_d/B_d/u_warm are single-stage, NOT
// per-horizon-step — x_pred is kept per-step only for telemetry.
typedef struct {
    float x_pred[GP_NMPC_N + 1][GP_NMPC_STATES];
    float A_d[GP_NMPC_STATES][GP_NMPC_STATES];
    float B_d[GP_NMPC_STATES][GP_NMPC_INPUTS];
    float u_warm;
    // Pesos de costo reconfigurables en runtime:
    float q_yaw;
    float r_effort;
    float r_slew;
} gp_nmpc_state_t;

void gp_nmpc_set_weights(gp_nmpc_state_t *state, float q_yaw, float r_effort, float r_slew);

void gp_nmpc_init(gp_nmpc_state_t *state);

void gp_nmpc_compute_jacobians(float v_x, float mu_scale,
                               float A_c[GP_NMPC_STATES][GP_NMPC_STATES],
                               float B_c[GP_NMPC_STATES][GP_NMPC_INPUTS]);

void gp_nmpc_step(const float states[3], float delta_sw, float r_ref,
                   float dt_ctrl, float mz_max, float mz_rate_max,
                   float mu_scale,
                   gp_nmpc_state_t *nmpc_state, float *mz_cmd);

#endif // GP_NMPC_H