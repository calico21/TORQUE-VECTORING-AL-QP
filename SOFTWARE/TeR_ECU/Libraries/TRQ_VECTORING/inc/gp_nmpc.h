#ifndef GP_NMPC_H
#define GP_NMPC_H

#include "gp_params.h"
#include "gp_vehicle_model.h"

#define GP_NMPC_N          8      // Condensed SQP-RTI horizon (was 10, control-hold). N=8 per spec.
#define GP_NMPC_DT         0.010f
#define GP_NMPC_STATES     2
#define GP_NMPC_INPUTS     1

// Gauss-Seidel sweeps/tick. RTI means ONE linearization + a warm-started
// solve per sample, not iterate-to-convergence — the shift from the prior
// tick's sequence carries the load, so this stays small and fixed
// (deterministic O(1), no early-exit branch to avoid timing jitter).
#define GP_NMPC_QP_ITER    6

// Full condensed control-SEQUENCE NMPC state. The prior struct held a
// single u_warm scalar (control-hold across the horizon); this carries the
// entire planned sequence so the RTI scheme can shift-and-warm-start it
// every tick instead of resolving from a cold start.
typedef struct {
    float x_pred[GP_NMPC_N + 1][GP_NMPC_STATES]; // Predicted traj: telemetry + next-tick frozen-gate source
    float A_d[GP_NMPC_STATES][GP_NMPC_STATES];
    float B_d[GP_NMPC_STATES][GP_NMPC_INPUTS];
    float u_seq[GP_NMPC_N];                       // Planned Mz sequence — RTI warm-start buffer
    float u_warm;                                 // Last EXTERNALLY applied Mz (gated by caller, unchanged semantics)
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