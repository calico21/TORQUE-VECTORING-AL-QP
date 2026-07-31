#include "gp_nmpc.h"
#include <math.h>
#include <string.h>

void gp_nmpc_init(gp_nmpc_state_t *state) {
    if (!state) return;
    memset(state, 0, sizeof(gp_nmpc_state_t));
}

void gp_nmpc_compute_jacobians(float v_x, 
                               float A_c[GP_NMPC_STATES][GP_NMPC_STATES], 
                               float B_c[GP_NMPC_STATES][GP_NMPC_INPUTS]) 
{
    float v_x_safe = (fabsf(v_x) < 1.0f) ? 1.0f : fabsf(v_x);

    float m  = GP_VEH_MASS;
    float Iz = GP_VEH_IZ;
    float a  = GP_VEH_LF;
    float b  = GP_VEH_LR;
    float Cf = GP_VEH_CF;
    float Cr = GP_VEH_CR;

    A_c[0][0] = -(Cf + Cr) / (m * v_x_safe);
    A_c[0][1] = ((b * Cr - a * Cf) / (m * v_x_safe)) - v_x_safe;
    A_c[1][0] = (b * Cr - a * Cf) / (Iz * v_x_safe);
    A_c[1][1] = -(a * a * Cf + b * b * Cr) / (Iz * v_x_safe);

    B_c[0][0] = 0.0f;
    B_c[1][0] = 1.0f / Iz;
}

void gp_nmpc_predict_trajectory(const float x_0[GP_NMPC_STATES],
                                 float v_x,
                                 float delta_sw,
                                 const float u_seq[GP_NMPC_N],
                                 gp_nmpc_state_t *state) 
{
    if (!state || !x_0) return;

    float A_c[GP_NMPC_STATES][GP_NMPC_STATES];
    float B_c[GP_NMPC_STATES][GP_NMPC_INPUTS];

    gp_nmpc_compute_jacobians(v_x, A_c, B_c);

    state->x_pred[0][0] = x_0[0]; // v_y
    state->x_pred[0][1] = x_0[1]; // r

    float dt = GP_NMPC_DT;
    float m  = GP_VEH_MASS;
    float Iz = GP_VEH_IZ;
    float a  = GP_VEH_LF;
    float Cf = GP_VEH_CF;

    float f_steer_vy = (Cf / m) * delta_sw;
    float f_steer_r  = (a * Cf / Iz) * delta_sw;

    for (int k = 0; k < GP_NMPC_N; k++) {
        float vy_k = state->x_pred[k][0];
        float r_k  = state->x_pred[k][1];
        float Mz_k = (u_seq != NULL) ? u_seq[k] : 0.0f;

        state->A_d[k][0][0] = 1.0f + A_c[0][0] * dt;
        state->A_d[k][0][1] = A_c[0][1] * dt;
        state->A_d[k][1][0] = A_c[1][0] * dt;
        state->A_d[k][1][1] = 1.0f + A_c[1][1] * dt;

        state->B_d[k][0][0] = B_c[0][0] * dt;
        state->B_d[k][1][0] = B_c[1][0] * dt;

        float dot_vy = A_c[0][0] * vy_k + A_c[0][1] * r_k + f_steer_vy;
        float dot_r  = A_c[1][0] * vy_k + A_c[1][1] * r_k + B_c[1][0] * Mz_k + f_steer_r;

        state->x_pred[k + 1][0] = vy_k + dot_vy * dt;
        state->x_pred[k + 1][1] = r_k  + dot_r  * dt;
    }
}

void gp_nmpc_solve_qp(const gp_nmpc_state_t *nmpc,
                      float r_ref,
                      float *u_opt) 
{
    if (!nmpc || !u_opt) return;

    float Q_r    = 100000.0f; // Yaw rate tracking weight
    float R_u    = 0.01f;     // Control effort weight
    float R_slew = 0.02f;     // Rate change penalty

    float C[GP_NMPC_N] = {0};
    float cum_B = 0.0f;

    for (int k = 0; k < GP_NMPC_N; k++) {
        cum_B += nmpc->B_d[k][1][0];
        C[k] = cum_B;
    }

    float H_00  = R_u + R_slew;
    float num_g = 0.0f;

    for (int k = 0; k < GP_NMPC_N; k++) {
        float r_unforced = nmpc->x_pred[k + 1][1];
        float r_err      = r_unforced - r_ref;

        H_00  += Q_r * (C[k] * C[k]);
        num_g += Q_r * C[k] * r_err;
    }

    float u_prev = (isnan(nmpc->u_warm[0]) || isinf(nmpc->u_warm[0])) ? 0.0f : nmpc->u_warm[0];
    
    // Direct analytical horizon solution
    float u_0 = -(num_g - R_slew * u_prev) / (H_00 + 1e-8f);

    if (isnan(u_0) || isinf(u_0)) u_0 = 0.0f;

    if (u_0 > GP_NMPC_MZ_MAX)  u_0 = GP_NMPC_MZ_MAX;
    if (u_0 < -GP_NMPC_MZ_MAX) u_0 = -GP_NMPC_MZ_MAX;

    u_opt[0] = u_0;
    for (int k = 1; k < GP_NMPC_N; k++) {
        u_opt[k] = u_0 * 0.85f;
    }
}

void gp_nmpc_step(const float states[3], 
                  float delta_sw,
                  float r_ref,
                  gp_nmpc_state_t *nmpc_state,
                  float *mz_cmd) 
{
    if (!states || !nmpc_state || !mz_cmd) return;

    if (isnan(states[0]) || isnan(states[1]) || isnan(states[2]) || isnan(delta_sw) || isnan(r_ref)) {
        *mz_cmd = 0.0f;
        return;
    }

    float x_0[GP_NMPC_STATES] = { states[1], states[2] }; // [v_y, r]
    float v_x = states[0];

    // Predict unforced vehicle trajectory (u = 0 over horizon)
    gp_nmpc_predict_trajectory(x_0, v_x, delta_sw, NULL, nmpc_state);

    // Solve unconstrained/clamped optimal control step
    float u_opt[GP_NMPC_N];
    gp_nmpc_solve_qp(nmpc_state, r_ref, u_opt);

    *mz_cmd = u_opt[0];

    // Store current u_0 for slew reference in next cycle
    nmpc_state->u_warm[0] = u_opt[0];
}