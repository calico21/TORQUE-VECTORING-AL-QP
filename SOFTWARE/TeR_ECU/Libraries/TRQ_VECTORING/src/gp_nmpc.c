#include "gp_nmpc.h"
#include "gp_math.h"   // GP_CLAMP, GP_MIN
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
    float m = GP_VEH_MASS, Iz = GP_VEH_IZ, a = GP_VEH_LF, b = GP_VEH_LR;
    float Cf = GP_VEH_CF, Cr = GP_VEH_CR;

    A_c[0][0] = -(Cf + Cr) / (m * v_x_safe);
    A_c[0][1] = ((b * Cr - a * Cf) / (m * v_x_safe)) - v_x_safe;
    A_c[1][0] = (b * Cr - a * Cf) / (Iz * v_x_safe);
    A_c[1][1] = -(a * a * Cf + b * b * Cr) / (Iz * v_x_safe);

    B_c[0][0] = 0.0f;
    B_c[1][0] = 1.0f / Iz;
}

static void gp_nmpc_discretize(float v_x, float dt, float A_d[2][2], float B_d[2]) {
    float A_c[2][2], B_c[2][1];
    gp_nmpc_compute_jacobians(v_x, A_c, B_c);
    A_d[0][0] = 1.0f + A_c[0][0] * dt;
    A_d[0][1] = A_c[0][1] * dt;
    A_d[1][0] = A_c[1][0] * dt;
    A_d[1][1] = 1.0f + A_c[1][1] * dt;
    B_d[0] = B_c[0][0] * dt;
    B_d[1] = B_c[1][0] * dt;
}

void gp_nmpc_step(const float states[3],
                   float delta_sw,
                   float r_ref,
                   float dt_ctrl,
                   float mz_max,
                   float mz_rate_max,
                   gp_nmpc_state_t *nmpc_state,
                   float *mz_cmd)
{
    if (!states || !nmpc_state || !mz_cmd) return;
    if (isnan(states[0]) || isnan(states[1]) || isnan(states[2]) ||
        isnan(delta_sw) || isnan(r_ref) || dt_ctrl <= 0.0f) {
        *mz_cmd = 0.0f;
        return;
    }

    float vx = states[0], vy0 = states[1], r0 = states[2];

    float A_d[2][2], B_d[2];
    gp_nmpc_discretize(vx, dt_ctrl, A_d, B_d);
    nmpc_state->A_d[0][0]=A_d[0][0]; nmpc_state->A_d[0][1]=A_d[0][1];
    nmpc_state->A_d[1][0]=A_d[1][0]; nmpc_state->A_d[1][1]=A_d[1][1];
    nmpc_state->B_d[0][0]=B_d[0];    nmpc_state->B_d[1][0]=B_d[1];

    float m = GP_VEH_MASS, Iz = GP_VEH_IZ, a = GP_VEH_LF, Cf = GP_VEH_CF;
    float f_vy = (Cf / m) * delta_sw * dt_ctrl;
    float f_r  = (a * Cf / Iz) * delta_sw * dt_ctrl;

    // 1. Free response (u = 0) — identical role to the old x_pred, kept for telemetry.
    float x0 = vy0, x1 = r0;
    nmpc_state->x_pred[0][0] = x0;
    nmpc_state->x_pred[0][1] = x1;
    float r_free[GP_NMPC_N];
    for (int k = 0; k < GP_NMPC_N; k++) {
        float nx0 = A_d[0][0]*x0 + A_d[0][1]*x1 + f_vy;
        float nx1 = A_d[1][0]*x0 + A_d[1][1]*x1 + f_r;
        x0 = nx0; x1 = nx1;
        nmpc_state->x_pred[k+1][0] = x0;
        nmpc_state->x_pred[k+1][1] = x1;
        r_free[k] = x1;
    }

    // 2. CORRECTED sensitivity: d(r_pred[k+1])/d(Mz held constant 0..k),
    // via the SAME A_d,B_d recursion used above — replaces the old plain
    // cumulative-sum-of-B, which implicitly assumed A == Identity and so
    // ignored yaw-rate self-damping. This was the dominant reason the old
    // internal gain model didn't match the plant it was built from.
    float s0 = 0.0f, s1 = 0.0f, C[GP_NMPC_N];
    for (int k = 0; k < GP_NMPC_N; k++) {
        float ns0 = A_d[0][0]*s0 + A_d[0][1]*s1 + B_d[0];
        float ns1 = A_d[1][0]*s0 + A_d[1][1]*s1 + B_d[1];
        s0 = ns0; s1 = ns1;
        C[k] = s1;
    }

    // 3. Single-decision-variable batch QP (control horizon = 1, held over
    // the prediction horizon) — now actually using the tuned weights from
    // gp_params.h instead of local magic numbers.
    float Q_r = GP_NMPC_Q_YAW, R_u = GP_NMPC_R_EFFORT, R_slew = GP_NMPC_R_SLEW;
    float H = R_u + R_slew;
    float g = -R_slew * nmpc_state->u_warm;
    for (int k = 0; k < GP_NMPC_N; k++) {
        float r_err = r_free[k] - r_ref;
        H += Q_r * C[k] * C[k];
        g += Q_r * C[k] * r_err;
    }
    float u_unc = -g / (H + 1e-8f);
    if (isnan(u_unc) || isinf(u_unc)) u_unc = 0.0f;

    // 4. Box + slew constraints THE OPTIMIZER IS AWARE OF this time: slew
    // is measured against u_warm, which the caller updates with the
    // ACTUALLY APPLIED torque post box-constraint/rate-limit — not the
    // stale unconstrained value the old code fed back to itself.
    float lo = GP_MAX(-mz_max, nmpc_state->u_warm - mz_rate_max);
    float hi = GP_MIN( mz_max, nmpc_state->u_warm + mz_rate_max);
    if (hi < lo) { float t = hi; hi = lo; lo = t; }

    *mz_cmd = GP_CLAMP(u_unc, lo, hi);
    // u_warm is intentionally NOT updated here — see caller.
}