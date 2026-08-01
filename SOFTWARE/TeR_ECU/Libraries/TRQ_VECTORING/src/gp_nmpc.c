#include "gp_nmpc.h"
#include "gp_math.h"   // GP_CLAMP, GP_MIN
#include <math.h>
#include <string.h>

void gp_nmpc_init(gp_nmpc_state_t *state) {
    if (!state) return;
    memset(state, 0, sizeof(gp_nmpc_state_t));
    // Asigna los valores por defecto definidos en gp_params.h
    state->q_yaw    = GP_NMPC_Q_YAW;
    state->r_effort = GP_NMPC_R_EFFORT;
    state->r_slew   = GP_NMPC_R_SLEW;
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

void gp_nmpc_set_weights(gp_nmpc_state_t *state, float q_yaw, float r_effort, float r_slew) {
    if (!state) return;
    // Protección contra valores nulos o negativos que romperían la matriz QP
    state->q_yaw    = GP_MAX(q_yaw,    0.0f);
    state->r_effort = GP_MAX(r_effort, 0.5f);
    state->r_slew   = GP_MAX(r_slew,   0.5f);
}

void gp_nmpc_step(const float states[3], float delta_sw, float r_ref,
                   float dt_ctrl, float mz_max, float mz_rate_max,
                   gp_nmpc_state_t *nmpc_state, float *mz_cmd)
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

    // 1. Respuesta libre (u = 0)
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

    // 2. Sensibilidad d(r_pred)/d(Mz)
    float s0 = 0.0f, s1 = 0.0f, C[GP_NMPC_N];
    for (int k = 0; k < GP_NMPC_N; k++) {
        float ns0 = A_d[0][0]*s0 + A_d[0][1]*s1 + B_d[0];
        float ns1 = A_d[1][0]*s0 + A_d[1][1]*s1 + B_d[1];
        s0 = ns0; s1 = ns1;
        C[k] = s1;
    }

    // Normalize q_yaw against the SAME ΣC[k]² that appears in H, computed
    // ONCE from the live (unmodified) C[k] array — not against C[N-1] alone,
    // and NOT combined with re-normalizing C[k] itself inside the loop.
    // This way Q_r * ΣC[k]² == q_yaw exactly, by construction: q_yaw,
    // r_effort, and r_slew all land in the same comparable units regardless
    // of vx, with no compounding.
    float c_sum_sq = 0.0f;
    for (int k = 0; k < GP_NMPC_N; k++) {
        c_sum_sq += C[k] * C[k];
    }
    c_sum_sq = GP_MAX(c_sum_sq, 1e-12f);   // guard vx -> 0

    float Q_r    = nmpc_state->q_yaw / c_sum_sq;
    float R_u    = nmpc_state->r_effort;
    float R_slew = nmpc_state->r_slew;

    float H = R_u + R_slew;
    float g = -R_slew * nmpc_state->u_warm;
    for (int k = 0; k < GP_NMPC_N; k++) {
        float r_err = r_free[k] - r_ref;
        H += Q_r * C[k] * C[k];      // raw C[k] — no second normalization
        g += Q_r * C[k] * r_err;
    }
    float u_unc = -g / (H + 1e-8f);
    if (isnan(u_unc) || isinf(u_unc)) u_unc = 0.0f;

    // 4. Limitación suave de derivadas y magnitud
    float center = nmpc_state->u_warm;
    float delta_u = u_unc - center;
    
    float delta_mag = gp_soft_cap(fabsf(delta_u), mz_rate_max, 1.0f / GP_NMPC_SOFTNESS);
    float u_slewed = center + copysignf(delta_mag, delta_u);

    float mag = gp_soft_cap(fabsf(u_slewed), mz_max, 1.0f / GP_NMPC_SOFTNESS);
    *mz_cmd = copysignf(mag, u_slewed);

    // Guardar memoria de par previo para la penalización de slew rate del siguiente ciclo
    nmpc_state->u_warm = *mz_cmd;
}