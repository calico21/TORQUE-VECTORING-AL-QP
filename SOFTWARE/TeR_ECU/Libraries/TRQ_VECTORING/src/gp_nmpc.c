#include "gp_nmpc.h"
#include "gp_math.h"
#include <math.h>
#include <string.h>

void gp_nmpc_init(gp_nmpc_state_t *state) {
    if (!state) return;
    memset(state, 0, sizeof(gp_nmpc_state_t));
    state->q_yaw    = GP_NMPC_Q_YAW;
    state->r_effort = GP_NMPC_R_EFFORT;
    state->r_slew   = GP_NMPC_R_SLEW;
}

void gp_nmpc_set_weights(gp_nmpc_state_t *state, float q_yaw, float r_effort, float r_slew) {
    if (!state) return;
    state->q_yaw    = GP_MAX(q_yaw,    0.0f);
    state->r_effort = GP_MAX(r_effort, 0.5f);
    state->r_slew   = GP_MAX(r_slew,   0.5f);
}

void gp_nmpc_compute_jacobians(float v_x, float mu_scale,
                               float A_c[GP_NMPC_STATES][GP_NMPC_STATES],
                               float B_c[GP_NMPC_STATES][GP_NMPC_INPUTS])
{
    float v_x_safe = (fabsf(v_x) < 1.0f) ? 1.0f : fabsf(v_x);
    float m = GP_MASS, Iz = GP_IZ, a = GP_LF, b = GP_LR;
    float Cf = GP_C_ALPHA_F * mu_scale, Cr = GP_C_ALPHA_R * mu_scale;

    A_c[0][0] = -(Cf + Cr) / (m * v_x_safe);
    A_c[0][1] = ((b * Cr - a * Cf) / (m * v_x_safe)) - v_x_safe;
    A_c[1][0] = (b * Cr - a * Cf) / (Iz * v_x_safe);
    A_c[1][1] = -(a * a * Cf + b * b * Cr) / (Iz * v_x_safe);

    B_c[0][0] = 0.0f;
    B_c[1][0] = 1.0f / Iz;
}

/*
 * gp_nmpc_discretize_rk2 — second-order (RK2 / truncated matrix-exponential)
 * discretization, replacing the prior first-order Euler (A_d = I + dt*A_c).
 *
 *   A_d = I + dt*A_c + (dt^2/2)*A_c^2
 *   B_d = dt*B_c + (dt^2/2)*A_c*B_c        (same form applied to Bdelta_c)
 *
 * Why this matters now and didn't (as much) before: at a single dt=5ms step
 * Euler's O(dt^2) local truncation error is small relative to the yaw
 * dynamics' time constant. But the condensed QP below propagates this same
 * A_d/B_d pair N=8 stages deep (~40ms of preview) via repeated
 * multiplication — the error compounds geometrically with horizon depth,
 * not linearly with a single step. A stale/inaccurate sensitivity trajectory
 * degrades exactly the thing this upgrade exists to deliver: a horizon the
 * controller can actually trust to plan against.
 */
static void gp_nmpc_discretize_rk2(float v_x, float dt, float mu_scale,
                                    float A_d[2][2], float B_d[2], float Bdelta_d[2])
{
    float A_c[2][2], B_c[2][1];
    gp_nmpc_compute_jacobians(v_x, mu_scale, A_c, B_c);

    float Cf = GP_C_ALPHA_F * mu_scale;
    // Exogenous steering-forcing channel: xdot += Bdelta_c * delta. Same
    // physical structure as B_c but for the driver's steering input rather
    // than the commanded Mz — needed so the free-response (u=0) trajectory
    // the QP condenses against still reflects "the car is mid-corner."
    float Bdelta_c[2] = { Cf / GP_MASS, (GP_LF * Cf) / GP_IZ };

    float A2[2][2] = {
        { A_c[0][0]*A_c[0][0] + A_c[0][1]*A_c[1][0], A_c[0][0]*A_c[0][1] + A_c[0][1]*A_c[1][1] },
        { A_c[1][0]*A_c[0][0] + A_c[1][1]*A_c[1][0], A_c[1][0]*A_c[0][1] + A_c[1][1]*A_c[1][1] }
    };
    float half_dt2 = 0.5f * dt * dt;

    A_d[0][0] = 1.0f + dt*A_c[0][0] + half_dt2*A2[0][0];
    A_d[0][1] =        dt*A_c[0][1] + half_dt2*A2[0][1];
    A_d[1][0] =        dt*A_c[1][0] + half_dt2*A2[1][0];
    A_d[1][1] = 1.0f + dt*A_c[1][1] + half_dt2*A2[1][1];

    // A_c * B_c  (B_c[0][0] is structurally 0 — Mz has no direct lateral-accel channel)
    float AB[2] = {
        A_c[0][0]*B_c[0][0] + A_c[0][1]*B_c[1][0],
        A_c[1][0]*B_c[0][0] + A_c[1][1]*B_c[1][0]
    };
    B_d[0] = dt*B_c[0][0] + half_dt2*AB[0];
    B_d[1] = dt*B_c[1][0] + half_dt2*AB[1];

    float ABd[2] = {
        A_c[0][0]*Bdelta_c[0] + A_c[0][1]*Bdelta_c[1],
        A_c[1][0]*Bdelta_c[0] + A_c[1][1]*Bdelta_c[1]
    };
    Bdelta_d[0] = dt*Bdelta_c[0] + half_dt2*ABd[0];
    Bdelta_d[1] = dt*Bdelta_c[1] + half_dt2*ABd[1];
}

void gp_nmpc_step(const float states[3], float delta_sw, float r_ref,
                   float dt_ctrl, float mz_max, float mz_rate_max,
                   float mu_scale,
                   gp_nmpc_state_t *nmpc_state, float *mz_cmd)
{
    if (!states || !nmpc_state || !mz_cmd) return;
    if (isnan(states[0]) || isnan(states[1]) || isnan(states[2]) ||
        isnan(delta_sw) || isnan(r_ref) || dt_ctrl <= 0.0f) {
        *mz_cmd = 0.0f;
        return;
    }

    const int N = GP_NMPC_N;
    float vx = states[0], vy0 = states[1], r0 = states[2];
    float vx_safe = GP_MAX(fabsf(vx), 0.5f);

    // 1. One linearization for the whole horizon (RTI: relinearize per
    //    SAMPLE, not per stage or per SQP sub-iteration).
    float A_d[2][2], B_d[2], Bdelta_d[2];
    gp_nmpc_discretize_rk2(vx, dt_ctrl, mu_scale, A_d, B_d, Bdelta_d);

    nmpc_state->A_d[0][0]=A_d[0][0]; nmpc_state->A_d[0][1]=A_d[0][1];
    nmpc_state->A_d[1][0]=A_d[1][0]; nmpc_state->A_d[1][1]=A_d[1][1];
    nmpc_state->B_d[0][0]=B_d[0];    nmpc_state->B_d[1][0]=B_d[1];

    // 2. Free response (u≡0), steering held constant across the preview —
    //    the only assumption available without a future-steering oracle.
    float x_free[GP_NMPC_N + 1][2];
    x_free[0][0] = vy0; x_free[0][1] = r0;
    for (int k = 0; k < N; k++) {
        x_free[k+1][0] = A_d[0][0]*x_free[k][0] + A_d[0][1]*x_free[k][1] + Bdelta_d[0]*delta_sw;
        x_free[k+1][1] = A_d[1][0]*x_free[k][0] + A_d[1][1]*x_free[k][1] + Bdelta_d[1]*delta_sw;
    }

    // 3. Condensing: since the model is LTI over this frozen linearization,
    //    the sensitivity of stage-k output to an earlier input u_j depends
    //    only on m=k-j. Build the impulse response once (2-state, so this
    //    is a 2x2 matrix-vector recursion, not a general N^2 Jacobian build).
    //    g_sens = d(yaw rate)/d(u), h_sens = d(vy)/d(u).
    float g_sens[GP_NMPC_N + 1] = {0};
    float h_sens[GP_NMPC_N + 1] = {0};
    {
        float s0 = 0.0f, s1 = 0.0f;
        for (int m = 1; m <= N; m++) {
            float ns0 = A_d[0][0]*s0 + A_d[0][1]*s1 + B_d[0];
            float ns1 = A_d[1][0]*s0 + A_d[1][1]*s1 + B_d[1];
            s0 = ns0; s1 = ns1;
            h_sens[m] = s0;
            g_sens[m] = s1;
        }
    }

    // 4. Frozen beta-gate weights from the PREVIOUS solve's trajectory.
    //    This is the Gauss-Newton move: the sideslip barrier's sigmoid gate
    //    is nonlinear in u, so RTI linearizes it at last tick's iterate
    //    instead of re-deriving it mid-solve — turning "J^T J" into an
    //    exact, cheap, positive-semidefinite quadratic form this tick.
    float gate_prev[GP_NMPC_N + 1];
    for (int k = 0; k <= N; k++) {
        float beta_prev = nmpc_state->x_pred[k][0] / vx_safe;
        gate_prev[k] = gp_sigmoid((fabsf(beta_prev) - GP_NMPC_BETA_MAX) * GP_NMPC_BETA_SHARPNESS);
    }

    // 5. Condensed dense Gauss-Newton Hessian H (NxN) + gradient g (N).
    //    All per-term scale factors consistently drop the usual "2x" from
    //    d/du(x^2)=2x -- valid because EVERY quadratic term here (tracking,
    //    effort, slew) drops it identically, so the stationarity condition
    //    H*u + g = 0 is unaffected (equivalent to minimizing 0.5*J instead
    //    of J -- same argmin).
    float H[GP_NMPC_N][GP_NMPC_N];
    float grad[GP_NMPC_N];
    memset(H, 0, sizeof(H));
    memset(grad, 0, sizeof(grad));

    float Q_r    = nmpc_state->q_yaw / (float)N;
    float Q_beta = GP_NMPC_Q_BETA    / (float)N;
    float R_u    = nmpc_state->r_effort;
    float R_slew = nmpc_state->r_slew;

    for (int k = 1; k <= N; k++) {
        float r_err_free = x_free[k][1] - r_ref;
        float beta_free  = x_free[k][0] / vx_safe;
        float wbeta       = Q_beta * gate_prev[k];

        for (int j = 0; j < k; j++) {
            float Gkj = g_sens[k - j];
            float Hkj = h_sens[k - j] / vx_safe;

            grad[j] += Q_r * Gkj * r_err_free + wbeta * Hkj * beta_free;

            for (int l = 0; l <= j; l++) {
                float Gkl = g_sens[k - l];
                float Hkl = h_sens[k - l] / vx_safe;
                float Hval = Q_r * Gkj * Gkl + wbeta * Hkj * Hkl;
                H[j][l] += Hval;
                if (l != j) H[l][j] += Hval;
            }
        }
    }
    for (int k = 0; k < N; k++) H[k][k] += R_u;

    // Slew penalty, tridiagonal. Anchored to nmpc_state->u_warm — the last
    // EXTERNALLY gated command (gp_torque_vectoring.c overwrites u_warm with
    // mz_req*os_gate*counter_steer_factor after this call returns), same
    // anchor semantics the old control-hold code used.
    H[0][0] += R_slew;
    grad[0] -= R_slew * nmpc_state->u_warm;
    for (int k = 1; k < N; k++) {
        H[k][k]     += R_slew;
        H[k-1][k-1] += R_slew;
        H[k][k-1]   -= R_slew;
        H[k-1][k]   -= R_slew;
    }

    // 6. RTI shift: warm-start THIS tick's sequence from last tick's
    //    solution, shifted by one stage (standard receding-horizon reuse).
    float u[GP_NMPC_N];
    for (int k = 0; k < N - 1; k++) u[k] = nmpc_state->u_seq[k + 1];
    u[N - 1] = nmpc_state->u_seq[N - 1];

    // 7. Fixed-iteration projected Gauss-Seidel. Exact per-coordinate
    //    minimizer since the cost is quadratic (H is PSD: R_u>0 keeps every
    //    diagonal strictly positive) -- this converges monotonically, no
    //    step-size tuning, no line search, deterministic iteration count.
    for (int iter = 0; iter < GP_NMPC_QP_ITER; iter++) {
        for (int i = 0; i < N; i++) {
            float sum = grad[i];
            for (int j = 0; j < N; j++) {
                if (j != i) sum += H[i][j] * u[j];
            }
            float Hii = (H[i][i] > 1e-6f) ? H[i][i] : 1e-6f;
            float u_i = -sum / Hii;

            u_i = GP_CLAMP(u_i, -mz_max, mz_max);
            float anchor = (i == 0) ? nmpc_state->u_warm : u[i - 1];
            u_i = GP_CLAMP(u_i, anchor - mz_rate_max, anchor + mz_rate_max);
            u[i] = u_i;
        }
    }

    // 8. Commit: apply u[0] now (receding horizon), persist the full
    //    sequence for next tick's shift, forward-simulate the realized
    //    trajectory for telemetry and next tick's frozen gate.
    for (int k = 0; k < N; k++) nmpc_state->u_seq[k] = u[k];

    float x0 = vy0, x1 = r0;
    nmpc_state->x_pred[0][0] = x0; nmpc_state->x_pred[0][1] = x1;
    for (int k = 0; k < N; k++) {
        float nx0 = A_d[0][0]*x0 + A_d[0][1]*x1 + B_d[0]*u[k] + Bdelta_d[0]*delta_sw;
        float nx1 = A_d[1][0]*x0 + A_d[1][1]*x1 + B_d[1]*u[k] + Bdelta_d[1]*delta_sw;
        x0 = nx0; x1 = nx1;
        nmpc_state->x_pred[k+1][0] = x0;
        nmpc_state->x_pred[k+1][1] = x1;
    }

    *mz_cmd = u[0];
    if (isnan(*mz_cmd) || isinf(*mz_cmd)) *mz_cmd = 0.0f;
}