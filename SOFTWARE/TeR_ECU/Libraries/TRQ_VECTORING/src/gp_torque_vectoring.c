#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "gp_torque_vectoring.h"
#include "gp_params.h"
#include "gp_math.h"
#include "gp_nmpc.h"


// ── Embedded ARM Hardware Profiling (Compiles ONLY for STM32 Target) ─
#if defined(__arm__) || defined(__ARM_ARCH)
#include "stm32f4xx.h"

volatile uint32_t g_tv_exec_cycles = 0;  // Measurable via ST-Link / STM32CubeIDE
volatile float g_tv_exec_us = 0.0f;     // Microseconds spent in solver

static inline void dwt_init_if_needed(void) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}
#endif

static const float Kp_map[16] = {
    100.0f, 150.0f, 250.0f, 300.0f,
    120.0f, 180.0f, 280.0f, 350.0f,
    150.0f, 220.0f, 320.0f, 400.0f,
    180.0f, 260.0f, 360.0f, 450.0f
};

static const float Ki_map[16] = {
    10.0f, 15.0f, 20.0f, 25.0f,
    12.0f, 18.0f, 22.0f, 28.0f,
    15.0f, 20.0f, 25.0f, 30.0f,
    18.0f, 25.0f, 30.0f, 35.0f
};

static const float Kd_map[16] = {
    5.0f, 10.0f, 15.0f, 20.0f,
    8.0f, 12.0f, 18.0f, 22.0f,
    10.0f, 15.0f, 20.0f, 25.0f,
    12.0f, 18.0f, 25.0f, 30.0f
};

void gp_tv_init(tv_state_t* state) {
    state->wz_int = 0.0f;
    state->delta_prev = 0.0f;
    for (int i = 0; i < 4; i++) {
        state->t_qp_prev[i] = 0.0f;
        state->t_out_prev[i] = 0.0f;
    }
    gp_tc_init(&state->tc);
    gp_ekf_init(&state->ekf);
    gp_nmpc_init(&state->nmpc);
    state->vy_est = 0.0f;
    state->vy_gps_last = 0.0f;
    state->vy_gps_age_ms = 1000.0f;
    
    // Zero-initialize isolated filters (Fixes cross-scenario leakage)
    state->ax_filt = 0.0f;
    state->ay_filt = 0.0f;
    state->t_ub_rl_filt = 0.0f;
    state->t_ub_rr_filt = 0.0f;
    
    state->t_lb_rl_filt = 0.0f;
    state->t_lb_rr_filt = 0.0f;
    state->delta_notch_x1 = 0.0f;
    state->delta_notch_x2 = 0.0f;
    state->delta_notch_y1 = 0.0f;
    state->delta_notch_y2 = 0.0f;

    float h = GP_W_REG + GP_W_SMOOTH;
    float a_sq = 2.0f / (GP_R_WHEEL * GP_R_WHEEL);
    state->alpha_qp = 1.0f / (h + GP_RHO_AL * a_sq);
    state->lam_prev = 0.0f;
    state->mz_sat_ratio = 1.0f;
}

#include "gp_ekf.h"  // Ensure gp_ekf.h is included at top of file

void gp_tv_step(
    float fx_driver, float delta, float vx, float vy, float wz, 
    float ay, float ax, const float omega[4], float brake_norm, 
    float temp_inv_rl, float temp_inv_rr, float vy_gps, uint8_t gps_valid,
    const gp_regen_limits_t* regen,
    float dt, tv_state_t* state, float t_cmd_out[4]
) {
    gp_regen_limits_t regen_default = {1, 9999.0f, 999999.0f};
    const gp_regen_limits_t* rg = regen ? regen : &regen_default;

#if defined(__arm__) || defined(__ARM_ARCH)
    dwt_init_if_needed();
    uint32_t start_cycles = DWT->CYCCNT;
#endif

    if (vx < 1.0f && brake_norm > 0.5f && fx_driver > 500.0f) {
        t_cmd_out[GP_FL] = 0.0f; t_cmd_out[GP_FR] = 0.0f;
        t_cmd_out[GP_RL] = 15.0f; t_cmd_out[GP_RR] = 15.0f;
        state->wz_int = 0.0f;
        state->tc.pi_integral[GP_RL] = 0.0f; state->tc.pi_integral[GP_RR] = 0.0f;
        state->t_out_prev[GP_RL] = 15.0f; state->t_out_prev[GP_RR] = 15.0f;
        state->t_qp_prev[GP_RL] = 15.0f; state->t_qp_prev[GP_RR] = 15.0f;
        state->t_lb_rl_filt = 0.0f; state->t_lb_rr_filt = 0.0f;

#if defined(__arm__) || defined(__ARM_ARCH)
        uint32_t end_cycles = DWT->CYCCNT;
        g_tv_exec_cycles = end_cycles - start_cycles;
        g_tv_exec_us = ((float)g_tv_exec_cycles / 168000000.0f) * 1000000.0f;
#endif
        return; 
    }

    // --- Deadzones on steering angle and yaw rate ---
    if (fabsf(delta) < GP_STEER_DEADZONE_RAD) { delta = 0.0f; }
    if (fabsf(wz) < GP_YAW_DEADZONE_RADS)     { wz = 0.0f;    }

    // --- 1st-Order Low-Pass Filter on Accelerometer Signals (State-Isolated) ---
    float alpha_lpf = GP_CLAMP(dt / (GP_ACCEL_LPF_TAU + dt), 0.0f, 1.0f);
    state->ax_filt += alpha_lpf * (ax - state->ax_filt);
    state->ay_filt += alpha_lpf * (ay - state->ay_filt);

    // ── Filter delta ONCE, here, before ANY consumer sees it ──────────
    // Two-pole (cascaded) EMA on delta before EKF, TC, and both yaw-moment
    // policy branches ever see it. A single 35ms-tau pole only rolls off at
    // 6 dB/octave -- at 25 Hz (the encoder-noise test frequency) that still
    // passes ~17% of the input amplitude, which is enough to drive a
    // persistent, non-decaying ripple through the EKF's integrated vy state
    // and through kp*wz_err (kp up to ~450). Cascading a second identical
    // pole doubles the rolloff to 12 dB/octave, attenuating 25Hz to ~3% of
    // input while leaving realistic driver steering (1.5-3 Hz chicane/
    // slalom frequencies) attenuated only ~11% -- not perceptible as lag.
    // 2nd-order notch filter tuned to GP_STEER_NOTCH_FREQ_HZ, applied BEFORE
    // any consumer (EKF, TC, both yaw-moment policy branches) sees delta.
    // Direct-form-I difference equation:
    //   y[n] = x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    // with b1=-2cos(w0), b2=1, a1=-2*r*cos(w0), a2=r^2. Coefficients are
    // recomputed from the live dt each tick (cheap: one cosf call) rather
    // than hardcoded for a fixed 200Hz assumption, so this stays correct
    // even if the loop rate ever changes.
    {
        const float w0 = 2.0f * GP_PI * GP_STEER_NOTCH_FREQ_HZ * dt;
        const float cosw0 = cosf(w0);
        const float notch_b1 = -2.0f * cosw0;
        const float notch_b2 = 1.0f;
        const float notch_a1 = -2.0f * GP_STEER_NOTCH_R * cosw0;
        const float notch_a2 = GP_STEER_NOTCH_R * GP_STEER_NOTCH_R;

        float x0 = delta;
        float y0 = x0 + notch_b1 * state->delta_notch_x1 + notch_b2 * state->delta_notch_x2
                       - notch_a1 * state->delta_notch_y1 - notch_a2 * state->delta_notch_y2;

        state->delta_notch_x2 = state->delta_notch_x1;
        state->delta_notch_x1 = x0;
        state->delta_notch_y2 = state->delta_notch_y1;
        state->delta_notch_y1 = y0;

        delta = y0;
    }

    float vx_safe = GP_MAX(fabsf(vx), 0.5f);
    
    // ─────────────────────────────────────────────────────────────────
    // 1. UNIFIED EXTENDED KALMAN FILTER (EKF) STEP
    // ─────────────────────────────────────────────────────────────────
    // A. Time Update (Prediction) -- now receives FILTERED delta
    gp_ekf_predict(&state->ekf, delta, state->ax_filt, state->ay_filt, wz, vx, dt);

    // B. Measurement Updates (Sequential Scalar Fusion)
    gp_ekf_update_gps(&state->ekf, vy_gps, gps_valid);
    gp_ekf_update_kinematic_ss(&state->ekf, state->ay_filt, wz, vx);

    // C. Extract State Estimates (Read-Only Telemetry / Feedback Isolation)
    vy = state->ekf.x[GP_EKF_STATE_VY];
    float wz_corr = state->ekf.wz_corrected;  // Gyro bias compensated yaw rate
    float beta = state->ekf.beta_est;          // Derived sideslip angle [rad]

    // NOTE: We DO NOT overwrite state->tc.mu_surface here. 
    // Traction Control retains full ownership of its own stable EMA filter.

    float fz_est[4];
    float fy_est[4];
    gp_estimate_fz(vx, state->ax_filt, state->ay_filt, fz_est);
    gp_estimate_fy(vx, vy, wz_corr, delta, fz_est, fy_est);
    
    // --- NEW CLEAN BLOCK ---
    // delta is already the filtered signal (see above, filtered before EKF).
    float k_us = gp_adaptive_k_us(fz_est);
    float wz_ref = (vx_safe * delta) / (GP_WB + k_us * vx_safe * vx_safe);

    // Clamp reference yaw rate to physical friction ceiling without declaring mu_avg early
    float wz_max = (0.5f * (state->tc.mu_surface[0] + state->tc.mu_surface[1]) * 9.81f) / vx_safe;
    wz_ref = GP_CLAMP(wz_ref, -wz_max, wz_max);

    float v_norm  = GP_CLAMP(vx_safe / 30.0f, 0.0f, 1.0f);
    float ay_norm = GP_CLAMP(fabsf(state->ay_filt) / 15.0f, 0.0f, 1.0f);
    
    float kp = gp_bilinear_interp_4x4(Kp_map, v_norm, ay_norm);
    float ki = gp_bilinear_interp_4x4(Ki_map, v_norm, ay_norm);
    float kd = gp_bilinear_interp_4x4(Kd_map, v_norm, ay_norm);

    float mu_avg_prev = 0.5f * (state->tc.mu_surface[0] + state->tc.mu_surface[1]);
    float mu_scale = GP_CLAMP(mu_avg_prev / GP_MU_NOM, 0.4f, 1.0f);
    kp *= mu_scale;
    ki *= mu_scale;

    float wz_err = wz_ref - wz_corr;

    // delta is already filtered -- Branch 3's derivative feedforward
    // differentiates the same clean signal the EKF and Branch 4 use.
    float delta_dot = (delta - state->delta_prev) / dt;
    state->delta_prev = delta;

    // ─────────────────────────────────────────────────────────────────
    // 2. DYNAMIC BETA STABILIZATION (CLIPPED VARIANCE SCALING)
    // ─────────────────────────────────────────────────────────────────
    float vy_std_clamped = GP_CLAMP(state->ekf.vy_std, 0.0f, 0.3f);
    float k_beta_dynamic = 4000.0f * (1.0f + 2.0f * vy_std_clamped);
    float beta_term = GP_CLAMP(-k_beta_dynamic * beta, -600.0f, 600.0f);

    float raw_int = state->wz_int + wz_err * dt * state->mz_sat_ratio;
    state->wz_int = GP_TV_WZ_I_MAX * tanhf(raw_int / GP_TV_WZ_I_MAX);

    float os_gate = 1.0f - gp_sigmoid((fabsf(wz_corr) - fabsf(wz_ref) - 0.2f) * 10.0f);
    float counter_steer_factor = 1.0f - gp_sigmoid(-(delta * wz_corr + 0.05f) * 40.0f);

    // ─────────────────────────────────────────────────────────────────
    // FRICTION/POWER BOUNDS — moved up here (was previously computed
    // AFTER the yaw-moment policy selection below). The NMPC branch needs
    // t_ub_friction[] to build a physically-grounded, speed/grip-aware
    // Mz ceiling instead of the old fixed GP_NMPC_MZ_MAX constant, so the
    // bound must exist before the policy #if runs.
    // ─────────────────────────────────────────────────────────────────
    float t_lb[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float t_ub_friction[4];
    float t_ub_power[4];

    float mu_avg = 0.5f * (state->tc.mu_surface[0] + state->tc.mu_surface[1]);

    gp_friction_ellipse_t_ub(fz_est, fy_est, mu_avg, t_ub_friction);
    gp_power_limited_t_ub(omega, t_ub_power);

    // ─────────────────────────────────────────────────────────────────
    // YAW MOMENT POLICY SELECTION (Branch 3 AL-QP vs. Branch 4 NMPC)
    // ─────────────────────────────────────────────────────────────────
    float mz_req = 0.0f;

    float mz_friction_cap = 0.5f * (t_ub_friction[GP_RL] + t_ub_friction[GP_RR])
                                * GP_TRACK_R / GP_R_WHEEL;
    float mz_max_dyn  = GP_MIN(GP_NMPC_MZ_MAX, mz_friction_cap);
    float mz_rate_max = GP_TV_RATE_LIMIT * dt * GP_TRACK_R / (2.0f * GP_R_WHEEL);

    #if defined(GP_TV_USE_NMPC) && (GP_TV_USE_NMPC == 1)
        // delta is already filtered; wz_ref (computed above from filtered
        // delta) is identical to what wz_ref_nmpc would compute, so reuse it
        // directly instead of recomputing.
        float states_nmpc[3] = { vx, vy, wz_corr };
        gp_nmpc_step(states_nmpc, delta, wz_ref, dt, mz_max_dyn, mz_rate_max,
                    mu_scale, &state->nmpc, &mz_req);
        mz_req = GP_CLAMP(mz_req * os_gate * counter_steer_factor,
                        -mz_max_dyn, mz_max_dyn);
        state->nmpc.u_warm = mz_req;
    #else
        // ── Branch 3: Feedback PID + Beta Stabilization ─────────────────
        float ff_mz = kd * delta_dot * (vx_safe / 10.0f);
        float fb_mz = kp * wz_err + ki * state->wz_int + beta_term;
        mz_req = GP_CLAMP((ff_mz + fb_mz) * os_gate * counter_steer_factor, -GP_TV_MAX_MZ, GP_TV_MAX_MZ);
    #endif
    
    // Derating Térmico (applies equally to drive AND regen — same silicon, same heat)
    float temp_limit = 75.0f;
    float derate_rl = 1.0f - gp_sigmoid((temp_inv_rl - temp_limit) * 0.5f);
    float derate_rr = 1.0f - gp_sigmoid((temp_inv_rr - temp_limit) * 0.5f);
    
    t_ub_power[GP_RL] *= derate_rl;
    t_ub_power[GP_RR] *= derate_rr;

    float alpha_ub = GP_CLAMP(dt / (0.010f + dt), 0.0f, 1.0f);
    
    state->t_ub_rl_filt += alpha_ub * (t_ub_friction[GP_RL] - state->t_ub_rl_filt);
    state->t_ub_rr_filt += alpha_ub * (t_ub_friction[GP_RR] - state->t_ub_rr_filt);

    float t_ub[4];
    t_ub[GP_FL] = 0.0f;
    t_ub[GP_FR] = 0.0f;
    t_ub[GP_RL] = GP_MIN(state->t_ub_rl_filt, t_ub_power[GP_RL]);
    t_ub[GP_RR] = GP_MIN(state->t_ub_rr_filt, t_ub_power[GP_RR]);

    // ── Regen (negative-torque) bound: mirror image of the drive-side logic ──
    // Kamm's circle is sign-agnostic, so t_ub_friction (already computed
    // above) doubles as the regen magnitude ceiling. Charge-power/thermal
    // derating is regen's analogue of t_ub_power. Both are EMA-filtered at
    // the SAME time constant as the drive bound so we don't introduce a
    // differently-tuned, chatter-prone edge on the negative side.
    if (rg->enable) {
        float t_lb_power[4];
        gp_power_limited_t_lb(omega, rg->max_charge_power_w, t_lb_power);
        t_lb_power[GP_RL] *= derate_rl;
        t_lb_power[GP_RR] *= derate_rr;

        state->t_lb_rl_filt += alpha_ub * (t_ub_friction[GP_RL] - state->t_lb_rl_filt);
        state->t_lb_rr_filt += alpha_ub * (t_ub_friction[GP_RR] - state->t_lb_rr_filt);

        float lb_mag_rl = GP_MIN(state->t_lb_rl_filt, t_lb_power[GP_RL]);
        float lb_mag_rr = GP_MIN(state->t_lb_rr_filt, t_lb_power[GP_RR]);

        // Enforce the TOTAL regen budget (e.g. accumulator charge-current
        // ceiling) by scaling BOTH wheels proportionally — never by clamping
        // one wheel independently, which would destroy exactly the
        // asymmetric split torque vectoring exists to create.
        //
        // SMOOTH, not hard-branched: this bound feeds DIRECTLY into the QP's
        // box constraint t_lb[], which then warm-starts next tick's solve
        // via t_qp_prev. At the limit, the friction-derived regen capacity
        // sits at/above the budget almost continuously, so a hard
        // mag_sum>budget branch here flips the SOLVER'S FEASIBLE REGION
        // every tick under sensor noise — worse than chattering the final
        // output (post-solve rescale below), because it changes what the
        // solver itself converges to and that error compounds through the
        // warm-start. gp_soft_cap gives the same strict ceiling
        // (capped_sum < max_total_trq always) with no discontinuity.
        float mag_sum = lb_mag_rl + lb_mag_rr;
        if (mag_sum > 1e-3f) {
            float capped_sum = gp_soft_cap(mag_sum, rg->max_total_trq,
                                            1.0f / GP_REGEN_SOFTNESS);
            float scale = capped_sum / mag_sum;
            lb_mag_rl *= scale;
            lb_mag_rr *= scale;
        }

        t_lb[GP_RL] = -lb_mag_rl;
        t_lb[GP_RR] = -lb_mag_rr;
    } else {
        // Regen not currently allowed (steering off-center, cell V/T out of
        // range, accumulator current maxed, etc.). Collapse to zero AND
        // reset the filters so re-engagement ramps in cleanly instead of
        // jumping from a stale value.
        state->t_lb_rl_filt = 0.0f;
        state->t_lb_rr_filt = 0.0f;
        t_lb[GP_RL] = 0.0f;
        t_lb[GP_RR] = 0.0f;
    }

    // Escudo de Fricción (drive side, unchanged)
    float max_sum = t_ub[GP_RL] + t_ub[GP_RR];
    float req_sum = fx_driver * GP_R_WHEEL;
    if (req_sum > max_sum) {
        fx_driver = max_sum / GP_R_WHEEL;
    }
    // Escudo de Fricción (regen side, mirrored): pre-clip fx_driver so the
    // QP's nominal warmstart is itself physically achievable under braking.
    float min_sum = t_lb[GP_RL] + t_lb[GP_RR];
    if (req_sum < min_sum) {
        fx_driver = min_sum / GP_R_WHEEL;
    }

    float t_nominal[4];
    gp_nominal_allocation(fx_driver, mz_req, t_nominal);

    float qp_result[4];
    float qp_residual;

    gp_qp_solve_rwd_closedform(
        t_nominal,
        state->t_qp_prev,
        fx_driver,
        t_lb,
        t_ub,
        qp_result,
        &qp_residual
    );

    float dt_req = t_nominal[GP_RR] - t_nominal[GP_RL];
    float dt_ach = qp_result[GP_RR] - qp_result[GP_RL];
    if (fabsf(dt_req) > 1.0f) {
        state->mz_sat_ratio = GP_CLAMP(dt_ach / dt_req, 0.0f, 1.0f);
    } else {
        state->mz_sat_ratio = 1.0f;
    }
    
    float max_delta_t = GP_TV_RATE_LIMIT * dt;

    for (int i = 0; i < 4; i++) {
        float delta_t = GP_CLAMP(qp_result[i] - state->t_qp_prev[i], -max_delta_t, max_delta_t);
        float tv_final = state->t_qp_prev[i] + delta_t;
        
        state->t_qp_prev[i] = tv_final;
        t_cmd_out[i] = tv_final;
    }

    // Post-solver regen-budget rescale — magnitude-only on the NEGATIVE
    // component, sign-preserving on the positive one.
    //
    // The prior version rescaled by (|T_RL|+|T_RR|), which conflates drive
    // and regen magnitude. At any TV operating point where one wheel drives
    // and the other regens simultaneously (e.g. trailing-throttle mid-corner
    // lift, or TV pushing one wheel negative while the other stays positive
    // to hold Mz), that formulation clamps DRIVE torque to satisfy a budget
    // that is only physically meaningful for the regenerating wheel(s) —
    // the charge-current ceiling has no bearing on how much the inverter can
    // motor. Scoping the rescale to min(T_i, 0) matches the pre-solve
    // t_lb[] derivation above, which never had this defect.
    //
    // SMOOTH, not hard-branched: identical gp_soft_cap() rationale as
    // before — a discrete `if (neg_mag > cap)` flips every tick once neg_mag
    // sits near the boundary under sensor noise. gp_soft_cap() gives a
    // ceiling that is ALWAYS <= max_total_trq but transitions continuously.
    if (rg->enable) {
        float neg_rl = GP_MIN(t_cmd_out[GP_RL], 0.0f);
        float neg_rr = GP_MIN(t_cmd_out[GP_RR], 0.0f);
        float neg_mag = fabsf(neg_rl) + fabsf(neg_rr);

        if (neg_mag > 1e-3f) {
            float capped_mag = gp_soft_cap(neg_mag, rg->max_total_trq,
                                            1.0f / GP_REGEN_SOFTNESS);
            float scale = capped_mag / neg_mag;

            // Only the regenerating (negative) side is touched; any
            // simultaneous drive (positive) torque passes through untouched.
            if (t_cmd_out[GP_RL] < 0.0f) t_cmd_out[GP_RL] *= scale;
            if (t_cmd_out[GP_RR] < 0.0f) t_cmd_out[GP_RR] *= scale;

            state->t_qp_prev[GP_RL] = t_cmd_out[GP_RL];
            state->t_qp_prev[GP_RR] = t_cmd_out[GP_RR];
        }
    }

    // Low-level TC Step
    gp_tc_step(t_cmd_out, omega, vx, vy, wz_corr, fz_est, dt, &state->tc);
    
    // Update EKF friction states using TC's measured surface slip forces
    float t_mean_abs = 0.5f * (fabsf(t_cmd_out[GP_RL]) + fabsf(t_cmd_out[GP_RR]));
    gp_ekf_update_friction(&state->ekf, state->tc.mu_surface[0], state->tc.mu_surface[1], t_mean_abs);

    for (int i = 0; i < 4; i++) {
        state->t_out_prev[i] = t_cmd_out[i];
    }

#if defined(__arm__) || defined(__ARM_ARCH)
    uint32_t end_cycles = DWT->CYCCNT;
    g_tv_exec_cycles = end_cycles - start_cycles;
    g_tv_exec_us = ((float)g_tv_exec_cycles / 168000000.0f) * 1000000.0f; // 168 MHz
#endif
}

size_t gp_tv_state_sizeof(void) {
    return sizeof(tv_state_t);
}