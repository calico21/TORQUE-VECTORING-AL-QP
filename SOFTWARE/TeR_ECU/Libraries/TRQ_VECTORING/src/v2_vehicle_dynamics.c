#include "v2_vehicle_dynamics.h"
#include "v2_lut_data.h"

void v2_init_params(v2_params_t *params) {
    if (!params) return;

    /* Parámetros EKF (vy, bz) - Sintonización de Alta Dinámica */
    params->q_vy                  = 1.50f;    /* Proceso vy: Permite rápida adaptación del estado */
    params->q_bz                  = 0.005f;   /* Proceso bz: Offset de gyro lento */
    params->r_pseudo_vy           = 0.80f;    /* Menor peso a la pseudo-medida cinemática */
    params->r_gps_vy              = 0.02f;    /* Alta confianza en la trama GPS cuando está activa */

    /* Parámetros SMC (Sliding Mode Control) */
    params->lambda_beta           = 180.0f;   /* Fuerte penalización a la deriva beta */
    params->lambda_int            = 15.0f;    /* Eliminación de error en régimen permanente */
    params->k_smc                 = 180.0f;   /* Ganancia de superficie [Nm] */
    params->phi_boundary_base     = 1.20f;    /* Capa límite suave para evitar chattering */
    params->k_ff                  = 32.0f;
    params->k_ffd                 = 4.0f;
    params->max_yaw_moment_nm     = 260.0f;
    params->smc_integral_limit    = 50.0f;
    params->peak_mu               = 1.35f;

    /* Traction Control & Límites */
    params->max_allowable_slip    = 0.12f;
    params->slip_cut_gain         = 12.0f;
    params->max_slew_nm_per_s     = 3252.3f;
    params->steer_deadzone_rad    = 0.0087f;  /* ~0.5 deg */
    params->yaw_deadzone_rads     = 0.0175f;  /* ~1.0 deg/s */
}

void v2_reset_state(v2_state_t *state) {
    if (!state) return;
    state->x_vy                 = 0.0f;
    state->x_bz                 = 0.0f;
    state->P[0][0]              = 0.1f; state->P[0][1] = 0.0f;
    state->P[1][0]              = 0.0f; state->P[1][1] = 0.01f;
    state->beta_est_rad         = 0.0f;

    state->smc_surface_integral = 0.0f;
    state->trq_prev_rl_nm       = 0.0f;
    state->trq_prev_rr_nm       = 0.0f;
    state->steer_prev_rad       = 0.0f;
    state->steer_dot_filt_rads  = 0.0f;
    state->cut_active_rl        = false;
    state->cut_active_rr        = false;
    state->initialized          = true;
}

size_t v2_state_sizeof(void) { return sizeof(v2_state_t); }

static float v2_slew_limit(float target, float prev, float max_delta) {
    float delta = V2_CLAMP(target - prev, -max_delta, max_delta);
    return prev + delta;
}

static void v2_ekf_step(
    float ay_ms2, float wz_meas_rads, float vx_ms,
    float vy_gps, uint8_t gps_valid, float dt,
    const v2_params_t *params, v2_state_t *state
) {
    float vx_safe = V2_MAX(vx_ms, V2_V_FLOOR_MS);

    /* 1. Predicción de Estado */
    float wz_corrected = wz_meas_rads - state->x_bz;
    float vy_dot = ay_ms2 - (vx_safe * wz_corrected);
    state->x_vy += vy_dot * dt;

    /* 2. Predicción de Covarianza */
    float p00 = state->P[0][0];
    float p01 = state->P[0][1];
    float p10 = state->P[1][0];
    float p11 = state->P[1][1];

    float v_dt = vx_safe * dt;
    state->P[0][0] = p00 + v_dt * (p10 + p01) + (v_dt * v_dt * p11) + (params->q_vy * dt);
    state->P[0][1] = p01 + v_dt * p11;
    state->P[1][0] = p10 + v_dt * p11;
    state->P[1][1] = p11 + (params->q_bz * dt);

    /* 3. Actualización Pseudo-Cinemática */
    float vy_ss = (V2_VEHICLE_LR * wz_corrected) - 
                  ((V2_VEHICLE_MASS * ay_ms2 * V2_VEHICLE_LF * vx_safe) / 
                   (V2_VEHICLE_WB * V2_C_ALPHA_REAR));

    float innov_pseudo = vy_ss - state->x_vy;
    float S_pseudo = state->P[0][0] + params->r_pseudo_vy;

    if (S_pseudo > 0.0001f) {
        float K0 = state->P[0][0] / S_pseudo;
        float K1 = state->P[1][0] / S_pseudo;

        state->x_vy += K0 * innov_pseudo;
        state->x_bz += K1 * innov_pseudo;

        float p00_old = state->P[0][0];
        float p01_old = state->P[0][1];

        state->P[0][0] -= K0 * p00_old;
        state->P[0][1] -= K0 * p01_old;
        state->P[1][0] -= K1 * p00_old;
        state->P[1][1] -= K1 * p01_old;
    }

    /* 4. Actualización GPS */
    if (gps_valid) {
        float innov_gps = vy_gps - state->x_vy;
        float S_gps = state->P[0][0] + params->r_gps_vy;

        if (S_gps > 0.0001f) {
            float K0 = state->P[0][0] / S_gps;
            float K1 = state->P[1][0] / S_gps;

            state->x_vy += K0 * innov_gps;
            state->x_bz += K1 * innov_gps;

            float p00_old = state->P[0][0];
            float p01_old = state->P[0][1];

            state->P[0][0] -= K0 * p00_old;
            state->P[0][1] -= K0 * p01_old;
            state->P[1][0] -= K1 * p00_old;
            state->P[1][1] -= K1 * p01_old;
        }
    }

    state->x_bz = V2_CLAMP(state->x_bz, -0.15f, 0.15f);
    state->beta_est_rad = atan2f(state->x_vy, vx_safe);
}

v2_trq_map_t v2_tv_step(
    float apps_pct, float brake_pressure_bar, float steer_rad,
    float wz_measured_rads, float ay_ms2, float vx,
    float wheel_rpm_rl, float wheel_rpm_rr,
    float vy_gps, uint8_t gps_valid,
    float torque_limit_nm, uint8_t regen_enabled, float regen_max_trq_nm,
    float dt, const v2_params_t *params, v2_state_t *state
) {
    if (!state->initialized) v2_reset_state(state);

    v2_ekf_step(ay_ms2, wz_measured_rads, vx, vy_gps, gps_valid, dt, params, state);

    float steer_dot_raw = (dt > 0.0001f) ? ((steer_rad - state->steer_prev_rad) / dt) : 0.0f;
    state->steer_prev_rad = steer_rad;
    float alpha_steer = V2_CLAMP(dt / (0.020f + dt), 0.0f, 1.0f);
    state->steer_dot_filt_rads += alpha_steer * (steer_dot_raw - state->steer_dot_filt_rads);

    if (fabsf(steer_rad) < params->steer_deadzone_rad) {
        steer_rad = 0.0f;
        state->steer_dot_filt_rads = 0.0f;
    }

    float wz_corrected = wz_measured_rads - state->x_bz;
    if (fabsf(wz_corrected) < params->yaw_deadzone_rads) wz_corrected = 0.0f;

    float vx_safe = V2_MAX(vx, V2_V_FLOOR_MS);

    float base_trq;
    if (brake_pressure_bar > 5.0f) {
        float brake_pct = brake_pressure_bar / 100.0f;
        base_trq = regen_enabled ? -(brake_pct * regen_max_trq_nm * 0.5f) : 0.0f;
    } else {
        float rear_rpm_wheel = 0.5f * (wheel_rpm_rl + wheel_rpm_rr);
        float rear_rpm_motor = rear_rpm_wheel * V2_GEAR_R;
        float lut_trq = v2_interp2d_drive_torque(apps_pct, rear_rpm_motor);
        base_trq = V2_CLAMP(lut_trq, 0.0f, torque_limit_nm * 0.5f);
    }

    float fz_static_rear = 0.5f * V2_VEHICLE_MASS * V2_GRAVITY_MS2 * (V2_VEHICLE_LF / V2_VEHICLE_WB);
    float dfz_lat = (V2_VEHICLE_MASS * ay_ms2 * V2_VEHICLE_H_CG) / V2_VEHICLE_TW;
    float fz_rl = V2_MAX(50.0f, fz_static_rear - dfz_lat);
    float fz_rr = V2_MAX(50.0f, fz_static_rear + dfz_lat);
    float fz_total = fz_rl + fz_rr;

    float base_trq_rl = (base_trq > 0.0f) ? (base_trq * (2.0f * fz_rl / fz_total)) : base_trq;
    float base_trq_rr = (base_trq > 0.0f) ? (base_trq * (2.0f * fz_rr / fz_total)) : base_trq;

    float denom = V2_VEHICLE_WB + (V2_VEHICLE_MASS * (V2_VEHICLE_LR * V2_C_ALPHA_REAR - V2_VEHICLE_LF * V2_C_ALPHA_FRONT) / 
                   (2.0f * V2_C_ALPHA_FRONT * V2_C_ALPHA_REAR * V2_VEHICLE_WB)) * (vx_safe * vx_safe);
    float r_ref = (denom > 0.001f) ? ((vx_safe / denom) * steer_rad) : 0.0f;

    float r_max = (params->peak_mu * V2_GRAVITY_MS2) / vx_safe;
    r_ref = V2_CLAMP(r_ref, -r_max, r_max);

    float yaw_error = wz_corrected - r_ref;

    state->smc_surface_integral += yaw_error * dt;
    state->smc_surface_integral = V2_CLAMP(state->smc_surface_integral, 
                                            -params->smc_integral_limit, 
                                             params->smc_integral_limit);

    float S = yaw_error + (params->lambda_beta * state->beta_est_rad) + (params->lambda_int * state->smc_surface_integral);

    float phi = params->phi_boundary_base * (1.0f + 0.03f * vx_safe);
    float sat_S = tanhf(S / phi);

    float m_ff = (steer_rad * params->k_ff) + (state->steer_dot_filt_rads * params->k_ffd);
    float m_z = -(params->k_smc * sat_S) + m_ff;
    m_z = V2_CLAMP(m_z, -params->max_yaw_moment_nm, params->max_yaw_moment_nm);

    float delta_trq = (m_z * (float)WHEEL_RADIUS) / V2_VEHICLE_TW;

    v2_trq_map_t out;
    out.rl_nm = base_trq_rl - delta_trq;
    out.rr_nm = base_trq_rr + delta_trq;

    float max_delta = params->max_slew_nm_per_s * dt;
    out.rl_nm = v2_slew_limit(out.rl_nm, state->trq_prev_rl_nm, max_delta);
    out.rr_nm = v2_slew_limit(out.rr_nm, state->trq_prev_rr_nm, max_delta);
    state->trq_prev_rl_nm = out.rl_nm;
    state->trq_prev_rr_nm = out.rr_nm;

    return out;
}

v2_trq_map_t v2_traction_control_step(
    v2_trq_map_t in, float wheel_rpm_rl, float wheel_rpm_rr, float vx,
    const v2_params_t *params, v2_state_t *state
) {
    vx = V2_MAX(vx, V2_V_FLOOR_MS);

    float v_rl = (wheel_rpm_rl * (2.0f * (float)PI / 60.0f)) * (float)WHEEL_RADIUS;
    float v_rr = (wheel_rpm_rr * (2.0f * (float)PI / 60.0f)) * (float)WHEEL_RADIUS;
    float slip_rl = (v_rl - vx) / vx;
    float slip_rr = (v_rr - vx) / vx;

    if (slip_rl > params->max_allowable_slip) state->cut_active_rl = true;
    else if (slip_rl < (params->max_allowable_slip - 0.03f)) state->cut_active_rl = false;
    if (slip_rr > params->max_allowable_slip) state->cut_active_rr = true;
    else if (slip_rr < (params->max_allowable_slip - 0.03f)) state->cut_active_rr = false;

    if (slip_rl > params->max_allowable_slip && in.rl_nm > 0.0f) {
        float excess = slip_rl - params->max_allowable_slip;
        in.rl_nm *= 1.0f / (1.0f + params->slip_cut_gain * excess);
    }
    if (slip_rr > params->max_allowable_slip && in.rr_nm > 0.0f) {
        float excess = slip_rr - params->max_allowable_slip;
        in.rr_nm *= 1.0f / (1.0f + params->slip_cut_gain * excess);
    }

    return in;
}