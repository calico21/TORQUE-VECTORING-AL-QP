#include "v1_vehicle_dynamics.h"
#include "v1_lut_data.h"

void v1_init_params(v1_params_t *params) {
    if (!params) return;
    params->kp_yaw                  = 85.0f;
    params->ki_yaw                  = 12.0f;
    params->k_ff                    = 32.0f;
    params->k_ffd                   = 4.5f;     /* 4.5 Nm / (rad/s) de impulso en turn-in */
    params->steer_dot_lpf_tau       = 0.020f;   /* Tau LPF de 20 ms */
    params->max_yaw_moment_nm       = 260.0f;
    params->pi_windup_limit_nm      = 70.0f;
    params->peak_mu                 = 1.35f;
    params->max_allowable_slip      = 0.12f;
    params->slip_cut_gain           = 12.0f;

    params->steer_deadzone_rad      = 0.0087f;  /* ~0.5 deg */
    params->yaw_deadzone_rads       = 0.0175f;  /* ~1.0 deg/s */
    params->max_slew_nm_per_s       = 3252.3f;
    params->speed_gain_taper        = 0.015f;   /* Taper suave con velocidad */
    params->enable_fz_load_transfer = true;
}

void v1_reset_state(v1_state_t *state) {
    if (!state) return;
    state->error_integral_nm  = 0.0f;
    state->trq_prev_rl_nm     = 0.0f;
    state->trq_prev_rr_nm     = 0.0f;
    state->steer_prev_rad     = 0.0f;
    state->steer_dot_filt_rads= 0.0f;
    state->cut_active_rl      = false;
    state->cut_active_rr      = false;
    state->initialized        = true;
}

size_t v1_state_sizeof(void) { return sizeof(v1_state_t); }

static float v1_slew_limit(float target, float prev, float max_delta) {
    float delta = V1_CLAMP(target - prev, -max_delta, max_delta);
    return prev + delta;
}

v1_trq_map_t v1_tv_step(
    float apps_pct, float brake_pressure_bar, float steer_rad,
    float wz_measured_rads, float ay_ms2, float vx,
    float wheel_rpm_rl, float wheel_rpm_rr,
    float torque_limit_nm, uint8_t regen_enabled, float regen_max_trq_nm,
    float dt, const v1_params_t *params, v1_state_t *state
) {
    if (!state->initialized) v1_reset_state(state);

    /* 1. Derivación de velocidad de volante d(steer)/dt y filtrado LPF */
    float steer_dot_raw = (dt > 0.0001f) ? ((steer_rad - state->steer_prev_rad) / dt) : 0.0f;
    state->steer_prev_rad = steer_rad;

    float alpha_steer = V1_CLAMP(dt / (params->steer_dot_lpf_tau + dt), 0.0f, 1.0f);
    state->steer_dot_filt_rads += alpha_steer * (steer_dot_raw - state->steer_dot_filt_rads);

    /* Zonas muertas de entrada */
    if (fabsf(steer_rad) < params->steer_deadzone_rad) {
        steer_rad = 0.0f;
        state->steer_dot_filt_rads = 0.0f;
    }
    if (fabsf(wz_measured_rads) < params->yaw_deadzone_rads) wz_measured_rads = 0.0f;

    vx = V1_MAX(vx, V1_V_FLOOR_MS);

    /* 2. Demanda de par longitudinal base */
    float base_trq;
    if (brake_pressure_bar > 5.0f) {
        float brake_pct = brake_pressure_bar / 100.0f;
        base_trq = regen_enabled ? -(brake_pct * regen_max_trq_nm * 0.5f) : 0.0f;
    } else {
        float rear_rpm_wheel = 0.5f * (wheel_rpm_rl + wheel_rpm_rr);
        float rear_rpm_motor = rear_rpm_wheel * V1_GEAR_R;
        float lut_trq = v1_interp2d_drive_torque(apps_pct, rear_rpm_motor);
        base_trq = V1_CLAMP(lut_trq, 0.0f, torque_limit_nm * 0.5f);
    }

    /* 3. Transferencia de carga lateral cuasi-estática (Fz Bias) */
    float base_trq_rl = base_trq;
    float base_trq_rr = base_trq;

    if (params->enable_fz_load_transfer && base_trq > 0.0f) {
        float fz_static_rear = 0.5f * V1_VEHICLE_MASS * V1_GRAVITY_MS2 * V1_VEHICLE_REAR_RATIO;
        float dfz_lat = (V1_VEHICLE_MASS * ay_ms2 * V1_VEHICLE_H_CG) / V1_VEHICLE_TW;

        /* En curva a izquierdas (ay > 0), la carga se desplaza a la rueda exterior (RR) */
        float fz_rl = V1_MAX(50.0f, fz_static_rear - dfz_lat);
        float fz_rr = V1_MAX(50.0f, fz_static_rear + dfz_lat);
        float fz_total = fz_rl + fz_rr;

        /* Reparto proporcional conservando la suma total: T_RL + T_RR = 2 * T_base */
        base_trq_rl = base_trq * (2.0f * fz_rl / fz_total);
        base_trq_rr = base_trq * (2.0f * fz_rr / fz_total);
    }

    /* 4. Speed Gain Scheduling según vx */
    float speed_scale = V1_CLAMP(1.2f - params->speed_gain_taper * vx, 0.6f, 1.2f);
    float kp_eff  = params->kp_yaw * speed_scale;
    float kff_eff = params->k_ff * speed_scale;

    /* 5. Modelo bicicleta lineal saturado por adherencia física */
    float denom = V1_VEHICLE_WB + V1_K_UNDERSTEER * (vx * vx);
    float r_ref = (denom > 0.001f) ? ((vx / denom) * steer_rad) : 0.0f;

    float r_max = (params->peak_mu * V1_GRAVITY_MS2) / vx;
    r_ref = V1_CLAMP(r_ref, -r_max, r_max);

    float yaw_error = r_ref - wz_measured_rads;

    /* Feedforward compuesto: Término de posición + término derivativo Turn-in */
    float m_ff = (steer_rad * kff_eff) + (state->steer_dot_filt_rads * params->k_ffd);
    float m_p  = yaw_error * kp_eff;

    float total_unsat = m_ff + m_p + state->error_integral_nm;
    bool is_saturated = (fabsf(total_unsat) >= params->max_yaw_moment_nm);
    bool same_sign    = ((total_unsat * yaw_error) > 0.0f);

    if (!is_saturated || !same_sign) {
        state->error_integral_nm += params->ki_yaw * yaw_error * dt;
        state->error_integral_nm = V1_CLAMP(state->error_integral_nm,
                                             -params->pi_windup_limit_nm,
                                              params->pi_windup_limit_nm);
    }

    float m_z = V1_CLAMP(m_ff + m_p + state->error_integral_nm,
                          -params->max_yaw_moment_nm, params->max_yaw_moment_nm);
    float delta_trq = (m_z * (float)WHEEL_RADIUS) / V1_VEHICLE_TW;

    v1_trq_map_t out;
    out.rl_nm = base_trq_rl - delta_trq;
    out.rr_nm = base_trq_rr + delta_trq;

    /* 6. Limitador de velocidad de cambio de par (Slew-Rate Limiter) */
    float max_delta = params->max_slew_nm_per_s * dt;
    out.rl_nm = v1_slew_limit(out.rl_nm, state->trq_prev_rl_nm, max_delta);
    out.rr_nm = v1_slew_limit(out.rr_nm, state->trq_prev_rr_nm, max_delta);
    state->trq_prev_rl_nm = out.rl_nm;
    state->trq_prev_rr_nm = out.rr_nm;

    return out;
}

v1_trq_map_t v1_traction_control_step(
    v1_trq_map_t in, float wheel_rpm_rl, float wheel_rpm_rr, float vx,
    const v1_params_t *params, v1_state_t *state
) {
    vx = V1_MAX(vx, V1_V_FLOOR_MS);

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