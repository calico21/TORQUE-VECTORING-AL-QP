#include <stdio.h>
#include <stdint.h>
#include "gp_torque_vectoring.h"

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
    state->vy_est = 0.0f;
    state->vy_gps_last = 0.0f;
    state->vy_gps_age_ms = 1000.0f; // Stale initial age
    
    float h = GP_W_REG + GP_W_SMOOTH;
    float a_sq = 2.0f / (GP_R_WHEEL * GP_R_WHEEL);
    state->alpha_qp = 1.0f / (h + GP_RHO_AL * a_sq);
    state->lam_prev = 0.0f;
    state->mz_sat_ratio = 1.0f; // Full authority initial assumption
}

void gp_tv_step(
    float fx_driver, float delta, float vx, float vy, float wz, 
    float ay, float ax, const float omega[4], float brake_norm, 
    float temp_inv_rl, float temp_inv_rr, float vy_gps, uint8_t gps_valid,
    float dt, tv_state_t* state, float t_cmd_out[4]
) {
    if (vx < 1.0f && brake_norm > 0.5f && fx_driver > 500.0f) {
        t_cmd_out[GP_FL] = 0.0f; t_cmd_out[GP_FR] = 0.0f;
        t_cmd_out[GP_RL] = 15.0f; t_cmd_out[GP_RR] = 15.0f;
        state->wz_int = 0.0f;
        state->tc.pi_integral[GP_RL] = 0.0f; state->tc.pi_integral[GP_RR] = 0.0f;
        state->t_out_prev[GP_RL] = 15.0f; state->t_out_prev[GP_RR] = 15.0f;
        state->t_qp_prev[GP_RL] = 15.0f; state->t_qp_prev[GP_RR] = 15.0f;
        return; 
    }

    // --- FIX #4: Deadzones on steering angle and yaw rate ---
    if (fabsf(delta) < GP_STEER_DEADZONE_RAD) { delta = 0.0f; }
    if (fabsf(wz) < GP_YAW_DEADZONE_RADS)     { wz = 0.0f;    }

    // --- FIX #6: 1st-Order Low-Pass Filter on Accelerometer Signals ---
    static float ax_filt = 0.0f;
    static float ay_filt = 0.0f;
    float alpha_lpf = GP_CLAMP(dt / (GP_ACCEL_LPF_TAU + dt), 0.0f, 1.0f);
    ax_filt += alpha_lpf * (ax - ax_filt);
    ay_filt += alpha_lpf * (ay - ay_filt);

    float vx_safe = GP_MAX(fabsf(vx), 0.5f);
    
    // --- FIX #2: Fading-memory side-slip observer with active GPS fusion ---
    float vy_dot = ay_filt - (vx_safe * wz);
    float vy_ss = (GP_LR * wz) - ((GP_MASS * ay_filt * GP_LF * vx_safe) / (GP_WB * GP_C_ALPHA_R));
    float k_corr = 2.0f;
    float vy_corr = vy_dot - k_corr * (state->vy_est - vy_ss);

    if (gps_valid) {
        float k_gps = 1.5f; // GPS correction weight
        vy_corr -= k_gps * (state->vy_est - vy_gps);
        state->vy_gps_last = vy_gps;
        state->vy_gps_age_ms = 0.0f;
    } else {
        state->vy_gps_age_ms += dt * 1000.0f;
    }

    state->vy_est += vy_corr * dt;
    vy = state->vy_est;

    // Sideslip real del chasis, no solo yaw rate — necesario para DYC de 2 estados
    float beta = atan2f(vy, vx_safe);

    float fz_est[4];
    float fy_est[4];
    // Use filtered accelerations to prevent chatter in friction bounds
    gp_estimate_fz(vx, ax_filt, ay_filt, fz_est);
    gp_estimate_fy(vx, vy, wz, delta, fz_est, fy_est);
    
    float k_us = gp_adaptive_k_us(fz_est);
    float wz_ref = (vx_safe * delta) / (GP_WB + k_us * vx_safe * vx_safe);
    
    float v_norm  = GP_CLAMP(vx_safe / 30.0f, 0.0f, 1.0f);
    float ay_norm = GP_CLAMP(fabsf(ay_filt) / 15.0f, 0.0f, 1.0f);
    
    float kp = gp_bilinear_interp_4x4(Kp_map, v_norm, ay_norm);
    float ki = gp_bilinear_interp_4x4(Ki_map, v_norm, ay_norm);
    float kd = gp_bilinear_interp_4x4(Kd_map, v_norm, ay_norm);

    // Gain scheduling por fricción: si el suelo da menos grip, pedimos menos Mz
    float mu_avg_prev = 0.5f * (state->tc.mu_surface[0] + state->tc.mu_surface[1]);
    float mu_scale = GP_CLAMP(mu_avg_prev / GP_MU_NOM, 0.4f, 1.0f);
    kp *= mu_scale;
    ki *= mu_scale;

    float wz_err = wz_ref - wz;
    float delta_dot = (delta - state->delta_prev) / dt;
    state->delta_prev = delta;

    // Realimentación de sideslip: término estabilizador proporcional a beta
    const float k_beta = 4000.0f;
    float beta_term = -k_beta * beta;

    float raw_int = state->wz_int + wz_err * dt * state->mz_sat_ratio;
    state->wz_int = GP_TV_WZ_I_MAX * tanhf(raw_int / GP_TV_WZ_I_MAX);

    float os_gate = 1.0f - gp_sigmoid((fabsf(wz) - fabsf(wz_ref) - 0.2f) * 10.0f);
    float counter_steer_factor = 1.0f - gp_sigmoid(-(delta * wz + 0.05f) * 40.0f);

    float ff_mz = kd * delta_dot * (vx_safe / 10.0f);
    float fb_mz = kp * wz_err + ki * state->wz_int + beta_term;
    float mz_req = GP_CLAMP((ff_mz + fb_mz) * os_gate * counter_steer_factor, -GP_TV_MAX_MZ, GP_TV_MAX_MZ);
    
    // 7. Límites de los Neumáticos y LÍMITES TÉRMICOS (Derating)
    float t_lb[4] = {0.0f, 0.0f, 0.0f, 0.0f}; 
    float t_ub_friction[4];
    float t_ub_power[4];
    
    float mu_avg = 0.5f * (state->tc.mu_surface[0] + state->tc.mu_surface[1]);

    gp_friction_ellipse_t_ub(fz_est, fy_est, mu_avg, t_ub_friction);
    gp_power_limited_t_ub(omega, t_ub_power);
    
    // --- DERATING TÉRMICO (Soft-Cut) ---
    float temp_limit = 75.0f;
    float derate_rl = 1.0f - gp_sigmoid((temp_inv_rl - temp_limit) * 0.5f);
    float derate_rr = 1.0f - gp_sigmoid((temp_inv_rr - temp_limit) * 0.5f);
    
    t_ub_power[GP_RL] *= derate_rl;
    t_ub_power[GP_RR] *= derate_rr;

    // --- FIX: 10ms LPF pre-filtering on friction bounds to stop active-set flapping ---
    static float t_ub_rl_filt = 0.0f;
    static float t_ub_rr_filt = 0.0f;
    float alpha_ub = GP_CLAMP(dt / (0.010f + dt), 0.0f, 1.0f);
    
    t_ub_rl_filt += alpha_ub * (t_ub_friction[GP_RL] - t_ub_rl_filt);
    t_ub_rr_filt += alpha_ub * (t_ub_friction[GP_RR] - t_ub_rr_filt);

    float t_ub[4];
    t_ub[GP_FL] = 0.0f;
    t_ub[GP_FR] = 0.0f;
    t_ub[GP_RL] = GP_MIN(t_ub_rl_filt, t_ub_power[GP_RL]);
    t_ub[GP_RR] = GP_MIN(t_ub_rr_filt, t_ub_power[GP_RR]);

    // --- ESCUDO DE FRICCIÓN (Friction Budgeting) ---
    float max_sum = t_ub[GP_RL] + t_ub[GP_RR];
    float req_sum = fx_driver * GP_R_WHEEL;
    if (req_sum > max_sum) {
        fx_driver = max_sum / GP_R_WHEEL;
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

    // Anti-windup por back-calculation
    float dt_req = t_nominal[GP_RR] - t_nominal[GP_RL];
    float dt_ach = qp_result[GP_RR] - qp_result[GP_RL];
    if (fabsf(dt_req) > 1.0f) {
        state->mz_sat_ratio = GP_CLAMP(dt_ach / dt_req, 0.0f, 1.0f);
    } else {
        state->mz_sat_ratio = 1.0f;
    }
    
    float max_delta_t = GP_TV_RATE_LIMIT * dt;
    float current_max_slew = 0.0f;

    for (int i = 0; i < 4; i++) {
        float delta_t = GP_CLAMP(qp_result[i] - state->t_qp_prev[i], -max_delta_t, max_delta_t);
        
        // Track true physical slew rate (Nm/s)
        float slew_val = fabsf(delta_t) / dt;
        if (slew_val > current_max_slew) {
            current_max_slew = slew_val;
        }

        float tv_final = state->t_qp_prev[i] + delta_t;
        
        state->t_qp_prev[i] = tv_final;
        t_cmd_out[i] = tv_final;
    }
    state->max_slew_logged = current_max_slew;
    for (int i = 0; i < 4; i++) {
        float delta_t = GP_CLAMP(qp_result[i] - state->t_qp_prev[i], -max_delta_t, max_delta_t);
        float tv_final = state->t_qp_prev[i] + delta_t;
        
        state->t_qp_prev[i] = tv_final;
        t_cmd_out[i] = tv_final;
    }

    gp_tc_step(t_cmd_out, omega, vx, vy, wz, fz_est, dt, &state->tc);
    
    for (int i = 0; i < 4; i++) {
        state->t_out_prev[i] = t_cmd_out[i];
    }
}