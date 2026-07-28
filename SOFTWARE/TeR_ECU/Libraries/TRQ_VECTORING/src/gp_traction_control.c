#include "gp_traction_control.h"

void gp_tc_init(tc_state_t* state) {
    for (int i = 0; i < 4; i++) {
        state->pi_integral[i] = 0.0f;
        state->kappa_filt[i] = 0.0f;
        state->omega_last_raw[i] = 0.0f;
        state->omega_prev_ema[i] = 0.0f;
        state->rls_P[i] = 1000.0f;
        state->rls_theta[i] = 30000.0f;
        state->theta_prev[i] = 30000.0f;
        state->kappa_prev[i] = 0.0f;
        state->fx_prev[i] = 0.0f;
        state->kappa_opt[i] = 0.12f;
    }
    state->mu_surface[0] = GP_TC_MU_NOM;
    state->mu_surface[1] = GP_TC_MU_NOM;
}

static float gp_tc_kappa_star(float fz, float mu_rt) {
    float b_eff = GP_MAX(GP_TC_B0 + GP_TC_B1 * fz, 2.0f);
    float kappa_nom = tanf(GP_PI / (2.0f * GP_TC_C_PAC)) / b_eff;
    float mu_correction = 1.0f + 0.15f * (1.5f - GP_CLAMP(mu_rt, 0.5f, 2.0f)) / 1.5f;
    return GP_CLAMP(kappa_nom * mu_correction, 0.04f, 0.22f);
}

static float gp_tc_combined_slip_factor(float vx, float vy, float wz) {
    float vx_safe = fabsf(vx) + 0.5f;
    float alpha_f = fabsf((vy + wz * GP_LF) / vx_safe);
    float alpha_r = fabsf((vy - wz * GP_LR) / vx_safe);
    float alpha_avg = 0.5f * (alpha_f + alpha_r);
    float ratio = GP_CLAMP(alpha_avg / GP_TC_ALPHA_PEAK, 0.0f, 0.95f);
    return 1.0f / sqrtf(1.0f + ratio * ratio);
}

static float gp_tc_compute_kappa(float omega, float vx) {
    float denom = GP_TC_V_MIN + gp_softplus(vx - GP_TC_V_MIN);
    return GP_CLAMP((omega * GP_R_WHEEL - vx) / denom, -0.80f, 0.80f);
}

void gp_tc_step(
    float t_req_out[4], 
    const float omega[4], 
    float vx, float vy, float wz, 
    const float fz[4], 
    float dt, 
    tc_state_t* state
) {
    float t_mean_abs = (fabsf(t_req_out[GP_RL]) + fabsf(t_req_out[GP_RR])) * 0.5f;
    
    float omega_ema_rl = 0.1f * omega[GP_RL] + 0.9f * state->omega_prev_ema[GP_RL];
    float raw_omega_dot_rl = (omega_ema_rl - state->omega_prev_ema[GP_RL]) / dt;

    float omega_ema_rr = 0.1f * omega[GP_RR] + 0.9f * state->omega_prev_ema[GP_RR];
    float raw_omega_dot_rr = (omega_ema_rr - state->omega_prev_ema[GP_RR]) / dt;

    float omega_dot_rl = GP_CLAMP(raw_omega_dot_rl, -5000.0f, 5000.0f);
    float omega_dot_rr = GP_CLAMP(raw_omega_dot_rr, -5000.0f, 5000.0f);

    float fx_rl = (t_req_out[GP_RL] - GP_I_WHEEL_EST * omega_dot_rl) / GP_R_WHEEL;
    float fx_rr = (t_req_out[GP_RR] - GP_I_WHEEL_EST * omega_dot_rr) / GP_R_WHEEL;
    
    float fx_wheels[4] = {0.0f};
    fx_wheels[GP_RL] = fx_rl;
    fx_wheels[GP_RR] = fx_rr;
    
    // Magnitude-based: mu is a grip-utilization ratio, not directionally
    // signed. Signed fx/fz clamps every regen (negative fx) sample to
    // GP_TC_MU_LO regardless of how much tire capacity was actually used,
    // biasing the surface estimate on every braking event. fabsf makes the
    // estimator equally informed by drive and regen.
    float mu_rl = fabsf(fx_rl) / GP_MAX(fz[GP_RL], 100.0f);
    float mu_rr = fabsf(fx_rr) / GP_MAX(fz[GP_RR], 100.0f);
    float mu_meas[2];

    mu_meas[0] = GP_CLAMP(mu_rl, GP_TC_MU_LO, GP_TC_MU_HI);
    mu_meas[1] = GP_CLAMP(mu_rr, GP_TC_MU_LO, GP_TC_MU_HI);

    float gate_mu = gp_sigmoid(t_mean_abs - 20.0f);
    float alpha_gated = GP_TC_ALPHA_MU_EMA * gate_mu;

    for (int w = 0; w < 2; w++) {
        state->mu_surface[w] = GP_CLAMP(
            (1.0f - alpha_gated) * state->mu_surface[w] +
            alpha_gated * mu_meas[w],
            GP_TC_MU_LO,
            GP_TC_MU_HI
        );
    }
    
    float cs_factor = gp_tc_combined_slip_factor(vx, vy, wz);
    float speed_gate = gp_sigmoid((vx - GP_TC_V_MIN) * 3.0f);
    
    float kp_eff = (GP_TC_KP * 20.0f) * (1.0f + GP_TC_V_KP_SCALE / (fabsf(vx) + 0.5f));
    float ki_eff = (GP_TC_KI * 20.0f);

    int rear_wheels[2] = {GP_RL, GP_RR};
    
    for (int w = 0; w < 2; w++) {
        int i = rear_wheels[w];

        // Capture the TV/regen command's sign BEFORE this function mutates
        // t_req_out[i]. Everything below operates in magnitude space
        // projected onto this sign axis, so ONE control law defends against
        // wheelspin (sign_i > 0, identical to original behavior) and against
        // regen lock-up (sign_i < 0, new) without duplicating any logic.
        float sign_i = (t_req_out[i] < 0.0f) ? -1.0f : 1.0f;
        float mag_in = fabsf(t_req_out[i]);
        
        float kappa_raw = gp_tc_compute_kappa(omega[i], vx);
        
        float alpha_lp = 0.90f; 
        state->kappa_filt[i] = alpha_lp * state->kappa_filt[i] + (1.0f - alpha_lp) * kappa_raw;
        
        float prev_k = state->kappa_prev[i];
        
        float d_kappa = state->kappa_filt[i] - prev_k; 
        float d_fx = fx_wheels[i] - state->fx_prev[i];
        
        state->kappa_prev[i] = state->kappa_filt[i];
        state->fx_prev[i] = fx_wheels[i];
        
        if (fabsf(d_kappa) > 0.0001f && vx > 2.0f) {
            float lambda = 0.985f; 
            float P = state->rls_P[i];
            float phi = d_kappa;
            float y = d_fx;
            
            float denom = lambda + P * phi * phi;
            float K = (P * phi) / denom; 
            
            float theta_hat = state->rls_theta[i];
            float error_rls = y - phi * theta_hat;
            
            state->rls_theta[i] = GP_CLAMP(theta_hat + K * error_rls, -50000.0f, 150000.0f);
            state->rls_P[i] = GP_CLAMP((P - K * phi * P) / lambda, 10.0f, 10000.0f);
        }
        
        float dtheta = state->rls_theta[i] - state->theta_prev[i];
        
        float kappa_secant = (prev_k * state->rls_theta[i] - 
                              state->kappa_filt[i] * state->theta_prev[i]) / 
                             (dtheta + copysignf(10.0f, dtheta));
                             
        float secant_ok = gp_sigmoid(fabsf(dtheta) / 500.0f - 2.0f);
        
        float lr = 0.000005f; 
        float kappa_grad = state->kappa_opt[i] + lr * state->rls_theta[i] * dt;

        state->kappa_opt[i] = secant_ok * GP_CLAMP(kappa_secant, 0.05f, 0.22f) + 
                              (1.0f - secant_ok) * GP_CLAMP(kappa_grad, 0.05f, 0.22f);
                              
        state->theta_prev[i] = state->rls_theta[i];
        
        // kappa_analytical / kappa_opt are magnitudes (>=0 by construction —
        // see gp_tc_kappa_star's clamp and the [0.05,0.22] clamps above).
        // Project BOTH the target and the measured slip onto the commanded
        // direction (sign_i): this is what gives TC a lock-up guard under
        // regen that's structurally identical to the wheelspin guard under
        // drive, instead of a special-cased second control law.
        float kappa_analytical = gp_tc_kappa_star(fz[i], state->mu_surface[w]) * cs_factor;
        float kappa_target_mag = 0.5f * kappa_analytical + 0.5f * state->kappa_opt[i];
        float kappa_meas_mag = sign_i * state->kappa_filt[i];

        float error = kappa_meas_mag - kappa_target_mag; 
        
        float raw_integral = state->pi_integral[i] + error * dt * speed_gate;
        state->pi_integral[i] = GP_TC_I_MAX * tanhf(raw_integral / GP_TC_I_MAX); 
        
        float pi_out = kp_eff * error + ki_eff * state->pi_integral[i];
        
        float omega_ema = 0.1f * omega[i] + 0.9f * state->omega_prev_ema[i];
        float omega_dot = (omega_ema - state->omega_prev_ema[i]) / dt;

        state->omega_prev_ema[i] = omega_ema;
        state->omega_last_raw[i] = omega[i];
        
        float error_gate = gp_sigmoid(error * 50.0f); 
        // Curb-strike / lock-up derivative kick, mirrored: the original term
        // only caught a sudden POSITIVE omega spike (wheel launched off a
        // curb -> wheelspin risk). Under regen the dangerous spike is a
        // sudden NEGATIVE omega_dot (wheel snapping toward lock). sign_i
        // selects which physical direction is being commanded this tick, so
        // only the relevant branch can ever fire for a given wheel.
        float deriv_kick_pos = 20.0f * gp_softplus((omega_dot - 250.0f) * 0.05f);
        float deriv_kick_neg = 20.0f * gp_softplus((-omega_dot - 250.0f) * 0.05f);
        float deriv_kick = 2.0f * ((sign_i > 0.0f) ? deriv_kick_pos : deriv_kick_neg);

        pi_out -= deriv_kick * error_gate; 
        
        float reduction = speed_gate * gp_softplus(pi_out * GP_TC_CLAMP_BETA) / GP_TC_CLAMP_BETA;

        // Magnitude-space actuation gate: TC may only ever shrink the
        // MAGNITUDE of the commanded torque toward zero — never invert its
        // sign, never amplify it — for EITHER drive or regen. The original
        // single-sided gp_softplus(t_cmd*BETA)/BETA implicitly assumed
        // t_req_out[i] >= 0 and collapsed ANY negative (regen) command to
        // ~0. Operating in |magnitude| and reapplying sign_i at the end
        // reproduces the original behavior bit-for-bit when sign_i > 0 and
        // correctly extends it to regen when sign_i < 0.
        float mag_cmd = mag_in - reduction;
        float mag_out = gp_softplus(mag_cmd * GP_TC_CLAMP_BETA) / GP_TC_CLAMP_BETA;

        // Traction control may only maintain or reduce torque magnitude, never amplify it
        mag_out = GP_MIN(mag_out, mag_in);

        t_req_out[i] = sign_i * mag_out;
    }
}