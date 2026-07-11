/*
 * gp_traction_control.c
 */

#include "gp_traction_control.h"

void gp_tc_init(tc_state_t* state) {
    for (int i = 0; i < 4; i++) {
        state->pi_integral[i] = 0.0f;
        state->kappa_filt[i] = 0.0f;
        state->omega_prev[i] = 0.0f;
    }
    state->mu_surface = GP_TC_MU_NOM;
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
    float fx_rl = t_req_out[GP_RL] / GP_R_WHEEL;
    float fx_rr = t_req_out[GP_RR] / GP_R_WHEEL;
    
    float mu_rl = fx_rl / GP_MAX(fz[GP_RL], 100.0f);
    float mu_rr = fx_rr / GP_MAX(fz[GP_RR], 100.0f);
    float mu_meas = GP_CLAMP((mu_rl + mu_rr) * 0.5f, GP_TC_MU_LO, GP_TC_MU_HI);
    
    float gate_mu = gp_sigmoid(t_mean_abs - 20.0f);
    float alpha_gated = GP_TC_ALPHA_MU_EMA * gate_mu;
    state->mu_surface = GP_CLAMP((1.0f - alpha_gated) * state->mu_surface + alpha_gated * mu_meas, GP_TC_MU_LO, GP_TC_MU_HI);

    float cs_factor = gp_tc_combined_slip_factor(vx, vy, wz);
    float speed_gate = gp_sigmoid((vx - GP_TC_V_MIN) * 3.0f);
    
    // Multiplicamos limpiamente las ganancias base en lugar de hackear la salida del PI
    float kp_eff = (GP_TC_KP * 20.0f) * (1.0f + GP_TC_V_KP_SCALE / (fabsf(vx) + 0.5f));
    float ki_eff = (GP_TC_KI * 20.0f);

    int rear_wheels[2] = {GP_RL, GP_RR};
    
    for (int w = 0; w < 2; w++) {
        int i = rear_wheels[w];
        
        float kappa_star = gp_tc_kappa_star(fz[i], state->mu_surface) * cs_factor;
        float kappa_raw = gp_tc_compute_kappa(omega[i], vx);
        
        // FIX 1: Filtro de ruido mecánico. Ignoramos todo transitorio por encima de ~3.5 Hz
        float alpha_lp = 0.90f; // 90% inercia, 10% señal nueva
        state->kappa_filt[i] = alpha_lp * state->kappa_filt[i] + (1.0f - alpha_lp) * kappa_raw;
        
        float error = state->kappa_filt[i] - kappa_star; 
        
        float raw_integral = state->pi_integral[i] + error * dt * speed_gate;
        state->pi_integral[i] = GP_TC_I_MAX * tanhf(raw_integral / GP_TC_I_MAX); 
        
        float pi_out = kp_eff * error + ki_eff * state->pi_integral[i];
        
        // FIX 2: Filtro derivativo. Filtramos omega ANTES de restarla para la derivada.
        float omega_ema = 0.1f * omega[i] + 0.9f * state->omega_prev[i];
        float omega_dot = (omega_ema - state->omega_prev[i]) / dt;
        state->omega_prev[i] = omega_ema; // Guardamos la velocidad ya suavizada
        
        // Feedforward derivativo anticipativo (Umbral de 250 y verificando que haya error real)
        /* Gate suave: vale ~1 si hay error positivo (patinaje), ~0 si no hay error */
        float error_gate = gp_sigmoid(error * 50.0f); 

        /* Derivada continua: solo crece a partir de 250 rad/s^2, pero sin esquinas matemáticas.
        Multiplicamos por 20.0f para escalar de vuelta el factor 0.05f interno */
        float deriv_kick = 2.0f * (20.0f * gp_softplus((omega_dot - 250.0f) * 0.05f));

        /* Se aplica el recorte de par predictivo de forma puramente continua */
        pi_out -= deriv_kick * error_gate; // O += si vuestra lógica de recorte suma torque negativo
        
        float reduction = speed_gate * gp_softplus(pi_out * GP_TC_CLAMP_BETA) / GP_TC_CLAMP_BETA;
        float t_cmd = t_req_out[i] - reduction;
        
        t_req_out[i] = gp_softplus(t_cmd * GP_TC_CLAMP_BETA) / GP_TC_CLAMP_BETA;
    }
}