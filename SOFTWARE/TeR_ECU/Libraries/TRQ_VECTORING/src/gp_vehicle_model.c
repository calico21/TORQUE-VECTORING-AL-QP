/*
 * gp_vehicle_model.c
 */

#include "gp_vehicle_model.h"

void gp_estimate_fz(float vx, float ax, float ay, float fz_out[4]) {
    float fz_static = 0.25f * GP_MASS * GP_GRAVITY;
    
    float dFz_lon = GP_MASS * ax * GP_H_CG / (GP_WB + 1e-3f);
    float dFz_lat = GP_MASS * ay * GP_H_CG / (GP_TRACK_F + GP_TRACK_R + 1e-3f);
    
    // Carga aerodinámica (Downforce proporcional a v^2)
    float downforce_rear = 0.5f * GP_AIR_DENSITY * (vx * vx) * GP_AERO_CL_REAR * GP_AERO_AREA;
    
    float fz_raw[4];
    fz_raw[GP_FL] = fz_static - dFz_lon + dFz_lat;
    fz_raw[GP_FR] = fz_static - dFz_lon - dFz_lat;
    // Solo sumamos el downforce al eje trasero (para dar ventaja al Torque Vectoring)
    fz_raw[GP_RL] = fz_static + dFz_lon + dFz_lat + (downforce_rear * 0.5f);
    fz_raw[GP_RR] = fz_static + dFz_lon - dFz_lat + (downforce_rear * 0.5f);
    
    for (int i = 0; i < 4; i++) {
        fz_out[i] = 50.0f + gp_softplus(fz_raw[i] - 50.0f);
    }
}

void gp_estimate_fy(float vx, float vy, float wz, float delta, const float fz[4], float fy_out[4]) {
    float vx_safe = fabsf(vx) + 0.5f;
    
    // Ángulos de deslizamiento cinemáticos (Slip angles)
    float alpha_f = delta - atan2f(vy + wz * GP_LF, vx_safe);
    float alpha_r =       - atan2f(vy - wz * GP_LR, vx_safe);
    
    float fz_f_total = fz[GP_FL] + fz[GP_FR] + 1e-3f;
    float fz_r_total = fz[GP_RL] + fz[GP_RR] + 1e-3f;
    
    float fy_f_lin = GP_C_ALPHA_F * alpha_f;
    float fy_r_lin = GP_C_ALPHA_R * alpha_r;
    
    float fy_f_max = GP_MU_NOM * fz_f_total;
    float fy_r_max = GP_MU_NOM * fz_r_total;
    
    // Saturación Tanh (el STM32F4 FPU soporta tanhf nativamente)
    float fy_f_axle = fy_f_max * tanhf(fy_f_lin / (fy_f_max + 1e-3f));
    float fy_r_axle = fy_r_max * tanhf(fy_r_lin / (fy_r_max + 1e-3f));
    
    // Reparto por porcentaje de carga vertical (Split Fz)
    fy_out[GP_FL] = fy_f_axle * fz[GP_FL] / fz_f_total;
    fy_out[GP_FR] = fy_f_axle * fz[GP_FR] / fz_f_total;
    fy_out[GP_RL] = fy_r_axle * fz[GP_RL] / fz_r_total;
    fy_out[GP_RR] = fy_r_axle * fz[GP_RR] / fz_r_total;
}

void gp_friction_ellipse_t_ub(const float fz[4], const float fy_est[4], float mu_est, float t_ub_out[4]) {
    float mu_safe = GP_CLAMP(mu_est, 0.4f, 2.0f);
    
    for (int i = 0; i < 4; i++) {
        float max_fy_capacity = mu_safe * fz[i];
        // Fx_sq_max = (mu*Fz)^2 - Fy^2
        float fx_sq_max = (max_fy_capacity * max_fy_capacity) - (fy_est[i] * fy_est[i]);
        
        // sqrt(softplus(x * 4) / 4)
        float fx_max = gp_softplus_sqrt(fx_sq_max * 4.0f) * 0.5f; 
        
        t_ub_out[i] = fx_max * GP_R_WHEEL;
    }
}

void gp_power_limited_t_ub(const float omega_wheel[4], float t_ub_out[4]) {
    for (int i = 0; i < 4; i++) {
        float omega_safe = gp_softplus(omega_wheel[i] * GP_R_WHEEL);
        float t_power = GP_P_MAX_WHL / (omega_safe + 1e-3f);
        t_ub_out[i] = GP_CLAMP(t_power, 0.0f, 2000.0f);
    }
}

float gp_adaptive_k_us(const float fz[4]) {
    float fz_front_mean = 0.5f * (fz[GP_FL] + fz[GP_FR]);
    float fz_rear_mean  = 0.5f * (fz[GP_RL] + fz[GP_RR]);
    float delta_fz = fz_rear_mean - fz_front_mean;
    return GP_K_US * (1.0f + GP_K_US_FZ * delta_fz);
}

void gp_moment_arms(float arms_out[4]) {
    float hw_f = 0.5f * GP_TRACK_F / GP_R_WHEEL;
    float hw_r = 0.5f * GP_TRACK_R / GP_R_WHEEL;
    
    arms_out[GP_FL] = -hw_f;
    arms_out[GP_FR] =  hw_f;
    arms_out[GP_RL] = -hw_r;
    arms_out[GP_RR] =  hw_r;
}