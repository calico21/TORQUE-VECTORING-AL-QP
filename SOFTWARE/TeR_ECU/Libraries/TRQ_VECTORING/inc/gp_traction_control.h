/*
 * gp_traction_control.h
 * Control de Tracción Adaptativo (Intermediate Mode)
 */

#ifndef GP_TRACTION_CONTROL_H
#define GP_TRACTION_CONTROL_H

#include "gp_vehicle_model.h"

// ── Parámetros del Control de Tracción (Hoosier R20) ────────
#define GP_TC_B0             14.0f     // Rigidez Pacejka BCD sin carga
#define GP_TC_B1             -0.0018f  // Sensibilidad a la carga de B
#define GP_TC_C_PAC          1.65f     // Factor de forma C
#define GP_TC_ALPHA_PEAK     0.20f     // [rad] Peak lateral slip angle (~11.5 deg)
#define GP_TC_KP             120.0f    // Ganancia Proporcional base
#define GP_TC_KI             35.0f     // Ganancia Integral
#define GP_TC_I_MAX          3.5f      // Saturación Anti-windup
#define GP_TC_V_KP_SCALE     3.0f      // [m/s] Boost de Kp a baja velocidad
#define GP_TC_ALPHA_KAPPA_LP 0.60f     // Filtro Pasa-bajos de medición de Slip
#define GP_TC_ALPHA_MU_EMA   0.008f    // Filtro EMA de superficie mu
#define GP_TC_MU_LO          0.40f     // Límite inferior de estimación mu
#define GP_TC_MU_HI          2.00f     // Límite superior de estimación mu
#define GP_TC_MU_NOM         1.50f     // Asunción nominal de mu
#define GP_TC_V_MIN          1.5f      // [m/s] Puerta de activación TC
#define GP_TC_CLAMP_BETA     50.0f     // Sharpness del softplus
#define GP_I_WHEEL_EST       1.20f

// ── Estructura de Estado (Memoria entre ciclos) ─────────────
typedef struct {
    float pi_integral[4]; 
    float kappa_filt[4];  
    float mu_surface[2];     
    float omega_last_raw[4];
    float omega_prev_ema[4];
    
    // --- NUEVO: ESTADOS DEL OBSERVADOR RLS ---
    float rls_P[4];        // Covarianza del error (Incertidumbre)
    float rls_theta[4];    // Pendiente estimada (dFx / dKappa)
    float kappa_prev[4];   // Memoria de slip para la derivada
    float fx_prev[4];      // Memoria de fuerza para la derivada
    float kappa_opt[4];    // Target de slip dinámico (Gradient Ascent)
} tc_state_t;

// ── Prototipos de funciones ─────────────────────────────────

// Inicializa el estado del TC con valores por defecto
void gp_tc_init(tc_state_t* state);

// Ejecuta un paso del Traction Control (modifica t_req_out en el sitio)
void gp_tc_step(
    float t_req_out[4],       // In/Out: Torques demandados por el Torque Vectoring
    const float omega[4],     // Velocidad angular de las ruedas [rad/s]
    float vx,                 // Velocidad longitudinal [m/s]
    float vy,                 // Velocidad lateral [m/s]
    float wz,                 // Yaw rate [rad/s]
    const float fz[4],        // Carga vertical por rueda [N]
    float dt,                 // Delta time [s] (0.005 para 200Hz)
    tc_state_t* state         // Puntero al estado del TC
);

#endif // GP_TRACTION_CONTROL_H