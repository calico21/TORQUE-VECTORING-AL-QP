/*
 * gp_torque_vectoring.h
 * Controlador Principal de Torque Vectoring 
 */

#ifndef GP_TORQUE_VECTORING_H
#define GP_TORQUE_VECTORING_H

#include "gp_vehicle_model.h"
#include "gp_solver.h"
#include "gp_traction_control.h"

// ── Parámetros del Controlador TV ───────────────────────────
#define GP_TV_MAX_MZ         1500.0f  // [Nm] Momento de guiñada máximo permitido
#define GP_TV_WZ_I_MAX       200.0f   // [Nm] Límite del Anti-windup integral
#define GP_TV_RATE_LIMIT     5000.0f  // [Nm/s] Límite de rampa de torque para cuidar palieres
#define GP_TV_EMA_ALPHA      0.2f     // Filtro Pasa-bajos salida final de torque

// ── Estructura de Estado Principal ──────────────────────────
typedef struct {
    float wz_int;         // Integral del error de Yaw Rate
    float delta_prev;     // Ángulo de volante previo (para derivada/Feedforward)
    float t_qp_prev[4];   // Última solución del solver (Warm-start)
    float t_out_prev[4];  // Último torque aplicado (tras rate limit)
    tc_state_t tc;        // Estado interno del Control de Tracción
    float alpha_qp;  
    float lam_prev;
} tv_state_t;

// ── Prototipos de funciones ─────────────────────────────────

// Inicializa las memorias y estados
void gp_tv_init(tv_state_t* state);

// Bucle principal de control (se llama a 200Hz / dt = 0.005s)
void gp_tv_step(
    float fx_driver,      // Torque global demandado por el piloto (traducido a Fuerza X [N])
    float delta,          // Ángulo de volante en las ruedas [rad]
    float vx,             // Velocidad longitudinal [m/s]
    float vy,             // Velocidad lateral [m/s]
    float wz,             // Yaw rate medido [rad/s]
    float ay,             // Aceleración lateral [m/s2]
    float ax,             // Aceleración longitudinal [m/s2]
    const float omega[4], // Velocidad de las ruedas [rad/s]
    float brake_norm,     // Pedal de freno normalizado [0-1]
    float dt,             // Delta time [s]
    tv_state_t* state,    // Memoria del controlador
    float t_cmd_out[4]    // SALIDA: Torque a mandar a los inversores [Nm]
);

#endif // GP_TORQUE_VECTORING_H