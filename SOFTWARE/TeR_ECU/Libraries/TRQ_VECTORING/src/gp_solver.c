/*
 * gp_solver.c
 */

#include "gp_solver.h"
#include <stddef.h>

void gp_nominal_allocation(float fx_driver, float mz_target, float t_nom_out[4]) {
    float arms[4];
    gp_moment_arms(arms);

    // Al ser RWD, la fuerza longitudinal se divide entre 2
    float t_fx = fx_driver * GP_R_WHEEL * 0.5f;

    // Denominador del momento: brazos activos (RL y RR)
    float denom = (arms[GP_RL] * arms[GP_RL]) + (arms[GP_RR] * arms[GP_RR]);
    if (denom < 1e-6f) denom = 1e-6f; // Protección división por cero

    float t_mz_rl = (arms[GP_RL] * mz_target) / denom;
    float t_mz_rr = (arms[GP_RR] * mz_target) / denom;

    t_nom_out[GP_FL] = 0.0f;
    t_nom_out[GP_FR] = 0.0f;
    t_nom_out[GP_RL] = t_fx + t_mz_rl;
    t_nom_out[GP_RR] = t_fx + t_mz_rr;
}

void gp_qp_solve_rwd(
    const float t_warmstart[4],
    const float t_prev[4],
    float fx_driver,
    const float t_lb[4],
    const float t_ub[4],
    float alpha_qp,       // <--- Recibimos alpha
    float* lam_prev_ptr,  // <--- Recibimos el puntero a lam
    float t_out[4],
    float* qp_residual
) {
    // 1. Constantes del AL-QP
    float h = GP_W_REG + GP_W_SMOOTH;
    float a_eq = 1.0f / GP_R_WHEEL;
    float b_eq = fx_driver;

    // ELIMINADO: a_sq y alpha ya no se calculan aquí

    // 2. Pre-cálculo de objetivos para las ruedas traseras
    float t_blend_rl = (GP_W_REG * t_warmstart[GP_RL] + GP_W_SMOOTH * t_prev[GP_RL]) / h;
    float t_blend_rr = (GP_W_REG * t_warmstart[GP_RR] + GP_W_SMOOTH * t_prev[GP_RR]) / h;

    // 3. Inicialización del warm-start
    float t_rl = GP_CLAMP(t_warmstart[GP_RL], t_lb[GP_RL], t_ub[GP_RL]);
    float t_rr = GP_CLAMP(t_warmstart[GP_RR], t_lb[GP_RR], t_ub[GP_RR]);
    
    // AÑADIDO: Cargamos el multiplicador dual del ciclo anterior
    float lam = *lam_prev_ptr;

    // 4. Bucle principal de optimización (16 iteraciones)
    for (int i = 0; i < GP_QP_ITER; i++) {
        // Violación de la restricción geométrica
        float viol = a_eq * (t_rl + t_rr) - b_eq;

        // Gradientes para RL y RR
        float g_rl = h * (t_rl - t_blend_rl) + a_eq * (lam + GP_RHO_AL * viol);
        float g_rr = h * (t_rr - t_blend_rr) + a_eq * (lam + GP_RHO_AL * viol);

        // Descenso de gradiente proyectado usando alpha_qp
        t_rl = GP_CLAMP(t_rl - alpha_qp * g_rl, t_lb[GP_RL], t_ub[GP_RL]);
        t_rr = GP_CLAMP(t_rr - alpha_qp * g_rr, t_lb[GP_RR], t_ub[GP_RR]);

        // Actualización del multiplicador de Lagrange
        lam = lam + GP_RHO_AL * (a_eq * (t_rl + t_rr) - b_eq);
    }

    // AÑADIDO: Guardamos el multiplicador para el siguiente ciclo de RTOS
    *lam_prev_ptr = lam;

    // 5. Empaquetar la salida
    t_out[GP_FL] = 0.0f;
    t_out[GP_FR] = 0.0f;
    t_out[GP_RL] = t_rl;
    t_out[GP_RR] = t_rr;

    // Guardar el residuo final para telemetría/diagnóstico
    if (qp_residual != NULL) {
        *qp_residual = fabsf(a_eq * (t_rl + t_rr) - b_eq);
    }
}