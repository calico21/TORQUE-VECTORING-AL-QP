/*
 * gp_solver.h
 * Augmented-Lagrangian QP Solver optimizado para configuración RWD
 */

#ifndef GP_SOLVER_H
#define GP_SOLVER_H

#include "gp_vehicle_model.h"

// ── Parámetros del Solver AL-QP ─────────────────────────────
#define GP_QP_ITER    16     // Iteraciones fijas del solver
#define GP_W_REG      1.0f   // Peso de tracking cuadrático
#define GP_W_SMOOTH   0.3f   // Peso de suavidad (evita vibraciones en palieres)
#define GP_RHO_AL     10.0f  // Penalización del Augmented-Lagrangian

// ── Prototipos de funciones ─────────────────────────────────

// Asignación nominal (Punto de partida analítico)
void gp_nominal_allocation(float fx_driver, float mz_target, float t_nom_out[4]);

// Solver AL-QP desenrollado para tracción trasera
void gp_qp_solve_rwd(
    const float t_warmstart[4],
    const float t_prev[4],
    float fx_driver,
    const float t_lb[4],
    const float t_ub[4],
    float alpha_qp,       // <--- NUEVO: Constante pre-calculada
    float* lam_prev_ptr,  // <--- NUEVO: Puntero al multiplicador dual
    float t_out[4],
    float* qp_residual
);

#endif // GP_SOLVER_H