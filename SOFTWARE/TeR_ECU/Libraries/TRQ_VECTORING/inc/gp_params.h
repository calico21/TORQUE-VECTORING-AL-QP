#ifndef GP_PARAMS_H
#define GP_PARAMS_H

/* ============================================================================
 * gp_params.h — SINGLE canonical source for every Bayesian-tuned or hand-tuned
 * scalar gain shared across the solver, traction-control, and torque-vectoring
 * translation units.
 *
 * PRIOR STATE: GP_W_SMOOTH / GP_W_REG / GP_TC_KP were #defined identically in
 * THREE places (gp_solver.h, gp_traction_control.h, gp_torque_vectoring.h).
 * tune_weights.py regex-patches only the first two files. Because C macro
 * redefinition silently takes "last definition wins" within a translation
 * unit (and each .c file only sees whichever copy it #included last), a
 * tuning run can update gp_solver.c and gp_traction_control.c correctly while
 * gp_torque_vectoring.c keeps stale values with ZERO compiler diagnostic —
 * two TUs computing against different physical constants for the same
 * control law. This file removes the hazard structurally: there is now
 * exactly one place these numbers can live, and exactly one place the tuner
 * needs to write.
 * ============================================================================ */

#define GP_W_SMOOTH   5.787f   // Actuator rate penalty weight
#define GP_W_REG      0.405f   // Torque regularization weight
#define GP_RHO_AL     5.0f     // Augmented Lagrangian constraint enforcement
#define GP_QP_ITER    16       // Fixed AL-QP iteration count -> deterministic O(1)

#define GP_TC_KP      25.271f  // Traction control proportional gain
#define GP_TC_KI      12.0f    // Traction control integral gain

#endif // GP_PARAMS_H