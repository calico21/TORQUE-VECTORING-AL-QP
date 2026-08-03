#ifndef GP_PARAMS_H
#define GP_PARAMS_H

/* ============================================================================
 * gp_params.h — SINGLE canonical source for every Bayesian-tuned or hand-tuned
 * scalar gain shared across the solver, traction-control, and torque-vectoring
 * translation units.
 *
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

#define GP_W_SMOOTH   5.999f   // Actuator rate penalty weight
#define GP_W_REG    0.401f
#define GP_RHO_AL     5.0f     // Augmented Lagrangian constraint enforcement
#define GP_QP_ITER    16       // Fixed AL-QP iteration count -> deterministic O(1)

#define GP_TC_KP      23.113f   // Traction control proportional gain
#define GP_TC_KI      12.0f    // Traction control integral gain

// ── Regen active-set transition softness ───────────────────────────
// Shared by both the pre-solve t_lb[] derivation and the post-solve budget
// rescale in gp_torque_vectoring.c. Previously two independently-declared
// 4.0f locals with different names (GP_REGEN_BOUND_SOFTNESS /
// GP_REGEN_BUDGET_SOFTNESS) — same hazard gp_params.h exists to eliminate,
// one scope down. Widening this trades chatter-smoothness for how tightly
// the regen bound is respected near the boundary; narrowing it recovers a
// harder edge at the cost of reintroducing noise-sensitivity.
#define GP_REGEN_SOFTNESS   4.0f   // [Nm] transition width, both regen soft-caps

// ── Regen / Torque-Vectoring-under-braking ──────────────────────────
// Nominal accumulator pack voltage used ONLY to convert the configurable
// regen current ceiling (TeR.config.regen_max_current, in Amps) into an
// electrical charge-power ceiling for shaping the per-wheel regen bound.
// TODO(team): replace with a live AMS pack-voltage decode once the exact
// accessor is confirmed in ams.dbc. A fixed nominal is a safe, conservative
// placeholder as long as it's not set above the pack's real minimum voltage.
#define GP_NOMINAL_PACK_VOLTAGE_V   400.0f

// ── Branch 4: NMPC Tuning Constants ──────────────────────────────────
#define GP_NMPC_Q_YAW       9716.4f     // Yaw rate error penalty
#define GP_NMPC_R_EFFORT       0.0351f    // Control effort penalty
#define GP_NMPC_R_SLEW        0.193f     // Rate-of-change penalty
#define GP_NMPC_MZ_MAX      200.0f      // Maximum yaw moment request [Nm]
#define GP_NMPC_SOFTNESS      10.0f     // [Nm] soft-cap transition width
#define GP_NMPC_BETA_MAX        0.14f   // [rad] ~8 deg soft envelope threshold
#define GP_NMPC_BETA_SHARPNESS  30.0f   // Sigmoid gate steepness
#define GP_NMPC_Q_BETA          4410.7f     // Soft barrier penalty weight

#define GP_EKF_MODEL_BLEND      0.3f    // 0.0 = pure kinematic, 1.0 = pure bicycle model
#endif // GP_PARAMS_H