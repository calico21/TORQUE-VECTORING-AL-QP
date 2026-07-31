#include "gp_solver.h"
#include <stddef.h>

void gp_nominal_allocation(float fx_driver, float mz_target, float t_nom_out[4]) {
    float arms[4];
    gp_moment_arms(arms);

    float t_fx = fx_driver * GP_R_WHEEL * 0.5f;
    float denom = (arms[GP_RL] * arms[GP_RL]) + (arms[GP_RR] * arms[GP_RR]);
    if (denom < 1e-6f) denom = 1e-6f;

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
    float alpha_qp,       
    float* lam_prev_ptr,  
    float t_out[4],
    float* qp_residual
) {
    float h = GP_W_REG + GP_W_SMOOTH;
    float a_eq = 1.0f / GP_R_WHEEL;
    float b_eq = fx_driver;

    float t_blend_rl = (GP_W_REG * t_warmstart[GP_RL] + GP_W_SMOOTH * t_prev[GP_RL]) / h;
    float t_blend_rr = (GP_W_REG * t_warmstart[GP_RR] + GP_W_SMOOTH * t_prev[GP_RR]) / h;

    float t_rl = GP_CLAMP(t_warmstart[GP_RL], t_lb[GP_RL], t_ub[GP_RL]);
    float t_rr = GP_CLAMP(t_warmstart[GP_RR], t_lb[GP_RR], t_ub[GP_RR]);
    
    float lam = *lam_prev_ptr;

    for (int i = 0; i < GP_QP_ITER; i++) {
        float viol = a_eq * (t_rl + t_rr) - b_eq;
        float g_rl = h * (t_rl - t_blend_rl) + a_eq * (lam + GP_RHO_AL * viol);
        float g_rr = h * (t_rr - t_blend_rr) + a_eq * (lam + GP_RHO_AL * viol);

        t_rl = GP_CLAMP(t_rl - alpha_qp * g_rl, t_lb[GP_RL], t_ub[GP_RL]);
        t_rr = GP_CLAMP(t_rr - alpha_qp * g_rr, t_lb[GP_RR], t_ub[GP_RR]);

        lam = lam + GP_RHO_AL * (a_eq * (t_rl + t_rr) - b_eq);
    }

    *lam_prev_ptr = GP_CLAMP(lam, -5000.0f, 5000.0f);

    t_out[GP_FL] = 0.0f;
    t_out[GP_FR] = 0.0f;
    t_out[GP_RL] = t_rl;
    t_out[GP_RR] = t_rr;

    if (qp_residual != NULL) {
        *qp_residual = fabsf(a_eq * (t_rl + t_rr) - b_eq);
    }
}

/*
 * gp_qp_solve_rwd_closedform — SMOOTH ACTIVE-SET RELAXATION
 * ============================================================================
 * PRIOR BEHAVIOR (removed): a hard `SAT_HYSTERESIS = 3.0f` Nm Schmitt-trigger
 * band decided, via if/else, whether the two-wheel equality-constrained
 * solution was "interior" or "one wheel saturated". At torque magnitudes of
 * 150-450 Nm (nominal operating range), a 3 Nm band is under 1% of signal
 * amplitude. Because t_ub[] itself moves every 5ms tick (it's a function of
 * live mu/fz estimates, not a fixed constant), any operating point sitting
 * near the friction-ellipse boundary, an infeasible Mz demand, or a
 * brake-release transient crosses that 3 Nm band under ordinary estimator
 * noise -- and EACH crossing is a full branch flip to a structurally
 * different closed-form expression, not a small perturbation of the same
 * one. That branch-flip is exactly what showed up as the 25-90 Hz "WARN
 * Chattering" cluster in the sanity suite (tests B, D, F, G).
 *
 * FIX: replace the discrete flag with a continuous activation weight
 * sat_rl, sat_rr in [0,1] computed from the SIGNED margin to each wheel's own
 * box (same gp_sigmoid() gating idiom already used elsewhere in this codebase
 * for os_gate / gate_mu / derate_rl / derate_rr). Four bounded, always-safe
 * candidate solutions (interior / RL-saturated / RR-saturated / both-saturated
 * independent-clamp) are blended via the exact product-of-complements
 * partition of unity:
 *
 *     w_free + w_A + w_B + w_both == 1   (identically, for any sat_rl, sat_rr)
 *
 * Properties:
 *   - Recovers the ORIGINAL discrete solution bit-for-bit far from any bound
 *     (sat_rl, sat_rr -> 0 or 1 as sigmoid saturates).
 *   - Zero hysteresis / zero memory -> no Schmitt-trigger flapping possible.
 *   - Output is a strict convex combination of four already-bounded
 *     candidates -> can never leave the box constraints, no new failure mode
 *     introduced.
 *   - Cost: 2 extra gp_sigmoid() calls + ~12 FMAs per tick. Negligible next
 *     to the 16-iteration AL-QP this function exists to avoid running.
 * ============================================================================
 */
void gp_qp_solve_rwd_closedform(
    const float t_warmstart[4], const float t_prev[4], float fx_driver,
    const float t_lb[4], const float t_ub[4], float t_out[4], float* qp_residual
) {
    const float h    = GP_W_REG + GP_W_SMOOTH;
    const float a_eq = 1.0f / GP_R_WHEEL;
    const float b_eq = fx_driver;

    // Width of the smooth active-set transition [Nm]. Replaces SAT_HYSTERESIS.
    // Tune alongside GP_TC gains if the friction-ellipse boundary region needs
    // to be wider/narrower; unlike the old hysteresis this has no memory, so
    // widening it costs smoothness margin, not stability margin.
    const float GP_SAT_SOFTNESS = 3.0f; // Nm

    const float t_bl_rl = (GP_W_REG * t_warmstart[GP_RL] + GP_W_SMOOTH * t_prev[GP_RL]) / h;
    const float t_bl_rr = (GP_W_REG * t_warmstart[GP_RR] + GP_W_SMOOTH * t_prev[GP_RR]) / h;

    const float lb_rl = t_lb[GP_RL], ub_rl = t_ub[GP_RL];
    const float lb_rr = t_lb[GP_RR], ub_rr = t_ub[GP_RR];

    // --- Exact equality-constrained (box-unconstrained) stationarity solution.
    // a_eq*(t_rl_free + t_rr_free) == b_eq identically -- this is the KKT
    // condition for the equality constraint alone, verified algebraically:
    // substituting lam below into a_eq*(t_rl+t_rr) telescopes to exactly b_eq.
    const float lam = h * (a_eq * (t_bl_rl + t_bl_rr) - b_eq) / (2.0f * a_eq * a_eq);
    const float t_rl_free = t_bl_rl - lam * a_eq / h;
    const float t_rr_free = t_bl_rr - lam * a_eq / h;

    // --- Smooth activation weights: 0 deep inside the box, 1 deep outside,
    // 0.5 exactly at the boundary. margin > 0 <=> inside by `margin` Nm.
    const float margin_rl = fminf(t_rl_free - lb_rl, ub_rl - t_rl_free);
    const float margin_rr = fminf(t_rr_free - lb_rr, ub_rr - t_rr_free);
    const float sat_rl = gp_sigmoid(-margin_rl / GP_SAT_SOFTNESS);
    const float sat_rr = gp_sigmoid(-margin_rr / GP_SAT_SOFTNESS);

    // --- Candidate A: RL saturates to its nearest bound, RR resolves the
    // equality constraint alone (clamped into its own box as a safety net for
    // the rare doubly-infeasible case -- no separate discrete branch needed).
    const float t_rl_A = GP_CLAMP(t_rl_free, lb_rl, ub_rl);
    const float t_rr_A = GP_CLAMP((b_eq / a_eq) - t_rl_A, lb_rr, ub_rr);

    // --- Candidate B: RR saturates, RL resolves (symmetric to A).
    const float t_rr_B = GP_CLAMP(t_rr_free, lb_rr, ub_rr);
    const float t_rl_B = GP_CLAMP((b_eq / a_eq) - t_rr_B, lb_rl, ub_rl);

    // --- Candidate "both": genuinely infeasible demand, independent clamp.
    const float t_rl_clamp_only = GP_CLAMP(t_rl_free, lb_rl, ub_rl);
    const float t_rr_clamp_only = GP_CLAMP(t_rr_free, lb_rr, ub_rr);

    // --- Convex blend. Weights sum to 1 identically (product-of-complements
    // partition), so t_rl/t_rr can never leave the convex hull of the four
    // already-bounded candidates above.
    const float w_free = (1.0f - sat_rl) * (1.0f - sat_rr);
    const float w_A    = sat_rl * (1.0f - sat_rr);
    const float w_B    = (1.0f - sat_rl) * sat_rr;
    const float w_both = sat_rl * sat_rr;

    const float t_rl = w_free * t_rl_free + w_A * t_rl_A + w_B * t_rl_B + w_both * t_rl_clamp_only;
    const float t_rr = w_free * t_rr_free + w_A * t_rr_A + w_B * t_rr_B + w_both * t_rr_clamp_only;

    t_out[GP_FL] = 0.0f;
    t_out[GP_FR] = 0.0f;
    t_out[GP_RL] = t_rl;
    t_out[GP_RR] = t_rr;

    if (qp_residual != NULL) {
        *qp_residual = fabsf(a_eq * (t_rl + t_rr) - b_eq);
    }
}