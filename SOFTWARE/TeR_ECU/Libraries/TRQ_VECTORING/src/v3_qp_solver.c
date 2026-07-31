/*
 * v3_qp_solver.c
 *
 *  Branch: feat/v3-embedded-qp-production
 *  Module: AL-QP Solver implementation (C99, real-time safe, fixed-iterations with dual damping)
 */

#include "v3_qp_solver.h"
#include <math.h>
#include <string.h>

uint8_t v3_qp_config_init(v3_qp_config_t *cfg, const float wt[V3_N_WHEELS],
                         const float ws[V3_N_WHEELS], float rho) {
    if (cfg == NULL || wt == NULL || ws == NULL || rho <= 0.0f) {
        if (cfg != NULL) cfg->valid = 0;
        return 0;
    }

    for (uint8_t i = 0; i < (uint8_t)V3_N_WHEELS; i++) {
        float sum_w = wt[i] + ws[i];
        if (sum_w <= 0.0f) {
            cfg->valid = 0;
            return 0;
        }
        cfg->wt[i] = wt[i];
        cfg->ws[i] = ws[i];
        cfg->recip_w[i] = 1.0f / sum_w;
    }
    cfg->rho = rho;
    cfg->valid = 1;
    return 1;
}

void v3_qp_state_reset(v3_qp_state_t *state) {
    if (state != NULL) {
        state->lambda = 0.0f;
    }
}

void v3_qp_solve(const v3_qp_config_t *cfg, v3_qp_state_t *state,
                 const v3_qp_input_t *in, v3_qp_output_t *out) {
    if (cfg == NULL || state == NULL || in == NULL || out == NULL || cfg->valid == 0) {
        out->x[0] = 0.0f;
        out->x[1] = 0.0f;
        out->lambda = 0.0f;
        out->residual = 0.0f;
        out->feasible = 0;
        return;
    }

    float sum_lb = 0.0f;
    float sum_ub = 0.0f;
    for (uint8_t i = 0; i < (uint8_t)V3_N_WHEELS; i++) {
        sum_lb += in->lb[i];
        sum_ub += in->ub[i];
    }

    if (in->t_demand < sum_lb || in->t_demand > sum_ub) {
        out->feasible = 0;
        for (uint8_t i = 0; i < (uint8_t)V3_N_WHEELS; i++) {
            out->x[i] = fminf(fmaxf(in->x_nom[i], in->lb[i]), in->ub[i]);
        }
        out->lambda = state->lambda;
        out->residual = fabsf((out->x[0] + out->x[1]) - in->t_demand);
        return;
    }

    out->feasible = 1;

    float x[V3_N_WHEELS];
    for (uint8_t i = 0; i < (uint8_t)V3_N_WHEELS; i++) {
        float lb_i = in->lb[i];
        float ub_i = in->ub[i];
        if (!isfinite(lb_i)) lb_i = 0.0f;
        if (!isfinite(ub_i)) ub_i = 0.0f;
        
        float initial_guess = in->x_prev[i];
        if (!isfinite(initial_guess)) initial_guess = in->x_nom[i];
        x[i] = fminf(fmaxf(initial_guess, lb_i), ub_i);
    }

    float lambda = state->lambda;
    if (!isfinite(lambda)) lambda = 0.0f;

    float c[V3_N_WHEELS];
    for (uint8_t i = 0; i < (uint8_t)V3_N_WHEELS; i++) {
        float x_nom_i = isfinite(in->x_nom[i]) ? in->x_nom[i] : 0.0f;
        float x_prev_i = isfinite(in->x_prev[i]) ? in->x_prev[i] : 0.0f;
        c[i] = (cfg->wt[i] * x_nom_i + cfg->ws[i] * x_prev_i) * cfg->recip_w[i];
    }

    /* Bucle O(1) con amortiguamiento dual para evitar sobreimpulsos y saturaciones */
    const float dual_damping = 0.2f;

    for (uint32_t iter = 0u; iter < V3_QP_FIXED_ITERS; iter++) {
        float sum_x = x[0] + x[1];
        float residual = sum_x - in->t_demand;

        float dual_term = lambda + cfg->rho * residual;

        for (uint8_t i = 0; i < (uint8_t)V3_N_WHEELS; i++) {
            float unconstrained = c[i] - (dual_term * cfg->recip_w[i]);
            x[i] = fminf(fmaxf(unconstrained, in->lb[i]), in->ub[i]);
        }

        float new_sum_x = x[0] + x[1];
        float new_residual = new_sum_x - in->t_demand;
        
        /* Actualización amortiguada del multiplicador */
        lambda += dual_damping * cfg->rho * new_residual;

        if (lambda > V3_QP_LAMBDA_MAX) lambda = V3_QP_LAMBDA_MAX;
        if (lambda < -V3_QP_LAMBDA_MAX) lambda = -V3_QP_LAMBDA_MAX;
    }

    state->lambda = lambda;

    for (uint8_t i = 0; i < (uint8_t)V3_N_WHEELS; i++) {
        out->x[i] = x[i];
    }
    out->lambda = lambda;
    out->residual = fabsf((x[0] + x[1]) - in->t_demand);
}