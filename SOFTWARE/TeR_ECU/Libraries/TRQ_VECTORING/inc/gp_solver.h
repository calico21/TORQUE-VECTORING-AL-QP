#ifndef GP_SOLVER_H
#define GP_SOLVER_H

#include "gp_vehicle_model.h"

#define GP_QP_ITER    16
#define GP_W_REG      1.0f
#define GP_W_SMOOTH   0.3f
#define GP_RHO_AL     10.0f

void gp_nominal_allocation(float fx_driver, float mz_target, float t_nom_out[4]);

void gp_qp_solve_rwd(
    const float t_warmstart[4],
    const float t_prev[4],
    float fx_driver,
    const float t_lb[4],
    const float t_ub[4],
    float alpha_qp,       // <-- Reintegrado
    float* lam_prev_ptr,  // <-- Reintegrado
    float t_out[4],
    float* qp_residual
);

void gp_qp_solve_rwd_closedform(
    const float t_warmstart[4],
    const float t_prev[4],
    float fx_driver,
    const float t_lb[4],
    const float t_ub[4],
    float t_out[4],
    float* qp_residual
);

#endif // GP_SOLVER_H