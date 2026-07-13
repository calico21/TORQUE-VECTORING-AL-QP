#ifndef GP_TORQUE_VECTORING_H
#define GP_TORQUE_VECTORING_H

#include "gp_vehicle_model.h"
#include "gp_solver.h"
#include "gp_traction_control.h"

#define GP_TV_MAX_MZ         1500.0f
#define GP_TV_WZ_I_MAX       200.0f
#define GP_TV_RATE_LIMIT     5000.0f
#define GP_TV_EMA_ALPHA      0.2f

typedef struct {
    float wz_int;
    float delta_prev;
    float t_qp_prev[4];
    float t_out_prev[4];
    tc_state_t tc;
    float vy_est;     
    float alpha_qp;   
    float lam_prev;  
} tv_state_t;

void gp_tv_init(tv_state_t* state);

void gp_tv_step(
    float fx_driver, float delta, float vx, float vy, float wz, 
    float ay, float ax, const float omega[4], float brake_norm, float dt, 
    tv_state_t* state, float t_cmd_out[4]
);

#endif // GP_TORQUE_VECTORING_H