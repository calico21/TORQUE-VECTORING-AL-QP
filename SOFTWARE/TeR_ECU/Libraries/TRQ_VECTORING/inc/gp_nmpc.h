#ifndef GP_NMPC_H
#define GP_NMPC_H

#include "gp_params.h"

// Horizon setup: 5 steps @ 10ms = 50ms prediction horizon
#define GP_NMPC_N          5        
#define GP_NMPC_DT         0.010f   
#define GP_NMPC_STATES     2        // x = [v_y, r]^T
#define GP_NMPC_INPUTS     1        // u = [M_z]^T (Yaw moment request)

// Fallback vehicle constants if not declared in gp_params.h
#ifndef GP_VEH_MASS
#define GP_VEH_MASS        230.0f   // Mass [kg]
#define GP_VEH_IZ          110.0f   // Yaw moment of inertia [kg*m^2]
#define GP_VEH_LF          0.800f   // CG to front axle distance [m]
#define GP_VEH_LR          0.730f   // CG to rear axle distance [m]
#define GP_VEH_CF          85000.0f // Front cornering stiffness [N/rad]
#define GP_VEH_CR          95000.0f // Rear cornering stiffness [N/rad]
#endif

typedef struct {
    float x_pred[GP_NMPC_N + 1][GP_NMPC_STATES];         // Predicted states [v_y, r]
    float A_d[GP_NMPC_N][GP_NMPC_STATES][GP_NMPC_STATES]; // Discretized A matrices
    float B_d[GP_NMPC_N][GP_NMPC_STATES][GP_NMPC_INPUTS]; // Discretized B matrices
    float u_warm[GP_NMPC_N];                              // Warm-start control sequence
} gp_nmpc_state_t;

// API Prototypes
void gp_nmpc_init(gp_nmpc_state_t *state);

void gp_nmpc_compute_jacobians(float v_x, 
                               float A_c[GP_NMPC_STATES][GP_NMPC_STATES], 
                               float B_c[GP_NMPC_STATES][GP_NMPC_INPUTS]);

void gp_nmpc_predict_trajectory(const float x_0[GP_NMPC_STATES],
                                 float v_x,
                                 float delta_sw,
                                 const float u_seq[GP_NMPC_N],
                                 gp_nmpc_state_t *state);

void gp_nmpc_solve_qp(const gp_nmpc_state_t *nmpc,
                      float r_ref,
                      float *u_opt);

void gp_nmpc_step(const float states[3], 
                  float delta_sw,
                  float r_ref,
                  gp_nmpc_state_t *nmpc_state,
                  float *mz_cmd);

#endif // GP_NMPC_H