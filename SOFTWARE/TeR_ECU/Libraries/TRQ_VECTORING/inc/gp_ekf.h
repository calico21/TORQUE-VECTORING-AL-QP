#ifndef GP_EKF_H
#define GP_EKF_H

#include <stdint.h>
#include <math.h>
#include "gp_params.h"
#include "gp_math.h"
#include "gp_vehicle_model.h"     // For GP_MASS, GP_LF, GP_LR, GP_WB, GP_C_ALPHA_R
#include "gp_traction_control.h"  // For GP_TC_MU_LO, GP_TC_MU_HI

// State Vector Indices (Lean 2-State Filter)
#define GP_EKF_STATE_VY      0  // Lateral velocity [m/s]
#define GP_EKF_STATE_BW      1  // Gyro yaw rate bias [rad/s]
#define GP_EKF_NUM_STATES    2

typedef struct {
    float x[GP_EKF_NUM_STATES];                    // State vector [vy, bw]
    float P[GP_EKF_NUM_STATES][GP_EKF_NUM_STATES]; // Error Covariance Matrix (2x2)
    float Q[GP_EKF_NUM_STATES];                    // Process noise diagonal
    
    float delta_ref;                               // Stored steering angle [rad]
    float R_gps_vy;                                // GPS lateral velocity variance
    float R_pseudo_vy;                             // Kinematic steady-state vy variance
    float R_mu;                                    // Reserved for friction variance

    float beta_est;                                // Chassis sideslip angle [rad]
    float vy_std;                                  // Real-time standard deviation of vy [m/s]
    float wz_corrected;                            // Bias-corrected yaw rate [rad/s]
} gp_ekf_t;

void gp_ekf_init(gp_ekf_t* ekf);

void gp_ekf_predict(
    gp_ekf_t* ekf, 
    float delta, float ax_filt, float ay_filt, float wz_raw, float vx, 
    float dt
);

void gp_ekf_update_gps(
    gp_ekf_t* ekf, 
    float vy_gps, uint8_t gps_valid
);

void gp_ekf_update_kinematic_ss(
    gp_ekf_t* ekf,
    float ay_filt, float wz_raw, float vx
);

void gp_ekf_update_friction(
    gp_ekf_t* ekf, 
    float mu_meas_rl, float mu_meas_rr, float t_mean_abs
);

#endif // GP_EKF_H