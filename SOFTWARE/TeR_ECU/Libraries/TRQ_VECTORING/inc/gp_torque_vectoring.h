#ifndef GP_TORQUE_VECTORING_H
#define GP_TORQUE_VECTORING_H

#include <stdint.h>
#include "gp_vehicle_model.h"
#include "gp_solver.h"
#include "gp_traction_control.h"

#define GP_TV_MAX_MZ                1500.0f
#define GP_TV_WZ_I_MAX              200.0f
#define GP_TV_RATE_LIMIT            5000.0f
#define GP_TV_EMA_ALPHA             0.2f
#define GP_MAX_BRAKE_PRESSURE_BAR   50.0f

// Noise Gating & Signal Filtering Constants
#define GP_STEER_DEADZONE_RAD       0.0087f  // ~0.5 deg steering angle deadzone
#define GP_YAW_DEADZONE_RADS        0.0175f  // ~1.0 deg/s yaw rate deadzone
#define GP_ACCEL_LPF_TAU            0.0200f  // 20ms LPF (8 Hz cutoff) for accelerometers

typedef struct {
    float wz_int;
    float delta_prev;
    float t_qp_prev[4];
    float t_out_prev[4];
    tc_state_t tc;
    float vy_est;     
    float alpha_qp;
    float lam_prev;
    float mz_sat_ratio;
    float vy_gps_last;      // last GPS-derived lateral velocity [m/s]
    float vy_gps_age_ms;    // time since last GPS fix update [ms]

    // ── Diagnostic Telemetry Additions ──────────────
    float mz_req_logged;         // Desired yaw moment requested by PID
    float mz_achieved_logged;    // Actual yaw moment delivered by wheel torque split
    float qp_residual_logged;    // Equality constraint residual magnitude
    uint32_t branch_flap_count;  // Cumulative active-set branch flips (chattering index)
    uint8_t active_set_case;     // 0: Interior, 1: Single Saturation, 2: Double/Unreachable
    float max_slew_logged;       // Peak slew rate observed in current cycle
} tv_state_t;

void gp_tv_init(tv_state_t* state);

void gp_tv_step(
    float fx_driver, float delta, float vx, float vy, float wz, 
    float ay, float ax, const float omega[4], float brake_norm, 
    float temp_inv_rl, float temp_inv_rr, float vy_gps, uint8_t gps_valid,
    float dt, tv_state_t* state, float t_cmd_out[4]
);

#endif // GP_TORQUE_VECTORING_H