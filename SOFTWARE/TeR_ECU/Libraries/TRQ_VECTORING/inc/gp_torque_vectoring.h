#ifndef GP_TORQUE_VECTORING_H
#define GP_TORQUE_VECTORING_H

#include <stdint.h>
#include "gp_vehicle_model.h"
#include "gp_solver.h"           // pulls in gp_params.h (GP_W_SMOOTH, GP_W_REG, ...)
#include "gp_traction_control.h" // pulls in gp_params.h (GP_TC_KP, GP_TC_KI, ...)
#include "gp_ekf.h"  // Unified EKF state estimator

/* GP_W_SMOOTH / GP_W_REG / GP_TC_KP used to be redefined here a third time.
 * Removed: they are transitively available via the includes above, and
 * gp_params.h is now the only file tune_weights.py needs to patch. See
 * gp_params.h for rationale. */

// Controller Limits & Thresholds
#define GP_TV_MAX_MZ                1500.0f
#define GP_TV_WZ_I_MAX              200.0f
#define GP_TV_RATE_LIMIT            5000.0f
#define GP_TV_EMA_ALPHA             0.2f
#define GP_MAX_BRAKE_PRESSURE_BAR   50.0f

// Noise Gating & Signal Filtering Constants
#define GP_STEER_DEADZONE_RAD       0.0087f  // ~0.5 deg steering angle deadzone
#define GP_YAW_DEADZONE_RADS        0.0175f  // ~1.0 deg/s yaw rate deadzone
#define GP_ACCEL_LPF_TAU            0.0200f  // 20ms LPF (8 Hz cutoff) for accelerometers

// ── Profiling Telemetry Exports (Cortex-M Target Only) ────────────────
#if defined(__arm__) || defined(__ARM_ARCH)
extern volatile uint32_t g_tv_exec_cycles;
extern volatile float g_tv_exec_us;
#endif

typedef struct {
    float wz_int;
    float delta_prev;
    float t_qp_prev[4];
    float t_out_prev[4];
    tc_state_t tc;
    gp_ekf_t ekf;
    float vy_est;
    float alpha_qp;
    float lam_prev;
    float mz_sat_ratio;
    float vy_gps_last;
    float vy_gps_age_ms;
    
    // --- State-Isolated Filters (Prevents SIL Cross-Scenario Leakage) ---
    float ax_filt;
    float ay_filt;
    float t_ub_rl_filt;
    float t_ub_rr_filt;
} tv_state_t;

void gp_tv_init(tv_state_t* state);

void gp_tv_step(
    float fx_driver, float delta, float vx, float vy, float wz, 
    float ay, float ax, const float omega[4], float brake_norm, 
    float temp_inv_rl, float temp_inv_rr, float vy_gps, uint8_t gps_valid,
    float dt, tv_state_t* state, float t_cmd_out[4]
);

#endif // GP_TORQUE_VECTORING_H