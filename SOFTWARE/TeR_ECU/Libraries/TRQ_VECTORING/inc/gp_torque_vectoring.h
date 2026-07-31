#ifndef GP_TORQUE_VECTORING_H
#define GP_TORQUE_VECTORING_H

#include <stdint.h>
#include "gp_vehicle_model.h"
#include "gp_solver.h"           // pulls in gp_params.h (GP_W_SMOOTH, GP_W_REG, ...)
#include "gp_traction_control.h" // pulls in gp_params.h (GP_TC_KP, GP_TC_KI, ...)
#include "gp_ekf.h"  // Unified EKF state estimator
#include "gp_nmpc.h"  

/* GP_W_SMOOTH / GP_W_REG / GP_TC_KP used to be redefined here a third time.
 * Removed: they are transitively available via the includes above, and
 * gp_params.h is now the only file tune_weights.py needs to patch. See
 * gp_params.h for rationale. */

// Controller Limits & Thresholds
#define GP_TV_MAX_MZ                1500.0f
#define GP_TV_WZ_I_MAX              200.0f
#define GP_TV_RATE_LIMIT            3252.3f
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

// Regen authority, rebuilt every 5 ms tick by the caller (gp_interface.c).
// `enable` mirrors the existing regen_allowed() safety gate (steering
// deadzone, cell V/T, accumulator current) — the solver never re-implements
// that logic. `max_total_trq` mirrors TeR.config.regen_max_trq (Nm,
// magnitude, BOTH wheels combined). `max_charge_power_w` is the electrical
// charge-power ceiling derived from TeR.config.regen_max_current, shaping
// the per-wheel bound the same way GP_P_MAX_WHL shapes the drive-side bound.
typedef struct {
    uint8_t enable;
    float   max_total_trq;
    float   max_charge_power_w;
} gp_regen_limits_t;

typedef struct {
    float wz_int;
    float delta_prev;
    float t_qp_prev[4];
    float t_out_prev[4];
    tc_state_t tc;
    gp_ekf_t ekf;
    gp_nmpc_state_t nmpc;
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
    float t_lb_rl_filt;   // NEW: filtered per-wheel regen (negative-torque) bound
    float t_lb_rr_filt;   // NEW
    float delta_nmpc_filt; // NMPC steering LPF state (~15ms tau)
} tv_state_t;

void gp_tv_init(tv_state_t* state);

void gp_tv_step(
    float fx_driver, float delta, float vx, float vy, float wz, 
    float ay, float ax, const float omega[4], float brake_norm, 
    float temp_inv_rl, float temp_inv_rr, float vy_gps, uint8_t gps_valid,
    const gp_regen_limits_t* regen,
    float dt, tv_state_t* state, float t_cmd_out[4]
);

#endif // GP_TORQUE_VECTORING_H