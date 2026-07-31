#ifndef V1_VEHICLE_DYNAMICS_H
#define V1_VEHICLE_DYNAMICS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "TeR_CONSTANTS.h"   /* PI, WHEEL_RADIUS, RED_RATIO — pure #defines, zero HAL coupling */
#include "v1_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define V1_VEHICLE_WB       1.530f
#define V1_VEHICLE_TW       1.220f
#define V1_K_UNDERSTEER     0.0025f
#define V1_GRAVITY_MS2      9.81f
#define V1_V_FLOOR_MS       0.5f

/* v1_lut_data.h's RPM axis is explicitly MOTOR-side (0-20000 RPM). Upstream,
 * TeR.wheelInfo.rl_rpm/rr_rpm are already WHEEL-side (erpm/MOTOR_POLES*RED_RATIO
 * in TeR_STATEMACHINE.c::permaTask). Feeding wheel RPM straight into the LUT —
 * what the original branch did — pins every lookup into the [0,2000] RPM
 * bracket above ~7 km/h, since wheel RPM never exceeds ~1400 at 30 m/s. This
 * constant undoes the reduction to recover true motor RPM before indexing. */
#define V1_GEAR_R           (1.0f / RED_RATIO)

typedef struct {
    float rl_nm;
    float rr_nm;
} v1_trq_map_t;

typedef struct {
    float kp_yaw;
    float ki_yaw;
    float k_ff;
    float max_yaw_moment_nm;
    float pi_windup_limit_nm;
    float peak_mu;
    float max_allowable_slip;
    float slip_cut_gain;

    /* v1.1 robustness additions */
    float steer_deadzone_rad;
    float yaw_deadzone_rads;
    float max_slew_nm_per_s;
} v1_params_t;

typedef struct {
    float error_integral_nm;
    float trq_prev_rl_nm;
    float trq_prev_rr_nm;
    bool  cut_active_rl;   /* diagnostic-only latch, see .c — NOT read by the control law */
    bool  cut_active_rr;
    bool  initialized;
} v1_state_t;

void   v1_init_params(v1_params_t *params);
void   v1_reset_state(v1_state_t *state);
size_t v1_state_sizeof(void); /* ctypes layout-drift guard, mirrors gp_tv_state_sizeof */

/*
 * Pure functional core — zero TeR_CAN.h / HAL dependency by construction.
 * All I/O explicit; this is what makes the control law SIL-testable
 * (v1_sanity_checks.py) without linking the STM32 HAL + DBC codec tree.
 *
 * vx:            ground-speed estimate. Caller should prefer a non-driven-axle
 *                source (front wheel speed) — see tv_mds.c::v1_ground_speed_ms.
 *                Deriving vx from the DRIVEN rear wheels contaminates the TC
 *                slip denominator with the exact slip TC exists to catch.
 * regen_enabled: same gate as regen_allowed() in TeR_TRQMANAGER.c — this core
 *                does not re-implement accumulator V/T/I checks; single source
 *                of truth stays there.
 */
v1_trq_map_t v1_tv_step(
    float apps_pct,
    float brake_pressure_bar,
    float steer_rad,
    float wz_measured_rads,
    float vx,
    float wheel_rpm_rl,
    float wheel_rpm_rr,
    float torque_limit_nm,
    uint8_t regen_enabled,
    float regen_max_trq_nm,
    float dt,
    const v1_params_t *params,
    v1_state_t *state
);

v1_trq_map_t v1_traction_control_step(
    v1_trq_map_t in,
    float wheel_rpm_rl,
    float wheel_rpm_rr,
    float vx,
    const v1_params_t *params,
    v1_state_t *state
);

#ifdef __cplusplus
}
#endif

#endif /* V1_VEHICLE_DYNAMICS_H */