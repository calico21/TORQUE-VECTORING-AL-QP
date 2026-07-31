#ifndef V2_VEHICLE_DYNAMICS_H
#define V2_VEHICLE_DYNAMICS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "TeR_CONSTANTS.h"
#include "v2_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define V2_VEHICLE_WB               1.530f   /* Batalla L [m] */
#define V2_VEHICLE_TW               1.220f   /* Ancho de vía w [m] */
#define V2_VEHICLE_LF               0.811f   /* Distancia CdG a eje delantero [m] */
#define V2_VEHICLE_LR               0.719f   /* Distancia CdG a eje trasero [m] */
#define V2_VEHICLE_MASS             300.0f   /* Masa total [kg] */
#define V2_VEHICLE_IZ               220.0f   /* Inercia de guñada I_z [kg*m^2] */
#define V2_VEHICLE_H_CG             0.330f   /* Altura CdG [m] */
#define V2_C_ALPHA_FRONT            35000.0f /* Rigidez de deriva delantera N/rad */
#define V2_C_ALPHA_REAR             32000.0f /* Rigidez de deriva trasera N/rad */
#define V2_GRAVITY_MS2              9.81f
#define V2_V_FLOOR_MS               0.5f

#define V2_GEAR_R                   (1.0f / RED_RATIO)

typedef struct {
    float rl_nm;
    float rr_nm;
} v2_trq_map_t;

typedef struct {
    /* Parámetros EKF (vy, bz) */
    float q_vy;                     /* Ruido de proceso vy */
    float q_bz;                     /* Ruido de proceso bz */
    float r_pseudo_vy;              /* Ruido de medida pseudo-cinemática */
    float r_gps_vy;                 /* Ruido de medida GPS */

    /* Parámetros SMC */
    float lambda_beta;              /* Peso de supresión de deriva beta */
    float lambda_int;               /* Peso integral de superficie */
    float k_smc;                    /* Ganancia de conmutación de superficie [Nm] */
    float phi_boundary_base;        /* Ancho de capa límite anti-chattering base */
    float k_ff;                     /* Feedforward estático */
    float k_ffd;                    /* Turn-in feedforward dSteer/dt */
    float max_yaw_moment_nm;
    float smc_integral_limit;
    float peak_mu;

    /* Traction Control & Robustez */
    float max_allowable_slip;
    float slip_cut_gain;
    float max_slew_nm_per_s;
    float steer_deadzone_rad;
    float yaw_deadzone_rads;
} v2_params_t;

typedef struct {
    /* 2-State EKF: x = [vy, bz]^T */
    float x_vy;                     /* Velocidad lateral estimada [m/s] */
    float x_bz;                     /* Offset/bias de giroscopio estimado [rad/s] */
    float P[2][2];                  /* Matriz de covarianza de error 2x2 */
    float beta_est_rad;             /* Ángulo de deriva estimado beta [rad] */

    /* Estado Controlador SMC */
    float smc_surface_integral;     /* Acumulador integral de superficie S */
    float trq_prev_rl_nm;
    float trq_prev_rr_nm;
    float steer_prev_rad;
    float steer_dot_filt_rads;
    bool  cut_active_rl;
    bool  cut_active_rr;
    bool  initialized;
} v2_state_t;

void   v2_init_params(v2_params_t *params);
void   v2_reset_state(v2_state_t *state);
size_t v2_state_sizeof(void);

v2_trq_map_t v2_tv_step(
    float apps_pct,
    float brake_pressure_bar,
    float steer_rad,
    float wz_measured_rads,
    float ay_ms2,
    float vx,
    float wheel_rpm_rl,
    float wheel_rpm_rr,
    float vy_gps,
    uint8_t gps_valid,
    float torque_limit_nm,
    uint8_t regen_enabled,
    float regen_max_trq_nm,
    float dt,
    const v2_params_t *params,
    v2_state_t *state
);

v2_trq_map_t v2_traction_control_step(
    v2_trq_map_t in,
    float wheel_rpm_rl,
    float wheel_rpm_rr,
    float vx,
    const v2_params_t *params,
    v2_state_t *state
);

#ifdef __cplusplus
}
#endif

#endif /* V2_VEHICLE_DYNAMICS_H */