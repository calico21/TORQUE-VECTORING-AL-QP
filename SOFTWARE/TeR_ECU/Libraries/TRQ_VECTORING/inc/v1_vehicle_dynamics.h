#ifndef V1_VEHICLE_DYNAMICS_H
#define V1_VEHICLE_DYNAMICS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "TeR_CONSTANTS.h"   /* PI, WHEEL_RADIUS, RED_RATIO */
#include "v1_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define V1_VEHICLE_WB               1.530f   /* Batalla [m] */
#define V1_VEHICLE_TW               1.220f   /* Ancho de vía [m] */
#define V1_VEHICLE_MASS             300.0f   /* Masa total del monoplaza + piloto [kg] */
#define V1_VEHICLE_H_CG             0.330f   /* Altura del centro de gravedad [m] */
#define V1_VEHICLE_REAR_RATIO       0.530f   /* 53% del peso sobre el eje trasero */
#define V1_K_UNDERSTEER             0.0025f  /* Gradiente subvirador [rad/(m/s^2)] */
#define V1_GRAVITY_MS2              9.81f
#define V1_V_FLOOR_MS               0.5f

#define V1_GEAR_R                   (1.0f / RED_RATIO)

typedef struct {
    float rl_nm;
    float rr_nm;
} v1_trq_map_t;

typedef struct {
    float kp_yaw;                 /* Ganancia proporcional de guñada [Nm / (rad/s)] */
    float ki_yaw;                 /* Ganancia integral de guñada [Nm / rad] */
    float k_ff;                   /* Ganancia feedforward estática [Nm / rad] */
    float k_ffd;                  /* Turn-in: Ganancia feedforward por velocidad de volante [Nm / (rad/s)] */
    float steer_dot_lpf_tau;      /* Constante de tiempo del filtro LPF de dirección [s] */
    float max_yaw_moment_nm;
    float pi_windup_limit_nm;
    float peak_mu;                /* Coeficiente de fricción dinámico (Dry=1.35, Wet=0.80) */
    float max_allowable_slip;
    float slip_cut_gain;

    /* Filtrado y Robustez */
    float steer_deadzone_rad;
    float yaw_deadzone_rads;
    float max_slew_nm_per_s;
    float speed_gain_taper;       /* Factor de atenuación de ganancias a alta velocidad */
    bool  enable_fz_load_transfer;/* Activar reparto base proporcional a carga vertical Fz */
} v1_params_t;

typedef struct {
    float error_integral_nm;
    float trq_prev_rl_nm;
    float trq_prev_rr_nm;
    float steer_prev_rad;
    float steer_dot_filt_rads;    /* Velocidad de volante filtrada [rad/s] */
    bool  cut_active_rl;
    bool  cut_active_rr;
    bool  initialized;
} v1_state_t;

void   v1_init_params(v1_params_t *params);
void   v1_reset_state(v1_state_t *state);
size_t v1_state_sizeof(void);

v1_trq_map_t v1_tv_step(
    float apps_pct,
    float brake_pressure_bar,
    float steer_rad,
    float wz_measured_rads,
    float ay_ms2,                 /* Aceleración lateral medida/estimada [m/s^2] */
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