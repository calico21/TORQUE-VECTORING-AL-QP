#include "v1_vehicle_dynamics.h"
#include "v1_lut_data.h"
#include "TeR_UTILS.h"
#include "tv_mds.h"
#include <math.h>

#define GRAVITY_MS2 9.81f
#define V_FLOOR_MS  0.5f

static v1_params_t v1_params;
static v1_state_t  v1_state = { 
    .error_integral_nm = 0.0f, 
    .cut_active_rl     = false, 
    .cut_active_rr     = false, 
    .initialized       = false 
};

void v1_init_params(v1_params_t *params) {
    if (!params) return;
    params->kp_yaw             = 85.0f;   /* Nm / (rad/s) */
    params->ki_yaw             = 12.0f;   /* Nm / rad */
    params->k_ff               = 32.0f;   /* Nm / rad */
    params->max_yaw_moment_nm  = 260.0f;  /* Max DYC moment [Nm] */
    params->pi_windup_limit_nm = 70.0f;   /* Integrator clamp [Nm] */
    params->peak_mu            = 1.35f;   /* Nominal dry grip coefficient */

    params->max_allowable_slip = 0.12f;   /* 12% slip ratio target */
    params->slip_cut_gain      = 12.0f;   /* Continuous attenuation factor */
}

void v1_reset_state(void) {
    v1_state.error_integral_nm = 0.0f;
    v1_state.cut_active_rl     = false;
    v1_state.cut_active_rr     = false;
    v1_state.initialized       = true;
}

/* 1 kHz Deterministic Torque Vectoring (Plugs into DriveConfig.drivingMode) */
trqMap_t v1_trqVectoring(trq_t limit) {
    if (!v1_state.initialized) {
        v1_init_params(&v1_params);
        v1_reset_state();
    }

    trqMap_t outMap = { .rLeft = 0, .rRight = 0 };

    /* 1. Base Torque Demand (APPS LUT / Regenerative Brake Blending) */
    trq_t base_trq = 0;
    float rear_rpm = 0.5f * ((float)TeR.wheelInfo.rl_rpm + (float)TeR.wheelInfo.rr_rpm);
    float pedal_pct = (float)TeR.apps.apps_av / 255.0f;

    if (ter_bpps_bpps_decode(TeR.bpps.bpps) > 5.0f) {
        /* Brake Pedal Priority: Linear Regen Allocation */
        float brake_pct = ter_bpps_bpps_decode(TeR.bpps.bpps) / 100.0f;
        base_trq = (trq_t)(-brake_pct * TeR.config.regen_max_trq * 0.5f);
    } else {
        /* Drive Torque Lookup */
        float lut_trq = v1_interp2d_drive_torque(pedal_pct, rear_rpm);
        base_trq = (trq_t)clampf(lut_trq, 0.0f, (float)limit * 0.5f);
    }

    /* 2. Sensor Signal Decoding & Unit Conversions */
    float steer_rad = ter_steer_angle_decode(TeR.steer.angle) * ((float)PI / 180.0f);
    float wz_measured = ter_ang_rate_yaw_rate_z_decode(TeR.angRate.yaw_rate_z) * ((float)PI / 180.0f);
    float vx = (TeR.wheelInfo.speed < V_FLOOR_MS) ? V_FLOOR_MS : TeR.wheelInfo.speed;

    /* 3. Adhesion-Saturated Bicycle Reference Model */
    float denom = V1_VEHICLE_WB + V1_K_UNDERSTEER * (vx * vx);
    float r_ref = (denom > 0.001f) ? ((vx / denom) * steer_rad) : 0.0f;

    /* Physical Adhesion Limit Saturation: r_max = (mu * g) / v_x */
    float r_max = (v1_params.peak_mu * GRAVITY_MS2) / vx;
    r_ref = clampf(r_ref, -r_max, r_max);

    /* 4. Feedforward + PI Error Controller */
    float yaw_error = r_ref - wz_measured;
    float m_ff = steer_rad * v1_params.k_ff;
    float m_p  = yaw_error * v1_params.kp_yaw;

    /* Anti-Windup: Conditional Integration */
    float total_unsat = m_ff + m_p + v1_state.error_integral_nm;
    bool is_saturated = (fabsf(total_unsat) >= v1_params.max_yaw_moment_nm);
    bool same_sign    = ((total_unsat * yaw_error) > 0.0f);

    if (!is_saturated || !same_sign) {
        v1_state.error_integral_nm += v1_params.ki_yaw * yaw_error * 0.005f; /* 200 Hz dt = 5ms */
        v1_state.error_integral_nm = clampf(v1_state.error_integral_nm, 
                                            -v1_params.pi_windup_limit_nm, 
                                             v1_params.pi_windup_limit_nm);
    }

    float m_z = clampf(m_ff + m_p + v1_state.error_integral_nm, 
                       -v1_params.max_yaw_moment_nm, 
                        v1_params.max_yaw_moment_nm);

    /* 5. Rear Axle Moment Allocation: delta_T = (M_z * R) / w */
    float delta_trq = (m_z * (float)WHEEL_RADIUS) / V1_VEHICLE_TW;

    outMap.rLeft  = (trq_t)(base_trq - delta_trq);
    outMap.rRight = (trq_t)(base_trq + delta_trq);

    return outMap;
}

/* Decoupled Continuous Slip-Cut Traction Control (Plugs into DriveConfig.tractionControl) */
trqMap_t v1_tractionControl(trqMap_t in) {
    float vx = (TeR.wheelInfo.speed < V_FLOOR_MS) ? V_FLOOR_MS : TeR.wheelInfo.speed;

    /* Calculate Rear Wheel Ground Velocities [m/s] */
    float v_rl = (TeR.wheelInfo.rl_rpm * (2.0f * (float)PI / 60.0f)) * (float)WHEEL_RADIUS;
    float v_rr = (TeR.wheelInfo.rr_rpm * (2.0f * (float)PI / 60.0f)) * (float)WHEEL_RADIUS;

    /* Slip Ratios */
    float slip_rl = (v_rl - vx) / vx;
    float slip_rr = (v_rr - vx) / vx;

    /* Hysteresis Flag Updates (for telemetry/logging) */
    if (slip_rl > v1_params.max_allowable_slip) v1_state.cut_active_rl = true;
    else if (slip_rl < (v1_params.max_allowable_slip - 0.03f)) v1_state.cut_active_rl = false;

    if (slip_rr > v1_params.max_allowable_slip) v1_state.cut_active_rr = true;
    else if (slip_rr < (v1_params.max_allowable_slip - 0.03f)) v1_state.cut_active_rr = false;

    /* Smooth Continuous Attenuation Factor (Prevents Chattering) */
    if (slip_rl > v1_params.max_allowable_slip && in.rLeft > 0) {
        float excess_slip = slip_rl - v1_params.max_allowable_slip;
        float factor_rl = 1.0f / (1.0f + v1_params.slip_cut_gain * excess_slip);
        in.rLeft = (trq_t)(in.rLeft * factor_rl);
    }

    if (slip_rr > v1_params.max_allowable_slip && in.rRight > 0) {
        float excess_slip = slip_rr - v1_params.max_allowable_slip;
        float factor_rr = 1.0f / (1.0f + v1_params.slip_cut_gain * excess_slip);
        in.rRight = (trq_t)(in.rRight * factor_rr);
    }

    return in;
}