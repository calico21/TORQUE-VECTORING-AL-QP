/*
 * gp_interface.c
 * Integration wrapper between AL-QP Torque Vectoring and TeR_TRQMANAGER
 */

#include <stdint.h>
#include <math.h>
#include "gp_interface.h"
#include "gp_torque_vectoring.h"
#include "TeR_INERTIAL.h" 
#include "TeR_CAN.h"      
#include "stm32f4xx_hal.h"
#include "TeR_CONSTANTS.h"    // ELEC2MECH_EFF
#include "TeR_TRQMANAGER.h"   // regen_allowed()

#define GP_DEG2RAD  0.0174532925f
#define GP_KMH2MS   0.2777777778f
#define GP_RPM2RADS 0.1047197551f
#define GP_LOOPTIME 0.005f  // 200 Hz loop (5 ms)
#define GP_MAX_BRAKE_PRESSURE_BAR 50.0f

// Static controller state
static tv_state_t gp_state;

// Residual from the last QP solver execution
static float gp_last_qp_residual = 0.0f;

// Global execution time monitor (us)
volatile float gp_execution_time_us = 0.0f;

void gp_init(void) {
    gp_tv_init(&gp_state);
    gp_last_qp_residual = 0.0f;
}

const tv_state_t* gp_get_state(void) {
    return &gp_state;
}

/* 
 * Core execution pipeline: reads sensors, steps the control loop, 
 * calculates MCU benchmarks, and queues CAN telemetry.
 */
static inline void gp_execute_step_and_transmit(float fx_driver, trqMap_t *out_map) {
    // 1. START MCU CYCLE COUNTER BENCHMARK
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start_ticks = DWT->CYCCNT;

    // 2. READ PEDAL & BRAKE INPUTS
    float brake_pressure_bar = ter_bpps_bpps_decode(TeR.bpps.bpps);
    float brake_norm = brake_pressure_bar / GP_MAX_BRAKE_PRESSURE_BAR;
    brake_norm = GP_CLAMP(brake_norm, 0.0f, 1.0f);

    // 3. READ SENSORS (Fixed to match TeR_CAN.h struct layout)
    float front_v_kmh = ter_front_v_front_v_decode(TeR.speed.front_v);
    float vx = 0.0f;

    // Use non-driven front wheels for ground speed if available
    if (front_v_kmh > 0.1f) {
        vx = front_v_kmh * GP_KMH2MS;
    } else {
        // Fallback: average rear driven wheel speeds
        float rear_rpm_avg = 0.5f * ((float)TeR.wheelInfo.rl_rpm + (float)TeR.wheelInfo.rr_rpm);
        vx = rear_rpm_avg * GP_RPM2RADS * GP_R_WHEEL;
    }

    float vy = 0.0f; // Tracked internally by EKF
    float delta_volante = ter_steer_angle_decode(TeR.steer.angle) * GP_DEG2RAD;
    float delta_rueda = delta_volante / 5.0f;

    float wz = IMU.w_z * GP_DEG2RAD;
    float ay = IMU.a_y;             
    float ax = IMU.a_x;             

    float omega[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    omega[GP_RL] = (float)TeR.wheelInfo.rl_rpm * GP_RPM2RADS;
    omega[GP_RR] = (float)TeR.wheelInfo.rr_rpm * GP_RPM2RADS;

    // Inverter power stage temperatures
    float temp_inv_rl = (float)TeR.invInfo.left_power_stage_temp;
    float temp_inv_rr = (float)TeR.invInfo.right_power_stage_temp;

    // GPS lateral velocity from TeR.velbody (populated in TeR_GPS.c)
    float vy_gps = ter_vel_body_v_y_decode(TeR.velbody.v_y);
    uint8_t gps_valid = (fabsf(vy_gps) < 15.0f) ? 1 : 0;

    // 4. EXECUTE CORE MATHEMATICAL TV/TC STEP
    float t_cmd_out[4] = {0.0f};
    // 3.5 REGEN AUTHORITY — reuse the single safety-gate source of truth
    // (steering deadzone, cell V/T, accumulator current) and translate the
    // configurable regen current ceiling into an electrical charge-power
    // budget for the TV solver's per-wheel regen shaping.
    gp_regen_limits_t regen_limits;
    regen_limits.enable = regen_allowed();
    regen_limits.max_total_trq = (float)TeR.config.regen_max_trq;
    regen_limits.max_charge_power_w =
        (float)TeR.config.regen_max_current * GP_NOMINAL_PACK_VOLTAGE_V / ELEC2MECH_EFF;
    gp_tv_step(fx_driver, delta_rueda, vx, vy, wz, ay, ax, 
               omega, brake_norm, temp_inv_rl, temp_inv_rr, 
               vy_gps, gps_valid, &regen_limits,
               GP_LOOPTIME, &gp_state, t_cmd_out);

    // Map outputs to driver pipeline map
    out_map->rLeft  = (trq_t)t_cmd_out[GP_RL];
    out_map->rRight = (trq_t)t_cmd_out[GP_RR];

    // 5. STOP BENCHMARK TIMER (STM32F405 @ 168 MHz)
    uint32_t end_ticks = DWT->CYCCNT;
    uint32_t execution_ticks = end_ticks - start_ticks;
    gp_execution_time_us = (float)execution_ticks / 168.0f; 

    // 6. PACK AND DISPATCH TELEMETRY OVER CAN SCHEDULER
    uint8_t can_dyn[8], can_tc[8], can_act[8], can_diag[8];
    gp_pack_telemetry(&gp_state, gp_last_qp_residual, can_dyn, can_tc, can_act, can_diag);

    can_scheduler_insert_non_periodic_msg(can_dyn,  8, CAN_ID_TV_DYNAMICS,   0);
    can_scheduler_insert_non_periodic_msg(can_tc,   8, CAN_ID_TC_ESTIMATOR,  0);
    can_scheduler_insert_non_periodic_msg(can_act,  8, CAN_ID_TV_ACTUATORS,  0);
    can_scheduler_insert_non_periodic_msg(can_diag, 8, CAN_ID_TV_DIAGNOSTICS, 0);
}

trqMap_t gp_mode_intermediate(trq_t limit) {
    trqMap_t out_map = {0, 0};

    // Calculate requested force based on APPS pedal
    float apps_norm = (float)ter_apps_apps_av_decode(TeR.apps.apps_av) / 100.0f;
    apps_norm = GP_CLAMP(apps_norm, 0.0f, 1.0f);
    
    float total_torque_req = apps_norm * (float)limit;
    float fx_driver = total_torque_req / GP_R_WHEEL;

    gp_execute_step_and_transmit(fx_driver, &out_map);
    return out_map;
}

trqMap_t gp_mode_intermediate_custom_req(float total_torque_req) {
    trqMap_t out_map = {0, 0};
    float fx_driver = total_torque_req / GP_R_WHEEL;

    gp_execute_step_and_transmit(fx_driver, &out_map);
    return out_map;
}

void gp_pack_telemetry(
    const tv_state_t* state, 
    float qp_residual,
    uint8_t can_dyn[8], 
    uint8_t can_tc[8], 
    uint8_t can_act[8],
    uint8_t can_diag[8]
) {
    // --- FRAME 1: Dynamics and KKT (ID: 0x100) ---
    int16_t vy_pack = (int16_t)(state->vy_est * 100.0f);
    can_dyn[0] = (vy_pack >> 8) & 0xFF; can_dyn[1] = vy_pack & 0xFF;
    
    int16_t wz_int_pack = (int16_t)(state->wz_int * 100.0f);
    can_dyn[2] = (wz_int_pack >> 8) & 0xFF; can_dyn[3] = wz_int_pack & 0xFF;

    uint16_t kopt_rl = (uint16_t)(state->tc.kappa_opt[GP_RL] * 10000.0f);
    can_dyn[4] = (kopt_rl >> 8) & 0xFF; can_dyn[5] = kopt_rl & 0xFF;

    uint16_t kopt_rr = (uint16_t)(state->tc.kappa_opt[GP_RR] * 10000.0f);
    can_dyn[6] = (kopt_rr >> 8) & 0xFF; can_dyn[7] = kopt_rr & 0xFF;

    // --- FRAME 2: RLS Pacejka Estimator (ID: 0x101) ---
    int16_t theta_rl = (int16_t)(state->tc.rls_theta[GP_RL] / 10.0f);
    can_tc[0] = (theta_rl >> 8) & 0xFF; can_tc[1] = theta_rl & 0xFF;

    int16_t theta_rr = (int16_t)(state->tc.rls_theta[GP_RR] / 10.0f);
    can_tc[2] = (theta_rr >> 8) & 0xFF; can_tc[3] = theta_rr & 0xFF;

    uint16_t mu_rl = (uint16_t)(state->tc.mu_surface[0] * 1000.0f);
    can_tc[4] = (mu_rl >> 8) & 0xFF; can_tc[5] = mu_rl & 0xFF;
    
    uint16_t mu_rr = (uint16_t)(state->tc.mu_surface[1] * 1000.0f);
    can_tc[6] = (mu_rr >> 8) & 0xFF; can_tc[7] = mu_rr & 0xFF;

    // --- FRAME 3: Physical Actuators (ID: 0x102) ---
    int16_t trq_rl = (int16_t)(state->t_out_prev[GP_RL] * 10.0f);
    can_act[0] = (trq_rl >> 8) & 0xFF; can_act[1] = trq_rl & 0xFF;

    int16_t trq_rr = (int16_t)(state->t_out_prev[GP_RR] * 10.0f);
    can_act[2] = (trq_rr >> 8) & 0xFF; can_act[3] = trq_rr & 0xFF;

    uint16_t kfilt_rl = (uint16_t)(state->tc.kappa_filt[GP_RL] * 10000.0f);
    can_act[4] = (kfilt_rl >> 8) & 0xFF; can_act[5] = kfilt_rl & 0xFF;

    uint16_t kfilt_rr = (uint16_t)(state->tc.kappa_filt[GP_RR] * 10000.0f);
    can_act[6] = (kfilt_rr >> 8) & 0xFF; can_act[7] = kfilt_rr & 0xFF;

    // --- FRAME 4: Solver Diagnostics & Residual (ID: 0x103) ---
    uint16_t qp_pack = (uint16_t)(qp_residual * 1000.0f);
    can_diag[0] = (qp_pack >> 8) & 0xFF; can_diag[1] = qp_pack & 0xFF;

    // mz_sat_ratio, not alpha_qp: alpha_qp is a compile-time-derived constant
    // in the production (closed-form) solve path and never changes at
    // runtime — transmitting it as "Learning Step (Alpha)" implied a live
    // adaptive quantity that doesn't exist here. mz_sat_ratio is the actual
    // live anti-windup back-calculation signal (achieved-Δτ / requested-Δτ,
    // clamped [0,1]) — the number that tells a race engineer whether the
    // tires ran out of allocation room this lap.
    uint16_t mz_sat_pack = (uint16_t)(GP_CLAMP(state->mz_sat_ratio, 0.0f, 1.0f) * 10000.0f);
    can_diag[2] = (mz_sat_pack >> 8) & 0xFF; can_diag[3] = mz_sat_pack & 0xFF;

    can_diag[4] = 0; can_diag[5] = 0;
    can_diag[6] = 0; can_diag[7] = 0;
}