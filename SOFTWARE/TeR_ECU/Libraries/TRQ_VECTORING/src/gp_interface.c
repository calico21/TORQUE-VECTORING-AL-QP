/*
 * gp_interface.c
 */

#include "gp_interface.h"
#include "gp_torque_vectoring.h"
#include "TeR_INERTIAL.h" // Para leer la IMU global
#include "TeR_CAN.h"      // Para leer la estructura global del monoplaza
#include "stm32f4xx_hal.h" // Necesario para acceder a los registros DWT y CoreDebug

// IDs propuestos para el Bus CAN
#define CAN_ID_TV_DYNAMICS   0x100
#define CAN_ID_TC_ESTIMATOR  0x101

#define GP_DEG2RAD  0.0174532925f
#define GP_KMH2MS   0.2777777778f
#define GP_RPM2RADS 0.1047197551f
#define GP_LOOPTIME 0.005f  // El bucle corre a 200Hz (5ms)

// Memoria estática del controlador
static tv_state_t gp_state;

// Variable global (o estática) para poder leerla desde el Live Monitor o Debugger
volatile float gp_execution_time_us = 0.0f;

void gp_init(void) {
    gp_tv_init(&gp_state);
}

trqMap_t gp_mode_intermediate(trq_t limit) {
    // 1. INICIAR CONTADOR DE CICLOS
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start_ticks = DWT->CYCCNT;

    trqMap_t out_map = {0, 0};

    // Lectura del Deseo del Piloto (APPS)
    float apps_norm = (float)TeR.apps.apps_av / 255.0f;
    apps_norm = GP_CLAMP(apps_norm, 0.0f, 1.0f);
    float total_torque_req = apps_norm * (float)limit;
    float fx_driver = total_torque_req / GP_R_WHEEL;

    // Lectura y Conversión Sensórica
    float vx = TeR.wheelInfo.speed * GP_KMH2MS;
    float vy = 0.0f; 

    float delta_volante = ter_steer_angle_decode(TeR.steer.angle) * GP_DEG2RAD;
    float delta_rueda = delta_volante / 5.0f;

    float wz = IMU.w_z * GP_DEG2RAD;
    float ay = IMU.a_y;             
    float ax = IMU.a_x;             

    float omega[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    omega[GP_RL] = TeR.wheelInfo.rl_rpm * GP_RPM2RADS;
    omega[GP_RR] = TeR.wheelInfo.rr_rpm * GP_RPM2RADS;

    // Lectura del Freno (Ahora en Bares de presión)
    #define MAX_BRAKE_PRESSURE_BAR 30.0f 
    
    float brake_norm = (float)TeR.bpps.bpps / MAX_BRAKE_PRESSURE_BAR;
    brake_norm = GP_CLAMP(brake_norm, 0.0f, 1.0f);

    // Ejecución del núcleo matemático
    float t_cmd_out[4] = {0.0f};
    gp_tv_step(fx_driver, delta_rueda, vx, vy, wz, ay, ax, omega, brake_norm, GP_LOOPTIME, &gp_state, t_cmd_out);

    // Empaquetado de salida
    out_map.rLeft  = (trq_t)t_cmd_out[GP_RL];
    out_map.rRight = (trq_t)t_cmd_out[GP_RR];

    // 2. PARAR CONTADOR Y CALCULAR TIEMPO
    uint32_t end_ticks = DWT->CYCCNT;
    uint32_t execution_ticks = end_ticks - start_ticks;
    
    // STM32F446RE corre a 180 MHz (1 tick = 1/180 us)
    gp_execution_time_us = (float)execution_ticks / 180.0f; 

    return out_map;
}

#include <stdint.h>
#include "gp_torque_vectoring.h"

void gp_pack_telemetry(const tv_state_t* state, uint8_t can_dyn[8], uint8_t can_tc[8], uint8_t can_act[8]) {
    
    // --- TRAMA 1: Dinámica y KKT (ID: 0x100) ---
    // vy_est (Deriva Lateral) -> Escala * 100
    int16_t vy_pack = (int16_t)(state->vy_est * 100.0f);
    can_dyn[0] = (vy_pack >> 8) & 0xFF; can_dyn[1] = vy_pack & 0xFF;
    
    // Integral del Yaw Rate (wz_int) -> Escala * 100
    int16_t wz_int_pack = (int16_t)(state->wz_int * 100.0f);
    can_dyn[2] = (wz_int_pack >> 8) & 0xFF; can_dyn[3] = wz_int_pack & 0xFF;

    // Kappa Target RL (RLS + Gradient Ascent) -> Escala * 10000
    uint16_t kopt_rl = (uint16_t)(state->tc.kappa_opt[GP_RL] * 10000.0f);
    can_dyn[4] = (kopt_rl >> 8) & 0xFF; can_dyn[5] = kopt_rl & 0xFF;

    // Kappa Target RR -> Escala * 10000
    uint16_t kopt_rr = (uint16_t)(state->tc.kappa_opt[GP_RR] * 10000.0f);
    can_dyn[6] = (kopt_rr >> 8) & 0xFF; can_dyn[7] = kopt_rr & 0xFF;

    // --- TRAMA 2: Observador RLS Pacejka (ID: 0x101) ---
    // Theta RL (Pendiente) -> Escala / 10
    int16_t theta_rl = (int16_t)(state->tc.rls_theta[GP_RL] / 10.0f);
    can_tc[0] = (theta_rl >> 8) & 0xFF; can_tc[1] = theta_rl & 0xFF;

    // Theta RR -> Escala / 10
    int16_t theta_rr = (int16_t)(state->tc.rls_theta[GP_RR] / 10.0f);
    can_tc[2] = (theta_rr >> 8) & 0xFF; can_tc[3] = theta_rr & 0xFF;

    // Mu RL y RR -> Escala * 1000
    uint16_t mu_rl = (uint16_t)(state->tc.mu_surface[0] * 1000.0f);
    can_tc[4] = (mu_rl >> 8) & 0xFF; can_tc[5] = mu_rl & 0xFF;
    uint16_t mu_rr = (uint16_t)(state->tc.mu_surface[1] * 1000.0f);
    can_tc[6] = (mu_rr >> 8) & 0xFF; can_tc[7] = mu_rr & 0xFF;

    // --- TRAMA 3: Actuadores Físicos (ID: 0x102) ---
    // Torque Comando RL (Lo que va al inversor) -> Escala * 10
    int16_t trq_rl = (int16_t)(state->t_out_prev[GP_RL] * 10.0f);
    can_act[0] = (trq_rl >> 8) & 0xFF; can_act[1] = trq_rl & 0xFF;

    // Torque Comando RR -> Escala * 10
    int16_t trq_rr = (int16_t)(state->t_out_prev[GP_RR] * 10.0f);
    can_act[2] = (trq_rr >> 8) & 0xFF; can_act[3] = trq_rr & 0xFF;

    // Slip Actual Filtrado RL -> Escala * 10000
    uint16_t kfilt_rl = (uint16_t)(state->tc.kappa_filt[GP_RL] * 10000.0f);
    can_act[4] = (kfilt_rl >> 8) & 0xFF; can_act[5] = kfilt_rl & 0xFF;

    // Slip Actual Filtrado RR -> Escala * 10000
    uint16_t kfilt_rr = (uint16_t)(state->tc.kappa_filt[GP_RR] * 10000.0f);
    can_act[6] = (kfilt_rr >> 8) & 0xFF; can_act[7] = kfilt_rr & 0xFF;
}