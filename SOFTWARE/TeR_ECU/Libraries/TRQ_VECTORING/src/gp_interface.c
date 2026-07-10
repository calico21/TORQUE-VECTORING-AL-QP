/*
 * gp_interface.c
 */

#include "gp_interface.h"
#include "gp_torque_vectoring.h"
#include "TeR_INERTIAL.h" // Para leer la IMU global
#include "TeR_CAN.h"      // Para leer la estructura global del monoplaza
#include "stm32f4xx_hal.h" // Necesario para acceder a los registros DWT y CoreDebug

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
    /* Descomentar cuando la variable CAN esté definida:
    omega[GP_RL] = TeR.wheelInfo.rl_rpm * GP_RPM2RADS; 
    omega[GP_RR] = TeR.wheelInfo.rr_rpm * GP_RPM2RADS;
    */

    // Asumimos que bpps viene en 0-255 al igual que el acelerador
    float brake_norm = (float)TeR.bpps.bpps / 255.0f;
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