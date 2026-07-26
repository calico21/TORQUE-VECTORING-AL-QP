/*
 * gp_interface.h
 * Wrapper de integración entre TV y TeR_TRQMANAGER
 */

#ifndef GP_INTERFACE_H
#define GP_INTERFACE_H

#include <stdint.h>
#include "TeR_TRQMANAGER.h"
#include "gp_torque_vectoring.h"

// CAN Bus Message IDs for Telemetry
#define CAN_ID_TV_DYNAMICS   0x100
#define CAN_ID_TC_ESTIMATOR  0x101
#define CAN_ID_TV_ACTUATORS  0x102
#define CAN_ID_TV_DIAGNOSTICS 0x103  // Frame 4: Solver residual & AL-QP params

// Inicializa las memorias y estados estáticos
void gp_init(void);

// Función principal compatible con la estructura trqPipeline_t (*drivingMode)
trqMap_t gp_mode_intermediate(trq_t limit);

// Devuelve el puntero al estado interno por si se quiere inspeccionar desde fuera
const tv_state_t* gp_get_state(void);

// Empaquetado de telemetría CAN (4 tramas de 8 bytes)
void gp_pack_telemetry(
    const tv_state_t* state, 
    float qp_residual,
    uint8_t can_dyn[8], 
    uint8_t can_tc[8], 
    uint8_t can_act[8],
    uint8_t can_diag[8]
);

#endif // GP_INTERFACE_H