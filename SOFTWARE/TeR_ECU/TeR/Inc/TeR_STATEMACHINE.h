/*
 * stateMachine.h
 *
 *  Created on: Feb 1, 2024
 *      Author: Ozuba
 *
 * Se ha elegido el aproach de consultar todas las condiciones antes de ejecutar el estado
 */

#ifndef INC_TER_STATEMACHINE_H_
#define INC_TER_STATEMACHINE_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "TeR_CAN.h"
#include "TeR_TRQMANAGER.h"
#include "TeR_CONSTANTS.h"
#include "TeR_UTILS.h"
#include "TeR_CONFIG.h"
//-------------------------[Asignacion de entradas Estandar]--------------------------//
#define TSMS_GPIO_Port DIN0_GPIO_Port
#define TSMS_Pin DIN0_Pin

#define BL_GPIO_Port DOUT0_GPIO_Port
#define BL_Pin DOUT0_Pin

//------------------------------------------------------------------------------------//

typedef enum {
	WAIT_SL, // Comprueba SL esta bien
	RDY2PRECH, // Espera a recibir el comando de precarga
	PRECHARGING, //Estado transitorio, monitoriza que todo va bien
	PRECHARGED, //Espera a que se reciba el comando de r2d
	DRIVING //Se permite el movimiento del vehiculo
} state_t; //Estados


void stateMachine(void *argument); //Task
#endif /* INC_TER_STATEMACHINE_H_ */
