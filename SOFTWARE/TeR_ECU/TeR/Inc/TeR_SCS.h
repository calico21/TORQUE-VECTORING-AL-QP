/*
 * scs.h
 *
 *  Created on: Mar 28, 2024
 *      Author: piero oihan ozuba
 */

#ifndef INC_TER_SCS_H_
#define INC_TER_SCS_H_

#include <string.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "TeR_CAN.h"
#include "TeR_SL.h"

#define SCS {TER_APPS_FRAME_ID} //Añadir aqui las señales criticas
#define SCS_TIMEOUT 500  //Define el tiempo en unidades del timer (ms) que una scs puede desviarse como maximo



//Publicas
uint8_t initSCS(TIM_HandleTypeDef *timBase); // Takes a timebase (Timer@1khz) and a checking interrupt
uint8_t logSCS(uint32_t id); //Comprueba si la id esta en la lista y loguea su timestamp

void checkSCS(void); //Comprueba si alguna ID está caducada respecto al valor del timestamp actual, devuelve el ID de ella o 0 si esta en orden
uint8_t startSCS(void); //Arranca el sistema de SCS
uint8_t stopSCS(void); //Detiene el sistema de SCS (util para testing en elevador cuando quieres probar ciertas cosas)


#endif /* INC_TER_SCS_H_ */
