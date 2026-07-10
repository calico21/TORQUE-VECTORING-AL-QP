/*
 * scs.c
 *
 *  Created on: Mar 28, 2024
 *      Author: oihan piero ozuba
 */

#include "TeR_SCS.h"
/*
 * Hay que añadir a tu gestor de interrupciones favorito el callback de checkeo
 *
 * Este módulo se encarga de gestionar las SCS del vehículo.
 * Si deja de recibirse cualquier señal considerada SCS (ver normativa) se deberán de tomar las
 * acciones pertinentes
 *
 */


//FreeRTOS dependencies
extern osThreadId_t systemCriticalTaskHandle;
extern osMutexId_t SL_RELAY_MUTEXHandle; // SL mutex

uint32_t prevConfig;

//FreeRTOS TASK
void systemCritical(void *argument) {
	prevConfig = 0xFFFFFFFF;
	for (;;) {
		osDelay(50);
		if (prevConfig != TeR.config.scs_enable) { // Ha cambiado la configuracion de las SCS?
			prevConfig = TeR.config.scs_enable; // actualizar previo estado al actual
			if (TeR.config.scs_enable == TER_ECU_CONFIG_SCS_ENABLE_ENABLE_CHOICE) { // tomamos acciones pertinentes
				startSCS();
			} else {
				stopSCS();
			}
		}
		checkSCS(); // checkeamos las SCS
	}
}

//Internal Id and timestamp lists
uint32_t scsIds[] = SCS; //Las señales las añade la macro del preprocesor
const uint8_t nSCS = sizeof(scsIds) / sizeof(uint32_t);
uint32_t timestamps[sizeof(scsIds) / sizeof(uint32_t)];

uint32_t lastFailSCS; //Id de la ultima señal que falló (Debugging purposes)

//Timer pointers used by module
TIM_HandleTypeDef *base;

//Module control functions
uint8_t initSCS(TIM_HandleTypeDef *timBase) {
//Alamacenamos los timers de uso
	base = timBase;
	return 1;
}

uint8_t startSCS(void) { //Activa la comprobación activa de tiempos
	//Resetea a 0 el timer y los timestamps para evitar errores al volver a arrancar
	memset(&timestamps, 0, sizeof(timestamps));
	base->Instance->CNT = 0;
	//Arranca los timers
	HAL_TIM_Base_Start(base); //arranca nuestra base de tiempo (Ojo hará overflow en 52 días jajaj)
	TeR.status.scs = 1;
	return 1;
}

uint8_t stopSCS(void) { //Desactiva la comprobación activa de tiempos
	HAL_TIM_Base_Stop(base); //Congela el timer haciendo que los checks difieran 0 a partir de ahora
	TeR.status.scs = 0;
	return 1;
}

//Log and check callback functions
uint8_t logSCS(uint32_t id) {
	uint8_t i = 0;
	while (i < nSCS && scsIds[i] != id) //Ojo el orden importa el segundo no se evalua si el primero falla (Crash al reves, acceso fuera del array)
		i++; //Comprueba si el ID está en el la lista de señales criticas, se para cuando hay un match o se ha excedido el tamaño del array
	if (i >= nSCS) { //Hemos recorrido el array entero (i no es un indice valido N-1)
		return 0; //No era una SCS
	} else { //Era una SCS
		timestamps[i] = base->Instance->CNT; //Loguea el valor del counter en su slot
		return 1; //Es una SCS
	}
	return -1; //Wtf no debería llegar aqui nunca
}

void checkSCS(void) {
	for (uint8_t i = 0; i < nSCS; i++) {
		if (base->Instance->CNT - timestamps[i] > SCS_TIMEOUT) { // SCS Fault
			lastFailSCS = scsIds[i]; //Guarda la id de la ultima señal problematica, util a modo de debug
			//easyCommand(TER_COMMAND_CMD_DISCHARGE_CHOICE); // Descarga el COCHE
			HAL_GPIO_WritePin(SC_EN_GPIO_Port, SC_EN_Pin, 0); //OPEN SC
			set_sl_request(SL_SCS, 0);
			TeR.apps.apps_av = 0; //Porsiaka
		} else {
			set_sl_request(SL_SCS, 1); //CLOSE SCS relay
		}
	}
}



