/*
 * TeR_UTILS.c
 *
 *  Created on: Jun 20, 2024
 *      Author: Ozuba
 */
#include "TeR_UTILS.h"

//Comprueba que un error sucede durante más de tMax
/* return: 0 ERROR, 1 OK
 * ok = 1 es bien, ok = 0 es mal
 * tMax, tiempo en ms
 * */
uint8_t checkPersistance(persist_t *instance, uint8_t ok, uint32_t tMax) {

	if (*instance > 0) { //Estabamos en error
		if (ok) { //No tenemos error
			*instance = 0; //Ponemos el timestamp a 0, ya no hay error
		} else if (osKernelGetTickCount() - *instance >= tMax) { //El error supera maxtime
			return 0; //Damos el error
		}
	} else if (!ok) { // no estabamos en error y ahora si
		*instance = osKernelGetTickCount();
	}

	return 1; //Tenemos Error pero no hemos superado maxTime
}

uint8_t heldFor(persist_t *instance, uint8_t cond, uint32_t tMax) {
    if (!cond) {            // no se cumple -> reset
        *instance = 0;
        return 0;
    }
    if (*instance == 0)     // acaba de empezar a cumplirse
        *instance = osKernelGetTickCount();
    return (osKernelGetTickCount() - *instance >= tMax) ? 1 : 0;
}
// Mapea un intervalo
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max) {
//Saturar las salidas si la entrada excede el límite de calibracion
	if (x < in_min)
		return out_min;
	if (x > in_max)
		return out_max;
//Mapear si estamos en rango seguro
	long val = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	return val;
}

//Mapea un intervalo (float)
float mapf(float x, float in_min, float in_max, float out_min,
		float out_max) {
//Saturar las salidas si la entrada excede el límite de calibracion
	if (x < in_min)
		return out_min;
	if (x > in_max)
		return out_max;
//Mapear si estamos en rango seguro
	float val = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	return val;
}
float clampf(float x, float lo, float hi) {
	return x < lo ? lo : (x > hi ? hi : x);
}
int32_t clamp(int32_t x, int32_t lo, int32_t hi) {
	return x < lo ? lo : (x > hi ? hi : x);
}

uint8_t read_btn(uint8_t *lock, uint8_t read) {
	if (osKernelGetTickCount() < 3000) { // esto es para que no pueda tocar nada sin querer en el arranque, 3 segundos
		return 0;
	}
	if (*lock && read) { //Si bloquado y boton == 1, devuelvo 0
		return 0;
	}
	//Si no hay lock o no estaba a 1 actualizo
	//bloqueo si pulsado
	//No hago nada sin no pulsado
	*lock = read;
	return read;
}
