/*
 * pid.c
 *
 *  Created on: Mar 30, 2024
 *      Author: ozuba
 *
 * A simple PID library using floating arithmetic for its usage all along the control systems
 * of the car. (Torque Vectoring, Thermal Management...). It is thought to be running in a periodic interrupt
 * but it can be easily modified to perceive time using a timeBase timer..
 *
 *
 * -------------------------------------------------[Seguridad]--------------------------------------------
 * Esto es un controlador, alguien va montado encima, si haces el tonto, es peligroso. Esta librería
 * Implementa medidas de seguridad como la posibilidad de saturar individualmente los terminos
 * Integral Derivativo
 */
#include "pid.h"

PID_t* initPID(float Kp, float Ki, float Kd, float loopTime, float iMax) {
	PID_t *pid; //temporal pointer for init
	pid = (PID_t*) calloc(1, sizeof(PID_t)); //Allocates one pid instance Zeroes memory to prevent disaster, ¡sizeof(pid_t)!
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->T = loopTime; //Tiempo del lazo
	pid->antiWindup = iMax; // Valor de saturación para el termino integral
	return pid;
}

void deInitPID(PID_t **pid) {
	if (pid && *pid) { // is the pointer passed valid, and is pointing to something valid?
		free(*pid); // safely free that pointer, only if it exists and is not null pointing
		*pid = NULL; // make it point to null
	}
}

float pid(PID_t *pid, float ref, float feedback) {
//PID
	pid->error = (ref - feedback); //Proporcional
	pid->errorI += pid->error * pid->T; //Integral
	pid->errorD = (pid->error - pid->prevError) / pid->T; //Derivativo
//Antiwindup check
	pid->errorI =
			(pid->errorI < pid->antiWindup) ? pid->errorI : +pid->antiWindup; //Upper bound check
	pid->errorI =
			(pid->errorI > -pid->antiWindup) ? pid->errorI : -pid->antiWindup; //Lower bound check

	return pid->Kp * pid->error + pid->Ki * pid->errorI + pid->Kd * pid->errorD; // P + I + D
}
