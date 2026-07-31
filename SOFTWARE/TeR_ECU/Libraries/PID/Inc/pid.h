/*
 * pid.h
 *
 *  Created on: Mar 30, 2024
 *      Author: ozuba
 * A simple PID library using floating arithmetic for its usage all along the control systems
 * of the car.
 */

#ifndef INC_PID_H_
#define INC_PID_H_
#include <stdlib.h> //For memory allocation


typedef struct {
	//State Variables
	float error; //Error proporcional
	float errorD; //Error derivativo
	float errorI; //Error integral

	//Config Varialbes
	float Kp;  // Proportional gain
	float Ki;   // Integral gain
	float Kd;  // Derivative gain

	float prevError;
	float antiWindup; //Valor saturacion termino integral (Puede entenderse en las unidades de la integral, velocidad angular máxima en el caso del yawRate)
	float T; //Tiempo del lazo
} PID_t;



PID_t* initPID(float Kp, float Ki, float Kd, float loopTime, float iMax); //Returns a PID instance in the HEAP
void deInitPID(PID_t **pid); //Frees PID memory and sets pointer to NULL
float pid(PID_t *pid, float ref, float feedback); //Calculates PID response



#endif /* INC_PID_H_ */
