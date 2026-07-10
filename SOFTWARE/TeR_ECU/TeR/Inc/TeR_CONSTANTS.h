/*
 * TeR_CONSTANTS.h
 *
 *  Created on: Apr 19, 2024
 *      Author: ozuba
 */

#ifndef INC_TER_CONSTANTS_H_
#define INC_TER_CONSTANTS_H_


//GENERAL
#define PI 3.14159265358979323846


//INVERTER RELATED
#define MOTOR_POLES 10 //For erpm to rpm or speed calculation


//TER RELATED
#define WHEEL_RADIUS 0.2032 //in m
#define RED_RATIO (1.0/5.0) //Relación de la reductora para la conversión PAR/VELOCIDAD con el motor
#define ELEC2MECH_EFF 0.94 //estimación de eficiencia electrico-mecanica
#define MAX_BPPS_VALUE 50.0
#endif /* INC_TER_CONSTANTS_H_ */
