/*
 * TeR_SERVOS.h
 *
 *  Created on: Apr 18, 2025
 *      Author: piero
 */

#ifndef INC_TER_SERVOS_H_
#define INC_TER_SERVOS_H_
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
typedef struct{
	int16_t angle;
}flap_t;

#define SERVO_MIN_PULSE_SEC 0.0005
#define SERVO_MAX_PULSE_SEC 0.0025
#define SERVO_MAX_ANGLE     180.0

void setAngle(int8_t angle,uint8_t channel);

#endif /* INC_TER_SERVOS_H_ */
