/*
 * TeR_SERVOS.c
 *
 *  Created on: Apr 18, 2025
 *      Author: piero
 *
 */
#include "TeR_SERVOS.h"
#include "TeR_CAN.h"

extern TIM_HandleTypeDef htim3;
flap_t flapL;
flap_t flapR;
uint8_t servos_on; //flag for servo enable/disable
void servos(void *argument) {
	for (;;) {
		osDelay(100);
		if (TeR.config.flap_enable == TER_ECU_CONFIG_FLAP_ENABLE_ON_CHOICE) {
			if (servos_on != 1) {
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
				servos_on = 1;
			}
			flapL.angle =
					TeR.apps.apps_av >= TeR.config.flap_pedal_setpoint ? 33 : 0;
			flapR.angle =
					TeR.apps.apps_av >= TeR.config.flap_pedal_setpoint ? 33 : 0;
			flapL.angle =
					TeR.config.flap_l_reverse
							== TER_ECU_CONFIG_FLAP_L_REVERSE_REVERSE_CHOICE ?
							-flapL.angle : flapL.angle;
			flapR.angle =
					TeR.config.flap_r_reverse
							== TER_ECU_CONFIG_FLAP_R_REVERSE_REVERSE_CHOICE ?
							-flapR.angle : flapR.angle;

			flapL.angle = flapL.angle + TeR.config.flap_l_offset;
			flapR.angle = flapR.angle + TeR.config.flap_r_offset;
			//flapL.angle = flapL.angle <=0 ? 0:flapL.angle;
			flapR.angle = flapR.angle >=360 ? 360:flapR.angle;
			flapL.angle = flapL.angle >=360 ? 360:flapL.angle;

			setAngle(flapL.angle, TIM_CHANNEL_1);
			setAngle(flapR.angle, TIM_CHANNEL_2);
		} else {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			servos_on = 0;
		}
	}

}

void setAngle(int8_t angle, uint8_t channel) {
	uint32_t incFreq = 2 * HAL_RCC_GetPCLK1Freq() / (htim3.Instance->PSC + 1);
	uint16_t low = incFreq * SERVO_MIN_PULSE_SEC;
	uint16_t high = incFreq * SERVO_MAX_PULSE_SEC;
	switch (channel) {
	case TIM_CHANNEL_1: // left servo
		TIM3->CCR1 = (angle) * (high - low) / SERVO_MAX_ANGLE + low;
		break;

	case TIM_CHANNEL_2: // right servo
		TIM3->CCR2 = (angle) * (high - low) / SERVO_MAX_ANGLE + low;
		break;
	}
}

