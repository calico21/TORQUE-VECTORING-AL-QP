/*
 * TeR_IMU.h
 *
 *  Created on: Jan 9, 2025
 *      Author: eracing
 */

#ifndef INC_TER_INERTIAL_H_
#define INC_TER_INERTIAL_H_
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include <string.h>
#include "i2c.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "asm330lhh_reg.h"
#include "lis3mdl_reg.h"

typedef struct {
	float roll,pitch,yaw; // attitude
	float a_x,a_y,a_z; //Linear Accel
	float w_x,w_y,w_z; //Angular Rate
}imu_t;
//Complementary filter constants
#define ALPHA 0.6
#define DT 0.01  // Time step (e.g., 10ms)



//Task
void inertial(void *argument);

//Device Config Wrappers
void configIMU(void);
void configMAG(void);

//Make the importer be able to access
extern imu_t IMU;



// Complementary filter constant (adjust as needed)
void compFilter(float gx, float gy, float gz, float ax, float ay, float az,
                         float mx, float my, float mz, float *roll, float *pitch, float *yaw);



#endif /* INC_TER_INERTIAL_H_ */
