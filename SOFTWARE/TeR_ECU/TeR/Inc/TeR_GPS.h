/*
 * TeR_GPS.h
 *
 *  Created on: Jan 17, 2025
 *      Author: ozuba
 */

#ifndef INC_TER_GPS_H_
#define INC_TER_GPS_H_
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "cmsis_os2.h"
#include "ubx.h"
//UART FLAG
#define UART_RX_FLAG  (0x01U)
//Task
void gps(void *argument);
//IO abstraction
uint8_t gps_read(uint8_t *dest, size_t size);
uint8_t gps_write(uint8_t *src, size_t size);
uint8_t gps_wait_for_data(void);
#endif /* INC_TER_GPS_H_ */
