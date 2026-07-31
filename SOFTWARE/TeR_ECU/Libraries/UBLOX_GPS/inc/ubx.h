/*
 * ubx.h
 *
 *  Created on: Jan 29, 2025
 *      Author: ozuba
 */

/*Library workflow is the following
 - Setup ubx_device comms abstraction
 - Send ubx request with class + id + payload(Optional)
 - Receive ubx, payload field is a union of all posible payloads
 */

#ifndef UBLOX_GPS_INC_UBX_H_
#define UBLOX_GPS_INC_UBX_H_
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "cmsis_os2.h"
#include "ubx_msgs.h"

//ubx_device struct handles write and read interfaces for HAL independance
typedef struct __attribute__((packed)){
	uint8_t (*write)(uint8_t *src, size_t size); //Takes info and sends to device return success
	uint8_t (*read)(uint8_t *dest, size_t size); //Requests N bytes from device return if data reading request was successfull, remember to set dataRead to 0 here
	uint8_t (*wait_for_data)(void); //returns 0 if waited succesfully, returns 1 if any error ocurred
}ubx_device_t;



uint16_t checksum(uint8_t* buffer,size_t size); //Ubx fletcher checksum algorithm
uint8_t send_ubx(ubx_device_t* device,uint8_t class, uint8_t id, void* payload,uint16_t p_size);
uint8_t poll_ubx(ubx_device_t* device,uint8_t class, uint8_t id, void* payload,uint16_t p_size);//Sends request and fills the message
#endif /* UBLOX_GPS_INC_UBX_H_ */
