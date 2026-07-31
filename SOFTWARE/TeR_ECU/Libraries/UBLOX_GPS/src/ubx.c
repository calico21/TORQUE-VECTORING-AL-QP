/*
 * ubx.c
 *
 *  Created on: Jan 29, 2025
 *      Author: ozuba
 *
 *      sizeof(buffer), being buffer a VLA (variable length array) is calculated at runtime
 *      be careful with stack size
 */

#include "ubx.h"

uint16_t checksum(uint8_t *buffer, size_t size) {
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	for (int i = 0; i < size; i++) {
		ck_a = ck_a + buffer[i];
		ck_b = ck_b + ck_a;
	}
	return ((uint16_t) ck_b << 8) | (uint16_t) ck_a; //return checksum
}

uint8_t send_ubx(ubx_device_t *device, uint8_t class, uint8_t id,
		void *payload, uint16_t p_size) {
	uint32_t length = 6 + p_size + 2;
	uint8_t buffer[length]; //VLA with Preambles class, id, length and checksum;
	buffer[0] = 0xb5;
	buffer[1] = 0x62;
	buffer[2] = class;
	buffer[3] = id;
	//Payload length
	memcpy(&buffer[4], &p_size, sizeof(uint16_t)); //Copy two bytes for length
	//Payload
	if (payload != NULL) { //Check for null payload
		memcpy(&buffer[6], payload, p_size); //Copy the payload
	}
	//Calculate and put checksum
	uint16_t sum = checksum(&buffer[2], sizeof(buffer) - 4);
	memcpy(&buffer[6 + p_size], &sum, sizeof(sum)); //Copy checksum bytes
	//send with the callback function
	return device->write(buffer, length); //Write all
}

uint8_t poll_ubx(ubx_device_t *device, uint8_t class, uint8_t id,
		void *payload, uint16_t p_size) {
	//Prepare reception
	uint32_t length = 6 + p_size + 2;
	uint8_t buffer[length]; //VLA with Preambles class, id, length and checksum;
	//Start background listen (enables DMA listening)
	if (device->read(buffer, length)) {
		return 1; //return error
	}
	//Send message request
	send_ubx(device, class, id, NULL, 0);
	//Wait until it sets the received flag
	if(device->wait_for_data()){
		return 2; //timeouted
	}
	//handle checksum
	uint16_t rec_sum = ((uint16_t) buffer[length - 1] << 8)
			| (uint16_t) buffer[length - 2];
	if (checksum(&buffer[2], sizeof(buffer) - 4) != rec_sum) {
		return 3; //checksum error
	}
	//if all is okay copy the ubx to dest
	memcpy(payload, &buffer[6], p_size);

	return 0;
}

