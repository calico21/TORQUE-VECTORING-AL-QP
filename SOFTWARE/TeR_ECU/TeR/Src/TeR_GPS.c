/*
 * TeR_GPS.c
 *
 *  Created on: Jan 17, 2025
 *      Author: ozuba
 */

#include "TeR_GPS.h"
#include "TeR_CAN.h"

//Rtos uart event
osEventFlagsId_t uartEventFlags;

ubx_nav_pvt_t pvt;
ubx_device_t gps_d;
// UART1RXCallback function (private)
void uart1RxCallback(UART_HandleTypeDef *huart);

void gps(void *argument) {
	osDelay(0xFFFFFFFF);
	uartEventFlags = osEventFlagsNew(NULL); // Event definition
	//Assing function pointers to gps_d structure
	gps_d.write = &gps_write;
	gps_d.read = &gps_read;
	gps_d.wait_for_data = &gps_wait_for_data;
	//Enable UART1RXCallback
	HAL_UART_RegisterCallback(&huart1, HAL_UART_RX_COMPLETE_CB_ID,
			uart1RxCallback);

	//Prepare ubx to disable nmea
	ubx_cfg_prt p_cfg;
	memset(&p_cfg, 0, sizeof(p_cfg)); //set blank
	p_cfg.portID = 0x01; //Uart 1
	p_cfg.txReady = 0x00;
	p_cfg.mode = 0x000008D0; //No idea jajaj
	p_cfg.baudRate = 38400; //Current baud
	p_cfg.inProtoMask = 0b0000000000000001; //Activate just ubx
	p_cfg.outProtoMask = 0b0000000000000001; //Activate just ubx
	//Desactiva nmea
	send_ubx(&gps_d, 0x06, 0x00, &p_cfg, sizeof(p_cfg));

	//Activa el modo automotive
	ubx_cfg_nav5_t nav5;
	memset(&nav5, 0, sizeof(nav5)); //set blank
	nav5.mask = 0b000000000000001; // Apply just dyn model
	nav5.dynModel = 4; //Automotive mode
	//Configura modo automotive
	send_ubx(&gps_d, 0x06, 0x00, &nav5, sizeof(nav5));

	osDelay(500);
	for (;;) {
		//GPS test
		osDelay(100);
		poll_ubx(&gps_d, 0x01, 0x07, &pvt, sizeof(pvt)); //Continously poll for nav data
		//Dump gps data to IMU message
		TeR.latlong.latitude = pvt.lat;
		TeR.latlong.longitude = pvt.lon;
		TeR.velbody.v_x = pvt.velN/1000;
		TeR.velbody.v_y = pvt.velE/1000;
		TeR.velbody.v_z = pvt.velD/1000;
	}
}

uint8_t gps_read(uint8_t *dest, size_t size) {
	osEventFlagsClear(uartEventFlags, UART_RX_FLAG); // CLEAR UART FLAG, not really necessary as eventflagswait clears it by default
	return HAL_UART_Receive_DMA(&huart1, dest, size);
}

uint8_t gps_write(uint8_t *src, size_t size) {
	return HAL_UART_Transmit(&huart1, src, size, 100); //deactivate nmea
}

uint8_t gps_wait_for_data(void) {
	//Makes the os able to do other tasks while waiting for gps data
	if (osEventFlagsWait(uartEventFlags, UART_RX_FLAG, osFlagsWaitAll,
			1000) == UART_RX_FLAG) {
		return 0; //Data is available
	} else {
		return 1; //Timeouted
	}
}
// Existe la posibilidad de que la recepción salte ANTES de la declaracion del evento, lo paso a callbacks todo y queda mas fino
/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 if (huart == &huart1) { // only set flag when is our uart
 osEventFlagsSet(uartEventFlags, UART_RX_FLAG); //Set the data received to 1
 }
 }*/
void uart1RxCallback(UART_HandleTypeDef *huart) {
	osEventFlagsSet(uartEventFlags, UART_RX_FLAG); //Set the data received to 1
}

