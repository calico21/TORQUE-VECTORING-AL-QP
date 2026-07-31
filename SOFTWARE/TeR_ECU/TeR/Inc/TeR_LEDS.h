/*
 * TeR_LED.h
 *
 *  Created on: Jan 9, 2025
 *      Author: eracing
 */

#ifndef INC_TER_LEDS_H_
#define INC_TER_LEDS_H_
#include "stm32f405xx.h"
#include "cmsis_os2.h"
//#include "spi.h"
#include "spawn.h"
#include "math.h"

#define N_LEDS 200

typedef struct{
uint8_t r,g,b;
float a; //Brighness value 0-1
}pxl_t;

void sendPixel(pxl_t pixel);
void sendStrip(pxl_t* list, size_t size);



void leds(void *argument);
void generateRainbow(pxl_t *strip, size_t size, uint16_t offset);
void HSVtoRGB(uint16_t hue, uint8_t sat, uint8_t val, uint8_t *r, uint8_t *g, uint8_t *b);

#endif /* INC_TER_LEDS_H_ */
