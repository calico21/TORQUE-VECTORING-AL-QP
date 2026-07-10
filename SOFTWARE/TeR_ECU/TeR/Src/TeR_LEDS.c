/*
 * TeR_LED.c
 *
 *  Created on: Jan 9, 2025
 *      Author: eracing
 */
#include "TeR_LEDS.h"
pxl_t strip[N_LEDS]; // Array to hold image
uint16_t offset;
extern osThreadId_t ledsTaskHandle;

void leds(void *argument) {

	for (;;) {
		osDelay(0xFFFFFFFF);
	    // Adjust for animation speed
	}
}

void sendStrip(pxl_t *list, size_t size) {
	osKernelLock(); //Critical section
	for (uint16_t n = 0; n < size; n++) {
		sendPixel(list[n]); //Send pixels in orders
	}
	osKernelUnlock();
	osDelay(1);
}

void sendPixel(pxl_t pixel) {
	uint8_t r, g, b;
	r = pixel.r * pixel.a;
	g = pixel.g * pixel.a;
	b = pixel.b * pixel.a;
	uint32_t color = g << 16 | r << 8 | b; //Shift and place pixel trace
//Expand each bit to be 3 to ensure timing requirements
	uint8_t sendData[24];
	int indx = 0;

	for (int i = 23; i >= 0; i--) {
		if (((color >> i) & 0x01) == 1)
			sendData[indx++] = 0b110;  // store 1
		else
			sendData[indx++] = 0b100;  // store 0
	}
	//HAL_SPI_Transmit(&hspi2, sendData, 24, 1000); //Send pixel through spi
}

void generateRainbow(pxl_t *strip, size_t size, uint16_t offset) {
	for (uint16_t i = 0; i < size; i++) {
		uint16_t hue = (i * 360 / size + offset) % 360; // Calculate hue for each LED
		uint8_t r, g, b;
		HSVtoRGB(hue, 255, 255, &r, &g, &b); // Full saturation and brightness
		strip[i].r = r;
		strip[i].g = g;
		strip[i].b = b;
		strip[i].a = 0.5; // Full brightness
	}
}

void HSVtoRGB(uint16_t hue, uint8_t sat, uint8_t val, uint8_t *r, uint8_t *g,
		uint8_t *b) {
	float h = hue / 60.0f; // Hue sector (0 to 6)
	float s = sat / 255.0f;
	float v = val / 255.0f;

	int i = (int) h % 6;
	float f = h - i;
	float p = v * (1 - s);
	float q = v * (1 - f * s);
	float t = v * (1 - (1 - f) * s);

	switch (i) {
	case 0:
		*r = v * 255;
		*g = t * 255;
		*b = p * 255;
		break;
	case 1:
		*r = q * 255;
		*g = v * 255;
		*b = p * 255;
		break;
	case 2:
		*r = p * 255;
		*g = v * 255;
		*b = t * 255;
		break;
	case 3:
		*r = p * 255;
		*g = q * 255;
		*b = v * 255;
		break;
	case 4:
		*r = t * 255;
		*g = p * 255;
		*b = v * 255;
		break;
	case 5:
		*r = v * 255;
		*g = p * 255;
		*b = q * 255;
		break;
	}
}

