/*
 * display.h
 *
 *  Created on: 6. sep. 2018
 *      Author: mvols
 */

#ifndef DEVICES_HEADER_DISPLAY_H_
#define DEVICES_HEADER_DISPLAY_H_

#include "../drivers_header/spi.h"
#include "../drivers_header/delay.h"

typedef enum{
	font_small=0,
	font_medium,
	font_large
}font_size_t;

#define DISPLAY_LINES 			128
#define DISPLAY_DOTS_PER_LINE	128

#define DISPLAY_UPDATE_FLAG	(1<<0)
#define DISPLAY_COM			(1<<1)
#define DISPLAY_ALL_CLEAR	(1<<2)

void display_init(void);

void display_dummy(void);
void display_clear_and_update(void);
void display_update(void);

void display_put_string(uint8_t x, uint8_t y, char* data, font_size_t size);
void display_draw(uint8_t value, uint8_t x, uint8_t y);
void display_clear(void);

#endif /* DEVICES_HEADER_DISPLAY_H_ */
