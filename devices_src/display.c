/*
 * display.c
 *
 *  Created on: 6. sep. 2018
 *      Author: mvols
 */

#include "../devices_header/display.h"
#include "ugui.h"

// Display buffer for storing the entire frame
uint8_t display_buffer[DISPLAY_LINES][DISPLAY_DOTS_PER_LINE / 8];
// COM inversion variable
static uint8_t com_value = 0;

// Required by graphical library
UG_GUI gui;

void UserSetPixel (UG_S16 x, UG_S16 y, UG_COLOR c);
void UserSetPixel (UG_S16 x, UG_S16 y, UG_COLOR c)
{
	if (x > DISPLAY_DOTS_PER_LINE)
		x = DISPLAY_DOTS_PER_LINE - 1;
	if (y > DISPLAY_LINES)
		y = DISPLAY_LINES - 1;
	display_draw(c == 0, (uint8_t)x, (uint8_t)y);
}

void display_init(void)
{
	// Initialize SPI and clear display
	spi_init();
	display_clear_and_update();

	// Setup graphical library
	UG_Init(&gui, UserSetPixel, DISPLAY_DOTS_PER_LINE, DISPLAY_LINES);
	UG_FillScreen(C_WHITE);
	UG_FontSelect(&FONT_12X16);
	UG_SetBackcolor(C_WHITE);
	UG_SetForecolor(C_BLACK);
}

void display_put_string(uint8_t x, uint8_t y, char* data, font_size_t size)
{
	switch (size)
	{
	case font_small:
		UG_FontSelect(&FONT_5X8);
		break;
	case font_medium:
		UG_FontSelect(&FONT_8X12);
		break;
	case font_large:
	default:
		UG_FontSelect(&FONT_12X16);
	}
	UG_PutString(x, y, data);
}

void display_draw(uint8_t value, uint8_t x, uint8_t y)
{
	// Rotate by 90 degrees clockwise
	uint8_t old_x = x;
	uint8_t old_y = y;

	x = DISPLAY_LINES - 1 - old_y;
	y = old_x;

	// Set pixel in display buffer
	if (value)
		display_buffer[y][x>>3] &= ~(1<<(x&0x7));
	else
		display_buffer[y][x>>3] |= (1<<(x&0x7));
}

void display_clear(void)
{
	// Clear display buffer contents
	for (uint32_t line = 0; line < DISPLAY_LINES; line++)
		for (uint32_t dots = 0; dots < (DISPLAY_DOTS_PER_LINE / 8); dots++)
			display_buffer[line][dots] = 0xFF;
}

void display_dummy(void)
{
	// Toggle COM
	com_value ^= DISPLAY_COM;
	uint8_t cmd = com_value;

	// Dummy command, only updates COM
	spi_cs_clear(display);
	// LS013B7DH03 uses LSB first SPI communication.
	spi_set_byte_order(lsbf);

	spi_write_byte(cmd);

	spi_set_byte_order(msbf);
	spi_cs_set(display);
}

void display_clear_and_update(void)
{
	display_clear();

	// Toggle COM
	com_value ^= DISPLAY_COM;
	uint8_t cmd = DISPLAY_ALL_CLEAR | com_value;

	// Clear display
	spi_cs_clear(display);
	// LS013B7DH03 uses LSB first SPI communication.
	spi_set_byte_order(lsbf);

	spi_write_byte(cmd);

	spi_set_byte_order(msbf);
	spi_cs_set(display);
}

void display_update(void)
{
	// Toggle COM
	com_value ^= DISPLAY_COM;
	uint8_t cmd = DISPLAY_UPDATE_FLAG | com_value;

	// Write display buffer to display
	spi_cs_clear(display);
	// LS013B7DH03 uses LSB first SPI communication.
	spi_set_byte_order(lsbf);

	spi_write_byte(cmd);
	// Iterate over each line
	for (uint32_t line = 0; line < DISPLAY_LINES; line++)
	{
		spi_write_byte(line + 1);
		for (uint32_t dots = 0; dots < (DISPLAY_DOTS_PER_LINE / 8); dots++)
			spi_write_byte(display_buffer[line][dots]);
		spi_write_byte(0x00);
	}
	spi_write_byte(0x00);

	spi_set_byte_order(msbf);
	spi_cs_set(display);
}
