/*
 * mode_switch.c
 *
 *  Created on: 7. sep. 2018
 *      Author: mvols
 */

#include "../devices_header/mode_switch.h"

pin_setup_t switches[SWITCH_COUNT] = {
		{SW1_PORT, SW1_PIN},
		{SW2_PORT, SW2_PIN},
		{SW3_PORT, SW3_PIN},
		{SW4_PORT, SW4_PIN},
		{SW5_PORT, SW5_PIN},
		{SW6_PORT, SW6_PIN},
};

void mode_switch_init(void)
{
	for (int i = 0; i < SWITCH_COUNT; i++)
		GPIO_PinModeSet(switches[i].port, switches[i].pin, gpioModeInputPullFilter, 1);
}

uint8_t mode_switch_read(void)
{
	uint8_t value = 0;
	for (int i = 0; i < SWITCH_COUNT; i++)
		if (!GPIO_PinInGet(switches[i].port, switches[i].pin)) // Signal is low when ON
			value |= (1<<i);
	return value;
}
