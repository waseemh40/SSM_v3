/*
 * mode_switch.h
 *
 *  Created on: 7. sep. 2018
 *      Author: mvols
 */

#ifndef DEVICES_HEADER_MODE_SWITCH_H_
#define DEVICES_HEADER_MODE_SWITCH_H_

#include "../drivers_header/pinmap.h"
#include "em_gpio.h"

#define SWITCH_COUNT	6

#define SW1_PORT	gpioPortD
#define SW1_PIN		8
#define SW2_PORT	gpioPortF
#define SW2_PIN		12
#define SW3_PORT	gpioPortB
#define SW3_PIN		11
#define SW4_PORT	gpioPortA
#define SW4_PIN		10
#define SW5_PORT	gpioPortA
#define SW5_PIN		9
#define SW6_PORT	gpioPortA
#define SW6_PIN		8

typedef struct{
	GPIO_Port_TypeDef port;
	uint32_t pin;
} pin_setup_t;

void mode_switch_init(void);
uint8_t mode_switch_read(void);

#endif /* DEVICES_HEADER_MODE_SWITCH_H_ */
