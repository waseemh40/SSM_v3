/*
 * led_and_switch.h
 *
 *  Created on: Oct 4, 2018
 *      Author: waseemh
 */

#ifndef DEVICES_HEADER_LED_AND_SWITCH_H_
#define DEVICES_HEADER_LED_AND_SWITCH_H_

#include "../drivers_header/pinmap.h"
#include "../drivers_header/delay.h"
#include "../devices_header/display.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

	void 		init_led_switch(void);
	uint8_t 	read_switch(void);
	void 		status_led_gps(bool status);
	void 		status_led_radio(bool status);
	void 		set_status_led(bool red, bool green);

#endif /* DEVICES_HEADER_LED_AND_SWITCH_H_ */
