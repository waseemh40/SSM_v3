/*
 * analog.h
 *
 *  Created on: 20. sep. 2018
 *      Author: mvols
 */

#ifndef DEVICES_HEADER_ANALOG_H_
#define DEVICES_HEADER_ANALOG_H_

#include "em_adc.h"
#include "em_cmu.h"

#define BATTERY_RES1_K	240
#define BATTERY_RES2_K	100

#define TGRAD			-1920

typedef enum {
	BATTERY = 0,
	TEMPERATURE
} analog_type_t;

void analog_init(void);
int analog_read(analog_type_t type);

#endif /* DEVICES_HEADER_ANALOG_H_ */
