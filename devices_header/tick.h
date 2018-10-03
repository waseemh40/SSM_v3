/*
 * tick.h
 *
 *  Created on: 8. sep. 2018
 *      Author: mvols
 */

#ifndef DEVICES_HEADER_TICK_H_
#define DEVICES_HEADER_TICK_H_

#include <stdlib.h>
#include "em_chip.h"

void tick_init(void);
void tick_register_callback(void (*user_cb)(void), uint64_t period);
uint64_t tick_get(void);
void tick_delay(uint32_t ticks);

#endif /* DEVICES_HEADER_TICK_H_ */
