/*
 * delay.h
 *
 *  Created on: Apr 7, 2017
 *      Author: waseemh
 */

#ifndef SRC_DELAY_H_
#define SRC_DELAY_H_

#include "../drivers_header/pinmap.h"
#include "em_timer.h"

#define DELAY_TIMER 		TIMER2
#define DELAY_CLK			cmuClock_TIMER2
#define CLK_KHZ				7000
#define CLK_PRESCALAR		10

/*
 * public variables
 */
/*
 * private functions
 */
/*
 * public functions
 */
void 		delay_init(void);
void 		delay_enable(void);
void 		delay_disable(void);
void 		delay_ms(uint8_t ms_base2);

#endif /* SRC_DELAY_H_ */
