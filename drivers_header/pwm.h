/*
 * pwm.h
 *
 *  Created on: Mar 24, 2017
 *      Author: waseemh
 */

#ifndef SRC_PWM_H_
#define SRC_PWM_H_

#include "../drivers_header/pinmap.h"
#include "em_timer.h"

	//macros
#define 	PWM_TIMER 		TIMER1
#define 	PWM_CLK			cmuClock_TIMER1
#define	 	PWM_CHNL 		0				//CC channel					//NOT used
#define 	PWM_TOP			349				//pwm_frequency=HFRCO/PWM_TOP+1	//NOT used
#define 	PWM_CMPR		300				//pwm_ds=(PWM_CMPR/PWM_TOP)		//NOT used

	//functions
void 		pwm_init(void);
void 		pwm_enable(void);
void 		pwm_disable(void);
void 		pwm_reset(void);
void 		pwm_us_delay(uint16_t us_delay);
void 		pwm_generate_pulses(int num_pulses);	//NOT used
void 		pwm_continuous_mode();					//NOT used
#endif /* SRC_PWM_H_ */
