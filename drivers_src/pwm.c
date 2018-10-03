/*
 * pwm.c
 *
 *  Created on: Mar 24, 2017
 *      Author: waseemh
 */

#include "../drivers_header/pwm.h"
void pwm_init(){
				/*
	TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
	timerCCInit.cofoa=timerOutputActionSet;
	timerCCInit.cmoa=timerOutputActionClear;
  	timerCCInit.mode=timerCCModePWM;
  	TIMER_Init_TypeDef timerPWMInit = TIMER_INIT_DEFAULT;
  	timerPWMInit.enable=false;
  	timerPWMInit.prescale=timerPrescale1;
	CMU_ClockEnable(PWM_CLK, true);
	TIMER_TopSet(PWM_TIMER, PWM_TOP);
	TIMER_CounterSet(PWM_TIMER, 0);
	TIMER_CompareSet(PWM_TIMER, PWM_CHNL, PWM_CMPR);
	TIMER_CompareBufSet(PWM_TIMER, PWM_CHNL, PWM_CMPR);
	PWM_TIMER->ROUTE |= (1 << 16) |(1 << 0);		//Location 1, CC_2 routing for PE10
	TIMER_IntEnable(PWM_TIMER, TIMER_IF_CC0);
	TIMER_InitCC(PWM_TIMER, PWM_CHNL, &timerCCInit);
	TIMER_Init(PWM_TIMER, &timerPWMInit);
				 */
  	TIMER_Init_TypeDef timerCCInit = TIMER_INIT_DEFAULT;
  	timerCCInit.enable=false;
  	timerCCInit.prescale=timerPrescale1;
	CMU_ClockEnable(PWM_CLK, true);
	TIMER_TopSet(PWM_TIMER, 0);
	TIMER_CounterSet(PWM_TIMER, 0);
	TIMER_IntEnable(PWM_TIMER, TIMER_IF_OF);
	TIMER_Init(PWM_TIMER, &timerCCInit);
	return;
}

void pwm_enable(void){
	TIMER_Enable(PWM_TIMER,true);
	return;
}

void pwm_disable(void){
	TIMER_Enable(PWM_TIMER,false);
	return;
}

void pwm_reset(void){
	TIMER_Reset(PWM_TIMER);
	return;
}

void pwm_us_delay(uint16_t us_delay){
	uint32_t 	converted_top	=0;
	converted_top=(7*us_delay);
	TIMER_TopSet(PWM_TIMER, converted_top);
	TIMER_TopBufSet(PWM_TIMER, converted_top);
	TIMER_CounterSet(PWM_TIMER, 0);
	pwm_enable();
	while(!TIMER_IntGetEnabled(PWM_TIMER));
	TIMER_IntClear(PWM_TIMER,TIMER_IF_OF);
	pwm_disable();
	return;
}
		//NOT used functions
void pwm_generate_pulses(int num_pulses){
	int loop_var=0;
	TIMER_CompareSet(PWM_TIMER, PWM_CHNL, PWM_CMPR);
	pwm_enable();
	for (loop_var=1;loop_var<num_pulses;loop_var++){
		while(!TIMER_IntGetEnabled(PWM_TIMER));
		TIMER_IntClear(PWM_TIMER,TIMER_IF_CC0);
	}
	TIMER_CompareBufSet(PWM_TIMER, PWM_CHNL, PWM_TOP+1);
	return;
}

void pwm_continuous_mode(){
	TIMER_CompareBufSet(PWM_TIMER, PWM_CHNL, PWM_TOP+1);
	pwm_enable();
	return;
}
