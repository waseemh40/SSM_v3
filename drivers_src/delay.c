/*
 * delay.c
 *
 *  Created on: Apr 7, 2017
 *      Author: waseemh
 */


#include "../drivers_header/delay.h"

/*
 * private variables
 */

/*
 * public variables
 */

/*
 * private functions
 */

/*
 * public functions
 */
void delay_init(void){
  	TIMER_Init_TypeDef timerDELAYInit = TIMER_INIT_DEFAULT;
  	timerDELAYInit.enable=false;
  	timerDELAYInit.prescale=timerPrescale1024;
	CMU_ClockEnable(DELAY_CLK, true);
	TIMER_TopSet(DELAY_TIMER, 0);
	TIMER_CounterSet(DELAY_TIMER, 0);
	TIMER_IntEnable(DELAY_TIMER, TIMER_IF_OF);
	TIMER_Init(DELAY_TIMER, &timerDELAYInit);
	return;

}

void delay_enable(){
	TIMER_Enable(DELAY_TIMER,true);
	return;
}

void delay_disable(){
	TIMER_Enable(DELAY_TIMER,false);
	return;
}

void delay_ms(uint8_t ms_base2){
	uint32_t 	converted_top	=0;
	if(ms_base2>13){ms_base2=13;}
	converted_top=((CLK_KHZ>>CLK_PRESCALAR)<<ms_base2);
	TIMER_TopSet(DELAY_TIMER, converted_top);
	TIMER_TopBufSet(DELAY_TIMER, converted_top);
	TIMER_CounterSet(DELAY_TIMER, 0);
	delay_enable();
	while(!TIMER_IntGetEnabled(DELAY_TIMER));
	TIMER_IntClear(DELAY_TIMER,TIMER_IF_OF);
	delay_disable();
	return;
}
