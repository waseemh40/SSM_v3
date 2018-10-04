/*
 * led_and_switch.c
 *
 *  Created on: Oct 4, 2018
 *      Author: waseemh
 */


#include "../devices_header/led_and_switch.h"


	void 		init_led_switch(void){
		CMU_ClockEnable(cmuClock_GPIO, true);
		display_init();
		display_clear();
		delay_init();
			//Leds
		GPIO_PinModeSet(LED_RED_GREEN_PORT, LED_R, gpioModePushPull, 0);
		GPIO_PinModeSet(LED_RED_GREEN_PORT, LED_G, gpioModePushPull, 0);
		GPIO_PinModeSet(LED_GPS_RADIO_PORT, LED_GPS, gpioModePushPull, 0);
		GPIO_PinModeSet(LED_GPS_RADIO_PORT, LED_RADIO, gpioModePushPull, 0);
			//address switch
		GPIO_PinModeSet(SWITCH_1_PORT, SW_1, gpioModeInputPull , 1);
		GPIO_PinModeSet(SWITCH_2_PORT, SW_2, gpioModeInputPull , 1);
		GPIO_PinModeSet(SWITCH_3_PORT, SW_3, gpioModeInputPull , 1);
		GPIO_PinModeSet(SWITCH_456_PORT, SW_4, gpioModeInputPull , 1);
		GPIO_PinModeSet(SWITCH_456_PORT, SW_5, gpioModeInputPull , 1);
		GPIO_PinModeSet(SWITCH_456_PORT, SW_6, gpioModeInputPull , 1);
		return;
	}

	uint8_t 	read_switch(void){
		uint8_t var=0;
		var|=(GPIO_PinInGet(SWITCH_1_PORT, SW_1)<<0);
		var|=(GPIO_PinInGet(SWITCH_2_PORT, SW_2)<<1);
		var|=(GPIO_PinInGet(SWITCH_3_PORT, SW_3)<<2);
		var|=(GPIO_PinInGet(SWITCH_456_PORT, SW_4)<<3);
		var|= (GPIO_PinInGet(SWITCH_456_PORT, SW_5)<<4);
		var|= (GPIO_PinInGet(SWITCH_456_PORT, SW_6)<<5) ;
		return var;
	}

	void 		status_led_gps(bool status){
		if (status==true){
			GPIO_PinOutSet(LED_GPS_RADIO_PORT, LED_GPS);
		}
		else {
		GPIO_PinOutClear(LED_GPS_RADIO_PORT, LED_GPS);
		}
		return;
	}

	void 		status_led_radio(bool status){
		if (status==true){
			GPIO_PinOutSet(LED_GPS_RADIO_PORT, LED_RADIO);
		}
		else {
		GPIO_PinOutClear(LED_GPS_RADIO_PORT, LED_RADIO);
		}
		return;
	}

	void 		set_status_led(bool red, bool green){
		if (red==true){
			GPIO_PinOutSet(LED_RED_GREEN_PORT, LED_R);
		}
		else {
		GPIO_PinOutClear(LED_RED_GREEN_PORT, LED_R);
		}
		if (green==true){
			GPIO_PinOutSet(LED_RED_GREEN_PORT, LED_G);
		}
		else {
		GPIO_PinOutClear(LED_RED_GREEN_PORT, LED_G);
		}
		return;
	}
