/*
 * pinMap.h
 *
 *  Created on: Mar 23, 2017
 *      Author: waseemh
 */

#ifndef SRC_PINMAP_H_
#define SRC_PINMAP_H_

#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_system.h"
#include "em_chip.h"
#include "em_core.h"
#include "em_rtc.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

		/*
		 * mapping rule:
		 * gpioPort_n.pinNumber_x
		 * e.g. SPI_port.MOSI
		 */
		/*
		 * Oscillators and Clocks
		 */
#define 	LFXCO_FREQ			32768
		/*
		 * Pins Definition
		 */
		//power enable
//#define 	GPS_PWR_EN			0
#define 	SD_CARD_PWR_EN		0
//#define 	RADIO_PWR_EN		2
//#define 	RS485_12V_PWR_EN	3
#define	 	RS232_FORCEOFF 		5
		//SPI signals
#define 	MOSI				0
#define 	MISO				1
#define 	SCK					2
		//RS485 signals
#define 	RS485_TX			14
#define 	RS485_RX			15
#define 	RS485_DE			12
#define 	RS485_RE			13
		//RS232 signals
#define 	RS232_TX			10
#define 	RS232_RX			11
		//switches and LEDs
#define		LEDEN				10
#define 	SELECT				13
		//SWD signals
#define 	SWCLK				0
#define 	SWDIO				1
#define 	SWO					2
		//RADIO IO signals
#define 	RADIO_IO_0			0
#define 	RADIO_IO_1			1
#define 	RADIO_IO_2			8
#define 	RADIO_IO_3			9
#define 	RADIO_IO_4			10
#define 	RADIO_IO_5			11
#define 	CS_RADIO	 		4
		//GPS signals
#define		GPS_INT				5
#define 	GPS_TIME_PULSE		4
#define 	CS_GPS		 		3
		//SD card signals
#define 	CS_SD_CARD	 		2
		//Display
#define 	CS_DISPLAY	 		5
		//Status LEDs
#define 	LED_GPS		 		2
#define 	LED_RADIO	 		3
#define 	LED_R		 		8
#define 	LED_G		 		9
		/*
		 * Port Definition
		 */
#define 	SPI_PORT			gpioPortD
#define 	RS485_PORT			gpioPortE
#define 	RS232_PORT			gpioPortE
#define 	SWD_PORT			gpioPortF
#define 	LED_PORT 			gpioPortE
#define 	RS232_FORCEOFF_PORT	gpioPortA
#define 	CS_SD_CARD_PORT		gpioPortA
#define 	CS_GPS_PORT			gpioPortD
#define 	CS_RADIO_PORT		gpioPortA
#define 	CS_DISPLAY_PORT		gpioPortF
#define 	PWR_EN_PORT			gpioPortA
#define		GPS_SIG_PORT		gpioPortD
#define 	RADIO_IO_0345_PORT	gpioPortC
#define 	RADIO_IO_12_PORT	gpioPortC
#define 	LED_GPS_RADIO_PORT	gpioPortC
#define 	LED_RED_GREEN_PORT	gpioPortE

#endif /* SRC_PINMAP_H_ */
