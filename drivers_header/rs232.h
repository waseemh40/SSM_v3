/*
 * rs232.h
 *
 *  Created on: Mar 27, 2017
 *      Author: waseemh
 */

#ifndef SRC_RS232_H_
#define SRC_RS232_H_

#include "../drivers_header/pinmap.h"
#include "em_usart.h"
#include "../fifo_buffers_header/fifo_rs232.h"

#define 	RS232_BAUDRATE 		115200
#define 	RS232_USART			USART0
#define 	RS232_USART_CLK		cmuClock_USART0
#define 	RS232_RX_IRQn		USART0_RX_IRQn
#define 	RS232_TX_IRQn		USART0_TX_IRQn
#define 	RS232_TX_ISR		USART0_TX_IRQHandler
#define 	RS232_RX_ISR		USART0_RX_IRQHandler

void 	rs232_init(void);
void 	rs232_enable(void);
void 	rs232_disable(void);
int 	rs232_transmit_string(const unsigned char* data,uint8_t length);
int 	rs232_transmit_char(unsigned char data );
char 	rs232_receive( void );			//not required in current scenario;implemented as blocking, change to INT if required
void 	rs232_reset(void);
void 	rs232_shutdown(void);

#endif /* SRC_RS232_H_ */
