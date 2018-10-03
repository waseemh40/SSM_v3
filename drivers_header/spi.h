/*
 * spi.h
 *
 *  Created on: Mar 28, 2017
 *      Author: waseemh
 */

#ifndef SRC_SPI_H_
#define SRC_SPI_H_

#include "../drivers_header/pinmap.h"
#include "em_usart.h"

#define 	SPI_DATARATE 		3500000				//100kbps default
#define 	SPI_USART			USART1
#define 	SPI_USART_CLK		cmuClock_USART1
#define 	SPI_RX_IRQn			USART1_RX_IRQn
#define 	SPI_TX_IRQn			USART1_TX_IRQn
#define 	SPI_TX_ISR			USART1_TX_IRQHandler
#define 	SPI_RX_ISR			USART1_RX_IRQHandler

typedef enum{
	gps=0,
	sd_card,
	radio,
	display
}device_t;

typedef enum{
	msbf=0,
	lsbf
}byte_order_t;
/*
 * private functions
 */

/*
 * public functions
 */
void 		spi_init(void);
void 		spi_enable(void);
void 		spi_disable(void);
void 		spi_set_byte_order(byte_order_t order);
void 		spi_reset(void);
void 		spi_cs_set(device_t device);
void 		spi_cs_clear(device_t device);
uint8_t 	spi_read_write_byte(uint8_t data);
void 		spi_write_byte(uint8_t data );
uint8_t 	spi_read_byte( void );


#endif /* SRC_SPI_H_ */
