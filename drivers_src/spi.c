/*
 * spi.c
 *
 *  Created on: Mar 28, 2017
 *      Author: waseemh
 */


#include "../drivers_header/spi.h"

void spi_init(void){
	USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;
	usartInit.enable       	= usartDisable;
	usartInit.msbf       	= true;
	usartInit.master 		= true;
	usartInit.baudrate		= SPI_DATARATE;
	CMU_ClockEnable(SPI_USART_CLK, true);
	GPIO_PinModeSet(SPI_PORT, MOSI, gpioModePushPull, 1);
	GPIO_PinModeSet(SPI_PORT, SCK, gpioModePushPull, 1);
	GPIO_PinModeSet(CS_GPS_PORT, CS_GPS, gpioModePushPull, 1);
	GPIO_PinModeSet(CS_SD_CARD_PORT, CS_SD_CARD, gpioModePushPull, 1);
	GPIO_PinModeSet(CS_RADIO_PORT, CS_RADIO, gpioModePushPull, 1);
	GPIO_PinModeSet(CS_DISPLAY_PORT, CS_DISPLAY, gpioModePushPull, 0);
	GPIO_PinModeSet(SPI_PORT, MISO, gpioModeInput, 0);
	GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);
	USART_Reset(SPI_USART);
	USART_InitSync(SPI_USART, &usartInit);
	SPI_USART->ROUTE |= USART_ROUTE_LOCATION_LOC1 | USART_ROUTE_CLKPEN | USART_ROUTE_RXPEN | USART_ROUTE_TXPEN ;		//LOC1, TX & RX are enabled
	USART_IntClear(SPI_USART, _USART_IF_MASK);
	//USART_IntEnable(SPI_USART, USART_IF_RXDATAV);
	//USART_IntEnable(SPI_USART,USART_IF_TXBL);
	//USART_IntEnable(SPI_USART,USART_IF_TXC);
	//NVIC_ClearPendingIRQ(SPI_TX_IRQn);
	//NVIC_ClearPendingIRQ(SPI_RX_IRQn);
	//NVIC_EnableIRQ(SPI_TX_IRQn);
	//NVIC_EnableIRQ(SPI_RX_IRQn);
	spi_enable();
	return;
}

void spi_enable(void){
	USART_Enable(SPI_USART, usartEnable);
	return;
}

void spi_disable(void){
	USART_Enable(SPI_USART, usartDisable);
	return;
}

void spi_set_byte_order(byte_order_t order)
{
	if (order == msbf)
		SPI_USART->CTRL |= USART_CTRL_MSBF;
	else
		SPI_USART->CTRL &= ~USART_CTRL_MSBF;
}

void spi_reset(void){
	USART_Reset(SPI_USART);
	GPIO_PinOutSet(CS_GPS_PORT, CS_GPS);
	GPIO_PinOutSet(CS_RADIO_PORT, CS_RADIO);
	GPIO_PinOutSet(CS_SD_CARD_PORT, CS_SD_CARD);
	GPIO_PinOutClear(CS_DISPLAY_PORT, CS_DISPLAY);
	return;
}

void spi_cs_set(device_t device){
	if (device==gps){
		GPIO_PinOutSet(CS_GPS_PORT, CS_GPS);
	}
	else if (device==radio){
		GPIO_PinOutSet(CS_RADIO_PORT, CS_RADIO);
	}
	else if (device==sd_card){
		GPIO_PinOutSet(CS_SD_CARD_PORT, CS_SD_CARD);
	}
	else if (device==display)
	{
		GPIO_PinOutClear(CS_DISPLAY_PORT, CS_DISPLAY);
	}
	else{
		;
	}
	return;
}

void spi_cs_clear(device_t device){
	if (device==gps){
		GPIO_PinOutClear(CS_GPS_PORT, CS_GPS);

		GPIO_PinOutSet(CS_RADIO_PORT, CS_RADIO);
		GPIO_PinOutSet(CS_SD_CARD_PORT, CS_SD_CARD);
		GPIO_PinOutClear(CS_DISPLAY_PORT, CS_DISPLAY);
	}
	else if (device==radio){
		GPIO_PinOutClear(CS_RADIO_PORT, CS_RADIO);

		GPIO_PinOutSet(CS_GPS_PORT, CS_GPS);
		GPIO_PinOutSet(CS_SD_CARD_PORT, CS_SD_CARD);
		GPIO_PinOutClear(CS_DISPLAY_PORT, CS_DISPLAY);
	}
	else if (device==sd_card){
		GPIO_PinOutClear(CS_SD_CARD_PORT, CS_SD_CARD);

		GPIO_PinOutSet(CS_GPS_PORT, CS_GPS);
		GPIO_PinOutSet(CS_RADIO_PORT, CS_RADIO);
		GPIO_PinOutClear(CS_DISPLAY_PORT, CS_DISPLAY);
	}
	else if (device==display){
		GPIO_PinOutSet(CS_DISPLAY_PORT, CS_DISPLAY);

		GPIO_PinOutSet(CS_SD_CARD_PORT, CS_SD_CARD);
		GPIO_PinOutSet(CS_GPS_PORT, CS_GPS);
		GPIO_PinOutSet(CS_RADIO_PORT, CS_RADIO);
	}
	else{
		GPIO_PinOutClear(CS_DISPLAY_PORT, CS_DISPLAY);
		GPIO_PinOutSet(CS_GPS_PORT, CS_GPS);
		GPIO_PinOutSet(CS_RADIO_PORT, CS_RADIO);
		GPIO_PinOutSet(CS_SD_CARD_PORT, CS_SD_CARD);
	}
	return;
}

uint8_t spi_read_write_byte(uint8_t data){
	uint8_t rx_data=0;
	rx_data=USART_SpiTransfer(SPI_USART,data);
	//USART_IntClear(SPI_USART,USART_IF_TXBL);
	//USART_IntClear(SPI_USART,USART_IF_TXC);
	//USART_IntClear(SPI_USART,USART_IF_RXDATAV);
	return rx_data;
}

void spi_write_byte(uint8_t data ){
	//uint8_t rx_data=0;
	USART_SpiTransfer(SPI_USART,data);
	//USART_IntClear(SPI_USART,USART_IF_TXBL);
	//USART_IntClear(SPI_USART,USART_IF_TXC);
	//USART_IntClear(SPI_USART,USART_IF_RXDATAV);
	return;
}

uint8_t spi_read_byte( void ){
	uint8_t rx_data=0;
	rx_data=USART_SpiTransfer(SPI_USART,0xFF);
	//USART_IntClear(SPI_USART,USART_IF_TXBL);
	//USART_IntClear(SPI_USART,USART_IF_TXC);
	//USART_IntClear(SPI_USART,USART_IF_RXDATAV);
	return rx_data;
}
