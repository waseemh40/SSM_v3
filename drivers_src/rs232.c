/*
 * rs232.c
 *
 *  Created on: Mar 27, 2017
 *      Author: waseemh
 */

#include "../drivers_header/rs232.h"


void rs232_init(void){
	USART_InitAsync_TypeDef usartInit = USART_INITASYNC_DEFAULT;
	usartInit.enable       	= usartDisable;
	usartInit.baudrate     	= RS232_BAUDRATE;
	CMU_ClockEnable(RS232_USART_CLK, true);
	GPIO_PinModeSet(RS232_PORT, RS232_TX, gpioModePushPull, 1);
	GPIO_PinModeSet(RS232_FORCEOFF_PORT, RS232_FORCEOFF, gpioModePushPull, 1);
	GPIO_PinModeSet(RS232_PORT, RS232_RX, gpioModeInput, 0);
	USART_Reset(RS232_USART);
	USART_InitAsync(RS232_USART, &usartInit);
	RS232_USART->ROUTE |= (0<<8) | USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;		//LOC1, TX & RX are enabled
	USART_IntClear(RS232_USART, _USART_IF_MASK);
	USART_IntEnable(RS232_USART, USART_IF_RXDATAV);
	NVIC_ClearPendingIRQ(RS232_TX_IRQn);
	NVIC_ClearPendingIRQ(RS232_RX_IRQn);
	NVIC_EnableIRQ(RS232_TX_IRQn);
	NVIC_EnableIRQ(RS232_RX_IRQn);
	fifo_rs232_init();
	return;
}

void rs232_enable(void){
	USART_Enable(RS232_USART, usartEnable);
	return;
}

void rs232_disable(void){
	USART_Enable(RS232_USART, usartDisable);
	return;
}

int rs232_transmit_string(const unsigned char* data,uint8_t length){
	int loop_var=0;
	int ret_val=0;
	if(!fifo_rs232_is_full(fifo_tx_data)){
		for(loop_var=0;loop_var<length;loop_var++){
			fifo_rs232_add(fifo_tx_data,data[loop_var]);
		}
		USART_IntEnable(RS232_USART,USART_IF_TXBL);
		ret_val=1;
	}
	else {
		ret_val=-1;
	}
	return ret_val;
}

int rs232_transmit_char(uint8_t data){
	int ret_val=0;
	if(!fifo_rs232_is_full(fifo_tx_data)){
		fifo_rs232_add(fifo_tx_data,data);
		USART_IntEnable(RS232_USART,USART_IF_TXBL);
		ret_val=1;
	}
	else{
		ret_val=-1;
	}
	return ret_val;
}

char rs232_receive(void){
	char temp_char='@';
	if(!fifo_rs232_is_empty(fifo_rx_data)){
		temp_char=fifo_rs232_remove(fifo_rx_data);
	}
	return temp_char;
}

void rs232_reset(void){
	USART_Reset(RS232_USART);
	return;
}
void rs232_shutdown(void){
	GPIO_PinOutClear(RS232_FORCEOFF_PORT, RS232_FORCEOFF);
	return;

}

/*
 * INT handlers
 */
void RS232_TX_ISR(){
	USART_IntClear(RS232_USART, USART_IF_TXBL);
	 if (RS232_USART->STATUS & USART_STATUS_TXBL){
		if(!fifo_rs232_is_empty(fifo_tx_data)){
			USART_Tx(RS232_USART,(uint8_t) fifo_rs232_remove(fifo_tx_data));
		}
	  else{
		  	USART_IntDisable(RS232_USART,USART_IF_TXBL);
	  	}
	  }
}


void RS232_RX_ISR(){
	USART_IntClear(RS232_USART, USART_IF_RXDATAV);
	 if (RS232_USART->STATUS & USART_STATUS_RXDATAV){
	 		if(!fifo_rs232_is_full(fifo_rx_data)){
	 			fifo_rs232_add(fifo_rx_data, USART_Rx(RS232_USART));
	 		}
	 	}

}

