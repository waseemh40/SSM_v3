/*
 * rs485.c
 *
 *  Created on: Mar 29, 2017
 *      Author: waseemh
 */

#include "../drivers_header/rs485.h"

/*
 * private variables
 */
static			fifo_tbr_type_t			current_rx_fifo=fifo_tbr_rx_data;
static 			char 					isr_rx_tx_char='0';

/*
 * public variables
 */

/*
 * private functions
 */
/*
 * public functions
 */
void rs485_init(void){
	LEUART_Init_TypeDef leuartInit = {
			.enable=	leuartDisable,
			.baudrate=	RS485_BAUDRATE,
			.databits=	leuartDatabits8,
			.parity=	leuartNoParity,
			.refFreq=   0,
			.stopbits= 	leuartStopbits1
	};
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_CORELEDIV2);
	CMU_ClockEnable(cmuClock_LFB, true);
	LEUART_FreezeEnable(RS485_UART,true);
	CMU_ClockEnable(RS485_UART_CLK, true);
	GPIO_PinModeSet(RS485_PORT, RS485_TX, gpioModePushPull, 1);
	GPIO_PinModeSet(RS485_PORT, RS485_DE, gpioModePushPull, 1);
	GPIO_PinModeSet(RS485_PORT, RS485_RE, gpioModePushPull, 1);
	GPIO_PinModeSet(RS485_PORT, RS485_RX, gpioModeInput, 0);
	LEUART_Reset(RS485_UART);
	LEUART_Init(RS485_UART, &leuartInit);
	RS485_UART->ROUTE |= (0x1UL <<9) | LEUART_ROUTE_TXPEN | LEUART_ROUTE_RXPEN;		//LOC2, TX & RX are enabled
	LEUART_IntClear(RS485_UART, _LEUART_IF_MASK);
	LEUART_IntEnable(RS485_UART, LEUART_IF_RXDATAV);
	NVIC_ClearPendingIRQ(RS485_IRQn);
	NVIC_EnableIRQ(RS485_IRQn);
	LEUART_FreezeEnable(RS485_UART,false);
	rs485_rx_mode();
	fifo_tbr_init();
	return;
}

void rs485_enable(void){
	LEUART_Enable(RS485_UART, leuartEnable);
	return;
}

void rs485_disable(void){
	LEUART_Enable(RS485_UART, leuartDisable);
	return;
}

void rs485_reset(void){
	LEUART_Reset(RS485_UART);
	return;
}

int rs485_transmit_string(char* data,uint8_t length){
	int loop_var=0;
	int ret_val=0;
	if(!fifo_tbr_is_full(fifo_tbr_tx_cmd)){
	for(loop_var=0;loop_var<length;loop_var++){
		fifo_tbr_add(fifo_tbr_tx_cmd,data[loop_var]);
	}
	rs485_tx_mode();
	LEUART_IntEnable(RS485_UART,LEUART_IF_TXBL);
	ret_val=1;
	}
	else {
		ret_val=-1;
	}
	return ret_val;
}

void rs485_transmit_string_new(char* data,uint8_t length){
	int loop_var=0;
	rs485_tx_mode();
	for(loop_var=0;loop_var<length;loop_var++){
		LEUART_Tx(RS485_UART,(uint8_t)data[loop_var] );
		delay_ms(0);
	}
	rs485_rx_mode();
	return;

}

int rs485_transmit_char(char data){
	int ret_val=0;
	if(!fifo_tbr_is_full(fifo_tbr_tx_cmd)){
		fifo_tbr_add(fifo_tbr_tx_cmd,data);
		rs485_tx_mode();
		LEUART_IntEnable(RS485_UART,LEUART_IF_TXBL);
		ret_val=1;
	}
	else{
		ret_val=-1;
	}
	return ret_val;
}

char rs485_recieve_char(void){
	char temp_char='@';
	if(!fifo_tbr_is_empty(current_rx_fifo)){
		temp_char=fifo_tbr_remove(current_rx_fifo);
	}
	return temp_char;
}
/*void change_rx_fifo_type(fifo_tbr_type_t rx_fifo_type){
	current_rx_fifo=rx_fifo_type;
	return;
}*/
void rs485_tx_mode(void){
	GPIO_PinOutSet(RS485_PORT, RS485_RE);
	GPIO_PinOutSet(RS485_PORT, RS485_DE);
	return;
}
void rs485_rx_mode(void){
	GPIO_PinOutClear(RS485_PORT, RS485_DE);
	GPIO_PinOutClear(RS485_PORT, RS485_RE);
	return;
}
/*
 * INT handlers
 */
void RS485_ISR(){
	uint32_t	int_mask=LEUART_IntGetEnabled(RS485_UART);
	 if (int_mask & LEUART_IF_TXBL){
		while(!fifo_tbr_is_empty(fifo_tbr_tx_cmd)){
			isr_rx_tx_char=fifo_tbr_remove(fifo_tbr_tx_cmd);
			LEUART_Tx(RS485_UART,(uint8_t)isr_rx_tx_char );
			delay_ms(0);
		}
		LEUART_IntDisable(RS485_UART,LEUART_IF_TXBL);
		rs485_rx_mode();
	  }
	 if (int_mask & LEUART_IF_RXDATAV){
		 isr_rx_tx_char=(char) LEUART_Rx(RS485_UART);
			 if(!fifo_tbr_is_full(current_rx_fifo)){
				 fifo_tbr_add(current_rx_fifo, isr_rx_tx_char);
			}
	 }
	 LEUART_IntClear(RS485_UART, int_mask);
}
