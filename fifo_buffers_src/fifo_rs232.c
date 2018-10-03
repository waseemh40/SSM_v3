/*
 * fifo_rs232.c
 *
 *  Created on: Apr 6, 2017
 *      Author: waseemh
 */



#include "../fifo_buffers_header/fifo_rs232.h"

/*
 * private variables
 */
static int fifo_rx_size=0;
static int fifo_tx_size=0;
static int fifo_rx_front=0;
static int fifo_tx_front=0;
static int fifo_rx_rear=0;
static int fifo_tx_rear=0;
	//Linear FIFOs
static char fifo_tx[FIFO_TX_RS232_SIZE];
static char fifo_rx[FIFO_RX_RS232_SIZE];
/*
 * public variables
 */

/*
 * public functions
 */

void fifo_rs232_init(void){
	fifo_rx_size=0;
	fifo_rx_front=0;
	fifo_rx_rear=0;
	fifo_tx_size=0;
	fifo_tx_front=0;
	fifo_tx_rear=0;
}

bool fifo_rs232_is_empty(fifo_rs232_type_t fifo_type){
	bool temp_flag;
	if (fifo_type==fifo_rx_data){
		if (fifo_rx_size==0) temp_flag=true;
		else temp_flag=false;
	}
	else {
		if (fifo_tx_size==0) temp_flag=true;
		else temp_flag=false;
	}
	return temp_flag;
}

bool fifo_rs232_is_full(fifo_rs232_type_t fifo_type){
	bool temp_flag;
	if (fifo_type==fifo_rx_data){
		if (fifo_rx_size==FIFO_RX_RS232_SIZE) temp_flag=true;
		else temp_flag=false;
	}
	else {
		if (fifo_tx_size==FIFO_TX_RS232_SIZE) temp_flag=true;
		else temp_flag=false;
	}
	return temp_flag;
}

char fifo_rs232_remove(fifo_rs232_type_t fifo_type){
	char temp_data;
	switch (fifo_type){
	case fifo_rx_data:{
		if (fifo_rx_size>0){
			temp_data=fifo_rx[fifo_rx_front];
			fifo_rx_front++;
			fifo_rx_size--;
			if(fifo_rx_front==FIFO_RX_RS232_SIZE){
				fifo_rx_front=0;
			}
		}
		else {
			temp_data=(char)0;
		}
	break;
	}
	case fifo_tx_data:{
		if (fifo_tx_size>0){
			temp_data=fifo_tx[fifo_tx_front];
			fifo_tx_front++;
			fifo_tx_size--;
			if(fifo_tx_front==FIFO_TX_RS232_SIZE){
				fifo_tx_front=0;
			}
		}
		else {
			temp_data=(char)0;
		}
	break;
	}
	}
	return temp_data;
}

void fifo_rs232_add(fifo_rs232_type_t fifo_type, char data){
	switch (fifo_type){
	case fifo_rx_data:{
		if (fifo_rx_size<FIFO_RX_RS232_SIZE){
			fifo_rx[fifo_rx_rear]=data;
			fifo_rx_rear++;
			fifo_rx_size++;
			if(fifo_rx_rear==FIFO_RX_RS232_SIZE){
				fifo_rx_rear=0;
			}
		}
	break;
	}

	case fifo_tx_data:{
		if (fifo_tx_size<FIFO_TX_RS232_SIZE){
			fifo_tx[fifo_tx_rear]=data;
			fifo_tx_rear++;
			fifo_tx_size++;
			if(fifo_tx_rear==FIFO_TX_RS232_SIZE){
				fifo_tx_rear=0;
			}
		}
	break;
	}
	}
}
