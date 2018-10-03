/*
 * fifo_tbr.c
 *
 *  Created on: Mar 29, 2017
 *      Author: waseemh
 */

#include "../fifo_buffers_header/fifo_tbr.h"

/*
 * private variables
 */
static int fifo_rx_data_size=0;
static int fifo_rx_cmd_size=0;
static int fifo_tx_cmd_size=0;
static int fifo_rx_cmd_front=0;
static int fifo_tx_cmd_front=0;
static int fifo_rx_data_front=0;
static int fifo_rx_cmd_rear=0;
static int fifo_tx_cmd_rear=0;
static int fifo_rx_data_rear=0;
	//Linear FIFOs
static char fifo_tx_cmd[FIFO_TBR_TX_CMD_SIZE];
static char fifo_rx_cmd[FIFO_TBR_RX_CMD_SIZE];
static char fifo_rx_data[FIFO_TBR_RX_DATA_SIZE];
/*
 * public variables
 */

/*
 * public functions
 */

void fifo_tbr_init(void){
	fifo_rx_data_size=0;
	fifo_rx_data_front=0;
	fifo_rx_data_rear=0;
	fifo_rx_cmd_size=0;
	fifo_rx_cmd_front=0;
	fifo_rx_cmd_rear=0;
	fifo_tx_cmd_size=0;
	fifo_tx_cmd_front=0;
	fifo_tx_cmd_rear=0;
}

bool fifo_tbr_is_empty(fifo_tbr_type_t fifo_type){
	bool temp_flag;
	if (fifo_type==fifo_tbr_rx_data){
		if (fifo_rx_data_size==0) temp_flag=true;
		else temp_flag=false;
	}
	else if (fifo_type==fifo_tbr_rx_cmd){
		if (fifo_rx_cmd_size==0) temp_flag=true;
		else temp_flag=false;
	}
	else {
		if (fifo_tx_cmd_size==0) temp_flag=true;
		else temp_flag=false;
	}
	return temp_flag;
}

bool fifo_tbr_is_full(fifo_tbr_type_t fifo_type){
	bool temp_flag;
	if (fifo_type==fifo_tbr_rx_data){
		if (fifo_rx_data_size==FIFO_TBR_RX_DATA_SIZE) temp_flag=true;
		else temp_flag=false;
	}
	else if (fifo_type==fifo_tbr_rx_cmd){
		if (fifo_rx_cmd_size==FIFO_TBR_RX_CMD_SIZE) temp_flag=true;
		else temp_flag=false;
	}
	else if(fifo_tbr_tx_cmd){
		if (fifo_tx_cmd_size==FIFO_TBR_TX_CMD_SIZE) temp_flag=true;
		else temp_flag=false;
	}
	return temp_flag;
}

char fifo_tbr_remove(fifo_tbr_type_t fifo_type){
	char temp_data;
	switch (fifo_type){
	case fifo_tbr_rx_data:{
		if (fifo_rx_data_size>0){
			temp_data=fifo_rx_data[fifo_rx_data_front];
			fifo_rx_data_front++;
			fifo_rx_data_size--;
			if(fifo_rx_data_front==FIFO_TBR_RX_DATA_SIZE){
				fifo_rx_data_front=0;
			}
		}
		else {
			temp_data=(char)0;
		}
	break;
	}
	case fifo_tbr_rx_cmd:{
		if (fifo_rx_cmd_size>0){
			temp_data=fifo_rx_cmd[fifo_rx_cmd_front];
			fifo_rx_cmd_front++;
			fifo_rx_cmd_size--;
			if(fifo_rx_cmd_front==FIFO_TBR_RX_CMD_SIZE){
				fifo_rx_cmd_front=0;
			}
		}
		else {
			temp_data=(char)0;
		}
	break;
	}
	case fifo_tbr_tx_cmd:{
		if (fifo_tx_cmd_size>0){
			temp_data=fifo_tx_cmd[fifo_tx_cmd_front];
			fifo_tx_cmd_front++;
			fifo_tx_cmd_size--;
			if(fifo_tx_cmd_front==FIFO_TBR_TX_CMD_SIZE){
				fifo_tx_cmd_front=0;
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

void fifo_tbr_add(fifo_tbr_type_t fifo_type, char data){
	switch (fifo_type){
	case fifo_tbr_rx_data:{
		if (fifo_rx_data_size<FIFO_TBR_RX_DATA_SIZE){
			fifo_rx_data[fifo_rx_data_rear]=data;
			fifo_rx_data_rear++;
			fifo_rx_data_size++;
			if(fifo_rx_data_rear==FIFO_TBR_RX_DATA_SIZE){
				fifo_rx_data_rear=0;
			}
		}
	break;
	}
	case fifo_tbr_rx_cmd:{
		if (fifo_rx_cmd_size<FIFO_TBR_RX_CMD_SIZE){
			fifo_rx_cmd[fifo_rx_cmd_rear]=data;
			fifo_rx_cmd_rear++;
			fifo_rx_cmd_size++;
			if(fifo_rx_cmd_rear==FIFO_TBR_RX_CMD_SIZE){
				fifo_rx_cmd_rear=0;
			}
		}
	break;
	}
	case fifo_tbr_tx_cmd:{
		if (fifo_tx_cmd_size<FIFO_TBR_TX_CMD_SIZE){
			fifo_tx_cmd[fifo_tx_cmd_rear]=data;
			fifo_tx_cmd_rear++;
			fifo_tx_cmd_size++;
			if(fifo_tx_cmd_rear==FIFO_TBR_TX_CMD_SIZE){
				fifo_tx_cmd_rear=0;
			}
		}
	break;
	}
	}
}
