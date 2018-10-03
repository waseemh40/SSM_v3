/*
 * fifo_tbr.h
 *
 *  Created on: Mar 29, 2017
 *      Author: waseemh
 */

#ifndef SRC_FIFO_TBR_H_
#define SRC_FIFO_TBR_H_

#include <stdlib.h>
#include "../drivers_header/pinmap.h"


#define 	FIFO_TBR_RX_DATA_SIZE		1024
#define 	FIFO_TBR_RX_CMD_SIZE		0
#define 	FIFO_TBR_TX_CMD_SIZE		16

typedef enum{
	fifo_tbr_rx_data=0,
	fifo_tbr_rx_cmd,
	fifo_tbr_tx_cmd
}fifo_tbr_type_t;

void 		fifo_tbr_init(void);
void 		fifo_tbr_add(fifo_tbr_type_t fifo_type,char data);
bool 		fifo_tbr_is_empty(fifo_tbr_type_t fifo_type);
bool 		fifo_tbr_is_full(fifo_tbr_type_t fifo_type);
char 		fifo_tbr_remove(fifo_tbr_type_t fifo_type);

#endif /* SRC_FIFO_TBR_H_ */
