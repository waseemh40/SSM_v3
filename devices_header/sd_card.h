/*
 * sd_card.h
 *
 *  Created on: Apr 19, 2017
 *      Author: waseemh
 */

#ifndef SRC_SD_CARD_H_
#define SRC_SD_CARD_H_

#include "../drivers_header/spi.h"
#include "../drivers_header/delay.h"

/*
 * macros for sd card
 */
	//command list
#define 	CMD_0		0x40
#define 	CMD_1		0x41
#define 	CMD_8		0x48
#define 	CMD_9		0x49
#define 	CMD_10		0x4A
#define 	CMD_12		0x4C
#define 	CMD_16		0x50
#define 	CMD_17		0x51
#define 	CMD_18		0x52
#define 	CMD_23		0x57
#define 	CMD_24		0x58
#define 	CMD_25		0x59
#define 	CMD_41		0x69
#define 	CMD_55		0x77
#define 	CMD_58		0x7A
	//arguments
#define 	ARG_0		0x00
#define 	ARG_1		0x01
#define 	ARG_AA		0xAA
	//CRC for commands
#define 	CRC_0		0x95
#define 	CRC_8		0x0F
#define 	CRC_58		0x75

	//tokens
#define		DATA_TKN_24	0xFE
#define		DATA_TKN_25	0xFC
#define		STOP_TOKEN	0xFD

#define 	SD_CARD_BLOCK_SIZE	512
/*
 * private variables
 */

/*
 * public variables
 */

/*private functions
 *
 */
void 	send_cmd(uint8_t cmd);
void 	start_transfer(void);
void 	end_transfer(void);
/*
 * public functions
 */
bool	sd_card_init(void);
void 	sd_card_on(void);
void 	sd_card_off(void);
bool 	sd_card_write(uint32_t addr, char *write_buf,uint32_t sector_count);
bool 	sd_card_read(uint32_t addr, char *read_buf,uint32_t sector_count);
bool	sd_card_specs(char *csd);

#endif /* SRC_SD_CARD_H_ */
