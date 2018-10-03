/*
 * sd_card.c
 *
 *  Created on: Apr 19, 2017
 *      Author: waseemh
 */

#include "../devices_header/sd_card.h"

/*
 * private functions
 */
void start_transfer(void){
	spi_cs_clear(sd_card);
	return;
}

void end_transfer(void){
	spi_cs_set(sd_card);
	send_cmd(0xFF);
	return;
}

void send_cmd(uint8_t cmd){
	switch (cmd){
	case CMD_0:{
		spi_write_byte(0xFF);
		spi_write_byte(CMD_0);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(CRC_0);
		break;

	}
	case CMD_1:{
		spi_write_byte(0xFF);
		spi_write_byte(CMD_1);
		spi_write_byte(0x40);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		break;
	}
	case CMD_8:{
		spi_write_byte(0xFF);
		spi_write_byte(CMD_8);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_1);
		spi_write_byte(ARG_AA);
		spi_write_byte(0x87);
		break;
	}
	case CMD_16:{
		spi_write_byte(0xFF);
		spi_write_byte(CMD_16);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(0x02);
		spi_write_byte(ARG_0);
		spi_write_byte(0x87);
		break;
	}
	case CMD_41:{
		spi_write_byte(0xFF);
		spi_write_byte(CMD_41);
		spi_write_byte(0x40);	//0x40
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(0x41);	//Modification for Kingston from ARG_0...
		spi_write_byte(0x77);	//0x77 for 0x40
		break;
	}
	case CMD_55:{
		spi_write_byte(0xFF);
		spi_write_byte(CMD_55);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(0x65);
		break;
	}
	case CMD_58:{
		spi_write_byte(0xFF);
		spi_write_byte(CMD_58);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(CRC_58);
		break;
	}
	default:{
		spi_write_byte(0xFF);
		break;
	}
	}
	return;
}



/*
 * public functions
 */

bool sd_card_init(void){
	int 	outer_loop_var=0;
	int 	inner_loop_var=0;
	uint8_t	reply=0;
	GPIO_PinModeSet(PWR_EN_PORT,SD_CARD_PWR_EN,gpioModePushPull,1);
	sd_card_on();
	start_transfer();		//to set all other...
	end_transfer();
		//power on delay
	delay_ms(7);
		//80 dummy cycles
	for(outer_loop_var=0;outer_loop_var<10;outer_loop_var++){
		send_cmd(0xFF);
	}
		//CMD_0 and response
	start_transfer();
	send_cmd(CMD_0);
	reply=0xFF;
	while(reply!=0x01){
		reply=spi_read_write_byte(0xFF);
		outer_loop_var++;
		if(outer_loop_var>=100){
			end_transfer();
			return false;
			break;
		}
	}
	end_transfer();
		//CMD_8 and response
	start_transfer();
	send_cmd(CMD_8);
	reply=0xFF;
	outer_loop_var=0;
	while(reply!=0x01){
		reply=spi_read_write_byte(0xFF);
		outer_loop_var++;
		if(outer_loop_var>=100){
			end_transfer();
			return false;
			break;
		}
	}
	reply=spi_read_write_byte(0xFF);
	reply=spi_read_write_byte(0xFF);
	reply=spi_read_write_byte(0xFF);
	reply=spi_read_write_byte(0xFF);
	if (reply!=0xAA){
		end_transfer();
		return false;
	}
	end_transfer();
		//ACMD_41 and response
	reply=0xFF;
	while(reply!=0x00){
		start_transfer();
		send_cmd(CMD_55);
		reply=spi_read_write_byte(0xFF);		//Modification for Kingston...
		end_transfer();
		start_transfer();
		send_cmd(CMD_41);
		inner_loop_var=0;
		while(reply!=0x00){
			reply=spi_read_write_byte(0xFF);
			inner_loop_var++;
			if(inner_loop_var>3){break;}
		}
		delay_ms(7);
		outer_loop_var++;
		if(outer_loop_var>=100){
			end_transfer();
			return false;
			break;
		}
	}
	end_transfer();
		//CMD_58 and CCS bit
	start_transfer();
	send_cmd(CMD_58);
	reply=0xFF;
	outer_loop_var=0;
	while(reply!=0x00){
		reply=spi_read_write_byte(0xFF);
		outer_loop_var++;
		if(outer_loop_var>=100){
			end_transfer();
			return false;
			break;
		}
	}
	reply=spi_read_write_byte(0xFF);
	if (reply!=0xC0){
		end_transfer();
		return false;
	}
	reply=spi_read_write_byte(0xFF);
	reply=spi_read_write_byte(0xFF);
	reply=spi_read_write_byte(0xFF);
	end_transfer();
	return true;
}

void sd_card_on(void){
	GPIO_PinOutClear(PWR_EN_PORT,SD_CARD_PWR_EN);
	return;
}

void sd_card_off(void){
	GPIO_PinOutSet(PWR_EN_PORT,SD_CARD_PWR_EN);
	return;
}

bool sd_card_read(uint32_t addr, char *read_buf,uint32_t scetor_count){
	int 	outer_loop_var=0;
	int 	inner_loop_var=0;
	uint8_t	reply=0;
	bool 	flag=true;
	for(inner_loop_var=0;inner_loop_var<scetor_count;inner_loop_var++){
		start_transfer();
		spi_write_byte(0xFF);
		spi_write_byte(CMD_17);
		spi_write_byte((uint8_t)(addr>>24));
		spi_write_byte((uint8_t)(addr>>16));
		spi_write_byte((uint8_t)(addr>>8));
		spi_write_byte((uint8_t)(addr>>0));
		spi_write_byte(0xFF);	//CRC
		reply=0xFF;
		outer_loop_var=0;
		while(reply!=0x00){
			reply=spi_read_write_byte(0xFF);
			outer_loop_var++;
			if(outer_loop_var==10){
				flag =false;
				break;
			}
		}
		reply=0xFF;
		outer_loop_var=0;
		while(reply!=0xFE){
			reply=spi_read_write_byte(0xFF);
			outer_loop_var++;
			delay_ms(3);	//delay_ms(7);
			if(outer_loop_var==10){
				flag =false;
				break;
			}
		}
		delay_ms(3);	//7
		if (flag==true){
			for(outer_loop_var=0;outer_loop_var<SD_CARD_BLOCK_SIZE+2;outer_loop_var++){
				if(outer_loop_var<SD_CARD_BLOCK_SIZE){
				read_buf[outer_loop_var]=(char)spi_read_write_byte(0xFF);
				}
				else{
					spi_read_write_byte(0xFF);	//discard CRC
				}
			}
		}
		end_transfer();
		delay_ms(3);	//
		addr++;
	}
	/*	//CMD17 and response
	start_transfer();
	spi_write_byte(0xFF);
	spi_write_byte(CMD_17);
	spi_write_byte((uint8_t)(addr>>24));
	spi_write_byte((uint8_t)(addr>>16));
	spi_write_byte((uint8_t)(addr>>8));
	spi_write_byte((uint8_t)(addr>>0));
	spi_write_byte(0xFF);	//CRC
	reply=0xFF;
	outer_loop_var=0;
	while(reply!=0x00){
		reply=spi_read_write_byte(0xFF);
		outer_loop_var++;
		if(outer_loop_var==10){
			flag =false;
			break;
		}
	}
	reply=0xFF;
	outer_loop_var=0;
	while(reply!=0xFE){
		reply=spi_read_write_byte(0xFF);
		outer_loop_var++;
		delay_ms(7);
		if(outer_loop_var==10){
			flag =false;
			break;
		}
	}
	delay_ms(7);
	if (flag==true){
		for(outer_loop_var=0;outer_loop_var<SD_CARD_BLOCK_SIZE+2;outer_loop_var++){
			if(outer_loop_var<SD_CARD_BLOCK_SIZE){
			read_buf[outer_loop_var]=(char)spi_read_write_byte(0xFF);
			}
			else{
				spi_read_write_byte(0xFF);	//discard CRC
			}
		}
	}
	end_transfer();*/
	return flag;
}

bool sd_card_write(uint32_t addr, char *write_buf,uint32_t scetor_count){
	int 	outer_loop_var=0;
	int 	inner_loop_var=0;
	uint8_t	reply=0;
	bool 	flag=true;

	for(inner_loop_var=0;inner_loop_var<scetor_count;inner_loop_var++){
		start_transfer();
		spi_write_byte(0xFF);
		spi_write_byte(CMD_24);
		spi_write_byte((uint8_t)(addr>>24));
		spi_write_byte((uint8_t)(addr>>16));
		spi_write_byte((uint8_t)(addr>>8));
		spi_write_byte((uint8_t)(addr>>0));
		spi_write_byte(0xFF);	//CRC
		reply=0xFF;
		outer_loop_var=0;
		while(reply!=0x00){
			reply=spi_read_write_byte(0xFF);
			outer_loop_var++;
			if(outer_loop_var==10){
				flag =false;
				break;
			}
		}
		spi_write_byte(0xFF);		//1 byte gap
		spi_write_byte(0xFE);
		if(flag==true){
			for(outer_loop_var=0;outer_loop_var<SD_CARD_BLOCK_SIZE+2;outer_loop_var++){
				spi_read_write_byte((uint8_t)write_buf[outer_loop_var]);
				}
			while(1){
				reply=spi_read_write_byte(0xFF);
				if(reply==0xFF){
					break;
				}
				delay_ms(3);	//7
			}
		}
		end_transfer();
		addr++;
		delay_ms(3);		//7
	}
/*	start_transfer();
	spi_write_byte(0xFF);
	spi_write_byte(CMD_24);
	spi_write_byte((uint8_t)(addr>>24));
	spi_write_byte((uint8_t)(addr>>16));
	spi_write_byte((uint8_t)(addr>>8));
	spi_write_byte((uint8_t)(addr>>0));
	spi_write_byte(0xFF);	//CRC
	reply=0xFF;
	outer_loop_var=0;
	while(reply!=0x00){
		reply=spi_read_write_byte(0xFF);
		outer_loop_var++;
		if(outer_loop_var==10){
			flag =false;
			break;
		}
	}
	spi_write_byte(0xFF);		//1 byte gap
	spi_write_byte(0xFE);
	if(flag==true){
		for(outer_loop_var=0;outer_loop_var<SD_CARD_BLOCK_SIZE+2;outer_loop_var++){
			spi_read_write_byte((uint8_t)write_buf[outer_loop_var]);
			}
		while(1){
			reply=spi_read_write_byte(0xFF);
			if(reply==0xFF){
				break;
			}
			delay_ms(7);
		}
	}
	end_transfer();*/
	return flag;
}

bool sd_card_specs(char *csd){
	int 	outer_loop_var=0;
	uint8_t	reply=0;
	bool 	flag=true;
		//CMD17 and response
	start_transfer();
	spi_write_byte(0xFF);
	spi_write_byte(CMD_9);
	spi_write_byte(0x00);
	spi_write_byte(0x00);
	spi_write_byte(0x00);
	spi_write_byte(0x00);
	spi_write_byte(0xFF);	//CRC
	reply=0xFF;
	outer_loop_var=0;
	while(reply!=0x00){
		reply=spi_read_write_byte(0xFF);
		outer_loop_var++;
		if(outer_loop_var==10){
			flag =false;
			break;
		}
	}
	reply=0xFF;
	outer_loop_var=0;
	while(reply!=0xFE){
		reply=spi_read_write_byte(0xFF);
		outer_loop_var++;
		if(outer_loop_var==10){
			flag =false;
			break;
		}
	}
	if (flag==true){
		for(outer_loop_var=0;outer_loop_var<18;outer_loop_var++){
			csd[outer_loop_var]=(char)spi_read_write_byte(0xFF);
		}
	}
	end_transfer();
	return flag;
}
