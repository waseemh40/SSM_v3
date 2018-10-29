/*
 * sd_card.c
 *
 *  Created on: Apr 19, 2017
 *      Author: waseemh
 */

#include "../devices_header/sd_card.h"

static	uint8_t 	sector_erase_size=0;
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
		spi_write_byte(0xff);//ARG_0
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
	case CMD_23:{
		spi_write_byte(0xFF);
		spi_write_byte(CMD_23);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(ARG_0);
		spi_write_byte(sector_erase_size);
		spi_write_byte(0xff);
		break;
	}
	default:{
		spi_write_byte(0xFF);
		break;
	}
	}
	return;
}


void set_sector_size(uint8_t size){
	uint8_t retry=0;
		//ACMD_23 and response
	start_transfer();
	send_cmd(CMD_55);
	retry=0;
	while(spi_read_write_byte(0xFF)==0xff){
		retry++;
		if(retry>500){
			end_transfer();
			return;
		}
	}
	end_transfer();
	sector_erase_size=size;
	start_transfer();
	send_cmd(CMD_23);
	retry=0;
	while(spi_read_write_byte(0xFF)==0xff){
		retry++;
		if(retry>500){
			end_transfer();
			return;
		}
	}
	end_transfer();
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
		//delay_ms(7);
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
	int 		outer_loop_var=0;
	int 		inner_loop_var=0;
	uint32_t	offset=0;
	uint8_t		reply=0;
	bool 		flag=true;
	uint32_t	retry=0;

	start_transfer();
	spi_write_byte(0xFF);
	spi_write_byte(CMD_18);
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
		if(outer_loop_var>=10){
			flag =false;
			break;
		}
	}
	offset=0;
	if(flag==true){
		for(outer_loop_var=0;outer_loop_var<scetor_count;outer_loop_var++){
			inner_loop_var=0;
			reply=0xFF;
			while(reply!=0xFE){
				reply=spi_read_write_byte(0xFF);
				inner_loop_var++;
				if(inner_loop_var>=100){
					flag =false;
					break;
					spi_write_byte(CMD_12);
					spi_write_byte((uint8_t)(ARG_0));
					spi_write_byte((uint8_t)(ARG_0));
					spi_write_byte((uint8_t)(ARG_0));
					spi_write_byte((uint8_t)(ARG_0));
					spi_write_byte(0xFF);	//CRC
					end_transfer();
					return flag;
				}
			}
				//actual packet
			for(inner_loop_var=0;inner_loop_var<SD_CARD_BLOCK_SIZE+2;inner_loop_var++){
				if(inner_loop_var<SD_CARD_BLOCK_SIZE){
				read_buf[inner_loop_var+offset]=(char)spi_read_write_byte(0xFF);
				}
				else{
					spi_read_write_byte(0xFF);	//discard CRC
				}
			}
			offset=SD_CARD_BLOCK_SIZE;
		}

		spi_write_byte(CMD_12);
		spi_write_byte((uint8_t)(ARG_0));
		spi_write_byte((uint8_t)(ARG_0));
		spi_write_byte((uint8_t)(ARG_0));
		spi_write_byte((uint8_t)(ARG_0));
		spi_write_byte(0xFF);	//CRC

			retry=0;
		while(spi_read_write_byte(0xFF)!=0x00){
			retry++;
			if(retry>500){
				flag=false;
				break;
			}
		}	//1-8 bytes and afterwards 00
		spi_read_write_byte(0xFF);
		retry=0;
		while(spi_read_write_byte(0xFF)!=0xFF){
			retry++;
			if(retry>500){
				flag=false;
				break;
			}
		}	//afterwards 0xff

	}
	end_transfer();
/*	for(inner_loop_var=0;inner_loop_var<scetor_count;inner_loop_var++){
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
			if(outer_loop_var==100){
				flag =false;
				break;
			}
		}
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
		addr++;
	}
	*/
	return flag;
}

bool sd_card_write(uint32_t addr, char *write_buf,uint32_t scetor_count){
	int 	outer_loop_var=0;
	int 	inner_loop_var=0;
	uint8_t	reply=0;
	bool 	flag=true;
	uint8_t	retry=0;

	set_sector_size((uint8_t)scetor_count);
	//for(inner_loop_var=0;inner_loop_var<scetor_count;inner_loop_var++){
		start_transfer();
		spi_write_byte(0xFF);
		spi_write_byte(CMD_25);
		spi_write_byte((uint8_t)(addr>>24));
		spi_write_byte((uint8_t)(addr>>16));
		spi_write_byte((uint8_t)(addr>>8));
		spi_write_byte((uint8_t)(addr>>0));
		spi_write_byte(0xFF);	//CRC
		reply=0xFF;
		outer_loop_var=0;
			//CMD response
		while(reply!=0x00){
			reply=spi_read_write_byte(0xFF);
			outer_loop_var++;
			if(outer_loop_var>=100){
				flag =false;
				break;
			}
		}
		//////////////////////
		spi_write_byte(0xFF);		//1 byte gap
		spi_write_byte(DATA_TKN_25);		//???
			//write data packet
		retry=0;
		if(flag==true){
			for(inner_loop_var=0;inner_loop_var<scetor_count;inner_loop_var++){
					//actual packet
				for(outer_loop_var=0;outer_loop_var<SD_CARD_BLOCK_SIZE+2;outer_loop_var++){
					spi_read_write_byte((uint8_t)write_buf[outer_loop_var]);
					}
					//repnose
				while(1){
					reply=spi_read_write_byte(0xFF);
					if(reply==0xFF){
						break;
					}
					else{
						retry++;
						if(retry>500){
							flag=false;
							break;
						}
					}
				}
			}
		}
		spi_write_byte(STOP_TOKEN);		//???
		spi_write_byte(0xFF);		//???
		retry=0;
		while(!spi_read_write_byte(0xFF)){
			retry++;
			if(retry>500){
				flag=false;
				break;
			}
		}
		end_transfer();
		//addr++;
	//}
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
		if(outer_loop_var>=10){
			flag =false;
			break;
		}
	}
	reply=0xFF;
	outer_loop_var=0;
	while(reply!=0xFE){
		reply=spi_read_write_byte(0xFF);
		outer_loop_var++;
		if(outer_loop_var>=10){
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
