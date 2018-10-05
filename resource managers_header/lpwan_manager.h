/*
 * lpwan_manager.h
 *
 *  Created on: Jun 29, 2017
 *      Author: waseemh
 */

#ifndef RESOURCE_MANAGERS_HEADER_LPWAN_MANAGER_H_
#define RESOURCE_MANAGERS_HEADER_LPWAN_MANAGER_H_

#include "../resource managers_header/app_manager.h"
#include "../lmic/lmic.h"
#include "../hal/debug.h"
#include "../lmic/hal.h"
#include "em_rmu.h"
		/*
		 * Node parameters
		 */
	// LoRaWAN Application identifier (0xLSB, 0xxx, ......, 0xMSB)
static const u1_t APPEUI[8]  = {  0x88,	0x99,	0x11,	0x55,	0x44,	0x22,	0x11,	0x00};
//static const u1_t APPEUI[8]  = { 0x46,	0x59,	0x00,	0xF0,	0x7E,	0xD5,	0xB3,	0x70};

	// LoRaWAN DevEUI, unique device ID (0xLSB, 0xxx, ......, 0xMSB)
static const u1_t DEVEUI[8]  = { 0x00,	0x85,	0x00,	0x00,	0x00,	0x00,	0x78,	0x82};
//static const u1_t DEVEUI[8]  = { 0x46,	0x59,	0x00,	0xF0,	0x7E,	0xD5,	0xB3,	0x70};

	//DEVKEY (0xMSB, 0xxx, ......, 0xLSB i.e. normal format)
static const u1_t DEVKEY[16] = {0x00,	0x11,	0x00,	0x22,	0x00,	0x33,	0x00,	0x44,	0x00,	0x55,	0x00,	0x66,	0x00,	0x77,	0x00,	0x88};
//static const u1_t DEVKEY[16] = {0x9C,	0x6F,	0xC7,	0x34,	0xA1,	0x22,	0x4C,	0x54,	0x15,	0xFB,	0x78,	0x09,	0x90,	0x83,	0xA3,	0x6E};

		/*
		 * public variables
		 */
extern	uint8_t 		lora_buffer[512];
extern	uint8_t			lora_msg_length;
void schedule_user_job(void);
void register_user_callback(void (*user_cb)(void));
		/*
		 * public functions
		 */
void 	lpwan_init(void);
void 	onEvent (ev_t ev);

#endif /* RESOURCE_MANAGERS_HEADER_LPWAN_MANAGER_H_ */
