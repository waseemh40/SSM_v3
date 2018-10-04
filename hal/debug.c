/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/


#include "debug.h"
#include "../lmic/lmic.h"
#include "../drivers_header/rs232.h"
#include "../devices_header/display.h"

void debug_init ()
{
	rs232_init();
	rs232_enable();
	// print banner
    debug_str((unsigned char*)"\r\n============= DEBUG STARTED =============\r\n");
}

void debug_led (u1_t val)
{
	if (val)
		rgb_on(false,true,true);
	else
		rgb_shutdown();
}

void debug_char (u1_t c)
{
	rs232_transmit_char( c);
	delay_ms(0);
}

void debug_hex (u1_t b) {
    debug_char("0123456789ABCDEF"[b>>4]);
    debug_char("0123456789ABCDEF"[b&0xF]);
}

void debug_buf (const u1_t* buf, u2_t len) {
    while(len--) {
        debug_hex(*buf++);
        debug_char(' ');
    }
    debug_char('\r');
    debug_char('\n');
}

void debug_uint (u4_t v) {
    for(s1_t n=24; n>=0; n-=8) {
        debug_hex(v>>n);
    }
}

void debug_str (const u1_t* str) {
	rs232_transmit_string(str,(uint8_t)strlen((char *)str));
	delay_ms(2);
}

void debug_val (const u1_t* label, u4_t val) {
    debug_str(label);
    debug_uint(val);
    debug_char('\r');
    debug_char('\n');
}

void debug_event (int ev) {
	//char buf[32];
    static const char* evnames[] = {
        [EV_SCAN_TIMEOUT]   = "SCAN_TIMEOUT",
        [EV_BEACON_FOUND]   = "BEACON_FOUND",
        [EV_BEACON_MISSED]  = "BEACON_MISSED",
        [EV_BEACON_TRACKED] = "BEACON_TRACKED",
        [EV_JOINING]        = "JOINING",
        [EV_JOINED]         = "JOINED",
        [EV_RFU1]           = "RFU1",
        [EV_JOIN_FAILED]    = "JOIN_FAILED",
        [EV_REJOIN_FAILED]  = "REJOIN_FAILED",
        [EV_TXCOMPLETE]     = "TXCOMPLETE",
        [EV_LOST_TSYNC]     = "LOST_TSYNC",
        [EV_RESET]          = "RESET",
        [EV_RXCOMPLETE]     = "RXCOMPLETE",
        [EV_LINK_DEAD]      = "LINK_DEAD",
        [EV_LINK_ALIVE]     = "LINK_ALIVE",
    };
    //sprintf(buf,"\tev=%d %s\t\n",ev,evnames[ev]);
    debug_str(evnames);
}
