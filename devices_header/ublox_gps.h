/*
 * ublox_gps.h
 *
 *  Created on: Apr 25, 2017
 *      Author: waseemh
 */

#ifndef SRC_UBLOX_GPS_H_
#define SRC_UBLOX_GPS_H_

#include "../drivers_header/spi.h"
#include "../drivers_header/delay.h"
#include "../drivers_header/rs232.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ublox_msg.h"
/*
 * Macros
 */
#define 	RETRY	25
/*
 * structs and typedefs
 */
typedef struct {
	bool		valid;
	uint32_t	gps_timestamp;
	uint16_t	year;
	uint8_t		month;
	uint8_t		day;
	uint8_t		hour;
	uint8_t		min;
	uint8_t		sec;
	uint8_t		t_flags;	//added later on
	uint32_t	tAcc;		//added later on
	uint32_t	nano;		//added later on
	uint8_t		numSV;		//added later on
	uint8_t		pDOP;		//added later later on
	uint32_t	longitude;
	uint32_t	latitude;
	uint32_t	height;
	uint8_t 	fix_type;

}nav_data_t;

typedef struct
{
	uint32_t iTOW;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t valid;
	uint32_t tAcc;
	int32_t nano;
	uint8_t fixType;
	uint8_t flags;
	uint8_t flags2;
	uint8_t numSv;
	int32_t lon;
	int32_t lat;
	int32_t height;
	int32_t hMSL;
	uint32_t hAcc;
	uint32_t vAcc;
	int32_t velN;
	int32_t velE;
	int32_t velD;
	int32_t gSpeed;
	int32_t headMot;
	uint32_t sAcc;
	uint32_t headAcc;
	uint16_t pDOP;
	uint8_t reserved1[6];
	int32_t headVeh;
	int16_t magDec;
	uint16_t magAcc;
} nav_pvt_t;

typedef struct
{
	uint8_t gnssId;
	uint8_t svId;
	uint8_t cno;
	int8_t elev;
	int16_t azim;
	int16_t prRes;
	uint32_t flags;
} nav_sat_satellites_t;

typedef struct
{
	uint32_t iTOW;
	uint8_t version;
	uint8_t numSvs;
	uint8_t reserved1[2];
	nav_sat_satellites_t nav_sat_satellites[25];
} nav_sat_t;

#define GPS_PORT_ID_DDC 0
#define GPS_PORT_ID_UART 1
#define GPS_PORT_ID_USB 3
#define GPS_PORT_ID_SPI 4

typedef struct
{
	uint8_t PortID;
	uint8_t reserved1;
	uint16_t txReady;
	uint32_t mode;
	uint32_t baudRate;
	uint16_t inProtoMask;
	uint16_t outProtoMask;
	uint16_t flags;
	uint8_t reserved2[2];
} cfg_prt_uart_t;

typedef struct
{
	uint8_t PortID;
	uint8_t reserved1;
	uint16_t txReady;
	uint8_t reserved2[8];
	uint16_t inProtoMask;
	uint16_t outProtoMask;
	uint8_t reserved3[2];
	uint8_t reserved4[2];
} cfg_prt_usb_t;

typedef struct
{
	uint8_t PortID;
	uint8_t reserved1;
	uint16_t txReady;
	uint32_t mode;
	uint8_t reserved2[4];
	uint16_t inProtoMask;
	uint16_t outProtoMask;
	uint16_t flags;
	uint8_t reserved3[2];
} cfg_prt_spi_t;

typedef struct
{
	uint8_t PortID;
	uint8_t reserved1;
	uint16_t txReady;
	uint32_t mode;
	uint8_t reserved2[4];
	uint16_t inProtoMask;
	uint16_t outProtoMask;
	uint16_t flags;
	uint8_t reserved3[2];
} cfg_prt_ddc_t;

typedef struct
{
	uint8_t version;
	uint8_t reserved1;
	uint8_t maxStartupStateDur;
	uint8_t reserved2;
	uint32_t flags;
	uint32_t updatePeriod;
	uint32_t searchPeriod;
	uint32_t gridOffset;
	uint16_t onTime;
	uint16_t minAcqTime;
	uint8_t reserved3[20];
} cfg_pm2_t;

typedef struct
{
	uint8_t version;
	uint8_t powerSetupValue;
	uint16_t period;
	uint16_t onTime;
	uint8_t reserved1[2]
} cfg_pms_t;

typedef struct
{
	uint8_t reserved1;
	uint8_t lpMode;
} cfg_rxm_t;

typedef struct
{
	uint32_t clearMask;
	uint32_t saveMask;
	uint32_t loadMask;
} cfg_cfg_t;

typedef struct
{
	uint8_t mode;
	uint8_t usage;
	uint8_t maxSBAS;
	uint8_t scanmode2;
	uint32_t scanmode1;
} cfg_sbas_t;

/*
 * public variables
 */
/*
 * private functions
 */
bool			send_cmd_rx_ack(uint8_t const *cmd,uint8_t size_cmd);
bool 			port_config(void);
bool			disbale_sbas(void);
bool			config_low_power(void);
bool			enter_low_power(void);
bool			poll_psm(void);
uint8_t 		receiver_nav_status(void);
uint16_t		fletcher16( uint8_t const *data, size_t bytes );
nav_data_t		parse_message(uint8_t data[]);

void gps_read_and_parse(void);
uint64_t gps_get_navigation_data(nav_pvt_t** data);
uint64_t gps_get_navsat_data(nav_sat_t** data);
bool gps_send_cmd(uint8_t class, uint8_t msg_id, uint16_t length, uint8_t* payload, bool wait_for_ack);

/*
 * public functions
 */
void 			gps_int_pin_toggle(void);
void 			gps_on(void);
void 			gps_off(void);
bool			gps_init(void);
bool			gps_status(void);
bool 			gps_low_power(void);
nav_data_t 		gps_get_nav_data(void);
void gps_sw_backup(void);
#endif /* SRC_UBLOX_GPS_H_ */
