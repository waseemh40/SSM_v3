/*
 * ublox_gps.c
 *
 *  Created on: Apr 25, 2017
 *      Author: waseemh
 */
#include "../devices_header/ublox_gps.h"
#include "../devices_header/tick.h"

cfg_rxm_t cfg_rxm_data;
uint64_t cfg_rxm_data_updated_ticks = 0;

nav_pvt_t nav_pvt_data;
uint64_t nav_pvt_data_updated_ticks = 0;
uint64_t gps_get_navigation_data(nav_pvt_t** data)
{
	uint64_t timestamp = nav_pvt_data_updated_ticks;
	nav_pvt_data_updated_ticks = 0;

	*data = &nav_pvt_data;

	return timestamp;
}

nav_sat_t nav_sat_data;
uint64_t nav_sat_data_updated_ticks = 0;
uint64_t gps_get_navsat_data(nav_sat_t** data)
{
	uint64_t timestamp = nav_sat_data_updated_ticks;
	nav_sat_data_updated_ticks = 0;

	*data = &nav_sat_data;

	return timestamp;
}

#define GPS_PARSE_BUFFER_SIZE 512
uint8_t gps_parse_buffer[GPS_PARSE_BUFFER_SIZE];
uint32_t gps_parse_buffer_cnt = 0;

#define GET_LE16(x)	(((uint16_t)(((uint8_t*)x)[0])<<0) | \
						((uint16_t)(((uint8_t*)x)[1])<<8))
#define GET_LE32(x)	(((uint32_t)(((uint8_t*)x)[0])<<0) | \
						((uint32_t)(((uint8_t*)x)[1])<<8) | \
						((uint32_t)(((uint8_t*)x)[2])<<16) | \
						((uint32_t)(((uint8_t*)x)[3])<<24))

void gps_parse_ubx(uint8_t* data, uint32_t count);

uint8_t gps_ack_class = 0;
uint8_t gps_ack_msg_id = 0;
bool gps_ack_received = false;

#define GPS_BUFFER_SIZE 1024
uint8_t gps_buffer[GPS_BUFFER_SIZE];
uint32_t gps_buffer_rd = 0;
uint32_t gps_buffer_wr = 0;
uint32_t gps_buffer_cnt = 0;

static bool gps_buffer_add(uint8_t data)
{
	if (gps_buffer_cnt >= GPS_BUFFER_SIZE)
		return false;

	gps_buffer[gps_buffer_wr++] = data;
	if (gps_buffer_wr >= GPS_BUFFER_SIZE)
		gps_buffer_wr = 0;
	gps_buffer_cnt++;

	return true;
}

static bool gps_buffer_get(uint8_t* data)
{
	if (gps_buffer_cnt == 0)
		return false;

	*data = gps_buffer[gps_buffer_rd++];
	if (gps_buffer_rd >= GPS_BUFFER_SIZE)
		gps_buffer_rd = 0;
	gps_buffer_cnt--;

	return true;
}

void gps_parse(void)
{
	uint8_t data;
	while (gps_buffer_get(&data))
	{
		gps_parse_buffer[gps_parse_buffer_cnt++] = data;

		if (gps_parse_buffer[0] == '$') // NMEA
		{
			// Reset parsing in case of invalid character
			if (data > 0x80)
				gps_parse_buffer_cnt = 0;

			// Parse when encountering <LF> = \n
			if (data == (uint8_t)'\n')
			{
				// parse_nmea(gps_parse_buffer, gps_parse_buffer_cnt);
				gps_parse_buffer_cnt = 0;
			}
		} else if (gps_parse_buffer[0] == SYNCH_1) // UBX
		{
			if (gps_parse_buffer_cnt >= 6)
			{
				uint16_t length = GET_LE16(&gps_parse_buffer[4]);
				if (gps_parse_buffer_cnt == (8 + length)) // Header + payload + checksum
				{
					gps_parse_ubx(gps_parse_buffer, gps_parse_buffer_cnt);
					gps_parse_buffer_cnt = 0;
				}
			}
		} else
		{
			gps_parse_buffer_cnt = 0;
		}
	}
}

bool gps_wait_for_ack(uint8_t class, uint8_t msg_id, uint32_t timeout)
{
	gps_ack_class = 0;
	gps_ack_msg_id = 0;
	gps_ack_received = false;

	uint64_t start_time = tick_get();
	while (tick_get() < (start_time + timeout))
	{
		gps_read_and_parse();

		if (gps_ack_received && (gps_ack_class == class) && (gps_ack_msg_id == msg_id))
			return true;
	}

	return false;
}

void gps_parse_ubx(uint8_t* data, uint32_t count)
{
	if ((data[0] != SYNCH_1) || (data[1] != SYNCH_2))
		return;

	// Parse header
	uint8_t class = data[2];
	uint8_t msg_id = data[3];
	uint16_t length = GET_LE16(&data[4]);

	// Control checksum, ignore if mismatch
	if (fletcher16(&data[2], count-4) != (data[count-2] | (data[count-1]<<8)))
		return;

	if (class == ACK)
	{
		if (msg_id == 0x01)
		{
			gps_ack_class = data[6];
			gps_ack_msg_id = data[6 + 1];
			gps_ack_received = true;
		}
	} else if (class == MON)
	{
		if (msg_id == VER)
		{
			for (volatile int i = 0; i < 10; i++);
		}
	} else if (class == CFG)
	{
		if (msg_id == RXM_CFG)
		{
			for (int i = 0; i < length; i++)
				((uint8_t*)&cfg_rxm_data)[i] = data[6 + i];

			cfg_rxm_data_updated_ticks = tick_get();
		}
	} else if (class == NAV)
	{
		if (msg_id == PVT)
		{
			for (int i = 0; i < length; i++)
				((uint8_t*)&nav_pvt_data)[i] = data[6 + i];

			nav_pvt_data_updated_ticks = tick_get();
		} else if (msg_id == SAT)
		{
			for (int i = 0; i < length; i++)
				((uint8_t*)&nav_sat_data)[i] = data[6 + i];

			nav_sat_data_updated_ticks = tick_get();
		}
	}
}

void gps_read(uint32_t count)
{
	spi_cs_clear(gps);
	delay_ms(1);

	for (int i = 0; i < count; i++)
		gps_buffer_add(spi_read_byte());

	spi_cs_set(gps);
	delay_ms(1);
}

void gps_write(uint8_t* data, uint32_t count)
{
	spi_cs_clear(gps);
	delay_ms(1);

	for (int i = 0; i < count; i++)
		gps_buffer_add(spi_read_write_byte(data[i]));

	spi_cs_set(gps);
	delay_ms(1);
}

void gps_read_and_parse(void)
{
	gps_read(100);
	gps_parse();

	while (gps_parse_buffer_cnt != 0)
	{
		gps_read(100);
		gps_parse();
	}
}

bool gps_send_cmd(uint8_t class, uint8_t msg_id, uint16_t length, uint8_t* payload, bool wait_for_ack)
{
	uint8_t gps_write_buffer[1024];

	gps_write_buffer[0] = SYNCH_1;
	gps_write_buffer[1] = SYNCH_2;

	gps_write_buffer[2] = class;
	gps_write_buffer[3] = msg_id;

	gps_write_buffer[4] = length&0xFF;
	gps_write_buffer[5] = (length>>8)&0xFF;

	for (int i = 0; i < length; i++)
		gps_write_buffer[6 + i] = payload[i];

	uint16_t checksum = fletcher16(&gps_write_buffer[2], length + 4);

	gps_write_buffer[6 + length] = checksum&0xFF;
	gps_write_buffer[6 + length + 1] = (checksum>>8)&0xFF;

	gps_write(gps_write_buffer, 6 + length + 2);

	if (!wait_for_ack)
		return true;
	return gps_wait_for_ack(class, msg_id, 100);
}

uint16_t fletcher16(uint8_t const *data, size_t size)
{
	uint16_t crc_a = 0;
	uint16_t crc_b = 0;

    while (size--)
    {
    	crc_a += *(data++);
    	crc_b += crc_a;
    }

    crc_a &= 0xff;
    crc_b &= 0xff;

	return (crc_a | (crc_b << 8));
}

bool port_config(void)
{
	bool flag_spi = false;
	bool flag_usb = false;
	bool flag_ddc = false;

	// SPI port
	cfg_prt_spi_t cfg_prt_spi = {
			.PortID = GPS_PORT_ID_SPI,
			.reserved1 = 0,
			.txReady = 0,
			.mode = 50 << 8,
			.reserved2 = {0, 0, 0, 0},
			.inProtoMask = 1, // inUbx
			.outProtoMask = 1, // outUbx
			.flags = 0,
			.reserved2 = {0, 0}
	};
	flag_spi = gps_send_cmd(CFG, PRT, sizeof(cfg_prt_spi_t), (uint8_t*)&cfg_prt_spi, true);

	// USB port
	cfg_prt_usb_t cfg_prt_usb = {
			.PortID = GPS_PORT_ID_USB,
			.reserved1 = 0,
			.txReady = 0,
			.reserved2 = {0, 0},
			.inProtoMask = 0,
			.outProtoMask = 0,
			.reserved3 = {0, 0},
			.reserved4 = {0, 0}
	};
	flag_usb = gps_send_cmd(CFG, PRT, sizeof(cfg_prt_usb_t), (uint8_t*)&cfg_prt_usb, true);

	// DDC port
	cfg_prt_ddc_t cfg_prt_ddc = {
			.PortID = GPS_PORT_ID_DDC,
			.reserved1 = 0,
			.txReady = 0,
			.mode = 0,
			.reserved2 = {0, 0, 0, 0},
			.inProtoMask = 0,
			.outProtoMask = 0,
			.flags = 0,
			.reserved3 = {0, 0}
	};
	flag_ddc = gps_send_cmd(CFG, PRT, sizeof(cfg_prt_ddc_t), (uint8_t*)&cfg_prt_ddc, true);

	if (flag_spi && flag_usb && flag_ddc)
		return true;

	return false;
}

bool config_low_power(void)
{
	cfg_pm2_t cfg_pm2 = {
			.version = 1,
			.reserved1 = 0,
			.maxStartupStateDur = 0,
			.reserved2 = 0,
			.flags = (1<<17) | (1<<11) | (1<<10), //
			.updatePeriod = 10000,
			.searchPeriod = 60000,
			.gridOffset = 0,
			.onTime = 0,
			.minAcqTime = 5,
			.reserved3 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
	};

	return gps_send_cmd(CFG, PM2, sizeof(cfg_pm2_t), (uint8_t*)&cfg_pm2, true);
}

bool enter_low_power(void)
{
	cfg_rxm_t cfg_rxm = {
			.reserved1 = 0x08,
			.lpMode = 1
	};

	return gps_send_cmd(CFG, RXM_CFG, sizeof(cfg_rxm_t), (uint8_t*)&cfg_rxm, true);
}
bool disable_sbas(void)
{
	cfg_sbas_t cfg_sbas = {
			.mode = 0,
			.usage = 0,
			.maxSBAS = 0,
			.scanmode2 = 0,
			.scanmode1 = 0
	};

	return gps_send_cmd(CFG, SBAS, sizeof(cfg_sbas_t), (uint8_t*)&cfg_sbas, true);
}

bool gps_init(void)
{
	spi_init();
	GPIO_PinModeSet(GPS_SIG_PORT, GPS_INT, gpioModePushPull,0);
					//configure ports
	bool flag_port = port_config();

	gps_send_cmd(MON, VER, 0, NULL, false);
	gps_read_and_parse();

	cfg_pms_t cfg_pms_off = {
		.version = 0,
		.powerSetupValue = 0,
		.period = 0,
		.onTime = 0,
		.reserved1 = {0,0}
	};
	volatile bool check = gps_send_cmd(CFG, PMS, sizeof(cfg_pms_t), (uint8_t*)&cfg_pms_off, true);

	nav_pvt_data_updated_ticks = 0;
	uint8_t fix_type = 0;
	while (fix_type != 3)
	{
		// Request new navigation data
		gps_send_cmd(NAV, PVT, 0, NULL, false);

		uint64_t start_time = tick_get();
		while ((tick_get()-start_time) < 6000)
		{
			gps_read_and_parse();

			if (nav_pvt_data_updated_ticks != 0)
			{
				nav_pvt_data_updated_ticks = 0;
				fix_type = nav_pvt_data.fixType;
				break;
			}
		}
	}
/*
	// Configure and enter low power
	bool flag_enter_lp = enter_low_power();
	bool flag_config_lp = config_low_power();
	bool flag_sbas = disable_sbas();

	cfg_pms_t cfg_pms = {
		.version = 0,
		.powerSetupValue = 2,
		.period = 60,
		.onTime = 5,
		.reserved1 = {0,0}
	};
	check = gps_send_cmd(CFG, PMS, sizeof(cfg_pms_t), (uint8_t*)&cfg_pms, true);

	cfg_cfg_t cfg_cfg = {
			.clearMask = 0,
			.saveMask = 1<<4,
			.loadMask = 0
	};

	volatile bool check2 = gps_send_cmd(CFG, CFG_CFG, sizeof(cfg_cfg_t), (uint8_t*)&cfg_cfg, true);

	//grid search delay....
	//delay_ms(250);

	// Poll PSM and exit if OK
	for(int i = 0; i < 60; i++)
	{
		enter_low_power();

		cfg_rxm_data_updated_ticks = 0;
		gps_send_cmd(CFG, RXM_CFG, 0, NULL, false);

		gps_read_and_parse();
		if ((cfg_rxm_data_updated_ticks != 0) &&
				(cfg_rxm_data.lpMode == 1))
			break;

		delay_ms(10);
	 }*/

	return flag_port; // && flag_config_lp && flag_sbas && flag_enter_lp;
}

#include "em_emu.h"
typedef unsigned char      u1_t;
extern void opmode (u1_t mode);
void gps_sw_backup(void)
{
	volatile uint8_t gps_data[512];
	volatile uint32_t gps_size = 0;
	GPIO_PinModeSet(gpioPortA, 5, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortE, 12, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortE, 13, gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);

    opmode(0);

	spi_cs_clear(gps);
	delay_ms(1);

	for(int i = 0; i < (sizeof(cfg_msg_sw_backup)/sizeof(uint8_t)); i++)
		spi_read_write_byte(cfg_msg_sw_backup[i]);

	spi_cs_set(gps);
	delay_ms(1);

	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
	CMU_ClockSelectSet(cmuClock_HF, cmuOsc_LFXO);
	CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_32);
	CMU_OscillatorEnable(cmuOsc_HFXO, false, false);


	TIMER2->IEN = 0;
	SysTick->CTRL  =   0;
	GPIO->CTRL = 1;

	CMU_ClockEnable(cmuClock_GPIO, false);
	CMU_ClockEnable(cmuClock_USART0, false);
	CMU_ClockEnable(cmuClock_USART1, false);
	//CMU_ClockEnable(cmuClock_LEUART0, false);
	CMU_ClockEnable(cmuClock_HFPER, false);
	CMU_ClockEnable(cmuClock_USB, false);
	CMU_ClockEnable(cmuClock_USBC, false);
	CMU_ClockEnable( cmuClock_CORELE, false );

	//while(1);

	EMU_EnterEM4();
	while(1);
}

void gps_int_pin_toggle(void)
{
	GPIO_PinOutSet(GPS_SIG_PORT, GPS_INT);
	delay_ms(1);
	GPIO_PinOutClear(GPS_SIG_PORT, GPS_INT);
	delay_ms(1);
}
