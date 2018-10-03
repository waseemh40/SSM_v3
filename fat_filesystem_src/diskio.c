/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "../fat_filesystem_header/diskio.h"		/* FatFs lower layer API */

/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */

		/*
		 * shared local variables
		 */
static bool init_flag = false;
/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{

	switch (pdrv) {
	case DEV_RAM :
		//result = RAM_disk_status();
		if (init_flag){
			return 0x00;
		}
		else{
			return 0x01;
		}

	case DEV_MMC :
		//result = MMC_disk_status();
		return STA_NOINIT;

	case DEV_USB :
		//result = USB_disk_status();
		return STA_NOINIT;
	}

	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{

	switch (pdrv) {
	case DEV_RAM :
		//result = RAM_disk_initialize();
		init_flag = sd_card_init();
		if (init_flag){
			return 0x00;
		}
		else{
			return 0x01;
		}

	case DEV_MMC :
		return STA_NOINIT;

	case DEV_USB :
		return STA_NOINIT;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	bool ret_flag=false;


	switch (pdrv) {
	case DEV_RAM :
		ret_flag = sd_card_read( (uint32_t)sector,(char *)buff, (uint32_t)count);

		if (ret_flag==true){
			return RES_OK;
		}
		else{
			return RES_ERROR;
		}

	case DEV_MMC :
		return RES_ERROR;
	case DEV_USB :
		return RES_ERROR;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	bool ret_flag=false;

	switch (pdrv) {
	case DEV_RAM :
		ret_flag = sd_card_write( (uint32_t)sector,(char *)buff, (uint32_t)count);

		if (ret_flag==true){
			return RES_OK;
		}
		else{
			return RES_ERROR;
		}

	case DEV_MMC :
		return RES_ERROR;
	case DEV_USB :
		return RES_ERROR;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;


	switch (pdrv) {
	case DEV_RAM :

		// Process of the command for the RAM drive
		res=RES_ERROR;
		return res;

	case DEV_MMC :

		// Process of the command for the MMC/SD card
		if(cmd==CTRL_SYNC){
			res=RES_OK;
		}
		else{
			res=RES_PARERR;
		}
		return res;

	case DEV_USB :

		// Process of the command the USB drive
		res=RES_ERROR;
		return res;
	}

	return RES_PARERR;
}

