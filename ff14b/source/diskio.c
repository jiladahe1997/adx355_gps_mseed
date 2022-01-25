/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "main.h"
#include "rtthread.h"
static uint8_t* dma_rx_buffer; //10kb
static uint8_t* dma_tx_buffer; //10kb
extern SD_HandleTypeDef hsd1;

/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	HAL_SD_CardStateTypeDef ret;

	switch (pdrv) {
	case DEV_MMC :
		// ret = HAL_SD_GetCardState(&hsd1);
		// return ret == HAL_SD_CARD_TRANSFER ? RES_OK : STA_NOINIT;
		return RES_OK;
	default:
		break;
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
	case DEV_MMC :
		dma_rx_buffer = rt_malloc(10240);
		dma_tx_buffer = rt_malloc(10240);
		if(dma_rx_buffer == NULL || dma_tx_buffer == NULL) {
			return RES_ERROR;
		}
		rt_kprintf("address of SDMMC DMA:[%p] [%p]\n", dma_rx_buffer, dma_tx_buffer);
		/* sd卡在sdcard_driver.c中进行初始化 */
		return RES_OK;
	default:
		break;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

/* 接收buffer，越大SD速度越快，必须为BLOCKSIZE的整数倍 
   BLOCKSIZE标准值为512字节
   BLOCKSIZE*20 10kb缓冲区
*/
#include <stdint.h>
volatile uint8_t RxCplt=0;;
volatile uint8_t TxCplt=0;;


void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  RxCplt=1;
}

uint8_t aRxBuffer[BLOCKSIZE*100]__attribute__((section (".RAM_D1"))) = {0};
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	HAL_StatusTypeDef result;
	rt_tick_t t1;

	switch (pdrv) {
	case DEV_MMC :
		//wait for sd card ok, 1000ms timeout
		t1 = rt_tick_get();
		while(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER){
			rt_tick_t t2 = rt_tick_get();
			if(t2-t1 > rt_tick_from_millisecond(1000)){
				return RES_ERROR;
			}
		}

		result = HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t*)dma_rx_buffer, (uint32_t)sector, (uint32_t)count);
		if(result != HAL_OK){
			rt_kprintf("HAL_SD_ReadBlocks_DMA error, ret is:[%d]\n",result);
			return RES_ERROR;
		}
		while(RxCplt==0);
		RxCplt=0;
		rt_memcpy(buff, dma_rx_buffer, count*BLOCKSIZE);

		// translate the reslut code here
		switch (result)
		{
		case HAL_OK:
			return RES_OK;
		case HAL_ERROR:
			return RES_ERROR;
		case HAL_BUSY:
			return RES_ERROR;
		case HAL_TIMEOUT:
			return RES_ERROR;
		default:
			return RES_ERROR;
		}
	default:
		break;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  TxCplt=1;
}
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	int result;
	rt_tick_t t1;

	switch (pdrv) {
	case DEV_MMC :
		// translate the arguments here
		t1 = rt_tick_get();
		while(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER){
			rt_tick_t t2 = rt_tick_get();
			if(t2-t1 > rt_tick_from_millisecond(1000)){
				return RES_ERROR;
			}
		}
		rt_memcpy(dma_tx_buffer, buff, count*BLOCKSIZE);
		result = HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t*)dma_tx_buffer, (uint32_t)sector, (uint32_t)count);
		while(TxCplt==0);
		TxCplt=0;

		// translate the reslut code here
		switch (result)
		{
		case HAL_OK:
			return RES_OK;
		case HAL_ERROR:
			return RES_ERROR;
		case HAL_BUSY:
			return RES_ERROR;
		case HAL_TIMEOUT:
			return RES_ERROR;
		default:
			return RES_ERROR;
		}
	default:
		break;
	}

	return RES_PARERR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{

	switch (pdrv) {
	case DEV_MMC :

		// Process of the command for the MMC/SD card
		switch (cmd)
		{
		case CTRL_SYNC:
			/* code */
			while(HAL_SD_GetCardState(&hsd1)!=HAL_SD_CARD_TRANSFER);
			return RES_OK;
			break;
		
		case GET_SECTOR_COUNT:
			*(DWORD*)buff = hsd1.SdCard.BlockNbr;
			return RES_OK;
			break;

		case GET_BLOCK_SIZE:
			*(DWORD*)buff = hsd1.SdCard.BlockSize;
			return RES_OK;
			break;
		
		default:
			break;
		}

		return RES_ERROR;
	default:
		break;
	}

	return RES_PARERR;
}

