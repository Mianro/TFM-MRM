/*
 * w25n01gv.h
 *
 *  Created on: Jul 19, 2023
 *      Author: mrm
 */

#ifndef _w25n01gv_h_
#define _w25n01gv_h_

#include <stdint.h> // uint8_t
#include "spi.h"   // HAL header files, SPI and GPIO defines
#include "gpio.h"

#include "lfs.h"

#define W25_CMD_WRITE_ENABLE        0x06 //Compliant
#define W25_CMD_WRITE_DISABLE       0x04 //Compliant
#define W25_CMD_READ_STATUS_REG		0x05 //Compliant
#define W25_CMD_WRITE_STATUS_REG	0x01 //Compliant
#define W25_CMD_READ_JEDEC_ID       0x9F //Compliant
#define W25_CMD_READ_DATA           0x03 //Compliant
#define W25_CMD_PAGE_PROGRAM        0x02 //Compliant
#define W25_CMD_BLOCK_ERASE			0xD8 //Compliant
#define W25_CMD_PROGRAM_EXECUTE		0x10
#define W25_CMD_PAGE_DATA_READ		0x13

// Status Register 1 bits (see section 7.1 Status Registers)
#define W25_STATUS1_BUSY            1<<0
#define W25_STATUS1_WEL             1<<1
#define W25_STATUS1_BP0             1<<2
#define W25_STATUS1_BP1             1<<3
#define W25_STATUS1_BP2             1<<4
#define W25_STATUS1_TB              1<<5
#define W25_STATUS1_SEC             1<<6
#define W25_STATUS1_SRP0            1<<7


extern mxc_spi_target_t target;
#define TIMEOUT                 100 // MS Timeout for HAL function calls
#define PAGE_PROGRAM_TIMEOUT    1000 // MS Timeout for Program
#define SECTOR_ERASE_TIMEOUT    1000 // MS Timeout for Sector Erase, Program, and Chip erase
#define CHIP_ERASE_TIMEOUT     	60000 // MS Timeout for Chip erase

#define W25_JEDEC_ID_BUF_SIZE   3   // bytes
#define W25_UNIQUE_ID_BUF_SIZE  8   // bytes
#define W25_PROGRAM_BLOCK_SIZE  2048  // bytes - can write from 1 up to 2048 bytes
#define W25_SECTOR_SIZE         131072 // bytes - used for erasing portions device
#define W25_DEVICE_SIZE         (65536*2048) // bytes (1GBit = 128MBytes)
#define W25_SECTOR_COUNT        1024

#define W25_ERASE_GRAN              131072
#define W25_NUM_GRAN                1024

/* W25Q128 chip select is active LOW */
#define W25_CS_ENABLE()   MXC_GPIO_OutClr(MXC_GPIO0, MXC_GPIO_PIN_20);
#define W25_CS_DISABLE()  MXC_GPIO_OutSet(MXC_GPIO0, MXC_GPIO_PIN_20);

#define SPI1 MXC_SPI1
#define SPI1_IRQ SPI1_IRQn

// Compliant Functions

void W25_Init(void); 												// Winbond W25 Flash Memory Init
int W25_ReadJedecID(uint8_t *buf, int bufSize); 					// Return 3 byte Manufacturer and device ID (requires 3 byte buffer)
int W25_ReadStatusReg(uint8_t addrsr);									// Return Status register
int W25_WriteStatusReg(uint8_t addrsr, uint8_t sr);									// Writes Status Register
int W25_Busy(void); 												// Returns 0:Not busy, or 1:Busy
int W25_DelayWhileBusy(uint32_t msTimeout); 						// Delay up to TIMEOUT value
int W25_WriteEnable(void);											// Send Write Enable command, allowing writing to the device
int W25_WriteDisable(void);											// Send Write Disable command, preventing writing to the device
int W25_WriteNoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
int W25_Read(uint8_t *buf, uint32_t address, uint16_t bufSize);		// Read Data from the FLASH device - no limit (the whole device can be read with this)
int W25_WritePage(uint8_t *buf, uint32_t address, uint16_t bufSize); 	// Write a page (or partial page) of data (up to 256 bytes - does not cross page boundaries)
int W25_SectorErase(uint32_t address);								// Erase a 128KB byte sector
int W25_ChipErase(void);											// Erase the entire chip (May take 40 seconds or more depending on the device)
int W25_PageDataRead(uint16_t address);
int W25_ProgramExecute(uint16_t address);

// LittleFS functions
int W25_readLittlefs(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);

int W25_writeLittlefs(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);

int W25_eraseLittlefs(const struct lfs_config *c, lfs_block_t block);

int W25_syncLittlefs(const struct lfs_config *c );

#endif // _w25n01gv_h_
