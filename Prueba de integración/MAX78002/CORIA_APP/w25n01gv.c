/*
 * w25n01gv.c
 *
 *  Created on: Jul 19, 2023
 *      Author: mrm
 */

#include "w25n01gv.h"

#include "spi.h"   // HAL header files, SPI and GPIO defines
#include "gpio.h"
#include <stdio.h> // printf()
#include <string.h>
#include <stdlib.h> // strtol()
#include "mxc_delay.h"

#include "lfs.h"
//////////////////////////////////////////////////////
// Some useful defs and macros
#define LINEAR_TO_COLUMNECC(laddr) ((laddr) % PAGE_ECCSIZE)
#define LINEAR_TO_COLUMN(laddr) ((laddr) % pageSize)
#define LINEAR_TO_PAGE(laddr) ((laddr) / pageSize)
#define LINEAR_TO_PAGEECC(laddr) ((laddr) / PAGE_ECCSIZE)
#define LINEAR_TO_BLOCK(laddr) (LINEAR_TO_PAGE(laddr) / PAGES_PER_BLOCK)
#define BLOCK_TO_PAGE(block) ((block) * PAGES_PER_BLOCK)
#define BLOCK_TO_LINEAR(block) (BLOCK_TO_PAGE(block) * pageSize)
//more defines
#define pageSize              		2048

// Device size parameters
#define PAGE_SIZE			2048
#define PAGES_PER_BLOCK		64
#define BLOCKS_PER_DIE		1024

void W25_Init(void){

}

// Return 3 byte JEDEC Manufacturer and device ID (requires 3 byte buffer)
// Winbond 8.2.29 Read JEDEC ID (9Fh)
int W25_ReadJedecID(uint8_t *buf, int bufSize) {
	int retval;
	uint8_t txData[2];
	txData[0] = W25_CMD_READ_JEDEC_ID;
	if(bufSize < W25_JEDEC_ID_BUF_SIZE)
		return E_UNKNOWN; 														// buffer too small
	//W25_CS_ENABLE(); 															// Drive Winbond chip select, /CS low
	retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)&txData, 2, (uint8_t *)NULL, 0, 0, &target); 	// Send the ID command
	if(retval == E_NO_ERROR)
		retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)NULL, 0, (uint8_t *)buf, bufSize, 1, &target);
	//W25_CS_DISABLE(); 														// No need, doing CS assert/deassert by hw
	return retval;
} // W25_ReadJEDECID()

// Returns value of Status Register-1 (byte)
// Winbond 8.2.4 Read Status Register-1 (05h)
// See section 7.1 for bit values
int W25_ReadStatusReg(uint8_t addrsr) {
	uint8_t cmdaddr[2] = {W25_CMD_READ_STATUS_REG,addrsr};
	uint8_t status_reg;
	int retval;
	//W25_CS_ENABLE(); 																// Drive Winbond chip select, /CS low
	retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)&cmdaddr, sizeof(cmdaddr), (uint8_t *)NULL, 0, 0, &target); 	// Send the ID command
		if(retval == E_NO_ERROR)
			retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)NULL, 0, (uint8_t *)&status_reg, sizeof(status_reg), 1, &target);
	//W25_CS_DISABLE();
	return retval == E_NO_ERROR ? status_reg : 0xFF; 										// return 0xFF if error
} // W25_ReadStatusReg1()

int W25_WriteStatusReg(uint8_t addrsr, uint8_t sr) {
	uint8_t cmdaddr[2] = {W25_CMD_WRITE_STATUS_REG, addrsr};
	int retval;
	//W25_CS_ENABLE();
	retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)cmdaddr, sizeof(cmdaddr), (uint8_t *)NULL, 0, 0, &target);
	if(retval == E_NO_ERROR)
		retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)&sr, sizeof(sr), (uint8_t *)NULL, 0, 1, &target);
	//W25_CS_DISABLE();
	return retval;
}

// Send Write Enable command
// Winbond 8.2.5 Write Enable (06h)
// See section 7.1, page 17, and section 8.2.1, page 30
// This sets the WEL bit, S1, in status register 1, allowing the part to be written.
int W25_WriteEnable(void) {
	uint8_t cmd = W25_CMD_WRITE_ENABLE;
	//W25_CS_ENABLE(); 																// Drive Winbond chip select, /CS low
	int retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)NULL, 0, 1, &target); // Send Write Enable command
	//W25_CS_DISABLE();
	return retval;
} // W25_WriteEnable()

// Send Write Disable command
// Winbond 8.2.6 Write Disable (04h)
// See section 7.1, page 17, and section 8.2.1, page 30
// This clears the WEL bit, S1, in status register 1, preventing writing to the part
int W25_WriteDisable(void) {
	uint8_t cmd = W25_CMD_WRITE_DISABLE;
	//W25_CS_ENABLE(); 																	// Drive Winbond chip select, /CS low
	int retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)NULL, 0, 1, &target); // Send Write Disable command
	//W25_CS_DISABLE();
	return retval;
} // W25_WriteDisable()

// Winbond 8.2.6 Read Data (03h)
// The only limit for quantity of data is memory / device size
int W25_Read(uint8_t *buf, uint32_t address, uint16_t bufSize)
{
	uint8_t cmdaddr[4] = {W25_CMD_READ_DATA,address>>8,address,0xFF};
	int retval;
	//W25_CS_ENABLE(); 																	// Drive Winbond chip select, /CS low
	retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)&cmdaddr, sizeof(cmdaddr), (uint8_t *)NULL, 0, 0, &target);
	if(retval != E_NO_ERROR) {
		return retval;
	}
	//memset(buf,0,bufSize); 																// Buffer is transmitted during receive
	retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)NULL, 0, (uint8_t *)buf, bufSize, 1, &target);
	if(retval != E_NO_ERROR) {
		return retval;
	}
	//W25_CS_DISABLE();
	return retval;
} // W25_ReadData()

// Winbond 8.2.17 Padge Data Read (13h)
// Transfer data from memory array to 2112 Byte Data Buffer
int W25_PageDataRead(uint16_t address)
{
  int retval;																// Make sure we can write...
  //W25_CS_ENABLE(); 																	// Drive Winbond chip select, /CS low
  uint8_t cmdaddr[4] = {W25_CMD_PAGE_DATA_READ, 0xFF, address>>8, address};
  retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)&cmdaddr, sizeof(cmdaddr), (uint8_t *)NULL, 0, 1, &target);
  //W25_CS_DISABLE();
  W25_DelayWhileBusy(SECTOR_ERASE_TIMEOUT);
  return retval;
} // W25_PageDataRead()

// Winbond 8.2.15 Page Program (02h) ONLY TO DATA BUFFER NEEDS PROGRAM EXECUTE TO BE WRITTEN IN MEMORY
// Write one byte up to 2112 bytes (a page) of data
// If a 2112 byte page boundary is crossed, it must be addressed with an additional Page Program command.
int W25_WritePage(uint8_t *buf, uint32_t address, uint16_t bufSize)
{
  int retval;
  W25_WriteEnable(); 																// Make sure we can write...
  //W25_CS_ENABLE(); 																	// Drive Winbond chip select, /CS low
  uint8_t cmdaddr[3] = {W25_CMD_PAGE_PROGRAM, address>>8, address};
  retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)cmdaddr, sizeof(cmdaddr), (uint8_t *)NULL, 0, 0, &target);
  if(retval == E_NO_ERROR)
  	  retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)buf, bufSize, (uint8_t *)NULL, 0, 1, &target);
  //W25_CS_DISABLE();
  W25_DelayWhileBusy(PAGE_PROGRAM_TIMEOUT);
  return retval;
} // W25_PageProgram()

// Winbond 8.2.13 Program Execute (10h)
// Transfer data from Data buffer to memory array
int W25_ProgramExecute(uint16_t address)
{
  int retval;
  //W25_CS_ENABLE(); 																	// Drive Winbond chip select, /CS low
  uint8_t cmdaddr[4] = {W25_CMD_PROGRAM_EXECUTE, 0xFF, address>>8, address};
  retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)cmdaddr, sizeof(cmdaddr), (uint8_t *)NULL, 0, 1, &target);
  //W25_CS_DISABLE();
  W25_DelayWhileBusy(SECTOR_ERASE_TIMEOUT);
  return retval;
} // W25_PageDataRead()

// Winbond 8.2.17 Sector Erase (D8h)
// Erase all data within the addressed 128K sector.
int W25_SectorErase(uint32_t address)
{
  int retval;
  W25_WriteEnable(); 																// Make sure we can write...
  //W25_CS_ENABLE(); 																	// Drive Winbond chip select, /CS low
  uint8_t cmdaddr[4] = {W25_CMD_BLOCK_ERASE,0xFF,address>>8,address};
  retval = MXC_SPI_ControllerTransactionB(SPI1, (uint8_t *)&cmdaddr, sizeof(cmdaddr), (uint8_t *)NULL, 0, 1, &target);
  //W25_CS_DISABLE();
  W25_DelayWhileBusy(SECTOR_ERASE_TIMEOUT);
  return retval;
} // W25_SectorErase()

// Winbond 8.2.20 Chip Erase (60h)
// Erase all data within the FLASH device
int W25_ChipErase(void)
{
	return -1;
} // W25_ChipErase()

// Returns 0:Not busy, or 1:Busy
int W25_Busy(void)
{
  return (W25_ReadStatusReg(0xC0) & W25_STATUS1_BUSY);
}

// Loop while busy and not timeout
int W25_DelayWhileBusy(uint32_t msTimeout)
{
  int busy;
  uint32_t count = 0;
  do {
    busy = W25_Busy();
    MXC_Delay(1000); // Using blocking delay while figuring out how to properly use systick counting timer
    count++;
  } while(busy && count < msTimeout);
  int retval = busy ? E_TIME_OUT:E_NO_ERROR;
  return retval;
}

int W25_WriteNoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	int retval;
	uint16_t pageremain;
	pageremain=2048-WriteAddr%2048;
	if(NumByteToWrite<=pageremain)
		pageremain=NumByteToWrite;
	while(1) {
		retval = W25_WritePage(pBuffer, WriteAddr, pageremain);
		if(NumByteToWrite==pageremain) {
			break;
		}
	 	else 										// NumByteToWrite > pageremain
		{
			pBuffer += pageremain;
			WriteAddr += pageremain;
			NumByteToWrite -= pageremain;
			if(NumByteToWrite > 2048) {
				pageremain = 2048;
			}
			else {
				pageremain = NumByteToWrite;
			}
		}
	};
	return retval;
}

// lfs functions implementation


int W25_readLittlefs(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {

	uint16_t page_numb = (block * 64) + (off/2048);
	uint16_t column_numb = off%2048;
	W25_PageDataRead(page_numb);
	MXC_Delay(10000);
	W25_Read((uint8_t *)buffer, column_numb, size);
	MXC_Delay(10000);
	return LFS_ERR_OK;
}


int W25_writeLittlefs(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {

	uint16_t page_numb = (block * 64) + (off/2048);
	uint16_t column_numb = off%2048;
	W25_WritePage((uint8_t *)buffer, column_numb, size);
	MXC_Delay(10000);
	W25_ProgramExecute(page_numb);
	MXC_Delay(10000);
	return LFS_ERR_OK;
}



int W25_eraseLittlefs(const struct lfs_config *c, lfs_block_t block) {

	W25_SectorErase(block);
	return  LFS_ERR_OK;

}

int W25_syncLittlefs(const struct lfs_config *c )
{
	return  LFS_ERR_OK;

}

