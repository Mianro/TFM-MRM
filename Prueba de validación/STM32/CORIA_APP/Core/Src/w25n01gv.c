/*
 * w25n01gv.c
 *
 *  Created on: Jul 19, 2023
 *      Author: mrm
 */


// Copyright Jim Merkle, 2/17/2020
// Module: w25q128.c

#include "w25n01gv.h"
#include "main.h" // STM32 HAL defines
#include <stdio.h> // printf()
#include <string.h>
#include <stdlib.h> // strtol()
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
	uint8_t idcmd = W25_CMD_READ_JEDEC_ID;
	if(bufSize < W25_JEDEC_ID_BUF_SIZE)
		return HAL_ERROR; 														// buffer too small
	W25_CS_ENABLE(); 															// Drive Winbond chip select, /CS low
	retval = HAL_SPI_Transmit(&hspi1, &idcmd , sizeof(idcmd) + 1, TIMEOUT); 	// Send the ID command
	if(retval == HAL_OK)
		retval = HAL_SPI_Receive(&hspi1, buf, W25_JEDEC_ID_BUF_SIZE, TIMEOUT);
	W25_CS_DISABLE();
	return retval;
} // W25_ReadJEDECID()

// Returns value of Status Register-1 (byte)
// Winbond 8.2.4 Read Status Register-1 (05h)
// See section 7.1 for bit values
int W25_ReadStatusReg(uint8_t addrsr) {
	uint8_t cmdaddr[2] = {W25_CMD_READ_STATUS_REG,addrsr};
	uint8_t status_reg;
	int retval;
	W25_CS_ENABLE(); 																// Drive Winbond chip select, /CS low
	retval = HAL_SPI_Transmit(&hspi1, cmdaddr , sizeof(cmdaddr), TIMEOUT); 				// Send Read Status Reg command
	if(retval == HAL_OK)
		retval = HAL_SPI_Receive(&hspi1, &status_reg, sizeof(status_reg), TIMEOUT);
	W25_CS_DISABLE();
	return retval == HAL_OK ? status_reg:0xFF; 										// return 0xFF if error
} // W25_ReadStatusReg1()

int W25_WriteStatusReg(uint8_t addrsr, uint8_t sr) {
	uint8_t cmdaddr[2] = {W25_CMD_WRITE_STATUS_REG, addrsr};
	int retval;
	W25_CS_ENABLE();
	retval = HAL_SPI_Transmit(&hspi1, cmdaddr , sizeof(cmdaddr), TIMEOUT);
	if(retval == HAL_OK)
		retval = HAL_SPI_Transmit(&hspi1, &sr, sizeof(sr), TIMEOUT);
	W25_CS_DISABLE();
	return retval;
}

// Send Write Enable command
// Winbond 8.2.5 Write Enable (06h)
// See section 7.1, page 17, and section 8.2.1, page 30
// This sets the WEL bit, S1, in status register 1, allowing the part to be written.
int W25_WriteEnable(void) {
	uint8_t cmd = W25_CMD_WRITE_ENABLE;
	W25_CS_ENABLE(); 																// Drive Winbond chip select, /CS low
	int retval = HAL_SPI_Transmit(&hspi1, &cmd , sizeof(cmd), TIMEOUT); 			// Send Write Enable command
	W25_CS_DISABLE();
	return retval;
} // W25_WriteEnable()

// Send Write Disable command
// Winbond 8.2.6 Write Disable (04h)
// See section 7.1, page 17, and section 8.2.1, page 30
// This clears the WEL bit, S1, in status register 1, preventing writing to the part
int W25_WriteDisable(void) {
	uint8_t cmd = W25_CMD_WRITE_DISABLE;
	W25_CS_ENABLE(); 																	// Drive Winbond chip select, /CS low
	int retval = HAL_SPI_Transmit(&hspi1, &cmd , sizeof(cmd), TIMEOUT); 				// Send Write Disable command
	W25_CS_DISABLE();
	return retval;
} // W25_WriteDisable()

// Winbond 8.2.6 Read Data (03h)
// The only limit for quantity of data is memory / device size
int W25_Read(uint8_t *buf, uint32_t address, uint16_t bufSize)
{
	uint8_t cmdaddr[4] = {W25_CMD_READ_DATA,address>>8,address,0xFF};
	int retval;
	W25_CS_ENABLE(); 																	// Drive Winbond chip select, /CS low
	retval = HAL_SPI_Transmit(&hspi1, cmdaddr , sizeof(cmdaddr), 500); 					// Send Read Data command with address
	if(retval != HAL_OK) {
		return retval;
	}
	//memset(buf,0,bufSize); 																// Buffer is transmitted during receive
	retval = HAL_SPI_Receive(&hspi1, buf, bufSize, bufSize * 100); 								// need longer time-outs when using slow SPI clock
	if(retval != HAL_OK) {
		return retval;
	}
	W25_CS_DISABLE();
	return retval;
} // W25_ReadData()

// Winbond 8.2.17 Padge Data Read (13h)
// Transfer data from memory array to 2112 Byte Data Buffer
int W25_PageDataRead(uint16_t address)
{
  int retval;																// Make sure we can write...
  W25_CS_ENABLE(); 																	// Drive Winbond chip select, /CS low
  uint8_t cmdaddr[4] = {W25_CMD_PAGE_DATA_READ, 0xFF, address>>8, address};
  retval = HAL_SPI_Transmit(&hspi1, cmdaddr , sizeof(cmdaddr), TIMEOUT); 			// Send Sector Erase command with address
  W25_CS_DISABLE();
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
  W25_CS_ENABLE(); 																	// Drive Winbond chip select, /CS low
  uint8_t cmdaddr[3] = {W25_CMD_PAGE_PROGRAM, address>>8, address};
  retval = HAL_SPI_Transmit(&hspi1, cmdaddr , sizeof(cmdaddr), TIMEOUT); 			// Send Page Program command with address
  if(retval == HAL_OK)
	  retval = HAL_SPI_Transmit(&hspi1, buf, bufSize, TIMEOUT * bufSize);
  W25_CS_DISABLE();
  W25_DelayWhileBusy(PAGE_PROGRAM_TIMEOUT);
  return retval;
} // W25_PageProgram()

// Winbond 8.2.13 Program Execute (10h)
// Transfer data from Data buffer to memory array
int W25_ProgramExecute(uint16_t address)
{
  int retval;
  W25_CS_ENABLE(); 																	// Drive Winbond chip select, /CS low
  uint8_t cmdaddr[4] = {W25_CMD_PROGRAM_EXECUTE, 0xFF, address>>8, address};
  retval = HAL_SPI_Transmit(&hspi1, cmdaddr , sizeof(cmdaddr), TIMEOUT); 			// Send Sector Erase command with address
  W25_CS_DISABLE();
  W25_DelayWhileBusy(SECTOR_ERASE_TIMEOUT);
  return retval;
} // W25_PageDataRead()

// Winbond 8.2.17 Sector Erase (D8h)
// Erase all data within the addressed 128K sector.
int W25_SectorErase(uint32_t address)
{
  int retval;
  W25_WriteEnable(); 																// Make sure we can write...
  W25_CS_ENABLE(); 																	// Drive Winbond chip select, /CS low
  uint8_t cmdaddr[4] = {W25_CMD_BLOCK_ERASE,0xFF,address>>8,address};
  retval = HAL_SPI_Transmit(&hspi1, cmdaddr , sizeof(cmdaddr), TIMEOUT); 			// Send Sector Erase command with address
  W25_CS_DISABLE();
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
  uint32_t initial_count = HAL_GetTick();
  int busy;
  uint32_t deltaticks;
  uint32_t count = 0;
  do {
    busy = W25_Busy();
    deltaticks = HAL_GetTick() - initial_count;
    count++;
  } while(busy && deltaticks < msTimeout);
  int retval = busy ? HAL_TIMEOUT:HAL_OK;
  return retval;
}

int W25_WriteNoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
	int retval;
	uint16_t pageremain;
	pageremain=2048-WriteAddr%2048;
	if(NumByteToWrite<=pageremain)
		pageremain=NumByteToWrite;
	while(1) {
		retval = W25_WritePage(pBuffer,WriteAddr,pageremain);
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

/*
 * @brief littlefs read interface
 * @param [in] c lfs_config数据结构
 * @param [in] block 要读的块
 * @param [in] off 在当前块的偏移
 * @param [out] buffer 读取到的数据
 * @param [in] size 要读取的字节数
 * @return 0 成功 <0 错误
 * @note littlefs 一定不会存在跨越块存储的情况
 */
int W25_readLittlefs(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {

	uint16_t page_numb = (block * 64) + (off/2048);
	uint16_t column_numb = off%2048;
	W25_PageDataRead(page_numb);
	HAL_Delay(10);
	W25_Read((uint8_t *)buffer, column_numb, size);
	HAL_Delay(10);
	return LFS_ERR_OK;
}



/*
 * @brief littlefs write interface
 * @param [in] c lfs_config数据结构
 * @param [in] block 要读的块
 * @param [in] off 在当前块的偏移
 * @param [out] buffer 读取到的数据
 * @param [in] size 要读取的字节数
 * @return 0 成功 <0 错误
 * @note littlefs 一定不会存在跨越块存储的情况
 */
int W25_writeLittlefs(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {

	uint16_t page_numb = (block * 64) + (off/2048);
	uint16_t column_numb = off%2048;
	W25_WritePage((uint8_t *)buffer, column_numb, size);
	HAL_Delay(10);
	W25_ProgramExecute(page_numb);
	HAL_Delay(10);
	return LFS_ERR_OK;
}



/*
 * @brief littlefs 擦除一个块
 * @param [in] c lfs_config数据结构
 * @param [in] block 要擦出的块
 * @return 0 成功 <0 错误
 */
int W25_eraseLittlefs(const struct lfs_config *c, lfs_block_t block) {

	W25_SectorErase(block);
	return  LFS_ERR_OK;

}

int W25_syncLittlefs(const struct lfs_config *c )
{
	return  LFS_ERR_OK;

}
