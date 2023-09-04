/**
 * @file        main.c
 * @brief       GPIO Example
 * @details
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

/***** Includes *****/
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "mxc_sys.h"
#include "fcr_regs.h"
#include "icc.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "pb.h"
#include "board.h"
#include "mxc.h"
#include "gpio.h"
#include "uart.h"
#include "dma.h"
#include "mxc_pins.h"
#include "spi.h"
#include "rtc.h"

#include "lfs.h"
#include "lfs_util.h"
#include "w25n01gv.h"

/***** Definitions *****/

#define CLS "\033[2J"
#define HOME "\033[H"

#define MXC_GPIO_PORT_OUT1 MXC_GPIO0
#define MXC_GPIO_PIN_OUT1 MXC_GPIO_PIN_25

#define MXC_GPIO_PORT_OUT2 MXC_GPIO0
#define MXC_GPIO_PIN_OUT2 MXC_GPIO_PIN_24

#define MXC_GPIO_PORT_OUTSS MXC_GPIO0
#define MXC_GPIO_PIN_OUTSS MXC_GPIO_PIN_20

#define SPI0 MXC_SPI0
#define SPI0_IRQ SPI0_IRQn

#define VALUE 0xFFFF
#define DATA_LEN 8 // Words

/***** Globals *****/
uint8_t rx_data[DATA_LEN];
uint8_t tx_data[DATA_LEN];
volatile int SPI1_FLAG, SPI0_FLAG;
int TX_DMA_CH, RX_DMA_CH;
mxc_spi_target_t target;
uint8_t buf[2112];

// lfs
// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;
lfs_file_t file_max;

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
    // block device operations
    .read  = W25_readLittlefs,
    .prog  = W25_writeLittlefs,
    .erase = W25_eraseLittlefs,
    .sync  = W25_syncLittlefs,

    // block device configuration
    .read_size = 512,
    .prog_size = 512,
    .block_size = W25_PROGRAM_BLOCK_SIZE * W25_SECTOR_COUNT,
    .block_count = W25_SECTOR_COUNT,
    .cache_size = 512,
    .lookahead_size = 512,
    .block_cycles = 1000,
};

void SPI1_IRQHandler(void)
{
	MXC_SPI_Handler(SPI1);
}
void SPI0_IRQHandler(void)
{
	MXC_SPI_Handler(SPI0);
}
void DMA_TX_IRQHandler(void)
{
    MXC_SPI_DMA_TX_Handler(SPI1);
}

void DMA_RX_IRQHandler(void)
{
    MXC_SPI_DMA_RX_Handler(SPI1);
}

void SPI1_Callback(void *data, int error)
{
    SPI1_FLAG = error;
}
void SPI0_Callback(void *data, int error)
{
    SPI0_FLAG = error;
}


int main(void)
{
	int retVal, err;
    mxc_gpio_cfg_t gpio_out1;
    mxc_gpio_cfg_t gpio_out2;
    mxc_spi_init_t init;
	SPI1_FLAG = 1;

    // Wait for PMIC 1.8V to become available, about 180ms after power up.
    MXC_Delay(MXC_DELAY_SEC(1));

    Console_Init();
    printf(CLS);
    printf(HOME);
    printf("ESTAR_CORIA APP vD0.1 (MAX78002)\n\r");

    /* Setup output pin 1. */
    gpio_out1.port = MXC_GPIO_PORT_OUT1;
    gpio_out1.mask = MXC_GPIO_PIN_OUT1;
    gpio_out1.pad = MXC_GPIO_PAD_NONE;;
    gpio_out1.func = MXC_GPIO_FUNC_OUT;
    MXC_GPIO_Config(&gpio_out1);

    /* Setup output pin 2. */
    gpio_out2.port = MXC_GPIO_PORT_OUT2;
    gpio_out2.mask = MXC_GPIO_PIN_OUT2;
    gpio_out2.pad = MXC_GPIO_PAD_NONE;
    gpio_out2.func = MXC_GPIO_FUNC_OUT;
    MXC_GPIO_Config(&gpio_out2);


	MXC_GPIO_OutSet(gpio_out1.port, gpio_out1.mask);
	MXC_GPIO_OutSet(gpio_out2.port, gpio_out2.mask);

	printf("GPIO DONE\n\r");
	MXC_Delay(MXC_DELAY_SEC(10));

    // Initialization Settings.
    init.spi = SPI1;
    init.freq = 1000000;
    init.spi_pins = NULL; // Use default, predefined pins
    init.mode = MXC_SPI_INTERFACE_STANDARD; // 4-wire
    init.type = MXC_SPI_TYPE_CONTROLLER;
    init.clk_mode = MXC_SPI_CLKMODE_1; // CPOL: 0, CPHA: 0
    init.frame_size = 8;
    init.callback = SPI1_Callback;
    init.ts_control = MXC_SPI_TSCONTROL_HW_AUTO; // HW will deassert/assert TS pins.
    init.target.active_polarity = 0;
    init.target.init_mask = 0x01; // Initialize Target Select 0 pin.
    init.vssel = MXC_GPIO_VSSEL_VDDIOH;
    // Select target for transaction.
    target.index = 0; // TS0
    // DMA Settings.
    init.use_dma = false;

    retVal = MXC_SPI_Init_v2(&init);
    if (retVal != E_NO_ERROR) {
        printf("\nSPI1 INITIALIZATION ERROR\n");
        return retVal;
    }
    MXC_SPI_SetMode(SPI1, SPI_MODE_1);
    // Blocking SPI v2 Implementation is Interrupt driven.
	NVIC_EnableIRQ(SPI1_IRQ);
	W25_ReadJedecID(rx_data, 3);
	printf("Jedec ID: %02X %02X %02X \n\r", rx_data[0], rx_data[1], rx_data[2]);

	// MAKE SURE WE CAN WRITE ALL MEMORY FIRST
	W25_WriteStatusReg(0xA0, 0b00000000);
	int srA = W25_ReadStatusReg(0xA0);
	printf("srA: %02X\n\r", srA);
	MXC_Delay(100000);

	// mount the filesystem
	err = lfs_mount(&lfs, &cfg);
	printf("err: %d\n\r", err);
	MXC_Delay(5000);

	if (err == 0) {
	  // read current count
	  uint32_t boot_count = 0;

	  lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
	  lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));
	  lfs_file_close(&lfs, &file);
	  // print the boot count from STM32
	  printf("boot_count: %d\n\r", boot_count);

	  uint32_t max_count = 0;
	  lfs_file_open(&lfs, &file_max, "max_count", LFS_O_RDWR | LFS_O_CREAT);
	  lfs_file_read(&lfs, &file_max, &max_count, sizeof(max_count));
	  max_count += 1;
	  lfs_file_rewind(&lfs, &file_max);
	  lfs_file_write(&lfs, &file_max, &max_count, sizeof(max_count));
	  lfs_file_close(&lfs, &file_max);
	  lfs_unmount(&lfs);
	  printf("max_count: %d\n\r", max_count);
	}

    retVal = MXC_SPI_Shutdown(SPI1);

    if (retVal != E_NO_ERROR) {
        printf("\n-->SPI1 SHUTDOWN ERROR: %d\n", retVal);
        return retVal;
    }
    // Initialization Settings.
    init.spi = SPI0;
    init.freq = 1000000;
    init.spi_pins = NULL; // Use default, predefined pins
    init.mode = MXC_SPI_INTERFACE_STANDARD; // 4-wire
    init.type = MXC_SPI_TYPE_CONTROLLER;
    init.clk_mode = MXC_SPI_CLKMODE_1; // CPOL: 0, CPHA: 0
    init.frame_size = 8;
    init.callback = SPI0_Callback;
    init.ts_control = MXC_SPI_TSCONTROL_HW_AUTO; // HW will deassert/assert TS pins.
    init.target.active_polarity = 0;
    init.target.init_mask = 0x01; // Initialize Target Select 0 pin.
    init.vssel = MXC_GPIO_VSSEL_VDDIOH;
    // Select target for transaction.
    target.index = 0; // TS0
    // DMA Settings.
    init.use_dma = false;

    retVal = MXC_SPI_Init_v2(&init);
    if (retVal != E_NO_ERROR) {
        printf("\nSPI0 INITIALIZATION ERROR\n");
        return retVal;
    }
    MXC_SPI_SetMode(SPI0, SPI_MODE_1);

    // SPI Request (Callback)
	SPI0_FLAG = 1;

    tx_data[0] = 0x9F;

    // Blocking SPI v2 Implementation is Interrupt driven.
	NVIC_EnableIRQ(SPI0_IRQ);
	MXC_SPI_ControllerTransactionB(SPI0, (uint8_t *)tx_data, 4, (uint8_t *)rx_data,
								  0, 0, &target);
	MXC_SPI_ControllerTransactionB(SPI0, (uint8_t *)tx_data, 0, (uint8_t *)rx_data,
									  3, 1, &target);
	printf("\n\r\n\r\n\rRAM ID: %02X %02X \n\r", rx_data[0], rx_data[1]);

    retVal = MXC_SPI_Shutdown(SPI0);

    if (retVal != E_NO_ERROR) {
        printf("\n-->SPI0 SHUTDOWN ERROR: %d\n", retVal);
        return retVal;
    }



    while (1) {
    	MXC_Delay(1000000);
    }

    return 0;
}
