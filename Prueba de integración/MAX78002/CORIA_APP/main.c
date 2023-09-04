/**
 * @file        main.c
 * @brief       ESTAR_Coria MAX78002 Integration Test APP
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
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

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
#include "cnn.h"
#include "sampledata.h"
#include "sampleoutput.h"

#include "lfs.h"
#include "lfs_util.h"
#include "w25n01gv.h"

/***** Definitions *****/

#define CLS "\033[2J"
#define HOME "\033[H"

#define MXC_GPIO_PORT_IN1 MXC_GPIO0
#define MXC_GPIO_PIN_IN1 MXC_GPIO_PIN_3

#define MXC_GPIO_PORT_OUT1 MXC_GPIO0
#define MXC_GPIO_PIN_OUT1 MXC_GPIO_PIN_25

#define MXC_GPIO_PORT_OUT2 MXC_GPIO0
#define MXC_GPIO_PIN_OUT2 MXC_GPIO_PIN_24

#define MXC_GPIO_PORT_OUT3 MXC_GPIO0
#define MXC_GPIO_PIN_OUT3 MXC_GPIO_PIN_2

#define MXC_GPIO_PORT_OUTSS MXC_GPIO0
#define MXC_GPIO_PIN_OUTSS MXC_GPIO_PIN_20

#define SPI0 MXC_SPI0
#define SPI0_IRQ SPI0_IRQn

#define VALUE 0xFFFF
#define DATA_LEN 8 // Words

#define REV(x) ( ((x&0xff000000)>>24) | (((x&0x00ff0000)<<8)>>16) | (((x&0x0000ff00)>>8)<<16) | ((x&0x000000ff) << 24) )

/***** Globals *****/
uint8_t rx_data[DATA_LEN];
uint8_t tx_data[DATA_LEN];
volatile int SPI1_FLAG, SPI0_FLAG;
int TX_DMA_CH, RX_DMA_CH;
mxc_spi_target_t target;
uint8_t buf[2112];
uint32_t data[200];
volatile uint32_t cnn_time; // Stopwatch
uint16_t cnn_out_buffer[100];


// lfs
// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;
lfs_file_t file_max;
lfs_file_t file_input_data;
lfs_file_t file_input_60;
lfs_file_t file_out;

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

void fail(void)
{
    printf("\n*** FAIL ***\n\n");
    while (1) {}
}

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

// 3-channel 32x32 data input (3072 bytes total / 1024 bytes per channel):
// HWC 32x32, channels 0 to 2
uint32_t input_60[1024];

void load_input(void)
{
    // This function loads the sample data input -- replace with actual data

    memcpy32((uint32_t *)0x54860000, input_60, 1024);
}

// Expected output of layer 4 for mnist given the sample input (known-answer test)
// Delete this function for production code
static const uint32_t sample_output[] = SAMPLE_OUTPUT;
int check_output(void)
{
    int i;
    uint32_t mask, len;
    volatile uint32_t *addr;
    const uint32_t *ptr = sample_output;

    while ((addr = (volatile uint32_t *)*ptr++) != 0) {
        mask = *ptr++;
        len = *ptr++;
        for (i = 0; i < len; i++)
            if ((*addr++ & mask) != *ptr++) {
                printf("Data mismatch (%d/%d) at address 0x%08x: Expected 0x%08x, read 0x%08x.\n",
                       i + 1, len, addr - 1, *(ptr - 1), *(addr - 1) & mask);
                return CNN_FAIL;
            }
    }

    return CNN_OK;
}

// Classification layer:
static int32_t ml_data[CNN_NUM_OUTPUTS];
static q15_t ml_softmax[CNN_NUM_OUTPUTS];

void softmax_layer(void)
{
    cnn_unload((uint32_t *)ml_data);
    softmax_q17p14_q15((const q31_t *)ml_data, CNN_NUM_OUTPUTS, ml_softmax);
}

int main(void)
{
	int retVal, err;
	int i;
	int digs, tens;
    mxc_gpio_cfg_t gpio_out1;
    mxc_gpio_cfg_t gpio_out2;
    mxc_gpio_cfg_t gpio_out3;
    mxc_gpio_cfg_t gpio_in1;
    mxc_gpio_cfg_t gpio;
    mxc_spi_init_t init;
	SPI1_FLAG = 1;

	MXC_ICC_Enable(MXC_ICC0); // Enable cache
    MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ISO);
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_ISO);
    MXC_GCR->pm &= ~MXC_F_GCR_PM_ISO_PD; // enable ISO during sleep
    MXC_GCR->ipll_ctrl |= MXC_F_GCR_IPLL_CTRL_EN; // Enable IPLL
    SystemCoreClockUpdate();
    /* Setup input pin 2. */
	gpio_in1.port = MXC_GPIO_PORT_IN1;
	gpio_in1.mask = MXC_GPIO_PIN_IN1;
	gpio_in1.pad = MXC_GPIO_PAD_NONE;
	gpio_in1.func = MXC_GPIO_FUNC_IN;
	MXC_GPIO_Config(&gpio_in1);

    MXC_Delay(MXC_DELAY_SEC(1));

    Console_Init();
    printf(CLS);
    printf(HOME);
    printf("ESTAR_CORIA APP vD1.1 (MAX78002)\n\r");

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

    /* Setup output pin 2. */
    gpio_out3.port = MXC_GPIO_PORT_OUT3;
    gpio_out3.mask = MXC_GPIO_PIN_OUT3;
    gpio_out3.pad = MXC_GPIO_PAD_NONE;
    gpio_out3.func = MXC_GPIO_FUNC_OUT;
    MXC_GPIO_Config(&gpio_out3);


	MXC_GPIO_OutSet(gpio_out1.port, gpio_out1.mask);
	MXC_GPIO_OutSet(gpio_out2.port, gpio_out2.mask);
	MXC_GPIO_OutClr(gpio_out3.port, gpio_out3.mask);

	printf("GPIO DONE\n\r");
	printf("Waiting for STM\n\r");
	while(MXC_GPIO_InGet(gpio_in1.port, gpio_in1.mask) == false){
		MXC_Delay(10000);
	}

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
	// MAKE SURE WE CAN WRITE ALL MEMORY FIRST
	W25_WriteStatusReg(0xA0, 0b00000000);
	// mount the filesystem
	err = lfs_mount(&lfs, &cfg);
	MXC_Delay(5000);

	if (err == 0) {
	  //read data from stm32
	  lfs_file_open(&lfs, &file_input_60, "input_60", LFS_O_RDWR | LFS_O_CREAT);
	  lfs_file_read(&lfs, &file_input_60, &input_60, sizeof(input_60));
	  lfs_file_close(&lfs, &file_input_60);

	  lfs_unmount(&lfs);


	  for (uint16_t i = 0; i <= 1023 ; i++){
		  printf("%08X ", input_60[i]);
		  MXC_Delay(1);
		  if(((i + 1) % 7) == 0){
			  printf("\n\r");
			  MXC_Delay(1);
		  }
	  }
	}

    retVal = MXC_SPI_Shutdown(SPI1);

    if (retVal != E_NO_ERROR) {
        printf("\n-->SPI1 SHUTDOWN ERROR: %d\n", retVal);
        return retVal;
    }

    // Enable peripheral, enable CNN interrupt, turn on CNN clock
    // CNN clock: ISO (50 MHz) div 1
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_ISO, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);

    printf("\n*** CNN Inference Test ***\n");

    cnn_init(); // Bring state machine into consistent state
    printf("CNN Init DONE\n\r");
	cnn_load_weights(); // Load kernels
	printf("CNN Load weights DONE\n\r");
	cnn_load_bias();
	printf("CNN Load bias DONE\n\r");
	cnn_configure(); // Configure state machine
	printf("CNN Configure DONE\n\r");
	load_input(); // Load data input
    printf("CNN Input Loaded DONE\n\r");
	cnn_start(); // Start CNN processing
	printf("CNN Start DONE\n\r");
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; // SLEEPDEEP=0
	while (cnn_time == 0) __WFI(); // Wait for CNN
    //MXC_Delay(1000000);

    if (check_output() != CNN_OK)
        fail();
    softmax_layer();

    printf("\n*** PASS ***\n\n");

    cnn_disable(); // Shut down CNN clock, disable peripheral

#ifdef CNN_INFERENCE_TIMER
    printf("Approximate inference time: %u us\n\n", cnn_time);
#endif

    /*printf("Classification results:\n");
    for (i = 0; i < CNN_NUM_OUTPUTS; i++) {
        digs = (1000 * ml_softmax[i] + 0x4000) >> 15;
        tens = digs % 10;
        digs = digs / 10;
        printf("[%7d] -> Class %d: %d.%d%%\n", ml_data[i], i, digs, tens);
    }*/

    retVal = MXC_SPI_Init_v2(&init);
    if (retVal != E_NO_ERROR) {
        printf("\nSPI1 INITIALIZATION ERROR\n");
        return retVal;
    }
    MXC_SPI_SetMode(SPI1, SPI_MODE_1);
    // Blocking SPI v2 Implementation is Interrupt driven.
	NVIC_EnableIRQ(SPI1_IRQ);
	// MAKE SURE WE CAN WRITE ALL MEMORY FIRST
	W25_WriteStatusReg(0xA0, 0b00000000);
	// mount the filesystem
	err = lfs_mount(&lfs, &cfg);
	MXC_Delay(5000);
	if (err == 0) {
		lfs_file_open(&lfs, &file_out, "file_out", LFS_O_RDWR | LFS_O_CREAT);
		lfs_file_rewind(&lfs, &file_out);
		lfs_file_write(&lfs, &file_out, &ml_softmax, sizeof(ml_softmax));
		lfs_file_close(&lfs, &file_out);
		lfs_unmount(&lfs);
		printf("\nResults back to FileSystem\n");
	}
    retVal = MXC_SPI_Shutdown(SPI1);

    if (retVal != E_NO_ERROR) {
        printf("\n-->SPI1 SHUTDOWN ERROR: %d\n", retVal);
        return retVal;
    }
    /* Setup output pin 20. */
    gpio.port = MXC_GPIO0;
    gpio.mask = MXC_GPIO_PIN_20;
    gpio.pad = MXC_GPIO_PAD_NONE;;
    gpio.func = MXC_GPIO_FUNC_IN;
    MXC_GPIO_Config(&gpio);
    /* Setup output pin 21. */
    gpio.port = MXC_GPIO0;
    gpio.mask = MXC_GPIO_PIN_21;
    gpio.pad = MXC_GPIO_PAD_NONE;;
    gpio.func = MXC_GPIO_FUNC_IN;
    MXC_GPIO_Config(&gpio);
    /* Setup output pin 22. */
    gpio.port = MXC_GPIO0;
    gpio.mask = MXC_GPIO_PIN_22;
    gpio.pad = MXC_GPIO_PAD_NONE;;
    gpio.func = MXC_GPIO_FUNC_IN;
    MXC_GPIO_Config(&gpio);
    /* Setup output pin 23. */
    gpio.port = MXC_GPIO0;
    gpio.mask = MXC_GPIO_PIN_23;
    gpio.pad = MXC_GPIO_PAD_NONE;;
    gpio.func = MXC_GPIO_FUNC_IN;
    MXC_GPIO_Config(&gpio);

    printf("\n Sending signal to STM   \n");
    MXC_GPIO_OutSet(gpio_out3.port, gpio_out3.mask);
    printf("\nBye Bye - MRM       \n");
    MXC_LP_EnterStandbyMode();

    while (1) {
    }

    return 0;
}
