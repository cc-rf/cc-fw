/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * This is template for main module created by New Kinetis SDK 2.x Project Wizard. Enjoy!
 **/

#include <string.h>
#include <stdio.h>

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

/* FreeRTOS kernel includes. */
/*#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"*/


#include <cc/spi.h>
#include <cc/io.h>
#include <cc/phy.h>
#include <cc/cc1200.h>
#include <cc/isr.h>
#include <cc/mac.h>

/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)

static void mac_rx(u8 *buf, u8 len)
{
    LED_GREEN_TOGGLE();
    printf("rx: len=%u\r\n", len);
}

mac_cfg_t cc_mac_cfg = {
        .rx = mac_rx
};

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */

/*static void hello_task(void *pvParameters) {
    //printf("task started.\r\n");



  for (int j = 3; j >= 0; j--) {
	//printf("<%i>\r\n", j);
      vTaskDelay(1000);
    //vTaskSuspend(NULL);
  }
}*/

static void tick(void);
static void isr_test(void);

extern void vcom_init(void);
extern void vcom_task(void);

/*!
 * @brief Application entry point.
 */
int main(void) {
  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
    //printf("<boot>\r\n");

    vcom_init();

    /*for (int j0 = 10; j0 >= 0; j0--)
        for (int j = 10000000; j >= 0; j--);
*/
    printf("<starting>\r\n");

    LED_RED_ON();

  BOARD_InitDebugConsole();
    /* Add your code here */
    //printf("boot\r\n");
    //printf("(SEM) boot\r\n");


    cc_spi_init();


    cc_strobe(CC1200_SRES);
    int i = 0;

    do {
        ++i;
    } while (cc_strobe(CC1200_SNOP) & CC1200_STATUS_CHIP_RDYn);

    u8 pn = cc_get(CC1200_PARTNUMBER);
    //printf("init count = %i\npart number = 0x%02X\n", i, pn);

    if (pn == 0x20) {
        mac_init(&cc_mac_cfg);
        LED_RED_OFF();
        //LED_GREEN_ON();

        //mac_rx_enable();

        //for (int j = 10000000; j >= 0; j--);
        //LED_BLUE_OFF();
    } else {
        for(;;) { /* Infinite loop to avoid leaving the main function */
            for (int j = 10000; j >= 0; j--);
            LED_RED_TOGGLE();
            __asm("NOP"); /* something to use as a breakpoint stop while looping */
        }
    }

  /* Create RTOS task */
  //printf("starting task\r\n");
  /*xTaskCreate(hello_task, "Hello_task", configMINIMAL_STACK_SIZE, NULL, hello_task_PRIORITY, NULL);
  vTaskStartScheduler();*/

    u8 buf[] = "Hello!\n";

  for(;;) { /* Infinite loop to avoid leaving the main function */
      vcom_task();
      for (int j = 10000000; j >= 0; j--);
      mac_tx(buf, 8);
      LED_RED_TOGGLE();
  }
}

static void tick(void)
{
    static u32 count = 0;

    if (!count) {
        LED_RED_TOGGLE();
    }

    count = (count + 1) % 1000000;
}


static void my_isr(void)
{
    LED_RED_OFF();
    LED_GREEN_ON();
}

static void isr_test(void)
{
    cc_isr(ISR_PIN_GPIO3, ISR_EDGE_LOW, my_isr);
    cc_set(CC1200_IOCFG3, 50 /*CHIP_RDYn*/);
    //cc_set(CC1200_IOCFG3, 59 /*XOSC_STABLE*/);
    cc_strobe(CC1200_SNOP);

    while (1) __asm("nop");
}

