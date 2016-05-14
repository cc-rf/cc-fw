/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#include <board.h>
#include "fsl_common.h"
#include "fsl_port.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Function Name : BOARD_InitPins */
void BOARD_InitPins(void)
{
    /* Initialize UART1 pins below */
    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortE);

    /* Affects PORTE_PCR0 register */
    PORT_SetPinMux(PORTE, 0u, kPORT_MuxAlt3);
    /* Affects PORTE_PCR1 register */
    PORT_SetPinMux(PORTE, 1u, kPORT_MuxAlt3);

    /* LED GPIO Port Clocks */
    CLOCK_EnableClock(kCLOCK_PortA);
    //CLOCK_EnableClock(kCLOCK_PortD);
    PORT_SetPinMux(BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_GREEN_GPIO_PIN, kPORT_MuxAsGpio);
    //PORT_SetPinMux(BOARD_LED_BLUE_GPIO_PORT, BOARD_LED_BLUE_GPIO_PIN, kPORT_MuxAsGpio);
    LED_RED_INIT(LOGIC_LED_OFF);
    LED_GREEN_INIT(LOGIC_LED_OFF);
    //LED_BLUE_INIT(LOGIC_LED_OFF);

    /* SPI1 */
    CLOCK_EnableClock(kCLOCK_PortD);
    PORT_SetPinMux(PORTD,  4u, kPORT_MuxAlt7);       // SPI1_PCS0   J2  6
    PORT_SetPinMux(PORTD,  5u, kPORT_MuxAlt7);       // SPI1_SCK    J2  12
    PORT_SetPinMux(PORTD,  6u, kPORT_MuxAlt7);       // SPI1_SOUT   J2  8
    PORT_SetPinMux(PORTD,  7u, kPORT_MuxAlt7);       // SPI1_SIN    J2  10

    /* CC Interrupt Pins */
    CLOCK_EnableClock(kCLOCK_PortC);
    //PORT_SetPinMux(PORTC,  0u, kPORT_MuxAsGpio);     // RF1_GPIO3   J2  5
    PORT_SetPinMux(PORTC,  2u, kPORT_MuxAsGpio);     // RF1_GPIO3   J24 8
    //CLOCK_EnableClock(kCLOCK_PortD);
    //PORT_SetPinMux(PORTD,  2u, kPORT_MuxAsGpio);     // RF1_GPIO3   J1  2

}
