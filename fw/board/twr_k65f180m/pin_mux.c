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
#include "fsl_device_registers.h"
#include "fsl_port.h"
#include "pin_mux.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Function Name : BOARD_InitPins */
void BOARD_InitPins(void)
{
    /* Initialize UART2 pins below */
    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortE);
    /* Affects PORTE_PCR16 register */
    PORT_SetPinMux(PORTE, 16u, kPORT_MuxAlt3);
    /* Affects PORTE_PCR17 register */
    PORT_SetPinMux(PORTE, 17u, kPORT_MuxAlt3);

    /* LED GPIO Port Clocks */
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinMux(BOARD_LED_YELLOW_GPIO_PORT, BOARD_LED_YELLOW_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_LED_ORANGE_GPIO_PORT, BOARD_LED_ORANGE_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_GREEN_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_LED_BLUE_GPIO_PORT, BOARD_LED_BLUE_GPIO_PIN, kPORT_MuxAsGpio);
    LED_YELLOW_INIT(LOGIC_LED_OFF);
    LED_ORANGE_INIT(LOGIC_LED_OFF);
    LED_GREEN_INIT(LOGIC_LED_OFF);
    LED_BLUE_INIT(LOGIC_LED_OFF);

    /* Enable trace pins (is this needed?? */
    PORT_SetPinMux(PORTE, 0u, kPORT_MuxAlt5);
    PORT_SetPinMux(PORTE, 1u, kPORT_MuxAlt5);
    PORT_SetPinMux(PORTE, 2u, kPORT_MuxAlt5);
    PORT_SetPinMux(PORTE, 3u, kPORT_MuxAlt5);
    PORT_SetPinMux(PORTE, 4u, kPORT_MuxAlt5);

    // Phillip: Is this needed for JTAG I/O afer boot?
    PORT_SetPinMux(PORTA, 1u, kPORT_MuxAlt7);       // JTAG_TDI
    PORT_SetPinMux(PORTA, 2u, kPORT_MuxAlt7);       // JTAG_TDO

    /* SPI0 */
    //PORT_SetPinMux(PORTE, 16u, kPORT_MuxAlt2);       // SPI0_PCS0
    //PORT_SetPinMux(PORTE, 17u, kPORT_MuxAlt2);       // SPI0_SCK
    //PORT_SetPinMux(PORTE, 18u, kPORT_MuxAlt2);       // SPI0_SOUT
    //PORT_SetPinMux(PORTE, 19u, kPORT_MuxAlt2);       // SPI0_SIN

    /* SPI2 */
    CLOCK_EnableClock(kCLOCK_PortD);
    PORT_SetPinMux(PORTD, 12u, kPORT_MuxAlt2);       // SPI2_SCK    J12 12
    PORT_SetPinMux(PORTD, 13u, kPORT_MuxAlt2);       // SPI2_SOUT   J12 10
    PORT_SetPinMux(PORTD, 14u, kPORT_MuxAlt2);       // SPI2_SIN    J12  9
    PORT_SetPinMux(PORTD, 15u, kPORT_MuxAlt2);       // SPI2_PCS1   J12 11  [Remove J16 from 1-2]

    /* CC Interrupt Pins */
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinMux(PORTC, 19u, kPORT_MuxAsGpio);     // RF1_GPIO3   J12 19  [Solder DNP R41]
}
