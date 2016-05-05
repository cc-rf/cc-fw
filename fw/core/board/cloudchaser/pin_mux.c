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
    /* Ports */
    CLOCK_EnableClock(kCLOCK_PortA);                // UART0, JTAG
    CLOCK_EnableClock(BOARD_LED_ABCD_PORT_CLOCK);   // LED
    CLOCK_EnableClock(kCLOCK_PortC);                // SPI0, GPIO_RF1
    CLOCK_EnableClock(kCLOCK_PortD);                // SPI2, GPIO_RF2
    CLOCK_EnableClock(kCLOCK_PortE);                // GPIO_PA1, GPIO_PA1, GPIO_MULTI

    /* LED */
    CLOCK_EnableClock(BOARD_LED_ABCD_PORT_CLOCK);
    PORT_SetPinMux(BOARD_LED_ABCD_PORT, BOARD_LED_A_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_LED_ABCD_PORT, BOARD_LED_B_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_LED_ABCD_PORT, BOARD_LED_C_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_LED_ABCD_PORT, BOARD_LED_D_GPIO_PIN, kPORT_MuxAsGpio);
    LED_ABCD_INIT(A, LOGIC_LED_OFF);
    LED_ABCD_INIT(B, LOGIC_LED_OFF);
    LED_ABCD_INIT(C, LOGIC_LED_OFF);
    LED_ABCD_INIT(D, LOGIC_LED_OFF);

    /* JTAG pins */
/*    PORT_SetPinMux(PORTA, 0u, kPORT_MuxAlt7);       // JTAG_TCLK/SWD_CLK -- also default
    PORT_SetPinMux(PORTA, 1u, kPORT_MuxAlt7);       // JTAG_TDI -- also default
    PORT_SetPinMux(PORTA, 2u, kPORT_MuxAlt7);       // JTAG_TDO -- also default
    PORT_SetPinMux(PORTA, 3u, kPORT_MuxAlt7);       // JTAG_TMS/SWD_DIO -- also default
    PORT_SetPinMux(PORTA, 4u, kPORT_MuxAlt7);       // NMI_b -- also default -- 7
    PORT_SetPinMux(PORTA, 6u, kPORT_MuxAlt7);       // TRACE_CLKOUT -- also default
    PORT_SetPinMux(PORTA, 7u, kPORT_MuxAlt7);       // TRACE_D3 -- also default
    PORT_SetPinMux(PORTA, 8u, kPORT_MuxAlt7);       // TRACE_D2 -- also default
    PORT_SetPinMux(PORTA, 9u, kPORT_MuxAlt7);       // TRACE_D1 -- also default
    PORT_SetPinMux(PORTA, 10u, kPORT_MuxAlt7);      // TRACE_D0 -- also default*/

    /* UART0 */
    PORT_SetPinMux(PORTA, 14u, kPORT_MuxAlt3);      // UART0_TX
    PORT_SetPinMux(PORTA, 15u, kPORT_MuxAlt3);      // UART0_RX
    //PORT_SetPinMux(PORTA, 16u, kPORT_MuxAlt3);      // UART0_RTS_b
    //PORT_SetPinMux(PORTA, 17u, kPORT_MuxAlt3);      // UART0_CTS_b
    //PORT_SetPinMux(PORTB, 2u, kPORT_MuxAlt3);       // UART0_RTS_b
    //PORT_SetPinMux(PORTB, 3u, kPORT_MuxAlt3);       // UART0_CTS_b

    /* SPI0 */
    PORT_SetPinMux(PORTC,  4u, kPORT_MuxAlt2);      // SPI0_PCS0
    PORT_SetPinMux(PORTC,  5u, kPORT_MuxAlt2);      // SPI0_SCK
    PORT_SetPinMux(PORTC,  6u, kPORT_MuxAlt2);      // SPI0_SOUT
    PORT_SetPinMux(PORTC,  7u, kPORT_MuxAlt2);      // SPI0_SIN

    /* SPI2 */
    PORT_SetPinMux(PORTD, 11u, kPORT_MuxAlt2);      // SPI2_PCS0
    PORT_SetPinMux(PORTD, 12u, kPORT_MuxAlt2);      // SPI2_SCK
    PORT_SetPinMux(PORTD, 13u, kPORT_MuxAlt2);      // SPI2_SOUT
    PORT_SetPinMux(PORTD, 14u, kPORT_MuxAlt2);      // SPI2_SIN

    /* CC1200 (RF1) Interrupts */
    PORT_SetPinMux(PORTC, 10u, kPORT_MuxAsGpio);    // RF1_GPIO3
    PORT_SetPinMux(PORTC, 11u, kPORT_MuxAsGpio);    // RF1_GPIO2
    PORT_SetPinMux(PORTC, 12u, kPORT_MuxAsGpio);    // RF1_GPIO0

    /* CC1200 (RF2) Interrupts */
    PORT_SetPinMux(PORTD,  7u, kPORT_MuxAsGpio);    // RF2_GPIO3
    PORT_SetPinMux(PORTD,  8u, kPORT_MuxAsGpio);    // RF2_GPIO2
    PORT_SetPinMux(PORTD,  9u, kPORT_MuxAsGpio);    // RF2_GPIO0

    /* CC1190 (PA1) Control */
    PORT_SetPinMux(PORTE,  6u, kPORT_MuxAsGpio);    // PA1_HGM
    PORT_SetPinMux(PORTE,  8u, kPORT_MuxAsGpio);    // PA1_LNA_EN
    PORT_SetPinMux(PORTE,  9u, kPORT_MuxAsGpio);    // PA1_PA_EN

    /* CC1190 (PA2) Control */
    PORT_SetPinMux(PORTE,  7u, kPORT_MuxAsGpio);    // PA2_HGM
    PORT_SetPinMux(PORTE, 10u, kPORT_MuxAsGpio);    // PA2_LNA_EN
    PORT_SetPinMux(PORTE, 11u, kPORT_MuxAsGpio);    // PA2_PA_EN

    /* Multifunction GPIOs */
    PORT_SetPinMux(PORTE,  0u, kPORT_MuxAlt2);      // GPIO_HDR #6   0:ADC1_SE4a 2:SPI1_PCS1 3:UART1_TX    4:SDHC0_D1     6:I2C1_SDA 7:RTC_CLKOUT
    PORT_SetPinMux(PORTE,  1u, kPORT_MuxAlt2);      // GPIO_HDR #4   0:ADC1_SE5a 2:SPI1_SOUT 3:UART1_RX    4:SDHC0_D0     6:I2C1_SCL 7:SPI1_SIN
    PORT_SetPinMux(PORTE,  2u, kPORT_MuxAlt2);      // GPIO_HDR #3   0:ADC1_SE6a 2:SPI1_SCK  3:UART1_CTS_b 4:SDHC0_DCLK
    PORT_SetPinMux(PORTE,  3u, kPORT_MuxAlt2);      // GPIO_HDR #7   0:ADC1_SE7a 2:SPI1_SIN  3:UART1_RTS_b 4:SDHC0_CMD               7:SPI1_SOUT
    PORT_SetPinMux(PORTE,  4u, kPORT_MuxAsGpio);    // GPIO_HDR #5
    PORT_SetPinMux(PORTE,  5u, kPORT_MuxAsGpio);    // GPIO_HDR #2
    PORT_SetPinMux(PORTB,  4u, kPORT_MuxAsGpio);    // GPIO_HDR #8
    PORT_SetPinMux(PORTB,  5u, kPORT_MuxAsGpio);    // GPIO_HDR #9
}
