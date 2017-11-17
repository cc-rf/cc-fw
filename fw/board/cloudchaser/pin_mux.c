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
    CLOCK_EnableClock(kCLOCK_PortB);                // UART/GPIO, LED
    CLOCK_EnableClock(kCLOCK_PortC);                // SPI0, GPIO_RF1
    CLOCK_EnableClock(kCLOCK_PortD);                // SPI2, GPIO_RF2, I2C0
    CLOCK_EnableClock(kCLOCK_PortE);                // GPIO_PA1, GPIO_PA2, GPIO_MULTI

    /* LED */
    #if BOARD_REVISION == 1

        const port_pin_config_t port_pin_config = {
                kPORT_PullDisable,
                kPORT_FastSlewRate,
                kPORT_PassiveFilterDisable,
                kPORT_OpenDrainDisable,
                kPORT_LowDriveStrength,
                kPORT_MuxAsGpio,
                kPORT_LockRegister,
        };

        const gpio_pin_config_t gpio_pin_config = {
                .pinDirection = kGPIO_DigitalOutput,
                .outputLogic = 0
        };

        PORT_SetPinConfig(BOARD_LED_PORT, BOARD_LED_GPIO_0, &port_pin_config);  // LED_GREEN_0
        PORT_SetPinConfig(BOARD_LED_PORT, BOARD_LED_GPIO_1, &port_pin_config);  // LED_GREEN_1
        PORT_SetPinConfig(BOARD_LED_PORT, BOARD_LED_GPIO_2, &port_pin_config);  // LED_GREEN_2
        PORT_SetPinConfig(BOARD_LED_PORT, BOARD_LED_GPIO_3, &port_pin_config);  // LED_GREEN_3

        GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_0, &gpio_pin_config);
        GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_1, &gpio_pin_config);
        GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_2, &gpio_pin_config);
        GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_3, &gpio_pin_config);

    #elif BOARD_REVISION == 2

        const port_pin_config_t port_pin_config = {
                kPORT_PullDisable,
                kPORT_FastSlewRate,
                kPORT_PassiveFilterDisable,
                kPORT_OpenDrainEnable,
                kPORT_LowDriveStrength,
                kPORT_MuxAsGpio,
                kPORT_LockRegister,
        };

        const gpio_pin_config_t gpio_pin_config = {
                .pinDirection = kGPIO_DigitalOutput,
                .outputLogic = 0
        };

        PORT_SetPinConfig(BOARD_LED_PORT, BOARD_LED_GPIO_0, &port_pin_config);  // LED_BLUE_0
        PORT_SetPinConfig(BOARD_LED_PORT, BOARD_LED_GPIO_1, &port_pin_config);  // LED_BLUE_1

        GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_0, &gpio_pin_config);
        GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_1, &gpio_pin_config);

    #endif

    /* LP5562 LED x2 on I2C0 */
    #if BOARD_REVISION == 2

    const port_pin_config_t port_pin_config_i2c0 = {
            .pullSelect = kPORT_PullUp,
            .slewRate = kPORT_FastSlewRate,
            .passiveFilterEnable = kPORT_PassiveFilterDisable,
            .openDrainEnable = kPORT_OpenDrainEnable,
            .driveStrength = kPORT_LowDriveStrength,
            .mux = kPORT_MuxAlt7,
            .lockRegister = kPORT_LockRegister,
    };


    PORT_SetPinConfig(PORTD, 2u, &port_pin_config_i2c0);      // I2C0_SCL
    PORT_SetPinConfig(PORTD, 3u, &port_pin_config_i2c0);      // I2C0_SDA
    #endif

    /* UART0 */
    PORT_SetPinMux(PORTA, 14u, kPORT_MuxAlt3);      // UART_HDR #3      UART0_TX
    PORT_SetPinMux(PORTA, 15u, kPORT_MuxAlt3);      // UART_HDR #5      UART0_RX
    #if BOARD_REVISION == 1
    PORT_SetPinMux(PORTA, 16u, kPORT_MuxAlt3);      // UART_HDR #4      UART0_RTS_b
    PORT_SetPinMux(PORTA, 17u, kPORT_MuxAlt3);      // UART_HDR #6      UART0_CTS_b
    #endif
    #if BOARD_REVISION == 2
    PORT_SetPinMux(PORTB,  2u, kPORT_MuxAlt3);      // UART_HDR #X      UART0_RTS_b
    PORT_SetPinMux(PORTB,  3u, kPORT_MuxAlt3);      // UART_HDR #X      UART0_CTS_b
    #endif

    /* SPI0 */
    PORT_SetPinMux(PORTC,  4u, kPORT_MuxAlt2);      // SPI0_PCS0
    PORT_SetPinMux(PORTC,  5u, kPORT_MuxAlt2);      // SPI0_SCK
    PORT_SetPinMux(PORTC,  6u, kPORT_MuxAlt2);      // SPI0_SOUT
    PORT_SetPinMux(PORTC,  7u, kPORT_MuxAlt2);      // SPI0_SIN

    #if BOARD_REVISION <= 1
    /* SPI2 */
    PORT_SetPinMux(PORTD, 11u, kPORT_MuxAlt2);      // SPI2_PCS0
    PORT_SetPinMux(PORTD, 12u, kPORT_MuxAlt2);      // SPI2_SCK
    PORT_SetPinMux(PORTD, 13u, kPORT_MuxAlt2);      // SPI2_SOUT
    PORT_SetPinMux(PORTD, 14u, kPORT_MuxAlt2);      // SPI2_SIN
    #endif

    /* CC1200 (RF1) Interrupts */
    PORT_SetPinMux(PORTC, 10u, kPORT_MuxAsGpio);    // RF1_GPIO3
    PORT_SetPinMux(PORTC, 11u, kPORT_MuxAsGpio);    // RF1_GPIO2
    PORT_SetPinMux(PORTC, 12u, kPORT_MuxAsGpio);    // RF1_GPIO0

    #if BOARD_REVISION <= 1
    /* CC1200 (RF2) Interrupts */
    PORT_SetPinMux(PORTD,  7u, kPORT_MuxAsGpio);    // RF2_GPIO3
    PORT_SetPinMux(PORTD,  8u, kPORT_MuxAsGpio);    // RF2_GPIO2
    PORT_SetPinMux(PORTD,  9u, kPORT_MuxAsGpio);    // RF2_GPIO0
    #endif

    /* CC1190 (PA1) Control */
    PORT_SetPinMux(PORTE,  6u, kPORT_MuxAsGpio);    // PA1_HGM
    PORT_SetPinMux(PORTE,  8u, kPORT_MuxAsGpio);    // PA1_LNA_EN
    PORT_SetPinMux(PORTE,  9u, kPORT_MuxAsGpio);    // PA1_PA_EN

    #if BOARD_REVISION <= 1
    /* CC1190 (PA2) Control */
    PORT_SetPinMux(PORTE,  7u, kPORT_MuxAsGpio);    // PA2_HGM
    PORT_SetPinMux(PORTE, 10u, kPORT_MuxAsGpio);    // PA2_LNA_EN
    PORT_SetPinMux(PORTE, 11u, kPORT_MuxAsGpio);    // PA2_PA_EN

    /* Multifunction GPIOs */
    PORT_SetPinMux(PORTE,  0u, kPORT_MuxAsGpio);                // GPIO_HDR #6   ** uflag2 pwr          0:ADC1_SE4a 2:SPI1_PCS1 3:UART1_TX    4:SDHC0_D1     6:I2C1_SDA 7:RTC_CLKOUT
    PORT_SetPinMux(PORTE,  1u, kPORT_MuxAlt2);                  // GPIO_HDR #4                          0:ADC1_SE5a 2:SPI1_SOUT 3:UART1_RX    4:SDHC0_D0     6:I2C1_SCL 7:SPI1_SIN
    PORT_SetPinMux(PORTE,  2u, kPORT_MuxAlt2);                  // GPIO_HDR #3                          0:ADC1_SE6a 2:SPI1_SCK  3:UART1_CTS_b 4:SDHC0_DCLK
    PORT_SetPinMux(PORTE,  3u, kPORT_MuxAlt2);                  // GPIO_HDR #7                          0:ADC1_SE7a 2:SPI1_SIN  3:UART1_RTS_b 4:SDHC0_CMD               7:SPI1_SOUT
    PORT_SetPinMux(PORTE,  4u, kPORT_MuxAlt2);                  // GPIO_HDR #5   ** uflag2 input                    2:SPI1_PCS0 3:UART3_TX
    PORT_SetPinMux(PORTE,  5u, kPORT_PinDisabledOrAnalog);      // GPIO_HDR #2                                      2:SPI1_PCS2 3:UART3_TX
    PORT_SetPinMux(PORTB,  4u, kPORT_MuxAsGpio);                // GPIO_HDR #8   ** extra pwr
    PORT_SetPinMux(PORTB,  5u, kPORT_MuxAsGpio);                // GPIO_HDR #9   ** pflag input
    #endif

    /**
     * J6 #2 PTD10 B2
     * J6 #3 PTD12 B1   3:FTM3_FLT0
     * J6 #4 PTD13 C3
     * J6 #5 PTD14 C2
     * J6 #6 PTD7  A1   4:FTM0_CH7  6:FTM0_FLT1
     * J6 #7 PTD8  C9
     * J6 #8 PTD9  B9
     *
     * J5 #3 PTD2       4:FTM3_CH2
     * J5 #4 PTD3       4:FTM3_CH3
     * J5 #5 PTE24      3:UART4_TX
     * J5 #6 PTE25      3:UART4_RX
     * J5 #7 PTA11      3:FTM2_CH1  6:FTM2_QD_PHB/TPM2_CH1
     * J5 #8 PTA12      3:FTM1_CH0  7:FTM1_QD_PHA/TPM1_CH0
     * J5 #9 PTE11      6:FTM3_CH6
     *
     * J3 #9 PTE5       6:FTM3_CH0
     *
     *
     */
}
