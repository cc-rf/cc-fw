/**
 * Support for the HS USB Device Charge Detect (DCD) Module.
 *
 * The goal here is primarily to support the data contact detect function,
 * which comes after vbus detection and is completed after debouncing has
 * been performed on the data lines. Afterwards, and the part that is not
 * supported here, is the charging port detection although it may be
 * useful in the future if we can also detect USB batteries.
 */
#include <uhdcd.h>
#include <usr/type.h>
#include <fsl_device_registers.h>
#include <MK66F18.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <assert.h>
#include <itm.h>
#include <virtual_com.h>

typedef struct __packed {
    uhdcd_error_t error;
    uhdcd_state_t state;

} uhdcd_t;

static xQueueHandle queue = NULL;

uhdcd_error_t uhdcd_run(uhdcd_state_t *state)
{
    if (!queue) {
        queue = xQueueCreate(1, sizeof(uhdcd_t)); assert(queue);
    }

    CLOCK_EnableClock(kCLOCK_UsbhsDcd0);

    USBHSDCD->CONTROL |= USBHSDCD_CONTROL_SR(1); // Software Reset

    USBHSDCD->CLOCK = USBHSDCD_CLOCK_CLOCK_SPEED(CLOCK_GetCoreSysClkFreq() / 1000000)
            | USBHSDCD_CLOCK_CLOCK_UNIT(1);

    // Configure clock and timing as needed... (TODO)

    USBHSDCD->CONTROL |= USBHSDCD_CONTROL_IE(1); // Enable Interrupts

    //USBHSDCD->CONTROL |= USBHSDCD_CONTROL_BC12(x); // Select Charging Specification Revision

    // Set USB PHY Pulldown Flag
    u32 pd = (USBPHY->ANACTRL & USBPHY_ANACTRL_DEV_PULLDOWN_MASK) >> USBPHY_ANACTRL_DEV_PULLDOWN_SHIFT;

    if (!pd) {
        itm_puts(0, "usb: set pd for dcd\n");
        USBPHY->ANACTRL |= USBPHY_ANACTRL_DEV_PULLDOWN(1);
    }

    // Enable Interrupt
    NVIC_SetPriority(USBHSDCD_IRQn, USB_DEVICE_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(USBHSDCD_IRQn);

    USBHSDCD->CONTROL |= USBHSDCD_CONTROL_START(1); // Start Operation

    uhdcd_t result = { UHDCD_ERROR_FAIL, UHDCD_STATE_NONE };

    if (xQueueReceive(queue, &result, portMAX_DELAY)) {
        //USBHSDCD->CONTROL &= ~USBHSDCD_CONTROL_SR(0); // Software Reset
    }

    // TODO: Needs more cleanup logic

    if (!pd) {
        USBPHY->ANACTRL &= ~USBPHY_ANACTRL_DEV_PULLDOWN(1);
    }

    *state = result.state;
    return  result.error;
}


__attribute__((interrupt,noclone)) void USBHSDCD_IRQHandler(void)
{
    u32 status = USBHSDCD->STATUS;

    USBHSDCD->CONTROL |= USBHSDCD_CONTROL_IACK(1); // Acknowledge Interrupt

    uhdcd_t result = { UHDCD_ERROR_NONE, UHDCD_STATE_NONE };

    if (status & USBHSDCD_STATUS_ERR(1)) {
        result.error = UHDCD_ERROR_FAIL;

        if (status & USBHSDCD_STATUS_TO(1)) {
            result.error |= UHDCD_ERROR_TIME;
        }
    }

    result.state = (uhdcd_state_t)((status & USBHSDCD_STATUS_SEQ_STAT_MASK) >> USBHSDCD_STATUS_SEQ_STAT_SHIFT);

    itm_printf(0, "uhdcd: error=%u state=%u\r\n", result.error, result.state);

    xQueueSendFromISR(queue, &result, NULL);
}
