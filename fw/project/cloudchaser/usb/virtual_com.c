#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"

#include <stdio.h>
#include <stdlib.h>

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_cdc_acm.h"
#include "usb_device_ch9.h"

#include "composite.h"

#include <usr/type.h>
#include <kio/itm.h>

#include <MK66F18.h>
#include <fsl_sim.h>
#include <usr/serf.h>


#include "usb_device_descriptor.h"
#include "virtual_com.h"


#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0)
#include "usb_phy.h"
#endif
#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
#include "fsl_smc.h"
#endif

#include "pin_mux.h"



#define USB_VCOM_TASK_STACK_SIZE            (TASK_STACK_SIZE_MEDIUM / sizeof(StackType_t))
#define USB_VCOM_RX_TASK_STACK_SIZE         (TASK_STACK_SIZE_LARGE / sizeof(StackType_t))
#define USB_VCOM_TX_QUEUE_LEN               4
#define USB_VCOM_RX_QUEUE_LEN               4

typedef struct {
    size_t len;
    u8 *buf;

} usb_io_t;

static struct usb_vcom {
    xTaskHandle task;
    StaticTask_t task_static;
    StackType_t task_stack[USB_VCOM_TASK_STACK_SIZE];

    xTaskHandle rx_task;
    StaticTask_t rx_task_static;
    StackType_t rx_task_stack[USB_VCOM_RX_TASK_STACK_SIZE];

    xQueueHandle txq;
    StaticQueue_t txq_static;
    usb_io_t txq_buf[USB_VCOM_TX_QUEUE_LEN];

    xQueueHandle rxq;
    StaticQueue_t rxq_static;
    usb_io_t rxq_buf[USB_VCOM_RX_QUEUE_LEN];

    SemaphoreHandle_t txs;
    StaticSemaphore_t txs_static;

    u8 rx_buf[DATA_BUFF_SIZE];

    //volatile bool receiving;
    volatile s8 sending;

    u8 instance;

    USB_DATA_ALIGNMENT usb_cdc_acm_info_t acm;

} usb_vcom[USB_CDC_INSTANCE_COUNT]; // __attribute__((section(".heap")));

static vcom_rx_t vcom_rx;

usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param);
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);

/*******************************************************************************
* Variables
******************************************************************************/
extern usb_device_endpoint_struct_t g_cdcVcomDicEndpoints[USB_CDC_INSTANCE_COUNT][USB_CDC_VCOM_DIC_ENDPOINT_COUNT];

static usb_device_composite_struct_t *g_deviceComposite;

/* Line codinig of cdc device */
static uint8_t s_lineCoding[LINE_CODING_SIZE] = {
    /* E.g. 0x00,0xC2,0x01,0x00 : 0x0001C200 is 115200 bits per second */
    (LINE_CODING_DTERATE >> 0U) & 0x000000FFU,
    (LINE_CODING_DTERATE >> 8U) & 0x000000FFU,
    (LINE_CODING_DTERATE >> 16U) & 0x000000FFU,
    (LINE_CODING_DTERATE >> 24U) & 0x000000FFU,
    LINE_CODING_CHARFORMAT,
    LINE_CODING_PARITYTYPE,
    LINE_CODING_DATABITS};

/* Abstract state of cdc device */
static uint8_t s_abstractState[COMM_FEATURE_DATA_SIZE] = {(STATUS_ABSTRACT_STATE >> 0U) & 0x00FFU,
                                                          (STATUS_ABSTRACT_STATE >> 8U) & 0x00FFU};

/* Country code of cdc device */
static uint8_t s_countryCode[COMM_FEATURE_DATA_SIZE] = {(COUNTRY_SETTING >> 0U) & 0x00FFU,
                                                        (COUNTRY_SETTING >> 8U) & 0x00FFU};

/* CDC ACM information */

/*******************************************************************************
* Code
******************************************************************************/
/*!
 * @brief CDC class specific callback function.
 *
 * This function handles the CDC class specific requests.
 *
 * @param handle          The CDC ACM class handle.
 * @param event           The CDC ACM class event type.
 * @param param           The parameter of the class specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */


usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    uint32_t len;
    uint16_t *uartBitmap;
    uint8_t ep;
    usb_device_cdc_acm_request_param_struct_t *acmReqParam;
    usb_device_endpoint_callback_message_struct_t *epCbParam;
    acmReqParam = (usb_device_cdc_acm_request_param_struct_t *)param;
    epCbParam = (usb_device_endpoint_callback_message_struct_t *)param;

    uint8_t instance;

    for (instance = 0; instance < USB_CDC_INSTANCE_COUNT; ++instance) {
        if (g_deviceComposite->cdcVcom[instance].cdcAcmHandle == handle) break;
    }

    if (instance == USB_CDC_INSTANCE_COUNT) {
        itm_puts(0, "usb: instance not found\r\n");
        return kStatus_USB_Error;
    }

    usb_cdc_acm_info_t *acmInfo = &usb_vcom[instance].acm;

    switch (event)
    {
        case kUSB_DeviceCdcEventSendResponse:
        {
            if (usb_vcom[instance].sending) {
                --usb_vcom[instance].sending;
                xSemaphoreGive(usb_vcom[instance].txs);
            }

            if ((1 == g_deviceComposite->cdcVcom[instance].attach) && (1 == g_deviceComposite->cdcVcom[instance].startTransactions)) {
                error = kStatus_USB_Success;
            }
        }
        break;
        case kUSB_DeviceCdcEventRecvResponse:
        {
            if ((1 == g_deviceComposite->cdcVcom[instance].attach) && (1 == g_deviceComposite->cdcVcom[instance].startTransactions))
            {
                const u32 recv_size = epCbParam->length;

                if (recv_size && recv_size != UINT32_MAX) {

                    const usb_io_t io = {
                            .len = recv_size,
                            .buf = pvPortMalloc(recv_size)
                    };

                    assert(io.buf);
                    memcpy(io.buf, usb_vcom[instance].rx_buf, recv_size);

                    if (!xQueueSend(usb_vcom[instance].rxq, &io, pdMS_TO_TICKS(10))) {
                        itm_puts(0, "usb: rx queue fail\r\n");
                        vPortFree(io.buf);
                    }
                }

                error = kStatus_USB_Success;
            }
        }
        break;

        case kUSB_DeviceCdcEventSerialStateNotif:
            ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 0;
            error = kStatus_USB_Success;
            break;

        case kUSB_DeviceCdcEventSendEncapsulatedCommand:
            break;

        case kUSB_DeviceCdcEventGetEncapsulatedResponse:
            break;

        case kUSB_DeviceCdcEventSetCommFeature:
            if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == acmReqParam->setupValue)
            {
                if (1 == acmReqParam->isSetup)
                {
                    *(acmReqParam->buffer) = s_abstractState;
                }
                else
                {
                    *(acmReqParam->length) = 0;
                }
            }
            else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == acmReqParam->setupValue)
            {
                if (1 == acmReqParam->isSetup)
                {
                    *(acmReqParam->buffer) = s_countryCode;
                }
                else
                {
                    *(acmReqParam->length) = 0;
                }
            }
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventGetCommFeature:
            if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == acmReqParam->setupValue)
            {
                *(acmReqParam->buffer) = s_abstractState;
                *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
            }
            else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == acmReqParam->setupValue)
            {
                *(acmReqParam->buffer) = s_countryCode;
                *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
            }
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventClearCommFeature:
            break;
        case kUSB_DeviceCdcEventGetLineCoding:
            *(acmReqParam->buffer) = s_lineCoding;
            *(acmReqParam->length) = LINE_CODING_SIZE;
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventSetLineCoding:
        {
            if (1 == acmReqParam->isSetup)
            {
                *(acmReqParam->buffer) = s_lineCoding;
            }
            else
            {
                *(acmReqParam->length) = 0;
            }
        }
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventSetControlLineState:
        {
            error = kStatus_USB_Success;

            usb_vcom[instance].acm.dteStatus = (u8) acmReqParam->setupValue;

            /* activate/deactivate Tx carrier */
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION)
            {
                acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
            }
            else
            {
                acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
            }

            /* activate carrier and DTE */
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE)
            {
                acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
            }
            else
            {
                acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
            }

            /* Indicates to DCE if DTE is present or not */
            acmInfo->dtePresent = (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) ? true : false;

            /* Initialize the serial state buffer */
            acmInfo->serialStateBuf[0] = NOTIF_REQUEST_TYPE;                /* bmRequestType */
            acmInfo->serialStateBuf[1] = USB_DEVICE_CDC_NOTIF_SERIAL_STATE; /* bNotification */
            acmInfo->serialStateBuf[2] = 0x00;                              /* wValue */
            acmInfo->serialStateBuf[3] = 0x00;
            // phillip: this might be wrong sometimes? (this field = wIndex)
            acmInfo->serialStateBuf[4] = (u8)(acmReqParam->interfaceIndex & 0xFF);
            acmInfo->serialStateBuf[5] = (u8)(acmReqParam->interfaceIndex >> 8);
            acmInfo->serialStateBuf[6] = UART_BITMAP_SIZE; /* wLength */
            acmInfo->serialStateBuf[7] = 0x00;

            /* Lower byte of UART BITMAP */
            uartBitmap = (uint16_t *)&acmInfo->serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE - 2];
            *uartBitmap = acmInfo->uartState;


            if (0 == ((usb_device_cdc_acm_struct_t *)handle)->hasSentState) {
                len = (uint32_t)(NOTIF_PACKET_SIZE + UART_BITMAP_SIZE);

                ep = ((u8 []){USB_CDC_VCOM0_CIC_INTERRUPT_IN_ENDPOINT, USB_CDC_VCOM1_CIC_INTERRUPT_IN_ENDPOINT, USB_CDC_VCOM2_CIC_INTERRUPT_IN_ENDPOINT})[instance];

                error = USB_DeviceCdcAcmSend(handle, ep, acmInfo->serialStateBuf, len);

                if (kStatus_USB_Success != error)
                {
                    usb_echo("kUSB_DeviceCdcEventSetControlLineState error!");
                }
                ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 1;
            }


            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) {
                /* DTE_ACTIVATED */
                if (1 == g_deviceComposite->cdcVcom[instance].attach) {
                    g_deviceComposite->cdcVcom[instance].startTransactions = 1;
                }
            } else {
                /* DTE_DEACTIVATED */
                if (1 == g_deviceComposite->cdcVcom[instance].attach) {
                    g_deviceComposite->cdcVcom[instance].startTransactions = 0;
                }
            }

            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION) {
                /* CARRIER_ACTIVATED */
                ep = ((u8 []){USB_CDC_VCOM0_DIC_BULK_OUT_ENDPOINT, USB_CDC_VCOM1_DIC_BULK_OUT_ENDPOINT, USB_CDC_VCOM2_DIC_BULK_OUT_ENDPOINT})[instance];
                USB_DeviceCdcAcmRecv(handle, ep, usb_vcom[instance].rx_buf, g_cdcVcomDicEndpoints[instance][0].maxPacketSize);

            } else {
                /* TODO: CARRIER_DEACTIVATED */
            }
        }
        break;
        case kUSB_DeviceCdcEventSendBreak:
            break;
        default:
            break;
    }

    if (error) {
        itm_printf(0, "ev=%u er=%u\n", event, error);
    }

    return error;
}


static void usb_vcom_rx_task(void *param)
{
    const struct usb_vcom *const vcom = (struct usb_vcom *)param;
    const xQueueHandle rxq = vcom->rxq;
    const u8 instance = vcom->instance;
    class_handle_t const cdc = g_deviceComposite->cdcVcom[instance].cdcAcmHandle;
    const u8 ep = ((u8 []){USB_CDC_VCOM0_DIC_BULK_OUT_ENDPOINT, USB_CDC_VCOM1_DIC_BULK_OUT_ENDPOINT, USB_CDC_VCOM2_DIC_BULK_OUT_ENDPOINT})[instance];

    usb_io_t io;

    while (1) {
        if (xQueueReceive(rxq, &io, portMAX_DELAY)) {
            if (uxQueueMessagesWaiting(rxq) < USB_VCOM_RX_QUEUE_LEN) {
                if ((1 == g_deviceComposite->cdcVcom[instance].attach) && (1 == g_deviceComposite->cdcVcom[instance].startTransactions)) {
                    USB_DeviceCdcAcmRecv(cdc, ep, usb_vcom[instance].rx_buf, g_cdcVcomDicEndpoints[instance][0].maxPacketSize);
                }
            }

            vcom_rx(instance, io.len, io.buf);
            vPortFree(io.buf);
        }
    }
}


static void usb_vcom_task(void *param)
{
    struct usb_vcom *const vcom = (struct usb_vcom *)param;
    const u8 instance = vcom->instance;
    class_handle_t const cdc = g_deviceComposite->cdcVcom[instance].cdcAcmHandle;
    const u8 ep = ((u8[]){USB_CDC_VCOM0_DIC_BULK_IN_ENDPOINT, USB_CDC_VCOM1_DIC_BULK_IN_ENDPOINT, USB_CDC_VCOM2_DIC_BULK_IN_ENDPOINT})[instance];

    usb_status_t error = kStatus_USB_Error;

    usb_io_t io;

    vcom->sending = 0;

    while (1) {
        if (!xQueueReceive(vcom->txq, &io, portMAX_DELAY)) continue;
        ++vcom->sending;
        //itm_printf(0, "<itm> usb: send io=0x%08X size=%lu receiving=%u\n", io, io.len, receiving);

        if ((error = USB_DeviceCdcAcmSend(cdc, ep, io.buf, io.len))) {
            --vcom->sending;
            itm_printf(0, "usb: tx error=%u\r\n", error);

            vPortFree(io.buf);
            io.buf = NULL;
            continue;

        }

        if (!xSemaphoreTake(vcom->txs, portMAX_DELAY /*pdMS_TO_TICKS(100)*/)) {
            //error = USB_DeviceCdcAcmSendCancel(cdc, ep);
            --vcom->sending;
            itm_printf(0, "usb: tx timeout, len=%u\r\n", io.len);
        }

        vPortFree(io.buf);
        io.buf = NULL;
    }
}


bool vcom_init(vcom_rx_t rx_cb)
{
    memset(usb_vcom, 0, sizeof(usb_vcom));
    vcom_rx = rx_cb;

    for (u8 i = 0; i < USB_CDC_INSTANCE_COUNT; ++i) {
        usb_vcom[i].instance = i;

        usb_vcom[i].txq = xQueueCreateStatic(USB_VCOM_TX_QUEUE_LEN, sizeof(usb_vcom[i].txq_buf[0]), (u8 *)usb_vcom[i].txq_buf, &usb_vcom[i].txq_static);
        usb_vcom[i].rxq = xQueueCreateStatic(USB_VCOM_RX_QUEUE_LEN, sizeof(usb_vcom[i].rxq_buf[0]), (u8 *)usb_vcom[i].rxq_buf, &usb_vcom[i].rxq_static);
        usb_vcom[i].txs = xSemaphoreCreateBinaryStatic(&usb_vcom[i].txs_static);
        //xSemaphoreGive(usb_vcom[i].txs);
        usb_vcom[i].task = xTaskCreateStatic(usb_vcom_task, "usb:tx", USB_VCOM_TASK_STACK_SIZE, &usb_vcom[i], TASK_PRIO_HIGH - 1 - i, usb_vcom[i].task_stack, &usb_vcom[i].task_static); assert(usb_vcom[i].task);
        usb_vcom[i].rx_task = xTaskCreateStatic(usb_vcom_rx_task, "usb:rx", USB_VCOM_RX_TASK_STACK_SIZE, &usb_vcom[i], TASK_PRIO_HIGH - 2 - i, usb_vcom[i].rx_task_stack, &usb_vcom[i].rx_task_static); assert(usb_vcom[i].rx_task);
    }

    return true;
}

static inline bool isInterrupt()
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ;
}

bool usb_attached(u8 port)
{
    return port < USB_CDC_INSTANCE_COUNT && g_deviceComposite->cdcVcom[port].attach && g_deviceComposite->cdcVcom[port].startTransactions;
}

void usb_write(u8 port, u8 *buf, size_t len)
{
    if (!usb_attached(port) || !buf) return;

    u8 *frame = NULL;
    size_t size = serf_encode(0x00, buf, len, &frame);

    if (frame) {
        usb_write_direct(port, frame, size);
    }
}

void usb_write_direct(u8 port, u8 *buf, size_t len)
{
    if (!usb_attached(port) || !buf) {
        if (buf) vPortFree(buf);
        return;
    }

    usb_io_t io = { len, buf };

    //itm_printf(0, "<itm> usb: queu io=0x%08X size=%lu\n", io, io->len);

    if (!isInterrupt()) {
        if (!xQueueSend(usb_vcom[port].txq, &io, pdMS_TO_TICKS(1000))) {
            itm_puts(0, "usb: queue failed\r\n");
            vPortFree(buf);
        }
    } else {
        if (!xQueueSendFromISR(usb_vcom[port].txq, &io, NULL)) {
            itm_puts(0, "usb: queue failed\r\n");
            vPortFree(buf);
        }
    }
}


/*!
 * @brief Virtual COM device set configuration function.
 *
 * This function sets configuration for CDC class.
 *
 * @param handle The CDC ACM class handle.
 * @param configure The CDC ACM class configure index.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCdcVcomSetConfigure(class_handle_t handle, uint8_t configure)
{
    if (USB_COMPOSITE_CONFIGURE_INDEX == configure)
    {
        u8 instance;
        for (instance = 0; instance < USB_CDC_INSTANCE_COUNT; ++instance) {
            if (g_deviceComposite->cdcVcom[instance].cdcAcmHandle == handle) {
                break;
            }
        }

        const u8 ep = ((u8[]){USB_CDC_VCOM0_DIC_BULK_OUT_ENDPOINT, USB_CDC_VCOM1_DIC_BULK_OUT_ENDPOINT, USB_CDC_VCOM2_DIC_BULK_OUT_ENDPOINT})[instance];

        //itm_printf(0, "usb: attach com #%i p=%u\r\n", instance, g_deviceComposite->cdcVcom[instance].attach);

        g_deviceComposite->cdcVcom[instance].attach = 1;
        /* Schedule buffer for receive */
        USB_DeviceCdcAcmRecv(g_deviceComposite->cdcVcom[instance].cdcAcmHandle, ep,
                             usb_vcom[instance].rx_buf, g_cdcVcomDicEndpoints[instance][0].maxPacketSize);
    }
    return kStatus_USB_Success;
}

/*!
 * @brief Virtual COM device initialization function.
 *
 * This function initializes the device with the composite device class information.
 *
 * @param deviceComposite The pointer to the composite device structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCdcVcomInit(usb_device_composite_struct_t *deviceComposite)
{
    g_deviceComposite = deviceComposite;
    return kStatus_USB_Success;
}
