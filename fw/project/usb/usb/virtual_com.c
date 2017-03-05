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
//#include "fsl_debug_console.h"
#include <usr/type.h>
#include <itm.h>
#include <uhdcd.h>
#include <MK66F18.h>
#include <fsl_sim.h>


#include "usb_device_descriptor.h"
#include "virtual_com.h"
#if (defined(FSL_FEATURE_SOC_MPU_COUNT) && (FSL_FEATURE_SOC_MPU_COUNT > 0U))
#include "fsl_mpu.h"
#endif /* FSL_FEATURE_SOC_MPU_COUNT */
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0)
#include "usb_phy.h"
#endif
#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
#include "fsl_smc.h"
#endif

#include "pin_mux.h"



#define USB_VCOM_TASK_STACK_SIZE            (TASK_STACK_SIZE_DEFAULT / sizeof(StackType_t))
#define USB_VCOM_RX_TASK_STACK_SIZE         (TASK_STACK_SIZE_LARGE / sizeof(StackType_t))
#define USB_VCOM_DEVICE_TASK_STACK_SIZE     (TASK_STACK_SIZE_LARGE / sizeof(StackType_t))
#define USB_VCOM_TX_QUEUE_LEN               12
#define USB_VCOM_RX_QUEUE_LEN               12

typedef struct {
    size_t len;
    u8 *buf;

} usb_io_t;

static struct {
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

    #if USB_DEVICE_CONFIG_USE_TASK
        xTaskHandle device_task;
        StaticTask_t device_task_static;
        StackType_t device_task_stack[USB_VCOM_DEVICE_TASK_STACK_SIZE];
    #endif

} usb_vcom; // __attribute__((section(".heap")));


static volatile bool flush_needed = false;


/*******************************************************************************
* Definitions
******************************************************************************/
/* USB clock source and frequency*/
#define USB_FS_CLK_SRC kCLOCK_UsbSrcIrc48M
#define USB_FS_CLK_FREQ 48000000U
#define USB_HS_PHY_CLK_SRC kCLOCK_UsbPhySrcExt
#define USB_HS_PHY_CLK_FREQ BOARD_XTAL0_CLK_HZ
#define USB_HS_CLK_SRC kCLOCK_UsbSrcUnused
#define USB_HS_CLK_FREQ 0U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);
usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param);
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);

/*******************************************************************************
* Variables
******************************************************************************/
extern usb_device_endpoint_struct_t g_UsbDeviceCdcVcomDicEndpoints[];
extern usb_device_class_struct_t g_UsbDeviceCdcVcomConfig;
/* Data structure of virtual com device */
static usb_cdc_vcom_struct_t s_cdcVcom;

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
USB_DATA_ALIGNMENT static usb_cdc_acm_info_t s_usbCdcAcmInfo = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0, 0, 0};
/* Data buffer for receiving and sending*/
USB_DATA_ALIGNMENT static uint8_t s_currRecvBuf[DATA_BUFF_SIZE];
volatile static uint32_t s_recvSize = 0;

/* USB device class information */
static usb_device_class_config_struct_t s_cdcAcmConfig[1] = {{
    USB_DeviceCdcVcomCallback, 0, &g_UsbDeviceCdcVcomConfig,
}};

/* USB device class configuraion information */
static usb_device_class_config_list_struct_t s_cdcAcmConfigList = {
    s_cdcAcmConfig, USB_DeviceCallback, 1,
};

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
volatile static uint8_t s_waitForDataReceive = 0;
volatile static uint8_t s_comOpen = 0;
#endif



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

static volatile bool receiving = false;
static volatile bool sending = false;

usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    uint32_t len;
    uint16_t *uartBitmap;
    usb_cdc_acm_info_t *acmInfo = &s_usbCdcAcmInfo;
    usb_device_cdc_acm_request_param_struct_t *acmReqParam;
    usb_device_endpoint_callback_message_struct_t *epCbParam;
    acmReqParam = (usb_device_cdc_acm_request_param_struct_t *)param;
    epCbParam = (usb_device_endpoint_callback_message_struct_t *)param;

    //itm_printf(0, "<usb> cdc cb: event=%lu\r\n", event);

    switch (event)
    {
        case kUSB_DeviceCdcEventSendResponse:
        {
            if ((epCbParam->length != 0) && (!(epCbParam->length % g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize)))
            {
                /* If the last packet is the size of endpoint, then send also zero-ended packet,
                 ** meaning that we want to inform the host that we do not have any additional
                 ** data, so it can flush the output.
                 */
                error = USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_BULK_IN_ENDPOINT, NULL, 0);
            }
            else if ((1 == s_cdcVcom.attach) && (1 == s_cdcVcom.startTransactions))
            {
                if ((epCbParam->buffer != NULL) || ((epCbParam->buffer == NULL) && (epCbParam->length == 0)))
                {

                    //itm_printf(0, "<itm> usb: sent. receiving=%u\r\n", receiving);

                    if (!receiving) {
                        receiving = true;
                        /* User: add your own code for send complete event */
                        /* Schedule buffer for next receive event */
                        error = USB_DeviceCdcAcmRecv(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                                                     g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize);
#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) && \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
                        s_waitForDataReceive = 1;
                        USB0->INTEN &= ~USB_INTEN_SOFTOKEN_MASK;
#endif
                    }

                    if (uxSemaphoreGetCount(usb_vcom.txs) && !uxQueueMessagesWaiting(usb_vcom.txq)) {
                        // TODO: test this and determine if it is needed
                        //itm_puts(0, "<itm> usb: post-tx flush\r\n");
                        error = USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_BULK_IN_ENDPOINT, NULL, 0);
                    }

                    if (sending) {
                        sending = false;
                        xSemaphoreGive(usb_vcom.txs);
                    }
                }
            }
        }
        break;
        case kUSB_DeviceCdcEventRecvResponse:
        {
            receiving = false;

            if ((1 == s_cdcVcom.attach) && (1 == s_cdcVcom.startTransactions))
            {
                s_recvSize = epCbParam->length;

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
                s_waitForDataReceive = 0;
                USB0->INTEN |= USB_INTEN_SOFTOKEN_MASK;
#endif

                if (s_recvSize) {
                    //flush_needed = true;

                    if (s_recvSize != UINT32_MAX) {
                        // phillip: copy out to queue
                        const usb_io_t io = {
                                .len = s_recvSize,
                                .buf = malloc(s_recvSize)
                        };

                        if (s_recvSize) {
                            assert(io.buf);
                            memcpy(io.buf, s_currRecvBuf, s_recvSize);
                        }

                        //itm_printf(0, "usb: queue %u bytes\r\n", s_recvSize);
                        if (!xQueueSend(usb_vcom.rxq, &io, portMAX_DELAY/*pdMS_TO_TICKS(10)*/)) {
                            itm_puts(0, "usb: rx queue fail\r\n");
                            free(io.buf);
                        }

                        s_recvSize = 0;
                    }


                    //if (uxSemaphoreGetCount(usb_tx_s) && !uxQueueMessagesWaiting(usb_tx_q)) {
                    //    // TODO: test this and determine if it is needed
                    //    //itm_puts(0, "<itm> usb: post-rx flush\r\n");
                    //    error = USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_BULK_IN_ENDPOINT, NULL, 0);
                    //}
                }

                if (!receiving) {
                    receiving = true;
                    /* Schedule buffer for next receive event */
                    error = USB_DeviceCdcAcmRecv(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                                                 g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize);
#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
                    s_waitForDataReceive = 1;
                    USB0->INTEN &= ~USB_INTEN_SOFTOKEN_MASK;
#endif
                }
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
            s_usbCdcAcmInfo.dteStatus = acmReqParam->setupValue;
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
            acmInfo->serialStateBuf[4] = 0x00; /* wIndex */
            acmInfo->serialStateBuf[5] = 0x00;
            acmInfo->serialStateBuf[6] = UART_BITMAP_SIZE; /* wLength */
            acmInfo->serialStateBuf[7] = 0x00;
            /* Notifiy to host the line state */
            acmInfo->serialStateBuf[4] = acmReqParam->interfaceIndex;
            /* Lower byte of UART BITMAP */
            uartBitmap = (uint16_t *)&acmInfo->serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE - 2];
            *uartBitmap = acmInfo->uartState;
            len = (uint32_t)(NOTIF_PACKET_SIZE + UART_BITMAP_SIZE);
            if (0 == ((usb_device_cdc_acm_struct_t *)handle)->hasSentState)
            {
                error = USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT, acmInfo->serialStateBuf, len);
                if (kStatus_USB_Success != error)
                {
                    usb_echo("kUSB_DeviceCdcEventSetControlLineState error!");
                }
                ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 1;
            }

            /* Update status */
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION)
            {
                /*  To do: CARRIER_ACTIVATED */
            }
            else
            {
                /* To do: CARRIER_DEACTIVATED */
            }
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE)
            {
                /* DTE_ACTIVATED */
                if (1 == s_cdcVcom.attach)
                {
                    s_cdcVcom.startTransactions = 1;
#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
                    s_waitForDataReceive = 1;
                    USB0->INTEN &= ~USB_INTEN_SOFTOKEN_MASK;
                    s_comOpen = 1;
                    usb_echo("USB_APP_CDC_DTE_ACTIVATED\r\n");
#endif

                    // phillip: start receive?
                    /*if (!receiving) {
                        receiving = true;
                        error = USB_DeviceCdcAcmRecv(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                                                     g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize);
                        if (error) {
                            itm_printf(0, "<itm> usb: receive kickoff failed: error=%u\r\n", error);
                        }
                    }*/

                    // phillip: we are receiving already at this point, I believe...
                    receiving = true;
                }
            }
            else
            {
                /* DTE_DEACTIVATED */
                if (1 == s_cdcVcom.attach)
                {
                    s_cdcVcom.startTransactions = 0;
                    receiving = false;
                }
            }
        }
        break;
        case kUSB_DeviceCdcEventSendBreak:
            break;
        default:
            break;
    }

    return error;
}

/*!
 * @brief USB device callback function.
 *
 * This function handles the usb device specific requests.
 *
 * @param handle          The USB device handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the device specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    uint16_t *temp16 = (uint16_t *)param;
    uint8_t *temp8 = (uint8_t *)param;

    //itm_printf(0, "<usb> device cb: event=%lu\r\n", event);

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            s_cdcVcom.attach = 0;
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0)
            if (kStatus_USB_Success == USB_DeviceClassGetSpeed(CONTROLLER_ID, &s_cdcVcom.speed))
            {
                USB_DeviceSetSpeed(handle, s_cdcVcom.speed);
            }
#endif
        }
        break;
        case kUSB_DeviceEventSetConfiguration:
            if (param)
            {
                s_cdcVcom.currentConfiguration = *temp8;
                s_cdcVcom.attach = 1;
                if (!receiving && USB_CDC_VCOM_CONFIGURE_INDEX == (*temp8))
                {
                    /* Schedule buffer for receive */
                    USB_DeviceCdcAcmRecv(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                                         g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize);
                }
            }
            break;
        case kUSB_DeviceEventSetInterface:
            if (s_cdcVcom.attach)
            {
                uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FFU);
                if (interface < USB_CDC_VCOM_INTERFACE_COUNT)
                {
                    s_cdcVcom.currentInterfaceAlternateSetting[interface] = alternateSetting;
                }
            }
            break;
        case kUSB_DeviceEventGetConfiguration:
            break;
        case kUSB_DeviceEventGetInterface:
            break;
        case kUSB_DeviceEventGetDeviceDescriptor:
            if (param)
            {
                error = USB_DeviceGetDeviceDescriptor(handle, (usb_device_get_device_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetConfigurationDescriptor:
            if (param)
            {
                error = USB_DeviceGetConfigurationDescriptor(handle,
                                                             (usb_device_get_configuration_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetStringDescriptor:
            if (param)
            {
                /* Get device string descriptor request */
                error = USB_DeviceGetStringDescriptor(handle, (usb_device_get_string_descriptor_struct_t *)param);
            }
            break;
        default:
            break;
    }

    return error;
}

/*!
 * @brief USB Interrupt service routine.
 *
 * This function serves as the USB interrupt service routine.
 *
 * @return None.
 */
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
void USBHS_IRQHandler(void)
{
    USB_DeviceEhciIsrFunction(s_cdcVcom.deviceHandle);
}
#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
void USB0_IRQHandler(void)
{
    USB_DeviceKhciIsrFunction(s_cdcVcom.deviceHandle);
}
#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
void USB0_IRQHandler(void)
{
    USB_DeviceLpcIp3511IsrFunction(s_cdcVcom.deviceHandle);
}
#endif

/*!
 * @brief Application initialization function.
 *
 * This function initializes the application.
 *
 * @return None.
 */
void USB_DeviceApplicationInit(void)
{
    uint8_t irqNo;

    SystemCoreClockUpdate();

#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0)
    uint8_t ehciIrq[] = USBHS_IRQS;
    irqNo = ehciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];

    // TODO: Test using PFD clock here for hub detection & enumeration
    //   This maps to "external PLL" in fsl_clock
    //CLOCK_EnableUsbhs0PfdClock(24U, kCLOCK_UsbPfdSrcFracDivBy2);

    // NOTE: Perhaps do not enable manually
    //CLOCK_EnableUsbfs0Clock(USB_FS_CLK_SRC, USB_FS_CLK_FREQ); // added but probably not needed

    //SIM_SetUsbVoltRegulatorEnableMode(0);
    //SIM_SetUsbVoltRegulatorEnableMode(kSIM_UsbVoltRegEnableInAllModes);

    CLOCK_EnableUsbhs0PhyPllClock(USB_HS_PHY_CLK_SRC, USB_HS_PHY_CLK_FREQ);
    CLOCK_EnableUsbhs0Clock(USB_HS_CLK_SRC, USB_HS_CLK_FREQ);

    // Disable pullup and wait?
/*    itm_printf(0, "enable pullup [0]: val=%lu\n", USBHS->OTGSC & USBHS_OTGSC_DP_MASK);
    USBHS->OTGSC |= USBHS_OTGSC_DP_MASK;
    itm_printf(0, "enable pullup [1]: val=%lu\n", USBHS->OTGSC & USBHS_OTGSC_DP_MASK);
    vTaskDelay(pdMS_TO_TICKS(10));
    itm_printf(0, "disable pullup [0]: val=%lu\n", USBHS->OTGSC & USBHS_OTGSC_DP_MASK);
    USBHS->OTGSC &= ~USBHS_OTGSC_DP_MASK;
    itm_printf(0, "disable pullup [1]: val=%lu\n", USBHS->OTGSC & USBHS_OTGSC_DP_MASK);*/

    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ);

#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0)
    uint8_t khciIrq[] = USB_IRQS;
    irqNo = khciIrq[CONTROLLER_ID - kUSB_ControllerKhci0];

    CLOCK_EnableUsbfs0Clock(USB_FS_CLK_SRC, USB_FS_CLK_FREQ);
#endif
#if (defined(FSL_FEATURE_SOC_MPU_COUNT) && (FSL_FEATURE_SOC_MPU_COUNT > 0U))
    MPU_Enable(MPU, 0);
#endif /* FSL_FEATURE_SOC_MPU_COUNT */


    /**
     * phillip: DCD here?
     */
    /*itm_puts(0, "usb: run dcd\r\n");

    uhdcd_state_t state;
    uhdcd_error_t error = uhdcd_run(&state);

    itm_printf(0, "usb: dcd done: error=%u state=%u\r\n", error, state);*/


/*
 * If the SOC has USB KHCI dedicated RAM, the RAM memory needs to be clear after
 * the KHCI clock is enabled. When the demo uses USB EHCI IP, the USB KHCI dedicated
 * RAM can not be used and the memory can't be accessed.
 */
#if (defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U))
#if (defined(FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS) && (FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS > 0U))
    for (int i = 0; i < FSL_FEATURE_USB_KHCI_USB_RAM; i++)
    {
        ((uint8_t *)FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS)[i] = 0x00U;
    }
#endif /* FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS */
#endif /* FSL_FEATURE_USB_KHCI_USB_RAM */

    s_cdcVcom.speed = /*USB_SPEED_FULL*/USB_SPEED_HIGH;
    s_cdcVcom.attach = 0;
    s_cdcVcom.cdcAcmHandle = (class_handle_t)NULL;
    s_cdcVcom.deviceHandle = NULL;

    if (kStatus_USB_Success != USB_DeviceClassInit(CONTROLLER_ID, &s_cdcAcmConfigList, &s_cdcVcom.deviceHandle))
    {
        usb_echo("usb: init failed\r\n");
    }
    else
    {
        //usb_echo("usb: initialized\r\n");
        s_cdcVcom.cdcAcmHandle = s_cdcAcmConfigList.config->classHandle;
    }

    NVIC_SetPriority((IRQn_Type)irqNo, USB_DEVICE_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ((IRQn_Type)irqNo);

    USB_DeviceRun(s_cdcVcom.deviceHandle);
}

/*!
 * @brief USB task function.
 *
 * This function runs the task for USB device.
 *
 * @return None.
 */
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTask(void *handle)
{
    while (1U)
    {
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0)
        USB_DeviceEhciTaskFunction(handle);
#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0)
        USB_DeviceKhciTaskFunction(handle);
#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
        USB_DeviceLpcIp3511TaskFunction(handle);
#endif
    }
}
#endif


static void usb_vcom_rx_task(void *param)
{
    const usb_rx_cb_t const rx = (usb_rx_cb_t)param;
    const xQueueHandle rxq = usb_vcom.rxq;
    usb_io_t io;

    while (1) {
        if (xQueueReceive(rxq, &io, portMAX_DELAY)) {
            rx(io.len, io.buf);
            free(io.buf);
        }
    }
}


static void usb_vcom_task(void *param __unused)
{
    usb_status_t error = kStatus_USB_Error;

    USB_DeviceApplicationInit();
    assert(s_cdcVcom.deviceHandle);

    #if USB_DEVICE_CONFIG_USE_TASK

    usb_vcom.device_task = xTaskCreateStatic(USB_DeviceTask, "usb:dev", USB_VCOM_DEVICE_TASK_STACK_SIZE,
                                             s_cdcVcom.deviceHandle, TASK_PRIO_HIGH - 2,
                                             usb_vcom.device_task_stack, &usb_vcom.device_task_static);
    #endif

    usb_io_t io;

    while (1) {
        if (!xQueueReceive(usb_vcom.txq, &io, portMAX_DELAY)) continue;
        xSemaphoreTake(usb_vcom.txs, portMAX_DELAY);
        sending = true;
        //itm_printf(0, "<itm> usb: send io=0x%08X size=%lu receiving=%u\n", io, io.len, receiving);

        if ((error = USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, io.buf, io.len))) {
            itm_printf(0, "<itm> usb: tx error=%u\r\n", error);
            xSemaphoreGive(usb_vcom.txs);
            continue;
        }

        xSemaphoreTake(usb_vcom.txs, portMAX_DELAY);

        free(io.buf);
        io.buf = NULL;

        xSemaphoreGive(usb_vcom.txs);
    }
}

bool vcom_init(usb_rx_cb_t rx_cb)
{
    usb_vcom.txq = xQueueCreateStatic(USB_VCOM_TX_QUEUE_LEN, sizeof(usb_vcom.txq_buf[0]), (u8 *)usb_vcom.txq_buf, &usb_vcom.txq_static);
    usb_vcom.rxq = xQueueCreateStatic(USB_VCOM_RX_QUEUE_LEN, sizeof(usb_vcom.rxq_buf[0]), (u8 *)usb_vcom.rxq_buf, &usb_vcom.rxq_static);
    usb_vcom.txs = xSemaphoreCreateBinaryStatic(&usb_vcom.txs_static);
    xSemaphoreGive(usb_vcom.txs);
    usb_vcom.task = xTaskCreateStatic(usb_vcom_task, "usb:tx", USB_VCOM_TASK_STACK_SIZE, NULL, TASK_PRIO_HIGH - 1, usb_vcom.task_stack, &usb_vcom.task_static); assert(usb_vcom.task);
    usb_vcom.rx_task = xTaskCreateStatic(usb_vcom_rx_task, "usb:rx", USB_VCOM_RX_TASK_STACK_SIZE, rx_cb, TASK_PRIO_HIGH - 2, usb_vcom.rx_task_stack, &usb_vcom.rx_task_static); assert(usb_vcom.rx_task);
    return true;
}

static inline bool isInterrupt()
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ;
}

bool usb_attached(void)
{
    return s_cdcVcom.attach && s_cdcVcom.startTransactions;
}

extern size_t frame_encode(u8 code, size_t size, u8 data[], u8 **frame);

void usb_write(u8 *buf, size_t len)
{
    if (!usb_attached() || !buf) return;

    u8 *frame = NULL;
    size_t size = frame_encode(0x00, len, buf, &frame);

    if (frame) {
        usb_write_direct(frame, size);
    }
}

void usb_write_direct(u8 *buf, size_t len)
{
    if (!usb_attached() || !buf) {
        if (buf) free(buf);
        return;
    }

    //usb_io_t *io = malloc(sizeof(usb_io_t) + len); assert(io);
    //io->len = len;
    //memcpy(io->buf, buf, len);

    usb_io_t io = { len, buf };

    //itm_printf(0, "<itm> usb: queu io=0x%08X size=%lu\n", io, io->len);

    if (!isInterrupt()) {
        if (!xQueueSend(usb_vcom.txq, &io, pdMS_TO_TICKS(1000))) {
            itm_puts(0, "usb: queue failed\r\n");
            free(buf);
        }
    } else {
        if (!xQueueSendFromISR(usb_vcom.txq, &io, NULL)) {
            itm_puts(0, "usb: queue failed\r\n");
            free(buf);
        }
    }

    /*xSemaphoreTake(usb_tx_s, portMAX_DELAY);
    if (flush_needed) flush_needed = false;
    itm_printf(0, "<itm> usb/tx: size=%lu\r\n", len);
    USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, buf, len);
    xSemaphoreTake(usb_tx_s, portMAX_DELAY);
    xSemaphoreGive(usb_tx_s);*/
}

