#include <fabi.h>
#include "ftm.h"

#include <kio/itm.h>

#include <fsl_ftm.h>
#include <fsl_clock.h>
#include <FreeRTOS.h>

#include <fsl_dma_manager.h>
#include <task.h>
#include <MK66F18.h>

#define FABI_BUS_COUNT      2

#define GPIO                GPIOE
#define GPIO_BIT            ((u8) 0u)
#define GPIO_REG_BYTE(x)    (x) // ((u8 *)&(x))[1]


#define FABI_NOTIFY             (1u<<23u)
#define FABI_TASK_STACK_SIZE    TASK_STACK_SIZE_DEFAULT


#define FABI_CHAN_MASK      ((u8)((u8)(1u << (u8)FABI_CHAN_COUNT) - 1u))

// Only works for data in SRAM_U, above 0x2000_0000. For this project, that's the stack or section m_band.
// https://community.arm.com/processors/f/discussions/6679/bit-banding-in-sram-region-cortex-m4
#define BITBAND_REGION_BIT(src_byte_addr, bit)  (((((uint32_t)(src_byte_addr) & 0x000fffffu) << 5u) | ((uint32_t)(src_byte_addr) & 0xfff00000u) | 0x02000000u) + (((uint32_t)(bit)) << 2u))


typedef u8 fabi_out_t;


static void handle_edma(struct _edma_handle *handle, void *userData, bool transferDone, uint32_t tcds);
static void fabi_dma_run(size_t size);
fabi_rgb_t *fabi_buffer(void);

static edma_tcd_t tcd[3] __aligned(32);
static edma_handle_t edma_handle[3];
static edma_transfer_config_t tr[3] = {{0}};

static u8 band_in[FABI_LED_MAX * sizeof(fabi_rgb_t) * 8] __section(".band");
static fabi_out_t band_out[FABI_LED_MAX * sizeof(fabi_rgb_t) * 8] __section(".band");

static void fabi_task(void *param);
TaskHandle_t fabi_task_handle;
StaticTask_t fabi_task_static;
StackType_t fabi_task_stack[FABI_TASK_STACK_SIZE];


void fabi_init(void)
{
    status_t status;

    ftm_init();

    // Initialize EDMA
    edma_config_t edma_config;

    EDMA_GetDefaultConfig(&edma_config);
    edma_config.enableRoundRobinArbitration = true;

    EDMA_Init(DMA0, &edma_config);


    // Initialize DMA Manager
    dmamanager_handle_t *const dmam = DMAMGR_Handle();

    // Set up DMA Channels
    dma_request_source_t dreq = kDmaRequestMux0FTM0Channel0;


    status = DMAMGR_RequestChannel(dmam, dreq, DMAMGR_DYNAMIC_ALLOCATE, &edma_handle[0]);

    if (status) {
        printf("ftm dma setup/0 fail: status=%lu\n", status);
        return;
    }

    dreq = kDmaRequestMux0FTM0Channel1;
    status = DMAMGR_RequestChannel(dmam, dreq, DMAMGR_DYNAMIC_ALLOCATE, &edma_handle[1]);

    if (status) {
        printf("ftm dma setup/1 fail: status=%lu\n", status);
        return;
    }

    dreq = kDmaRequestMux0FTM0Channel2;
    status = DMAMGR_RequestChannel(dmam, dreq, DMAMGR_DYNAMIC_ALLOCATE, &edma_handle[2]);

    if (status) {
        printf("ftm dma setup/2 fail: status=%lu\n", status);
        return;
    }

    // Necessary if radio interrupts should prioritize over this one.
    NVIC_SetPriority(((IRQn_Type [][FSL_FEATURE_EDMA_MODULE_CHANNEL])DMA_CHN_IRQS)[0][edma_handle[2].channel], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);


    static fabi_out_t flag = FABI_CHAN_MASK << GPIO_BIT;

    // Could also use EDMA_PrepareTransfer() for these:

    tr[0].srcAddr = (u32) &flag;
    tr[0].destAddr = (u32) &GPIO_REG_BYTE(GPIO->PSOR); // Port Set Out Register
    tr[0].srcOffset = 0;
    tr[0].srcTransferSize = sizeof(fabi_out_t) > 1 ? kEDMA_TransferSize4Bytes : kEDMA_TransferSize1Bytes;
    tr[0].destTransferSize = sizeof(fabi_out_t) > 1 ? kEDMA_TransferSize4Bytes : kEDMA_TransferSize1Bytes;
    tr[0].minorLoopBytes = sizeof(fabi_out_t);
    tr[0].majorLoopCounts = 0; // bit count

    tr[1] = tr[0];
    tr[1].srcAddr = (u32) band_out;
    tr[1].destAddr = (u32) &GPIO_REG_BYTE(GPIO->PDOR); // Port Data Out Register
    tr[1].srcOffset = sizeof(fabi_out_t);

    tr[2] = tr[0];
    tr[2].srcAddr = (u32) &flag;
    tr[2].destAddr = (u32) &GPIO_REG_BYTE(GPIO->PCOR); // Port Clear Out Register


    EDMA_InstallTCDMemory(&edma_handle[0], &tcd[0], 1);
    EDMA_InstallTCDMemory(&edma_handle[1], &tcd[1], 1);
    EDMA_InstallTCDMemory(&edma_handle[2], &tcd[2], 1);

    EDMA_DisableChannelInterrupts(DMA0, edma_handle[0].channel, UINT32_MAX);
    EDMA_DisableChannelInterrupts(DMA0, edma_handle[1].channel, UINT32_MAX);
    EDMA_EnableChannelInterrupts(DMA0, edma_handle[2].channel, kEDMA_MajorInterruptEnable);

    EDMA_SetCallback(&edma_handle[2], handle_edma, &edma_handle[2]);

    // Needed?
    //EDMA_TcdSetTransferConfig(&tcd[0], &tr[0], &tcd[1]);
    //EDMA_TcdSetTransferConfig(&tcd[1], &tr[1], &tcd[2]);
    //EDMA_TcdSetTransferConfig(&tcd[2], &tr[2], NULL);

    // Needed?
    //EDMA_SetTransferConfig(DMA0, edma_handle[0].channel, &tr[0], &tcd[1]);
    //EDMA_SetTransferConfig(DMA0, edma_handle[1].channel, &tr[1], &tcd[2]);
    //EDMA_SetTransferConfig(DMA0, edma_handle[2].channel, &tr[2], NULL);


    /*#define INIT_COUNT 144
    fabi_rgb_t *init1 = pvPortMalloc(sizeof(fabi_rgb_t) * INIT_COUNT);
    fabi_rgb_t *init2 = pvPortMalloc(sizeof(fabi_rgb_t) * INIT_COUNT);

    memset(init1, 0, sizeof(fabi_rgb_t) * INIT_COUNT);
    memset(init2, 0, sizeof(fabi_rgb_t) * INIT_COUNT);

    memset(band_out, 0, sizeof(band_out[0]) * FABI_LED_MAX * sizeof(fabi_rgb_t) * 8);

    u16 counter, i;

    while (1) {

        for (counter = 0; counter <= UINT8_MAX; ++counter) {

            for (i = 0; i < INIT_COUNT; ++i) {
                init1[i] = (fabi_rgb_t) {(8 - i % 8u)*2, (i % 8u)*3, (counter > 127 ? 255 - counter : counter)/4 };
                init2[i] = (fabi_rgb_t) {(8 - i % 8u)*2, (counter > 127 ? 255 - counter : counter)/4, (i % 8u)*3 };
            }

            fabi_write(1, init1, INIT_COUNT);
            fabi_write(2, init2, INIT_COUNT);

            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }*/

    fabi_task_handle = xTaskCreateStatic(
            (TaskFunction_t) fabi_task, "fabi", FABI_TASK_STACK_SIZE, NULL, TASK_PRIO_DEFAULT, fabi_task_stack, &fabi_task_static
    );
}


static void fabi_task(void *param)
{
    #define INIT_COUNT 60

    fabi_rgb_t *init1 = pvPortMalloc(sizeof(fabi_rgb_t) * INIT_COUNT);
    fabi_rgb_t *init2 = pvPortMalloc(sizeof(fabi_rgb_t) * INIT_COUNT);

    memset(init1, 0, sizeof(fabi_rgb_t) * INIT_COUNT);
    memset(init2, 0, sizeof(fabi_rgb_t) * INIT_COUNT);

    u16 i;

    /*while (1) {

        for (counter = 0; counter <= UINT8_MAX; ++counter) {

            for (i = 0; i < INIT_COUNT; ++i) {
                init1[i] = (fabi_rgb_t) {(8 - i % 8u)*2, (i % 8u)*3, (counter > 127 ? 255 - counter : counter)/4 };
                init2[i] = (fabi_rgb_t) {(8 - i % 8u)*2, (counter > 127 ? 255 - counter : counter)/4, (i % 8u)*3 };
            }

            fabi_write(1, init1, INIT_COUNT);
            fabi_write(2, init2, INIT_COUNT);

            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }*/

    while (1) {

        for (i = 0; i < INIT_COUNT; ++i) {
            init1[i] = (fabi_rgb_t) { 0x00, 0xff, 0x00 };

            fabi_write(0b11, init1, INIT_COUNT);
            //vTaskDelay(pdMS_TO_TICKS(2));

            init1[i] = (fabi_rgb_t) { 0x00, i, 0x00 };
        }

    }
}

fabi_rgb_t *fabi_buffer(void)
{
    return (fabi_rgb_t *) band_in;
}


void fabi_write(u8 mask, fabi_rgb_t *data, size_t size)
{
    const u32 bits = size * sizeof(fabi_rgb_t) * 8;
    u32 mod;
    u8 bit;
    u8 out_bits, biti;

    mask &= FABI_CHAN_MASK;

    memcpy(band_in, data, size * sizeof(fabi_rgb_t));

    // Explode the bits into array with one bit output element.
    for (u32 i = 0; i < bits; ++i) {
        mod = i % 8;
        bit = *(u8 *)(i - mod + (u32 *)BITBAND_REGION_BIT(band_in, 7 - mod));

        out_bits = mask;

        while (out_bits) {
            biti = __builtin_ctz(out_bits);
            *(fabi_out_t *)BITBAND_REGION_BIT(&band_out[i], GPIO_BIT + biti) = bit;
            out_bits = out_bits & (u8)(out_bits - 1u);
        }
    }

    fabi_dma_run(size * sizeof(fabi_rgb_t));
}

static void fabi_dma_run(size_t size)
{
    const size_t bits = size * 8;

    tr[0].majorLoopCounts = tr[1].majorLoopCounts = tr[2].majorLoopCounts = bits;
    edma_handle[2].userData = xTaskGetCurrentTaskHandle();

    /*EDMA_TcdReset(&tcd[0]);
    EDMA_TcdReset(&tcd[1]);
    EDMA_TcdReset(&tcd[2]);

    EDMA_ResetChannel(DMA0, edma_handle[0].channel);
    EDMA_ResetChannel(DMA0, edma_handle[1].channel);
    EDMA_ResetChannel(DMA0, edma_handle[2].channel);*/

    taskENTER_CRITICAL();

    DMAMUX_DisableChannel(DMAMUX0, edma_handle[0].channel);
    DMAMUX_DisableChannel(DMAMUX0, edma_handle[1].channel);
    DMAMUX_DisableChannel(DMAMUX0, edma_handle[2].channel);

    EDMA_SubmitTransfer(&edma_handle[0], &tr[0]);
    EDMA_SubmitTransfer(&edma_handle[1], &tr[1]);
    EDMA_SubmitTransfer(&edma_handle[2], &tr[2]);

    EDMA_StartTransfer(&edma_handle[0]);
    EDMA_StartTransfer(&edma_handle[1]);
    EDMA_StartTransfer(&edma_handle[2]);

    DMAMUX_EnableChannel(DMAMUX0, edma_handle[0].channel);
    DMAMUX_EnableChannel(DMAMUX0, edma_handle[1].channel);
    DMAMUX_EnableChannel(DMAMUX0, edma_handle[2].channel);

    FTM->CNT = 0;
    //FTM_ClearStatusFlags(FTM, UINT32_MAX);
    FTM_StartTimer(FTM, kFTM_SystemClock);

    taskEXIT_CRITICAL();

    u32 notify = 0;

    do {
        if (!xTaskNotifyWait(0, FABI_NOTIFY, &notify, portMAX_DELAY))
            notify = 0;
    } while (!(notify & FABI_NOTIFY));

    FTM_StopTimer(FTM);
}


static void handle_edma(struct _edma_handle *handle, void *userData, bool transferDone, uint32_t tcds)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (transferDone && userData)
        xTaskNotifyFromISR(userData, FABI_NOTIFY, eSetBits, &xHigherPriorityTaskWoken);

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
