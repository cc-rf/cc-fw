#include <string.h>
#include <stdio.h>

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include <cc/spi.h>
#include <cc/io.h>
#include <cc/phy.h>
#include <cc/freq.h>
#include <cc/cc1200.h>
#include <cc/isr.h>
#include <cc/tmr.h>
#include <cc/mac.h>
#include <cc/chan.h>
#include <core_cm4.h>
#include <cc/sys/kinetis/pit.h>



extern void vcom_init(void);

static void main_task(void *param);
static void input_task(void *param);
static void output_task(void *param);

static void mac_rx(cc_dev_t dev, u8 *buf, u8 len);

mac_cfg_t cc_mac_cfg = {
        .rx = mac_rx
};

#define MODE_TX     1
#define MODE_RX     2
#define MODE        MODE_RX

#include <itm.h>
#include <util/uart.h>
#include <malloc.h>

pit_t xsec_timer_lo, xsec_timer, wdog_timer;

int main(void)
{
    BOARD_InitPins();
    LED_ABCD_ALL_ON();
    BOARD_BootClockHSRUN();
    BOARD_InitDebugConsole();
    itm_init();
    printf("<boot>\r\n");


    printf("\nclocks:\n  core\t\t\t= %lu\n  bus\t\t\t= %lu\n  flexbus\t\t= %lu\n  flash\t\t\t= %lu\n  pllfllsel\t\t= %lu\n  er32k\t\t\t= %lu\n  osc0er\t\t= %lu\n  osc0erundiv\t\t= %lu\n  mcgfixedfreq\t\t= %lu\n  mcginternalref\t= %lu\n  mcgfll\t\t= %lu\n  mcgpll0\t\t= %lu\n  mcgirc48m\t\t= %lu\n  lpo\t\t\t= %lu\n\n",
       CLOCK_GetFreq(kCLOCK_CoreSysClk),
       CLOCK_GetFreq(kCLOCK_BusClk),
       CLOCK_GetFreq(kCLOCK_FlexBusClk),
       CLOCK_GetFreq(kCLOCK_FlashClk),
       CLOCK_GetFreq(kCLOCK_PllFllSelClk),
       CLOCK_GetFreq(kCLOCK_Er32kClk),
       CLOCK_GetFreq(kCLOCK_Osc0ErClk),
       CLOCK_GetFreq(kCLOCK_Osc0ErClkUndiv),
       CLOCK_GetFreq(kCLOCK_McgFixedFreqClk),
       CLOCK_GetFreq(kCLOCK_McgInternalRefClk),
       CLOCK_GetFreq(kCLOCK_McgFllClk),
       CLOCK_GetFreq(kCLOCK_McgPll0Clk),
       CLOCK_GetFreq(kCLOCK_McgIrc48MClk),
       CLOCK_GetFreq(kCLOCK_LpoClk)
    );



    xsec_timer_lo = pit_alloc(&(pit_cfg_t){
            .period = pit_nsec_tick(1000)
    });

    xsec_timer = pit_chain(xsec_timer_lo, &(pit_cfg_t){
            .period = UINT32_MAX
    });

    pit_start(xsec_timer);
    pit_start(xsec_timer_lo);

    if (xTaskCreate(main_task, "main", TASK_STACK_SIZE_DEFAULT, NULL, TASK_PRIO_HIGHEST, NULL) != pdPASS) goto done;

    vTaskStartScheduler();

    done:

    LED_ABCD_ALL_OFF();

    while (1) {
        LED_ABCD_ALL_TOGGLE();
        for (int i = 0; i < 1000000; ++i);
    }
}

static void print_hex(u8 *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        printf("0x%02X ", buf[i]);
    }

    printf("\r\n");
}

#define DEVICE 1

struct packet {
    u8 id;
    u32 seq;
    char filler[20];
};

static uart_t uart;

typedef struct __packed uart_frame {
    size_t len;
    u8 *data;
    
} uart_frame_t;

typedef struct __packed ucmd_hdr {
    u8 cmd;

} ucmd_hdr_t;

typedef struct __packed ucmd_tx {
    ucmd_hdr_t hdr;  /* cmd == 0x11 */
    u8 flags;
    u8 channel;
    u8 count;
    u8 delay;
    u8 data[];

} ucmd_tx_t;

typedef struct __packed ucmd_rx {
    ucmd_hdr_t hdr;  /* cmd == 0x12 */
    u8 radio;
    u8 channel;

} ucmd_rx_t;

typedef struct __packed ucmd_mod {
    ucmd_hdr_t hdr;     /* cmd == 0x13 */
    u8 radio;           /* == 0 or 1 */
    u8 mode;            /* == 0 or 1 */

} ucmd_mod_t;

static xQueueHandle output_queue;

static void main_task(void *param)
{
    (void)param;
    LED_ABCD_ALL_OFF();

    printf("<main task>\r\n");

    uart = uart_init(0, 230400);

    xQueueHandle input_queue = xQueueCreate(32, sizeof(uart_frame_t));

    if (!input_queue) goto done;

    //output_queue = xQueueCreate(4, sizeof(uart_frame_t));
    //if (!output_queue) goto done;

    if (xTaskCreate(input_task, "input", TASK_STACK_SIZE_DEFAULT, (void *)input_queue, TASK_PRIO_HIGH, NULL) != pdPASS) goto done;
    //if (xTaskCreate(output_task, "output", TASK_STACK_SIZE_DEFAULT, (void *)output_queue, TASK_PRIO_HIGHEST, NULL) != pdPASS) goto done;

    if (!mac_init(&cc_mac_cfg)) {
        goto done;
    }

    cc_set_mod_cfg_1(0);
    cc_set_mod_cfg_1(1);


#if MODE == MODE_TX

    u32 xsec0, xsec1;

    struct packet tx_data = {
            .id = 2,
            .seq = 0,
            .filler = "ABCDEFGHIJKLMNOPQRS"
    };

    //mac_rx_enable();
    mac_tx_begin(13);

    printf("tx: f=%lu\r\n", cc_get_freq(DEVICE));

    while (1) {
        //xsec0 = pit_get_elapsed(xsec_timer);
        if (!(++tx_data.seq % 10)) LED_B_TOGGLE();
        mac_tx(false, (u8 *)&tx_data, sizeof(tx_data));
        //printf("tx: chan=%u,%u duration=%luus count=%lu\r\n", chan, chan_cur_id, xsec, tx_data.seq);
        //while ((xsec1 = pit_get_elapsed(xsec_timer) - xsec0) < 8333) mac_task();
        /*do {
            xsec1 = pit_get_elapsed(xsec_timer) - xsec0;
        } while (xsec1 < 8000);*/
        //vTaskDelay((8 - (xsec1 > 8000 ? 8000 : xsec1)/1000) / portTICK_PERIOD_MS);
    }

#elif MODE == MODE_RX

    //printf("rx: f=%lusr=%lut=%luus\r\n", cc_get_freq(DEVICE),  cc_get_symbol_rate(DEVICE), (u32)(cc_get_rx_timeout(DEVICE)/1000));

    mac_rx_enable();

    uart_frame_t frame;

    while (1) {
        if (!(frame.len = uart_read_frame(uart, &frame.data))) {
            printf("uart rx: empty\n");
            continue;
        }

        if (!xQueueSend(input_queue, &frame, 0)) {
            printf("uart rx: queue full\n");
        }
    }

#elif MODE == MODE_TX + MODE_RX

    mac_rx_enable();

    char buf[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ\r\n"; // 29 bytes

    while (1) {
        mac_tx((u8 *)buf, strlen(buf)+1);
        LED_A_TOGGLE();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

#endif

    done:
    //vTaskDelete(NULL);

    LED_ABCD_ALL_OFF();
    LED_C_ON();

    while (1) {
        LED_C_TOGGLE();
        LED_D_TOGGLE();
        for (int i = 0; i < 1000000; ++i);
    }
}

static void input_task(void *param)
{
    xQueueHandle const input_queue = (xQueueHandle)param;
    uart_frame_t frame;
    //size_t tx_count = 0;

    while (1) {
        if (xQueueReceive(input_queue, &frame, portMAX_DELAY) != pdTRUE) {
            printf("input_task: queue receive failed\n");
            continue;
        }

        if (frame.len >= sizeof(ucmd_hdr_t)) {
            ucmd_hdr_t *hdr = (ucmd_hdr_t *)frame.data;

            if (hdr->cmd == 0x11 && frame.len >= sizeof(ucmd_tx_t)) {
                ucmd_tx_t *ucmd = (ucmd_tx_t *) frame.data;
                frame.len -= sizeof(ucmd_tx_t);

                const bool cca = false;//(ucmd->flags & 1) == 0;

                printf("ucmd(tx): flags=0x%02X channel=%u len=%u count=%u delay=%u\n",
                       ucmd->flags, ucmd->channel, frame.len, ucmd->count, ucmd->delay
                );

                mac_tx_begin((chan_t) ucmd->channel);

                while (ucmd->count--) {
                    mac_tx(cca, ucmd->data, frame.len);
                    vTaskDelay((TickType_t) ucmd->delay / portTICK_PERIOD_MS);
                }

                mac_tx_end();

                printf("ucmd(tx): done\n");
            } else if (hdr->cmd == 0x12 && frame.len >= sizeof(ucmd_rx_t)) {
                ucmd_rx_t *ucmd = (ucmd_rx_t *) frame.data;
                if (ucmd->radio >= CC_DEV_MIN && ucmd->radio <= CC_DEV_MAX && (ucmd->channel < 50 || ucmd->channel == 0xFF))
                    mac_set_rx_channel((cc_dev_t)ucmd->radio, ucmd->channel);
            } else if (hdr->cmd == 0x13 && frame.len >= sizeof(ucmd_mod_t)) {
                ucmd_mod_t *ucmd = (ucmd_mod_t *) frame.data;
                if (ucmd->radio >= CC_DEV_MIN && ucmd->radio <= CC_DEV_MAX && (ucmd->mode == 0 || ucmd->mode == 1)) {
                    if (ucmd->mode == 0)
                        mac_set_mod_cfg_0((cc_dev_t) ucmd->radio);
                    else
                        mac_set_mod_cfg_1((cc_dev_t) ucmd->radio);
                }
            } else {
                frame.len = frame.len + sizeof(ucmd_tx_t) - sizeof(ucmd_hdr_t);
                printf("ucmd: cmd=0x%02X len=%u\n",
                       hdr->cmd, frame.len
                );
            }

        } else {
            printf("uart rx frame: len=%u\n", frame.len);
        }

        free(frame.data);
    }
}

/*static void output_task(void *param)
{
    (void)param;
    uart_frame_t frame;

    while (1) {
        if (xQueueReceive(output_queue, &frame, portMAX_DELAY) != pdTRUE) continue;
        uart_write(uart, frame.data, frame.len);
        free(frame.data);
    }
}*/

u32 pkt_count[CC_NUM_DEVICES] = {0}, pkt_count_0[CC_NUM_DEVICES] = {0};
pit_tick_t pkt_tmr_0[CC_NUM_DEVICES] = {0}, pkt_tmr_1[CC_NUM_DEVICES] = {0};

static void mac_rx(cc_dev_t dev, u8 *buf, u8 len)
{
    if (!pkt_count[dev]) {
        pkt_tmr_0[dev] = pit_get_elapsed(xsec_timer);
    }

    ++pkt_count[dev];

    if (!(pkt_count[dev] % 120)) {
        pkt_tmr_1[dev] = pit_get_elapsed(xsec_timer);
        float div = (float)(pkt_tmr_1[dev] - pkt_tmr_0[dev]) / 1000000;
        u32 pkt_rate = (u32)((pkt_count[dev] - pkt_count_0[dev]) / div);
        printf("[%u] rate = %*lu p/s\t\tn=%lu t=%luus\tN=%lu\n",
               dev, 3, pkt_rate, (pkt_count[dev] - pkt_count_0[dev]), (pkt_tmr_1[dev] - pkt_tmr_0[dev]),
               pkt_count[dev]
        );
        pkt_tmr_0[dev] = pkt_tmr_1[dev];
        pkt_count_0[dev] = pkt_count[dev];
    }

    if (dev == CC_DEV_MIN) {
        if (!(pkt_count[dev] % 12)) LED_A_TOGGLE();
    } else {
        if (!(pkt_count[dev] % 12)) LED_B_TOGGLE();
    }

    /*uart_frame_t const frame = {
            .data = buf,
            .len = len
    };

    xQueueSend(output_queue, &frame, portMAX_DELAY);*/
    uart_write(uart, buf, len);
}


int _write(int handle, char *buffer, int size)
{
    if (buffer == 0)
        return -1;

    if ((handle != 1) && (handle != 2))
    {
        return -1;
    }

    itm_write(0, (const u8 *)buffer, (size_t)size);

    return size;
}



/* http://www.freertos.org/Debugging-Hard-Faults-On-Cortex-M-Microcontrollers.html */

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
//static void HardFault_Handler( void ) __attribute__( ( naked ) );

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
__attribute__((naked, interrupt)) void HardFault_Handler(void)
{
    __asm volatile
    (
    " tst lr, #4                                                \n"
            " ite eq                                                    \n"
            " mrseq r0, msp                                             \n"
            " mrsne r0, psp                                             \n"
            //" ldr r1, [r0, #24]                                         \n"
            //" ldr r2, handler2_address_const                            \n"
            //" bx r2                                                     \n"
            //" handler2_address_const: .word hard_fault    \n"
            //" mov r1, lr                                                \n"
            " ldr r2, =hard_fault                                       \n"
            " bx r2                                                     \n"

    );
}

/*__attribute__((naked))*/ void UsageFault_Handler(void)
{
    __asm volatile
    (
    " tst lr, #4                                                \n"
            " ite eq                                                    \n"
            " mrseq r0, msp                                             \n"
            " mrsne r0, psp                                             \n"
            //" ldr r1, [r0, #24]                                         \n"
            //" ldr r2, handler3_address_const                            \n"
            //" bx r2                                                     \n"
            //" handler3_address_const: .word usage_fault    \n"
            //" mov r1, lr                                                \n"
            " ldr r2, =usage_fault                                      \n"
            " bx r2                                                     \n"
    );
}

/* TODO: Maybe don't use volatile since copies will not need to be such? */
typedef volatile struct {
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t r2;
    volatile uint32_t r3;
    volatile uint32_t r12;
    volatile uint32_t lr;
    volatile uint32_t pc;

    union {
        volatile uint32_t PSR;
        struct {
            uint32_t IPSR : 8;
            uint32_t EPSR : 19;
            uint32_t APSR : 5;
        };
    } psr;

} fault_reg_t;

/* From Segger Application Note on Hard Faults */
static struct {

    struct {
        volatile unsigned int r0;            // Register R0
        volatile unsigned int r1;            // Register R1
        volatile unsigned int r2;            // Register R2
        volatile unsigned int r3;            // Register R3
        volatile unsigned int r12;           // Register R12
        volatile unsigned int lr;            // Link register
        volatile unsigned int pc;            // Program counter
        union {
            volatile unsigned int byte;
            struct {
                unsigned int IPSR : 8;           // Interrupt Program Status register (IPSR)
                unsigned int EPSR : 19;          // Execution Program Status register (EPSR)
                unsigned int APSR : 5;           // Application Program Status register (APSR)
            } bits;
        } psr;                               // Program status register.
    } SavedRegs;

union {
    volatile unsigned int byte;
    struct {
        unsigned int MEMFAULTACT    : 1;   // Read as 1 if memory management fault is active
        unsigned int BUSFAULTACT    : 1;   // Read as 1 if bus fault exception is active
        unsigned int UnusedBits1    : 1;
        unsigned int USGFAULTACT    : 1;   // Read as 1 if usage fault exception is active
        unsigned int UnusedBits2    : 3;
        unsigned int SVCALLACT      : 1;   // Read as 1 if SVC exception is active
        unsigned int MONITORACT     : 1;   // Read as 1 if debug monitor exception is active
        unsigned int UnusedBits3    : 1;
        unsigned int PENDSVACT      : 1;   // Read as 1 if PendSV exception is active
        unsigned int SYSTICKACT     : 1;   // Read as 1 if SYSTICK exception is active
        unsigned int USGFAULTPENDED : 1;   // Usage fault pended; usage fault started but was replaced by a higher-priority exception
        unsigned int MEMFAULTPENDED : 1;   // Memory management fault pended; memory management fault started but was replaced by a higher-priority exception
        unsigned int BUSFAULTPENDED : 1;   // Bus fault pended; bus fault handler was started but was replaced by a higher-priority exception
        unsigned int SVCALLPENDED   : 1;   // SVC pended; SVC was started but was replaced by a higher-priority exception
        unsigned int MEMFAULTENA    : 1;   // Memory management fault handler enable
        unsigned int BUSFAULTENA    : 1;   // Bus fault handler enable
        unsigned int USGFAULTENA    : 1;   // Usage fault handler enable
    } bits;
} syshndctrl;                          // System Handler Control and State Register (0xE000ED24)

union {
    volatile unsigned char byte;
    struct {
        unsigned char IACCVIOL    : 1;     // Instruction access violation
        unsigned char DACCVIOL    : 1;     // Data access violation
        unsigned char UnusedBits  : 1;
        unsigned char MUNSTKERR   : 1;     // Unstacking error
        unsigned char MSTKERR     : 1;     // Stacking error
        unsigned char UnusedBits2 : 2;
        unsigned char MMARVALID   : 1;     // Indicates the MMAR is valid
    } bits;
} mfsr;                                // Memory Management Fault Status Register (0xE000ED28)

union {
    volatile unsigned int byte;
    struct {
        unsigned int IBUSERR    : 1;       // Instruction access violation
        unsigned int PRECISERR  : 1;       // Precise data access violation
        unsigned int IMPREISERR : 1;       // Imprecise data access violation
        unsigned int UNSTKERR   : 1;       // Unstacking error
        unsigned int STKERR     : 1;       // Stacking error
        unsigned int UnusedBits : 2;
        unsigned int BFARVALID  : 1;       // Indicates BFAR is valid
    } bits;
} bfsr;                                // Bus Fault Status Register (0xE000ED29)
volatile unsigned int bfar;            // Bus Fault Manage Address Register (0xE000ED38)

union {
    volatile unsigned short byte;
    struct {
        unsigned short UNDEFINSTR : 1;     // Attempts to execute an undefined instruction
        unsigned short INVSTATE   : 1;     // Attempts to switch to an invalid state (e.g., ARM)
        unsigned short INVPC      : 1;     // Attempts to do an exception with a bad value in the EXC_RETURN number
        unsigned short NOCP       : 1;     // Attempts to execute a coprocessor instruction
        unsigned short UnusedBits : 4;
        unsigned short UNALIGNED  : 1;     // Indicates that an unaligned access fault has taken place
        unsigned short DIVBYZERO  : 1;     // Indicates a divide by zero has taken place (can be set only if DIV_0_TRP is set)
    } bits;
} ufsr;                                // Usage Fault Status Register (0xE000ED2A)

union {
    volatile unsigned int byte;
    struct {
        unsigned int UnusedBits  : 1;
        unsigned int VECTBL      : 1;      // Indicates hard fault is caused by failed vector fetch
        unsigned int UnusedBits2 : 27;
        unsigned int FORCED      : 1;      // Indicates hard fault is taken because of bus fault/memory management fault/usage fault
        unsigned int DEBUGEVT    : 1;      // Indicates hard fault is triggered by debug event
    } bits;
} hfsr;                                // Hard Fault Status Register (0xE000ED2C)

union {
    volatile unsigned int byte;
    struct {
        unsigned int HALTED   : 1;         // Halt requested in NVIC
        unsigned int BKPT     : 1;         // BKPT instruction executed
        unsigned int DWTTRAP  : 1;         // DWT match occurred
        unsigned int VCATCH   : 1;         // Vector fetch occurred
        unsigned int EXTERNAL : 1;         // EDBGRQ signal asserted
    } bits;
} dfsr;                                // Debug Fault Status Register (0xE000ED30)

volatile unsigned int afsr;            // Auxiliary Fault Status Register (0xE000ED3C), Vendor controlled (optional)
} HardFaultRegs;




static unsigned long long hard_fault_count;
static unsigned long long hard_fault_bkpt_count;
static bool hard_fault_continue;
static unsigned long long usage_fault_count;

static fault_reg_t hard_fault_last_reg;

void hard_fault( fault_reg_t *fr/*, uint32_t lr __attribute__((unused))*/ )
{
    static int hard_fault_count_init;

    printf("!!!!!!!!!! HARD FAULT !!!!!!!!!!!!!!\r\n");
    NVIC_SystemReset();

    if (hard_fault_count_init != 0xABCD) {
        hard_fault_count_init = 0xABCD;
        hard_fault_count = 1;
        hard_fault_bkpt_count = 0;
        hard_fault_continue = false;
    }
    else
        ++hard_fault_count;

    hard_fault_last_reg = *fr;

    if (SCB->HFSR & SCB_HFSR_DEBUGEVT_Msk) {
        ++hard_fault_bkpt_count;

        if (*(uint16_t *)fr->pc != 0xBEAB)
            while (!hard_fault_continue) asm("nop");

        // This is what was in the segger code...
        //SCB->HFSR |=  SCB_HFSR_DEBUGEVT_Msk;     // Reset Hard Fault status
        SCB->HFSR &= ~SCB_HFSR_DEBUGEVT_Msk;

        fr->pc += 2;
        return;                       // Return to interrupted application
    }

    /*if (*((uint16_t *)fr->pc) == 0xBEAB)
    {
        while (1) {
            __NOP();
        }
    }*/

    while (!hard_fault_continue) asm("nop");

    /*if (((SCB->DFSR & SCB_DFSR_BKPT_Msk) != 0)
        && ((SCB->HFSR & SCB_HFSR_DEBUGEVT_Msk) != 0))
    {
        if (*((uint16_t *)fr->pc) == 0xBEAB)
        {
            // Clear the exception cause in exception status.
            SCB->HFSR = SCB_HFSR_DEBUGEVT_Msk;

            // Continue after the BKPT
            fr->r0 = (uint32_t)-1;
            fr->pc += 2;

            ++hard_fault_semi_count;

            __BKPT(0);

            while (1) {
                __NOP();
            }

            return;
        }

        while (1) {
            __NOP();
        }
    }

    while (1) {
        __NOP();
    }*/
}

void usage_fault( fault_reg_t *fr/*, uint32_t lr __attribute__((unused))*/ )
{
    static int usage_fault_count_init;

    if (usage_fault_count_init != 0xBEEF) {
        usage_fault_count_init = 0xBEEF;
        usage_fault_count = 1;
    }
    else
        ++usage_fault_count;
    
    if (SCB->CFSR & (1UL<<16)) {
        while (1) {
            __NOP();
        }
    }

    while (1) {
        __NOP();
    }

    /*if (SCB->CFSR & (1UL<<16)) {
        fr->r0 = (uint32_t)-1;
        fr->pc += 2;

        ++usage_fault_semi_count;

        //__BKPT(0);

        //while (1) {
        //    __NOP();
        //}
    }*/

    /*while (1) {
        __NOP();
    }*/
}

#include <stdio.h>
#include <errno.h>

/*!
 * @brief Function to override ARMGCC default function _sbrk
 *
 * _sbrk is called by malloc. ARMGCC default _sbrk compares "SP" register and
 * heap end, if heap end is larger than "SP", then _sbrk returns error and
 * memory allocation failed. This function changes to compare __HeapLimit with
 * heap end.
 */
caddr_t _sbrk(int incr)
{
    extern char end __asm("end");
    extern char heap_limit __asm("__HeapLimit");
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == NULL)
        heap_end = &end;

    prev_heap_end = heap_end;

    if (heap_end + incr > &heap_limit)
    {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;

    return (caddr_t)prev_heap_end;
}
