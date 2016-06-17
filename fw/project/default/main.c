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
#include <cc/cfg.h>
#include <cc/amp.h>

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
        for (int i = 0; i < 10000000; ++i);
    }
}

static void print_hex(u8 *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        printf("0x%02X ", buf[i]);
    }

    printf("\r\n");
}

static bool radio_init(cc_dev_t dev);
static void isr_mcu_wake(cc_dev_t dev);
static void isr_rssi_valid(cc_dev_t dev);
static void isr_carrier_sense(cc_dev_t dev);


static void main_task(void *param)
{
    (void)param;
    LED_ABCD_ALL_OFF();

    printf("<main task>\r\n");

    const cc_dev_t dev_rx = 0, dev_tx = 1;

    if (!radio_init(dev_rx) || !radio_init(dev_tx)) goto done;

    cc_isr(dev_rx, ISR_PIN_GPIO3, ISR_EDGE_RISING, isr_mcu_wake);
    cc_set(dev_rx, CC1200_IOCFG3, CC1200_IOCFG_GPIO_CFG_MCU_WAKEUP);
    cc_isr(dev_rx, ISR_PIN_GPIO2, ISR_EDGE_RISING, isr_rssi_valid);
    cc_set(dev_rx, CC1200_IOCFG2, CC1200_IOCFG_GPIO_CFG_RSSI_VALID);

    cc_isr(dev_rx, ISR_PIN_GPIO0, ISR_EDGE_RISING, isr_carrier_sense);
    cc_set(dev_rx, CC1200_IOCFG0, CC1200_IOCFG_GPIO_CFG_CARRIER_SENSE);

    cc_isr(dev_tx, ISR_PIN_GPIO3, ISR_EDGE_RISING, isr_mcu_wake);
    cc_set(dev_tx, CC1200_IOCFG3, CC1200_IOCFG_GPIO_CFG_MCU_WAKEUP);

    //cc_update(dev, CC1200_MODCFG_DEV_E, CC1200_MODCFG_DEV_E_MODEM_MODE_M, CC1200_MODCFG_DEV_E_MODEM_MODE_CARRIER_SENSE);

    u32 freq = 907235124;

    cc_set_freq(dev_rx, freq);
    printf("[%u] f = %lu\n", dev_rx, cc_get_freq(dev_rx));
    cc_set_freq(dev_tx, freq);
    printf("[%u] f = %lu\n", dev_tx, cc_get_freq(dev_tx));

    amp_ctrl(dev_rx, AMP_LNA, false);
    amp_ctrl(dev_rx, AMP_HGM, false);
    cc_strobe(dev_rx, CC1200_SRX);

    //vTaskDelay(10 / portTICK_PERIOD_MS);

    struct {
        u8 len;
        u8 data;
    } pkt = { 1, 0 };

    amp_ctrl(dev_tx, AMP_PA, false);
    amp_ctrl(dev_tx, AMP_HGM, false);

    cc_fifo_write(dev_tx, (u8 *)&pkt, 2);
    cc_strobe(dev_tx, CC1200_STX);
    //cc_strobe(dev_tx, CC1200_STATE_FSTXON);

    vTaskSuspend(NULL);

    done:
    //vTaskDelete(NULL);

    LED_ABCD_ALL_OFF();
    LED_C_ON();

    while (1) {
        LED_C_TOGGLE();
        LED_D_TOGGLE();
        for (int i = 0; i < 10000000; ++i);
    }
}

static void isr_mcu_wake(cc_dev_t dev)
{
    u8 st = cc_strobe(dev, CC1200_SNOP | CC1200_ACCESS_READ);
    u8 ms1 = cc_get(dev, CC1200_MARC_STATUS1);
    printf("[%u] st = 0x%02X \t\t ms1  = 0x%02X\r\n", dev, st, ms1);
}

static void isr_rssi_valid(cc_dev_t dev)
{
    struct {
        s8 rssi1;
        u8 rssi0;
    } rssi;

    cc_read(dev, CC1200_RSSI1, (u8 *)&rssi, sizeof(rssi));
    printf("[%u] <RS> rssi = %i \t\t cs = %u\r\n", dev, rssi.rssi1, rssi.rssi0 & CC1200_RSSI0_CARRIER_SENSE);

}

static void isr_carrier_sense(cc_dev_t dev)
{
    struct {
        s8 rssi1;
        u8 rssi0;
    } rssi;

    cc_read(dev, CC1200_RSSI1, (u8 *)&rssi, sizeof(rssi));
    printf("[%u] <CS> rssi = %i \t\t cs = %u\r\n", dev, rssi.rssi1, rssi.rssi0 & CC1200_RSSI0_CARRIER_SENSE);

}

static bool radio_init(cc_dev_t dev)
{
    cc_spi_init(dev);
    amp_init(dev);

    cc_strobe(dev, CC1200_SRES);
    int i = 0;

    while ((i++ < 1000) && (cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_CHIP_RDYn));

    u8 pn = cc_get(dev, CC1200_PARTNUMBER);

    if (pn != 0x20) {
        cc_dbg("[%u] error: part number 0x%02X != 0x20", dev, pn);
        return false;
    }

    if (!cc_cfg_regs(dev, CC_CFG_DEFAULT, COUNT_OF(CC_CFG_DEFAULT))) {
        cc_dbg("[%u] error: could not configure (default)", dev);
        return false;
    }

    static const struct cc_cfg_reg CC_CFG_LOCAL[] = {
            {CC1200_IOCFG3, CC1200_IOCFG_GPIO_CFG_HW0},
            {CC1200_IOCFG2, CC1200_IOCFG_GPIO_CFG_HW0},
            {CC1200_IOCFG1, CC1200_IOCFG_GPIO_CFG_HIGHZ},
            {CC1200_IOCFG0, CC1200_IOCFG_GPIO_CFG_HW0},

            /* NOTE: Enabling CC1200_RFEND_CFG1_RX_TIME_QUAL_M when using a RX timeout means that
             * there will be times when the radio stays in RX due to a valid CS or PQT but will hang there
             * regardless of whether a packet was received. It seems to linger on the channel potentially
             * forever despite the lack of a sync word detection.
             * For the timeout to always trigger an interrupt, RX_TIME_QUAL must be zero.
             * */
            {CC1200_RFEND_CFG1, CC1200_RFEND_CFG1_RXOFF_MODE_IDLE | (0x7<<1) | CC1200_RFEND_CFG1_RX_TIME_QUAL_M},
            {CC1200_RFEND_CFG0, CC1200_RFEND_CFG0_TXOFF_MODE_IDLE | CC1200_RFEND_CFG0_TERM_ON_BAD_PACKET_EN},

            //{CC1200_PREAMBLE_CFG1, /*0xD*/0x4 << 2},
            //{CC1200_SYNC_CFG1,  0x09 | CC1200_SYNC_CFG1_SYNC_MODE_16/* changed from 11 */},
            {CC1200_PKT_CFG2,   CC1200_PKT_CFG2_CCA_MODE_ALWAYS},
            {CC1200_PKT_CFG1,   CC1200_PKT_CFG1_CRC_CFG_ON_INIT_1D0F | CC1200_PKT_CFG1_ADDR_CHECK_CFG_OFF | CC1200_PKT_CFG1_APPEND_STATUS},
            {CC1200_PKT_CFG0,   CC1200_PKT_CFG0_LENGTH_CONFIG_VARIABLE},
            {CC1200_PKT_LEN,    255},
            {CC1200_SERIAL_STATUS, CC1200_SERIAL_STATUS_IOC_SYNC_PINS_EN}, // Enable access to GPIO state in CC1200_GPIO_STATUS.GPIO_STATE

            {CC1200_RNDGEN,     CC1200_RNDGEN_EN}, // Needed for random backoff for LBT CCA: https://e2e.ti.com/support/wireless_connectivity/f/156/t/370230
            {CC1200_FIFO_CFG,   0 /*!CC1200_FIFO_CFG_CRC_AUTOFLUSH*/},
    };

    if (!cc_cfg_regs(dev, CC_CFG_LOCAL, COUNT_OF(CC_CFG_LOCAL))) {
        cc_dbg("[%u] error: could not configure (local)", dev);
        return false;
    }

    return cc_isr_init(dev);
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

    itm_puts(0, "!!!!!!!!!! HARD FAULT !!!!!!!!!!!!!!\r\n");
    //NVIC_SystemReset();

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

    itm_puts(0, "!!!!!!!!!! USAGE FAULT !!!!!!!!!!!!!!\r\n");

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
