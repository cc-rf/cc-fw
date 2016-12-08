
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include <string.h>
#include <stdio.h>
#include <itm.h>


#include <fsl_i2c_edma.h>
#include <fsl_dma_manager.h>
#include <semphr.h>

static edma_handle_t i2c1_edma_handle = {{0}};
static i2c_master_edma_handle_t i2c1_handle = {{0}};
static xSemaphoreHandle i2c_sem;
static xSemaphoreHandle i2c_mtx;

void i2c1_callback(I2C_Type *base, i2c_master_edma_handle_t *handle, status_t status, void *userData);


void i2c_init(void)
{
    status_t status;

    i2c_mtx = xSemaphoreCreateMutex();
    i2c_sem = xSemaphoreCreateBinary();

    DMAMGR_Init();

    if ((status = DMAMGR_RequestChannel(kDmaRequestMux0I2C1, DMAMGR_DYNAMIC_ALLOCATE, &i2c1_edma_handle))) {
        printf("i2c_init: DMAMGR_RequestChannel(): status=%li\r\n", status);
    } else {
        i2c_master_config_t config;
        I2C_MasterGetDefaultConfig(&config);
        config.baudRate_Bps = 400000;

        I2C_MasterInit(I2C1, &config, CLOCK_GetFreq(I2C1_CLK_SRC));
        I2C_MasterCreateEDMAHandle(I2C1, &i2c1_handle, i2c1_callback, (void *) i2c_sem, &i2c1_edma_handle);

        NVIC_SetPriority(((IRQn_Type[]) DMA_CHN_IRQS)[i2c1_edma_handle.channel],
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    }
}

void i2c_io(u8 addr, u8 cmd, i2c_direction_t dir, void *data, size_t len)
{
    i2c_master_transfer_t xfer = {
            .flags = kI2C_TransferDefaultFlag,
            .slaveAddress = addr,
            .direction = dir,
            .subaddress = cmd,
            .subaddressSize = sizeof(cmd),
            .data = data,
            .dataSize = len,
    };

    xSemaphoreTake(i2c_mtx, portMAX_DELAY);
    const status_t status = I2C_MasterTransferEDMA(I2C1, &i2c1_handle, &xfer);

    if (status == kStatus_Success) {
        if (len > 1) {
            xSemaphoreTake(i2c_sem, portMAX_DELAY);
        }
    } else {
        printf("<fail> st=%li\r\n", status);
    }

    xSemaphoreGive(i2c_mtx);
}

void i2c_tx(u8 addr, u8 cmd, void *data, size_t len)
{
    return i2c_io(addr, cmd, kI2C_Write, data, len);
}

void i2c_rx(u8 addr, u8 cmd, void *data, size_t len)
{
    return i2c_io(addr, cmd, kI2C_Read, data, len);
}

void i2c1_callback(I2C_Type *base, i2c_master_edma_handle_t *handle, status_t status, void *userData)
{
    BaseType_t xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR((xSemaphoreHandle)userData, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

#include "bno055.h"

s8 bno055_tx(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    i2c_tx(dev_addr, reg_addr, reg_data, cnt);
    return BNO055_SUCCESS;
}

s8 bno055_rx(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    i2c_rx(dev_addr, reg_addr, reg_data, cnt);
    return BNO055_SUCCESS;
}

#define BNO055_I2C_ADDR         0x28

#define BNO055_CHIP_ID_ADDR     0x00
#define BNO055_ID               0xA0

void bno_init(void)
{
    i2c_init();

    struct bno055_t bno055 = {
            .dev_addr = BNO055_I2C_ADDR,
            .bus_read = bno055_rx,
            .bus_write = bno055_tx,
    };

    bno055_init(&bno055);

    printf("bno: id=%u exp=%u\r\n", bno055.chip_id, BNO055_ID);

    //bno055_set_operation_mode(BNO055_OPERATION_MODE_ACCONLY);

    struct bno055_accel_t accel;
    //struct bno055_accel_float_t accel;

    while (1) {
        bno055_read_accel_xyz(&accel);
        printf("%u\t%u\t%u\r\n", accel.x, accel.y, accel.z);
        //bno055_convert_float_accel_xyz_msq(&accel);
        //printf("%.02f\t%.02f\t%.02f\r\n", accel.x, accel.y, accel.z);
    }

}




extern void vcom_init(void);

static void main_task(void *param);

int main(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    itm_init();
    printf("<boot>\r\n");


    printf("\nclocks:\n  core\t\t\t= %lu\n  bus\t\t\t= %lu\n  flexbus\t\t= %lu\n  flash\t\t\t= %lu\n  pllfllsel\t\t= %lu\n  osc0er\t\t= %lu\n  osc0erundiv\t\t= %lu\n  mcgfixedfreq\t\t= %lu\n  mcginternalref\t= %lu\n  mcgfll\t\t= %lu\n  mcgpll0\t\t= %lu\n  mcgirc48m\t\t= %lu\n  lpo\t\t\t= %lu\n\n",
       CLOCK_GetFreq(kCLOCK_CoreSysClk),
       CLOCK_GetFreq(kCLOCK_BusClk),
       CLOCK_GetFreq(kCLOCK_FlexBusClk),
       CLOCK_GetFreq(kCLOCK_FlashClk),
       CLOCK_GetFreq(kCLOCK_PllFllSelClk),
       CLOCK_GetFreq(kCLOCK_Osc0ErClk),
       CLOCK_GetFreq(kCLOCK_Osc0ErClkUndiv),
       CLOCK_GetFreq(kCLOCK_McgFixedFreqClk),
       CLOCK_GetFreq(kCLOCK_McgInternalRefClk),
       CLOCK_GetFreq(kCLOCK_McgFllClk),
       CLOCK_GetFreq(kCLOCK_McgPll0Clk),
       CLOCK_GetFreq(kCLOCK_McgIrc48MClk),
       CLOCK_GetFreq(kCLOCK_LpoClk)
    );

    if (xTaskCreate(main_task, "main", TASK_STACK_SIZE_DEFAULT, NULL, TASK_PRIO_HIGHEST, NULL) != pdPASS) goto done;
    vTaskStartScheduler();

    //LED_BLUE_ON();
    LED_A_ON();
    //vcom_init();

    done:

    while (1) {
        for (int i = 0; i < 1000000; ++i);
    }
}

static void main_task(void *param)
{
    (void)param;

    printf("<main task>\r\n");
    bno_init();


    //LED_BLUE_ON();
    //vcom_init();
    //vTaskSuspend(NULL);

    done:
    //vTaskDelete(NULL);

    while (1) {
        for (int i = 0; i < 1000000; ++i);
    }
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
static bool hard_fault_continue = false;
static unsigned long long usage_fault_count;

static fault_reg_t hard_fault_last_reg;

void hard_fault( fault_reg_t *fr/*, uint32_t lr __attribute__((unused))*/ )
{
    static int hard_fault_count_init;

    printf("!!!!!!!!!! HARD FAULT !!!!!!!!!!!!!!\r\n");
    while (!hard_fault_continue) asm("nop");
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
