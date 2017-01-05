#include <board.h>
#include <stdio.h>

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

/* (OLD!!) __attribute__((naked))*/ void UsageFault_Handler(void)
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

    printf("\r\n\v\r\n ======== HARD FAULT ======== \r\n pc: %p\r\n\r\n", fr->pc);
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



