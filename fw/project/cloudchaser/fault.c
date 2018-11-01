#include <board.h>
#include <stdio.h>

/* http://www.freertos.org/Debugging-Hard-Faults-On-Cortex-M-Microcontrollers.html */

#define SYSHND_CTRL  (*(volatile unsigned int*)  (0xE000ED24u))  // System Handler Control and State Register
#define NVIC_MFSR    (*(volatile unsigned char*) (0xE000ED28u))  // Memory Management Fault Status Register
#define NVIC_BFSR    (*(volatile unsigned char*) (0xE000ED29u))  // Bus Fault Status Register
#define NVIC_UFSR    (*(volatile unsigned short*)(0xE000ED2Au))  // Usage Fault Status Register
#define NVIC_HFSR    (*(volatile unsigned short*)(0xE000ED2Cu))  // Hard Fault Status Register
#define NVIC_DFSR    (*(volatile unsigned short*)(0xE000ED30u))  // Debug Fault Status Register
#define NVIC_BFAR    (*(volatile unsigned int*)  (0xE000ED38u))  // Bus Fault Manage Address Register
#define NVIC_AFSR    (*(volatile unsigned short*)(0xE000ED3Cu))  // Auxiliary Fault Status Register

__attribute__((naked, interrupt, used)) void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r2, =hard_fault                                       \n"
        " bx r2                                                     \n"
    );
}

__attribute__((naked, interrupt, used)) void UsageFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r2, =usage_fault                                      \n"
        " bx r2                                                     \n"
    );
}


typedef struct {
    volatile uint32_t r0;            // Register R0
    volatile uint32_t r1;            // Register R1
    volatile uint32_t r2;            // Register R2
    volatile uint32_t r3;            // Register R3
    volatile uint32_t r12;           // Register R12
    volatile uint32_t lr;            // Link register
    volatile uint32_t pc;            // Program counter
    union {
        volatile uint32_t byte;
        struct {
            uint32_t IPSR : 8;           // Interrupt Program Status register (IPSR)
            uint32_t EPSR : 19;          // Execution Program Status register (EPSR)
            uint32_t APSR : 5;           // Application Program Status register (APSR)
        } bits;
    } psr;                               // Program status register.

} fault_reg_t;


typedef struct {

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

} fault_info_t;


static fault_info_t fault_info __used;
static bool hard_fault_continue __used = false;

void __used hard_fault(fault_reg_t *fr)
{
    fault_info.syshndctrl.byte = SYSHND_CTRL;  // System Handler Control and State Register
    fault_info.mfsr.byte       = NVIC_MFSR;    // Memory Fault Status Register
    fault_info.bfsr.byte       = NVIC_BFSR;    // Bus Fault Status Register
    fault_info.bfar            = NVIC_BFAR;    // Bus Fault Manage Address Register
    fault_info.ufsr.byte       = NVIC_UFSR;    // Usage Fault Status Register
    fault_info.hfsr.byte       = NVIC_HFSR;    // Hard Fault Status Register
    fault_info.dfsr.byte       = NVIC_DFSR;    // Debug Fault Status Register
    fault_info.afsr            = NVIC_AFSR;    // Auxiliary Fault Status Register

    itm_puts(0, "\r\n\v\r\n ======== HARD FAULT ======== \r\n\r\n");

    while (!hard_fault_continue) __NOP();
    NVIC_SystemReset();
}

void __used usage_fault( fault_reg_t *fr __unused )
{
    while (!hard_fault_continue) __NOP();
}



