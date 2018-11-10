
#include <usr/type.h>
#include <board/board.h>
#include "fsl_device_registers.h"

void SystemInit(void)
{
    #if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
        SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));    /* set CP10, CP11 Full Access */
    #endif /* ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) */

    #if (DISABLE_WDOG)
        WDOG->UNLOCK = WDOG_UNLOCK_WDOGUNLOCK(0xC520); /* Key 1 */
        WDOG->UNLOCK = WDOG_UNLOCK_WDOGUNLOCK(0xD928); /* Key 2 */
        /* WDOG->STCTRLH: ?=0,DISTESTWDOG=0,BYTESEL=0,TESTSEL=0,TESTWDOG=0,?=0,?=1,WAITEN=1,STOPEN=1,DBGEN=0,ALLOWUPDATE=1,WINEN=0,IRQRSTEN=0,CLKSRC=1,WDOGEN=0 */
        WDOG->STCTRLH = WDOG_STCTRLH_BYTESEL(0x00) |
                     WDOG_STCTRLH_WAITEN_MASK |
                     WDOG_STCTRLH_STOPEN_MASK |
                     WDOG_STCTRLH_ALLOWUPDATE_MASK |
                     WDOG_STCTRLH_CLKSRC_MASK |
                     0x0100U;
    #endif /* (DISABLE_WDOG) */

    /* https://sourceforge.net/p/gnuarmeclipse/se/ci/d67a27d285f8ac2de5190b889203caf4c7d02c77/ */
    /* Phillip: Enable usage fault handling? */
    //SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk;

    // phillip: disable usb vreg inrush current limiter
    // errata: Mask Set Errata for Mask 0N65N, Rev. 17Nov2015
    #if FSL_FEATURE_SIM_OPT_HAS_USB_PHY
    SIM->USBPHYCTL |= SIM_USBPHYCTL_USBDISILIM_MASK;
    #endif

    // phillip: disable usb vreg power to mcu (should pull from LDO instead?)
    // doesn't work... leads to hang... wtf?
    //#if FSL_FEATURE_SIM_OPT_HAS_USB_VOLTAGE_REGULATOR
    //  SIM_SetUsbVoltRegulatorEnableMode(0);
    //#endif

    board_boot();
}
