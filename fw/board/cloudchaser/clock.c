#include <board/board.h>
#include <board/clock.h>
#include <fsl_common.h>
#include <fsl_smc.h>


static void BOARD_InitOsc0(void);


/*
 * How to setup clock using clock driver functions:
 *
 * 1. CLOCK_SetSimSafeDivs, to make sure core clock, bus clock, flexbus clock
 *    and flash clock are in allowed range during clock mode switch.
 *
 * 2. Call CLOCK_Osc0Init to setup OSC clock, if it is used in target mode.
 *
 * 3. Set MCG configuration, MCG includes three parts: FLL clock, PLL clock and
 *    internal reference clock(MCGIRCLK). Follow the steps to setup:
 *
 *    1). Call CLOCK_BootToXxxMode to set MCG to target mode.
 *
 *    2). If target mode is FBI/BLPI/PBI mode, the MCGIRCLK has been configured
 *        correctly. For other modes, need to call CLOCK_SetInternalRefClkConfig
 *        explicitly to setup MCGIRCLK.
 *
 *    3). Don't need to configure FLL explicitly, because if target mode is FLL
 *        mode, then FLL has been configured by the function CLOCK_BootToXxxMode,
 *        if the target mode is not FLL mode, the FLL is disabled.
 *
 *    4). If target mode is PEE/PBE/PEI/PBI mode, then the related PLL has been
 *        setup by CLOCK_BootToXxxMode. In FBE/FBI/FEE/FBE mode, the PLL could
 *        be enabled independently, call CLOCK_EnablePll0 explicitly in this case.
 *
 * 4. Call CLOCK_SetSimConfig to set the clock configuration in SIM.
 */


void boot_clock_vlpr(void)
{
    const sim_clock_config_t simConfig = {
            .pllFllSel = 3U, .pllFllDiv = 0U, .pllFllFrac = 0U, .er32kSrc = 2U, .clkdiv1 = 0x00040000U,
    };

    CLOCK_SetSimSafeDivs();

    CLOCK_BootToBlpiMode(0U, kMCG_IrcFast, kMCG_IrclkEnable);

    CLOCK_SetSimConfig(&simConfig);

    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeVlpr(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateVlpr)
    {
    }
}

void boot_clock_lprun(void)
{
    // 96MHz
    /*const mcg_pll_config_t pll0Config = {
            .enableMode = 0U, .prdiv = 0x01U, .vdiv = 0x00U,
    };

    const sim_clock_config_t simConfig = {
            .pllFllSel = 1U, .pllFllDiv = 0U, .pllFllFrac = 0U, .er32kSrc = 2U, .clkdiv1 = 0x00020000U,
    };*/

    // 64MHz
    const mcg_pll_config_t pll0Config = {
            .enableMode = 0U, .prdiv = 0x02U, .vdiv = 0x00U,
    };

    const sim_clock_config_t simConfig = {
            .pllFllSel = 1U, .pllFllDiv = 0U, .pllFllFrac = 0U, .er32kSrc = 2U, .clkdiv1 = 0x00010000U,
    };

    // 32MHz
    /*const mcg_pll_config_t pll0Config = {
            .enableMode = 0U, .prdiv = 0x05U, .vdiv = 0x00U,
    };

    const sim_clock_config_t simConfig = {
            .pllFllSel = 1U, .pllFllDiv = 0U, .pllFllFrac = 0U, .er32kSrc = 2U, .clkdiv1 = 0x00000000U,
    };*/



    CLOCK_SetSimSafeDivs();

    BOARD_InitOsc0();

    CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

    CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow, 0U); // TODO: Find out why I was using FCRDIV of 5?

    CLOCK_SetSimConfig(&simConfig);

    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeRun(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateRun)
    {
    }
}

void boot_clock_run(void)
{
    const mcg_pll_config_t pll0Config = {
            .enableMode = 0U, .prdiv = 0x01U, .vdiv = 0x04U,
    };

    const sim_clock_config_t simConfig = {
            .pllFllSel = 1U, .pllFllDiv = 0U, .pllFllFrac = 0U, .er32kSrc = 2U, .clkdiv1 = 0x01140000U,
    };

    CLOCK_SetSimSafeDivs();

    BOARD_InitOsc0();

    CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

    CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow, 0U); // TODO: Find out why I was using FCRDIV of 5?

    CLOCK_SetSimConfig(&simConfig);

    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeRun(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateRun)
    {
    }
}

void boot_clock_run_hs(void)
{
    const mcg_pll_config_t pll0Config = {
            .enableMode = 0U, .prdiv = 0x01U, .vdiv = 0x0EU,
    };
    const sim_clock_config_t simConfig = {
            .pllFllSel = 1U, .er32kSrc = 2U, .clkdiv1 = 0x01150000U,
    };

    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeHsrun(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateHsrun)
    {
    }

    CLOCK_SetSimSafeDivs();

    BOARD_InitOsc0();

    CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

    CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow, 0U);

    CLOCK_SetSimConfig(&simConfig);
}

void boot_clock_run_hs_oc(void)
{
    const mcg_pll_config_t pll0Config = {
            .enableMode = 0U, .prdiv = 0x01U, .vdiv = 0x14U,
    };
    const sim_clock_config_t simConfig = {
            .pllFllSel = 1U, .er32kSrc = 2U, .clkdiv1 = 0x01170000U,
    };

    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeHsrun(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateHsrun)
    {
    }

    CLOCK_SetSimSafeDivs();

    BOARD_InitOsc0();

    CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

    CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow, 0U);

    CLOCK_SetSimConfig(&simConfig);
}

void BOARD_InitOsc0(void)
{
    const osc_config_t oscConfig = {.freq = BOARD_XTAL0_CLK_HZ,
            .capLoad = 0,
            .workMode = kOSC_ModeOscLowPower,
            .oscerConfig = {
                    .enableMode = kOSC_ErClkEnable | kOSC_ErClkEnableInStop,
#if (defined(FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER) && FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER)
                    .erclkDiv = 0U, /* phillip: why was this 2 (val = 1) before? I think some component requires osc0er to be < 24MHz? */
#endif
            }};

    CLOCK_InitOsc0(&oscConfig);

    /* Passing the XTAL0 frequency to clock driver. */
    CLOCK_SetXtal0Freq(BOARD_XTAL0_CLK_HZ);

    #if defined(BOARD_XTAL32K_CLK_HZ) && BOARD_XTAL32K_CLK_HZ > 0
    /* phillip: also set this one */
    CLOCK_SetXtal32Freq(BOARD_XTAL32K_CLK_HZ);
    #endif
}
