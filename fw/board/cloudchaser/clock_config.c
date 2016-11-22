/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_common.h"
#include "fsl_smc.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Clock configuration structure. */
typedef struct _clock_config
{
    mcg_config_t mcgConfig;       /*!< MCG configuration.      */
    sim_clock_config_t simConfig; /*!< SIM configuration.      */
    osc_config_t oscConfig;       /*!< OSC configuration.      */
    uint32_t coreClock;           /*!< core clock frequency.   */
} clock_config_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/* Configuration for enter VLPR mode. Core clock = 4MHz. */
const clock_config_t g_defaultClockConfigVlpr = {
    .mcgConfig =
        {
            .mcgMode = kMCG_ModeBLPI,            /* Work in BLPI mode. */
            .irclkEnableMode = kMCG_IrclkEnable, /* MCGIRCLK enable.   */
            .ircs = kMCG_IrcFast,                /* Select IRC4M.      */
            .fcrdiv = 0U,                        /* FCRDIV is 0.*/

            .frdiv = 0U,
            .drs = kMCG_DrsLow,         /* Low frequency range */
            .dmx32 = kMCG_Dmx32Default, /* DCO has a default range of 25% */
            .oscsel = kMCG_OscselOsc,   /* Select OSC */

            .pll0Config =
                {
                    .enableMode = 0U, /* Don't enable PLL. */
                    .prdiv = 0U,
                    .vdiv = 0U,
                },
            .pllcs = kMCG_PllClkSelPll0,
        },
    .simConfig =
        {
            .pllFllSel = 3U, /* PLLFLLSEL select IRC48MCLK. */
            .pllFllDiv = 0U,
            .pllFllFrac = 0U,
            .er32kSrc = 2U,         /* ERCLK32K selection, use RTC. */
            .clkdiv1 = 0x00040000U, /* SIM_CLKDIV1. */
        },
    .oscConfig = {.freq = BOARD_XTAL0_CLK_HZ,
                  .capLoad = 0U,
                  .workMode = kOSC_ModeOscLowPower,
                  .oscerConfig =
                      {
                          .enableMode = kOSC_ErClkEnable,
#if (defined(FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER) && FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER)
                          .erclkDiv = 0U,
#endif
                      }},
    .coreClock = 4000000U, /* Core clock frequency */
};

/* Configuration for enter RUN mode. Core clock = 120MHz. */
const clock_config_t g_defaultClockConfigRun = {
    .mcgConfig =
        {
            .mcgMode = kMCG_ModePEE,             /* Work in PEE mode. */
            .irclkEnableMode = kMCG_IrclkEnable, /* MCGIRCLK enable. */
            .ircs = kMCG_IrcSlow,                /* Select IRC32k. */
            .fcrdiv = 0U,                        /* FCRDIV is 0. */

            .frdiv = 5U,
            .drs = kMCG_DrsLow,         /* Low frequency range */
            .dmx32 = kMCG_Dmx32Fine,    /* DCO has a default range of 25% */
            .oscsel = kMCG_OscselOsc,   /* Select OSC */

            .pll0Config =
                {
                    .enableMode = 0, .prdiv = 0x01U, .vdiv = 0x04U,
                },
            .pllcs = kMCG_PllClkSelPll0,
        },
    .simConfig =
        {
            .pllFllSel = 1U, /* PLLFLLSEL select PLL. */
            .pllFllDiv = 0U,
            .pllFllFrac = 0U,
            .er32kSrc = 2U,         /* ERCLK32K selection, use RTC. */
            .clkdiv1 = 0x01140000U, /* SIM_CLKDIV1. */
        },
    .oscConfig = {.freq = BOARD_XTAL0_CLK_HZ,
                  .capLoad = 0,
                  .workMode = kOSC_ModeOscLowPower,
                  .oscerConfig =
                      {
                          .enableMode = kOSC_ErClkEnable | kOSC_ErClkEnableInStop,
#if (defined(FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER) && FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER)
                          .erclkDiv = 1,
#endif
                      }},
    .coreClock = 120000000U, /* Core clock frequency */
};

/* Configuration for HSRUN mode. Core clock = 180MHz. */
const clock_config_t g_defaultClockConfigHsrun = {
    .mcgConfig =
        {
            .mcgMode = kMCG_ModePEE,                   /* Work in PEE mode. */
            .irclkEnableMode = kMCG_IrclkEnableInStop, /* MCGIRCLK enable. */
            .ircs = kMCG_IrcSlow,                      /* Select IRC32k.*/
            .fcrdiv = 0U,                              /* FCRDIV is 0. */

            .frdiv = 5U,
            .drs = kMCG_DrsLow,         /* Low frequency range. */
            .dmx32 = kMCG_Dmx32Default, /* DCO has a default range of 25%. */
            .oscsel = kMCG_OscselOsc,   /* Select OSC. */

            .pll0Config =
                {
                    .enableMode = kMCG_PllEnableIndependent, .prdiv = 0x01u, .vdiv = 0x0Eu,
                },
            .pllcs = kMCG_PllClkSelPll0,
        },
    .simConfig =
        {
            .pllFllSel = 1U,        /* PLLFLLSEL select PLL. */
            .er32kSrc = 2U,         /* ERCLK32K selection, use RTC. */
            .clkdiv1 = 0x02260000U, /* SIM_CLKDIV1. */
        },
    .oscConfig = {.freq = BOARD_XTAL0_CLK_HZ,
                  .capLoad = 0,
                  .workMode = kOSC_ModeOscLowPower,
                  .oscerConfig =
                      {
                          .enableMode = kOSC_ErClkEnable,
#if (defined(FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER) && FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER)
                          .erclkDiv = 0U,
#endif
                      }},
    .coreClock = 180000000U, /* Core clock frequency */
};

/* Configuration for HSRUN mode. Core clock = 180MHz. */
const clock_config_t g_defaultClockConfigOchsrun = {
    .mcgConfig =
        {
            .mcgMode = kMCG_ModePEE,                   /* Work in PEE mode. */
            .irclkEnableMode = kMCG_IrclkEnableInStop, /* MCGIRCLK enable. */
            .ircs = kMCG_IrcFast,                      /* Select IRC32k.*/
            .fcrdiv = 0U,                              /* FCRDIV is 0. */

            .frdiv = 5U,
            .drs = kMCG_DrsHigh,        /* High frequency range. */
            .dmx32 = kMCG_Dmx32Fine,    /* DCO has a default range of 25%. */
            .oscsel = kMCG_OscselOsc,   /* Select OSC. */

            .pll0Config =
                {
                        /**
                         * 234MHz: prdiv = 0x01u  vdiv = 0x17u
                         */
                    .enableMode = kMCG_PllEnableIndependent, .prdiv = 0x01U, .vdiv = 0x17U,
                },
            .pllcs = kMCG_PllClkSelPll0,
        },
    .simConfig =
        {
            .pllFllSel = 1U,        /* PLLFLLSEL select PLL. */
            .er32kSrc = 2U,         /* ERCLK32K selection, use RTC. */
            .clkdiv1 = 0x01170000U, /* SIM_CLKDIV1. */
        },
    .oscConfig = {.freq = BOARD_XTAL0_CLK_HZ,
                  .capLoad = 0,
                  .workMode = kOSC_ModeOscLowPower,
                  .oscerConfig =
                      {
                          .enableMode = kOSC_ErClkEnable,
#if (defined(FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER) && FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER)
                          .erclkDiv = 0U,
#endif
                      }},
    .coreClock = 234000000U, /* Core clock frequency */
};


/*******************************************************************************
 * Code
 ******************************************************************************/
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

void BOARD_BootClockVLPR(void)
{
    CLOCK_SetSimSafeDivs();

    CLOCK_BootToBlpiMode(g_defaultClockConfigVlpr.mcgConfig.fcrdiv, g_defaultClockConfigVlpr.mcgConfig.ircs,
                         g_defaultClockConfigVlpr.mcgConfig.irclkEnableMode);

    CLOCK_SetSimConfig(&g_defaultClockConfigVlpr.simConfig);

    SystemCoreClock = g_defaultClockConfigVlpr.coreClock;

    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeVlpr(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateVlpr)
    {
    }
}

void BOARD_BootClockRUN(void)
{
    CLOCK_SetSimSafeDivs();

    // Phillip
    CLOCK_SetXtal32Freq(BOARD_XTAL32K_CLK_HZ);

    CLOCK_InitOsc0(&g_defaultClockConfigRun.oscConfig);
    CLOCK_SetXtal0Freq(BOARD_XTAL0_CLK_HZ);

    CLOCK_BootToPeeMode(g_defaultClockConfigRun.mcgConfig.oscsel, kMCG_PllClkSelPll0,
                        &g_defaultClockConfigRun.mcgConfig.pll0Config);

    CLOCK_SetInternalRefClkConfig(g_defaultClockConfigRun.mcgConfig.irclkEnableMode,
                                  g_defaultClockConfigRun.mcgConfig.ircs, g_defaultClockConfigRun.mcgConfig.fcrdiv);

    CLOCK_SetSimConfig(&g_defaultClockConfigRun.simConfig);

    SystemCoreClock = g_defaultClockConfigRun.coreClock;
}

void BOARD_BootClockHSRUN(void)
{
    // TODO: CLOCK_SetMcgConfig() ???

    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeHsrun(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateHsrun)
    {
    }

    CLOCK_SetSimSafeDivs();

    // Phillip
    CLOCK_SetXtal32Freq(BOARD_XTAL32K_CLK_HZ);

    CLOCK_InitOsc0(&g_defaultClockConfigHsrun.oscConfig);
    CLOCK_SetXtal0Freq(BOARD_XTAL0_CLK_HZ);

    CLOCK_BootToPeeMode(g_defaultClockConfigHsrun.mcgConfig.oscsel, kMCG_PllClkSelPll0,
                        &g_defaultClockConfigHsrun.mcgConfig.pll0Config);

    CLOCK_SetInternalRefClkConfig(g_defaultClockConfigHsrun.mcgConfig.irclkEnableMode,
                                  g_defaultClockConfigHsrun.mcgConfig.ircs, g_defaultClockConfigHsrun.mcgConfig.fcrdiv);

    CLOCK_SetSimConfig(&g_defaultClockConfigHsrun.simConfig);

    SystemCoreClock = g_defaultClockConfigHsrun.coreClock;
}

void BOARD_BootClockOCHSRUN(void)
{
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeHsrun(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateHsrun)
    {
    }

    CLOCK_SetSimSafeDivs();

    // Phillip
    CLOCK_SetXtal32Freq(BOARD_XTAL32K_CLK_HZ);

    CLOCK_InitOsc0(&g_defaultClockConfigOchsrun.oscConfig);
    CLOCK_SetXtal0Freq(BOARD_XTAL0_CLK_HZ);

    CLOCK_BootToPeeMode(g_defaultClockConfigOchsrun.mcgConfig.oscsel, kMCG_PllClkSelPll0,
                        &g_defaultClockConfigOchsrun.mcgConfig.pll0Config);

    CLOCK_SetInternalRefClkConfig(g_defaultClockConfigOchsrun.mcgConfig.irclkEnableMode,
                                  g_defaultClockConfigOchsrun.mcgConfig.ircs, g_defaultClockConfigOchsrun.mcgConfig.fcrdiv);

    CLOCK_SetSimConfig(&g_defaultClockConfigOchsrun.simConfig);

    SystemCoreClock = g_defaultClockConfigOchsrun.coreClock;
}
