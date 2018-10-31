#include "ftm.h"

#include <kio/itm.h>

#include <fsl_ftm.h>


/**
 * 800kHz signal. Period = 1.25us.
 * First channel edge @ 0.26us = 21% duty cycle
 * Second channel edge @ 0.7us = 56%
 * Third channel edge @ 1.06us = 85%
 */

void ftm_init(void)
{
    status_t status;
    ftm_config_t ftm_config;
    ftm_chnl_pwm_signal_param_t chnl_param[3] = {
            {
                    .chnlNumber = kFTM_Chnl_0,
                    .dutyCyclePercent = 21,
                    .firstEdgeDelayPercent = 0,
                    .level = kFTM_NoPwmSignal
            },
            {
                    .chnlNumber = kFTM_Chnl_1,
                    .dutyCyclePercent = 56,
                    .firstEdgeDelayPercent = 0,
                    .level = kFTM_NoPwmSignal
            },
            {
                    .chnlNumber = kFTM_Chnl_2,
                    .dutyCyclePercent = 85,
                    .firstEdgeDelayPercent = 0,
                    .level = kFTM_NoPwmSignal
            }
    };

    FTM_GetDefaultConfig(&ftm_config);

    //ftm_config.prescale = kFTM_Prescale_Divide_4;

    if ((status = FTM_Init(FTM, &ftm_config))) {
        printf("ftm init fail: status=%lu\n", status);
        return;
    }

    FTM_DisableInterrupts(FTM, UINT32_MAX);

    status = FTM_SetupPwm(
            FTM, chnl_param, ARRAY_SIZE(chnl_param), kFTM_EdgeAlignedPwm,
            800000, CLOCK_GetBusClkFreq()
    );

    if (status) {
        printf("ftm pwm setup fail: status=%lu\n", status);
        return;
    }

    // Enable DMA and Channel Interrupt
    FTM->CONTROLS[kFTM_Chnl_0].CnSC |= FTM_CnSC_DMA(1) | FTM_CnSC_CHIE(1);
    FTM->CONTROLS[kFTM_Chnl_1].CnSC |= FTM_CnSC_DMA(1) | FTM_CnSC_CHIE(1);
    FTM->CONTROLS[kFTM_Chnl_2].CnSC |= FTM_CnSC_DMA(1) | FTM_CnSC_CHIE(1);
}
