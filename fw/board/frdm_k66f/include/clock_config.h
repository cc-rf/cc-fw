#pragma once

#define BOARD_XTAL0_CLK_HZ 12000000U
#define BOARD_XTAL32K_CLK_HZ 32768U

void BOARD_BootClockVLPR(void);
void BOARD_BootClockRUN(void);
void BOARD_BootClockHSRUN(void);
