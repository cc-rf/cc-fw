#include <board.h>
#include <clock_config.h>
#include <pin_mux.h>

int main(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();

    while (1) {
        asm("nop");
    }
}
