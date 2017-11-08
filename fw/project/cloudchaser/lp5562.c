#include "lp5562.h"

#include <kio/iic.h>
#include <malloc.h>
#include <FreeRTOS.h>
#include <task.h>
#include <kio/itm.h>


#define LP5562_PROGRAM_LENGTH		32
#define LP5562_MAX_LEDS			4

/* ENABLE Register 00h */
#define LP5562_REG_ENABLE		0x00
#define LP5562_EXEC_ENG1_M		0x30
#define LP5562_EXEC_ENG2_M		0x0C
#define LP5562_EXEC_ENG3_M		0x03
#define LP5562_EXEC_M			0x3F
#define LP5562_MASTER_ENABLE		0x40	/* Chip master enable */
#define LP5562_LOGARITHMIC_PWM		0x80	/* Logarithmic PWM adjustment */
#define LP5562_EXEC_RUN			0x2A
#define LP5562_ENABLE_DEFAULT	\
	(LP5562_MASTER_ENABLE | LP5562_LOGARITHMIC_PWM)
#define LP5562_ENABLE_RUN_PROGRAM	\
	(LP5562_ENABLE_DEFAULT | LP5562_EXEC_RUN)

/* OPMODE Register 01h */
#define LP5562_REG_OP_MODE		0x01
#define LP5562_MODE_ENG1_M		0x30
#define LP5562_MODE_ENG2_M		0x0C
#define LP5562_MODE_ENG3_M		0x03
#define LP5562_LOAD_ENG1		0x10
#define LP5562_LOAD_ENG2		0x04
#define LP5562_LOAD_ENG3		0x01
#define LP5562_RUN_ENG1			0x20
#define LP5562_RUN_ENG2			0x08
#define LP5562_RUN_ENG3			0x02
#define LP5562_ENG1_IS_LOADING(mode)	\
	((mode & LP5562_MODE_ENG1_M) == LP5562_LOAD_ENG1)
#define LP5562_ENG2_IS_LOADING(mode)	\
	((mode & LP5562_MODE_ENG2_M) == LP5562_LOAD_ENG2)
#define LP5562_ENG3_IS_LOADING(mode)	\
	((mode & LP5562_MODE_ENG3_M) == LP5562_LOAD_ENG3)

/* BRIGHTNESS Registers */
#define LP5562_REG_R_PWM		LP5562_PWM_RED
#define LP5562_REG_G_PWM		LP5562_PWM_GREEN
#define LP5562_REG_B_PWM		LP5562_PWM_BLUE
#define LP5562_REG_W_PWM		LP5562_PWM_WHITE

/* CURRENT Registers */
#define LP5562_REG_R_CURRENT		LP5562_CURRENT_RED
#define LP5562_REG_G_CURRENT		LP5562_CURRENT_GREEN
#define LP5562_REG_B_CURRENT		LP5562_CURRENT_BLUE
#define LP5562_REG_W_CURRENT		LP5562_CURRENT_WHITE

/* CONFIG Register 08h */
#define LP5562_REG_CONFIG		0x08
#define LP5562_PWM_HF			0x40
#define LP5562_PWRSAVE_EN		0x20
#define LP5562_CLK_INT			0x01	/* Internal clock */
#define LP5562_DEFAULT_CFG		(LP5562_PWM_HF | LP5562_PWRSAVE_EN)

/* RESET Register 0Dh */
#define LP5562_REG_RESET		0x0D
#define LP5562_RESET			0xFF

/* PROGRAM ENGINE Registers */
#define LP5562_REG_PROG_MEM_ENG1	0x10
#define LP5562_REG_PROG_MEM_ENG2	0x30
#define LP5562_REG_PROG_MEM_ENG3	0x50

/* LEDMAP Register 70h */
#define LP5562_REG_ENG_SEL		0x70
#define LP5562_ENG_SEL_PWM		0
#define LP5562_ENG_FOR_RGB_M		0x3F
#define LP5562_ENG_SEL_RGB		0x1B	/* R:ENG1, G:ENG2, B:ENG3 */
#define LP5562_ENG_FOR_W_M		0xC0
#define LP5562_ENG1_FOR_W		0x40	/* W:ENG1 */
#define LP5562_ENG2_FOR_W		0x80	/* W:ENG2 */
#define LP5562_ENG3_FOR_W		0xC0	/* W:ENG3 */

/* Program Commands */
#define LP5562_CMD_DISABLE		0x00
#define LP5562_CMD_LOAD			0x15
#define LP5562_CMD_RUN			0x2A
#define LP5562_CMD_DIRECT		0x3F
#define LP5562_PATTERN_OFF		0


struct __packed lp5562 {
    iic_t iic;
    u8 addr;
};


static u8 lp5562_read(lp5562_t lp, u8 reg);
static void lp5562_write(lp5562_t lp, u8 reg, u8 val);


static struct lp5562 lp5562s[LP5562_NUM_DEVICES] = {{NULL}};

lp5562_t lp5562_init(u8 addr)
{
    u8 idx;

    for (idx = 0; idx < LP5562_NUM_DEVICES; ++idx) {
        if (!lp5562s[idx].addr) break;
    }

    if (idx == LP5562_NUM_DEVICES) return NULL;

    lp5562_t const lp = &lp5562s[idx];

    lp->iic = iic_init(0, 400000);
    lp->addr = addr;

    lp5562_reset(lp);
    vTaskDelay(pdMS_TO_TICKS(10));
    lp5562_write(lp, LP5562_REG_ENABLE, LP5562_ENABLE_DEFAULT);
    vTaskDelay(pdMS_TO_TICKS(1));
    lp5562_write(lp, LP5562_REG_CONFIG, LP5562_PWM_HF | LP5562_CLK_INT);
    lp5562_write(lp, LP5562_REG_OP_MODE, LP5562_CMD_DIRECT);
    lp5562_write(lp, LP5562_REG_ENG_SEL, LP5562_ENG_SEL_PWM);

    lp5562_set_current(lp, LP5562_CURRENT_BLUE, 255);
    lp5562_set_current(lp, LP5562_CURRENT_GREEN, 255);
    lp5562_set_current(lp, LP5562_CURRENT_RED, 255);
    lp5562_set_current(lp, LP5562_CURRENT_WHITE, 255);
    lp5562_set_pwm(lp, LP5562_PWM_BLUE, 0);
    lp5562_set_pwm(lp, LP5562_PWM_GREEN, 0);
    lp5562_set_pwm(lp, LP5562_PWM_RED, 0);
    lp5562_set_pwm(lp, LP5562_PWM_WHITE, 0);

    return lp;
}

static u8 lp5562_read(lp5562_t lp, u8 reg)
{
    u8 val = 0;
    iic_io(lp->iic, IIC_READ, lp->addr, reg, &val, sizeof(val));
    return val;
}

static void lp5562_write(lp5562_t lp, u8 reg, u8 val)
{
    iic_io(lp->iic, IIC_WRITE, lp->addr, reg, &val, sizeof(val));
}

void lp5562_reset(lp5562_t lp)
{
    // NOTE: No ACK, may need different txn or error cleared
    lp5562_write(lp, LP5562_REG_RESET, LP5562_RESET);
}

void lp5562_set_current(lp5562_t lp, u8 chan, u8 current)
{
    lp5562_write(lp, chan, current);

    /*if (chan == LP5562_CURRENT_RED) {
        itm_printf(0, "lp5562 red set=%u desired=%u\n", lp5562_read(lp, chan), current);
    }*/
}


void lp5562_set_pwm(lp5562_t lp, u8 chan, u8 pwm)
{
    lp5562_write(lp, chan, pwm);
}


u8 lp5562_get_current(lp5562_t lp, u8 chan)
{
    return lp5562_read(lp, chan);
}
