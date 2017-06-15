#include "kinetis.h"
#include <cc/amp.h>

#include <fsl_gpio.h>
#include <fsl_port.h>

#if !CC_PA_SUPPORT
#error CC_PA_SUPPORT must be set to 1
#endif

static inline void amp_init_gpio(const sys_gpio_t *const gpio);
static inline bool amp_ctrl_gpio(const sys_gpio_t *const gpio, const bool enable);

void amp_init(cc_dev_t dev)
{
    assert(CC_DEV_VALID(dev));
    amp_init_gpio(&cc_interface[dev].amp.hgm);
    amp_init_gpio(&cc_interface[dev].amp.lna);
    amp_init_gpio(&cc_interface[dev].amp.pa);
}

void amp_ctrl(cc_dev_t dev, amp_t amp, bool enable)
{
    assert(CC_DEV_VALID(dev));
    assert(amp >= AMP_HGM && amp <= AMP_PA);

    if (amp_ctrl_gpio(&cc_interface[dev].amp.gpio[amp], enable)) {
        cc_dbg_v("[%u] %s: %s", dev,
               ((const char *[]){"hgm", "lna", "pa"})[amp],
               enable ? "on" : "off"
        );
    }
}

static inline void amp_init_gpio(const sys_gpio_t *const gpio)
{
    if (SYS_PORT_VALID(gpio->port)) {
        PORT_SetPinInterruptConfig(SYS_PORT_PORTN(gpio->port), gpio->pin, kPORT_InterruptOrDMADisabled);

        const gpio_pin_config_t gpio_pin_config = {
                .pinDirection = kGPIO_DigitalOutput,
                .outputLogic = 0
        };

        GPIO_PinInit(SYS_PORT_GPION(gpio->port), gpio->pin, &gpio_pin_config);
    }
}

static inline bool amp_ctrl_gpio(const sys_gpio_t *const gpio, const bool enable)
{
    if (!SYS_PORT_VALID(gpio->port))
        return false;

    bool enabled = GPIO_ReadPinInput(SYS_PORT_GPION(gpio->port), gpio->pin) != 0;

    if (!(enable ^ enabled))
        return false;

    if (enable)
        GPIO_SetPinsOutput(SYS_PORT_GPION(gpio->port), 1u << gpio->pin);
    else
        GPIO_ClearPinsOutput(SYS_PORT_GPION(gpio->port), 1u << gpio->pin);

    return true;
}
