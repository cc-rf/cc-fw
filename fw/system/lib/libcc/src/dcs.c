#include <cc/common.h>
#include <cc/mac.h>
#include <cc/cfg.h>
#include <cc/phy.h>
#include <cc/amp.h>
#include <cc/chan.h>
#include <assert.h>
#include <FreeRTOS.h>
#include <semphr.h>


static bool dcs_init_dev(cc_dev_t dev);
static bool dcs_setup(cc_dev_t dev);

static void dcs_isr1(void);
static void dcs_rx_task(cc_dev_t dev);
static void dcs_tx_task(cc_dev_t dev);

bool dcs_init(void) // needs: rx handler
{
    for (cc_dev_t dev = CC_DEV_MIN; dev <= CC_DEV_MAX; ++dev)
        if (!dcs_init_dev(dev)) return false;
}

static bool dcs_init_dev(cc_dev_t dev)
{
    cc_spi_init(dev);
    if (!cc_isr_init(dev)) return false;
    return dcs_setup(dev);
}

static bool dcs_setup(cc_dev_t dev)
{
    // 1. config: default, phy, user
    // 3. isr config
    // 4. start rx (sep. function but internal)
}

void dcs_send(void *data, size_t len)
{
    // 1. package meta
    // 2. only queue or (2b) block and send
}

void dcs_task_tx(void *p __attribute__((unused)))
{
    xQueueHandle queue;
    void tx_info;

    while (1) {
        xQueueReceive(queue, &tx_info, portMAX_DELAY);

        // determine which radio
        // cancel current rx, or
        // fill TX fifo and STX from RX

        // complications:
        // - CCA, and whether to do LBT and whether to enable packet receipt
        // - Addressing
        // - Timing/timeout
        // - ACK requests
    }
}