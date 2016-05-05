#include <cc/mac.h>
#include <cc/common.h>
#include <cc/cfg.h>
#include <cc/phy.h>
#include <cc/amp.h>
#include <cc/io.h>
#include <cc/cc1200.h>
#include <cc/chan.h>
#include <assert.h>

#define MAC_NUM_DEVICES     CC_NUM_DEVICES
#define MAC_DEV_MIN         CC_DEV_MIN
#define MAC_DEV_MAX         CC_DEV_MAX
#define MAC_NUM_CHANNELS    50

static bool mac_setup(cc_dev_t dev);
static void mac_phy_signal(cc_dev_t dev, u8 ms1, void *param);
static void mac_phy_rx(cc_dev_t dev, u8 *buf, u8 len);


static const struct cc_cfg_reg CC_CFG_MAC[] = {
        //{CC1200_SETTLING_CFG, 0x3}, // Defaults except never auto calibrate
};

static mac_cfg_t mac_cfg;

static phy_cfg_t phy_cfg = {
        .rx = mac_phy_rx,
        .signal = mac_phy_signal
};

static struct {
    chan_grp_t group;
    chan_inf_t chan[MAC_NUM_CHANNELS];

} channels[MAC_NUM_DEVICES] = {
        {
                .group = {
                        .dev = 0,
                        .freq = {
                                .min = 902000000,
                                .max = 928000000
                        },
                        .size = MAC_NUM_CHANNELS
                },
        },
        {
                .group = {
                        .dev = 1,
                        .freq = {
                                .min = 902000000,
                                .max = 928000000
                        },
                        .size = MAC_NUM_CHANNELS
                },
        },
};

volatile chan_inf_t *chan_cur[MAC_NUM_DEVICES];

extern pit_t xsec_timer;
static pit_tick_t xsec_last[MAC_NUM_DEVICES];

bool mac_init(mac_cfg_t *cfg)
{
    assert(cfg);
    mac_cfg = *cfg;

    for (cc_dev_t dev = MAC_DEV_MIN; dev <= MAC_DEV_MAX; ++dev)
        if (!phy_init(dev, &phy_cfg) || !mac_setup(dev)) return false;

    return true;
}

static bool mac_setup(cc_dev_t dev)
{
    if (!cc_cfg_regs(dev, CC_CFG_DEFAULT, COUNT_OF(CC_CFG_DEFAULT))) {
        cc_dbg("[%u] error: could not configure (default)", dev);
        return false;
    }

    if (!cc_cfg_regs(dev, CC_CFG_MAC, COUNT_OF(CC_CFG_MAC))) {
        cc_dbg("[%u] error: could not configure (mac)", dev);
        return false;
    }

    amp_init(dev);

    /**
     * Need 3 delay parameters:
     *  - Time until CS invalid,
     *  - Time to remain in channel for next packet,
     *  - Overall max time in RX without any event.
     * It would be nice to set max time in RX after first byte received.
     * Perhaps start timer at SYNC_RXTX using max length as a guide.
     * Also when remaining in channel for more packets, need to have a limit
     * of how long we stay there.
     */

    cc_set_rx_timeout(dev, 10000000);

    chan_grp_init(&channels[dev].group);
    //chan_grp_calibrate(&channels[dev].group);
    chan_select(&channels[dev].group, 0);
    chan_cur[dev] = &channels[dev].chan[0];

    xsec_last[dev] = pit_get_elapsed(xsec_timer);
    return true;
}

void mac_task(void)
{
    phy_task();

    pit_tick_t xsec_cur = pit_get_elapsed(xsec_timer);

    for (cc_dev_t dev = MAC_DEV_MIN; dev <= MAC_DEV_MAX; ++dev) {
        if (phy_rx_enabled(dev) && ((xsec_cur - xsec_last[dev]) > 100000000ul)) {
            cc_dbg("force-restarting rx");
            xsec_last[dev] = xsec_cur;
            phy_rx_disable(dev);
            phy_rx_enable(dev);
        }
    }
}

#include <board.h>
#include <stdio.h>

static u8 channel_hop_cycle[MAC_NUM_DEVICES] = {0};

static inline void next_chan(cc_dev_t dev)
{
    incr:

    if (!channel_hop_cycle[dev]++) {
        if (!dev) LED_C_TOGGLE();
        else LED_D_TOGGLE();
    } else {
        if (channel_hop_cycle[dev] >= MAC_NUM_CHANNELS) channel_hop_cycle[dev] = 0;
    }

    //return;

    chan_cur[dev] = &channels[dev].chan[channel_hop_cycle[dev]];

#if MAC_NUM_DEVICES > 1
    for (u8 i = MAC_DEV_MIN; i <= MAC_DEV_MAX; ++i) {
        if (i == dev) continue;
        if (phy_rx_enabled(i) && chan_cur[dev]->id == chan_cur[i]->id) goto incr;
    }
#endif

    chan_select(&channels[dev].group, channel_hop_cycle[dev]);
}

//static volatile bool chan_switch_pending = false;

static void mac_phy_signal(cc_dev_t dev, u8 ms1, void *param)
{
    xsec_last[dev] = pit_get_elapsed(xsec_timer);

    switch (ms1) {
        case CC1200_MARC_STATUS1_NO_FAILURE:
        case CC1200_MARC_STATUS1_RX_TIMEOUT:
        case CC1200_MARC_STATUS1_RX_TERMINATION:
        case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
            if (param) next_chan(dev);
        case CC1200_MARC_STATUS1_RX_FINISHED:
            if (param) *(bool *)param = true;
            break;
    }
}

static void mac_phy_rx(cc_dev_t dev, u8 *buf, u8 len)
{
    //next_chan(dev);
    if (mac_cfg.rx) mac_cfg.rx(dev, buf, len);
    /* TODO: Maybe start channel transition before callback?
     * But what if TX response needs to happen on this channel first?
     */
    //next_chan(dev);
}

void mac_tx_begin(void)
{
    const dev_t dev = MAC_DEV_MAX;
    amp_ctrl(dev, AMP_HGM, true);
    amp_ctrl(dev, AMP_PA, true);
    next_chan(dev);
}

void mac_tx_end(void)
{
    const dev_t dev = MAC_DEV_MAX;
    amp_ctrl(dev, AMP_HGM, false);
    amp_ctrl(dev, AMP_PA, false);
}

/**
 * TODO: Collision detection, streaming, automatic TX channel selection
 */
void mac_tx(u8 *buf, u32 len)
{
    // LNA for CCA?
    const dev_t dev = MAC_DEV_MAX;
    phy_tx(dev, false, buf, len);
    //amp_ctrl(MAC_DEVICE_TX, AMP_HGM, /*phy_rx_enabled(MAC_DEVICE_TX)*/);
    //next_chan(dev);
}

void mac_rx_enable(void)
{

    for (cc_dev_t dev = MAC_DEV_MIN; dev <= MAC_DEV_MAX; ++dev) {
        amp_ctrl(dev, AMP_HGM, true);
        amp_ctrl(dev, AMP_LNA, true);
        phy_rx_enable(dev);
        cc_dbg("[%u] rx enable: f=%lu sr=%lu",
               dev, cc_get_freq(dev), cc_get_symbol_rate(dev));
    }

}

void mac_rx_disable(void)
{
    for (cc_dev_t dev = MAC_DEV_MIN; dev <= MAC_DEV_MAX; ++dev) {
        phy_rx_disable(dev);
        amp_ctrl(dev, AMP_LNA, false);
        amp_ctrl(dev, AMP_HGM, false);
    }
}