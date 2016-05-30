#include <cc/common.h>
#include <cc/mac.h>
#include <cc/cfg.h>
#include <cc/phy.h>
#include <cc/amp.h>
#include <cc/chan.h>
#include <assert.h>
#include <FreeRTOS.h>
#include <semphr.h>

#define MAC_NUM_DEVICES     CC_NUM_DEVICES
#define MAC_DEV_MIN         CC_DEV_MIN
#define MAC_DEV_MAX         CC_DEV_MAX
#define MAC_NUM_CHANNELS    50

static bool mac_setup(cc_dev_t dev);
static void mac_lock(cc_dev_t dev);
static void mac_unlock(cc_dev_t dev);
static void mac_phy_signal(cc_dev_t dev, u8 ms1, void *param);
static void mac_phy_rx(cc_dev_t dev, u8 *buf, u8 len);

static const struct cc_cfg_reg CC_CFG_MAC[] = {
        {CC1200_SETTLING_CFG, 0x3}, // Defaults except never auto calibrate
};

static mac_cfg_t mac_cfg;

static phy_cfg_t phy_cfg = {
        .rx = mac_phy_rx,
        .signal = mac_phy_signal
};

static struct {
    volatile chan_t chan_cur;
    volatile u32 chan_cur_pkt_cnt;
    u8 channel_hop_cycle;
    u32 rx_timeout;
    volatile bool tx_on;
    volatile bool tx_restart_rx;
    xSemaphoreHandle mtx;
    u8 rx_channel;

} mac[MAC_NUM_DEVICES];

static struct {
    chan_grp_t group;
    chan_inf_t chan[MAC_NUM_CHANNELS];
    chan_t hop_table[MAC_NUM_CHANNELS];

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
    mac[dev].mtx = xSemaphoreCreateRecursiveMutex();

    if (!mac[dev].mtx) return false;

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

    mac[dev].rx_timeout = 5000000;
    cc_set_rx_timeout(dev, mac[dev].rx_timeout);

    mac[dev].rx_channel = 0xFF;
    chan_grp_init(&channels[dev].group, channels[dev].hop_table);
    chan_grp_calibrate(&channels[dev].group);
    mac[dev].chan_cur = channels[dev].hop_table[0];
    chan_select(&channels[dev].group, mac[dev].chan_cur);
    mac[dev].chan_cur_pkt_cnt = 0;
    mac[dev].channel_hop_cycle = 0;
    return true;
}

static void mac_lock(cc_dev_t dev)
{
    xSemaphoreTakeRecursive(mac[dev].mtx, portMAX_DELAY);
}

static void mac_unlock(cc_dev_t dev)
{
    xSemaphoreGiveRecursive(mac[dev].mtx);
}


#include <board.h>
#include <stdio.h>


static inline void next_chan(cc_dev_t dev)
{
    if (mac[dev].rx_channel != 0xFF) {
        if (mac[dev].chan_cur != mac[dev].rx_channel) {
            mac[dev].chan_cur = mac[dev].rx_channel;
            chan_select(&channels[dev].group, mac[dev].chan_cur);
        }

        return;
    }

    incr:

    if (!mac[dev].channel_hop_cycle++) {
        if (!dev) LED_C_TOGGLE();
        else LED_D_TOGGLE();
    } else {
        if (mac[dev].channel_hop_cycle >= MAC_NUM_CHANNELS) mac[dev].channel_hop_cycle = 0;
    }

    //return;

    mac[dev].chan_cur = channels[dev].hop_table[mac[dev].channel_hop_cycle];

#if MAC_NUM_DEVICES > 1
    for (u8 i = MAC_DEV_MIN; i <= MAC_DEV_MAX; ++i) {
        if (i == dev) continue;
        if ((phy_rx_enabled(i) || mac[i].tx_on) && mac[dev].chan_cur == mac[i].chan_cur) goto incr;
    }
#endif

    chan_select(&channels[dev].group, mac[dev].chan_cur);
}

//static volatile bool chan_switch_pending = false;

static void mac_phy_signal(cc_dev_t dev, u8 ms1, void *param)
{
    mac_lock(dev);

    switch (ms1) {
        case CC1200_MARC_STATUS1_NO_FAILURE:
        case CC1200_MARC_STATUS1_RX_TIMEOUT:
        case CC1200_MARC_STATUS1_RX_TERMINATION:
        case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
            if (!mac[dev].tx_on) {
                if (mac[dev].chan_cur_pkt_cnt) {
                    mac[dev].chan_cur_pkt_cnt = 0;

                    if (mac[dev].rx_channel == 0xFF) {
                        cc_set_rx_timeout(dev, mac[dev].rx_timeout);
                    }
                }

                if (param) {
                    next_chan(dev);
                    *(bool *) param = true;
                }
            } else {
                if (param) *(bool *) param = false;
            }
            break;

        case CC1200_MARC_STATUS1_RX_FINISHED:
            if (!mac[dev].chan_cur_pkt_cnt++) {
                if (mac[dev].rx_channel == 0xFF) {
                    bool chan_changed = false;

                    for (u8 i = MAC_DEV_MIN; i <= MAC_DEV_MAX; ++i) {
                        if (i == dev) continue;
                        if ((phy_rx_enabled(i) || mac[i].tx_on) && mac[dev].chan_cur == mac[i].chan_cur) {
                            next_chan(dev);
                            chan_changed = true;
                            break;
                        }
                    }

                    if (chan_changed) {
                        mac[dev].chan_cur_pkt_cnt = 0;
                    } else {
                        cc_set_rx_timeout(dev, mac[dev].rx_timeout * 100);
                    }
                }
            }

            if (mac[dev].rx_channel != 0xFF) next_chan(dev);

            if (param) *(bool *) param = true;
            break;

        default:
            if (param) *(bool *) param = !mac[dev].tx_on;
    }

    mac_unlock(dev);
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

void mac_tx_begin(chan_t chan)
{
    const dev_t dev = MAC_DEV_MAX;

    mac_lock(dev);
    mac[dev].tx_on = true;

    if ((mac[dev].tx_restart_rx = phy_rx_enabled(dev))) {
        //phy_rx_disable(dev);
        mac_rx_disable();
    }

    amp_ctrl(dev, AMP_HGM, true);
    amp_ctrl(dev, AMP_PA, true);

    cc_strobe(dev, CC1200_SIDLE);
    mac[dev].chan_cur = chan;
    chan_select(&channels[dev].group, chan); // NOTE: not in IDLE!
    mac_unlock(dev);
}

void mac_tx_end(void)
{
    const dev_t dev = MAC_DEV_MAX;

    mac_lock(dev);
    amp_ctrl(dev, AMP_PA, false);

    mac[dev].tx_on = false;

    /*if (phy_rx_enabled(dev)) {
        amp_ctrl(dev, AMP_LNA, true);
    } else {
        amp_ctrl(dev, AMP_HGM, false);
        amp_ctrl(dev, AMP_LNA, false);
    }*/

    if (mac[dev].tx_restart_rx) {
        mac_rx_enable();
    }

    /*if (mac[dev].tx_restart_rx) {
        mac[dev].tx_restart_rx = false;
        amp_ctrl(dev, AMP_LNA, true);
        phy_rx_enable(dev);
    } else {
        amp_ctrl(dev, AMP_HGM, false);
        amp_ctrl(dev, AMP_LNA, false);
    }*/

    mac_unlock(dev);
}

/**
 * TODO: Collision detection, streaming, automatic TX channel selection
 */
void mac_tx(bool cca, u8 *buf, u32 len)
{
    const dev_t dev = MAC_DEV_MAX;

    if (cca) {
        amp_ctrl(dev, AMP_LNA, true);
    }

    phy_tx(dev, cca, buf, len);

    //amp_ctrl(MAC_DEVICE_TX, AMP_HGM, /*phy_rx_enabled(MAC_DEVICE_TX)*/);
    //next_chan(dev);
}

void mac_set_rx_channel(cc_dev_t dev, u8 channel)
{
    if (channel != mac[dev].rx_channel) {
        mac[dev].rx_channel = channel;

        if (channel == 0xFF) {
            cc_set_rx_timeout(dev, mac[dev].rx_timeout);
        } else {
            cc_set_rx_timeout(dev, mac[dev].rx_timeout*100);
        }
    }
}

void mac_rx_enable(void)
{
    for (cc_dev_t dev = MAC_DEV_MIN; dev <= MAC_DEV_MAX; ++dev) {
        mac_lock(dev);
        amp_ctrl(dev, AMP_HGM, true);
        amp_ctrl(dev, AMP_LNA, true);
        phy_rx_enable(dev);
        mac_unlock(dev);
        //cc_dbg("[%u] rx enable: f=%lu sr=%lu",
        //       dev, cc_get_freq(dev), cc_get_symbol_rate(dev));
    }

}

void mac_rx_disable(void)
{
    for (cc_dev_t dev = MAC_DEV_MIN; dev <= MAC_DEV_MAX; ++dev) {
        mac_lock(dev);
        phy_rx_disable(dev);
        amp_ctrl(dev, AMP_LNA, false);
        amp_ctrl(dev, AMP_HGM, false);
        mac_unlock(dev);
    }
}