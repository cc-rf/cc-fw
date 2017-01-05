#include <cc/nphy.h>
#include <cc/io.h>
#include <cc/cfg.h>

#include <cc/spi.h>
#include <alloca.h>
#include <string.h>
#include <assert.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <cc/sys/kinetis/pit.h>
#include <timers.h>
#include <malloc.h>
#include <cc/freq.h>


#include <cc/sys/kinetis/isrd.h>
#include <fsl_port.h>
#include <cc/type.h>


static void isr_mcu_wake(void);
static void cca_setup(void);
static void cca_run(void);
static void cca_end(void);

static void nphy_rx(bool flush);

static void nphy_task(void *param);
static void nphy_dispatch_task(void *param);

#define CC_RSSI_OFFSET      (s8)(-81 - 15 + 11)

static const struct cc_cfg_reg CC_CFG_PHY[] = {
        {CC1200_IOCFG3, CC1200_IOCFG_GPIO_CFG_HW0},
        {CC1200_IOCFG2, CC1200_IOCFG_GPIO_CFG_HW0},
        {CC1200_IOCFG1, CC1200_IOCFG_GPIO_CFG_HIGHZ},
        {CC1200_IOCFG0, CC1200_IOCFG_GPIO_CFG_HW0},

        /* NOTE: Enabling CC1200_RFEND_CFG1_RX_TIME_QUAL_M when using a RX timeout means that
         * there will be times when the radio stays in RX due to a valid CS or PQT but will hang there
         * regardless of whether a packet was received. It seems to linger on the channel potentially
         * forever despite the lack of a sync word detection.
         * For the timeout to always trigger an interrupt, RX_TIME_QUAL must be zero.
         * */
        {CC1200_RFEND_CFG1, CC1200_RFEND_CFG1_RXOFF_MODE_IDLE | (0x7<<1) /*| CC1200_RFEND_CFG1_RX_TIME_QUAL_M*/},
        {CC1200_RFEND_CFG0, CC1200_RFEND_CFG0_TXOFF_MODE_RX | CC1200_RFEND_CFG0_TERM_ON_BAD_PACKET_EN /*| 1*/},

        // These may not necessarily be part of the phy handling

        //{CC1200_PREAMBLE_CFG1, /*0xD*/0x4 << 2},
        //{CC1200_SYNC_CFG1,  0x09 | CC1200_SYNC_CFG1_SYNC_MODE_16/* changed from 11 */},
        {CC1200_PKT_CFG2,   /*CC1200_PKT_CFG2_CCA_MODE_ALWAYS*/CC1200_PKT_CFG2_CCA_MODE_RSSI_THR_NOT_RX/*CC1200_PKT_CFG2_CCA_MODE_RSSI_THR_ETSI_LBT*/},
        {CC1200_PKT_CFG1,   CC1200_PKT_CFG1_CRC_CFG_ON_INIT_1D0F | CC1200_PKT_CFG1_ADDR_CHECK_CFG_OFF | CC1200_PKT_CFG1_APPEND_STATUS | CC1200_PKT_CFG1_WHITE_DATA},
        {CC1200_PKT_CFG0,   CC1200_PKT_CFG0_LENGTH_CONFIG_VARIABLE},
        {CC1200_PKT_LEN,    255},
        //{CC1200_SERIAL_STATUS, CC1200_SERIAL_STATUS_IOC_SYNC_PINS_EN}, // Enable access to GPIO state in CC1200_GPIO_STATUS.GPIO_STATE

        {CC1200_RNDGEN,     CC1200_RNDGEN_EN}, // Needed for random backoff for LBT CCA: https://e2e.ti.com/support/wireless_connectivity/f/156/t/370230
        {CC1200_FIFO_CFG,   CC1200_FIFO_CFG_CRC_AUTOFLUSH /*!CC1200_FIFO_CFG_CRC_AUTOFLUSH*/},


        {CC1200_AGC_GAIN_ADJUST, (u8)CC_RSSI_OFFSET},
        {CC1200_SETTLING_CFG, 0x3}, // Defaults except never auto calibrate

};


static const cc_dev_t dev = 0;
static xTaskHandle waiting_task = NULL;

static struct {
    xTaskHandle task;
    xTaskHandle disp;
    xQueueHandle txq;
    xQueueHandle rxq;
    nphy_rx_t rx;

} nphy = {NULL};




extern u32 sync_timestamp(void);
u32 sync_time = 0;

static void ensure_rx(void);


#define FREQ_BASE       905000000
#define FREQ_BW         800000
#define CHAN_COUNT      25
#define CHAN_TIME       60//400/*100*///200  //30
#define MAX_CCA_RETRY   3
#define MAX_CCA_TIME    9//(chan_time/4)
#define MAX_PACKET_LEN  120

static const u32 freq_base      = FREQ_BASE;
static const u32 freq_side_bw   = FREQ_BW / 2;
static const u32 chan_count     = CHAN_COUNT;
static const u32 chan_time      = CHAN_TIME;

#include <cc/chan.h>
#include <fsl_rnga.h>


static struct {
    chan_grp_t group;
    chan_inf_t chan[CHAN_COUNT];
    chan_t hop_table[CHAN_COUNT];

} chnl = {
        .group = {
                .dev = 0,
                .freq = {
                        .base = FREQ_BASE,
                        .bw   = FREQ_BW
                },
                .size = CHAN_COUNT
        }
};

volatile u32 chan_cur = UINT32_MAX;

static inline void chan_set(const u32 chan)
{
    if (chan != chan_cur) {
        chan_cur = chan;

        u8 st;

        do {
            st = cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_STATE_M;

        } while (st != CC1200_STATE_IDLE);

        // this is probably useful however, we can't do anything with partial packets
        cc_strobe(dev, CC1200_SFRX);

        //NOTE: for debug purposes, only use 2 channels
        chan_select(&chnl.group, (chan_t) (10 + chan%2));
        //chan_select(&chnl.group, (chan_t)chan);

        ensure_rx(); // this kills the in-progress tx
    }
}




bool nphy_init(nphy_rx_t rx)
{
    // TODO: eventually move elsewhere
    RNGA_Init(RNG);

    cc_spi_init(dev);

    cc_strobe(dev, CC1200_SRES);
    int i = 0;

    while ((i++ < 1000) && (cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_CHIP_RDYn));

    u8 pn = cc_get(dev, CC1200_PARTNUMBER);

    if (pn != 0x20) {
        cc_dbg("[%u] error: part number 0x%02X != 0x20", dev, pn);
        return false;
    }

    if (!cc_cfg_regs(dev, CC_CFG_DEFAULT, COUNT_OF(CC_CFG_DEFAULT))) {
        cc_dbg("[%u] error: could not configure (default)", dev);
        return false;
    }

    if (!cc_cfg_regs(dev, CC_CFG_PHY, COUNT_OF(CC_CFG_PHY))) {
        cc_dbg("[%u] error: could not configure", dev);
        return false;
    }

    nphy.txq = xQueueCreate(8, sizeof(cc_pkt_t *));
    nphy.rxq = xQueueCreate(8, sizeof(cc_pkt_t *));
    nphy.rx = rx;

    if (!xTaskCreate(nphy_task, "nphy:main", TASK_STACK_SIZE_LARGE, NULL, TASK_PRIO_HIGHEST, &nphy.task)) {
        cc_dbg("[%u] error: unable to create main task", dev);
        isrd_configure(2, 10, kPORT_InterruptOrDMADisabled, NULL);
        return false;
    }

    if (!xTaskCreate(nphy_dispatch_task, "nphy:disp", TASK_STACK_SIZE_DEFAULT, NULL, TASK_PRIO_HIGH-1, &nphy.disp)) {
        cc_dbg("[%u] error: unable to create dispatch task", dev);
        isrd_configure(2, 10, kPORT_InterruptOrDMADisabled, NULL);
        return false;
    }

    chan_grp_init(&chnl.group, NULL);
    chan_grp_calibrate(&chnl.group);
    chan_set(0);

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(0), CC1200_IOCFG_GPIO_CFG_MCU_WAKEUP);
    isrd_configure(2, 10, kPORT_InterruptRisingEdge, isr_mcu_wake);

    cc_strobe(dev, CC1200_SRX);
    return true;
}

#define NOTIFY_MASK_ISR    2
#define NOTIFY_MASK_TX     4


void nphy_tx(cc_pkt_t *pkt)
{
    cc_pkt_t *qpkt = malloc(pkt->len + sizeof(*pkt)); assert(qpkt);
    memcpy(qpkt, pkt, pkt->len + sizeof(*pkt));
    xQueueSend(nphy.txq, &qpkt, portMAX_DELAY);
    //xTaskNotifyGive(nphy.task);
    xTaskNotify(nphy.task, NOTIFY_MASK_TX, eSetBits);
}

static void isr_mcu_wake(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //vTaskNotifyGiveFromISR(nphy.task, &xHigherPriorityTaskWoken);
    xTaskNotifyFromISR(nphy.task, NOTIFY_MASK_ISR, eSetBits, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void nphy_rx(bool flush)
{
    /* Assumption (may change later): variable packet length, 2 status bytes -> 3 total. */
    const static u8 PKT_OVERHEAD = 3;

    u8 len = cc_get(dev, CC1200_NUM_RXBYTES);

    if (len < PKT_OVERHEAD) {
        if (flush) cc_strobe(dev, CC1200_SFRX);

        if (len && flush) {
            cc_dbg("len=%u < PKT_OVERHEAD=%u", len, PKT_OVERHEAD);
        }

        return;
    }

    cc_pkt_t *spkt;
    u8 *buf = alloca(len);
    size_t pkt_count = 0;

    cc_fifo_read(dev, buf, len);

    while (len > PKT_OVERHEAD) {
        spkt = (cc_pkt_t *)buf;

        if (spkt->len > MAX_PACKET_LEN) {
            cc_dbg("[%u] c=%u malformed: len[header]=%u > len[max]=%u  (len[fifo]=%u)", dev, pkt_count+1, spkt->len, MAX_PACKET_LEN, len);
            break;
        }

        if (spkt->len > (len - PKT_OVERHEAD)) {
            cc_dbg("[%u] c=%u underflow: len[header]=%u > len[fifo]=%u", dev, pkt_count+1, spkt->len, len);
            break;
        }

        ++pkt_count;

        //u8 rxf = cc_get(dev, CC1200_RXFIRST);
        //u8 rxl = cc_get(dev, CC1200_RXLAST);
        //cc_dbg("[%u] rxf=%u rxl=%u n=%u", dev, rxf, rxl, len);

        if (spkt->len) {
            const s8 rssi = (s8) spkt->data[spkt->len];
            const u8 crc_ok = spkt->data[spkt->len + 1] & (u8) CC1200_LQI_CRC_OK_BM;
            const u8 lqi = spkt->data[spkt->len + 1] & (u8) CC1200_LQI_EST_BM;

            if (crc_ok) {
                cc_pkt_t *pkt = malloc(sizeof(*spkt) + spkt->len);
                assert(pkt);
                memcpy(pkt, spkt, sizeof(*spkt) + spkt->len);
                if (!xQueueSend(nphy.rxq, &pkt, 0)) {
                    cc_dbg("rx pkt queue fail");
                } // TODO: probably need to not wait here
            } else {
                cc_dbg("[%u] c=%u bad crc", dev, pkt_count);
            }
        } else {
            cc_dbg("[%u] c=%u empty", dev, pkt_count);
        }

        len -= spkt->len + PKT_OVERHEAD;
        buf += spkt->len + PKT_OVERHEAD;
    }

    if (flush && len) cc_strobe(dev, CC1200_SFRX);
}

static void ensure_rx(void)
{
    u8 st;

    do {
        st = cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M;

    } while (st == CC1200_STATE_SETTLING || st == CC1200_STATE_CALIBRATE);

    switch (st) {
        case CC1200_STATE_RX:
            return;

        case CC1200_STATE_IDLE:
        case CC1200_STATE_TX:
        case CC1200_STATE_FSTXON:
        default:
            break;

        case CC1200_STATE_RXFIFO_ERROR:
            cc_strobe(dev, CC1200_SIDLE); // seems necessary to prevent ISR error indication?
            cc_strobe(dev, CC1200_SFRX);
            cc_dbg_v("rx fifo error");
            break;

        case CC1200_STATE_TXFIFO_ERROR:
            cc_strobe(dev, CC1200_SIDLE); // seems necessary to prevent ISR error indication?
            cc_strobe(dev, CC1200_SFTX);
            cc_dbg_v("tx fifo error");
            break;
    }

    cc_strobe(dev, CC1200_SRX);

    do {
        st = cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M;

    } while (st != CC1200_STATE_RX);
}

void cca_setup(void)
{

    //cc_update(dev, CC1200_RFEND_CFG1, CC1200_RFEND_CFG1_RXOFF_MODE_M, CC1200_RFEND_CFG1_RXOFF_MODE_TX);
    cc_update(dev, CC1200_PKT_CFG2, CC1200_PKT_CFG2_CCA_MODE_M, CC1200_PKT_CFG2_CCA_MODE_RSSI_THR_ETSI_LBT);
    //cc_update(dev, CC1200_MODCFG_DEV_E, CC1200_MODCFG_DEV_E_MODEM_MODE_M, CC1200_MODCFG_DEV_E_MODEM_MODE_CARRIER_SENSE);
    //cc_update(dev, CC1200_SYNC_CFG1, CC1200_SYNC_CFG1_SYNC_THR_M, 0);
}

void cca_run(void)
{
    u8 reg;

    ensure_rx();

    do {
        reg = cc_get(dev, CC1200_RSSI0) & (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID);
    } while (reg != (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID));
}

void cca_end(void)
{
    cc_update(dev, CC1200_PKT_CFG2, CC1200_PKT_CFG2_CCA_MODE_M, CC1200_PKT_CFG2_CCA_MODE_ALWAYS);
    //cc_update(dev, CC1200_MODCFG_DEV_E, CC1200_MODCFG_DEV_E_MODEM_MODE_M, CC1200_MODCFG_DEV_E_MODEM_MODE_NORMAL);
    //cc_update(dev, CC1200_SYNC_CFG1, CC1200_SYNC_CFG1_SYNC_THR_M, 7); // NOTE: cheating here and using known value
}


static void nphy_task(void *param)
{
    cc_pkt_t *pkt = NULL;
    u8 ms, st, retry = 0;

    u32 ticks = 0;
    u32 remaining = chan_time;
    u32 tx_time = 0;
    u32 chan_ticks;
    u32 notify;

    bool cca = false;

#undef cc_dbg_v
#define cc_dbg_v cc_dbg

    while (1) {
        if (!remaining) remaining = chan_time;

        if (xTaskNotifyWait(0, UINT32_MAX, &notify, pdMS_TO_TICKS(remaining))) {

            if (notify & NOTIFY_MASK_ISR) {
                // handle isr
                ms = cc_get(dev, CC1200_MARC_STATUS1);

                //cc_dbg_v("ms1=0x%02X t=%lu", ms, sync_timestamp());

                if (pkt) {
                    switch (ms) {
                        case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
                            if (!retry--) {
                                cc_dbg_v("tx: cca max retry t=%lu", sync_timestamp());
                                cc_strobe(dev, CC1200_SFTX);
                                goto _end_tx;
                            }

                            /*RNGA_GetRandomData(RNG, &st, 1);
                            st = (u8)(3 + (st % 8));
                            vTaskDelay(pdMS_TO_TICKS(st));*/

                        _re_cca:
                            if ((sync_timestamp() - tx_time) >= MAX_CCA_TIME/*(chan_time/4)*/) {
                                cc_dbg_v("tx: cca timeout t=%lu", sync_timestamp());
                                cc_strobe(dev, CC1200_SFTX);
                                //free(pkt);
                                //pkt = NULL;
                                //goto _txq_check;
                                //break; // ^ basically the same
                                goto _end_tx; // for consistency
                            }

                            // fifo check: new: go idle if a fifo refill is needed
                            if (cc_get(dev, CC1200_NUM_TXBYTES) != (sizeof(*pkt) + pkt->len)) {
                                cc_strobe(dev, CC1200_SIDLE);
                                cc_strobe(dev, CC1200_SFTX);
                                cc_fifo_write(dev, (u8 *)pkt, sizeof(*pkt) + pkt->len);
                                cc_dbg_v("tx: fifo refill");
                            }

                            cc_dbg_v("tx: cca retry t=%lu ch=%u st=0x%02X", sync_timestamp(), chan_cur, cc_strobe(dev,CC1200_SNOP));

                            if (sync_time) {
                                _ts2: ticks = sync_timestamp() - sync_time;
                                chan_ticks = (ticks % chan_time);
                                remaining = chan_time - /*(ticks % chan_time)*/chan_ticks;

                                /*if (chan_ticks < 5) {
                                    vTaskDelay(pdMS_TO_TICKS(5-chan_ticks));
                                    goto _ts2;
                                }*/

                                if (remaining <= 3) {
                                    // have not yet observed this condition
                                    cc_dbg_v("cca channel wait");
                                    vTaskDelay(pdMS_TO_TICKS(1+remaining));
                                    goto _ts2;
                                }

                                chan_set((ticks / chan_time) % chan_count);
                            }

                            cca_run();
                            cc_strobe(dev, CC1200_STX); // TODO: Should this be SFTXON?
                            // TODO: update wait time but NOT channel
                            if (sync_time) {
                                ticks = sync_timestamp() - sync_time;
                                remaining = chan_time - (ticks % chan_time);
                            }

                            continue;

                        case CC1200_MARC_STATUS1_RX_FINISHED:
                            // an awkward situation, sort of -- packet received during cca, which is totally possible in some
                            // cases. might as well handle it as 'normal'... but, big
                            // !TODO: determine whether cca needs continuing at this point or what!
                            if (!sync_time || !chan_cur/*def not entirely right*/) sync_time = sync_timestamp();
                            if (!pkt) tx_time = 0; // only ungate next tx if not currently busy with a tx
                            nphy_rx(true);
                            cc_strobe(dev, CC1200_SIDLE); // NOTE: could be unnecessary/impactful
                            //break;
                            goto _re_cca;

                        case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
                            nphy_rx(false); // try to salvage packet(s)

                            // fall through for forced uncoditional flush

                        case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:
                            cc_strobe(dev, CC1200_SIDLE); // seems important (not sure yet)?
                            cc_strobe(dev, CC1200_SFRX);
                            cc_dbg_v("tx: rx fifo error");
                            // fall through

                        case CC1200_MARC_STATUS1_CRC:
                            // assumption: crc autoflush register set, nothing to do here other than continue tx
                            goto _re_cca;


                        case CC1200_MARC_STATUS1_NO_FAILURE:
                            // although other things could be happening, we are busy waiting for the result of a tx.
                            // it was assumed before (hoprefully) that there would be channel time to do so, so that means
                            // if the packet is sent close to the end of a slot, we'll be late to the next one. such is life.
                            continue;


                        case CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW:
                            // sometimes the fifo underflows during a cca when an rx or rx error happens.
                            // so just refill the fifo here and go again. this might be a degenerate case
                            // now that the NUM_TXBYTES check in the cca handling above is done.
                            cc_strobe(dev, CC1200_SFTX);
                            cc_fifo_write(dev, (u8 *)pkt, sizeof(*pkt) + pkt->len);
                            cc_dbg("tx: tx fifo underflow");
                            goto _re_cca;

                        case CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW:
                            cc_strobe(dev, CC1200_SIDLE); // seems important (more sure: rx fifo fails often shortly after)?
                            cc_strobe(dev, CC1200_SFRX); // ^ same as above reason
                            cc_strobe(dev, CC1200_SFTX);
                            cc_dbg_v("tx: tx fifo overflow");
                            goto _end_tx;

                        default:
                            st = cc_get(dev, CC1200_SNOP);
                            cc_dbg("tx: weird outcome: ms=0x%02X st=0x%02X", ms, st);

                            // fall through

                        case CC1200_MARC_STATUS1_TX_FINISHED:
                            if (!sync_time || !chan_cur) sync_time = sync_timestamp();
                            tx_time = sync_timestamp();

                            // DEBUG: extra info
                            //printf("tx/%u: seq=%u len=%u t=%lu\r\n", (u8)chan_cur, ((app_pkt_t *)pkt)->seq, ((app_pkt_t *)pkt)->len, sync_timestamp());

                        _end_tx:
                            free(pkt);
                            pkt = NULL;
                            // don't srx or update channel or check for tx, let it happen below.
                            break;
                    }
                } else if (ms) {
                    switch (ms) {
                        case CC1200_MARC_STATUS1_RX_FINISHED:
                            if (!sync_time) sync_time = sync_timestamp();
                            if (!pkt) tx_time = 0; // only ungate next tx if not currently busy with a tx
                            nphy_rx(true);
                            cc_strobe(dev, CC1200_SIDLE); // NOTE: could be unnecessary/impactful (copied from RX_FIN above)
                            break;

                        case CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW:
                        case CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW:
                            st = cc_strobe(dev, CC1200_SFTX);
                            cc_dbg("rx: tx fifo error: ms=0x%02X st=0x%02X", ms, st);
                            break;

                        case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
                            nphy_rx(false); // try to salvage packet(s)

                            // fall through for forced uncoditional flush

                        case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:
                            cc_strobe(dev, CC1200_SIDLE); // seems important (not sure yet)? (copied from above)
                            st = cc_strobe(dev, CC1200_SFRX);
                            cc_dbg_v("rx: rx fifo error: ms=0x%02X st=0x%02X", ms, st);
                            break;

                        case CC1200_MARC_STATUS1_CRC:
                            // assumption: crc autoflush register set, nothing to do here other than continue rx
                            break;

                        default:
                            st = cc_get(dev, CC1200_SNOP);
                            cc_dbg("task: weird outcome: ms=0x%02X st=0x%02X", ms, st);
                            break;
                    }
                }
            }

        } else {
            // timeout
        }



        if (sync_time) {
            ticks = sync_timestamp() - sync_time;
            chan_ticks = (ticks % chan_time);
            remaining = chan_time - chan_ticks;

            chan_set((ticks / chan_time) % chan_count);

        } else {
            remaining = chan_time;
        }


        if (!pkt && ((sync_timestamp() - tx_time) >= 5) && xQueuePeek(nphy.txq, &pkt, 0) && pkt/*for the ide...*/) {
            const u32 pkt_time = 1 + cc_get_tx_time(dev, pkt->len);

            if (remaining <= pkt_time) {
                cc_dbg_v("tx: delay: remaining=%lu pkt_time=%lu len=%u", remaining, pkt_time, pkt->len);
                pkt = NULL;
                remaining = 1;
                continue;

            } else {
                xQueueReceive(nphy.txq, &pkt, 0); // no reason in the universe that this should not just always pop the same pointer every time
                retry = MAX_CCA_RETRY;

                cca = true;//(pkt->len & 0x80) != 0;

                if (!cca) {
                    //cc_strobe(dev, CC1200_SIDLE);
                } else {
                    // ^ fucked with the condition above....
                    //pkt->len &= 0x7f;
                }

                pkt->len &= 0x7f;

                cc_fifo_write(dev, (u8 *) pkt, sizeof(*pkt) + pkt->len);

                if (cca) {
                    cca_run();
                }

                cc_strobe(dev, CC1200_STX);
                tx_time = sync_timestamp();
            }
        }

        if (!pkt) ensure_rx();
    }
}

static void nphy_dispatch_task(void *param)
{
    const xQueueHandle rxq = nphy.rxq;
    const nphy_rx_t rx = nphy.rx;
    cc_pkt_t *spkt = alloca(sizeof(cc_pkt_t) + 255);
    cc_pkt_t *pkt;

    while (1) {
        if (xQueueReceive(rxq, &pkt, portMAX_DELAY)) {
            memcpy(spkt, pkt, sizeof(*pkt) + pkt->len);
            free(pkt);
            if (rx) rx(spkt); // TODO: maybe require cb and remove check
        }
    }
}
