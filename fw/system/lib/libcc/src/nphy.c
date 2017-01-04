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

static void nphy_rx(void);

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
        {CC1200_FIFO_CFG,   0 /*!CC1200_FIFO_CFG_CRC_AUTOFLUSH*/},


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

#define FREQ_BASE   905000000
#define FREQ_BW     800000
#define CHAN_COUNT  25

static const u32 freq_base      = FREQ_BASE;
static const u32 freq_side_bw   = FREQ_BW / 2;
static const u32 chan_count     = CHAN_COUNT;
static const u32 chan_time      = 200;//30;

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

        const u8 st = cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M;

        // TODO: !!!! MAKE SURE THIS ISN'T STUPID !!!!
        if (st != CC1200_STATE_IDLE)
            cc_strobe(dev, CC1200_SIDLE);

        //NOTE: for debug purposes, only use 2 channels
        chan_select(&chnl.group, (chan_t) (10 + chan%2));
        //chan_select(&chnl.group, (chan_t)chan);
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

    if (!xTaskCreate(nphy_task, "nphy:main", TASK_STACK_SIZE_DEFAULT, NULL, TASK_PRIO_HIGH, &nphy.task)) {
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

static void nphy_rx(void)
{
    u8 buf[256];
    cc_pkt_t *const spkt = (cc_pkt_t *)buf;

    spkt->len = 0;

    /* Assumption (may change later): variable packet length, 2 status bytes -> 3 total. */
    const static u8 PKT_OVERHEAD = 3;

    u8 fifo_len = cc_get(dev, CC1200_NUM_RXBYTES);

    if (fifo_len > PKT_OVERHEAD) {
        //u8 rxf = cc_get(dev, CC1200_RXFIRST);
        //u8 rxl = cc_get(dev, CC1200_RXLAST);
        //cc_dbg("[%u] rxf=%u rxl=%u n=%u", dev, rxf, rxl, len);

        // (another assumption: only one packet in FIFO. TBD whether it ever might be more)
        cc_fifo_read(dev, (u8 *)spkt, fifo_len);

        const s8 rssi = (s8)spkt->data[spkt->len];
        const u8 crc_ok = spkt->data[spkt->len + 1] & (u8) CC1200_LQI_CRC_OK_BM;
        const u8 lqi = spkt->data[spkt->len + 1] & (u8) CC1200_LQI_EST_BM;

        if (!crc_ok) spkt->len = 0;
        else {
            // assumption: length field in fifo is sane
            cc_pkt_t *pkt = malloc(sizeof(*spkt) + spkt->len); assert(pkt);
            memcpy(pkt, spkt, sizeof(*spkt) + spkt->len);
            if (!xQueueSend(nphy.rxq, &pkt, 0)) {
                cc_dbg("rx pkt queue fail");
            } // TODO: probably need to not wait here
        }

    } else if (fifo_len) {
        cc_strobe(dev, CC1200_SFRX);
    }
}

static void ensure_rx(void)
{
    const u8 ms = cc_get(dev, CC1200_MARCSTATE) & CC1200_MARC_STATE_M;

    switch (ms) {
        case CC1200_MARC_STATE_RX:
        case CC1200_MARC_STATE_TXRX_SWITCH:
        case CC1200_MARC_STATE_IFADCON_TXRX:
            break;

        case CC1200_MARC_STATE_RX_FIFO_ERR:
            cc_strobe(dev, CC1200_SFRX);
            // fall through
        default:
            cc_strobe(dev, CC1200_SRX);
            break;
    }
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
    u8 reg = cc_get(dev, CC1200_MARCSTATE) & CC1200_MARC_STATE_M;

    if (reg != CC1200_MARC_STATE_RX) {
        cc_strobe(dev, CC1200_SRX);
        u8 msnew = cc_get(dev, CC1200_MARCSTATE) & CC1200_MARC_STATE_M;
        cc_dbg("not rx. MS=0x%02X -> 0x%02X", reg, msnew);
    }

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


typedef struct __packed {
    u8 len;
    u8 chn;
    u8 seq;
    u8 data[];

} app_pkt_t;


static void nphy_task(void *param)
{
    cc_pkt_t *pkt = NULL;
    u8 ms, st;

    u32 ticks = 0;
    u32 remaining = chan_time;
    u32 tx_time = 0;
    u32 chan_ticks;
    u32 notify;

    while (1) {

        // always do ...

        if (xTaskNotifyWait(0, UINT32_MAX, &notify, pdMS_TO_TICKS(remaining))) {

            if (notify & NOTIFY_MASK_ISR) {
                // handle isr
                ms = cc_get(dev, CC1200_MARC_STATUS1);

                cc_dbg_v("ms1=0x%02X t=%lu", ms, sync_timestamp());

                if (pkt) {
                    switch (ms) {
                        case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
                            RNGA_GetRandomData(RNG, &st, 1);
                            st = (u8)(3 + (st % 8));
                            vTaskDelay(pdMS_TO_TICKS(st));

                        _re_cca:
                            if ((sync_timestamp() - tx_time) > (chan_time/4)) {
                                cc_dbg("tx: abandanoning packet! (0)");
                                free(pkt);
                                pkt = NULL;
                                //goto _txq_check;
                                break; // ^ basically the same
                            }

                            cc_dbg("tx: cca retry ~ ms=0x%02X st=0x%02X rssi=%i", ms, cc_get(dev, CC1200_SNOP), cc_get_rssi(0));
                            // TODO: update_channel
                            if (sync_time) {
                                _ts2: ticks = sync_timestamp() - sync_time;
                                chan_ticks = (ticks % chan_time);
                                remaining = chan_time - /*(ticks % chan_time)*/chan_ticks;

                                /*if (chan_ticks < 10) {
                                    vTaskDelay(pdMS_TO_TICKS(10-chan_ticks));
                                    goto _ts;
                                }*/

                                if (remaining <= 5) {
                                    vTaskDelay(pdMS_TO_TICKS(10+remaining));
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
                            if (!sync_time) sync_time = sync_timestamp();
                            nphy_rx();
                            cc_strobe(dev, CC1200_SIDLE); // NOTE: could be unnecessary/impactful
                            //break;
                            goto _re_cca;

                        case CC1200_MARC_STATUS1_NO_FAILURE:
                            // although other things could be happening, we are busy waiting for the result of a tx.
                            // it was assumed before (hoprefully) that there would be channel time to do so, so that means
                            // if the packet is sent close to the end of a slot, we'll be late to the next one. such is life.
                            continue;

                        default:
                            cc_dbg("tx: weird outcome ms=0x%02X st=0x%02X", ms, cc_get(dev, CC1200_SNOP));
                            cc_strobe(dev, CC1200_SIDLE);
                            cc_strobe(dev, CC1200_SFRX);
                            cc_strobe(dev, CC1200_SFTX);
                            // fall through
                        case CC1200_MARC_STATUS1_TX_FINISHED:
                            free(pkt);
                            pkt = NULL;
                            // don't srx or update channel or check for tx, let it happen below.
                            break;
                    }
                } else if (ms) {
                    switch (ms) {
                        case CC1200_MARC_STATUS1_RX_FINISHED:
                            if (!sync_time) sync_time = sync_timestamp();
                            tx_time = 0; // next tx not gated by last tx?
                            nphy_rx();
                            break;

                        case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
                        case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:
                            cc_strobe(dev, CC1200_SFRX);
                            break;

                        default:
                            cc_dbg("task: weird outcome ms=0x%02X st=0x%02X", ms, cc_get(dev, CC1200_SNOP));
                            // TODO: maybe flush fifos and/or check etc...
                            cc_strobe(dev, CC1200_SIDLE);
                            cc_strobe(dev, CC1200_SFRX);
                            cc_strobe(dev, CC1200_SFTX);
                            break;
                    }
                }
            }

        } else {
            // timeout
        }

        _txq_check:;

        bool new_pkt = false;
        bool cca = false;

        if (!pkt) {

            // maybe only gate tx packets when haven't just received one, e.g. clear tx_time on rx
            if ((sync_timestamp() - tx_time) > 10 && xQueueReceive(nphy.txq, &pkt, 0) && pkt) {
                tx_time = sync_timestamp();
                new_pkt = true;
                cca = true;//(pkt->len & 0x80) != 0;

                if (!cca) {
                    //cc_strobe(dev, CC1200_SIDLE);
                } else {
                    // ^ fucked with the condition above....
                    //pkt->len &= 0x7f;
                }

                pkt->len &= 0x7f;
            }

        } else if (pkt && ((sync_timestamp() - tx_time) > (chan_time/4))) {
            cc_dbg("tx: abandanoning packet! (1:also:this is a weird case no?)");
            free(pkt);
            pkt = NULL;
            goto _txq_check;
        }

        if (sync_time && ((/*packet was not already/still pending tx*/pkt && new_pkt) ||/*no packet tx*/ !pkt)) {
            _ts: ticks = sync_timestamp() - sync_time;
            chan_ticks = (ticks % chan_time);
            remaining = chan_time - /*(ticks % chan_time)*/chan_ticks;

            if (pkt) {
                if (chan_ticks < 5) {
                    vTaskDelay(pdMS_TO_TICKS(5 - chan_ticks));
                    goto _ts;
                }

                if (remaining <= 5) {
                    vTaskDelay(pdMS_TO_TICKS(5 + remaining));

                    goto _ts;
                }
            }

            chan_set((ticks / chan_time) % chan_count);
        }

        if (new_pkt) {
            // DEBUG
            ((app_pkt_t *)pkt)->chn = (u8)chan_cur;
            // NOTE: Used to be above after queue receive
            cc_fifo_write(dev, (u8 *) pkt, sizeof(*pkt) + pkt->len);

            if (cca) {
                cca_run();
            }

            cc_strobe(dev, CC1200_STX);
            if (!sync_time) sync_time = sync_timestamp(); // TODO: Maybe move to when the isr indicates success?

            // DEBUG
            printf("tx/%u: seq=%u len=%u t=%lu\r\n", (u8)chan_cur, ((app_pkt_t *)pkt)->seq, ((app_pkt_t *)pkt)->len, sync_timestamp());

        } else if (!pkt) {
            // TODO: Maybe find a better condition for this. Maybe really important
            ensure_rx();
        }
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
