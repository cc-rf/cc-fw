#include "rdio/rdio.h"
#include "rdio/config.h"
#include "sys/spi.h"
#include "sys/clock.h"
#include "sys/isr.h"
#include "sys/trace.h"
#include "rdio.h"

#include <stdlib.h>
#include <assert.h>


#define rdio_trace_info     ccrf_trace_info
#define rdio_trace_warn     ccrf_trace_warn
#define rdio_trace_error    ccrf_trace_error
#define rdio_trace_debug    ccrf_trace_debug
#define rdio_trace_verbose  ccrf_trace_verbose


#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define swap16(v) __builtin_bswap16(v)
#else
#define swap16(v) (v)
#endif


rdio_t rdio_init(const rdio_config_t *config)
{
    rdio_t rdio = malloc(sizeof(struct radio)); assert(rdio);

    rdio->id = config->id;

    if (!ccrf_clock_init()) goto _fail;
    if (!ccrf_spi_init(rdio)) goto _fail;

    rdio_strobe(rdio, CC1200_SRES);

    int i = 0;

    while ((i++ < 1000) && (rdio_strobe_noop(rdio) & CC1200_STATUS_CHIP_RDYn));

    const u8 pn = rdio_reg_get(rdio, CC1200_PARTNUMBER, NULL);

    if (pn != 0x20) {
        rdio_trace_error("invalid part number");
        goto _fail;
    }

    if (!rdio_reg_config(rdio, RDIO_REG_CONFIG_DEFAULT, COUNT_OF(RDIO_REG_CONFIG_DEFAULT))) {
        rdio_trace_error("unable to apply config");
        goto _fail;
    }

    if (config->reg_config && config->reg_config_size) {
        if (!rdio_reg_config(rdio, config->reg_config, config->reg_config_size)) {
            rdio_trace_error("unable to apply user config");
            goto _fail;
        }
    }

    if (config->isr) {
        rdio_reg_set(rdio, CC1200_IOCFG3, CC1200_IOCFG_GPIO_CFG_MCU_WAKEUP);
        ccrf_isr_configure(rdio, CCRF_ISR_SRC_GPIO3, CCRF_ISR_EDGE_RISING, config->isr, config->isr_param);
    }

    goto _done;
    _fail:

    if (rdio) {
        free(rdio);
        rdio = NULL;
    }

    _done:
    return rdio;
}


rdio_state_t rdio_state(rdio_t rdio)
{
    const u8 ms = rdio_reg_get(rdio, CC1200_MARCSTATE, NULL) & CC1200_MARC_2PIN_STATE_M;
    switch (ms) {
        default:
        case CC1200_MARC_2PIN_STATE_SETTLING:
            return RDIO_STATE_SETTLE;
        case CC1200_MARC_2PIN_STATE_TX:
            return RDIO_STATE_TX;
        case CC1200_MARC_2PIN_STATE_IDLE:
            return RDIO_STATE_IDLE;
        case CC1200_MARC_2PIN_STATE_RX:
            return RDIO_STATE_RX;
    }
}


rdio_status_t rdio_mode_idle(rdio_t rdio)
{
    rdio_status_t st = rdio_strobe_noop(rdio);

    if (!(st & RDIO_STATUS_IDLE)) {
        st = rdio_strobe_idle(rdio);
    }

    return st;
}


rdio_status_t rdio_mode_rx(rdio_t rdio)
{
    rdio_status_t st;

    do {
        st = rdio_strobe_noop(rdio) & RDIO_STATUS_MASK;

    } while (st == RDIO_STATUS_SETTLING || st == RDIO_STATUS_CALIBRATE);

    switch (st) {
        case RDIO_STATUS_RX:
            return st;

        case RDIO_STATUS_TX:
            break;

        case RDIO_STATUS_FSTXON:
            break;

        case RDIO_STATUS_IDLE:
        default:
            break;

        case RDIO_STATUS_RXFIFO_ERROR:
            //rdio_strobe_idle(rdio);
            rdio_strobe_rxfl(rdio);
            break;

        case RDIO_STATUS_TXFIFO_ERROR:
            //rdio_strobe_idle(rdio);
            rdio_strobe_txfl(rdio);
            break;
    }

    return rdio_strobe_rx(rdio);
}

rdio_status_t rdio_cca_run(rdio_t rdio, bool check)
{
    rdio_status_t st;
    u8 reg;

    if (check) st = rdio_mode_rx(rdio);

    do {
        reg = rdio_reg_get(rdio, CC1200_RSSI0, &st) & (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID);

    } while (reg != (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID));

    return st;
}


bool rdio_reg_config(rdio_t rdio, const rdio_reg_config_t config[], size_t size)
{
    for (size_t i = 0; i < size; i++) {
        rdio_reg_set(rdio, config[i].addr, config[i].data);

        if (config[i].addr == CC1200_RNDGEN) continue;

        u8 data = rdio_reg_get(rdio, config[i].addr, NULL);

        if (config[i].addr == CC1200_SERIAL_STATUS) {
            // Mask out clock signals etc.
            data &= config[i].data;
        }

        if (data != config[i].data) {
            rdio_trace_error("[%u] validation error: addr=0x%04X data=0x%02X data[read-back]=0x%02X", rdio->id, config[i].addr, config[i].data, data);
            return false;
        }
    }

    return true;
}


rdio_status_t rdio_reg_read(rdio_t rdio, u16 addr, u8 *data, u8 size)
{
    return ccrf_spi_io(rdio, CC1200_ACCESS_READ | CC1200_ACCESS_BURST, addr, NULL, data, size);
}


rdio_status_t rdio_reg_write(rdio_t rdio, u16 addr, u8 *data, u8 size)
{
    return ccrf_spi_io(rdio, CC1200_ACCESS_WRITE | CC1200_ACCESS_BURST, addr, data, NULL, size);
}


rdio_status_t rdio_strobe(rdio_t rdio, u8 strobe)
{
    return ccrf_spi_io(rdio, 0, strobe, NULL, NULL, 0);
}


u8 rdio_reg_get(rdio_t rdio, u16 addr, rdio_status_t *status)
{
    const rdio_status_t st = rdio_reg_read(rdio, addr, (u8 *) &addr, sizeof(u8));
    if (status) *status = st;
    return (u8) addr;
}


rdio_status_t rdio_reg_set(rdio_t rdio, u16 addr, u8 value)
{
    return rdio_reg_write(rdio, addr, &value, sizeof(value));
}


u16 rdio_reg_get16(rdio_t rdio, u16 addr, rdio_status_t *status)
{
    const rdio_status_t st = rdio_reg_read(rdio, addr, (u8 *) &addr, sizeof(u16));
    if (status) *status = st;
    return addr;
}


rdio_status_t rdio_reg_set16(rdio_t rdio, u16 addr, u16 value)
{
    return rdio_reg_write(rdio, addr, (u8 *) &value, sizeof(value));
}


rdio_status_t rdio_reg_update(rdio_t rdio, u16 addr, u8 mask, u8 value)
{
    rdio_status_t status;

    const u8 current = rdio_reg_get(rdio, addr, &status);

    value = (current & ~mask) | (value & mask);

    if (value != current) {
        status = rdio_reg_set(rdio, addr, value);
    }

    return status;
}


rdio_status_t rdio_reg_update16(rdio_t rdio, u16 addr, u16 mask, u16 value)
{
    rdio_status_t status;

    const u16 current = rdio_reg_get16(rdio, addr, &status);

    value = (current & ~mask) | (value & mask);

    if (value != current) {
        status = rdio_reg_set16(rdio, addr, value);
    }

    return status;
}

