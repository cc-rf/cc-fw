#pragma once

#include <ccrf/ccrf.h>
#include "rdio/cc1200.h"
#include "sys/spi.h"


#if defined(BOARD_CLOUDCHASER) && BOARD_REVISION == 2
#define CC_XOSC_FREQ            38400000u       /* External oscillator frequency in Hz. */
#else
#define CC_XOSC_FREQ            40000000u       /* External oscillator frequency in Hz. */
#endif

#define CC_MULTIBAND            0               /* Enable multiband support. Otherwise fixed at 820-960MHz. */
#define CC_PA_SUPPORT           1               /* Enable functions for using power amplifier */


typedef enum __packed {
    RDIO_STATUS_MASK            = CC1200_STATUS_STATE_M,
    RDIO_STATUS_IDLE            = CC1200_STATE_IDLE,
    RDIO_STATUS_RX              = CC1200_STATE_RX,
    RDIO_STATUS_TX              = CC1200_STATE_TX,
    RDIO_STATUS_FSTXON          = CC1200_STATE_FSTXON,
    RDIO_STATUS_CALIBRATE       = CC1200_STATE_CALIBRATE,
    RDIO_STATUS_SETTLING        = CC1200_STATE_SETTLING,
    RDIO_STATUS_RXFIFO_ERROR    = CC1200_STATE_RXFIFO_ERROR,
    RDIO_STATUS_TXFIFO_ERROR    = CC1200_STATE_TXFIFO_ERROR

} rdio_status_t;

typedef enum __packed {
    RDIO_STATE_SETTLE,
    RDIO_STATE_TX,
    RDIO_STATE_IDLE,
    RDIO_STATE_RX

} rdio_state_t;

typedef u8 rdio_id_t;
typedef void (* rdio_isr_t)(void *param);

typedef struct rdio {
    rdio_id_t id;
    ccrf_spi_t spi;
    //volatile u8 flag[2];

} *rdio_t;

static inline rdio_id_t rdio_id(rdio_t rdio) { return rdio->id; }

typedef struct __packed {
    u16 addr;
    u8 data;

} rdio_reg_config_t;

typedef struct {
    rdio_id_t id;
    rdio_isr_t isr;
    void *isr_param;
    const rdio_reg_config_t *reg_config;
    size_t reg_config_size;

} rdio_config_t;

typedef u8 rdio_ccac_t;


rdio_t rdio_init(const rdio_config_t *config);

rdio_state_t rdio_state_read(rdio_t rdio) __ccrf_code;
//static inline volatile rdio_state_t rdio_state(rdio_t rdio) { return (rdio_state_t) ((rdio->flag[1] << 1) | rdio->flag[0]); }

rdio_status_t rdio_mode_idle(rdio_t rdio) __ccrf_code;
rdio_status_t rdio_mode_rx(rdio_t rdio) __ccrf_code;
rdio_status_t rdio_mode_tx(rdio_t rdio) __ccrf_code;
rdio_status_t rdio_cca_begin(rdio_t rdio, rdio_ccac_t *ccac) __ccrf_code;
rdio_status_t rdio_cca_end(rdio_t rdio, rdio_ccac_t ccac) __ccrf_code;
rdio_status_t rdio_rssi_read(rdio_t rdio, s16 *rssi) __ccrf_code;
rdio_status_t rdio_cw_set(rdio_t rdio, bool cw) __ccrf_code;

bool rdio_reg_config(rdio_t rdio, const rdio_reg_config_t config[], size_t size);

rdio_status_t rdio_reg_read(rdio_t rdio, u16 addr, u8 *data, u8 size) __ccrf_code;
rdio_status_t rdio_reg_write(rdio_t rdio, u16 addr, u8 *data, u8 size) __ccrf_code;

rdio_status_t rdio_strobe(rdio_t rdio, u8 strobe) __ccrf_code;
static inline rdio_status_t rdio_strobe_noop(rdio_t rdio) { return rdio_strobe(rdio, CC1200_SNOP); }
static inline rdio_status_t rdio_strobe_idle(rdio_t rdio) { return rdio_strobe(rdio, CC1200_SIDLE); }
static inline rdio_status_t rdio_strobe_rxfl(rdio_t rdio) { return rdio_strobe(rdio, CC1200_SFRX); }
static inline rdio_status_t rdio_strobe_txfl(rdio_t rdio) { return rdio_strobe(rdio, CC1200_SFTX); }
static inline rdio_status_t rdio_strobe_cal(rdio_t rdio) { return rdio_strobe(rdio, CC1200_SCAL); }
static inline rdio_status_t rdio_strobe_rx(rdio_t rdio) { return rdio_strobe(rdio, CC1200_SRX); }
static inline rdio_status_t rdio_strobe_tx(rdio_t rdio) { return rdio_strobe(rdio, CC1200_STX); }


static inline rdio_status_t rdio_fifo_read(rdio_t rdio, u8 *data, u8 size) { return rdio_reg_read(rdio, CC1200_FIFO_ACCESS, data, size); }
static inline rdio_status_t rdio_fifo_write(rdio_t rdio, u8 *data, u8 size) { return rdio_reg_write(rdio, CC1200_FIFO_ACCESS, data, size); }

static inline u8 rdio_reg_get(rdio_t rdio, u16 addr, rdio_status_t *status)
{
    u8 reg;
    const rdio_status_t st = rdio_reg_read(rdio, addr, &reg, sizeof(reg));
    if (status) *status = st;
    return reg;
}


static inline rdio_status_t rdio_reg_set(rdio_t rdio, u16 addr, u8 value)
{
    return rdio_reg_write(rdio, addr, &value, sizeof(value));
}


static inline u16 rdio_reg_get16(rdio_t rdio, u16 addr, rdio_status_t *status)
{
    u16 reg;
    const rdio_status_t st = rdio_reg_read(rdio, addr, (u8 *) &reg, sizeof(reg));
    if (status) *status = st;
    return reg;
}


static inline rdio_status_t rdio_reg_set16(rdio_t rdio, u16 addr, u16 value)
{
    return rdio_reg_write(rdio, addr, (u8 *) &value, sizeof(value));
}


static inline rdio_status_t rdio_reg_update(rdio_t rdio, u16 addr, u8 mask, u8 value, u8 *prev)
{
    rdio_status_t status;

    const u8 current = rdio_reg_get(rdio, addr, &status);

    value = (current & ~mask) | (value & mask);

    if (value != current) {
        status = rdio_reg_set(rdio, addr, value);
    }

    if (prev) *prev = current;

    return status;
}


static inline rdio_status_t rdio_reg_update16(rdio_t rdio, u16 addr, u16 mask, u16 value, u16 *prev)
{
    rdio_status_t status;

    const u16 current = rdio_reg_get16(rdio, addr, &status);

    value = (current & ~mask) | (value & mask);

    if (value != current) {
        status = rdio_reg_set16(rdio, addr, value);
    }

    if (prev) *prev = current;

    return status;
}
