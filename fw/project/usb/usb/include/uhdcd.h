#pragma once
#include <usr/type.h>

typedef enum __packed {
    UHDCD_ERROR_NONE,
    UHDCD_ERROR_FAIL = 1 << 0,
    UHDCD_ERROR_TIME = 1 << 1

} uhdcd_error_t;

typedef enum __packed {
    UHDCD_STATE_NONE,
    UHDCD_STATE_DATA_PIN_DETECT,
    UHDCD_STATE_CHRG_PRT_DETECT,
    UHDCD_STATE_CHRG_TYP_DETECT,

} uhdcd_state_t;


uhdcd_error_t uhdcd_run(uhdcd_state_t *state);
