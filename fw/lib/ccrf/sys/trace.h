#pragma once

#include <usr/type.h>


#if CCRF_CONFIG_PLATFORM_kinetis == 1

#include <kio/itm.h>

#define ccrf_trace_info(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ )
#define ccrf_trace_warn(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ )
#define ccrf_trace_error(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ )
#define ccrf_trace_debug(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ )
#define ccrf_trace_verbose(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ )

#endif
