#pragma once

#include <usr/type.h>
#include <string.h>


#if CCRF_CONFIG_PLATFORM_kinetis == 1

#if DEBUG

#include <kio/itm.h>

#define ccrf_trace_info(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ )
#define ccrf_trace_warn(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ )
#define ccrf_trace_error(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ )
#define ccrf_trace_debug(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ )
#define ccrf_trace_verbose(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ )

#else

#define ccrf_trace_info(format, ...) {}
#define ccrf_trace_warn(format, ...) {}
#define ccrf_trace_error(format, ...) {}
#define ccrf_trace_debug(format, ...) {}
#define ccrf_trace_verbose(format, ...) {}

#endif

#endif
