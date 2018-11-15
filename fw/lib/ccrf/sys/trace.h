#pragma once

#include <usr/type.h>
#include <board/trace.h>
#include <string.h>

#define ccrf_trace(format, ...) board_trace_f(format "\r\n", ##__VA_ARGS__ )
#define ccrf_trace_info         ccrf_trace
#define ccrf_trace_warn         ccrf_trace
#define ccrf_trace_error        ccrf_trace

#if DEBUG

#define ccrf_trace_debug        ccrf_trace
#define ccrf_trace_verbose      ccrf_trace

#else

#define ccrf_trace_debug(format, ...) {}
#define ccrf_trace_verbose(format, ...) {}

#endif

#define ccrf_assert             board_assert
