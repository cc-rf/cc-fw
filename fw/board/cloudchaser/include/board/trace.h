#pragma once

#include <usr/type.h>
#include <kio/itm.h>

#if DEBUG

#define board_trace_fr(format, ...) itm_printf(0, format, ##__VA_ARGS__ );
#define board_trace_f(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ );
#define board_trace_r(s) itm_puts(0, s);
#define board_trace(s) { board_trace_r(s); board_trace_r("\r\n"); }

#else

#define board_trace_fr(format, ...)  {}
#define board_trace_f(format, ...) {}
#define board_trace_r(s) {}
#define board_trace(s) {}

#endif
