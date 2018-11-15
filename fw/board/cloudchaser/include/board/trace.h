#pragma once

#include <usr/type.h>
#include <kio/itm.h>
#include <assert.h>


#define board_bkpt(value)   asm volatile ("bkpt "#value)


#if DEBUG || CONFIG_DISABLE_BOARD_TRACE != 1

    #define board_trace_fr(format, ...) itm_printf(0, format, ##__VA_ARGS__ );
    #define board_trace_f(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ );
    #define board_trace_r(s) itm_puts(0, s);

    #define board_trace(s) { board_trace_r(s); board_trace_r("\r\n"); }

#else

    #define board_trace_fr(format, ...)  {}
    #define board_trace_f(format, ...) {}
    #define board_trace_r(s) {}

    #define board_trace(s) {}

    #define board_die(...)     \
        board_trace( # __VA_ARGS__ ); \
        __BKPT(); while (1) asm("");

#endif

#if DEBUG
    #define board_die(...) \
        { \
            board_trace_f( # __VA_ARGS__ ); \
            board_bkpt(); \
            while (1) asm(""); \
        }
#else

    #define board_die(...) \
        { \
            board_trace_f( # __VA_ARGS__ ); \
            extern void board_reboot(void); \
            board_reboot(); while (1) asm(""); \
        }

#endif

#define board_assert(cond, ...)     if ( (cond) == 0 ) { board_die( # __VA_ARGS__ ); }

#undef assert
#define assert                      board_assert
