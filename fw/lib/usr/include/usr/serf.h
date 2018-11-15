#pragma once

#include <usr/type.h>
#include <usr/mbuf.h>


#define SERF_CODE_PROTO_M           0xE0 // 0b11100000
#define SERF_CODE_PROTO_VAL         0xA0 // 0b10100000
#define SERF_CODE_M                 0x1f // 0b00011111


typedef struct __packed {
    u8  code;
    u8 data[];

} serf_t;


/**
 * Encode a frame for serial transmission.
 *
 * @param code Frame ID.
 * @param size Frame size.
 * @param data Frame data buffer.
 * @param frame Output buffer pointer for encoded frame (malloc'd).
 * @return Size of resulting frame.
 */
mbuf_t serf_encode(u8 code, mbuf_t *mbuf) __fast_code;

/**
 * Decode a serial frame delimited by a zero at the end.
 *
 * @param mbuf Decode buffer.
 * @param frame_size Pointer to size of data to discard after using decoded data.
 * @return Size of decoded frame, if any.
 */
size_t serf_decode(mbuf_t mbuf, size_t *frame_size) __fast_code __nonnull_all;
