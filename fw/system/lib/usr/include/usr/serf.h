#include <usr/type.h>


#define SERF_CODE_PROTO_M    0xE0 // 0b11100000
#define SERF_CODE_PROTO_VAL  0xA0 // 0b10100000
#define SERF_CODE_M          0x1f // 0b00011111


typedef struct __packed {
    u8  code;
    u8  data[];

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
size_t serf_encode(u8 code, u8 data[], size_t size, u8 **frame);

/**
 * Decode a serial frame delimited by a zero at the end.
 *
 * @param data Input buffer.
 * @param size Input buffer size, updated to indicate leftover amount on successful decode.
 * @param frame Output buffer for decoded frame.
 * @param limit Maximum input/output buffer size.
 * @return Size of decoded frame, if any.
 */
size_t serf_decode(u8 *data, size_t *size, serf_t *frame, size_t limit);
