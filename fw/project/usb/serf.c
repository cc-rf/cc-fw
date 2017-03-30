#include <usr/type.h>
#include <usr/cobs.h>

#define SERF_CODE_PROTO_M    0xE0 // 0b11100000
#define SERF_CODE_PROTO_VAL  0xA0 // 0b10100000
#define SERF_CODE_M          0x1f // 0b00011111

typedef struct __packed {
    u8  code;
    u8  data[];

} serf_t;

