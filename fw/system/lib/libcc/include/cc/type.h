#pragma once

#include <usr/type.h>
#include <sys/cdefs.h> // TODO: maybe put this in type.h or some 'std.h' somewhere

// http://stackoverflow.com/questions/4415524/common-array-length-macro-for-c
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

typedef u8      cc_dev_t;
typedef u64     nsec_t;


typedef struct __packed {
    u8 len;
    u8 data[];

} rf_pkt_t;


/*
typedef enum __packed {
    PKT_TYPE_DATA,
    PKT_TYPE_SYNC

} pkt_type_t;

typedef enum __packed {
    PKT_FLAG_ACK_REQ = 1 << 0,  // ACK requested
    PKT_FLAG_ACK_RSP = 1 << 1,  // ACK indicated
    PKT_FLAG_ACK_WIN = 1 << 2,  //
    PKT_FLAG_IS_GATE = 1 << 3,
    PKT_FLAG_IS_BOSS = 1 << 4,

} pkt_flag_t;



typedef struct __packed {


} phy_pkt_hdr_t;

typedef struct __packed {
    u8 len;
    u16 addr;
    u8 seq;
    u8 type : 2;
    u8 flag : 6;

} pkt_hdr_t;

typedef struct __packed {
    u16 addr;
    u8 seq;

} pkt_hdr_ack_t;

typedef struct __packed {
    pkt_hdr_t hdr;
    u8 data[];

} pkt_t;

typedef struct __packed {
    pkt_hdr_t hdr;
    pkt_hdr_ack_t ack;
    u8 data[];

} ack_pkt_t;*/

/*// TODO: phase out
typedef struct __packed {
    u8 len;
    u8 data[];

} cc_pkt_t;*/
