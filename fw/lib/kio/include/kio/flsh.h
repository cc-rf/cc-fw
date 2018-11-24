#pragma once

#include <usr/type.h>
#include <usr/mbuf.h>
#include <fsl_common.h>
#include <ccrf/net.h>


#define FLASH_SANITY        0x13370000  // No need to change this one.
#define FLASH_VERSION       0xFECE5000  // Increment this when adding variables to the .user section.


typedef struct {
    u32 sanity;
    u32 version;

    u8 data[];

} user_flash_t;


typedef struct {
    u32 header;
    u32 user_rom;
    u32 fast_code;
    u32 text;
    u32 data;
    u32 total;

} flsh_size_t;


extern const u32 __flsh_date;       // Compile timestamp.
extern const u32 __flsh_version;    // Git commit id.


status_t flsh_init(void);
status_t flsh_user_cmit(void);
bool flsh_fota(net_t net, net_path_t path);
status_t flsh_updt_init(flsh_size_t *size);
status_t flsh_updt_part_1(mbuf_t header, mbuf_t user_rom);
status_t flsh_updt_part_2(mbuf_t fast_code, mbuf_t text, mbuf_t data);
status_t flsh_updt_done(u32 sanity);
