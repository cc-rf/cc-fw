#pragma once

#include <usr/type.h>
#include <usr/mbuf.h>
#include <fsl_common.h>


#define FLASH_SANITY        0xFE0BAEBE
#define FLASH_VERSION       0xBEDFECE5


typedef struct {
    u32 sanity;
    u32 version;

    u8 data[];

} user_flash_t;


status_t flsh_init(void) __fast_code;
status_t flsh_user_cmit(void) __fast_code;
status_t flsh_updt_init(void);
status_t flsh_updt_part_1(mbuf_t header, mbuf_t user_rom);
status_t flsh_updt_part_2(mbuf_t fast_code, mbuf_t text, mbuf_t data);
status_t flsh_updt_done(u32 sanity);
