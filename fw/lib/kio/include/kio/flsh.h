#pragma once

#include <usr/type.h>
#include <fsl_common.h>


#define FLASH_SANITY        0xFE0BAEBE
#define FLASH_VERSION       0xBEDFECE5


typedef struct {
    u32 sanity;
    u32 version;

    u8 data[];

} user_flash_t;


status_t flsh_init(void) __fast_code;
status_t flsh_erase(u32 begin[], u32 end[]) __fast_code;
status_t flsh_ewrite(u32 *begin, u32 *end, u32 dest[]) __fast_code;
status_t flsh_write(u32 *begin, u32 *end, u32 dest[])__fast_code;
status_t flsh_user_cmit(void) __fast_code;
