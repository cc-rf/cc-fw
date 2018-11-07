#pragma once

#include <usr/type.h>
#include <fsl_common.h>


status_t flsh_init(void) __fast_code;
status_t flsh_write(u32 begin, size_t size, u32 *data) __fast_code __nonnull_all;
