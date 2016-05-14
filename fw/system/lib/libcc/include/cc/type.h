#pragma once

#include <usr/type.h>

// http://stackoverflow.com/questions/4415524/common-array-length-macro-for-c
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

typedef u8      cc_dev_t;
typedef u64     nsec_t;


