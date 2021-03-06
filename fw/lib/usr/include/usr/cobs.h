#pragma once

#include <usr/type.h>


__fast_code static inline size_t cobs_encode_size_max(size_t sz)
{
    // max encode length: size + 1 + (size/254) + 1/*trailing zero*/
    // max decode length: size
    return sz + sz / 254 + 1;
}


__fast_code __nonnull_all static inline size_t cobs_encode(size_t sz, const u8 * restrict in, u8 * restrict out)
{
    size_t ri = 0, wi = 1, ci = 0;
    u8 c = 1;

    while (ri < sz) {

        if (!in[ri]) {

            out[ci] = c;
            c = 1;
            ci = wi++;
            ri++;

        } else {

            out[wi++] = in[ri++];

            if (++c == 0xFF) {

                out[ci] = c;
                c = 1;
                ci = wi++;
            }
        }
    }

    out[ci] = c;

    return wi;
}


__fast_code __nonnull_all static inline size_t cobs_decode(size_t sz, const u8 *in, u8 *out)
{
    size_t ri = 0, wi = 0;
    u8 c, i;

    while (ri < sz) {

        c = in[ri];

        if (ri + c > sz && c != 1)
            return 0;

        ++ri;

        for (i = 1; i < c; i++)
            out[wi++] = in[ri++];

        if (c != 0xFF && ri != sz)
            out[wi++] = 0u;
    }

    return wi;
}
