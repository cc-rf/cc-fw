#include <usr/mbuf.h>
#include <FreeRTOS.h>
#include <umm_malloc_cfg.h>
#include <string.h>

#define MBUF_DATA(mbuf_base)    (void *) ((u8 *) (mbuf_base) + sizeof(struct mbuf))


void mbuf_trace_data(mbuf_t mbuf)
{
    board_trace_fr("%08X: u=%u\n->", mbuf, mbuf->used);

    for (size_t i = 0; i < mbuf->used; ++i) {
        board_trace_fr(" %02X", mbuf->data[i]);
    }

    board_trace("");
}


void mbuf_init(mbuf_t mbuf, size_t size, u8 *data)
{
    mbuf->size = size;
    mbuf->used = 0;
    mbuf->base = NULL;
    mbuf->data = data;
}


mbuf_t mbuf_new(size_t size)
{
    mbuf_t mbuf = mbuf_alloc(size, NULL);
    mbuf_used(&mbuf, size);
    return mbuf;
}


mbuf_t mbuf_alloc(size_t size, u8 *data)
{
    #ifdef UMM_INTEGRITY_CHECK
    mbuf_assert_debug(INTEGRITY_CHECK(), "mbuf: integrity lost!");
    #endif

    void *base = pvPortMalloc(size + sizeof(struct mbuf));
    mbuf_t mbuf = (mbuf_t) base;

    mbuf_init(mbuf, size, MBUF_DATA(base));

    mbuf->base = mbuf;

    if (data) {
        if (mbuf->size) memcpy(mbuf->data, data, size);
        mbuf->used += size;
    }

    mbuf_trace("%08X: <+> base=%08X data=%08X s=%u u=%u", mbuf, mbuf->base, mbuf->data, mbuf->size, mbuf->used);

    return mbuf;
}


mbuf_t mbuf_make(size_t size, size_t used, u8 *data)
{
    mbuf_t mbuf = mbuf_alloc(size, NULL);
    mbuf_used(&mbuf, used);

    if (data) memcpy(mbuf->data, data, used);

    return mbuf;
}


void mbuf_free(mbuf_t *mbuf)
{
    if (mbuf) {
        if (*mbuf && (**mbuf).base) {
            mbuf_validate(mbuf, "free");
            mbuf_trace("%08X: <-> base=%08X data=%08X s=%u u=%u", *mbuf, (**mbuf).base, (**mbuf).data, (**mbuf).size, (**mbuf).used);

            (**mbuf).size = (**mbuf).used = 0;

            mbuf_t base = (**mbuf).base;

            (**mbuf).base = NULL;
            (**mbuf).data = NULL;

            vPortFree(base);
        }

        *mbuf = NULL;
    }
}


inline mbuf_t mbuf_copy(mbuf_t mbuf)
{
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Waddress"
    mbuf_validate(&mbuf, "copy");
    #pragma GCC diagnostic pop
    return mbuf_alloc(mbuf->used, mbuf->data);
}


mbuf_t mbuf_wrap(size_t size, u8 **data)
{
    mbuf_assert(data, mbuf_trace("data* == NULL"));
    mbuf_assert(*data, mbuf_trace("*data == NULL"));

    void *base = pvPortRealloc(*data, size + sizeof(struct mbuf));
    mbuf_t mbuf = (mbuf_t) base;

    if (size)
        memmove(MBUF_DATA(base), base, size);

    mbuf_init(mbuf, size, MBUF_DATA(base));

    mbuf->base = mbuf;

    mbuf->used += size;

    *data = NULL;

    return mbuf;
}


void mbuf_conv(mbuf_t *mbuf, size_t size, u8 **data)
{
    mbuf_assert(data, mbuf_trace("data* == NULL"));
    mbuf_assert(*data, mbuf_trace("*data == NULL"));

    if (size) {
        mbuf_done(mbuf);
        mbuf_grow(mbuf, size, *data);
    }
    else mbuf_done(mbuf);

    *data = NULL;
    vPortFree(*data);
}


void mbuf_size(mbuf_t *mbuf, size_t size)
{
    mbuf_validate(mbuf, "size");

    if (size != (**mbuf).size) {

        #if MBUF_TRACE
        size_t mbuf_size = (**mbuf).size;
        #endif

        size_t size_new = size + sizeof(struct mbuf);

        *mbuf = pvPortRealloc((**mbuf).base, size_new);

        if ((**mbuf).base != *mbuf) {
            mbuf_trace("%08X: new b=%08X", (**mbuf).base, *mbuf);
            (**mbuf).base = *mbuf;
            (**mbuf).data = MBUF_DATA((**mbuf).base);
        }

        (**mbuf).size = size_new - sizeof(struct mbuf);

        if ((**mbuf).used > (**mbuf).size)
            (**mbuf).used = (**mbuf).size;

        mbuf_trace("%08X: size s=%u->%u u=%u", *mbuf, mbuf_size, size, (**mbuf).used);
    }
}


void mbuf_grow(mbuf_t *mbuf, size_t size, u8 *data)
{
    mbuf_validate(mbuf, "grow");

    if (!size) return;

    mbuf_fits(mbuf, (**mbuf).used + size);

    if (data)
        memcpy(&((**mbuf).data[(**mbuf).used]), data, size);

    (**mbuf).used += size;

    mbuf_trace("%08X: grow s=%u u=%u (+%u)", *mbuf, (**mbuf).size, (**mbuf).used, size);
}


void mbuf_push(mbuf_t *mbuf, size_t size, u8 *data)
{
    mbuf_validate(mbuf, "push");

    if (!size) return;

    size_t size_new = (**mbuf).used + size;

    mbuf_fits(mbuf, size_new);

    if ((**mbuf).used)
        memmove(&((**mbuf).data[size]), (**mbuf).data, (**mbuf).used);

    if (data)
        memcpy((**mbuf).data, data, size);

    (**mbuf).used += size;

    mbuf_trace("%08X: push u=%u (+%u)", *mbuf, (**mbuf).used, size);
}

void mbuf_popf(mbuf_t *mbuf, size_t size, u8 *data)
{
    mbuf_validate(mbuf, "popf");

    if (!size) return;

    mbuf_assert(size <= (**mbuf).used, "cant popf: s=%u > u=%u", size, (**mbuf).used);

    if (data)
        memcpy(data, (**mbuf).data, size);

    if (((**mbuf).used -= size))
        memmove((**mbuf).data, &((**mbuf).data[size]), (**mbuf).used);

    mbuf_trace("%08X: popf u=%u (-%u)", *mbuf, (**mbuf).used, size);
}


void mbuf_popb(mbuf_t *mbuf, size_t size, u8 *data)
{
    mbuf_validate(mbuf, "popb");

    if (!size) return;

    mbuf_assert(size <= (**mbuf).used, "cant popb: s=%u > u=%u", size, (**mbuf).used);

    if (data)
        memcpy(data, &((**mbuf).data[(**mbuf).used - size]), size);

    (**mbuf).used -= size;

    mbuf_trace("%08X: popb u=%u (-%u)", *mbuf, (**mbuf).used, size);
}
