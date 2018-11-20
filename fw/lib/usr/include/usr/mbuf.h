#pragma once

#include <usr/type.h>
#include <board/trace.h>


#if CONFIG_MBUF_TRACE == 1
    #define mbuf_trace board_trace_f
#else
    #define mbuf_trace(...)

#endif


#define mbuf_die(mbuf, ...)        board_die( # __VA_ARGS__ )

#define mbuf_assert(cond, ...)     if (!(cond)) { board_trace_f( "<mbuf fail '" #cond "'> " # __VA_ARGS__ ); mbuf_die(NULL); }

#if CONFIG_MBUF_VALIDATE == 1

    #define mbuf_validate(mbuf, ...) \
        if (!(mbuf) || !mbuf_good(*(mbuf))) \
            { mbuf_die(mbuf, "<mbuf bad> " # __VA_ARGS__ ); }

#else

    #define mbuf_validate(...)

#endif

#if DEBUG

    #define mbuf_assert_debug mbuf_assert

#else

    #define mbuf_assert_debug

#endif


typedef struct mbuf {
    size_t size;
    size_t used;
    void *base;
    u8 *data;

} *mbuf_t;


void mbuf_trace_data(mbuf_t mbuf);

static inline struct mbuf mbuf_view(mbuf_t mbuf, size_t offset, size_t used) __fast_code;

void mbuf_init(mbuf_t mbuf, size_t size, u8 *data) __fast_code;

mbuf_t mbuf_new(size_t size) __fast_code;

mbuf_t mbuf_alloc(size_t size, u8 *data) __fast_code;

mbuf_t mbuf_make(size_t size, size_t used, u8 *data) __fast_code;

void mbuf_free(mbuf_t *mbuf) __fast_code;

mbuf_t mbuf_copy(mbuf_t mbuf) __fast_code;

mbuf_t mbuf_wrap(size_t size, u8 **data) __fast_code;

void mbuf_conv(mbuf_t *mbuf, size_t size, u8 **data) __fast_code;

static inline bool mbuf_good(mbuf_t mbuf) __fast_code;

static inline void mbuf_done(mbuf_t *mbuf) __fast_code;

static inline void mbuf_used(mbuf_t *mbuf, size_t used) __fast_code;

static inline void mbuf_fits(mbuf_t *mbuf, size_t size) __fast_code;

static inline void mbuf_extd(mbuf_t *mbuf, mbuf_t *extd) __fast_code;

static inline void mbuf_pack(mbuf_t *mbuf) __fast_code;

void mbuf_size(mbuf_t *mbuf, size_t size) __fast_code;

void mbuf_grow(mbuf_t *mbuf, size_t size, u8 *data) __fast_code;

void mbuf_push(mbuf_t *mbuf, size_t size, u8 *data) __fast_code;

void mbuf_popf(mbuf_t *mbuf, size_t size, u8 *data) __fast_code;

void mbuf_popb(mbuf_t *mbuf, size_t size, u8 *data) __fast_code;




static inline struct mbuf mbuf_view(mbuf_t mbuf, size_t offset, size_t used)
{
    struct mbuf new;

    mbuf_init(&new, used, mbuf->data + offset);

    new.used = used;

    return new;
}



static inline bool mbuf_good(mbuf_t mbuf)
{
    return mbuf && mbuf->base && mbuf->data && mbuf->used <= mbuf->size;
}


static inline void mbuf_done(mbuf_t *mbuf)
{
    mbuf_used(mbuf, 0);
}


static inline void mbuf_used(mbuf_t *mbuf, size_t used)
{
    if (mbuf && *mbuf) {
        mbuf_fits(mbuf, used);
        (*mbuf)->used = used;
    }
}


static inline void mbuf_fits(mbuf_t *mbuf, size_t size)
{
    if (size > (*mbuf)->size) {
        mbuf_size(mbuf, size);
    }
}


static inline void mbuf_extd(mbuf_t *mbuf, mbuf_t *extd)
{
    mbuf_grow(mbuf, (*extd)->used, (*extd)->data);
}


static inline void mbuf_pack(mbuf_t *mbuf)
{
    mbuf_size(mbuf, (*mbuf)->used);
}
