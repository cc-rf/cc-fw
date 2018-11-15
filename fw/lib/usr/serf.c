#include <usr/serf.h>
#include <usr/cobs.h>
#include <board/trace.h>

#include <stdlib.h>
#include <string.h>
#include <FreeRTOS.h>


mbuf_t serf_encode(u8 code, mbuf_t *mbuf)
{
    code = (u8)(SERF_CODE_PROTO_VAL | (code & SERF_CODE_M));

    serf_t serf = { .code = code };

    u8 *data;
    size_t size;

    if (!mbuf || !*mbuf) {

        data = (u8 *) &serf;
        size = sizeof(serf_t);

    } else {
        mbuf_push(mbuf, sizeof(serf_t), (u8 *) &serf);

        data = (*mbuf)->data;
        size = (*mbuf)->used;
    }

    mbuf_t frame = mbuf_new(cobs_encode_size_max(size) + 1);

    frame->used = cobs_encode(size, data, frame->data);
    frame->data[frame->used++] = 0;

    if (mbuf && *mbuf) {
        mbuf_popf(mbuf, sizeof(serf_t), NULL);
    }

    return frame;
}


size_t serf_decode(mbuf_t mbuf, size_t *size)
{
    size_t frame_size;

    *size = mbuf->used;

    for (frame_size = 0; frame_size < *size; ++frame_size) {
        if (!mbuf->data[frame_size])
            break;
    }

    if (frame_size == *size) {
        return 0;
    }
    
    size_t decoded_size = cobs_decode(frame_size, mbuf->data, mbuf->data);

    mbuf->data[decoded_size] = 0;

    *size -= frame_size + 1;

    return decoded_size;
}
