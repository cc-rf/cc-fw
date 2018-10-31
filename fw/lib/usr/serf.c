#include <usr/serf.h>

#include <usr/cobs.h>

#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <FreeRTOS.h>


size_t serf_encode(u8 code, u8 data[], size_t size, u8 **frame)
{
    code = (u8)(SERF_CODE_PROTO_VAL | (code & SERF_CODE_M));
    *frame = NULL;

    size_t frame_size = sizeof(serf_t) + size;

    serf_t *raw_frame = pvPortMalloc(frame_size); assert(raw_frame);
    *frame = pvPortMalloc(sizeof(serf_t) + cobs_encode_size_max(frame_size) + 1); assert(*frame);

    raw_frame->code = code;
    memcpy(raw_frame->data, data, size);

    frame_size = cobs_encode((u8 *)raw_frame, frame_size, *frame);
    vPortFree(raw_frame);

    if (!frame_size) {
        vPortFree(*frame);
        return 0;
    }

    (*frame)[frame_size] = 0;

    return frame_size + 1;
}


size_t serf_decode(u8 *data, size_t *size, serf_t *frame)
{
    assert(size);
    assert(data);
    assert(frame);

    size_t frame_size;

    for (frame_size = 0; frame_size < *size; ++frame_size) {
        if (!data[frame_size]) break;
    }

    if (frame_size == *size)
        return 0;

    // max encode length: size + 1 + (size/254) + 1/*trailing zero*/
    // max decode length: size

    size_t decoded_size = cobs_decode(data, frame_size, (u8 *) frame);
    ((u8 *)frame)[decoded_size] = 0;

    if ((*size -= frame_size + 1))
        memcpy(data, &data[frame_size + 1], *size);

    return decoded_size;
}
