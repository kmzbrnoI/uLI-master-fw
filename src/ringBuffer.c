/* Ring buffer implementation */

#include "ringBuffer.h"

void ringInit(volatile ring_generic* buf) {
    buf->ptr_b = 0;
    buf->ptr_e = 0;
    buf->empty = true;
}

void ringAddByte(volatile ring_generic* buf, uint8_t data) {
    buf->data[buf->ptr_e] = data;
    buf->ptr_e = (buf->ptr_e + 1) % RINGBUF_SIZE;
    buf->empty = false;
}

void ringRemoveFrame(volatile ring_generic* buf, uint8_t size) {
    uint8_t len = ringLength(buf);
    if (len > size)
		len = size;
    buf->ptr_b = (buf->ptr_b + len) % RINGBUF_SIZE;
    if (buf->ptr_b == buf->ptr_e)
		buf->empty = true;
}

void ringSerialize(volatile ring_generic* buf, uint8_t* out, uint8_t start, uint8_t length) {
    for (uint8_t i = 0; i < length; i++)
        out[i] = buf->data[(start + i) % RINGBUF_SIZE];
}

void ringRewindEnd(volatile ring_generic* buf, uint8_t end) {
	buf->ptr_e = end;
	if (buf->ptr_e == buf->ptr_b)
		buf->empty = true;
}