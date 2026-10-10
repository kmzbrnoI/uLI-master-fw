/* Ring buffer header file. */

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <inttypes.h>
#include <stdbool.h>

#define RINGBUF_SIZE 32

typedef struct {
    uint8_t ptr_b;    // pointer to begin (for 8 items 0..7)
    uint8_t ptr_e;    // pointer to end (for 8 items 0..7)
    uint8_t data[RINGBUF_SIZE]; // data
    bool empty;       // whether buffer is empty
} ring_generic;

/* ptr_b points to first byte
 * ptr_e points to byte after last byte
 * This specially implies that is it NOT POSSIBLE to differentiate empty and full buffer.
 * This is why buffer contains special \empty flag.
 */

/* Common situations:
 * full buffer: ptr_b == ptr_e && !empty
 * empty buffer: ptr_b == ptr_e && empty
 * Empty flag must be set when manipulating with ring buffer!
 */

void ringInit(volatile ring_generic* buf);
void ringAddByte(volatile ring_generic* buf, uint8_t dat);
void ringRemoveFrame(volatile ring_generic* buf, uint8_t size);
void ringSerialize(volatile ring_generic* buf, uint8_t* out, uint8_t start, uint8_t length);
void ringRewindEnd(volatile ring_generic* buf, uint8_t end); // rewind buf->ptr_e back to 'end'

static inline bool ringFull(volatile ring_generic* buf) {
    return ((buf->ptr_b == buf->ptr_e) && (!buf->empty));
}

static inline uint8_t ringLength(volatile ring_generic* buf) {
    return (ringFull(buf)) ? RINGBUF_SIZE : ((buf->ptr_e-buf->ptr_b) % RINGBUF_SIZE);
}

static inline bool ringEmpty(volatile ring_generic* buf) {
    return buf->empty;
}

static inline uint8_t ringFreeSpace(volatile ring_generic* buf) {
    return RINGBUF_SIZE - ringLength(buf);
}

static inline uint8_t ringDistance(volatile ring_generic* buf, uint8_t first, uint8_t second) {
    return (second-first) % RINGBUF_SIZE;
}

#endif

