/*
 * Ring buffer implementation
 * (c) Michal Petrilak, Jan Horacek 2015
 */

#include "ringBuffer.h"

void ringAddByte(volatile ring_generic *buf, BYTE data)
{
    buf->data[buf->ptr_e] = data;
    buf->ptr_e = (buf->ptr_e + 1) & buf->max;
    buf->empty = FALSE;
}

BYTE ringRemoveByte(volatile ring_generic* buf)
{
    BYTE result;
    if (ringLength(buf) == 0) return 0;
    result = buf->data[buf->ptr_b];
    buf->ptr_b = (buf->ptr_b + 1) & buf->max;
    if (buf->ptr_b == buf->ptr_e) buf->empty = TRUE;
    return result;
}

void ringRemoveFrame(volatile ring_generic* buf, BYTE num)
{
    BYTE x;
    x = ringLength(buf);
    if (x > num) x = num;
    buf->ptr_b = (buf->ptr_b + x) & buf->max;
    if (buf->ptr_b == buf->ptr_e) buf->empty = TRUE;
}

BYTE ringReadByte(volatile ring_generic* buf, BYTE offset)
{
    BYTE pos;
    pos = (buf->ptr_b + offset) & buf->max;
    return buf->data[pos];
}

BYTE ringLength(volatile ring_generic* buf)
{
    return ((buf->ptr_e - buf->ptr_b) & buf->max) + (!!ringFull(buf) * (buf->max+1));
}

BOOL ringFull(volatile ring_generic* buf)
{
    return (buf->ptr_b == buf->ptr_e) && (!buf->empty);
}

BOOL ringEmpty(volatile ring_generic* buf)
{
    return (buf->ptr_b == buf->ptr_e) && (buf->empty);
}

BYTE ringFreeSpace(volatile ring_generic* buf)
{
    return (buf->max+1) - ringLength(buf);
}

BYTE ringDistance(volatile ring_generic* buf, BYTE first, BYTE second)
{
    return ((second-first) & buf->max);
}

void ringSerialize(volatile ring_generic* buf, BYTE* out, BYTE start, BYTE length)
{
    int i;
    for (i = 0; i < length; i++)
        out[i] = buf->data[(start+i) & buf->max];
}

void ringRemoveFromMiddle(volatile ring_generic* buf, BYTE start, BYTE length)
{
    int i;
    for (i = start; i != ((buf->ptr_e-length)&buf->max); i++)
        buf->data[i%buf->max] = buf->data[(i+length)&buf->max];
    buf->ptr_e = i&buf->max;
    if (buf->ptr_b == buf->ptr_e) buf->empty = TRUE;
}

void ringAddToStart(volatile ring_generic* buf, BYTE* data, BYTE len)
{
    int i;
    
    // check full buffer    
    if (ringLength(buf) < len) return;
        
    buf->ptr_b = (buf->ptr_b-len)&buf->max;
    for (i = 0; i < len; i++) { buf->data[(buf->ptr_b+i)&buf->max] = data[i]; }
    if (len > 0) { buf->empty = FALSE; }
}

void ringClear(volatile ring_generic* buf)
{
    buf->ptr_b = buf->ptr_e;
    buf->empty = TRUE;
}
