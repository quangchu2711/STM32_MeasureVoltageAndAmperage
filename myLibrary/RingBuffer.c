#include "RingBuffer.h"

void RB_Init(RING_BUFFER *rb, uint8_t *data, uint16_t len)
{
	rb->buffer = data;
	rb->head = 0;
	rb->tail = 0;
	rb->maxLen = len;
}

int8_t RB_Push(RING_BUFFER *rb, uint8_t data)
{
	int8_t next;
	next = rb->head + 1;
	if (next > rb->maxLen) next = 0;
	if (next == rb->tail) return -1;             //if the head + 1 == tail, ring buffer is full
	rb->buffer[rb->head] = data;
	rb->head = next;
	return 0;
}

int8_t RB_Pop(RING_BUFFER *rb, uint8_t *data)
{
	if (rb->head == rb->tail) return -1;         //we don't have any data
	int8_t next;
	next = rb->tail + 1;
	if (next > rb->maxLen) next = 0;
	*data = rb->buffer[rb->tail];
	rb->tail = next;
	return 0;
}

uint16_t RB_Available(RING_BUFFER *rb)
{
	if (rb->head < rb->tail) return (rb->maxLen - (rb->tail - rb->head));
	return (rb->head - rb->tail);
}
