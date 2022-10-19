/**
  ******************************************************************************
  * @file    RingBuffer.h
  * @author  Chu Xuan Quang
  * @date    19-1-2022
  * @brief   Ring Buffer program body
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#ifndef RING_BUFFER_H
#define RING_BUFFER_H
#include <stdint.h>

typedef struct 
{
	uint8_t *buffer;
	int8_t head;
	int8_t tail;
	uint16_t maxLen;
} RING_BUFFER;

void RB_Init(RING_BUFFER *rb, uint8_t *data, uint16_t len);
int8_t RB_Push(RING_BUFFER *rb, uint8_t data);
int8_t RB_Pop(RING_BUFFER *rb, uint8_t *data);
uint16_t RB_Available(RING_BUFFER *rb);
#endif
