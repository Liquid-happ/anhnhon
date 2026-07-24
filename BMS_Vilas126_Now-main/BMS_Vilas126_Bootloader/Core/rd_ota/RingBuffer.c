/*
 * RingBuffer.c
 *
 *  Created on: Oct 21, 2020
 *      Author: duanlc
 */
#include "RingBuffer.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

volatile unsigned char ringbuffere[RINGBUFFER_LEN];
ringbuffer_t 		vrts_ringbuffer_Data;

bool ring_init(ringbuffer_t *cb, size_t capacity, size_t sz){
	cb->buffer = (void *)ringbuffere;  // cap phat bo nho dong
	if(cb->buffer == NULL){
		return false;
	}
	cb->buffer_end = (char *)cb->buffer + capacity * sz;
	cb->capacity = capacity;
	cb->count = 0;
	cb->sz = sz;
	cb->head = cb->buffer;
	cb->tail = cb->buffer;
	return true;
}

bool ring_free(ringbuffer_t *cb)
{
	if(cb->buffer){
		free(cb->buffer);
		cb->buffer = NULL;
	}else{
		return false;
	}
	return true;
}

bool ring_push_head(ringbuffer_t *cb, const void *item)
{
	if(cb->count == cb->capacity){
		return false;
	}

	memcpy(cb->head, item, cb->sz);
	cb->head = (char*)cb->head + cb->sz;
	if(cb->head == cb->buffer_end)
	cb->head = cb->buffer;
	if(cb->head >= cb->tail){
		cb->count = cb->head - cb->tail;
	}
	else if(cb->head < cb->tail){
		cb->count = cb->head + RINGBUFFER_LEN - cb->tail;
	}
	//cb->count++;

	return true;
}

bool ring_pop_tail(ringbuffer_t *cb, void *item)
{
	if(cb->count == 0){
		//printf("Ringbuffer is Empty ! \n");
		return false;
	}

	memcpy(item, cb->tail, cb->sz);
	cb->tail = (char*)cb->tail + cb->sz;
	if(cb->tail == cb->buffer_end)
		cb->tail = cb->buffer;
	if(cb->head >= cb->tail){
		cb->count = cb->head - cb->tail;
	}
	else if(cb->head < cb->tail){
		cb->count = cb->head + RINGBUFFER_LEN - cb->tail;
	}
	//cb->count--;
	return true;
}
