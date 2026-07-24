/*
 * RingBuffer.h
 *
 *  Created on: Oct 21, 2020
 *      Author: duanlc
 */

#ifndef GATEWAYMANAGER_RINGBUFFER_H_
#define GATEWAYMANAGER_RINGBUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>

#define RINGBUFFER_LEN 		(1024*4) // 3KB

extern volatile unsigned char ringbuffere[RINGBUFFER_LEN];

typedef struct ringbuffer_t
{
    void *buffer;   // data buffer
    void *buffer_end; // end of data buffer
    size_t capacity; // maximum number of items in the buffer
    volatile size_t count;   
    size_t sz;    // size of each item in the buffer
    void * volatile head;   
    void * volatile tail;    
} ringbuffer_t;
extern ringbuffer_t 		vrts_ringbuffer_Data;
bool ring_init(ringbuffer_t *cb, size_t capacity, size_t sz);

bool ring_free(ringbuffer_t *cb);

bool ring_push_head(ringbuffer_t *cb, const void *item);

bool ring_pop_tail(ringbuffer_t *cb, void *item);

#ifdef __cplusplus
}
#endif



#endif /* GATEWAYMANAGER_RINGBUFFER_H_ */
