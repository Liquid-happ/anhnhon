#ifndef COMMON_RB_H_
#define COMMON_RB_H_

#include <stddef.h>
#include <stdbool.h>

#define RINGBUFFER_LEN 		(1024*4) // 4KB

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

extern ringbuffer_t vrts_ringbuffer_Data;

bool ring_init(ringbuffer_t *cb, size_t capacity, size_t sz);
bool ring_free(ringbuffer_t *cb);
bool ring_push_head(ringbuffer_t *cb, const void *item);
bool ring_pop_tail(ringbuffer_t *cb, void *item);

#endif /* COMMON_RB_H_ */
