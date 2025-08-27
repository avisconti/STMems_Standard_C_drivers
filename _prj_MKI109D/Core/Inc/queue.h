#ifndef QUEUE_H
#define QUEUE_H

#include <stdint.h>

#define MSG_QUEUE_ALLOCATED_SIZE 4096
#define MSG_QUEUE_MAX_CAPACITY (MSG_QUEUE_ALLOCATED_SIZE - 1)

typedef struct {
  uint8_t _msg_queue[MSG_QUEUE_MAX_CAPACITY];
  uint32_t _head;
  uint32_t _tail;
} msg_queue;

typedef enum {
  MSG_QUEUE_OK,
  MSG_QUEUE_ERROR
} msg_queue_status;

uint32_t msg_queue_get_size(msg_queue *queue);
uint8_t msg_queue_is_empty(msg_queue *queue);
uint8_t msg_queue_is_full(msg_queue *queue);
uint32_t msg_enqueue(msg_queue *queue, uint8_t *buf, uint32_t size);
uint32_t msg_dequeue(msg_queue *queue, uint8_t *buf, uint32_t size);
uint32_t str_msg_dequeue(msg_queue *queue, uint8_t *str_buf, uint32_t str_buf_max_size);
void msg_queue_to_str(msg_queue *queue, char *str_buf, uint32_t str_buf_max_size);
uint32_t msg_queue_to_arr(msg_queue *queue, uint8_t *arr, uint32_t arr_max_size);
void msg_queue_flush(msg_queue *queue);

#endif
