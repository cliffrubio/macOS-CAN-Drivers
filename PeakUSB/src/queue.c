/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "pcan_priv.h"

#include <stdlib.h>
#include <string.h>

int pcan_queue_init(struct rx_queue *q, uint32_t cap)
{
    memset(q, 0, sizeof(*q));
    if (!cap)
        return -1;
    q->items = calloc(cap, sizeof(struct rx_item));
    if (!q->items)
        return -1;
    q->cap = cap;
    pthread_mutex_init(&q->lock, NULL);
    return 0;
}

void pcan_queue_free(struct rx_queue *q)
{
    if (!q || !q->items)
        return;
    pthread_mutex_destroy(&q->lock);
    free(q->items);
    memset(q, 0, sizeof(*q));
}

void pcan_queue_clear(struct rx_queue *q)
{
    if (!q->items)
        return;
    pthread_mutex_lock(&q->lock);
    q->head = q->tail = q->count = q->overrun = 0;
    pthread_mutex_unlock(&q->lock);
}

/* Returns 1 when the oldest frame had to be dropped to make room. */
/* was_empty, when not NULL, reports whether the queue was empty immediately
 * before this frame went in. It has to be sampled inside the same critical
 * section as the push: a reader draining the queue between a separate sample
 * and the push would make the caller skip the wakeup, and since every later
 * push then sees a non-empty queue, the reader would block forever. */
int pcan_queue_push(struct rx_queue *q, const TPCANMsgFD *m, uint64_t ts,
                    int *was_empty)
{
    int overflow = 0;
    if (!q->items)
        return 0;
    pthread_mutex_lock(&q->lock);
    if (was_empty)
        *was_empty = (q->count == 0);
    if (q->count == q->cap) {
        q->head = (q->head + 1) % q->cap;
        q->count--;
        q->overrun++;
        overflow = 1;
    }
    q->items[q->tail].msg = *m;
    q->items[q->tail].ts_us = ts;
    q->tail = (q->tail + 1) % q->cap;
    q->count++;
    pthread_mutex_unlock(&q->lock);
    return overflow;
}

int pcan_queue_pop(struct rx_queue *q, TPCANMsgFD *m, uint64_t *ts)
{
    if (!q->items)
        return 0;
    pthread_mutex_lock(&q->lock);
    if (!q->count) {
        pthread_mutex_unlock(&q->lock);
        return 0;
    }
    *m = q->items[q->head].msg;
    if (ts)
        *ts = q->items[q->head].ts_us;
    q->head = (q->head + 1) % q->cap;
    q->count--;
    pthread_mutex_unlock(&q->lock);
    return 1;
}
