/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 *
 * Receive ring buffer plus the CAN FD length/DLC and bit-timing helpers
 * shared by both protocol back ends.
 */
#include "kvaser_priv.h"

#include <stdlib.h>
#include <string.h>

int kv_queue_init(struct kv_queue *q, uint32_t cap)
{
    memset(q, 0, sizeof(*q));
    if (!cap)
        return -1;
    q->items = calloc(cap, sizeof(struct kv_rx_item));
    if (!q->items)
        return -1;
    q->cap = cap;
    pthread_mutex_init(&q->lock, NULL);
    return 0;
}

void kv_queue_free(struct kv_queue *q)
{
    if (!q || !q->items)
        return;
    pthread_mutex_destroy(&q->lock);
    free(q->items);
    memset(q, 0, sizeof(*q));
}

void kv_queue_clear(struct kv_queue *q)
{
    if (!q->items)
        return;
    pthread_mutex_lock(&q->lock);
    q->head = q->tail = q->count = q->overrun = 0;
    pthread_mutex_unlock(&q->lock);
}

/* Oldest frame is dropped when the queue is full, matching linuxcan. */
int kv_queue_push(struct kv_queue *q, const kv_msg *m)
{
    if (!q->items)
        return -1;
    pthread_mutex_lock(&q->lock);
    if (q->count == q->cap) {
        q->head = (q->head + 1) % q->cap;
        q->count--;
        q->overrun++;
    }
    q->items[q->tail].msg = *m;
    q->tail = (q->tail + 1) % q->cap;
    q->count++;
    pthread_mutex_unlock(&q->lock);
    return 0;
}

int kv_queue_pop(struct kv_queue *q, kv_msg *m)
{
    if (!q->items)
        return 0;
    pthread_mutex_lock(&q->lock);
    if (!q->count) {
        pthread_mutex_unlock(&q->lock);
        return 0;
    }
    *m = q->items[q->head].msg;
    q->head = (q->head + 1) % q->cap;
    q->count--;
    pthread_mutex_unlock(&q->lock);
    return 1;
}

/* Called from the receive thread only. Frames for a channel that is not open
 * are dropped rather than queued. */
void kv_push_frame(struct kv_channel *ch, uint32_t id, uint8_t flags,
                   const uint8_t *data, int len, uint8_t dlc, uint64_t ts)
{
    kv_msg m;
    if (!ch->open)
        return;
    memset(&m, 0, sizeof(m));
    m.id = id;
    m.flags = flags;
    m.dlc = dlc;
    m.len = (uint8_t)(len < 64 ? len : 64);
    if (data && m.len)
        memcpy(m.data, data, m.len);
    m.ts_us = ts;
    kv_queue_push(&ch->q, &m);
}

uint8_t kv_len_to_dlc(int len, int fd)
{
    if (len <= 8)
        return (uint8_t)(len < 0 ? 0 : len);
    if (!fd)
        return 8;
    if (len <= 12) return 9;
    if (len <= 16) return 10;
    if (len <= 20) return 11;
    if (len <= 24) return 12;
    if (len <= 32) return 13;
    if (len <= 48) return 14;
    return 15;
}

int kv_dlc_to_len(uint8_t dlc, int fd)
{
    static const int fd_len[16] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
    };
    if (dlc > 15)
        dlc = 15;
    if (!fd)
        return dlc > 8 ? 8 : dlc;
    return fd_len[dlc];
}

/* Pick the number of time quanta per bit. The firmware derives the prescaler
 * as clock / (bitrate * quanta), so a quanta count that does not divide the
 * clock exactly yields a fractional prescaler: the firmware then either
 * rejects the parameters or runs at a bit rate nobody asked for. Prefer the
 * largest count that divides exactly, since more quanta means finer control
 * over the sample point. */
static int quanta_for(int bps, int clock_hz)
{
    int q;
    if (bps <= 0)
        return 16;
    for (q = 25; q >= 8; q--) {
        int denom = bps * q;
        if (denom > 0 && clock_hz % denom == 0)
            return q;
    }
    /* Nothing divides exactly; fall back to the historical buckets. */
    if (bps >= 1000000)
        return 8;
    if (bps >= 800000)
        return 10;
    return 16;
}

void kv_timing_clk(int bps, int sample_pct, int clock_hz,
                   uint8_t *tseg1, uint8_t *tseg2, uint8_t *sjw)
{
    int quanta, t1, t2, sample;

    quanta = quanta_for(bps, clock_hz > 0 ? clock_hz : 80000000);
    sample = sample_pct > 0 ? sample_pct : 80;
    if (sample < 50) sample = 50;
    if (sample > 90) sample = 90;
    t1 = (int)((sample / 100.0 * quanta) + 0.5) - 1;
    t2 = quanta - 1 - t1;
    if (t2 < 1) { t2 = 1; t1 = quanta - 2; }
    if (t1 < 1) { t1 = 1; t2 = quanta - 2; }
    *tseg1 = (uint8_t)t1;
    *tseg2 = (uint8_t)t2;
    /* Scale the resynchronisation jump width with the bit length, capped at
     * the 4 quanta the controllers accept, rather than always using 1. */
    *sjw = (uint8_t)(t2 < 4 ? (t2 < 1 ? 1 : t2) : 4);
}

void kv_timing(int bps, int sample_pct, uint8_t *tseg1, uint8_t *tseg2, uint8_t *sjw)
{
    kv_timing_clk(bps, sample_pct, 80000000, tseg1, tseg2, sjw);
}

/* Timing for one phase of one channel: automatic from bit rate and sample
 * point, unless the caller supplied explicit segments in kv_open_opts. */
void kv_resolve_timing(int bps, const kv_open_opts *opts, int data_phase,
                       int clock_hz, uint8_t *tseg1, uint8_t *tseg2, uint8_t *sjw)
{
    int o1 = 0, o2 = 0, osjw = 0;

    kv_timing_clk(bps, opts ? opts->sample_point : 80, clock_hz, tseg1, tseg2, sjw);
    if (opts) {
        o1   = data_phase ? opts->data_tseg1 : opts->tseg1;
        o2   = data_phase ? opts->data_tseg2 : opts->tseg2;
        osjw = data_phase ? opts->data_sjw   : opts->sjw;
    }
    if (o1 > 0 && o1 <= 255 && o2 > 0 && o2 <= 255) {
        *tseg1 = (uint8_t)o1;
        *tseg2 = (uint8_t)o2;
        *sjw = (uint8_t)((osjw > 0 && osjw <= 255) ? osjw : 1);
    }
}
