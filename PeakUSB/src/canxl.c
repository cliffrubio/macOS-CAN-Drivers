/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * CAN XL support for the PCAN-USB XL (product ID 0x0030).
 *
 * XL reuses the uCAN USB framing, so discovery, the receive thread and the
 * command endpoint are shared with the FD family. What differs is the frame:
 * a priority ID instead of an arbitration ID, up to 2048 payload bytes, and
 * three independently programmed bit-timing phases.
 */
#include "pcan_priv.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Receive queue. Separate from the CC/FD ring: a 2 KiB payload times   */
/* 65536 entries would be 137 MB, so XL buffers far fewer frames.       */
/* ------------------------------------------------------------------ */

int pcan_xl_queue_init(struct pcan_channel *ch)
{
    if (ch->xlq)
        return 0;
    ch->xlq = calloc(PEAKUSB_XL_QUEUE, sizeof(struct xl_item));
    if (!ch->xlq)
        return -1;
    ch->xlq_cap = PEAKUSB_XL_QUEUE;
    ch->xlq_head = ch->xlq_tail = ch->xlq_count = ch->xlq_overrun = 0;
    return 0;
}

void pcan_xl_queue_free(struct pcan_channel *ch)
{
    free(ch->xlq);
    ch->xlq = NULL;
    ch->xlq_cap = ch->xlq_head = ch->xlq_tail = ch->xlq_count = 0;
}

/* Called from the receive thread, under the channel's queue lock. */
void pcan_xl_queue_push(struct pcan_channel *ch, const TPCANMsgXL *m,
                        uint64_t ts, int *was_empty)
{
    if (!ch->xlq)
        return;
    pthread_mutex_lock(&ch->q.lock);
    if (was_empty)
        *was_empty = (ch->xlq_count == 0);
    if (ch->xlq_count == ch->xlq_cap) {
        ch->xlq_head = (ch->xlq_head + 1) % ch->xlq_cap;
        ch->xlq_count--;
        ch->xlq_overrun++;
    }
    ch->xlq[ch->xlq_tail].msg = *m;
    ch->xlq[ch->xlq_tail].ts_us = ts;
    ch->xlq_tail = (ch->xlq_tail + 1) % ch->xlq_cap;
    ch->xlq_count++;
    pthread_mutex_unlock(&ch->q.lock);
}

int pcan_xl_queue_pop(struct pcan_channel *ch, TPCANMsgXL *m, uint64_t *ts)
{
    if (!ch->xlq)
        return 0;
    pthread_mutex_lock(&ch->q.lock);
    if (!ch->xlq_count) {
        pthread_mutex_unlock(&ch->q.lock);
        return 0;
    }
    *m = ch->xlq[ch->xlq_head].msg;
    if (ts)
        *ts = ch->xlq[ch->xlq_head].ts_us;
    ch->xlq_head = (ch->xlq_head + 1) % ch->xlq_cap;
    ch->xlq_count--;
    pthread_mutex_unlock(&ch->q.lock);
    return 1;
}

/* ------------------------------------------------------------------ */
/* Receive decode                                                      */
/* ------------------------------------------------------------------ */

/* An XL record, already bounded by the caller to its own declared size. */
void pcan_xl_decode(struct pcan_device *d, uint8_t *ptr, uint16_t size)
{
    struct canxl_msg_xl *rx = (struct canxl_msg_xl *)ptr;
    struct pcan_channel *ch;
    TPCANMsgXL m;
    uint32_t packed;
    unsigned dlc, avail;
    int ci, was_empty = 0;

    /* The fixed part runs to d[]; anything shorter carries no frame. */
    if ((size_t)size < offsetof(struct canxl_msg_xl, d))
        return;
    ci = rx->channel & 0x0f;
    if (ci < 0 || ci >= d->can_count)
        return;
    ch = &d->ch[ci];
    if (!ch->initialized || !ch->xl_mode)
        return;

    packed = rx->pid_rrs_dlc_sec_sdt;
    dlc = CANXL_DLC_GET(packed);
    if (dlc > sizeof(m.DATA))
        dlc = sizeof(m.DATA);
    /* The DLC is the sender's claim; the record length is the truth. */
    avail = (unsigned)size - (unsigned)offsetof(struct canxl_msg_xl, d);
    if (dlc > avail)
        dlc = avail;

    memset(&m, 0, sizeof(m));
    m.PID     = CANXL_PID(packed);
    m.VCID    = rx->vcid;
    m.DLC     = (WORD)dlc;
    m.SDT     = (BYTE)CANXL_SDT(packed);
    m.AF      = rx->af;
    m.RRS     = (rx->flags & CANXL_MSG_RRS) ? 1 : 0;
    m.SEC     = (rx->flags & CANXL_MSG_SEC) ? 1 : 0;
    m.MSGTYPE = PCAN_MESSAGE_EXTENDED;
    if (dlc)
        memcpy(m.DATA, rx->d, dlc);

    /* The adapter timestamps in nanoseconds; the API is microseconds. */
    pcan_xl_queue_push(ch, &m, rx->tag / 1000u, &was_empty);
    if (was_empty)
        pcan_event_signal(ch);
}

/* ------------------------------------------------------------------ */
/* Transmit                                                            */
/* ------------------------------------------------------------------ */

int pcan_xl_write_msg(struct pcan_channel *ch, const TPCANMsgXL *msg)
{
    struct pcan_device *d = ch->dev;
    uint8_t buf[2048 + 64];
    struct canxl_msg_xl *tx = (struct canxl_msg_xl *)buf;
    unsigned dlc = msg->DLC;
    unsigned payload, total;
    int transferred = 0, err;

    if (d->family != PCAN_FAM_XL)
        return -1;
    if (dlc < 1 || dlc > sizeof(msg->DATA))
        return -1;

    /* Payload is padded to a multiple of four; the record size follows. */
    payload = ((dlc + 3u) / 4u) * 4u;
    total = (unsigned)offsetof(struct canxl_msg_xl, d) + payload;

    memset(buf, 0, total);
    tx->size    = (uint16_t)total;
    tx->type    = CANXL_TX_MSG_XL;
    tx->channel = (uint8_t)ch->can_idx;
    tx->tag     = 0;
    tx->vcid    = msg->VCID;
    tx->client  = 0;
    tx->flags   = CANXL_MSG_XLF;
    if (msg->RRS)
        tx->flags |= CANXL_MSG_RRS;
    if (msg->SEC)
        tx->flags |= CANXL_MSG_SEC;
    tx->pid_rrs_dlc_sec_sdt = CANXL_PACK(msg->PID, dlc, msg->SDT);
    tx->af = msg->AF;
    memcpy(tx->d, msg->DATA, dlc);

    err = libusb_bulk_transfer(d->h, d->ep_data_out[ch->can_idx], buf,
                               (int)total, &transferred, PEAKUSB_TX_TIMEOUT);
    if (err) {
        pcan_log("XL bulk out: %s", libusb_strerror(err));
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Bit timing                                                          */
/* ------------------------------------------------------------------ */

/* Programme one phase. XL runs a 160 MHz core clock, so the three phases are
 * derived from the same divisor arithmetic as CC and FD, just written to
 * three separate commands. */
static int xl_timing(uint8_t *buf, size_t *len, int ci,
                     uint16_t opcode, uint16_t tseg1, uint16_t tseg2,
                     uint16_t sjw, uint16_t brp)
{
    struct canxl_timing *t = pcan_cmd_add(buf, len, ci, opcode);
    if (!t)
        return -1;
    t->tseg1 = (uint16_t)(tseg1 - 1);
    t->sjw_tseg2_brp = CANXL_TIMING_PACK(sjw, tseg2, brp);
    return 0;
}

int pcan_xl_configure(struct pcan_channel *ch)
{
    struct pcan_device *d = ch->dev;
    const struct pcan_bittiming *bt = &ch->bt;
    uint8_t buf[PEAKUSB_CMD_BUF];
    size_t len = 0;
    uint16_t brp = bt->brp ? bt->brp : 1;
    uint16_t dbrp = bt->dbrp ? bt->dbrp : brp;

    if (pcan_xl_queue_init(ch) != 0)
        return -1;

    /* Nominal (arbitration) phase, then the FD data phase, then XL. Without
     * all three the controller refuses to leave reset. */
    if (xl_timing(buf, &len, ch->can_idx, CANXL_CMD_TIMING_NOMINAL,
                  bt->tseg1, bt->tseg2, bt->sjw, brp) != 0)
        return -1;
    if (xl_timing(buf, &len, ch->can_idx, CANXL_CMD_TIMING_FD,
                  bt->dtseg1 ? bt->dtseg1 : bt->tseg1,
                  bt->dtseg2 ? bt->dtseg2 : bt->tseg2,
                  bt->dsjw ? bt->dsjw : bt->sjw, dbrp) != 0)
        return -1;
    if (xl_timing(buf, &len, ch->can_idx, CANXL_CMD_TIMING_XL,
                  bt->dtseg1 ? bt->dtseg1 : bt->tseg1,
                  bt->dtseg2 ? bt->dtseg2 : bt->tseg2,
                  bt->dsjw ? bt->dsjw : bt->sjw, dbrp) != 0)
        return -1;

    ch->xl_mode = 1;
    return pcan_cmd_send(d, buf, &len);
}
