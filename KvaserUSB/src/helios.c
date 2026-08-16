/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 *
 * "helios" command protocol, used by the USBcan II generation: the original
 * USBcan, VCI-2, USBcan II and the first Memorator. Ported from linuxcan
 * usbcanII/, whose command set lives in helios_cmds.h.
 *
 * These devices were previously absent from the ID table, because the driver
 * only spoke filo and helios differs enough to fail silently. The records are
 * the same commands by number but not by layout:
 *
 *   cmdRxCanMessage   helios 20 bytes, rawMessage at 4, 16-bit time at 18
 *                     filo   24 bytes, 48-bit time at 4, rawMessage at 10
 *   cmdChipStateEvent helios 12 bytes, counters at 4 and 5
 *                     filo   16 bytes, counters at 10 and 11
 *   cmdCanErrorEvent  helios 12 bytes, per-channel counters from offset 4
 *                     filo   16 bytes, one channel, counters at 12 and 13
 *   software info     helios 16-bit swOptions at 22, serial split in two
 *                     filo   32-bit swOptions at 4
 *
 * The clock also differs: helios timestamps are a 16-bit tick at 1 MHz, with
 * the high half delivered separately by CMD_CLOCK_OVERFLOW_EVENT. Without
 * tracking that, timestamps would wrap every 65 milliseconds.
 */
#include "kvaser_priv.h"

#include <string.h>

#define CMD_RX_STD_MESSAGE          12
#define CMD_TX_STD_MESSAGE          13
#define CMD_RX_EXT_MESSAGE          14
#define CMD_TX_EXT_MESSAGE          15
#define CMD_SET_BUSPARAMS_REQ       16
#define CMD_GET_CHIP_STATE_REQ      19
#define CMD_CHIP_STATE_EVENT        20
#define CMD_SET_DRIVERMODE_REQ      21
#define CMD_RESET_CHIP_REQ          24
#define CMD_START_CHIP_REQ          26
#define CMD_START_CHIP_RESP         27
#define CMD_STOP_CHIP_REQ           28
#define CMD_STOP_CHIP_RESP          29
#define CMD_CLOCK_OVERFLOW_EVENT    33
#define CMD_GET_CARD_INFO_REQ       34
#define CMD_GET_CARD_INFO_RESP      35
#define CMD_GET_SOFTWARE_INFO_REQ   38
#define CMD_GET_SOFTWARE_INFO_RESP  39
#define CMD_TX_ACKNOWLEDGE          50
#define CMD_CAN_ERROR_EVENT         51

/* The helios timer runs at 1 MHz, so a tick is already a microsecond. */
#define HELIOS_TICKS_PER_US         1u

#pragma pack(push, 1)
struct helios_rx {
    uint8_t cmdLen, cmdNo, channel, flags;
    uint8_t raw[14];
    uint16_t time;
};
struct helios_tx {
    uint8_t cmdLen, cmdNo, channel, transId;
    uint8_t raw[14];
    uint8_t flags, pad;
};
struct helios_simple {
    uint8_t cmdLen, cmdNo, transId, channel;
};
struct helios_busparams {
    uint8_t cmdLen, cmdNo, transId, channel;
    uint32_t bitRate;
    uint8_t tseg1, tseg2, sjw, noSamp;
};
struct helios_drivermode {
    uint8_t cmdLen, cmdNo, transId, channel, driverMode, pad;
    uint16_t pad2;
};
struct helios_chip {
    uint8_t cmdLen, cmdNo, transId, channel;
    uint8_t txerr, rxerr;
    uint16_t time;
    uint8_t busStatus, pad;
    uint16_t pad2;
};
struct helios_err {
    uint8_t cmdLen, cmdNo, transId, pad;
    uint8_t txerr0, rxerr0, txerr1, rxerr1;
    uint8_t busStatus0, busStatus1;
    uint16_t time;
};
struct helios_card_info {
    uint8_t cmdLen, cmdNo, transId, channelCount;
    uint32_t serialLow, serialHigh, clockResolution, mfgDate;
    uint8_t EAN[8], hwRevision, hwType;
    uint16_t pad2;
};
struct helios_swinfo {
    uint8_t cmdLen, cmdNo, transId;
    uint8_t applicationName[5];
    uint16_t maxOutstandingTx;
    uint8_t padding[6];
    uint32_t applicationVersion;
    uint16_t checkSum;
    uint16_t swOptions;
};
struct helios_clock_ovf {
    uint8_t cmdLen, cmdNo, transId, pad;
    uint32_t currentTime;
};
struct helios_txack {
    uint8_t cmdLen, cmdNo, channel, transId;
    uint16_t time;
    uint16_t pad;
};
#pragma pack(pop)

/* The device sends only the low 16 bits with each frame; the high half
 * arrives separately and is remembered here. */
static uint64_t helios_ts(struct kv_device *d, uint16_t ticks)
{
    return ((uint64_t)d->helios_time_hi + ticks) / HELIOS_TICKS_PER_US;
}

/* Identifier packing is shared with filo: the SJA1000 layout. */
static void pack_id(uint8_t *raw, uint32_t id, int ext)
{
    if (ext) {
        raw[0] = (uint8_t)((id >> 24) & 0x1F);
        raw[1] = (uint8_t)((id >> 18) & 0x3F);
        raw[2] = (uint8_t)((id >> 14) & 0x0F);
        raw[3] = (uint8_t)((id >> 6) & 0xFF);
        raw[4] = (uint8_t)(id & 0x3F);
    } else {
        raw[0] = (uint8_t)((id >> 6) & 0x1F);
        raw[1] = (uint8_t)(id & 0x3F);
    }
}

static uint32_t unpack_id(const uint8_t *raw, int ext)
{
    uint32_t id = (raw[0] & 0x1F);
    id = (id << 6) + (raw[1] & 0x3F);
    if (!ext)
        return id;
    id = (id << 4) + (raw[2] & 0x0F);
    id = (id << 8) + raw[3];
    id = (id << 6) + (raw[4] & 0x3F);
    return id;
}

static uint8_t map_flags(uint8_t kflags, int ext)
{
    uint8_t f = 0;
    if (kflags & MSGFLAG_REMOTE_FRAME) f |= KV_FLAG_RTR;
    if (kflags & MSGFLAG_ERROR_FRAME)  f |= KV_FLAG_ERROR;
    if (kflags & MSGFLAG_TX)           f |= KV_FLAG_ECHO;
    if (ext)                           f |= KV_FLAG_EXTENDED;
    return f;
}

static void apply_chip(struct kv_device *d, int ch, uint8_t tec, uint8_t rec,
                       uint8_t bus)
{
    if (ch < 0 || ch >= d->can_count)
        return;
    d->ch[ch].tec = tec;
    d->ch[ch].rec = rec;
    d->ch[ch].bus_status = bus;
}

void kv_helios_handle(struct kv_device *d, const uint8_t *p, int len)
{
    uint8_t cmd = p[1];
    uint16_t seq = (len > 2) ? p[2] : 0;

    /* As in leaf.c: apply the payload first, wake the waiter last. */
    switch (cmd) {
    case CMD_RX_STD_MESSAGE:
    case CMD_RX_EXT_MESSAGE: {
        const struct helios_rx *rx = (const struct helios_rx *)p;
        int ext = (cmd == CMD_RX_EXT_MESSAGE);
        uint32_t id;
        uint8_t dlc, flags;
        int ch;
        if (len < (int)sizeof(*rx))
            return;
        ch = rx->channel;
        if (ch < 0 || ch >= d->can_count)
            return;
        id = unpack_id(rx->raw, ext);
        dlc = rx->raw[5] & 0x0F;
        flags = map_flags(rx->flags, ext);
        kv_push_frame(&d->ch[ch], id, flags, &rx->raw[6],
                      dlc > 8 ? 8 : dlc, dlc, helios_ts(d, rx->time));
        break;
    }
    case CMD_CHIP_STATE_EVENT: {
        const struct helios_chip *c = (const struct helios_chip *)p;
        if (len < (int)sizeof(*c))
            return;
        apply_chip(d, c->channel, c->txerr, c->rxerr, c->busStatus);
        break;
    }
    case CMD_CAN_ERROR_EVENT: {
        /* One record covers both channels, unlike filo's per-channel event. */
        const struct helios_err *e = (const struct helios_err *)p;
        int ch;
        if (len < (int)sizeof(*e))
            return;
        for (ch = 0; ch < d->can_count && ch < 2; ch++) {
            uint8_t tec = ch ? e->txerr1 : e->txerr0;
            uint8_t rec = ch ? e->rxerr1 : e->rxerr0;
            uint8_t bus = ch ? e->busStatus1 : e->busStatus0;
            uint8_t data[8];
            apply_chip(d, ch, tec, rec, bus);
            data[0] = rec;
            data[1] = tec;
            data[2] = bus;
            data[3] = 0;        /* helios reports no error factor */
            kv_push_frame(&d->ch[ch], 0, KV_FLAG_ERROR, data, 4, 4,
                          helios_ts(d, e->time));
        }
        break;
    }
    case CMD_CLOCK_OVERFLOW_EVENT: {
        /* Carries the full current time; keep its high half for the 16-bit
         * stamps that accompany ordinary frames. */
        const struct helios_clock_ovf *c = (const struct helios_clock_ovf *)p;
        if (len < (int)sizeof(*c))
            return;
        d->helios_time_hi = c->currentTime & 0xFFFF0000u;
        break;
    }
    case CMD_TX_ACKNOWLEDGE: {
        const struct helios_txack *a = (const struct helios_txack *)p;
        if (len < (int)sizeof(*a))
            return;
        if (a->channel < d->can_count && d->ch[a->channel].tx_outstanding)
            d->ch[a->channel].tx_outstanding--;
        break;
    }
    case CMD_GET_SOFTWARE_INFO_RESP: {
        const struct helios_swinfo *si = (const struct helios_swinfo *)p;
        if (len < (int)sizeof(*si))
            return;
        d->fw_major = (uint8_t)(si->applicationVersion >> 24);
        d->fw_minor = (uint8_t)(si->applicationVersion >> 16);
        d->fw_build = (uint16_t)(si->applicationVersion & 0xFFFF);
        if (si->maxOutstandingTx)
            d->max_outstanding_tx = si->maxOutstandingTx > 512
                                  ? 512 : si->maxOutstandingTx;
        break;
    }
    case CMD_GET_CARD_INFO_RESP: {
        const struct helios_card_info *ci = (const struct helios_card_info *)p;
        if (len < (int)sizeof(*ci))
            return;
        if (ci->channelCount >= 1 && ci->channelCount <= KV_MAX_CAN)
            d->can_count = ci->channelCount;
        d->serial = ci->serialLow;
        break;
    }
    default:
        break;
    }
    kv_maybe_complete_wait(d, cmd, seq, p, len);
}

static int helios_send(struct kv_device *d, void *cmd, int len)
{
    uint8_t *p = cmd;
    p[0] = (uint8_t)len;
    return kv_send_raw(d, cmd, len);
}

int kv_helios_init_card(struct kv_device *d)
{
    struct helios_simple req;
    uint8_t reply[32];
    int err;

    d->helios_time_hi = 0;
    /* The helios timer is 1 MHz, so ticks are microseconds already. */
    d->hires_mhz = 1;

    memset(&req, 0, sizeof(req));
    req.cmdNo = CMD_GET_SOFTWARE_INFO_REQ;
    req.transId = CMD_GET_SOFTWARE_INFO_REQ;
    req.cmdLen = sizeof(req);
    kv_arm_wait(d, CMD_GET_SOFTWARE_INFO_RESP, 0, 0);
    err = helios_send(d, &req, sizeof(req));
    if (err)
        return err;
    if (kv_wait_cmd(d, CMD_GET_SOFTWARE_INFO_RESP, 0, 0, reply, sizeof(reply),
                    KV_CMD_TIMEOUT))
        kv_log("helios: GET_SOFTWARE_INFO timed out (continuing)");

    memset(&req, 0, sizeof(req));
    req.cmdNo = CMD_GET_CARD_INFO_REQ;
    req.transId = CMD_GET_CARD_INFO_REQ;
    req.cmdLen = sizeof(req);
    kv_arm_wait(d, CMD_GET_CARD_INFO_RESP, 0, 0);
    err = helios_send(d, &req, sizeof(req));
    if (err)
        return err;
    if (kv_wait_cmd(d, CMD_GET_CARD_INFO_RESP, 0, 0, reply, sizeof(reply),
                    KV_CMD_TIMEOUT))
        kv_log("helios: GET_CARD_INFO timed out");
    return 0;
}

int kv_helios_bus_on(struct kv_channel *ch, const kv_open_opts *opts)
{
    struct kv_device *d = ch->dev;
    struct helios_busparams bp;
    struct helios_drivermode dm;
    struct helios_simple st;
    uint8_t reply[32];
    uint8_t tseg1, tseg2, sjw;
    int err, bitrate;

    bitrate = opts && opts->bitrate > 0 ? opts->bitrate : 500000;
    /* These adapters clock the CAN core at 16 MHz, as the Leaf family does. */
    kv_resolve_timing(bitrate, opts, 0, 16000000, &tseg1, &tseg2, &sjw);

    memset(&dm, 0, sizeof(dm));
    dm.cmdNo = CMD_SET_DRIVERMODE_REQ;
    dm.channel = (uint8_t)ch->idx;
    dm.transId = (uint8_t)ch->idx;
    dm.driverMode = (opts && opts->listen_only) ? DRIVERMODE_SILENT
                                                : DRIVERMODE_NORMAL;
    dm.cmdLen = sizeof(dm);
    err = helios_send(d, &dm, sizeof(dm));
    if (err)
        return err;

    memset(&bp, 0, sizeof(bp));
    bp.cmdNo = CMD_SET_BUSPARAMS_REQ;
    bp.channel = (uint8_t)ch->idx;
    bp.transId = (uint8_t)ch->idx;
    bp.bitRate = (uint32_t)bitrate;
    bp.tseg1 = tseg1;
    bp.tseg2 = tseg2;
    bp.sjw = sjw;
    bp.noSamp = 1;
    bp.cmdLen = sizeof(bp);
    err = helios_send(d, &bp, sizeof(bp));
    if (err)
        return err;

    memset(&st, 0, sizeof(st));
    st.cmdNo = CMD_START_CHIP_REQ;
    st.channel = (uint8_t)ch->idx;
    st.transId = (uint8_t)ch->idx;
    st.cmdLen = sizeof(st);
    kv_arm_wait(d, CMD_START_CHIP_RESP, st.transId, 1);
    err = helios_send(d, &st, sizeof(st));
    if (err)
        return err;
    (void)kv_wait_cmd(d, CMD_START_CHIP_RESP, st.transId, 1, reply,
                      sizeof(reply), KV_CMD_TIMEOUT);

    memset(&st, 0, sizeof(st));
    st.cmdNo = CMD_GET_CHIP_STATE_REQ;
    st.channel = (uint8_t)ch->idx;
    st.transId = (uint8_t)ch->idx;
    st.cmdLen = sizeof(st);
    (void)helios_send(d, &st, sizeof(st));

    ch->open = 1;
    ch->tx_outstanding = 0;
    ch->tx_window = d->max_outstanding_tx ? d->max_outstanding_tx : 16;
    ch->listen_only = (uint8_t)(opts && opts->listen_only);
    ch->fd_mode = 0;
    return 0;
}

int kv_helios_bus_off(struct kv_channel *ch)
{
    struct helios_simple st;
    uint8_t reply[32];

    memset(&st, 0, sizeof(st));
    st.cmdNo = CMD_STOP_CHIP_REQ;
    st.channel = (uint8_t)ch->idx;
    st.transId = (uint8_t)ch->idx;
    st.cmdLen = sizeof(st);
    kv_arm_wait(ch->dev, CMD_STOP_CHIP_RESP, st.transId, 1);
    (void)helios_send(ch->dev, &st, sizeof(st));
    (void)kv_wait_cmd(ch->dev, CMD_STOP_CHIP_RESP, st.transId, 1, reply,
                      sizeof(reply), 500);
    ch->open = 0;
    return 0;
}

int kv_helios_write(struct kv_channel *ch, const kv_msg *msg)
{
    struct helios_tx tx;
    int ext = (msg->flags & KV_FLAG_EXTENDED) != 0;
    int n = msg->len > 8 ? 8 : msg->len;
    int err;

    memset(&tx, 0, sizeof(tx));
    tx.cmdNo = ext ? CMD_TX_EXT_MESSAGE : CMD_TX_STD_MESSAGE;
    tx.channel = (uint8_t)ch->idx;
    tx.transId = kv_next_transid(ch);
    pack_id(tx.raw, msg->id, ext);
    tx.raw[5] = (uint8_t)(n & 0x0F);
    memcpy(&tx.raw[6], msg->data, (size_t)n);
    /* helios puts the flags byte at offset 18, where filo has padding. */
    if (msg->flags & KV_FLAG_RTR)
        tx.flags |= MSGFLAG_REMOTE_FRAME;
    tx.cmdLen = sizeof(tx);
    err = helios_send(ch->dev, &tx, sizeof(tx));
    if (!err)
        ch->tx_outstanding++;
    return err;
}
