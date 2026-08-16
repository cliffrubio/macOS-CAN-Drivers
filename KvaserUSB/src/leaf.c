/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 *
 * "filo" command protocol, used by the Leaf family and by USBcan II /
 * Helios adapters. Ported from linuxcan leaf/ and usbcanII/.
 * Classic CAN only. CAN FD adapters speak the hydra protocol instead.
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
#define CMD_GET_CARD_INFO_REQ       34
#define CMD_GET_CARD_INFO_RESP      35
#define CMD_GET_SOFTWARE_INFO_REQ   38
#define CMD_GET_SOFTWARE_INFO_RESP  39
#define CMD_CAN_ERROR_EVENT         51
#define CMD_LOG_MESSAGE             106

#pragma pack(push, 1)
struct filo_tx {
    uint8_t cmdLen, cmdNo, channel, transId;
    uint8_t raw[14];
    uint8_t pad, flags;
};
struct filo_simple {
    uint8_t cmdLen, cmdNo, transId, channel;
};
struct filo_busparams {
    uint8_t cmdLen, cmdNo, transId, channel;
    uint32_t bitRate;
    uint8_t tseg1, tseg2, sjw, noSamp;
};
struct filo_drivermode {
    uint8_t cmdLen, cmdNo, transId, channel, driverMode, pad;
    uint16_t pad2;
};
struct filo_card_info {
    uint8_t cmdLen, cmdNo, transId, channelCount;
    uint32_t serialNumber;
    uint32_t padding1, clockResolution, mfgDate;
    uint8_t EAN[8], hwRevision, usbHsMode, hwType, canTimeStampRef;
};
struct filo_log {
    uint8_t cmdLen, cmdNo, channel, flags;
    uint16_t time[3];
    uint8_t dlc, timeOffset;
    uint32_t id;
    uint8_t data[8];
};
struct filo_rx {
    uint8_t cmdLen, cmdNo, channel, flags;
    uint16_t time[3];
    uint8_t raw[14];
};
struct filo_chip {
    uint8_t cmdLen, cmdNo, transId, channel;
    uint16_t time[3];
    uint8_t txerr, rxerr, busStatus, pad;
    uint16_t pad2;
};
struct filo_err {
    uint8_t cmdLen, cmdNo, transId, flags;
    uint16_t time[3];
    uint8_t channel, pad, txerr, rxerr, busStatus, errorFactor;
};
struct filo_swinfo {
    uint8_t cmdLen, cmdNo, transId, padding0;
    uint32_t swOptions;
    uint32_t firmwareVersion;
    uint16_t maxOutstandingTx;
};
#pragma pack(pop)

/* swOptions bits 0x60 select the high-resolution timer clock. */
#define FILO_CPU_FQ_MASK    0x60u
#define FILO_16_MHZ_CLK     0x00u
#define FILO_32_MHZ_CLK     0x20u
#define FILO_24_MHZ_CLK     0x40u

static uint64_t ticks48(const uint16_t t[3])
{
    return (uint64_t)t[0] | ((uint64_t)t[1] << 16) | ((uint64_t)t[2] << 32);
}

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

static void apply_chip(struct kv_device *d, int ch, uint8_t tec, uint8_t rec, uint8_t bus)
{
    if (ch < 0 || ch >= d->can_count)
        return;
    d->ch[ch].tec = tec;
    d->ch[ch].rec = rec;
    d->ch[ch].bus_status = bus;
}

void kv_leaf_handle(struct kv_device *d, const uint8_t *p, int len)
{
    uint8_t cmd = p[1];
    uint16_t seq = (len > 2) ? p[2] : 0;

    /* Apply the payload first, wake the waiter last, so a caller returning
     * from kv_wait_cmd() sees the fields the reply carried. */
    switch (cmd) {
    case CMD_RX_STD_MESSAGE:
    case CMD_RX_EXT_MESSAGE: {
        const struct filo_rx *rx = (const struct filo_rx *)p;
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
        kv_push_frame(&d->ch[ch], id, flags, &rx->raw[6], dlc > 8 ? 8 : dlc, dlc,
                      kv_ticks_to_us(d, ticks48(rx->time)));
        break;
    }
    case CMD_LOG_MESSAGE: {
        const struct filo_log *lg = (const struct filo_log *)p;
        uint32_t id;
        uint8_t flags;
        int ch, ext;
        if (len < (int)sizeof(*lg))
            return;
        ch = lg->channel;
        if (ch < 0 || ch >= d->can_count)
            return;
        id = lg->id & 0x1FFFFFFFu;
        ext = (lg->id & KV_EXT_MSG) || (lg->flags & MSGFLAG_EXTENDED_ID);
        flags = map_flags(lg->flags, ext);
        kv_push_frame(&d->ch[ch], id, flags, lg->data, lg->dlc > 8 ? 8 : lg->dlc, lg->dlc,
                      kv_ticks_to_us(d, ticks48(lg->time)));
        break;
    }
    case CMD_CHIP_STATE_EVENT: {
        const struct filo_chip *c = (const struct filo_chip *)p;
        if (len < (int)sizeof(*c))
            return;
        apply_chip(d, c->channel, c->txerr, c->rxerr, c->busStatus);
        break;
    }
    case CMD_CAN_ERROR_EVENT: {
        const struct filo_err *e = (const struct filo_err *)p;
        uint8_t data[8];
        if (len < (int)sizeof(*e))
            return;
        data[0] = e->rxerr;
        data[1] = e->txerr;
        data[2] = e->busStatus;
        data[3] = e->errorFactor;
        apply_chip(d, e->channel, e->txerr, e->rxerr, e->busStatus);
        if (e->channel < d->can_count)
            kv_push_frame(&d->ch[e->channel], 0, KV_FLAG_ERROR, data, 4, 4,
                          kv_ticks_to_us(d, ticks48(e->time)));
        break;
    }
    case CMD_GET_SOFTWARE_INFO_RESP: {
        const struct filo_swinfo *si = (const struct filo_swinfo *)p;
        uint32_t fq;
        if (len < (int)sizeof(*si))
            return;
        fq = si->swOptions & FILO_CPU_FQ_MASK;
        d->hires_mhz = (fq == FILO_32_MHZ_CLK) ? 32 :
                       (fq == FILO_24_MHZ_CLK) ? 24 : 16;
        d->fw_major = (uint8_t)(si->firmwareVersion >> 24);
        d->fw_minor = (uint8_t)(si->firmwareVersion >> 16);
        d->fw_build = (uint16_t)(si->firmwareVersion & 0xFFFF);
        /* How many frames the adapter will hold before it starts discarding
         * them silently. Capped, because a firmware reporting something
         * implausible should not disable the window entirely. */
        if (si->maxOutstandingTx)
            d->max_outstanding_tx = si->maxOutstandingTx > 512
                                  ? 512 : si->maxOutstandingTx;
        break;
    }
    case CMD_GET_CARD_INFO_RESP: {
        const struct filo_card_info *ci = (const struct filo_card_info *)p;
        if (len < (int)sizeof(*ci))
            return;
        if (ci->channelCount >= 1 && ci->channelCount <= KV_MAX_CAN)
            d->can_count = ci->channelCount;
        d->serial = ci->serialNumber;
        break;
    }
    default:
        break;
    }
    kv_maybe_complete_wait(d, cmd, seq, p, len);
}

static int leaf_send(struct kv_device *d, void *cmd, int len)
{
    uint8_t *p = cmd;
    p[0] = (uint8_t)len;
    return kv_send_raw(d, cmd, len);
}

int kv_leaf_init_card(struct kv_device *d)
{
    struct filo_simple req;
    uint8_t reply[32];
    int err;

    /* 16 MHz is the filo default until GET_SOFTWARE_INFO says otherwise. */
    d->hires_mhz = 16;

    memset(&req, 0, sizeof(req));
    req.cmdNo = CMD_GET_SOFTWARE_INFO_REQ;
    req.transId = CMD_GET_SOFTWARE_INFO_REQ;
    req.cmdLen = sizeof(req);
    err = leaf_send(d, &req, sizeof(req));
    if (err)
        return err;
    err = kv_wait_cmd(d, CMD_GET_SOFTWARE_INFO_RESP, 0, 0, reply, sizeof(reply), KV_CMD_TIMEOUT);
    if (err)
        kv_log("leaf: GET_SOFTWARE_INFO timed out (continuing)");

    memset(&req, 0, sizeof(req));
    req.cmdNo = CMD_GET_CARD_INFO_REQ;
    req.transId = CMD_GET_CARD_INFO_REQ;
    req.cmdLen = sizeof(req);
    err = leaf_send(d, &req, sizeof(req));
    if (err)
        return err;
    err = kv_wait_cmd(d, CMD_GET_CARD_INFO_RESP, 0, 0, reply, sizeof(reply), KV_CMD_TIMEOUT);
    if (err)
        kv_log("leaf: GET_CARD_INFO timed out");
    return 0;
}

int kv_leaf_bus_on(struct kv_channel *ch, const kv_open_opts *opts)
{
    struct kv_device *d = ch->dev;
    struct filo_busparams bp;
    struct filo_drivermode dm;
    struct filo_simple st;
    uint8_t reply[32];
    uint8_t tseg1, tseg2, sjw;
    int err, bitrate;

    bitrate = opts && opts->bitrate > 0 ? opts->bitrate : 500000;
    /* filo controllers clock the CAN core at 16 MHz. */
    kv_resolve_timing(bitrate, opts, 0, 16000000, &tseg1, &tseg2, &sjw);

    memset(&dm, 0, sizeof(dm));
    dm.cmdNo = CMD_SET_DRIVERMODE_REQ;
    dm.channel = (uint8_t)ch->idx;
    dm.transId = (uint8_t)ch->idx;
    dm.driverMode = (opts && opts->listen_only) ? DRIVERMODE_SILENT : DRIVERMODE_NORMAL;
    dm.cmdLen = sizeof(dm);
    err = leaf_send(d, &dm, sizeof(dm));
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
    err = leaf_send(d, &bp, sizeof(bp));
    if (err)
        return err;

    memset(&st, 0, sizeof(st));
    st.cmdNo = CMD_START_CHIP_REQ;
    st.channel = (uint8_t)ch->idx;
    st.transId = (uint8_t)ch->idx;
    st.cmdLen = sizeof(st);
    err = leaf_send(d, &st, sizeof(st));
    if (err)
        return err;
    (void)kv_wait_cmd(d, CMD_START_CHIP_RESP, st.transId, 1, reply, sizeof(reply), KV_CMD_TIMEOUT);

    memset(&st, 0, sizeof(st));
    st.cmdNo = CMD_GET_CHIP_STATE_REQ;
    st.channel = (uint8_t)ch->idx;
    st.transId = (uint8_t)ch->idx;
    st.cmdLen = sizeof(st);
    (void)leaf_send(d, &st, sizeof(st));
    ch->open = 1;
    ch->tx_outstanding = 0;
    ch->tx_window = d->max_outstanding_tx ? d->max_outstanding_tx : 16;
    ch->listen_only = (uint8_t)(opts && opts->listen_only);
    ch->fd_mode = 0;
    return 0;
}

int kv_leaf_bus_off(struct kv_channel *ch)
{
    struct filo_simple st;
    uint8_t reply[32];
    memset(&st, 0, sizeof(st));
    st.cmdNo = CMD_STOP_CHIP_REQ;
    st.channel = (uint8_t)ch->idx;
    st.transId = (uint8_t)ch->idx;
    st.cmdLen = sizeof(st);
    (void)leaf_send(ch->dev, &st, sizeof(st));
    (void)kv_wait_cmd(ch->dev, CMD_STOP_CHIP_RESP, st.transId, 1, reply, sizeof(reply), 500);
    ch->open = 0;
    return 0;
}

int kv_leaf_write(struct kv_channel *ch, const kv_msg *msg)
{
    struct filo_tx tx;
    int ext = (msg->flags & KV_FLAG_EXTENDED) != 0;
    int n = msg->len > 8 ? 8 : msg->len;

    memset(&tx, 0, sizeof(tx));
    tx.cmdNo = ext ? CMD_TX_EXT_MESSAGE : CMD_TX_STD_MESSAGE;
    tx.channel = (uint8_t)ch->idx;
    tx.transId = kv_next_transid(ch);
    pack_id(tx.raw, msg->id, ext);
    tx.raw[5] = (uint8_t)(n & 0x0F);
    memcpy(&tx.raw[6], msg->data, (size_t)n);
    if (msg->flags & KV_FLAG_RTR)
        tx.flags |= MSGFLAG_REMOTE_FRAME;
    tx.cmdLen = sizeof(tx);
    return leaf_send(ch->dev, &tx, sizeof(tx));
}
