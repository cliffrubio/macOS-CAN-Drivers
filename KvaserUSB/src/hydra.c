/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 *
 * "hydra" command protocol, used by every CAN FD capable Kvaser adapter
 * (Leaf Pro/v3, USBcan Pro v2, U100, Memorator v2, ...). Ported from
 * linuxcan mhydra/.
 *
 * Hydra devices route commands to "hardware entities" (HEs) rather than to
 * channel numbers, so kv_hydra_init_card() first asks the router to map each
 * CAN channel to an HE and remembers both directions of that mapping.
 */
#include "kvaser_priv.h"

#include <string.h>

#define CMD_TX_CAN_MESSAGE              33
#define CMD_SET_BUSPARAMS_REQ           16
#define CMD_SET_BUSPARAMS_RESP          85
#define CMD_GET_CHIP_STATE_REQ          19
#define CMD_CHIP_STATE_EVENT            20
#define CMD_SET_DRIVERMODE_REQ          21
#define CMD_START_CHIP_REQ              26
#define CMD_START_CHIP_RESP             27
#define CMD_STOP_CHIP_REQ               28
#define CMD_STOP_CHIP_RESP              29
#define CMD_GET_CARD_INFO_REQ           34
#define CMD_GET_CARD_INFO_RESP          35
#define CMD_GET_SOFTWARE_INFO_REQ       38
#define CMD_GET_SOFTWARE_INFO_RESP      39
#define CMD_CAN_ERROR_EVENT             51
#define CMD_SET_BUSPARAMS_FD_REQ        69
#define CMD_SET_BUSPARAMS_FD_RESP       70
#define CMD_LOG_MESSAGE                 106
#define CMD_MAP_CHANNEL_REQ             200
#define CMD_MAP_CHANNEL_RESP            201
#define CMD_GET_SOFTWARE_DETAILS_REQ    202
#define CMD_GET_SOFTWARE_DETAILS_RESP   203
#define CMD_TX_CAN_MESSAGE_FD           224
#define CMD_RX_MESSAGE_FD               226
#define CMD_GET_BUSPARAMS_REQ           17
#define CMD_GET_BUSPARAMS_RESP          18
#define CMD_GET_BUSLOAD_REQ             40
#define CMD_GET_BUSLOAD_RESP            41
#define CMD_ERROR_EVENT                 45
#define CMD_FLUSH_QUEUE                 48
#define CMD_FLUSH_QUEUE_RESP            66
#define CMD_TX_ACKNOWLEDGE              50
#define CMD_GET_CARD_INFO_2             32
#define CMD_CHECK_LICENSE_REQ           43
#define CMD_CHECK_LICENSE_RESP          44
#define CMD_HYDRA_TX_INTERVAL_REQ       67
#define CMD_HYDRA_TX_INTERVAL_RESP      68
#define CMD_AUTO_TX_BUFFER_REQ          72
#define CMD_AUTO_TX_BUFFER_RESP         73
#define CMD_SET_IO_PORTS_REQ            86
#define CMD_GET_IO_PORTS_REQ            87
#define CMD_GET_IO_PORTS_RESP           88
#define CMD_GET_DRIVERMODE_REQ          22
#define CMD_GET_DRIVERMODE_RESP         23
#define CMD_FATAL_ERROR                 137
#define CMD_GET_INTERFACE_INFO_REQ      36
#define CMD_GET_INTERFACE_INFO_RESP     37
#define CMD_GET_BUSPARAMS_TQ_REQ        71
#define CMD_GET_BUSPARAMS_TQ_RESP       100
#define CMD_SOUND                       78
#define CMD_GET_TRANSCEIVER_INFO_REQ    97
#define CMD_GET_TRANSCEIVER_INFO_RESP   98
#define CMD_SET_DEVICE_MODE             204
#define CMD_GET_DEVICE_MODE             205
#define CMD_GET_CAPABILITIES_REQ        95
#define CMD_GET_CAPABILITIES_RESP       96
#define CMD_EXTENDED                    255

/* AUTO_TX_BUFFER request types, from linuxcan's hydra command set. */
#define AUTOTXBUFFER_CMD_GET_INFO       1
#define AUTOTXBUFFER_CMD_CLEAR_ALL      2
#define AUTOTXBUFFER_CMD_ACTIVATE       3
#define AUTOTXBUFFER_CMD_DEACTIVATE     4
#define AUTOTXBUFFER_CMD_SET_INTERVAL   5
#define AUTOTXBUFFER_CMD_SET_MSG_COUNT  7

/* CMD_GET_CAPABILITIES sub-commands, and the driver-facing bit each maps to. */
#define CAP_SUB_CMD_SILENT_MODE         6
#define CAP_SUB_CMD_ERRFRAME            7
#define CAP_SUB_CMD_BUS_STATS           8
#define CAP_SUB_CMD_ERRCOUNT_READ       9
#define CAP_SUB_CMD_SINGLE_SHOT         10
#define CAP_STATUS_OK                   0

#define TRANSID_CAN                     0x40
#define SWOPTION_USE_HYDRA_EXT          0x200
#define OPEN_AS_CAN                     0
#define OPEN_AS_CANFD_ISO               1

/* swOptions bits 0x60 select the high-resolution timer clock. */
#define HYDRA_CPU_FQ_MASK               0x60u
#define HYDRA_80_MHZ_CLK                0x20u
#define HYDRA_24_MHZ_CLK                0x40u
/* swOptions bits 0x180 select the CAN controller base clock. */
#define HYDRA_CAN_CLK_MASK              0x180u
#define HYDRA_80_MHZ_CAN_CLK            0x080u
#define HYDRA_24_MHZ_CAN_CLK            0x100u

#define _ones(n) ((1UL << (n)) - 1)
#define RTPACKET_SRR(v)  ((uint32_t)(v) << 31)
#define RTPACKET_IDE(v)  ((uint32_t)(v) << 30)
#define RTPACKET_RTR(v)  ((uint32_t)(v) << 29)
#define RTPACKET_ID(v)   ((uint32_t)(v) & _ones(29))
#define TPACKET_AREQ(v)  ((uint32_t)(v) << 31)
#define RTPACKET_FDF(v)  ((uint32_t)(v) << 15)
#define RTPACKET_BRS(v)  ((uint32_t)(v) << 14)
#define RTPACKET_DLC(v)  (((uint32_t)(v) & 0xF) << 8)
#define RTPACKET_DLC_GET(v) (((v) >> 8) & 0xF)

void kv_hydra_set_dst(uint8_t *cmd, uint8_t he)
{
    cmd[1] = (uint8_t)((cmd[1] & 0xC0) | (he & 0x3F));
}

void kv_hydra_set_seq(uint8_t *cmd, uint16_t seq)
{
    uint16_t t = (uint16_t)(cmd[2] | (cmd[3] << 8));
    t = (uint16_t)((t & 0xF000) | (seq & 0x0FFF));
    cmd[2] = (uint8_t)(t & 0xFF);
    cmd[3] = (uint8_t)(t >> 8);
}

uint16_t kv_hydra_seq(const uint8_t *cmd)
{
    return (uint16_t)((cmd[2] | (cmd[3] << 8)) & 0x0FFF);
}

uint8_t kv_hydra_src_he(const uint8_t *cmd)
{
    uint16_t trans = (uint16_t)(cmd[2] | (cmd[3] << 8));
    return (uint8_t)(((cmd[1] & 0xC0) >> 2) | (trans >> 12));
}

static void hydra_zero(uint8_t cmd[KV_HYDRA_CMD])
{
    memset(cmd, 0, KV_HYDRA_CMD);
}

static int hydra_send(struct kv_device *d, uint8_t cmd[KV_HYDRA_CMD])
{
    return kv_send_raw(d, cmd, KV_HYDRA_CMD);
}

static uint8_t map_flags32(uint32_t kflags, int ext)
{
    uint8_t f = 0;
    if (kflags & MSGFLAG_REMOTE_FRAME) f |= KV_FLAG_RTR;
    if (kflags & MSGFLAG_ERROR_FRAME)  f |= KV_FLAG_ERROR;
    if (kflags & MSGFLAG_TX)           f |= KV_FLAG_ECHO;
    if (kflags & MSGFLAG_FDF)          f |= KV_FLAG_FD;
    if (kflags & MSGFLAG_BRS)          f |= KV_FLAG_BRS;
    if (kflags & MSGFLAG_ESI)          f |= KV_FLAG_ESI;
    if (ext || (kflags & MSGFLAG_EXTENDED_ID)) f |= KV_FLAG_EXTENDED;
    return f;
}

static int chan_from_he(struct kv_device *d, const uint8_t *cmd)
{
    uint8_t he = kv_hydra_src_he(cmd);
    if (he < 64 && d->he2channel[he] < KV_MAX_CAN)
        return d->he2channel[he];
    return -1;
}

static void apply_chip(struct kv_device *d, int ch, uint8_t tec, uint8_t rec, uint8_t bus)
{
    if (ch < 0 || ch >= d->can_count)
        return;
    d->ch[ch].tec = tec;
    d->ch[ch].rec = rec;
    d->ch[ch].bus_status = bus;
}

/* The payload is applied to the device struct first and the waiter is woken
 * last: a caller that returns from kv_wait_cmd() must be able to rely on the
 * fields that reply carried, such as the HE mapping or the clock settings. */
void kv_hydra_handle(struct kv_device *d, const uint8_t *p, int len)
{
    uint8_t cmd = p[0];
    uint16_t seq = kv_hydra_seq(p);

    if (cmd == CMD_EXTENDED && len >= 8) {
        uint8_t ext = p[6];
        if (ext == CMD_RX_MESSAGE_FD && len >= 32) {
            uint32_t flags, id, ctrl;
            uint64_t ticks;
            int ch = chan_from_he(d, p);
            int nbytes, fd;
            uint8_t dlc, outflags;
            memcpy(&flags, p + 8, 4);
            memcpy(&id, p + 12, 4);
            memcpy(&ctrl, p + 20, 4);
            memcpy(&ticks, p + 24, 8);
            dlc = (uint8_t)RTPACKET_DLC_GET(ctrl);
            fd = (flags & MSGFLAG_FDF) != 0;
            nbytes = kv_dlc_to_len(dlc, fd);
            if (nbytes > 64) nbytes = 64;
            /* The DLC is the sender's claim; the command length is the truth.
             * Never read payload the firmware did not actually send. */
            if (nbytes > len - 32) nbytes = len - 32;
            outflags = map_flags32(flags, (flags & MSGFLAG_EXTENDED_ID) != 0);
            if (ch >= 0)
                kv_push_frame(&d->ch[ch], id & 0x1FFFFFFFu, outflags,
                              p + 32, nbytes, dlc, kv_ticks_to_us(d, ticks));
        }
        kv_maybe_complete_wait(d, p[6], seq, p, len);
        return;
    }

    switch (cmd) {
    case CMD_MAP_CHANNEL_RESP: {
        uint8_t he = p[4];
        if ((seq & 0xFF0) == TRANSID_CAN) {
            int chan = seq & 0x00F;
            if (chan < KV_MAX_CAN && he != ILLEGAL_HE) {
                d->channel2he[chan] = he;
                if (he < 64)
                    d->he2channel[he] = (uint8_t)chan;
            }
        }
        break;
    }
    case CMD_GET_CARD_INFO_RESP: {
        uint32_t serial = 0;
        uint8_t nch = p[4 + 24];
        memcpy(&serial, p + 4, 4);
        d->serial = serial;
        if (nch >= 1 && nch <= KV_MAX_CAN)
            d->can_count = nch;
        break;
    }
    case CMD_GET_SOFTWARE_DETAILS_RESP: {
        uint32_t opt = 0, ver = 0, fq;
        memcpy(&opt, p + 4, 4);
        memcpy(&ver, p + 8, 4);
        d->hydra_ext = (opt & SWOPTION_USE_HYDRA_EXT) ? 1 : 0;
        fq = opt & HYDRA_CPU_FQ_MASK;
        d->hires_mhz = (fq == HYDRA_80_MHZ_CLK) ? 80 :
                       (fq == HYDRA_24_MHZ_CLK) ? 24 : 1;
        {
            uint32_t cq = opt & HYDRA_CAN_CLK_MASK;
            d->can_clk_mhz = (cq == HYDRA_24_MHZ_CAN_CLK) ? 24 : 80;
        }
        d->fw_major = (uint8_t)(ver >> 24);
        d->fw_minor = (uint8_t)(ver >> 16);
        d->fw_build = (uint16_t)(ver & 0xFFFF);
        break;
    }
    case CMD_LOG_MESSAGE: {
        /* linuxcan overlay: payload starts with cmdLen, cmdNo, channel, flags,
         * then a 48-bit tick counter as three little-endian 16-bit words. */
        uint8_t flags8 = p[7];
        uint8_t dlc = p[14];
        uint32_t id = 0;
        uint64_t ticks;
        int ch = chan_from_he(d, p);
        uint8_t outflags;
        memcpy(&id, p + 16, 4);
        ticks = (uint64_t)p[8] | ((uint64_t)p[9] << 8) |
                ((uint64_t)p[10] << 16) | ((uint64_t)p[11] << 24) |
                ((uint64_t)p[12] << 32) | ((uint64_t)p[13] << 40);
        if (ch < 0)
            ch = p[6];
        outflags = map_flags32(flags8, (id & KV_EXT_MSG) != 0 || (flags8 & MSGFLAG_EXTENDED_ID));
        if (ch >= 0 && ch < d->can_count)
            kv_push_frame(&d->ch[ch], id & 0x1FFFFFFFu, outflags, p + 20,
                          dlc > 8 ? 8 : dlc, dlc, kv_ticks_to_us(d, ticks));
        break;
    }
    case CMD_CAN_ERROR_EVENT: {
        /* Payload: time[3], flags, reserved, txerr, rxerr, busStatus,
         * errorFactor. Same data[] layout as the filo error frame. */
        uint64_t ticks = (uint64_t)p[4] | ((uint64_t)p[5] << 8) |
                         ((uint64_t)p[6] << 16) | ((uint64_t)p[7] << 24) |
                         ((uint64_t)p[8] << 32) | ((uint64_t)p[9] << 40);
        uint8_t data[8] = { p[13], p[12], p[14], p[15] };
        int ch = chan_from_he(d, p);
        apply_chip(d, ch, p[12], p[13], p[14]);
        if (ch >= 0 && ch < d->can_count)
            kv_push_frame(&d->ch[ch], 0, KV_FLAG_ERROR, data, 4, 4,
                          kv_ticks_to_us(d, ticks));
        break;
    }
    case CMD_CHIP_STATE_EVENT: {
        int ch = chan_from_he(d, p);
        apply_chip(d, ch, p[10], p[11], p[12]);
        break;
    }
    case CMD_TX_ACKNOWLEDGE: {
        /* The adapter has finished with one frame, so the window reopens.
         * Without this the outstanding count only ever climbs and transmit
         * would wedge after tx_window frames. */
        int ch = chan_from_he(d, p);
        if (ch >= 0 && ch < d->can_count && d->ch[ch].tx_outstanding)
            d->ch[ch].tx_outstanding--;
        break;
    }
    case CMD_GET_BUSLOAD_RESP: {
        /* active_samples out of sample_interval, as tenths of a percent. */
        int ch = chan_from_he(d, p);
        uint16_t interval, active;
        if (len < 16 || ch < 0 || ch >= d->can_count)
            break;
        interval = (uint16_t)(p[10] | (p[11] << 8));
        active   = (uint16_t)(p[12] | (p[13] << 8));
        if (interval) {
            uint32_t pm = (uint32_t)active * 1000u / interval;
            d->ch[ch].busload_per_mille = (uint16_t)(pm > 1000 ? 1000 : pm);
            d->ch[ch].busload_valid = 1;
        }
        break;
    }
    case CMD_GET_BUSPARAMS_RESP: {
        int ch = chan_from_he(d, p);
        if (len < 12 || ch < 0 || ch >= d->can_count)
            break;
        memcpy(&d->ch[ch].bp_bitrate, p + 4, 4);
        d->ch[ch].bp_tseg1 = p[8];
        d->ch[ch].bp_tseg2 = p[9];
        d->ch[ch].bp_sjw   = p[10];
        d->ch[ch].bp_valid = 1;
        break;
    }
    case CMD_GET_CAPABILITIES_RESP: {
        /* subCmdNo at +4, status at +6, the value at +8. Only a subset is
         * mapped; the rest concern logging, scripting and remote features
         * this driver does not expose. */
        uint16_t sub, status;
        uint32_t val = 0;
        if (len < 12)
            break;
        sub    = (uint16_t)(p[4] | (p[5] << 8));
        status = (uint16_t)(p[6] | (p[7] << 8));
        memcpy(&val, p + 8, 4);
        if (status != CAP_STATUS_OK || !val)
            break;
        switch (sub) {
        case CAP_SUB_CMD_SILENT_MODE:   d->caps |= KV_CAP_SILENT; break;
        case CAP_SUB_CMD_ERRFRAME:      d->caps |= KV_CAP_ERR_FRAMES; break;
        case CAP_SUB_CMD_BUS_STATS:     d->caps |= KV_CAP_BUS_STATS; break;
        case CAP_SUB_CMD_ERRCOUNT_READ: d->caps |= KV_CAP_ERR_COUNTERS; break;
        case CAP_SUB_CMD_SINGLE_SHOT:   d->caps |= KV_CAP_SINGLE_SHOT; break;
        default: break;
        }
        d->caps_valid = 1;
        break;
    }
    case CMD_AUTO_TX_BUFFER_RESP: {
        /* bufferCount at +5 tells us how many periodic slots exist. */
        if (len < 12)
            break;
        d->autotx_buffers = p[5];
        d->autotx_valid = 1;
        break;
    }
    case CMD_CHECK_LICENSE_RESP: {
        if (len < 12)
            break;
        memcpy(&d->license_mask, p + 4, 4);
        memcpy(&d->kvaser_license_mask, p + 8, 4);
        d->license_valid = 1;
        break;
    }
    case CMD_GET_IO_PORTS_RESP: {
        /* portVal at +4, then the echoed request, port number and status. */
        if (len < 22)
            break;
        memcpy(&d->io_port_val, p + 4, 4);
        d->io_port_status = p[21];
        d->io_port_valid = 1;
        break;
    }
    case CMD_GET_DRIVERMODE_RESP: {
        int ch = chan_from_he(d, p);
        if (len < 6 || ch < 0 || ch >= d->can_count)
            break;
        d->ch[ch].reported_mode = p[4];
        d->ch[ch].mode_valid = 1;
        break;
    }
    case CMD_FATAL_ERROR: {
        /* The firmware has given up. Surface it rather than letting the
         * channel go quiet with no explanation. */
        int ch = chan_from_he(d, p);
        uint8_t data[8];
        if (len < 12)
            break;
        memset(data, 0, sizeof(data));
        data[0] = p[11];
        kv_log("fatal firmware error, code %u", p[11]);
        if (ch < 0)
            ch = 0;
        if (ch < d->can_count)
            kv_push_frame(&d->ch[ch], 0, KV_FLAG_ERROR | KV_FLAG_STATUS,
                          data, 1, 1, 0);
        break;
    }
    case CMD_GET_TRANSCEIVER_INFO_RESP: {
        if (len < 11)
            break;
        memcpy(&d->xcvr_caps, p + 4, 4);
        d->xcvr_status = p[8];
        d->xcvr_type   = p[9];
        d->xcvr_valid  = 1;
        break;
    }
    case CMD_GET_INTERFACE_INFO_RESP: {
        if (len < 11)
            break;
        memcpy(&d->iface_caps, p + 4, 4);
        d->chip_type    = p[8];
        d->chip_subtype = p[9];
        d->iface_valid  = 1;
        break;
    }
    case CMD_GET_CARD_INFO_2: {
        /* 24 bytes of PCB identifier, not guaranteed to be terminated. */
        if (len < 28)
            break;
        memcpy(d->pcb_id, p + 4, 24);
        d->pcb_id[24] = '\0';
        d->pcb_valid = 1;
        break;
    }
    case CMD_GET_DEVICE_MODE: {
        if (len < 6)
            break;
        d->device_mode = p[4];
        d->device_mode_valid = 1;
        break;
    }
    case CMD_GET_BUSPARAMS_TQ_RESP: {
        /* Timing as time quanta: prop, phase1, phase2, sjw, brp, then the
         * same five again for the FD data phase. */
        int ch = chan_from_he(d, p);
        if (len < 24 || ch < 0 || ch >= d->can_count)
            break;
        d->ch[ch].tq_prop   = (uint16_t)(p[4]  | (p[5]  << 8));
        d->ch[ch].tq_phase1 = (uint16_t)(p[6]  | (p[7]  << 8));
        d->ch[ch].tq_phase2 = (uint16_t)(p[8]  | (p[9]  << 8));
        d->ch[ch].tq_sjw    = (uint16_t)(p[10] | (p[11] << 8));
        d->ch[ch].tq_brp    = (uint16_t)(p[12] | (p[13] << 8));
        d->ch[ch].tq_valid  = 1;
        break;
    }
    case CMD_ERROR_EVENT: {
        /* A firmware-level fault, distinct from a CAN bus error. Surface it
         * as an error frame so a reader notices, with the code in data[]. */
        int ch = chan_from_he(d, p);
        uint8_t data[8];
        uint64_t ticks;
        if (len < 14)
            break;
        ticks = (uint64_t)p[4] | ((uint64_t)p[5] << 8) |
                ((uint64_t)p[6] << 16) | ((uint64_t)p[7] << 24) |
                ((uint64_t)p[8] << 32) | ((uint64_t)p[9] << 40);
        memset(data, 0, sizeof(data));
        data[0] = p[11];                 /* errorCode */
        data[1] = p[12];                 /* addInfo1 low */
        data[2] = p[13];                 /* addInfo1 high */
        if (ch < 0)
            ch = 0;
        if (ch < d->can_count)
            kv_push_frame(&d->ch[ch], 0, KV_FLAG_ERROR | KV_FLAG_STATUS,
                          data, 4, 4, kv_ticks_to_us(d, ticks));
        break;
    }
    default:
        break;
    }
    kv_maybe_complete_wait(d, cmd, seq, p, len);
}

static int map_one(struct kv_device *d, const char *name, uint8_t channel, uint16_t seq)
{
    uint8_t cmd[KV_HYDRA_CMD];
    uint8_t reply[KV_HYDRA_CMD];
    int err;

    hydra_zero(cmd);
    cmd[0] = CMD_MAP_CHANNEL_REQ;
    kv_hydra_set_dst(cmd, ROUTER_HE);
    kv_hydra_set_seq(cmd, seq);
    strncpy((char *)(cmd + 4), name, 16);
    cmd[20] = channel;
    /* Arm first: the response can arrive before this thread would otherwise
     * have registered interest, and each miss costs a full second. */
    kv_arm_wait(d, CMD_MAP_CHANNEL_RESP, seq, 1);
    err = hydra_send(d, cmd);
    if (err)
        return err;
    return kv_wait_cmd(d, CMD_MAP_CHANNEL_RESP, seq, 1, reply, sizeof(reply), KV_CMD_TIMEOUT);
}

int kv_hydra_init_card(struct kv_device *d)
{
    uint8_t cmd[KV_HYDRA_CMD];
    uint8_t reply[KV_HYDRA_CMD];
    int i, err;

    memset(d->channel2he, ILLEGAL_HE, sizeof(d->channel2he));
    memset(d->he2channel, 0xFF, sizeof(d->he2channel));
    /* 24 MHz until GET_SOFTWARE_DETAILS reports the real clock. */
    d->hires_mhz = 24;
    d->can_clk_mhz = 80;

    for (i = 0; i < KV_MAX_CAN; i++) {
        err = map_one(d, "CAN", (uint8_t)i, (uint16_t)(TRANSID_CAN | i));
        if (err)
            kv_log("hydra: MAP CAN%d failed (%d)", i, err);
    }
    (void)map_one(d, "SYSDBG", 0, 0x61);

    hydra_zero(cmd);
    cmd[0] = CMD_GET_CARD_INFO_REQ;
    kv_hydra_set_dst(cmd, ILLEGAL_HE);
    err = hydra_send(d, cmd);
    if (!err)
        (void)kv_wait_cmd(d, CMD_GET_CARD_INFO_RESP, 0, 0, reply, sizeof(reply), KV_CMD_TIMEOUT);

    hydra_zero(cmd);
    cmd[0] = CMD_GET_SOFTWARE_INFO_REQ;
    kv_hydra_set_dst(cmd, ILLEGAL_HE);
    err = hydra_send(d, cmd);
    if (!err)
        (void)kv_wait_cmd(d, CMD_GET_SOFTWARE_INFO_RESP, 0, 0, reply, sizeof(reply), KV_CMD_TIMEOUT);

    hydra_zero(cmd);
    cmd[0] = CMD_GET_SOFTWARE_DETAILS_REQ;
    cmd[4] = 1; /* useHydraExt */
    kv_hydra_set_dst(cmd, ILLEGAL_HE);
    err = hydra_send(d, cmd);
    if (!err)
        (void)kv_wait_cmd(d, CMD_GET_SOFTWARE_DETAILS_RESP, 0, 0, reply, sizeof(reply), KV_CMD_TIMEOUT);
    return 0;
}

int kv_hydra_bus_on(struct kv_channel *ch, const kv_open_opts *opts)
{
    struct kv_device *d = ch->dev;
    uint8_t cmd[KV_HYDRA_CMD];
    uint8_t reply[KV_HYDRA_CMD];
    uint8_t he = d->channel2he[ch->idx];
    uint8_t tseg1, tseg2, sjw, dt1, dt2, dsjw;
    int bitrate, dbitrate, fd, err;
    uint32_t br, dbr;

    if (he == ILLEGAL_HE)
        return KV_ERR_IO;

    bitrate = opts && opts->bitrate > 0 ? opts->bitrate : 500000;
    dbitrate = opts && opts->data_bitrate > 0 ? opts->data_bitrate : 2000000;
    fd = opts && opts->can_fd;
    {
        int can_hz = (d->can_clk_mhz ? d->can_clk_mhz : 80) * 1000000;
        kv_resolve_timing(bitrate, opts, 0, can_hz, &tseg1, &tseg2, &sjw);
        kv_resolve_timing(dbitrate, opts, 1, can_hz, &dt1, &dt2, &dsjw);
    }

    hydra_zero(cmd);
    cmd[0] = CMD_SET_DRIVERMODE_REQ;
    kv_hydra_set_dst(cmd, he);
    cmd[4] = (opts && opts->listen_only) ? DRIVERMODE_SILENT : DRIVERMODE_NORMAL;
    err = hydra_send(d, cmd);
    if (err)
        return err;

    hydra_zero(cmd);
    kv_hydra_set_dst(cmd, he);
    br = (uint32_t)bitrate;
    memcpy(cmd + 4, &br, 4);
    cmd[8] = tseg1;
    cmd[9] = tseg2;
    cmd[10] = sjw;
    cmd[11] = 1;
    if (fd) {
        cmd[0] = CMD_SET_BUSPARAMS_FD_REQ;
        dbr = (uint32_t)dbitrate;
        memcpy(cmd + 16, &dbr, 4);
        cmd[20] = dt1;
        cmd[21] = dt2;
        cmd[22] = dsjw;
        cmd[23] = 1;
        cmd[24] = OPEN_AS_CANFD_ISO;
        err = hydra_send(d, cmd);
        if (err)
            return err;
        (void)kv_wait_cmd(d, CMD_SET_BUSPARAMS_FD_RESP, 0, 0, reply, sizeof(reply), KV_CMD_TIMEOUT);
    } else {
        cmd[0] = CMD_SET_BUSPARAMS_REQ;
        err = hydra_send(d, cmd);
        if (err)
            return err;
        (void)kv_wait_cmd(d, CMD_SET_BUSPARAMS_RESP, 0, 0, reply, sizeof(reply), KV_CMD_TIMEOUT);
    }

    hydra_zero(cmd);
    cmd[0] = CMD_START_CHIP_REQ;
    kv_hydra_set_dst(cmd, he);
    err = hydra_send(d, cmd);
    if (err)
        return err;
    (void)kv_wait_cmd(d, CMD_START_CHIP_RESP, 0, 0, reply, sizeof(reply), KV_CMD_TIMEOUT);

    hydra_zero(cmd);
    cmd[0] = CMD_GET_CHIP_STATE_REQ;
    kv_hydra_set_dst(cmd, he);
    (void)hydra_send(d, cmd);
    ch->open = 1;
    /* The firmware reports its window in GET_SOFTWARE_INFO; until then use a
     * deliberately small value rather than assuming the adapter is generous. */
    ch->tx_outstanding = 0;
    ch->tx_window = d->max_outstanding_tx ? d->max_outstanding_tx : 16;
    ch->fd_mode = (uint8_t)fd;
    ch->listen_only = (uint8_t)(opts && opts->listen_only);
    return 0;
}

/* Ask the firmware what it can do. Each capability is a separate round trip,
 * so this runs once at open rather than on demand. A device that does not
 * answer simply leaves the bits clear, and the product table stays the
 * fallback. */
void kv_hydra_query_caps(struct kv_device *d)
{
    static const uint16_t subs[] = {
        CAP_SUB_CMD_SILENT_MODE, CAP_SUB_CMD_ERRFRAME, CAP_SUB_CMD_BUS_STATS,
        CAP_SUB_CMD_ERRCOUNT_READ, CAP_SUB_CMD_SINGLE_SHOT
    };
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];
    size_t i;

    for (i = 0; i < sizeof(subs) / sizeof(subs[0]); i++) {
        hydra_zero(cmd);
        cmd[0] = CMD_GET_CAPABILITIES_REQ;
        kv_hydra_set_dst(cmd, ILLEGAL_HE);
        cmd[4] = (uint8_t)(subs[i] & 0xFF);
        cmd[5] = (uint8_t)(subs[i] >> 8);
        kv_arm_wait(d, CMD_GET_CAPABILITIES_RESP, 0, 0);
        if (hydra_send(d, cmd) == 0)
            (void)kv_wait_cmd(d, CMD_GET_CAPABILITIES_RESP, 0, 0,
                              reply, sizeof(reply), 200);
    }
    /* CAN FD capability is already known from the software-details reply. */
    if (d->fd_capable)
        d->caps |= KV_CAP_CAN_FD;
}

int kv_hydra_get_busparams(struct kv_channel *ch)
{
    struct kv_device *d = ch->dev;
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];
    uint8_t he = d->channel2he[ch->idx];

    if (he == ILLEGAL_HE)
        return KV_ERR_IO;
    ch->bp_valid = 0;
    hydra_zero(cmd);
    cmd[0] = CMD_GET_BUSPARAMS_REQ;
    kv_hydra_set_dst(cmd, he);
    kv_arm_wait(d, CMD_GET_BUSPARAMS_RESP, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    if (kv_wait_cmd(d, CMD_GET_BUSPARAMS_RESP, 0, 0, reply, sizeof(reply),
                    KV_CMD_TIMEOUT))
        return KV_ERR_TIMEOUT;
    return ch->bp_valid ? KV_OK : KV_ERR_TIMEOUT;
}

int kv_hydra_get_busload(struct kv_channel *ch)
{
    struct kv_device *d = ch->dev;
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];
    uint8_t he = d->channel2he[ch->idx];

    if (he == ILLEGAL_HE)
        return KV_ERR_IO;
    ch->busload_valid = 0;
    hydra_zero(cmd);
    cmd[0] = CMD_GET_BUSLOAD_REQ;
    kv_hydra_set_dst(cmd, he);
    kv_arm_wait(d, CMD_GET_BUSLOAD_RESP, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    if (kv_wait_cmd(d, CMD_GET_BUSLOAD_RESP, 0, 0, reply, sizeof(reply),
                    KV_CMD_TIMEOUT))
        return KV_ERR_TIMEOUT;
    return ch->busload_valid ? KV_OK : KV_ERR_PARAM;
}

int kv_hydra_flush_tx(struct kv_channel *ch)
{
    struct kv_device *d = ch->dev;
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];
    uint8_t he = d->channel2he[ch->idx];

    if (he == ILLEGAL_HE)
        return KV_ERR_IO;
    hydra_zero(cmd);
    cmd[0] = CMD_FLUSH_QUEUE;
    kv_hydra_set_dst(cmd, he);
    cmd[4] = 1;                       /* flags: transmit queue */
    kv_arm_wait(d, CMD_FLUSH_QUEUE_RESP, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    (void)kv_wait_cmd(d, CMD_FLUSH_QUEUE_RESP, 0, 0, reply, sizeof(reply), 500);
    /* Whatever the adapter was holding is gone, so the window is clear. */
    ch->tx_outstanding = 0;
    return KV_OK;
}

/* Periodic transmit. The adapter repeats the frame on its own timer, which
 * keeps a heartbeat alive even if the host stalls. interval_us of 0
 * deactivates the buffer instead. */
int kv_hydra_auto_tx(struct kv_channel *ch, int buf, const kv_msg *msg,
                     uint32_t interval_us)
{
    struct kv_device *d = ch->dev;
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];
    uint8_t he = d->channel2he[ch->idx];
    int n;

    if (he == ILLEGAL_HE)
        return KV_ERR_IO;
    if (buf < 0 || buf > 255)
        return KV_ERR_PARAM;

    hydra_zero(cmd);
    cmd[0] = CMD_AUTO_TX_BUFFER_REQ;
    kv_hydra_set_dst(cmd, he);
    memcpy(cmd + 4, &interval_us, 4);
    cmd[8] = interval_us ? AUTOTXBUFFER_CMD_ACTIVATE
                         : AUTOTXBUFFER_CMD_DEACTIVATE;
    cmd[9] = (uint8_t)buf;
    if (msg && interval_us) {
        uint32_t id = msg->id;
        n = msg->len > 8 ? 8 : msg->len;
        memcpy(cmd + 12, &id, 4);
        memcpy(cmd + 16, msg->data, (size_t)n);
        cmd[24] = (uint8_t)(msg->dlc ? msg->dlc : n);
        if (msg->flags & KV_FLAG_RTR)
            cmd[25] |= MSGFLAG_REMOTE_FRAME;
        if (msg->flags & KV_FLAG_EXTENDED)
            cmd[25] |= MSGFLAG_EXTENDED_ID;
    }
    kv_arm_wait(d, CMD_AUTO_TX_BUFFER_RESP, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    (void)kv_wait_cmd(d, CMD_AUTO_TX_BUFFER_RESP, 0, 0, reply, sizeof(reply),
                      KV_CMD_TIMEOUT);
    return KV_OK;
}

int kv_hydra_auto_tx_info(struct kv_device *d)
{
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];

    d->autotx_valid = 0;
    hydra_zero(cmd);
    cmd[0] = CMD_AUTO_TX_BUFFER_REQ;
    kv_hydra_set_dst(cmd, ILLEGAL_HE);
    cmd[8] = AUTOTXBUFFER_CMD_GET_INFO;
    kv_arm_wait(d, CMD_AUTO_TX_BUFFER_RESP, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    if (kv_wait_cmd(d, CMD_AUTO_TX_BUFFER_RESP, 0, 0, reply, sizeof(reply),
                    KV_CMD_TIMEOUT))
        return KV_ERR_TIMEOUT;
    return d->autotx_valid ? KV_OK : KV_ERR_PARAM;
}

/* Minimum gap the firmware leaves between frames it sends. Enforced by the
 * adapter, so unlike a host-side delay it survives a stalled caller. */
int kv_hydra_tx_interval(struct kv_channel *ch, uint32_t interval_us)
{
    struct kv_device *d = ch->dev;
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];
    uint8_t he = d->channel2he[ch->idx];

    if (he == ILLEGAL_HE)
        return KV_ERR_IO;
    hydra_zero(cmd);
    cmd[0] = CMD_HYDRA_TX_INTERVAL_REQ;
    kv_hydra_set_dst(cmd, he);
    memcpy(cmd + 4, &interval_us, 4);
    kv_arm_wait(d, CMD_HYDRA_TX_INTERVAL_RESP, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    (void)kv_wait_cmd(d, CMD_HYDRA_TX_INTERVAL_RESP, 0, 0, reply,
                      sizeof(reply), KV_CMD_TIMEOUT);
    return KV_OK;
}

int kv_hydra_license(struct kv_device *d)
{
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];

    d->license_valid = 0;
    hydra_zero(cmd);
    cmd[0] = CMD_CHECK_LICENSE_REQ;
    kv_hydra_set_dst(cmd, ILLEGAL_HE);
    kv_arm_wait(d, CMD_CHECK_LICENSE_RESP, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    if (kv_wait_cmd(d, CMD_CHECK_LICENSE_RESP, 0, 0, reply, sizeof(reply),
                    KV_CMD_TIMEOUT))
        return KV_ERR_TIMEOUT;
    return d->license_valid ? KV_OK : KV_ERR_PARAM;
}

/* Hardware-specific I/O port. Both the port numbering and the meaning of the
 * value are the device's business; this only carries them. */
int kv_hydra_io_port(struct kv_device *d, int port, uint32_t *value, int write)
{
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];

    if (port < 0 || port > 255)
        return KV_ERR_PARAM;
    hydra_zero(cmd);
    kv_hydra_set_dst(cmd, ILLEGAL_HE);
    if (write) {
        cmd[0] = CMD_SET_IO_PORTS_REQ;
        cmd[4] = (uint8_t)port;
        memcpy(cmd + 8, value, 4);
        return hydra_send(d, cmd) ? KV_ERR_IO : KV_OK;
    }
    cmd[0] = CMD_GET_IO_PORTS_REQ;
    cmd[16] = (uint8_t)port;
    d->io_port_valid = 0;
    kv_arm_wait(d, CMD_GET_IO_PORTS_RESP, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    if (kv_wait_cmd(d, CMD_GET_IO_PORTS_RESP, 0, 0, reply, sizeof(reply),
                    KV_CMD_TIMEOUT))
        return KV_ERR_TIMEOUT;
    if (!d->io_port_valid)
        return KV_ERR_PARAM;
    *value = d->io_port_val;
    return KV_OK;
}

int kv_hydra_driver_mode(struct kv_channel *ch)
{
    struct kv_device *d = ch->dev;
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];
    uint8_t he = d->channel2he[ch->idx];

    if (he == ILLEGAL_HE)
        return KV_ERR_IO;
    ch->mode_valid = 0;
    hydra_zero(cmd);
    cmd[0] = CMD_GET_DRIVERMODE_REQ;
    kv_hydra_set_dst(cmd, he);
    kv_arm_wait(d, CMD_GET_DRIVERMODE_RESP, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    if (kv_wait_cmd(d, CMD_GET_DRIVERMODE_RESP, 0, 0, reply, sizeof(reply),
                    KV_CMD_TIMEOUT))
        return KV_ERR_TIMEOUT;
    return ch->mode_valid ? KV_OK : KV_ERR_TIMEOUT;
}

int kv_hydra_transceiver(struct kv_channel *ch)
{
    struct kv_device *d = ch->dev;
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];
    uint8_t he = d->channel2he[ch->idx];

    if (he == ILLEGAL_HE)
        return KV_ERR_IO;
    d->xcvr_valid = 0;
    hydra_zero(cmd);
    cmd[0] = CMD_GET_TRANSCEIVER_INFO_REQ;
    kv_hydra_set_dst(cmd, he);
    kv_arm_wait(d, CMD_GET_TRANSCEIVER_INFO_RESP, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    if (kv_wait_cmd(d, CMD_GET_TRANSCEIVER_INFO_RESP, 0, 0, reply,
                    sizeof(reply), KV_CMD_TIMEOUT))
        return KV_ERR_TIMEOUT;
    return d->xcvr_valid ? KV_OK : KV_ERR_PARAM;
}

int kv_hydra_interface_info(struct kv_channel *ch)
{
    struct kv_device *d = ch->dev;
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];
    uint8_t he = d->channel2he[ch->idx];

    if (he == ILLEGAL_HE)
        return KV_ERR_IO;
    d->iface_valid = 0;
    hydra_zero(cmd);
    cmd[0] = CMD_GET_INTERFACE_INFO_REQ;
    kv_hydra_set_dst(cmd, he);
    kv_arm_wait(d, CMD_GET_INTERFACE_INFO_RESP, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    if (kv_wait_cmd(d, CMD_GET_INTERFACE_INFO_RESP, 0, 0, reply,
                    sizeof(reply), KV_CMD_TIMEOUT))
        return KV_ERR_TIMEOUT;
    return d->iface_valid ? KV_OK : KV_ERR_PARAM;
}

int kv_hydra_card_info2(struct kv_device *d)
{
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];

    d->pcb_valid = 0;
    hydra_zero(cmd);
    cmd[0] = CMD_GET_CARD_INFO_2;
    kv_hydra_set_dst(cmd, ILLEGAL_HE);
    kv_arm_wait(d, CMD_GET_CARD_INFO_2, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    if (kv_wait_cmd(d, CMD_GET_CARD_INFO_2, 0, 0, reply, sizeof(reply),
                    KV_CMD_TIMEOUT))
        return KV_ERR_TIMEOUT;
    return d->pcb_valid ? KV_OK : KV_ERR_PARAM;
}

int kv_hydra_device_mode(struct kv_device *d, int *mode, int write)
{
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];

    hydra_zero(cmd);
    kv_hydra_set_dst(cmd, ILLEGAL_HE);
    if (write) {
        cmd[0] = CMD_SET_DEVICE_MODE;
        cmd[4] = (uint8_t)*mode;
        return hydra_send(d, cmd) ? KV_ERR_IO : KV_OK;
    }
    cmd[0] = CMD_GET_DEVICE_MODE;
    d->device_mode_valid = 0;
    kv_arm_wait(d, CMD_GET_DEVICE_MODE, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    if (kv_wait_cmd(d, CMD_GET_DEVICE_MODE, 0, 0, reply, sizeof(reply),
                    KV_CMD_TIMEOUT))
        return KV_ERR_TIMEOUT;
    if (!d->device_mode_valid)
        return KV_ERR_PARAM;
    *mode = d->device_mode;
    return KV_OK;
}

int kv_hydra_busparams_tq(struct kv_channel *ch)
{
    struct kv_device *d = ch->dev;
    uint8_t cmd[KV_HYDRA_CMD], reply[KV_HYDRA_CMD];
    uint8_t he = d->channel2he[ch->idx];

    if (he == ILLEGAL_HE)
        return KV_ERR_IO;
    ch->tq_valid = 0;
    hydra_zero(cmd);
    cmd[0] = CMD_GET_BUSPARAMS_TQ_REQ;
    kv_hydra_set_dst(cmd, he);
    kv_arm_wait(d, CMD_GET_BUSPARAMS_TQ_RESP, 0, 0);
    if (hydra_send(d, cmd))
        return KV_ERR_IO;
    if (kv_wait_cmd(d, CMD_GET_BUSPARAMS_TQ_RESP, 0, 0, reply, sizeof(reply),
                    KV_CMD_TIMEOUT))
        return KV_ERR_TIMEOUT;
    return ch->tq_valid ? KV_OK : KV_ERR_PARAM;
}

/* Fire and forget: the device does not answer this one. */
int kv_hydra_sound(struct kv_device *d, int freq_hz, int duration_ms)
{
    uint8_t cmd[KV_HYDRA_CMD];

    hydra_zero(cmd);
    cmd[0] = CMD_SOUND;
    kv_hydra_set_dst(cmd, ILLEGAL_HE);
    cmd[4] = 0;                                  /* subCmd: play */
    cmd[6] = (uint8_t)(freq_hz & 0xFF);
    cmd[7] = (uint8_t)((freq_hz >> 8) & 0xFF);
    cmd[8] = (uint8_t)(duration_ms & 0xFF);
    cmd[9] = (uint8_t)((duration_ms >> 8) & 0xFF);
    return hydra_send(d, cmd) ? KV_ERR_IO : KV_OK;
}

int kv_hydra_bus_off(struct kv_channel *ch)
{
    uint8_t cmd[KV_HYDRA_CMD];
    uint8_t reply[KV_HYDRA_CMD];
    uint8_t he = ch->dev->channel2he[ch->idx];
    hydra_zero(cmd);
    cmd[0] = CMD_STOP_CHIP_REQ;
    kv_hydra_set_dst(cmd, he);
    (void)hydra_send(ch->dev, cmd);
    (void)kv_wait_cmd(ch->dev, CMD_STOP_CHIP_RESP, 0, 0, reply, sizeof(reply), 500);
    ch->open = 0;
    return 0;
}

int kv_hydra_write(struct kv_channel *ch, const kv_msg *msg)
{
    struct kv_device *d = ch->dev;
    uint8_t he = d->channel2he[ch->idx];
    int err;
    uint8_t tid = kv_next_transid(ch);
    int fd = (msg->flags & KV_FLAG_FD) || ch->fd_mode;
    int n = msg->len;

    if (he == ILLEGAL_HE)
        return KV_ERR_IO;

    if (d->hydra_ext || fd) {
        uint8_t buf[96];
        uint16_t cmdlen;
        uint32_t id, flags, fpga_id, fpga_ctrl;
        int nbytes = n;
        if (nbytes > 64) nbytes = 64;
        if (!(msg->flags & KV_FLAG_FD) && nbytes > 8) nbytes = 8;
        cmdlen = (uint16_t)((32 + nbytes + 7) & ~7);
        memset(buf, 0, sizeof(buf));
        buf[0] = CMD_EXTENDED;
        kv_hydra_set_dst(buf, he);
        kv_hydra_set_seq(buf, tid);
        buf[4] = (uint8_t)(cmdlen & 0xFF);
        buf[5] = (uint8_t)(cmdlen >> 8);
        buf[6] = CMD_TX_CAN_MESSAGE_FD;
        id = msg->id;
        flags = 0;
        if (msg->flags & KV_FLAG_RTR) flags |= MSGFLAG_REMOTE_FRAME;
        if (msg->flags & KV_FLAG_EXTENDED) flags |= MSGFLAG_EXTENDED_ID;
        if (msg->flags & KV_FLAG_FD) flags |= MSGFLAG_FDF;
        if (msg->flags & KV_FLAG_BRS) flags |= MSGFLAG_BRS;
        memcpy(buf + 8, &flags, 4);
        memcpy(buf + 12, &id, 4);
        fpga_id = RTPACKET_ID(id);
        if (msg->flags & KV_FLAG_EXTENDED)
            fpga_id |= RTPACKET_IDE(1) | RTPACKET_SRR(1);
        if (msg->flags & KV_FLAG_RTR)
            fpga_id |= RTPACKET_RTR(1);
        memcpy(buf + 16, &fpga_id, 4);
        fpga_ctrl = RTPACKET_DLC(msg->dlc ? msg->dlc : kv_len_to_dlc(nbytes, fd)) |
                    RTPACKET_FDF((msg->flags & KV_FLAG_FD) ? 1 : 0) |
                    RTPACKET_BRS((msg->flags & KV_FLAG_BRS) ? 1 : 0) |
                    TPACKET_AREQ(1);
        memcpy(buf + 20, &fpga_ctrl, 4);
        buf[24] = (uint8_t)nbytes;
        buf[25] = msg->dlc ? msg->dlc : kv_len_to_dlc(nbytes, fd);
        memcpy(buf + 32, msg->data, (size_t)nbytes);
        err = kv_send_raw(d, buf, cmdlen);
        if (!err)
            ch->tx_outstanding++;
        return err;
    }

    {
        uint8_t cmd[KV_HYDRA_CMD];
        uint32_t id = msg->id;
        int nbytes = n > 8 ? 8 : n;
        hydra_zero(cmd);
        cmd[0] = CMD_TX_CAN_MESSAGE;
        kv_hydra_set_dst(cmd, he);
        kv_hydra_set_seq(cmd, tid);
        memcpy(cmd + 4, &id, 4);
        memcpy(cmd + 8, msg->data, (size_t)nbytes);
        cmd[16] = (uint8_t)nbytes;
        if (msg->flags & KV_FLAG_RTR)
            cmd[17] |= MSGFLAG_REMOTE_FRAME;
        if (msg->flags & KV_FLAG_EXTENDED)
            cmd[17] |= MSGFLAG_EXTENDED_ID;
        cmd[18] = tid;
        cmd[20] = (uint8_t)ch->idx;
        err = hydra_send(d, cmd);
        if (!err)
            ch->tx_outstanding++;
        return err;
    }
}
