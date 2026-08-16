/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * libusb transport for PEAK USB adapters:
 *   classic PCAN-USB (0x000c), PCAN-USB Pro (0x000d), USB FD family (uCAN)
 * Protocol adapted from PEAK peak-linux-driver 9.2.0
 */
#include "pcan_priv.h"
#include "peak_usb.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stddef.h>
#include <time.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static libusb_context *g_ctx;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static struct pcan_device *g_devs[8];
static int g_ndev;
static struct pcan_handle g_map[PEAKUSB_MAX_HANDLES];
static int g_nmap;
static int g_inited;
static int g_debug;

void pcan_log(const char *fmt, ...)
{
    va_list ap;
    if (!g_debug)
        return;
    fputs("PeakUSB: ", stderr);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

void pcan_event_signal(struct pcan_channel *ch)
{
    uint8_t b = 1;
    if (ch->ev_w >= 0)
        (void)!write(ch->ev_w, &b, 1);
}

void pcan_event_clear_if_empty(struct pcan_channel *ch)
{
    uint8_t buf[64];
    if (ch->ev_r < 0)
        return;
    pthread_mutex_lock(&ch->q.lock);
    if (ch->q.count == 0) {
        pthread_mutex_unlock(&ch->q.lock);
        while (read(ch->ev_r, buf, sizeof(buf)) > 0)
            ;
        return;
    }
    pthread_mutex_unlock(&ch->q.lock);
}

/* PCAN-Basic handles are assigned in bus/address order, so the same adapter
 * keeps the same handle across runs as long as it stays in the same port. */
static const TPCANHandle k_usb_handles[PEAKUSB_MAX_HANDLES] = {
    PCAN_USBBUS1, PCAN_USBBUS2, PCAN_USBBUS3, PCAN_USBBUS4,
    PCAN_USBBUS5, PCAN_USBBUS6, PCAN_USBBUS7, PCAN_USBBUS8
};

TPCANHandle pcan_usb_handle_at(int idx)
{
    if (idx < 0 || idx >= PEAKUSB_MAX_HANDLES)
        return PCAN_NONEBUS;
    return k_usb_handles[idx];
}

static int is_fd_pid(uint16_t pid)
{
    return pid == PCAN_USBFD_PRODUCT_ID ||
           pid == PCAN_USBPROFD_PRODUCT_ID ||
           pid == PCAN_USBCHIP_PRODUCT_ID ||
           pid == PCAN_USBX6_PRODUCT_ID;
}

static int is_peak_pid(uint16_t pid)
{
    return is_fd_pid(pid) ||
           pid == PCAN_USB_PRODUCT_ID ||
           pid == PCAN_USBPRO_PRODUCT_ID;
}

static uint8_t family_from_pid(uint16_t pid)
{
    if (pid == PCAN_USB_PRODUCT_ID)
        return PCAN_FAM_USB;
    if (pid == PCAN_USBPRO_PRODUCT_ID)
        return PCAN_FAM_USBPRO;
    if (is_fd_pid(pid))
        return PCAN_FAM_FD;
    return PCAN_FAM_USB;
}

static const char *pid_name(uint16_t pid)
{
    switch (pid) {
    case PCAN_USBFD_PRODUCT_ID:      return "PCAN-USB FD";
    case PCAN_USBPROFD_PRODUCT_ID:   return "PCAN-USB Pro FD";
    case PCAN_USBCHIP_PRODUCT_ID:    return "PCAN-Chip USB";
    case PCAN_USBX6_PRODUCT_ID:      return "PCAN-USB X6";
    case PCAN_USB_PRODUCT_ID:        return "PCAN-USB";
    case PCAN_USBPRO_PRODUCT_ID:     return "PCAN-USB Pro";
    default:                         return "PCAN-USB";
    }
}

static int pid_can_count(uint16_t pid)
{
    switch (pid) {
    case PCAN_USBPRO_PRODUCT_ID:
    case PCAN_USBPROFD_PRODUCT_ID:
    case PCAN_USBX6_PRODUCT_ID:
        return 2;
    default:
        return 1;
    }
}

static int any_channel_open(void)
{
    int i, c;
    for (i = 0; i < g_ndev; i++)
        for (c = 0; c < g_devs[i]->can_count; c++)
            if (g_devs[i]->ch[c].initialized)
                return 1;
    return 0;
}

static int vendor_in(libusb_device_handle *h, uint16_t value,
                     void *data, int len)
{
    return libusb_control_transfer(
        h,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_OTHER,
        USB_VENDOR_REQUEST_INFO, value, 0, data, (uint16_t)len,
        PEAKUSB_CMD_TIMEOUT);
}

static int vendor_fkt(libusb_device_handle *h, uint16_t value,
                      void *data, int len)
{
    return libusb_control_transfer(
        h,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_OTHER,
        USB_VENDOR_REQUEST_FKT, value, 0, data, (uint16_t)len,
        PEAKUSB_CMD_TIMEOUT);
}

static int driver_loaded(struct pcan_device *d, int loaded)
{
    uint8_t buf[16];
    memset(buf, 0, sizeof(buf));
    buf[0] = 0; /* CAN interface */
    buf[1] = loaded ? 1 : 0;
    return vendor_fkt(d->h, USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED,
                      buf, sizeof(buf));
}

static int read_fw_info(libusb_device_handle *h, struct pcan_usbfd_fw_info *pfi)
{
    int n;
    memset(pfi, 0, sizeof(*pfi));
    n = vendor_in(h, PCAN_USBFD_FWINFO_REQ, pfi, (int)sizeof(*pfi));
    if (n < 28)
        return -1;
    return n;
}

static void apply_fw_info(struct pcan_device *d, const struct pcan_usbfd_fw_info *pfi)
{
    uint16_t type = pfi->type;
    int i;

    d->hw_type = pfi->hw_type;
    d->hw_ver = pfi->hw_version;
    memcpy(d->fw, pfi->fw_version, 3);
    memcpy(d->bl, pfi->bl_version, 3);
    d->serial = pfi->ser_no;
    d->dev_id[0] = pfi->dev_id[0];
    d->dev_id[1] = pfi->dev_id[1];

    if (pfi->can_bus_count)
        d->can_count = pfi->can_bus_count;
    if (d->can_count < 1)
        d->can_count = pid_can_count(d->pid);
    if (d->can_count > PEAKUSB_MAX_CAN)
        d->can_count = PEAKUSB_MAX_CAN;

    d->ep_cmd_out = 0x01;
    d->ep_cmd_in = 0x81;
    d->ep_data_in = 0x82;
    d->ep_data_out[0] = 0x02;
    d->ep_data_out[1] = 0x03;

    if (type >= PCAN_USBFD_TYPE_EXT) {
        if (pfi->cmd_out_ep)
            d->ep_cmd_out = pfi->cmd_out_ep;
        if (pfi->cmd_in_ep)
            d->ep_cmd_in = pfi->cmd_in_ep;
        if (pfi->data_in_ep)
            d->ep_data_in = pfi->data_in_ep;
        for (i = 0; i < d->can_count; i++)
            if (pfi->data_out_ep[i])
                d->ep_data_out[i] = pfi->data_out_ep[i];
    }

    for (i = 0; i < d->can_count; i++)
        d->ch[i].device_id = d->dev_id[i];

    pcan_log("%s bus=%u addr=%u fw=%u.%u.%u ch=%d cmd=%02x/%02x din=%02x dout=%02x/%02x",
             d->name, d->bus, d->addr, d->fw[0], d->fw[1], d->fw[2],
             d->can_count, d->ep_cmd_out, d->ep_cmd_in, d->ep_data_in,
             d->ep_data_out[0], d->ep_data_out[1]);
}

static uint32_t rd_le32(const void *p)
{
    const uint8_t *b = p;
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static uint16_t rd_le16(const void *p)
{
    const uint8_t *b = p;
    return (uint16_t)(b[0] | (b[1] << 8));
}

static void wr_le32(void *p, uint32_t v)
{
    uint8_t *b = p;
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

static void wr_le16(void *p, uint16_t v)
{
    uint8_t *b = p;
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
}

static int classic_cmd(struct pcan_device *d, uint8_t fn, uint8_t num,
                       uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
    struct pcan_usb_param cmd;
    int transferred = 0, err;

    memset(&cmd, 0, sizeof(cmd));
    cmd.Function = fn;
    cmd.Number = num;
    cmd.Param[0] = p0;
    cmd.Param[1] = p1;
    cmd.Param[2] = p2;
    cmd.Param[3] = p3;

    pthread_mutex_lock(&d->cmd_lock);
    err = libusb_bulk_transfer(d->h, d->ep_cmd_out, (uint8_t *)&cmd,
                               PCAN_USB_CMD_LEN, &transferred, PEAKUSB_CMD_TIMEOUT);
    pthread_mutex_unlock(&d->cmd_lock);
    if (err)
        pcan_log("classic cmd %u.%u: %s", fn, num, libusb_strerror(err));
    return err;
}

static int classic_cmd_get(struct pcan_device *d, uint8_t fn, uint8_t num,
                           uint8_t *p0, uint8_t *p1, uint8_t *p2, uint8_t *p3)
{
    struct pcan_usb_param cmd;
    int transferred = 0, err, i;

    for (i = 0; i < 3; i++) {
        err = classic_cmd(d, fn, num, p0 ? *p0 : 0, p1 ? *p1 : 0,
                          p2 ? *p2 : 0, p3 ? *p3 : 0);
        if (err)
            return err;
        memset(&cmd, 0, sizeof(cmd));
        pthread_mutex_lock(&d->cmd_lock);
        err = libusb_bulk_transfer(d->h, d->ep_cmd_in, (uint8_t *)&cmd,
                                   PCAN_USB_CMD_LEN, &transferred, PEAKUSB_CMD_TIMEOUT);
        pthread_mutex_unlock(&d->cmd_lock);
        if (!err) {
            if (p0)
                *p0 = cmd.Param[0];
            if (p1)
                *p1 = cmd.Param[1];
            if (p2)
                *p2 = cmd.Param[2];
            if (p3)
                *p3 = cmd.Param[3];
            return 0;
        }
        if (err != LIBUSB_ERROR_TIMEOUT)
            break;
    }
    pcan_log("classic get %u.%u: %s", fn, num, libusb_strerror(err));
    return err;
}

static int classic_read_ids(struct pcan_device *d)
{
    uint8_t a = 0, b = 0, c = 0, e = 0;

    if (classic_cmd_get(d, PCAN_USB_CMD_SN, PCAN_USB_GET, &a, &b, &c, &e) == 0)
        d->serial = ((uint32_t)a << 24) | ((uint32_t)b << 16) |
                    ((uint32_t)c << 8) | e;
    a = b = c = e = 0;
    if (classic_cmd_get(d, PCAN_USB_CMD_DEVID, PCAN_USB_GET, &a, &b, &c, &e) == 0) {
        d->dev_id[0] = a;
        d->ch[0].device_id = a;
    }
    return 0;
}

static uint32_t usbpro_ccbt(const struct pcan_channel *ch)
{
    uint16_t brp, sjw, tseg1, tseg2;
    uint8_t tsam, btr0, btr1;
    uint32_t ccbt;

    if (ch->btr0btr1) {
        btr0 = (uint8_t)(ch->btr0btr1 >> 8);
        btr1 = (uint8_t)(ch->btr0btr1 & 0xff);
        brp = (uint16_t)((btr0 & 0x3f) + 1);
        sjw = (uint16_t)(((btr0 >> 6) & 0x03) + 1);
        tseg1 = (uint16_t)((btr1 & 0x0f) + 1);
        tseg2 = (uint16_t)(((btr1 >> 4) & 0x07) + 1);
        tsam = (uint8_t)((btr1 >> 7) & 1);
        /* SJA1000 8 MHz -> USB Pro 56 MHz */
        brp = (uint16_t)(brp * 7u);
    } else {
        brp = ch->bt.brp ? ch->bt.brp : 1;
        sjw = ch->bt.sjw ? ch->bt.sjw : 1;
        tseg1 = ch->bt.tseg1 ? ch->bt.tseg1 : 1;
        tseg2 = ch->bt.tseg2 ? ch->bt.tseg2 : 1;
        tsam = ch->bt.tsam;
        if (ch->bt.clock_hz == 80000000u)
            brp = (uint16_t)((brp * 56u) / 80u);
        else if (ch->bt.clock_hz && ch->bt.clock_hz != USBPRO_SYSCLK_HZ)
            brp = (uint16_t)((uint64_t)brp * USBPRO_SYSCLK_HZ / ch->bt.clock_hz);
        if (brp < 1)
            brp = 1;
    }
    ccbt = ((uint32_t)tsam << 23);
    ccbt |= ((uint32_t)(tseg2 - 1) << 20);
    ccbt |= ((uint32_t)(tseg1 - 1) << 16);
    ccbt |= ((uint32_t)(sjw - 1) << 14);
    ccbt |= (uint32_t)(brp - 1);
    return ccbt;
}

static int usbpro_cmd(struct pcan_device *d, const uint8_t *rec, int rec_len)
{
    uint8_t buf[PEAKUSB_CMD_BUF];
    int transferred = 0, err;

    if (rec_len < 0 || rec_len + 4 > (int)sizeof(buf))
        return -1;
    memset(buf, 0, 4);
    buf[0] = 1; /* rec_count = 1 */
    memcpy(buf + 4, rec, (size_t)rec_len);

    pthread_mutex_lock(&d->cmd_lock);
    err = libusb_bulk_transfer(d->h, d->ep_cmd_out, buf, rec_len + 4,
                               &transferred, PEAKUSB_CMD_TIMEOUT);
    pthread_mutex_unlock(&d->cmd_lock);
    if (err)
        pcan_log("usbpro cmd type 0x%02x: %s", rec[0], libusb_strerror(err));
    return err;
}

static int usbpro_set_u16(struct pcan_device *d, uint8_t type, int ch, uint16_t v)
{
    struct pcan_usbpro_u16 rec;
    memset(&rec, 0, sizeof(rec));
    rec.data_type = type;
    rec.channel = (uint8_t)ch;
    wr_le16(&rec.value, v);
    return usbpro_cmd(d, (uint8_t *)&rec, (int)sizeof(rec));
}

static int usbpro_set_baud(struct pcan_channel *ch)
{
    struct pcan_usbpro_baudrate rec;
    memset(&rec, 0, sizeof(rec));
    rec.data_type = USBPRO_FKT_SETBAUDRATE;
    rec.channel = (uint8_t)ch->can_idx;
    wr_le32(&rec.CCBT, usbpro_ccbt(ch));
    return usbpro_cmd(ch->dev, (uint8_t *)&rec, (int)sizeof(rec));
}

static int usbpro_calibration(struct pcan_device *d, int on)
{
    struct pcan_usbpro_calibration rec;
    memset(&rec, 0, sizeof(rec));
    rec.data_type = USBPRO_FKT_SETGET_CALIBRATION;
    wr_le16(&rec.mode, (uint16_t)(on ? 1 : 0));
    return usbpro_cmd(d, (uint8_t *)&rec, (int)sizeof(rec));
}

static int usbpro_led(struct pcan_device *d, int ch, uint16_t mode)
{
    struct pcan_usbpro_set_can_led rec;
    memset(&rec, 0, sizeof(rec));
    rec.data_type = USBPRO_FKT_SET_CANLED;
    rec.channel = (uint8_t)ch;
    wr_le16(&rec.mode, mode);
    wr_le32(&rec.timeout, 1);
    return usbpro_cmd(d, (uint8_t *)&rec, (int)sizeof(rec));
}

static int usbpro_read_info(libusb_device_handle *h, struct pcan_device *d)
{
    struct pcan_usbpro_ext_firmware_info fi;
    struct pcan_usbpro_bootloader_info bi;
    int n;

    memset(&fi, 0, sizeof(fi));
    n = vendor_in(h, USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE, &fi, (int)sizeof(fi));
    if (n >= 12) {
        d->fw[0] = fi.version[0];
        d->fw[1] = fi.version[1];
        d->fw[2] = fi.version[2];
    }
    memset(&bi, 0, sizeof(bi));
    n = vendor_in(h, USB_VENDOR_REQUEST_wVALUE_INFO_BOOTLOADER, &bi, (int)sizeof(bi));
    if (n >= 20) {
        d->serial = bi.serial_num_high;
        d->hw_type = (uint8_t)bi.hw_type;
        d->hw_ver = (uint8_t)bi.hw_rev;
        memcpy(d->bl, bi.version, 3);
    }
    return 0;
}

static int usbpro_sizeof_rec(uint8_t type)
{
    switch (type) {
    /* The RX record carries a 32-bit timestamp that the TX record does not,
     * so the two families differ by 4 bytes and must not share a size. */
    case USBPRO_RX_8:
        return (int)sizeof(struct pcan_usbpro_canmsg_rx);
    case USBPRO_RX_4:
        return (int)sizeof(struct pcan_usbpro_canmsg_rx) - 4;
    case USBPRO_RX_0:
    case USBPRO_RTR_RX:
        return (int)sizeof(struct pcan_usbpro_canmsg_rx) - 8;
    case USBPRO_TX_8:
        return (int)sizeof(struct pcan_usbpro_canmsg_tx);
    case USBPRO_TX_4:
        return (int)sizeof(struct pcan_usbpro_canmsg_tx) - 4;
    case USBPRO_TX_0:
        return (int)sizeof(struct pcan_usbpro_canmsg_tx) - 8;
    case USBPRO_STATUS_ERROR_RX:
        return (int)sizeof(struct pcan_usbpro_status_error_rx);
    case USBPRO_CALIBRATION_RX:
        return 12;
    case USBPRO_BUSLOAD_RX:
        return 8;
    case USBPRO_FKT_SETBAUDRATE:
        return (int)sizeof(struct pcan_usbpro_baudrate);
    case USBPRO_FKT_SETCANBUSACTIVATE:
    case USBPRO_FKT_SETSILENTMODE:
    case USBPRO_FKT_SETFILTERMODE:
        return (int)sizeof(struct pcan_usbpro_u16);
    case USBPRO_FKT_SETGET_CALIBRATION:
        return (int)sizeof(struct pcan_usbpro_calibration);
    case USBPRO_FKT_SET_CANLED:
        return (int)sizeof(struct pcan_usbpro_set_can_led);
    default:
        return -1;
    }
}

int pcan_cmd_send(struct pcan_device *d, uint8_t *buf, size_t *len)
{
    int transferred = 0, err, retry;

    if (*len + 8 <= PEAKUSB_CMD_BUF) {
        memset(buf + *len, 0xff, 8);
        *len += 8;
    }

    pthread_mutex_lock(&d->cmd_lock);
    for (retry = 0; retry < 3; retry++) {
        transferred = 0;
        err = libusb_bulk_transfer(d->h, d->ep_cmd_out, buf, (int)*len,
                                   &transferred, PEAKUSB_CMD_TIMEOUT);
        if (err == 0)
            break;
        pcan_log("cmd bulk err %s (%d/3)", libusb_strerror(err), retry + 1);
        if (err != LIBUSB_ERROR_TIMEOUT)
            break;
    }
    pthread_mutex_unlock(&d->cmd_lock);
    *len = 0;
    return err;
}

void *pcan_cmd_add(uint8_t *buf, size_t *len, int ch, int op)
{
    struct canfd_command *c;
    if (*len + 8 > PEAKUSB_CMD_BUF)
        return NULL;
    memset(buf + *len, 0, 8);
    c = (struct canfd_command *)(buf + *len);
    c->opcode_channel = CANFD_CMD_OPCODE_CHANNEL(ch, op);
    *len += 8;
    return c;
}

static int send_led(struct pcan_device *d, int ch, uint8_t mode)
{
    uint8_t buf[PEAKUSB_CMD_BUF];
    size_t len = 0;
    struct ucan_usb_led *cmd = pcan_cmd_add(buf, &len, ch, CANFD_USB_CMD_LED_SET);
    if (!cmd)
        return -1;
    cmd->mode = mode;
    return pcan_cmd_send(d, buf, &len);
}

static int send_clock(struct pcan_device *d, int ch, uint8_t mode)
{
    uint8_t buf[PEAKUSB_CMD_BUF];
    size_t len = 0;
    struct ucan_usb_clock *cmd = pcan_cmd_add(buf, &len, ch, CANFD_USB_CMD_CLK_SET);
    if (!cmd)
        return -1;
    cmd->mode = mode;
    return pcan_cmd_send(d, buf, &len);
}

static int send_simple(struct pcan_device *d, int ch, int op)
{
    uint8_t buf[PEAKUSB_CMD_BUF];
    size_t len = 0;
    if (!pcan_cmd_add(buf, &len, ch, op))
        return -1;
    return pcan_cmd_send(d, buf, &len);
}

static uint8_t clock_mode(uint32_t hz)
{
    switch (hz) {
    case 20000000: return 0x5;
    case 24000000: return 0x4;
    case 30000000: return 0x3;
    case 40000000: return 0x2;
    case 60000000: return 0x1;
    default:       return CANFD_USB_CLK_80MHZ;
    }
}

static int filter_accepts(const struct pcan_channel *ch, const TPCANMsgFD *m)
{
    int i, ext;

    if (m->MSGTYPE & (PCAN_MESSAGE_STATUS | PCAN_MESSAGE_ERRFRAME))
        return 1;
    if (ch->filter_mode == PCAN_FILTER_OPEN)
        return 1;
    if (ch->filter_mode == PCAN_FILTER_CLOSE)
        return 0;
    ext = (m->MSGTYPE & PCAN_MESSAGE_EXTENDED) ? 1 : 0;
    for (i = 0; i < ch->filter_count; i++) {
        if (ch->filters[i].ext != ext)
            continue;
        if (m->ID >= ch->filters[i].from && m->ID <= ch->filters[i].to)
            return 1;
    }
    return 0;
}

static void deliver(struct pcan_channel *ch, TPCANMsgFD *m, uint64_t ts)
{
    int was_empty;
    if (!ch->initialized || !ch->receive_on)
        return;
    if ((m->MSGTYPE & PCAN_MESSAGE_STATUS) && !ch->allow_status)
        return;
    if ((m->MSGTYPE & PCAN_MESSAGE_RTR) && !ch->allow_rtr)
        return;
    if ((m->MSGTYPE & PCAN_MESSAGE_ERRFRAME) && !ch->allow_error)
        return;
    if (!filter_accepts(ch, m))
        return;

    was_empty = 0;
    if (pcan_queue_push(&ch->q, m, ts, &was_empty))
        ch->status |= PCAN_ERROR_QOVERRUN;
    if (was_empty)
        pcan_event_signal(ch);
}

static uint64_t classic_ticks_us(uint32_t ticks)
{
    return (uint64_t)ticks * PCAN_USB_TS_SCALE_MULTIPLIER / PCAN_USB_TS_SCALE_DIVISOR;
}

static uint64_t classic_ts_now(const struct pcan_device *d)
{
    return classic_ticks_us(((uint32_t)d->usb_ts_hi << 16) | d->usb_ts16);
}

/* The classic adapter sends a full 16-bit tick count with the first frame of
 * a URB and only the low byte for the ones after it. */
static uint64_t classic_take_ts(struct pcan_device *d, uint8_t **pp, uint8_t *end, int first)
{
    uint16_t ts16;
    uint8_t *p = *pp;

    if (first) {
        if (p + 2 > end)
            return classic_ts_now(d);
        ts16 = (uint16_t)(p[0] | (p[1] << 8));
        p += 2;
        if (d->usb_ts16 && ts16 < d->usb_ts16)
            d->usb_ts_hi++;
        d->usb_ts16 = ts16;
    } else {
        if (p + 1 > end)
            return classic_ts_now(d);
        if (*p < (d->usb_ts16 & 0xff)) {
            /* The low byte wrapped. Carrying 0x100 into the high byte can
             * itself overflow the 16-bit word, and that carry belongs in
             * usb_ts_hi; dropping it rewinds the clock by ~2.8 s. */
            uint32_t bumped = (uint32_t)(d->usb_ts16 & 0xff00) + 0x100;
            if (bumped > 0xffff)
                d->usb_ts_hi++;
            d->usb_ts16 = (uint16_t)((bumped & 0xff00) | *p);
        } else {
            d->usb_ts16 = (uint16_t)((d->usb_ts16 & 0xff00) | *p);
        }
        p++;
    }
    *pp = p;
    return classic_ts_now(d);
}

static void decode_classic(struct pcan_device *d, uint8_t *data, int len)
{
    uint8_t *p, *end;
    int rec_count, i, ts8 = 0;
    struct pcan_channel *ch = &d->ch[0];

    if (len < 2)
        return;
    rec_count = data[1];
    p = data + 2;
    end = data + len;

    for (i = 0; i < rec_count && p < end; i++) {
        uint8_t sl = *p++;
        uint8_t dlc = sl & STLN_DATA_LENGTH;
        uint64_t ts = 0;
        TPCANMsgFD m;

        if (sl & STLN_INTERNAL_DATA) {
            uint8_t fn, num;
            if (p + 2 > end)
                return;
            fn = *p++;
            num = *p++;
            if (sl & STLN_WITH_TIMESTAMP)
                ts = classic_take_ts(d, &p, end, !ts8++);
            if (fn == 1) {
                uint32_t stbits = 0;
                if (num & PCAN_USB_BUS_OFF) {
                    stbits |= PCAN_ERROR_BUSOFF;
                } else if (num & PCAN_USB_BUS_HEAVY) {
                    /* Heavy is the error-warning level. Error-passive is a
                     * property of the counters, not of this bit. */
                    stbits |= PCAN_ERROR_BUSHEAVY;
                    if (ch->rx_err_cnt >= 128 || ch->tx_err_cnt >= 128)
                        stbits |= PCAN_ERROR_BUSPASSIVE;
                } else if (num & PCAN_USB_BUS_LIGHT) {
                    stbits |= PCAN_ERROR_BUSLIGHT;
                }
                if (num & (PCAN_USB_CAN_RECEIVE_OVERRUN | PCAN_USB_QUEUE_OVERRUN))
                    ch->status |= PCAN_ERROR_OVERRUN;
                if (num & (PCAN_USB_XMT_BUFFER_FULL | PCAN_USB_QUEUE_XMT_FULL))
                    ch->status |= PCAN_ERROR_XMTFULL;
                {
                    uint32_t old_bus = ch->status & PCAN_ERROR_ANYBUSERR;
                    ch->status = (ch->status & ~PCAN_ERROR_ANYBUSERR) | stbits;
                    if (stbits != old_bus) {
                        memset(&m, 0, sizeof(m));
                        m.MSGTYPE = PCAN_MESSAGE_STATUS;
                        m.ID = ch->status;
                        deliver(ch, &m, ts);
                    }
                }
                while (dlc && p < end) {
                    p++;
                    dlc--;
                }
            } else if (fn == 5) {
                /* Bus-error record: ECC, then the two error counters. */
                if (p + 3 > end)
                    return;
                if (num == 0x00 || num == 0x80) {
                    ch->rx_err_cnt = p[1];
                    ch->tx_err_cnt = p[2];
                }
                memset(&m, 0, sizeof(m));
                m.MSGTYPE = PCAN_MESSAGE_ERRFRAME;
                m.ID = p[0];
                m.DLC = 2;
                m.DATA[0] = ch->rx_err_cnt;
                m.DATA[1] = ch->tx_err_cnt;
                deliver(ch, &m, ts);
                while (dlc && p < end) {
                    p++;
                    dlc--;
                }
            } else {
                /* Functions 2 (analog), 3 (bus load) and 4 (timestamp sync)
                 * carry a fixed payload; the length nibble does not describe
                 * them, so consuming it would desynchronise the packet. */
                int skip = (fn == 2) ? 2 : (fn == 3) ? 1 : (fn == 4) ? 2 : dlc;
                if (fn == 4 && p + 2 <= end) {
                    /* Periodic clock sync: the authoritative 16-bit tick. */
                    uint16_t sync = (uint16_t)(p[0] | (p[1] << 8));
                    if (d->usb_ts16 && sync < d->usb_ts16)
                        d->usb_ts_hi++;
                    d->usb_ts16 = sync;
                }
                while (skip > 0 && p < end) {
                    p++;
                    skip--;
                }
            }
            continue;
        }

        memset(&m, 0, sizeof(m));
        m.DLC = dlc > 8 ? 8 : dlc;
        if (sl & STLN_RTR)
            m.MSGTYPE |= PCAN_MESSAGE_RTR;
        if (sl & STLN_EXTENDED_ID) {
            uint32_t id;
            if (p + 4 > end)
                return;
            id = rd_le32(p);
            p += 4;
            m.ID = id >> 3;
            m.MSGTYPE |= PCAN_MESSAGE_EXTENDED;
        } else {
            uint16_t id;
            if (p + 2 > end)
                return;
            id = rd_le16(p);
            p += 2;
            m.ID = id >> 5;
        }
        ts = classic_take_ts(d, &p, end, !ts8++);
        if (!(sl & STLN_RTR) && dlc) {
            int n = dlc > 8 ? 8 : dlc;
            int skip = dlc;
            if (p + skip > end)
                return;
            memcpy(m.DATA, p, (size_t)n);
            p += skip;
        }
        deliver(ch, &m, ts);
    }
}

/* The USB Pro reports a 32-bit microsecond stamp, which wraps every ~71.6
 * minutes. Track the wraps so callers see a monotonic 64-bit value. */
static uint64_t usbpro_ts64(struct pcan_device *d, uint32_t ts32)
{
    if (ts32 < d->usbpro_ts_low)
        d->usbpro_ts_hi++;
    d->usbpro_ts_low = ts32;
    return ((uint64_t)d->usbpro_ts_hi << 32) | ts32;
}

static void decode_usbpro_rec(struct pcan_device *d, uint8_t *rec)
{
    uint8_t type = rec[0];
    TPCANMsgFD m;
    int ci;

    switch (type) {
    case USBPRO_RX_8:
    case USBPRO_RX_4:
    case USBPRO_RX_0:
    case USBPRO_RTR_RX: {
        struct pcan_usbpro_canmsg_rx *rx = (struct pcan_usbpro_canmsg_rx *)rec;
        uint8_t dlc = rx->len & 0x0f;
        ci = (rx->len >> 4) & 0x0f;
        if (ci < 0 || ci >= d->can_count)
            return;
        memset(&m, 0, sizeof(m));
        m.ID = rd_le32(&rx->id);
        m.DLC = dlc > 8 ? 8 : dlc;
        if (rx->flags & USBPRO_EXT)
            m.MSGTYPE |= PCAN_MESSAGE_EXTENDED;
        if (rx->flags & USBPRO_RTR || type == USBPRO_RTR_RX)
            m.MSGTYPE |= PCAN_MESSAGE_RTR;
        if (!(m.MSGTYPE & PCAN_MESSAGE_RTR) && m.DLC)
            memcpy(m.DATA, rx->data, m.DLC);
        deliver(&d->ch[ci], &m, usbpro_ts64(d, rd_le32(&rx->timestamp32)));
        return;
    }
    case USBPRO_STATUS_ERROR_RX: {
        struct pcan_usbpro_status_error_rx *er =
            (struct pcan_usbpro_status_error_rx *)rec;
        uint16_t raw = rd_le16(&er->status);
        uint32_t stbits = 0;
        ci = (er->channel >> 4) & 0x0f;
        if (ci >= d->can_count)
            ci = er->channel & 0x0f;
        if (ci < 0 || ci >= d->can_count)
            return;
        if (raw & USBPRO_STATUS_BUS_S)
            stbits |= PCAN_ERROR_BUSOFF;
        else if (raw & USBPRO_STATUS_ERROR_S)
            stbits |= PCAN_ERROR_BUSWARNING;
        if (raw & (USBPRO_STATUS_OVERRUN_S | USBPRO_STATUS_QOVERRUN_S))
            d->ch[ci].status |= PCAN_ERROR_OVERRUN;
        {
            uint32_t old_bus = d->ch[ci].status & PCAN_ERROR_ANYBUSERR;
            d->ch[ci].status = (d->ch[ci].status & ~PCAN_ERROR_ANYBUSERR) | stbits;
            if (stbits == old_bus)
                return;
        }
        memset(&m, 0, sizeof(m));
        m.MSGTYPE = PCAN_MESSAGE_STATUS;
        m.ID = d->ch[ci].status;
        deliver(&d->ch[ci], &m, usbpro_ts64(d, rd_le32(&er->timestamp32)));
        return;
    }
    case USBPRO_CALIBRATION_RX:
    case USBPRO_BUSLOAD_RX:
        return;
    default:
        return;
    }
}

static void decode_usbpro(struct pcan_device *d, uint8_t *data, int len)
{
    uint8_t *ptr;
    int remain, rec_count, sz;
    uint8_t tmp[280];

    if (d->frag_len > 0) {
        int need, take;
        if (d->frag_len >= (int)sizeof(d->frag)) {
            d->frag_len = 0;
        } else {
            sz = usbpro_sizeof_rec(d->frag[0]);
            if (sz <= 0) {
                d->frag_len = 0;
            } else {
                need = sz - d->frag_len;
                take = need > len ? len : need;
                memcpy(tmp, d->frag, (size_t)d->frag_len);
                memcpy(tmp + d->frag_len, data, (size_t)take);
                if (d->frag_len + take < sz) {
                    memcpy(d->frag + d->frag_len, data, (size_t)take);
                    d->frag_len += take;
                    return;
                }
                decode_usbpro_rec(d, tmp);
                data += take;
                len -= take;
                d->frag_len = 0;
            }
        }
    }

    if (len < 4)
        return;
    rec_count = (int)rd_le16(data);
    ptr = data + 4;
    remain = len - 4;
    while (rec_count-- > 0) {
        if (remain < 1)
            break;
        sz = usbpro_sizeof_rec(ptr[0]);
        if (sz <= 0) {
            pcan_log("usbpro unknown rec 0x%02x", ptr[0]);
            break;
        }
        if (sz > remain) {
            if (remain <= (int)sizeof(d->frag)) {
                memcpy(d->frag, ptr, (size_t)remain);
                d->frag_len = remain;
            }
            break;
        }
        decode_usbpro_rec(d, ptr);
        ptr += sz;
        remain -= sz;
    }
}

/* decode_buf only guarantees a record is at least 4 bytes and no longer than
 * it claims, so every field read here is bounded by the record's own size.
 * The record type selects the layout; the size decides what may be touched. */
static void decode_one(struct pcan_device *d, uint8_t *ptr, uint16_t size)
{
    struct canfd_msg *hdr = (struct canfd_msg *)ptr;
    uint16_t type;
    uint64_t ts;
    int ci = 0;
    TPCANMsgFD m;
    struct pcan_channel *ch;

    if ((size_t)size < sizeof(struct canfd_msg))
        return;
    type = hdr->type;
    ts = (uint64_t)hdr->ts_low | ((uint64_t)hdr->ts_high << 32);

    switch (type) {
    case CANFD_MSG_CAN_RX: {
        struct canfd_rx_msg *rx = (struct canfd_rx_msg *)ptr;
        uint16_t flags;
        int len, avail;
        /* The fixed part runs up to d[]; the payload is whatever follows. */
        if ((size_t)size < offsetof(struct canfd_rx_msg, d))
            return;
        ci = CANFD_MSG_CHANNEL(rx);
        if (ci < 0 || ci >= d->can_count)
            return;
        flags = rx->flags;
        memset(&m, 0, sizeof(m));
        m.ID = rx->can_id;
        m.DLC = CANFD_MSG_DLC(rx);
        if (flags & CANFD_MSG_EXT_ID)
            m.MSGTYPE |= PCAN_MESSAGE_EXTENDED;
        if (flags & CANFD_MSG_RTR)
            m.MSGTYPE |= PCAN_MESSAGE_RTR;
        if (flags & CANFD_MSG_EXT_DATA_LEN)
            m.MSGTYPE |= PCAN_MESSAGE_FD;
        if (flags & CANFD_MSG_BITRATE_SWITCH)
            m.MSGTYPE |= PCAN_MESSAGE_BRS;
        if (flags & CANFD_MSG_ERROR_STATE_IND)
            m.MSGTYPE |= PCAN_MESSAGE_ESI;
        if (flags & CANFD_MSG_API_SRR)
            m.MSGTYPE |= PCAN_MESSAGE_ECHO;
        len = (flags & CANFD_MSG_EXT_DATA_LEN) ? pcan_dlc2len(m.DLC)
                                               : (m.DLC > 8 ? 8 : m.DLC);
        if (len > 64)
            len = 64;
        /* The DLC is the sender's claim; the record length is the truth. */
        avail = (int)size - (int)offsetof(struct canfd_rx_msg, d);
        if (len > avail)
            len = avail;
        if (len > 0)
            memcpy(m.DATA, rx->d, (size_t)len);
        deliver(&d->ch[ci], &m, ts);
        return;
    }
    case CANFD_MSG_STATUS: {
        struct canfd_status_msg *st = (struct canfd_status_msg *)ptr;
        uint32_t stbits = 0;
        if ((size_t)size < sizeof(*st))
            return;
        ci = CANFD_STMSG_CHANNEL(st);
        if (ci < 0 || ci >= d->can_count)
            return;
        ch = &d->ch[ci];
        if (CANFD_STMSG_BUSOFF(st))
            stbits |= PCAN_ERROR_BUSOFF;
        else if (CANFD_STMSG_PASSIVE(st))
            stbits |= PCAN_ERROR_BUSPASSIVE;
        else if (CANFD_STMSG_WARNING(st))
            stbits |= PCAN_ERROR_BUSWARNING;
        {
            uint32_t old_bus = ch->status & PCAN_ERROR_ANYBUSERR;
            ch->status = (ch->status & ~PCAN_ERROR_ANYBUSERR) | stbits;
            /* Same as peak-linux: do not queue a status that did not change
             * (firmware sends OK on every bus-on / mode switch). */
            if (stbits == old_bus)
                return;
        }
        memset(&m, 0, sizeof(m));
        m.MSGTYPE = PCAN_MESSAGE_STATUS;
        m.ID = ch->status;
        m.DLC = 0;
        deliver(ch, &m, ts);
        return;
    }
    case CANFD_MSG_ERROR: {
        struct canfd_error_msg *er = (struct canfd_error_msg *)ptr;
        if ((size_t)size < sizeof(*er))
            return;
        ci = CANFD_ERMSG_CHANNEL(er);
        if (ci < 0 || ci >= d->can_count)
            return;
        ch = &d->ch[ci];
        ch->status |= PCAN_ERROR_OVERRUN;
        ch->rx_err_cnt = er->rx_err_cnt;
        ch->tx_err_cnt = er->tx_err_cnt;
        memset(&m, 0, sizeof(m));
        m.MSGTYPE = PCAN_MESSAGE_ERRFRAME;
        m.ID = er->code_g;
        m.DATA[0] = er->rx_err_cnt;
        m.DATA[1] = er->tx_err_cnt;
        deliver(ch, &m, ts);
        return;
    }
    case CANFD_USB_MSG_OVERRUN: {
        struct ucan_usb_ovr_msg *ov = (struct ucan_usb_ovr_msg *)ptr;
        if ((size_t)size < sizeof(*ov))
            return;
        ci = CANFD_USB_OVMSG_CHANNEL(ov);
        if (ci < 0 || ci >= d->can_count)
            return;
        d->ch[ci].status |= PCAN_ERROR_QOVERRUN;
        return;
    }
    case CANFD_USB_MSG_CALIBRATION:
    case CANFD_MSG_BUSLOAD:
    case CANFD_CMD_END_OF_COLLECTION:
    case 0xffff:
        return;
    default:
        pcan_log("unknown rec type 0x%03x size %u", type, size);
        return;
    }
}

static void decode_buf(struct pcan_device *d, uint8_t *data, int len)
{
    uint8_t *ptr = data;
    int remain = len;

    if (d->family == PCAN_FAM_USB) {
        /* The classic adapter emits fixed 64-byte IN packets, and one bulk
         * completion aggregates as many as fit. Each packet carries its own
         * record-count header and restarts the timestamp scheme, so it has to
         * be decoded packet by packet: handing the whole URB to the decoder
         * loses everything after the first packet. */
        int off;
        for (off = 0; off < len; off += PCAN_USB_PKT_SIZE) {
            int n = len - off;
            if (n > PCAN_USB_PKT_SIZE)
                n = PCAN_USB_PKT_SIZE;
            decode_classic(d, data + off, n);
        }
        return;
    }
    if (d->family == PCAN_FAM_USBPRO) {
        decode_usbpro(d, data, len);
        return;
    }

    if (d->frag_need > 0) {
        int take = d->frag_need - d->frag_len;
        if (take > remain)
            take = remain;
        if (d->frag_len + take <= (int)sizeof(d->frag))
            memcpy(d->frag + d->frag_len, ptr, (size_t)take);
        d->frag_len += take;
        ptr += take;
        remain -= take;
        if (d->frag_len >= d->frag_need) {
            decode_one(d, d->frag, (uint16_t)d->frag_need);
            d->frag_len = d->frag_need = 0;
        }
    }

    while (remain >= 4) {
        uint16_t size = ptr[0] | (ptr[1] << 8);
        if (size == 0)
            break;
        if (size < 4 || size > 256)
            break;
        if (size > remain) {
            d->frag_need = size;
            d->frag_len = remain;
            if (remain <= (int)sizeof(d->frag))
                memcpy(d->frag, ptr, (size_t)remain);
            break;
        }
        /* XL adapters carry XL frames alongside the shared CC/FD records. */
        if (d->family == PCAN_FAM_XL && ptr[2] == CANXL_RX_MSG_XL)
            pcan_xl_decode(d, ptr, size);
        else
            decode_one(d, ptr, size);
        ptr += size;
        remain -= size;
    }
}

static void LIBUSB_CALL rx_cb(struct libusb_transfer *xfer)
{
    struct pcan_device *d = xfer->user_data;

    if (xfer->status == LIBUSB_TRANSFER_COMPLETED && xfer->actual_length > 0)
        decode_buf(d, xfer->buffer, xfer->actual_length);

    if (d->rx_running &&
        xfer->status != LIBUSB_TRANSFER_NO_DEVICE &&
        xfer->status != LIBUSB_TRANSFER_CANCELLED) {
        if (libusb_submit_transfer(xfer) == 0)
            return;
        pcan_log("resubmit failed, dropping URB");
    }
    /* The transfer is ours again; stop_rx() waits for this count to reach 0
     * before freeing anything. */
    d->rx_pending--;
}

static void *rx_thread(void *arg)
{
    struct pcan_device *d = arg;
    while (d->rx_running) {
        struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
        int err = libusb_handle_events_timeout(g_ctx, &tv);
        if (err && err != LIBUSB_ERROR_TIMEOUT && err != LIBUSB_ERROR_INTERRUPTED)
            pcan_log("handle_events: %s", libusb_strerror(err));
    }
    return NULL;
}

static void free_rx_urbs(struct pcan_device *d)
{
    int i;
    for (i = 0; i < PEAKUSB_RX_URBS; i++) {
        if (d->rx_xfer[i]) {
            libusb_free_transfer(d->rx_xfer[i]);
            d->rx_xfer[i] = NULL;
        }
        free(d->rx_buf[i]);
        d->rx_buf[i] = NULL;
    }
}

/* Keep several IN transfers in flight at all times. This is what the closed
 * 0.13 library got wrong: with no buffer posted the adapter stops streaming
 * and frames only show up after a replug. */
static int start_rx(struct pcan_device *d)
{
    int i, err;

    d->frag_len = d->frag_need = 0;
    d->rx_pending = 0;
    d->rx_running = 1;
    for (i = 0; i < PEAKUSB_RX_URBS; i++) {
        d->rx_buf[i] = calloc(1, PEAKUSB_RX_URB_SIZE);
        d->rx_xfer[i] = libusb_alloc_transfer(0);
        if (!d->rx_buf[i] || !d->rx_xfer[i])
            goto fail;
        libusb_fill_bulk_transfer(d->rx_xfer[i], d->h, d->ep_data_in,
                                  d->rx_buf[i], PEAKUSB_RX_URB_SIZE,
                                  rx_cb, d, 0);
        err = libusb_submit_transfer(d->rx_xfer[i]);
        if (err) {
            pcan_log("submit urb %d: %s", i, libusb_strerror(err));
            goto fail;
        }
        d->rx_pending++;
    }
    if (pthread_create(&d->rx_thread, NULL, rx_thread, d) != 0)
        goto fail;
    pcan_log("RX thread started (%d x %d bytes)", PEAKUSB_RX_URBS, PEAKUSB_RX_URB_SIZE);
    return 0;

fail:
    d->rx_running = 0;
    for (i = 0; i < PEAKUSB_RX_URBS; i++)
        if (d->rx_xfer[i] && d->rx_pending > 0)
            libusb_cancel_transfer(d->rx_xfer[i]);
    for (i = 0; i < 50 && d->rx_pending > 0; i++) {
        struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 };
        libusb_handle_events_timeout(g_ctx, &tv);
    }
    free_rx_urbs(d);
    return -1;
}

static void stop_rx(struct pcan_device *d)
{
    int i;

    if (!d->rx_running && !d->rx_thread)
        return;
    d->rx_running = 0;
    for (i = 0; i < PEAKUSB_RX_URBS; i++)
        if (d->rx_xfer[i])
            libusb_cancel_transfer(d->rx_xfer[i]);
    if (d->rx_thread) {
        pthread_join(d->rx_thread, NULL);
        d->rx_thread = 0;
    }
    /* Only this thread pumps events now; wait for every cancellation callback
     * before the transfers and their buffers go away. */
    for (i = 0; i < 100 && d->rx_pending > 0; i++) {
        struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 };
        libusb_handle_events_timeout(g_ctx, &tv);
    }
    if (d->rx_pending > 0)
        pcan_log("%d URB(s) still in flight after cancel", d->rx_pending);
    free_rx_urbs(d);
}

static struct pcan_device *probe_device(libusb_device *raw)
{
    struct libusb_device_descriptor desc;
    libusb_device_handle *h = NULL;
    struct pcan_usbfd_fw_info pfi;
    struct pcan_device *d;
    int i, err;

    if (libusb_get_device_descriptor(raw, &desc) != 0)
        return NULL;
    if (desc.idVendor != PCAN_USB_VENDOR_ID || !is_peak_pid(desc.idProduct))
        return NULL;

    d = calloc(1, sizeof(*d));
    if (!d)
        return NULL;
    d->raw = raw;
    libusb_ref_device(raw);
    d->bus = libusb_get_bus_number(raw);
    d->addr = libusb_get_device_address(raw);
    d->pid = desc.idProduct;
    d->family = family_from_pid(d->pid);
    snprintf(d->name, sizeof(d->name), "%s", pid_name(d->pid));
    d->can_count = pid_can_count(d->pid);
    d->ep_cmd_out = 0x01;
    d->ep_cmd_in = 0x81;
    d->ep_data_in = 0x82;
    d->ep_data_out[0] = 0x02;
    d->ep_data_out[1] = 0x03;
    pthread_mutex_init(&d->cmd_lock, NULL);

    err = libusb_open(raw, &h);
    if (err == 0) {
        if (d->family == PCAN_FAM_FD) {
            if (read_fw_info(h, &pfi) >= 0)
                apply_fw_info(d, &pfi);
        } else if (d->family == PCAN_FAM_USBPRO) {
            usbpro_read_info(h, d);
        }
        libusb_close(h);
    } else {
        pcan_log("open for probe failed (%s), device may be busy",
                 libusb_strerror(err));
    }

    for (i = 0; i < d->can_count; i++) {
        d->ch[i].dev = d;
        d->ch[i].can_idx = i;
        d->ch[i].allow_status = 1;
        d->ch[i].allow_rtr = 1;
        d->ch[i].allow_error = 0;
        d->ch[i].receive_on = 1;
        d->ch[i].filter_mode = PCAN_FILTER_OPEN;
        d->ch[i].ev_r = d->ch[i].ev_w = -1;
        d->ch[i].device_id = d->dev_id[i];
    }
    return d;
}

/* Stop the receive thread and give the interface back. Safe to call on a
 * device that was never claimed, and idempotent. */
void pcan_usb_release_device(struct pcan_device *d)
{
    stop_rx(d);
    if (d->h) {
        libusb_release_interface(d->h, 0);
        libusb_close(d->h);
        d->h = NULL;
    }
    d->claimed = 0;
}

static void free_device(struct pcan_device *d)
{
    int c;
    if (!d)
        return;
    /* Close every channel first, since the last one stops the receive thread,
     * and only then release the queues that thread writes into. */
    for (c = 0; c < d->can_count; c++)
        if (d->ch[c].initialized)
            pcan_usb_close_channel(&d->ch[c]);
    /* A device can be claimed with the receive thread running even though no
     * channel is marked initialized: an open that failed after claiming, or
     * the window before CAN_Initialize sets the flag. Freeing the queues and
     * the device in that state hands the receive thread freed memory, so make
     * sure the transport is torn down whatever the per-channel flags say. */
    if (d->claimed)
        pcan_usb_release_device(d);
    for (c = 0; c < d->can_count; c++) {
        pcan_queue_free(&d->ch[c].q);
        pcan_xl_queue_free(&d->ch[c]);
    }
    if (d->raw)
        libusb_unref_device(d->raw);
    pthread_mutex_destroy(&d->cmd_lock);
    free(d);
}

static int cmp_dev(const void *a, const void *b)
{
    const struct pcan_device *da = *(struct pcan_device * const *)a;
    const struct pcan_device *db = *(struct pcan_device * const *)b;
    if (da->bus != db->bus)
        return (int)da->bus - (int)db->bus;
    return (int)da->addr - (int)db->addr;
}

static void rebuild_map(void)
{
    int i, c, n = 0;
    qsort(g_devs, (size_t)g_ndev, sizeof(g_devs[0]), cmp_dev);
    memset(g_map, 0, sizeof(g_map));
    for (i = 0; i < g_ndev && n < PEAKUSB_MAX_HANDLES; i++) {
        for (c = 0; c < g_devs[i]->can_count && n < PEAKUSB_MAX_HANDLES; c++) {
            g_map[n].handle = k_usb_handles[n];
            g_map[n].dev = g_devs[i];
            g_map[n].can_idx = c;
            g_map[n].present = 1;
            n++;
        }
    }
    g_nmap = n;
}

int pcan_usb_global_init(void)
{
    const char *e;
    pthread_mutex_lock(&g_lock);
    if (g_inited) {
        pthread_mutex_unlock(&g_lock);
        return 0;
    }
    e = getenv("PEAKUSB_DEBUG");
    g_debug = (e && *e && *e != '0');
    if (libusb_init(&g_ctx) != 0) {
        pthread_mutex_unlock(&g_lock);
        return -1;
    }
    g_inited = 1;
    pthread_mutex_unlock(&g_lock);
    return pcan_usb_refresh();
}

void pcan_usb_global_exit(void)
{
    int i;
    pthread_mutex_lock(&g_lock);
    for (i = 0; i < g_ndev; i++) {
        free_device(g_devs[i]);
        g_devs[i] = NULL;
    }
    g_ndev = 0;
    g_nmap = 0;
    if (g_ctx) {
        libusb_exit(g_ctx);
        g_ctx = NULL;
    }
    g_inited = 0;
    pthread_mutex_unlock(&g_lock);
}

int pcan_usb_refresh(void)
{
    libusb_device **list = NULL;
    ssize_t n, i;

    if (pcan_usb_global_init() != 0)
        return -1;

    pthread_mutex_lock(&g_lock);
    if (any_channel_open()) {
        pthread_mutex_unlock(&g_lock);
        return 0;
    }

    for (i = 0; i < g_ndev; i++) {
        free_device(g_devs[i]);
        g_devs[i] = NULL;
    }
    g_ndev = 0;

    n = libusb_get_device_list(g_ctx, &list);
    if (n < 0) {
        pthread_mutex_unlock(&g_lock);
        return -1;
    }
    for (i = 0; i < n && g_ndev < 8; i++) {
        struct libusb_device_descriptor desc;
        struct pcan_device *d;
        if (libusb_get_device_descriptor(list[i], &desc) != 0)
            continue;
        if (desc.idVendor != PCAN_USB_VENDOR_ID || !is_peak_pid(desc.idProduct))
            continue;
        d = probe_device(list[i]);
        if (d)
            g_devs[g_ndev++] = d;
    }
    libusb_free_device_list(list, 1);
    rebuild_map();
    pthread_mutex_unlock(&g_lock);
    pcan_log("enumerated %d adapter(s), %d channel(s)", g_ndev, g_nmap);
    return 0;
}

int pcan_usb_handle_count(void)
{
    return g_nmap;
}

struct pcan_handle *pcan_usb_lookup(TPCANHandle h)
{
    int i;
    for (i = 0; i < g_nmap; i++)
        if (g_map[i].present && g_map[i].handle == h)
            return &g_map[i];
    return NULL;
}

struct pcan_channel *pcan_usb_channel(TPCANHandle h)
{
    struct pcan_handle *m = pcan_usb_lookup(h);
    if (!m || !m->dev)
        return NULL;
    return &m->dev->ch[m->can_idx];
}

int pcan_usb_open_channel(struct pcan_channel *ch)
{
    struct pcan_device *d = ch->dev;
    int err, fds[2];

    if (!d)
        return -1;

    if (!d->claimed) {
        err = libusb_open(d->raw, &d->h);
        if (err) {
            pcan_log("libusb_open: %s", libusb_strerror(err));
            return err;
        }
        libusb_set_auto_detach_kernel_driver(d->h, 1);
        err = libusb_claim_interface(d->h, 0);
        if (err) {
            pcan_log("claim_interface: %s", libusb_strerror(err));
            libusb_close(d->h);
            d->h = NULL;
            return err;
        }
        {
            if (d->family == PCAN_FAM_FD) {
                struct pcan_usbfd_fw_info pfi;
                if (read_fw_info(d->h, &pfi) >= 0)
                    apply_fw_info(d, &pfi);
                driver_loaded(d, 1);
            } else if (d->family == PCAN_FAM_USBPRO) {
                usbpro_read_info(d->h, d);
                driver_loaded(d, 1);
            }
        }
        if (start_rx(d) != 0) {
            if (d->family != PCAN_FAM_USB)
                driver_loaded(d, 0);
            libusb_release_interface(d->h, 0);
            libusb_close(d->h);
            d->h = NULL;
            return -1;
        }
        if (d->family == PCAN_FAM_FD) {
            send_led(d, 0, CANFD_USB_LED_DEV);
            if (d->can_count > 1)
                send_led(d, 1, CANFD_USB_LED_DEV);
            /* enable USB calibration stream so IN URBs keep completing */
            {
                uint8_t buf[PEAKUSB_CMD_BUF];
                size_t len = 0;
                struct canfd_option *opt =
                    pcan_cmd_add(buf, &len, d->can_count - 1, CANFD_CMD_SET_EN_OPTION);
                if (opt) {
                    opt->mask = 0;
                    opt->ext_mask = CANFD_USB_OPTION_CALIBRATION;
                    pcan_cmd_send(d, buf, &len);
                }
            }
        } else if (d->family == PCAN_FAM_USBPRO) {
            usbpro_led(d, 0, USBPRO_LED_DEVICE);
            if (d->can_count > 1)
                usbpro_led(d, 1, USBPRO_LED_DEVICE);
            usbpro_calibration(d, 1);
        } else {
            classic_read_ids(d);
        }
        d->claimed = 1;
    }

    if (!ch->q.items) {
        if (pcan_queue_init(&ch->q, PEAKUSB_RX_QUEUE) != 0)
            return -1;
    } else {
        pcan_queue_clear(&ch->q);
    }

    if (ch->ev_r < 0) {
        if (pipe(fds) != 0)
            return -1;
        fcntl(fds[0], F_SETFL, O_NONBLOCK);
        fcntl(fds[1], F_SETFL, O_NONBLOCK);
        ch->ev_r = fds[0];
        ch->ev_w = fds[1];
    }
    return 0;
}

void pcan_usb_close_channel(struct pcan_channel *ch)
{
    struct pcan_device *d;
    int c, still = 0;

    if (!ch || !ch->dev)
        return;
    d = ch->dev;
    if (ch->bus_on)
        pcan_usb_bus_off(ch);
    ch->initialized = 0;
    ch->bus_on = 0;
    pcan_queue_clear(&ch->q);
    if (ch->ev_r >= 0) {
        close(ch->ev_r);
        ch->ev_r = -1;
    }
    if (ch->ev_w >= 0) {
        close(ch->ev_w);
        ch->ev_w = -1;
    }

    for (c = 0; c < d->can_count; c++)
        if (d->ch[c].initialized)
            still++;
    if (still || !d->claimed)
        return;

    if (d->family == PCAN_FAM_FD) {
        send_simple(d, 0, CANFD_CMD_RESET_MODE);
        if (d->can_count > 1)
            send_simple(d, 1, CANFD_CMD_RESET_MODE);
        send_led(d, 0, CANFD_USB_LED_OFF);
        if (d->can_count > 1)
            send_led(d, 1, CANFD_USB_LED_OFF);
        {
            uint8_t buf[PEAKUSB_CMD_BUF];
            size_t len = 0;
            struct canfd_option *opt =
                pcan_cmd_add(buf, &len, d->can_count - 1, CANFD_CMD_CLR_DIS_OPTION);
            if (opt) {
                opt->mask = 0;
                opt->ext_mask = CANFD_USB_OPTION_CALIBRATION;
                pcan_cmd_send(d, buf, &len);
            }
        }
        driver_loaded(d, 0);
    } else if (d->family == PCAN_FAM_USBPRO) {
        usbpro_calibration(d, 0);
        usbpro_led(d, 0, USBPRO_LED_OFF);
        if (d->can_count > 1)
            usbpro_led(d, 1, USBPRO_LED_OFF);
        driver_loaded(d, 0);
    }
    pcan_usb_release_device(d);
}

int pcan_usb_configure(struct pcan_channel *ch)
{
    struct pcan_device *d = ch->dev;
    uint8_t buf[PEAKUSB_CMD_BUF];
    size_t len = 0;
    struct canfd_wr_err_cnt *ec;
    struct canfd_option *opt;
    struct canfd_timing_slow *slow;
    struct canfd_timing_fast *fast;
    struct canfd_std_filter *flt;
    int i, idx = ch->can_idx;
    uint16_t to_set, to_clr;

    if (d->family == PCAN_FAM_USB) {
        uint8_t btr1 = (uint8_t)(ch->btr0btr1 & 0xff);
        uint8_t btr0 = (uint8_t)(ch->btr0btr1 >> 8);
        if (classic_cmd(d, PCAN_USB_CMD_BITRATE, PCAN_USB_SET, btr1, btr0, 0, 0))
            return -1;
        classic_cmd(d, PCAN_USB_CMD_ERR_FR, PCAN_USB_SET,
                    PCAN_USB_ERR_ECC | PCAN_USB_ERR_RXERR | PCAN_USB_ERR_TXERR |
                    PCAN_USB_ERR_RXERR_CNT | PCAN_USB_ERR_TXERR_CNT, 0, 0, 0);
        if (classic_cmd(d, PCAN_USB_CMD_BUS, PCAN_USB_SET_SILENT_MODE,
                        ch->listen_only ? 1 : 0, 0, 0, 0))
            return -1;
        d->usb_ts16 = 0;
        d->usb_ts_hi = 0;
        return 0;
    }

    if (d->family == PCAN_FAM_USBPRO) {
        if (usbpro_set_baud(ch))
            return -1;
        if (usbpro_set_u16(d, USBPRO_FKT_SETSILENTMODE, idx,
                           ch->listen_only ? 1 : 0))
            return -1;
        if (usbpro_set_u16(d, USBPRO_FKT_SETFILTERMODE, idx, 1))
            return -1;
        return 0;
    }

    if (send_clock(d, idx, clock_mode(ch->bt.clock_hz ? ch->bt.clock_hz : 80000000u)))
        return -1;

    len = 0;
    ec = pcan_cmd_add(buf, &len, idx, CANFD_CMD_WR_ERR_CNT);
    if (!ec)
        return -1;
    ec->sel_mask = CANFD_WRERRCNT_TE | CANFD_WRERRCNT_RE;
    ec->tx_counter = 0;
    ec->rx_counter = 0;

    to_set = CANFD_OPTION_ERROR;
    to_clr = CANFD_OPTION_BUSLOAD;
    if (ch->bt.fd) {
        to_set |= CANFD_OPTION_ISO_MODE;
        to_clr |= CANFD_OPTION_20AB_MODE;
    } else {
        to_set |= CANFD_OPTION_20AB_MODE;
        to_clr |= CANFD_OPTION_ISO_MODE;
    }

    opt = pcan_cmd_add(buf, &len, idx, CANFD_CMD_CLR_DIS_OPTION);
    if (!opt)
        return -1;
    opt->mask = to_clr;
    opt->ext_mask = 0;

    opt = pcan_cmd_add(buf, &len, idx, CANFD_CMD_SET_EN_OPTION);
    if (!opt)
        return -1;
    opt->mask = to_set;
    opt->ext_mask = 0;

    if (ch->bt.fd) {
        fast = pcan_cmd_add(buf, &len, idx, CANFD_CMD_TIMING_FAST);
        if (!fast)
            return -1;
        fast->sjw = (uint8_t)((ch->bt.dsjw - 1) & 0x0f);
        fast->tseg1 = (uint8_t)((ch->bt.dtseg1 - 1) & 0x1f);
        fast->tseg2 = (uint8_t)((ch->bt.dtseg2 - 1) & 0x0f);
        fast->brp = (uint16_t)(ch->bt.dbrp - 1);
    }

    slow = pcan_cmd_add(buf, &len, idx, CANFD_CMD_TIMING_SLOW);
    if (!slow)
        return -1;
    slow->ewl = 96;
    slow->sjw_t = (uint8_t)(((ch->bt.sjw - 1) & 0x7f) | (ch->bt.tsam ? 0x80 : 0));
    slow->tseg1 = (uint8_t)((ch->bt.tseg1 - 1) & 0xff);
    slow->tseg2 = (uint8_t)((ch->bt.tseg2 - 1) & 0x7f);
    slow->brp = (uint16_t)(ch->bt.brp - 1);

    if (pcan_cmd_send(d, buf, &len))
        return -1;

    /* accept-all standard-ID filter rows */
    len = 0;
    for (i = 0; i < (1 << CANFD_FLTSTD_ROW_IDX_BITS); i++) {
        flt = pcan_cmd_add(buf, &len, idx, CANFD_CMD_SET_STD_FILTER);
        if (!flt) {
            if (pcan_cmd_send(d, buf, &len))
                return -1;
            flt = pcan_cmd_add(buf, &len, idx, CANFD_CMD_SET_STD_FILTER);
            if (!flt)
                return -1;
        }
        flt->idx = (uint8_t)i;
        flt->mask = 0xffffffffu;
    }
    return pcan_cmd_send(d, buf, &len);
}

int pcan_usb_bus_on(struct pcan_channel *ch)
{
    int err;

    if (ch->dev->family == PCAN_FAM_USB) {
        err = classic_cmd(ch->dev, PCAN_USB_CMD_BUS, PCAN_USB_SET, 1, 0, 0, 0);
    } else if (ch->dev->family == PCAN_FAM_USBPRO) {
        err = usbpro_set_u16(ch->dev, USBPRO_FKT_SETCANBUSACTIVATE,
                             ch->can_idx, 1);
    } else {
        int op = ch->listen_only ? CANFD_CMD_LISTEN_ONLY_MODE : CANFD_CMD_NORMAL_MODE;
        err = send_simple(ch->dev, ch->can_idx, op);
    }
    if (!err) {
        ch->bus_on = 1;
        ch->status &= ~PCAN_ERROR_ANYBUSERR;
    }
    return err;
}

/* Digital I/O, PCAN-USB Chip only. The command carries a 32-bit mask whose
 * meaning depends on the opcode: a pin-direction map for CFG, a level map for
 * VAL, or the pins to raise or lower for SET_HIGH / SET_LOW. */
int pcan_usb_io_write(struct pcan_channel *ch, uint16_t opcode, uint32_t val)
{
    struct pcan_device *d = ch->dev;
    uint8_t buf[PEAKUSB_CMD_BUF];
    struct ucan_usb_io_ctrl *io;
    size_t len = 0;

    if (d->family != PCAN_FAM_FD || d->pid != PCAN_USBCHIP_PRODUCT_ID)
        return -1;
    io = pcan_cmd_add(buf, &len, ch->can_idx, opcode);
    if (!io)
        return -1;
    io->opcode = opcode;
    io->reserved = 0;
    io->io_val = val;
    return pcan_cmd_send(d, buf, &len);
}

/* PCAN_CHANNEL_IDENTIFYING: blink this channel's LED, or hand it back to the
 * firmware's own control. Classic PCAN-USB has no addressable LED. */
int pcan_usb_identify(struct pcan_channel *ch, int on)
{
    struct pcan_device *d = ch->dev;

    if (d->family == PCAN_FAM_FD)
        return send_led(d, ch->can_idx,
                        on ? CANFD_USB_LED_OFF : CANFD_USB_LED_DEV);
    if (d->family == PCAN_FAM_USBPRO)
        return usbpro_led(d, ch->can_idx,
                          on ? USBPRO_LED_BLINK_FAST : USBPRO_LED_DEVICE);
    return -1;
}

int pcan_usb_bus_off(struct pcan_channel *ch)
{
    int err;

    /* Give the adapter time to drain frames already handed to it. PEAK's own
     * driver documents 20 ms as the minimum for the classic device and 50 ms
     * for uCAN; without it, a frame written just before close is aborted in
     * the adapter and never reaches the bus. */
    {
        struct timespec ts;
        ts.tv_sec = 0;
        ts.tv_nsec = (ch->dev->family == PCAN_FAM_FD) ? 50000000L : 20000000L;
        nanosleep(&ts, NULL);
    }

    if (ch->dev->family == PCAN_FAM_USB)
        err = classic_cmd(ch->dev, PCAN_USB_CMD_BUS, PCAN_USB_SET, 0, 0, 0, 0);
    else if (ch->dev->family == PCAN_FAM_USBPRO)
        err = usbpro_set_u16(ch->dev, USBPRO_FKT_SETCANBUSACTIVATE,
                             ch->can_idx, 0);
    else
        err = send_simple(ch->dev, ch->can_idx, CANFD_CMD_RESET_MODE);
    ch->bus_on = 0;
    return err;
}

static uint32_t bitrate_of(const struct pcan_bittiming *bt, int data_phase);

static uint64_t mono_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000ull + (uint64_t)(t.tv_nsec / 1000);
}

uint32_t pcan_usb_bitrate(const struct pcan_channel *ch, int data_phase)
{
    return bitrate_of(&ch->bt, data_phase);
}

static uint32_t bitrate_of(const struct pcan_bittiming *bt, int data_phase)
{
    uint32_t brp   = data_phase ? bt->dbrp : bt->brp;
    uint32_t tseg1 = data_phase ? bt->dtseg1 : bt->tseg1;
    uint32_t tseg2 = data_phase ? bt->dtseg2 : bt->tseg2;
    uint32_t div = brp * (1u + tseg1 + tseg2);
    if (!bt->clock_hz || !div)
        return 500000;
    return bt->clock_hz / div;
}

/* Roughly how long this frame occupies the bus, in microseconds. Bit counts
 * include worst-case stuffing, so the estimate errs towards "slower", which
 * makes the throttle below conservative rather than optimistic. */
static uint32_t frame_wire_us(const struct pcan_channel *ch, const TPCANMsgFD *m)
{
    int ext = (m->MSGTYPE & PCAN_MESSAGE_EXTENDED) != 0;
    int fd  = (m->MSGTYPE & PCAN_MESSAGE_FD) != 0;
    int len = fd ? pcan_dlc2len(m->DLC) : (m->DLC > 8 ? 8 : m->DLC);
    uint32_t nom = bitrate_of(&ch->bt, 0);
    uint32_t us;

    if (!nom)
        nom = 500000;
    if (!fd) {
        uint32_t bits = (ext ? 80u : 56u) + 10u * (uint32_t)len;
        us = (bits * 1000000u) / nom;
    } else {
        uint32_t abits = ext ? 90u : 60u;
        uint32_t dbits = 40u + 10u * (uint32_t)len;
        uint32_t dat = (m->MSGTYPE & PCAN_MESSAGE_BRS) ? bitrate_of(&ch->bt, 1) : nom;
        if (!dat)
            dat = nom;
        us = (abits * 1000000u) / nom + (dbits * 1000000u) / dat;
    }
    return us ? us : 1;
}

/* The adapter buffers transmits and, once that buffer is full, discards frames
 * without telling anyone. There is no credit or echo from the firmware to
 * count against, so the driver models the bus instead: it tracks how far ahead
 * of real time the caller has queued and refuses further frames once that lead
 * exceeds PEAKUSB_TX_LEAD_US. Expressing the limit in wire time rather than a
 * frame count keeps it independent of any particular adapter's buffer depth.
 * Refusing is the whole point: PCAN_ERROR_XMTFULL tells the application to
 * retry, where silent loss cannot be detected at all. */
int pcan_usb_tx_throttle(struct pcan_channel *ch, const TPCANMsgFD *msg)
{
    uint64_t now = mono_us();

    if (ch->tx_free_us < now)
        ch->tx_free_us = now;
    if (ch->tx_free_us - now > PEAKUSB_TX_LEAD_US)
        return 1;                       /* buffer would overflow: refuse */
    /* PCAN_INTERFRAME_DELAY widens the modelled occupancy, so the throttle
     * paces transmits at the requested rate rather than at wire speed. */
    ch->tx_free_us += frame_wire_us(ch, msg) + ch->interframe_us;
    return 0;
}

int pcan_usb_write_msg(struct pcan_channel *ch, const TPCANMsgFD *msg)
{
    struct canfd_tx_msg tx;
    uint8_t dlc, len, *pkt;
    uint16_t flags = 0;
    uint32_t size;
    int transferred = 0, err;

    if (!ch->dev || !ch->dev->h)
        return -1;
    if (ch->dev->family != PCAN_FAM_FD && (msg->MSGTYPE & PCAN_MESSAGE_FD))
        return -1;

    dlc = msg->DLC & 0x0f;
    if (msg->MSGTYPE & PCAN_MESSAGE_FD)
        len = pcan_dlc2len(dlc);
    else
        len = dlc > 8 ? 8 : dlc;

    if (ch->dev->family == PCAN_FAM_USB) {
        uint8_t buf[PCAN_USB_PKT_SIZE];
        uint8_t *p, sl;
        uint32_t id;

        memset(buf, 0, sizeof(buf));
        buf[0] = PCAN_USB_FORMAT_CODE;
        buf[1] = 1;
        p = buf + 2;
        sl = dlc > 8 ? 8 : dlc;
        if (msg->MSGTYPE & PCAN_MESSAGE_RTR)
            sl |= STLN_RTR;
        if (msg->MSGTYPE & PCAN_MESSAGE_EXTENDED) {
            sl |= STLN_EXTENDED_ID;
            *p++ = sl;
            id = msg->ID << 3;
            wr_le32(p, id);
            p += 4;
        } else {
            *p++ = sl;
            id = msg->ID << 5;
            wr_le16(p, (uint16_t)id);
            p += 2;
        }
        if (!(msg->MSGTYPE & PCAN_MESSAGE_RTR) && len) {
            memcpy(p, msg->DATA, len);
            p += (dlc > 8 ? 8 : dlc);
        }
        buf[PCAN_USB_PKT_SIZE - 1] = ch->dev->usb_tx_seq++;
        err = libusb_bulk_transfer(ch->dev->h, ch->dev->ep_data_out[0],
                                   buf, PCAN_USB_PKT_SIZE, &transferred,
                                   PEAKUSB_TX_TIMEOUT);
        if (err) {
            pcan_log("classic TX: %s", libusb_strerror(err));
            if (err == LIBUSB_ERROR_PIPE)
                libusb_clear_halt(ch->dev->h, ch->dev->ep_data_out[0]);
            return -1;
        }
        return 0;
    }

    if (ch->dev->family == PCAN_FAM_USBPRO) {
        uint8_t buf[32];
        struct pcan_usbpro_canmsg_tx rec;
        uint8_t dtype;
        int rec_len;

        memset(&rec, 0, sizeof(rec));
        if ((msg->MSGTYPE & PCAN_MESSAGE_RTR) || len == 0)
            dtype = USBPRO_TX_0;
        else if (len <= 4)
            dtype = USBPRO_TX_4;
        else
            dtype = USBPRO_TX_8;
        rec.data_type = dtype;
        rec.len = (uint8_t)((ch->can_idx << 4) | (dlc & 0x0f));
        if (msg->MSGTYPE & PCAN_MESSAGE_EXTENDED)
            rec.flags |= USBPRO_EXT;
        if (msg->MSGTYPE & PCAN_MESSAGE_RTR)
            rec.flags |= USBPRO_RTR;
        wr_le32(&rec.id, msg->ID);
        memcpy(rec.data, msg->DATA, len > 8 ? 8 : len);
        rec_len = usbpro_sizeof_rec(dtype);
        memset(buf, 0, 4);
        buf[0] = 1;
        memcpy(buf + 4, &rec, (size_t)rec_len);
        err = libusb_bulk_transfer(ch->dev->h, ch->dev->ep_data_out[ch->can_idx],
                                   buf, rec_len + 4, &transferred, PEAKUSB_TX_TIMEOUT);
        if (err) {
            pcan_log("usbpro TX: %s", libusb_strerror(err));
            if (err == LIBUSB_ERROR_PIPE)
                libusb_clear_halt(ch->dev->h, ch->dev->ep_data_out[ch->can_idx]);
            return -1;
        }
        return 0;
    }

    memset(&tx, 0, sizeof(tx));

    if (msg->MSGTYPE & PCAN_MESSAGE_EXTENDED)
        flags |= CANFD_MSG_EXT_ID;
    if (msg->MSGTYPE & PCAN_MESSAGE_RTR)
        flags |= CANFD_MSG_RTR;
    if (msg->MSGTYPE & PCAN_MESSAGE_FD)
        flags |= CANFD_MSG_EXT_DATA_LEN;
    if (msg->MSGTYPE & PCAN_MESSAGE_BRS)
        flags |= CANFD_MSG_BITRATE_SWITCH;
    if (msg->MSGTYPE & PCAN_MESSAGE_ESI)
        flags |= CANFD_MSG_ERROR_STATE_IND;
    /* PCAN_ALLOW_ECHO_FRAMES: the adapter returns a copy of the frame it
     * transmitted, flagged so the reader can tell it apart from bus traffic. */
    if (ch->allow_echo)
        flags |= CANFD_MSG_API_SRR;

    size = ucan_align4(20u + len);
    tx.size = (uint16_t)size;
    tx.type = CANFD_MSG_CAN_TX;
    tx.channel_dlc = CANFD_MSG_CHANNEL_DLC(ch->can_idx, dlc);
    tx.flags = flags;
    tx.can_id = msg->ID;
    memcpy(tx.d, msg->DATA, len);

    pkt = (uint8_t *)&tx;
    err = libusb_bulk_transfer(ch->dev->h, ch->dev->ep_data_out[ch->can_idx],
                               pkt, (int)size, &transferred, PEAKUSB_TX_TIMEOUT);
    if (err) {
        pcan_log("TX bulk: %s", libusb_strerror(err));
        if (err == LIBUSB_ERROR_PIPE)
            libusb_clear_halt(ch->dev->h, ch->dev->ep_data_out[ch->can_idx]);
        return -1;
    }
    return 0;
}

void pcan_usb_reset_queue(struct pcan_channel *ch)
{
    pcan_queue_clear(&ch->q);
    pcan_event_clear_if_empty(ch);
    ch->status &= ~PCAN_ERROR_QOVERRUN;
}

__attribute__((constructor))
static void pcan_ctor(void)
{
    (void)pcan_usb_global_init();
}

__attribute__((destructor))
static void pcan_dtor(void)
{
    pcan_usb_global_exit();
}
