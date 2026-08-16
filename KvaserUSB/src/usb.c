/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 *
 * libusb transport and public API for Kvaser USB CAN adapters.
 * Device discovery, endpoint setup, the asynchronous receive path and the
 * command/response rendezvous live here; the per-family framing is in
 * leaf.c (filo/helios) and hydra.c (mhydra).
 */
#include "kvaser_priv.h"

#include <errno.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static libusb_context *g_ctx;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static struct kv_device *g_devs[KV_MAX_DEV];
static int g_ndev;
static int g_inited;
static int g_debug;

void kv_log(const char *fmt, ...)
{
    va_list ap;
    if (!g_debug)
        return;
    fputs("KvaserUSB: ", stderr);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static void wait_init(struct kv_wait *w)
{
    memset(w, 0, sizeof(*w));
    pthread_mutex_init(&w->lock, NULL);
    pthread_cond_init(&w->cv, NULL);
}

static void wait_destroy(struct kv_wait *w)
{
    pthread_mutex_destroy(&w->lock);
    pthread_cond_destroy(&w->cv);
}

void kv_maybe_complete_wait(struct kv_device *d, uint8_t cmd, uint16_t seq,
                            const void *buf, int len)
{
    struct kv_wait *w = &d->wait;
    pthread_mutex_lock(&w->lock);
    if (w->armed && w->cmd == cmd && (!w->match_seq || w->seq == seq)) {
        int n = len < (int)sizeof(w->buf) ? len : (int)sizeof(w->buf);
        memcpy(w->buf, buf, (size_t)n);
        w->len = n;
        w->armed = 0;
        pthread_cond_signal(&w->cv);
    }
    pthread_mutex_unlock(&w->lock);
}

/* Arm before transmitting. A reply that arrives between the send and the arm
 * would otherwise be discarded, stalling the caller for the full timeout. */
void kv_arm_wait(struct kv_device *d, uint8_t cmd, uint16_t seq, int match_seq)
{
    struct kv_wait *w = &d->wait;
    pthread_mutex_lock(&w->lock);
    w->cmd = cmd;
    w->seq = seq;
    w->match_seq = match_seq;
    w->len = 0;
    w->armed = 1;
    pthread_mutex_unlock(&w->lock);
}

int kv_wait_cmd(struct kv_device *d, uint8_t cmd, uint16_t seq, int match_seq,
                void *out, int out_len, int timeout_ms)
{
    struct timespec ts;
    int rc;
    struct kv_wait *w = &d->wait;

    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec += (timeout_ms % 1000) * 1000000L;
    ts.tv_sec += timeout_ms / 1000 + ts.tv_nsec / 1000000000L;
    ts.tv_nsec %= 1000000000L;

    pthread_mutex_lock(&w->lock);
    if (!(w->armed && w->cmd == cmd && w->seq == seq &&
          w->match_seq == match_seq) && w->len == 0) {
        w->cmd = cmd;
        w->seq = seq;
        w->match_seq = match_seq;
        w->len = 0;
        w->armed = 1;
    }
    rc = 0;
    while (w->armed && rc == 0)
        rc = pthread_cond_timedwait(&w->cv, &w->lock, &ts);
    w->armed = 0;
    /* A reply that already landed counts even if the deadline expired while
     * the condition variable was reacquiring the mutex, which POSIX permits. */
    if (w->len > 0) {
        int n = w->len < out_len ? w->len : out_len;
        if (out)
            memcpy(out, w->buf, (size_t)n);
        pthread_mutex_unlock(&w->lock);
        return KV_OK;
    }
    pthread_mutex_unlock(&w->lock);
    return KV_ERR_TIMEOUT;
}

int kv_send_raw(struct kv_device *d, const void *data, int len)
{
    int transferred = 0, err;
    if (!d->h)
        return KV_ERR_NO_DEVICE;
    pthread_mutex_lock(&d->io_lock);
    err = libusb_bulk_transfer(d->h, d->ep_out, (unsigned char *)data, len,
                               &transferred, KV_TX_TIMEOUT);
    pthread_mutex_unlock(&d->io_lock);
    if (err) {
        kv_log("bulk out: %s", libusb_strerror(err));
        return KV_ERR_IO;
    }
    return KV_OK;
}

static void handle_bytes(struct kv_device *d, uint8_t *data, int len)
{
    uint8_t *p = data;
    int remain = len;

    if (d->family == KV_FAMILY_HYDRA) {
        if (d->leftover_len > 0) {
            int need, take;

            /* An extended command carries its length at bytes 4..5, so the
             * first 8 bytes must be in hand before the total is known. */
            if (d->leftover_len < 8) {
                take = 8 - d->leftover_len;
                if (take > remain)
                    take = remain;
                memcpy(d->leftover + d->leftover_len, p, (size_t)take);
                d->leftover_len += take;
                p += take;
                remain -= take;
                if (d->leftover_len < 8)
                    return;
            }

            if (d->leftover[0] == 255)
                need = d->leftover[4] | (d->leftover[5] << 8);
            else
                need = KV_HYDRA_CMD;
            if (need < 8 || need > KV_HYDRA_MAX)
                need = KV_HYDRA_CMD;

            if (d->leftover_len < need) {
                take = need - d->leftover_len;
                if (take > remain)
                    take = remain;
                memcpy(d->leftover + d->leftover_len, p, (size_t)take);
                d->leftover_len += take;
                p += take;
                remain -= take;
            }
            if (d->leftover_len < need)
                return;
            kv_hydra_handle(d, d->leftover, need);
            d->leftover_len = 0;
        }
        while (remain > 0) {
            int need;
            /* Too short to read the length field: carry it to the next URB
             * rather than dropping it, which would desynchronise the stream. */
            if (remain < 8) {
                memcpy(d->leftover, p, (size_t)remain);
                d->leftover_len = remain;
                break;
            }
            if (p[0] == 255)
                need = p[4] | (p[5] << 8);
            else
                need = KV_HYDRA_CMD;
            if (need < 8 || need > KV_HYDRA_MAX)
                need = KV_HYDRA_CMD;
            if (need > remain) {
                memcpy(d->leftover, p, (size_t)remain);
                d->leftover_len = remain;
                break;
            }
            kv_hydra_handle(d, p, need);
            p += need;
            remain -= need;
        }
        return;
    }

    /* Filo / Helios: cmdLen is the first byte. Do not straddle max packet. */
    while (remain > 0) {
        int clen = p[0];
        if (clen < 2 || clen > KV_FILO_MAX) {
            /* skip to next packet boundary */
            int skip = d->maxpkt ? (remain % d->maxpkt) : remain;
            if (skip == 0)
                skip = remain;
            p += skip;
            remain -= skip;
            continue;
        }
        if (clen > remain)
            break;
        if (d->family == KV_FAMILY_HELIOS)
            kv_helios_handle(d, p, clen);
        else
            kv_leaf_handle(d, p, clen);
        p += clen;
        remain -= clen;
    }
}

static void LIBUSB_CALL rx_cb(struct libusb_transfer *xfer)
{
    struct kv_device *d = xfer->user_data;

    if (xfer->status == LIBUSB_TRANSFER_COMPLETED && xfer->actual_length > 0)
        handle_bytes(d, xfer->buffer, xfer->actual_length);

    if (d->rx_running &&
        xfer->status != LIBUSB_TRANSFER_NO_DEVICE &&
        xfer->status != LIBUSB_TRANSFER_CANCELLED) {
        if (libusb_submit_transfer(xfer) == 0)
            return;
        kv_log("resubmit failed, dropping URB");
    }
    /* The transfer is ours again; stop_rx() waits for this count to reach 0
     * before freeing anything. */
    d->rx_pending--;
}

static void *rx_thread(void *arg)
{
    struct kv_device *d = arg;
    while (d->rx_running) {
        struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
        int err = libusb_handle_events_timeout(g_ctx, &tv);
        if (err && err != LIBUSB_ERROR_TIMEOUT && err != LIBUSB_ERROR_INTERRUPTED)
            kv_log("handle_events: %s", libusb_strerror(err));
    }
    return NULL;
}

static void free_rx_urbs(struct kv_device *d)
{
    int i;
    for (i = 0; i < KV_RX_URBS; i++) {
        if (d->rx_xfer[i]) {
            libusb_free_transfer(d->rx_xfer[i]);
            d->rx_xfer[i] = NULL;
        }
        free(d->rx_buf[i]);
        d->rx_buf[i] = NULL;
    }
}

/* Keep several IN transfers in flight at all times: the adapter only streams
 * frames while the host has buffers posted. */
static int start_rx(struct kv_device *d)
{
    int i, err;

    d->leftover_len = 0;
    d->rx_pending = 0;
    d->rx_running = 1;
    for (i = 0; i < KV_RX_URBS; i++) {
        d->rx_buf[i] = calloc(1, KV_RX_URB_SIZE);
        d->rx_xfer[i] = libusb_alloc_transfer(0);
        if (!d->rx_buf[i] || !d->rx_xfer[i])
            goto fail;
        libusb_fill_bulk_transfer(d->rx_xfer[i], d->h, d->ep_in,
                                  d->rx_buf[i], KV_RX_URB_SIZE, rx_cb, d, 0);
        err = libusb_submit_transfer(d->rx_xfer[i]);
        if (err) {
            kv_log("submit urb %d: %s", i, libusb_strerror(err));
            goto fail;
        }
        d->rx_pending++;
    }
    if (pthread_create(&d->rx_thread, NULL, rx_thread, d) != 0)
        goto fail;
    return 0;

fail:
    d->rx_running = 0;
    for (i = 0; i < KV_RX_URBS; i++)
        if (d->rx_xfer[i] && d->rx_pending > 0)
            libusb_cancel_transfer(d->rx_xfer[i]);
    for (i = 0; i < 50 && d->rx_pending > 0; i++) {
        struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 };
        libusb_handle_events_timeout(g_ctx, &tv);
    }
    free_rx_urbs(d);
    return -1;
}

static void stop_rx(struct kv_device *d)
{
    int i;

    if (!d->rx_running && !d->rx_thread)
        return;
    d->rx_running = 0;
    for (i = 0; i < KV_RX_URBS; i++)
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
        kv_log("%d URB(s) still in flight after cancel", d->rx_pending);
    free_rx_urbs(d);
}

static int find_endpoints(libusb_device *raw, uint8_t *ep_in, uint8_t *ep_out, uint16_t *maxpkt)
{
    struct libusb_config_descriptor *cfg = NULL;
    const struct libusb_interface_descriptor *alt;
    int i;
    if (libusb_get_active_config_descriptor(raw, &cfg) != 0)
        return -1;
    if (cfg->bNumInterfaces < 1 || cfg->interface[0].num_altsetting < 1) {
        libusb_free_config_descriptor(cfg);
        return -1;
    }
    alt = &cfg->interface[0].altsetting[0];
    *ep_in = *ep_out = 0;
    *maxpkt = 64;
    for (i = 0; i < alt->bNumEndpoints; i++) {
        const struct libusb_endpoint_descriptor *ep = &alt->endpoint[i];
        if ((ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) != LIBUSB_TRANSFER_TYPE_BULK)
            continue;
        if ((ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) && !*ep_in) {
            *ep_in = ep->bEndpointAddress;
            *maxpkt = ep->wMaxPacketSize;
        } else if (!(ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) && !*ep_out) {
            *ep_out = ep->bEndpointAddress;
        }
    }
    libusb_free_config_descriptor(cfg);
    return (*ep_in && *ep_out) ? 0 : -1;
}

static uint32_t make_handle(uint8_t bus, uint8_t addr, uint8_t ch)
{
    return ((uint32_t)bus << 16) | ((uint32_t)addr << 8) | ch;
}

static struct kv_device *dev_for(uint32_t handle)
{
    uint8_t bus = (uint8_t)(handle >> 16);
    uint8_t addr = (uint8_t)(handle >> 8);
    int i;
    for (i = 0; i < g_ndev; i++)
        if (g_devs[i]->bus == bus && g_devs[i]->addr == addr)
            return g_devs[i];
    return NULL;
}

static struct kv_channel *ch_for(uint32_t handle)
{
    struct kv_device *d = dev_for(handle);
    int idx = (int)(handle & 0xFF);
    if (!d || idx < 0 || idx >= d->can_count)
        return NULL;
    return &d->ch[idx];
}

static int global_init(void)
{
    const char *dbg;
    if (g_inited)
        return 0;
    dbg = getenv("KVASERUSB_DEBUG");
    g_debug = dbg && *dbg && strcmp(dbg, "0") != 0;
    if (libusb_init(&g_ctx) != 0)
        return -1;
    g_inited = 1;
    return 0;
}

static struct kv_device *probe(libusb_device *raw)
{
    struct libusb_device_descriptor desc;
    const struct kv_product *prod;
    struct kv_device *d;
    int i;

    if (libusb_get_device_descriptor(raw, &desc) != 0)
        return NULL;
    if (desc.idVendor != KV_VID)
        return NULL;
    prod = kv_lookup_pid(desc.idProduct);
    if (!prod)
        return NULL;

    d = calloc(1, sizeof(*d));
    if (!d)
        return NULL;
    d->raw = raw;
    libusb_ref_device(raw);
    d->bus = libusb_get_bus_number(raw);
    d->addr = libusb_get_device_address(raw);
    d->pid = desc.idProduct;
    d->family = prod->family;
    d->fd_capable = prod->fd;
    d->can_count = prod->channels;
    snprintf(d->name, sizeof(d->name), "%s", prod->name);
    pthread_mutex_init(&d->io_lock, NULL);
    wait_init(&d->wait);
    find_endpoints(raw, &d->ep_in, &d->ep_out, &d->maxpkt);
    for (i = 0; i < KV_MAX_CAN; i++) {
        d->ch[i].dev = d;
        d->ch[i].idx = i;
        d->channel2he[i] = ILLEGAL_HE;
    }
    memset(d->he2channel, 0xFF, sizeof(d->he2channel));
    return d;
}

static void free_device(struct kv_device *d)
{
    int i;
    if (!d)
        return;
    /* Bus-off first, because it needs the receive thread for the response.
     * Then tear down the receive path, and only then release the queues
     * it feeds. */
    for (i = 0; i < KV_MAX_CAN; i++) {
        if (d->ch[i].open) {
            if (d->family == KV_FAMILY_HYDRA)
                kv_hydra_bus_off(&d->ch[i]);
            else if (d->family == KV_FAMILY_HELIOS)
                kv_helios_bus_off(&d->ch[i]);
            else
                kv_leaf_bus_off(&d->ch[i]);
        }
    }
    stop_rx(d);
    for (i = 0; i < KV_MAX_CAN; i++)
        kv_queue_free(&d->ch[i].q);
    if (d->claimed && d->h) {
        libusb_release_interface(d->h, 0);
        libusb_close(d->h);
        d->h = NULL;
    }
    if (d->raw)
        libusb_unref_device(d->raw);
    wait_destroy(&d->wait);
    pthread_mutex_destroy(&d->io_lock);
    free(d);
}

static int refresh(void)
{
    libusb_device **list = NULL;
    ssize_t n, i;
    struct kv_device *kept[KV_MAX_DEV];
    int nkept = 0, di;

    if (global_init())
        return -1;
    n = libusb_get_device_list(g_ctx, &list);
    if (n < 0)
        return -1;

    /* Carry already-known devices over so open channels survive a rescan;
     * anything left in g_devs[] afterwards has been unplugged. */
    for (i = 0; i < n && nkept < KV_MAX_DEV; i++) {
        struct libusb_device_descriptor desc;
        struct kv_device *dev = NULL;
        int k;
        if (libusb_get_device_descriptor(list[i], &desc) != 0)
            continue;
        if (desc.idVendor != KV_VID || !kv_lookup_pid(desc.idProduct))
            continue;
        for (k = 0; k < g_ndev; k++) {
            if (!g_devs[k])
                continue;
            if (g_devs[k]->raw == list[i] ||
                (g_devs[k]->bus == libusb_get_bus_number(list[i]) &&
                 g_devs[k]->addr == libusb_get_device_address(list[i]))) {
                dev = g_devs[k];
                g_devs[k] = NULL;
                break;
            }
        }
        if (!dev)
            dev = probe(list[i]);
        if (dev)
            kept[nkept++] = dev;
    }
    libusb_free_device_list(list, 1);

    for (di = 0; di < g_ndev; di++)
        if (g_devs[di])
            free_device(g_devs[di]);
    memset(g_devs, 0, sizeof(g_devs));
    g_ndev = nkept;
    for (di = 0; di < nkept; di++)
        g_devs[di] = kept[di];
    return 0;
}

static int claim_and_init(struct kv_device *d)
{
    int err;
    if (d->claimed)
        return 0;
    err = libusb_open(d->raw, &d->h);
    if (err) {
        kv_log("open: %s", libusb_strerror(err));
        return KV_ERR_BUSY;
    }
    (void)libusb_set_auto_detach_kernel_driver(d->h, 1);
    err = libusb_claim_interface(d->h, 0);
    if (err) {
        kv_log("claim: %s", libusb_strerror(err));
        libusb_close(d->h);
        d->h = NULL;
        return KV_ERR_BUSY;
    }
    /* Seed the fields the receive thread reads before it starts, so a reply
     * arriving immediately cannot be overwritten by this thread's defaults. */
    if (d->family == KV_FAMILY_HYDRA) {
        memset(d->channel2he, ILLEGAL_HE, sizeof(d->channel2he));
        memset(d->he2channel, 0xFF, sizeof(d->he2channel));
        d->hires_mhz = 24;
    } else {
        d->hires_mhz = 16;
    }

    if (find_endpoints(d->raw, &d->ep_in, &d->ep_out, &d->maxpkt) ||
        start_rx(d)) {
        libusb_release_interface(d->h, 0);
        libusb_close(d->h);
        d->h = NULL;
        return KV_ERR_IO;
    }
    d->claimed = 1;
    if (d->family == KV_FAMILY_HYDRA) {
        err = kv_hydra_init_card(d);
        if (!err)
            kv_hydra_query_caps(d);
    } else if (d->family == KV_FAMILY_HELIOS) {
        err = kv_helios_init_card(d);
    } else {
        err = kv_leaf_init_card(d);
    }
    /* A card that would not initialise must not keep the interface claimed,
     * or the adapter stays unusable and a retry short-circuits on d->claimed. */
    if (err) {
        stop_rx(d);
        libusb_release_interface(d->h, 0);
        libusb_close(d->h);
        d->h = NULL;
        d->claimed = 0;
    }
    return err;
}

static void release_if_idle(struct kv_device *d)
{
    int i;
    for (i = 0; i < d->can_count; i++)
        if (d->ch[i].open)
            return;
    stop_rx(d);
    if (d->h) {
        libusb_release_interface(d->h, 0);
        libusb_close(d->h);
        d->h = NULL;
    }
    d->claimed = 0;
}

int kv_scan(kv_channel_info *out, int max)
{
    int i, c, n = 0;
    if (!out || max <= 0)
        return KV_ERR_PARAM;
    pthread_mutex_lock(&g_lock);
    refresh();
    for (i = 0; i < g_ndev && n < max; i++) {
        struct kv_device *d = g_devs[i];
        for (c = 0; c < d->can_count && n < max; c++) {
            kv_channel_info *ci = &out[n++];
            memset(ci, 0, sizeof(*ci));
            ci->handle = make_handle(d->bus, d->addr, (uint8_t)c);
            ci->pid = d->pid;
            ci->channel = (uint8_t)c;
            ci->channel_count = (uint8_t)d->can_count;
            ci->fd_capable = d->fd_capable;
            ci->available = 1;
            ci->serial = d->serial;
            snprintf(ci->product, sizeof(ci->product), "%s", d->name);
            if (d->can_count > 1)
                snprintf(ci->name, sizeof(ci->name), "%s CAN%d", d->name, c + 1);
            else
                snprintf(ci->name, sizeof(ci->name), "%s", d->name);
        }
    }
    pthread_mutex_unlock(&g_lock);
    return n;
}

int kv_open(uint32_t handle, const kv_open_opts *opts)
{
    struct kv_channel *ch;
    struct kv_device *d;
    kv_open_opts def;
    int err;

    pthread_mutex_lock(&g_lock);
    if (refresh()) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_IO;
    }
    ch = ch_for(handle);
    if (!ch) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NO_DEVICE;
    }
    d = ch->dev;
    if (ch->open) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_BUSY;
    }
    if (opts && opts->can_fd && !d->fd_capable) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_PARAM;
    }
    err = claim_and_init(d);
    if (err) {
        pthread_mutex_unlock(&g_lock);
        return err;
    }
    /* The queue outlives close/open cycles: the receive thread may still be
     * draining a URB for this channel while another one stays open. */
    if (!ch->q.items) {
        if (kv_queue_init(&ch->q, KV_RX_QUEUE)) {
            release_if_idle(d);
            pthread_mutex_unlock(&g_lock);
            return KV_ERR_IO;
        }
    } else {
        kv_queue_clear(&ch->q);
    }
    memset(&def, 0, sizeof(def));
    def.bitrate = 500000;
    if (!opts)
        opts = &def;
    ch->opts = *opts;
    if (d->family == KV_FAMILY_HYDRA)
        err = kv_hydra_bus_on(ch, opts);
    else if (ch->dev->family == KV_FAMILY_HELIOS)
        err = kv_helios_bus_on(ch, opts);
    else
        err = kv_leaf_bus_on(ch, opts);
    if (err)
        release_if_idle(d);
    pthread_mutex_unlock(&g_lock);
    return err;
}

void kv_close(uint32_t handle)
{
    struct kv_channel *ch;
    pthread_mutex_lock(&g_lock);
    ch = ch_for(handle);
    if (ch && ch->open) {
        if (ch->dev->family == KV_FAMILY_HYDRA)
            kv_hydra_bus_off(ch);
        else if (ch->dev->family == KV_FAMILY_HELIOS)
            kv_helios_bus_off(ch);
        else
            kv_leaf_bus_off(ch);
        kv_queue_clear(&ch->q);
        release_if_idle(ch->dev);
    }
    pthread_mutex_unlock(&g_lock);
}

int kv_read(uint32_t handle, kv_msg *msg)
{
    struct kv_channel *ch;
    int got;
    if (!msg)
        return KV_ERR_PARAM;
    pthread_mutex_lock(&g_lock);
    ch = ch_for(handle);
    if (!ch || !ch->open) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NOT_OPEN;
    }
    got = kv_queue_pop(&ch->q, msg);
    pthread_mutex_unlock(&g_lock);
    return got ? KV_OK : KV_ERR_EMPTY;
}

int kv_write(uint32_t handle, const kv_msg *msg)
{
    struct kv_channel *ch;
    int err;
    if (!msg)
        return KV_ERR_PARAM;
    pthread_mutex_lock(&g_lock);
    ch = ch_for(handle);
    if (!ch || !ch->open) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NOT_OPEN;
    }
    if (ch->bus_status & M16C_BUS_OFF) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_BUS_OFF;
    }
    if (ch->dev->family == KV_FAMILY_HYDRA)
        err = kv_hydra_write(ch, msg);
    else
        err = kv_leaf_write(ch, msg);
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_status(uint32_t handle, int *bus_off, uint8_t *tec, uint8_t *rec)
{
    struct kv_channel *ch;
    pthread_mutex_lock(&g_lock);
    ch = ch_for(handle);
    if (!ch || !ch->open) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NOT_OPEN;
    }
    if (bus_off)
        *bus_off = (ch->bus_status & M16C_BUS_OFF) ? 1 : 0;
    if (tec)
        *tec = ch->tec;
    if (rec)
        *rec = ch->rec;
    pthread_mutex_unlock(&g_lock);
    return KV_OK;
}

int kv_reset(uint32_t handle)
{
    struct kv_channel *ch;
    kv_open_opts opts;
    int err;
    pthread_mutex_lock(&g_lock);
    ch = ch_for(handle);
    if (!ch || !ch->open) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NOT_OPEN;
    }
    opts = ch->opts;
    kv_queue_clear(&ch->q);
    if (ch->dev->family == KV_FAMILY_HYDRA) {
        kv_hydra_bus_off(ch);
        err = kv_hydra_bus_on(ch, &opts);
    } else {
        kv_leaf_bus_off(ch);
        err = kv_leaf_bus_on(ch, &opts);
    }
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_device_info(uint32_t handle, char *hw, int hw_len, char *fw, int fw_len, uint32_t *serial)
{
    struct kv_device *d;
    pthread_mutex_lock(&g_lock);
    d = dev_for(handle);
    if (!d) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NO_DEVICE;
    }
    if (hw && hw_len > 0)
        snprintf(hw, (size_t)hw_len, "%s", d->name);
    if (fw && fw_len > 0) {
        if (d->fw_major || d->fw_minor || d->fw_build)
            snprintf(fw, (size_t)fw_len, "%u.%u.%u", d->fw_major, d->fw_minor, d->fw_build);
        else
            snprintf(fw, (size_t)fw_len, "%s", d->family == KV_FAMILY_HYDRA ? "hydra" : "filo");
    }
    if (serial)
        *serial = d->serial;
    pthread_mutex_unlock(&g_lock);
    return KV_OK;
}

int kv_get_busparams(uint32_t handle, int *bitrate, int *tseg1, int *tseg2,
                     int *sjw)
{
    struct kv_channel *ch;
    int err;
    pthread_mutex_lock(&g_lock);
    ch = ch_for(handle);
    if (!ch || !ch->open) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NOT_OPEN;
    }
    if (ch->dev->family != KV_FAMILY_HYDRA) {
        /* filo has no read-back command; report what was programmed. */
        if (bitrate) *bitrate = ch->opts.bitrate ? ch->opts.bitrate : 500000;
        if (tseg1) *tseg1 = ch->opts.tseg1;
        if (tseg2) *tseg2 = ch->opts.tseg2;
        if (sjw) *sjw = ch->opts.sjw;
        pthread_mutex_unlock(&g_lock);
        return KV_OK;
    }
    err = kv_hydra_get_busparams(ch);
    if (err == KV_OK) {
        if (bitrate) *bitrate = (int)ch->bp_bitrate;
        if (tseg1) *tseg1 = ch->bp_tseg1;
        if (tseg2) *tseg2 = ch->bp_tseg2;
        if (sjw) *sjw = ch->bp_sjw;
    }
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_capabilities(uint32_t handle, uint32_t *caps)
{
    struct kv_device *d;
    if (!caps)
        return KV_ERR_PARAM;
    pthread_mutex_lock(&g_lock);
    d = dev_for(handle);
    if (!d) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NO_DEVICE;
    }
    /* Fall back to the product table when the firmware never answered. */
    *caps = d->caps_valid ? d->caps : (d->fd_capable ? KV_CAP_CAN_FD : 0u);
    pthread_mutex_unlock(&g_lock);
    return KV_OK;
}

int kv_bus_load(uint32_t handle, int *per_mille)
{
    struct kv_channel *ch;
    int err;
    if (!per_mille)
        return KV_ERR_PARAM;
    pthread_mutex_lock(&g_lock);
    ch = ch_for(handle);
    if (!ch || !ch->open) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NOT_OPEN;
    }
    if (ch->dev->family != KV_FAMILY_HYDRA) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_PARAM;
    }
    err = kv_hydra_get_busload(ch);
    if (err == KV_OK)
        *per_mille = ch->busload_per_mille;
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_flush_tx(uint32_t handle)
{
    struct kv_channel *ch;
    int err;
    pthread_mutex_lock(&g_lock);
    ch = ch_for(handle);
    if (!ch || !ch->open) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NOT_OPEN;
    }
    if (ch->dev->family != KV_FAMILY_HYDRA) {
        ch->tx_outstanding = 0;
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_PARAM;
    }
    err = kv_hydra_flush_tx(ch);
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_tx_outstanding(uint32_t handle, int *count)
{
    struct kv_channel *ch;
    if (!count)
        return KV_ERR_PARAM;
    pthread_mutex_lock(&g_lock);
    ch = ch_for(handle);
    if (!ch || !ch->open) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NOT_OPEN;
    }
    *count = ch->tx_outstanding;
    pthread_mutex_unlock(&g_lock);
    return KV_OK;
}

/* The hydra-only entry points below share one shape: validate, dispatch,
 * unlock. Devices on the filo protocol have none of these commands. */
#define KV_HYDRA_CH(h, chvar)                       \
    pthread_mutex_lock(&g_lock);                    \
    (chvar) = ch_for(h);                            \
    if (!(chvar) || !(chvar)->open) {               \
        pthread_mutex_unlock(&g_lock);              \
        return KV_ERR_NOT_OPEN;                     \
    }                                               \
    if ((chvar)->dev->family != KV_FAMILY_HYDRA) {  \
        pthread_mutex_unlock(&g_lock);              \
        return KV_ERR_PARAM;                        \
    }

int kv_auto_tx(uint32_t handle, int buf, const kv_msg *msg,
               uint32_t interval_us)
{
    struct kv_channel *ch;
    int err;
    if (!msg && interval_us)
        return KV_ERR_PARAM;
    KV_HYDRA_CH(handle, ch);
    err = kv_hydra_auto_tx(ch, buf, msg, interval_us);
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_auto_tx_stop(uint32_t handle, int buf)
{
    struct kv_channel *ch;
    int err;
    KV_HYDRA_CH(handle, ch);
    err = kv_hydra_auto_tx(ch, buf, NULL, 0);
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_auto_tx_count(uint32_t handle, int *buffers)
{
    struct kv_device *d;
    int err;
    if (!buffers)
        return KV_ERR_PARAM;
    pthread_mutex_lock(&g_lock);
    d = dev_for(handle);
    if (!d) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NO_DEVICE;
    }
    if (d->family != KV_FAMILY_HYDRA) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_PARAM;
    }
    err = kv_hydra_auto_tx_info(d);
    if (err == KV_OK)
        *buffers = d->autotx_buffers;
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_set_tx_interval(uint32_t handle, uint32_t interval_us)
{
    struct kv_channel *ch;
    int err;
    KV_HYDRA_CH(handle, ch);
    err = kv_hydra_tx_interval(ch, interval_us);
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_license(uint32_t handle, uint32_t *mask, uint32_t *kvaser_mask)
{
    struct kv_device *d;
    int err;
    pthread_mutex_lock(&g_lock);
    d = dev_for(handle);
    if (!d) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NO_DEVICE;
    }
    if (d->family != KV_FAMILY_HYDRA) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_PARAM;
    }
    err = kv_hydra_license(d);
    if (err == KV_OK) {
        if (mask) *mask = d->license_mask;
        if (kvaser_mask) *kvaser_mask = d->kvaser_license_mask;
    }
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_io_port_read(uint32_t handle, int port, uint32_t *value)
{
    struct kv_device *d;
    int err;
    if (!value)
        return KV_ERR_PARAM;
    pthread_mutex_lock(&g_lock);
    d = dev_for(handle);
    if (!d) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NO_DEVICE;
    }
    if (d->family != KV_FAMILY_HYDRA) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_PARAM;
    }
    err = kv_hydra_io_port(d, port, value, 0);
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_io_port_write(uint32_t handle, int port, uint32_t value)
{
    struct kv_device *d;
    int err;
    pthread_mutex_lock(&g_lock);
    d = dev_for(handle);
    if (!d) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NO_DEVICE;
    }
    if (d->family != KV_FAMILY_HYDRA) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_PARAM;
    }
    err = kv_hydra_io_port(d, port, &value, 1);
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_get_driver_mode(uint32_t handle, int *listen_only)
{
    struct kv_channel *ch;
    int err;
    if (!listen_only)
        return KV_ERR_PARAM;
    KV_HYDRA_CH(handle, ch);
    err = kv_hydra_driver_mode(ch);
    if (err == KV_OK)
        *listen_only = (ch->reported_mode == DRIVERMODE_SILENT);
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_transceiver(uint32_t handle, uint32_t *capabilities, int *type,
                   int *status)
{
    struct kv_channel *ch;
    int err;
    KV_HYDRA_CH(handle, ch);
    err = kv_hydra_transceiver(ch);
    if (err == KV_OK) {
        if (capabilities) *capabilities = ch->dev->xcvr_caps;
        if (type) *type = ch->dev->xcvr_type;
        if (status) *status = ch->dev->xcvr_status;
    }
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_interface_info(uint32_t handle, uint32_t *capabilities, int *chip_type,
                      int *chip_subtype)
{
    struct kv_channel *ch;
    int err;
    KV_HYDRA_CH(handle, ch);
    err = kv_hydra_interface_info(ch);
    if (err == KV_OK) {
        if (capabilities) *capabilities = ch->dev->iface_caps;
        if (chip_type) *chip_type = ch->dev->chip_type;
        if (chip_subtype) *chip_subtype = ch->dev->chip_subtype;
    }
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_card_info2(uint32_t handle, char *pcb_id, int pcb_id_len)
{
    struct kv_device *d;
    int err;
    if (!pcb_id || pcb_id_len <= 0)
        return KV_ERR_PARAM;
    pthread_mutex_lock(&g_lock);
    d = dev_for(handle);
    if (!d) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NO_DEVICE;
    }
    if (d->family != KV_FAMILY_HYDRA) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_PARAM;
    }
    err = kv_hydra_card_info2(d);
    if (err == KV_OK)
        snprintf(pcb_id, (size_t)pcb_id_len, "%s", d->pcb_id);
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_get_device_mode(uint32_t handle, int *mode)
{
    struct kv_device *d;
    int err;
    if (!mode)
        return KV_ERR_PARAM;
    pthread_mutex_lock(&g_lock);
    d = dev_for(handle);
    if (!d) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NO_DEVICE;
    }
    if (d->family != KV_FAMILY_HYDRA) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_PARAM;
    }
    err = kv_hydra_device_mode(d, mode, 0);
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_set_device_mode(uint32_t handle, int mode)
{
    struct kv_device *d;
    int err;
    pthread_mutex_lock(&g_lock);
    d = dev_for(handle);
    if (!d) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NO_DEVICE;
    }
    if (d->family != KV_FAMILY_HYDRA) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_PARAM;
    }
    err = kv_hydra_device_mode(d, &mode, 1);
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_get_busparams_tq(uint32_t handle, int *prop, int *phase1, int *phase2,
                        int *sjw, int *brp)
{
    struct kv_channel *ch;
    int err;
    KV_HYDRA_CH(handle, ch);
    err = kv_hydra_busparams_tq(ch);
    if (err == KV_OK) {
        if (prop) *prop = ch->tq_prop;
        if (phase1) *phase1 = ch->tq_phase1;
        if (phase2) *phase2 = ch->tq_phase2;
        if (sjw) *sjw = ch->tq_sjw;
        if (brp) *brp = ch->tq_brp;
    }
    pthread_mutex_unlock(&g_lock);
    return err;
}

int kv_beep(uint32_t handle, int freq_hz, int duration_ms)
{
    struct kv_device *d;
    int err;
    if (freq_hz < 0 || freq_hz > 65535 || duration_ms < 0 || duration_ms > 65535)
        return KV_ERR_PARAM;
    pthread_mutex_lock(&g_lock);
    d = dev_for(handle);
    if (!d) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_NO_DEVICE;
    }
    if (d->family != KV_FAMILY_HYDRA) {
        pthread_mutex_unlock(&g_lock);
        return KV_ERR_PARAM;
    }
    err = kv_hydra_sound(d, freq_hz, duration_ms);
    pthread_mutex_unlock(&g_lock);
    return err;
}

const char *kv_library_version(void)
{
    return KVASERUSB_LIB_VERSION;
}

const char *kv_strerror(int err)
{
    switch (err) {
    case KV_OK: return "OK";
    case KV_ERR_NO_DEVICE: return "No device";
    case KV_ERR_BUSY: return "Device busy";
    case KV_ERR_IO: return "USB I/O error";
    case KV_ERR_TIMEOUT: return "Timeout";
    case KV_ERR_NOT_OPEN: return "Not open";
    case KV_ERR_EMPTY: return "Queue empty";
    case KV_ERR_BUS_OFF: return "Bus off";
    case KV_ERR_PARAM: return "Bad parameter";
    case KV_ERR_TX_FULL: return "Transmit window full, retry";
    case KV_ERR_NO_DRIVER: return "No driver";
    default: return "Error";
    }
}

__attribute__((constructor))
static void kv_ctor(void)
{
    const char *dbg = getenv("KVASERUSB_DEBUG");
    g_debug = dbg && *dbg && strcmp(dbg, "0") != 0;
}

__attribute__((destructor))
static void kv_dtor(void)
{
    int i;
    pthread_mutex_lock(&g_lock);
    for (i = 0; i < g_ndev; i++)
        free_device(g_devs[i]);
    g_ndev = 0;
    if (g_ctx) {
        libusb_exit(g_ctx);
        g_ctx = NULL;
    }
    g_inited = 0;
    pthread_mutex_unlock(&g_lock);
}
