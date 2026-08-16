/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 *
 * Internal state shared by the KvaserUSB transport (usb.c) and the two
 * protocol back ends (leaf.c for filo/helios, hydra.c for mhydra).
 */
#ifndef KVASER_PRIV_H
#define KVASER_PRIV_H

#include <pthread.h>
#include <stdint.h>
#include <libusb.h>
#include "KvaserUSB.h"

#define KV_VID              0x0BFD
#define KV_MAX_DEV          8
#define KV_MAX_CAN          5
#define KV_RX_QUEUE         65536
#define KV_RX_URBS          8
#define KV_RX_URB_SIZE      4096
#define KV_CMD_TIMEOUT      1000
#define KV_TX_TIMEOUT       1000
#define KV_HYDRA_CMD        32
#define KV_HYDRA_MAX        128
#define KV_FILO_MAX         32
#define KV_EXT_MSG          0x80000000u

#define KV_FAMILY_LEAF      1
#define KV_FAMILY_HYDRA     2
#define KV_FAMILY_HELIOS    3   /* USBcan II generation */

#define M16C_BUS_RESET      0x01
#define M16C_BUS_PASSIVE    0x20
#define M16C_BUS_OFF        0x40

#define MSGFLAG_ERROR_FRAME  0x01
#define MSGFLAG_OVERRUN      0x02
#define MSGFLAG_REMOTE_FRAME 0x10
#define MSGFLAG_EXTENDED_ID  0x20
#define MSGFLAG_TX           0x40
#define MSGFLAG_FDF          0x010000
#define MSGFLAG_BRS          0x020000
#define MSGFLAG_ESI          0x040000

#define DRIVERMODE_NORMAL    0x01
#define DRIVERMODE_SILENT    0x02

#define ILLEGAL_HE           0x3e
#define ROUTER_HE            0x00

struct kv_rx_item {
    kv_msg msg;
};

struct kv_queue {
    struct kv_rx_item *items;
    uint32_t cap, head, tail, count, overrun;
    pthread_mutex_t lock;
};

struct kv_device;

struct kv_channel {
    struct kv_device *dev;
    int      idx;
    int      open;
    uint8_t  tec, rec;
    uint8_t  bus_status;
    uint8_t  listen_only;
    uint8_t  fd_mode;
    uint8_t  transid;
    kv_open_opts opts;      /* settings the channel was opened with */
    /* Transmit window. The firmware acknowledges each frame it has sent, so
     * sent-minus-acked is what the adapter still holds. Refusing once that
     * reaches the window is the difference between backpressure and silent
     * loss; the adapter drops frames without reporting when its queue fills. */
    uint16_t tx_outstanding;
    uint16_t tx_window;     /* from the firmware, or a conservative default */
    uint8_t  busload_valid;
    uint16_t busload_per_mille;
    uint8_t  bp_valid;      /* bus params read back from the device */
    uint32_t bp_bitrate;
    uint8_t  bp_tseg1, bp_tseg2, bp_sjw;
    uint8_t  reported_mode;  /* silent/normal, as the firmware reports it */
    uint8_t  mode_valid;
    uint16_t tq_prop, tq_phase1, tq_phase2, tq_sjw, tq_brp;
    uint8_t  tq_valid;
    struct kv_queue q;      /* allocated on first open, freed with the device */
};

struct kv_wait {
    int      armed;
    uint8_t  cmd;
    uint16_t seq;
    int      match_seq;
    uint8_t  buf[KV_HYDRA_MAX];
    int      len;
    pthread_mutex_t lock;
    pthread_cond_t  cv;
};

struct kv_device {
    libusb_device        *raw;
    libusb_device_handle *h;
    uint8_t  bus, addr;
    uint16_t pid;
    uint8_t  family;
    uint8_t  fd_capable;
    char     name[64];
    uint32_t serial;
    uint8_t  fw_major, fw_minor;
    uint16_t fw_build;
    int      can_count;
    uint8_t  ep_in, ep_out;
    uint16_t maxpkt;
    int      claimed;
    int      rx_running;
    int      rx_pending;    /* transfers still owned by libusb */
    pthread_t rx_thread;
    pthread_mutex_t io_lock;
    struct libusb_transfer *rx_xfer[KV_RX_URBS];
    uint8_t *rx_buf[KV_RX_URBS];
    uint8_t  leftover[KV_HYDRA_MAX];
    int      leftover_len;
    uint8_t  channel2he[KV_MAX_CAN];
    uint8_t  he2channel[64];
    uint8_t  hydra_ext;
    uint8_t  hires_mhz;     /* device timestamp clock, MHz (from swOptions) */
    uint16_t can_clk_mhz;   /* CAN controller base clock, MHz (from swOptions) */
    uint32_t caps;          /* KV_CAP_* reported by the firmware */
    uint8_t  caps_valid;
    uint16_t max_outstanding_tx;   /* firmware transmit window, 0 if unknown */
    uint8_t  autotx_buffers;       /* periodic-transmit slots the device has */
    uint8_t  autotx_valid;
    uint32_t license_mask, kvaser_license_mask;
    uint8_t  license_valid;
    uint32_t io_port_val;
    uint8_t  io_port_status, io_port_valid;
    uint32_t helios_time_hi;   /* high half of the 16-bit helios clock */
    uint32_t xcvr_caps;        /* transceiver, interface and card detail */
    uint8_t  xcvr_type, xcvr_status, xcvr_valid;
    uint32_t iface_caps;
    uint8_t  chip_type, chip_subtype, iface_valid;
    char     pcb_id[25];
    uint8_t  pcb_valid;
    uint8_t  device_mode, device_mode_valid;
    struct kv_wait wait;
    struct kv_channel ch[KV_MAX_CAN];
};

struct kv_product {
    uint16_t pid;
    uint8_t  family;
    uint8_t  channels;
    uint8_t  fd;
    const char *name;
};

const struct kv_product *kv_lookup_pid(uint16_t pid);
void kv_log(const char *fmt, ...);

/* Transaction ids wrap 1..255; 0 is reserved by the firmware. */
static inline uint8_t kv_next_transid(struct kv_channel *ch)
{
    if (++ch->transid == 0)
        ch->transid = 1;
    return ch->transid;
}

/* Device timestamps arrive as ticks of the adapter's high-resolution timer,
 * whose frequency (16/24/32 MHz filo, 24/80 MHz hydra) is announced in the
 * software-info swOptions. */
static inline uint64_t kv_ticks_to_us(const struct kv_device *d, uint64_t ticks)
{
    return d->hires_mhz ? ticks / d->hires_mhz : ticks;
}

int  kv_queue_init(struct kv_queue *q, uint32_t cap);
void kv_queue_free(struct kv_queue *q);
void kv_queue_clear(struct kv_queue *q);
int  kv_queue_push(struct kv_queue *q, const kv_msg *m);
int  kv_queue_pop(struct kv_queue *q, kv_msg *m);

void kv_push_frame(struct kv_channel *ch, uint32_t id, uint8_t flags,
                   const uint8_t *data, int len, uint8_t dlc, uint64_t ts);
uint8_t kv_len_to_dlc(int len, int fd);
int     kv_dlc_to_len(uint8_t dlc, int fd);
void    kv_timing(int bps, int sample_pct, uint8_t *tseg1, uint8_t *tseg2, uint8_t *sjw);
void    kv_timing_clk(int bps, int sample_pct, int clock_hz,
                      uint8_t *tseg1, uint8_t *tseg2, uint8_t *sjw);
void    kv_resolve_timing(int bps, const kv_open_opts *opts, int data_phase,
                          int clock_hz, uint8_t *tseg1, uint8_t *tseg2, uint8_t *sjw);

int  kv_send_raw(struct kv_device *d, const void *data, int len);
void kv_arm_wait(struct kv_device *d, uint8_t cmd, uint16_t seq, int match_seq);
int  kv_wait_cmd(struct kv_device *d, uint8_t cmd, uint16_t seq, int match_seq,
                 void *out, int out_len, int timeout_ms);
void kv_maybe_complete_wait(struct kv_device *d, uint8_t cmd, uint16_t seq,
                            const void *buf, int len);

void kv_leaf_handle(struct kv_device *d, const uint8_t *p, int len);
int  kv_leaf_init_card(struct kv_device *d);
int  kv_leaf_bus_on(struct kv_channel *ch, const kv_open_opts *opts);
int  kv_leaf_bus_off(struct kv_channel *ch);
int  kv_leaf_write(struct kv_channel *ch, const kv_msg *msg);

void kv_helios_handle(struct kv_device *d, const uint8_t *p, int len);
int  kv_helios_init_card(struct kv_device *d);
int  kv_helios_bus_on(struct kv_channel *ch, const kv_open_opts *opts);
int  kv_helios_bus_off(struct kv_channel *ch);
int  kv_helios_write(struct kv_channel *ch, const kv_msg *msg);

void kv_hydra_handle(struct kv_device *d, const uint8_t *p, int len);
int  kv_hydra_init_card(struct kv_device *d);
int  kv_hydra_bus_on(struct kv_channel *ch, const kv_open_opts *opts);
int  kv_hydra_bus_off(struct kv_channel *ch);
int  kv_hydra_write(struct kv_channel *ch, const kv_msg *msg);
void kv_hydra_query_caps(struct kv_device *d);
int  kv_hydra_get_busparams(struct kv_channel *ch);
int  kv_hydra_get_busload(struct kv_channel *ch);
int  kv_hydra_flush_tx(struct kv_channel *ch);
int  kv_hydra_auto_tx(struct kv_channel *ch, int buf, const kv_msg *msg,
                      uint32_t interval_us);
int  kv_hydra_auto_tx_info(struct kv_device *d);
int  kv_hydra_tx_interval(struct kv_channel *ch, uint32_t interval_us);
int  kv_hydra_license(struct kv_device *d);
int  kv_hydra_io_port(struct kv_device *d, int port, uint32_t *value, int write);
int  kv_hydra_driver_mode(struct kv_channel *ch);
int  kv_hydra_transceiver(struct kv_channel *ch);
int  kv_hydra_interface_info(struct kv_channel *ch);
int  kv_hydra_card_info2(struct kv_device *d);
int  kv_hydra_device_mode(struct kv_device *d, int *mode, int write);
int  kv_hydra_busparams_tq(struct kv_channel *ch);
int  kv_hydra_sound(struct kv_device *d, int freq_hz, int duration_ms);

void kv_hydra_set_dst(uint8_t *cmd, uint8_t he);
void kv_hydra_set_seq(uint8_t *cmd, uint16_t seq);
uint8_t kv_hydra_src_he(const uint8_t *cmd);
uint16_t kv_hydra_seq(const uint8_t *cmd);

#endif
