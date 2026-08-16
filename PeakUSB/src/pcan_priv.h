/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef PEAKUSB_PRIV_H
#define PEAKUSB_PRIV_H

#include <pthread.h>
#include <stdint.h>
#include <libusb.h>
#include "PeakUSB.h"
#include "ucan.h"
#include "canxl.h"

/* PEAKUSB_LIB_VERSION comes from the public header. */
#define PEAKUSB_API_VERSION      "4.9.0"
#define PEAKUSB_MAX_HANDLES      8
#define PEAKUSB_MAX_CAN          2
#define PEAKUSB_RX_QUEUE         65536
/* 256 XL frames is about 0.5 MB; the CC/FD depth would be 137 MB. */
#define PEAKUSB_XL_QUEUE         256
#define PEAKUSB_RX_URBS          8
#define PEAKUSB_RX_URB_SIZE      4096
#define PEAKUSB_CMD_BUF          1024
#define PEAKUSB_CMD_TIMEOUT      1000
#define PEAKUSB_TX_TIMEOUT       1000
#define PEAKUSB_MAX_FILTERS      64
/* How far ahead of the wire a caller may queue before CAN_Write reports
 * PCAN_ERROR_XMTFULL. Comfortably more than any adapter's transmit buffer,
 * so it only bites when the application is genuinely outrunning the bus. */
#define PEAKUSB_TX_LEAD_US       8000

struct pcan_id_range {
    uint32_t from;
    uint32_t to;
    uint8_t  ext;   /* 1 = 29-bit, 0 = 11-bit */
};

struct pcan_bittiming {
    uint32_t clock_hz;
    uint16_t brp;
    uint16_t tseg1;
    uint16_t tseg2;
    uint16_t sjw;
    uint8_t  tsam;
    uint8_t  fd;          /* 1 = CAN FD (ISO) */
    uint16_t dbrp;
    uint16_t dtseg1;
    uint16_t dtseg2;
    uint16_t dsjw;
};

struct rx_item {
    TPCANMsgFD msg;
    uint64_t   ts_us;
};

struct xl_item {
    TPCANMsgXL msg;
    uint64_t   ts_us;
};

struct rx_queue {
    struct rx_item *items;
    uint32_t cap;
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    uint32_t overrun;
    pthread_mutex_t lock;
};

struct pcan_device;

struct pcan_channel {
    struct pcan_device *dev;
    int      can_idx;
    int      initialized;
    int      fd_mode;
    uint32_t status;
    uint32_t device_id;
    uint8_t  listen_only;
    uint8_t  allow_status;
    uint8_t  allow_rtr;
    uint8_t  allow_error;
    uint8_t  receive_on;
    uint8_t  bus_on;
    uint8_t  busoff_autoreset;
    uint8_t  filter_mode;     /* PCAN_FILTER_OPEN / CLOSE / CUSTOM */
    uint8_t  filter_count;
    uint8_t  rx_err_cnt;
    uint8_t  tx_err_cnt;
    uint64_t tx_free_us;      /* modelled time the transmit buffer drains */
    uint8_t  allow_echo;      /* echo transmitted frames back to the reader */
    uint32_t interframe_us;   /* caller-requested gap between transmits */
    struct pcan_id_range filters[PEAKUSB_MAX_FILTERS];
    uint16_t btr0btr1;
    char     bitrate_fd[256];
    struct pcan_bittiming bt;
    struct rx_queue q;
    /* CAN XL frames carry up to 2048 bytes, so a ring as deep as the CC/FD
     * one would need hundreds of megabytes. XL therefore gets its own, much
     * shallower queue, sized for burst absorption rather than long buffering. */
    struct xl_item *xlq;
    uint32_t xlq_cap, xlq_head, xlq_tail, xlq_count, xlq_overrun;
    int      xl_mode;
    int      ev_r;
    int      ev_w;
};

enum pcan_family {
    PCAN_FAM_FD = 0,     /* uCAN: USB FD / Pro FD / Chip USB / X6 */
    PCAN_FAM_USB,        /* classic PCAN-USB 0x000c */
    PCAN_FAM_USBPRO,     /* PCAN-USB Pro 0x000d */
    PCAN_FAM_XL          /* PCAN-USB XL 0x0030, CAN XL capable */
};

struct pcan_device {
    libusb_device        *raw;
    libusb_device_handle *h;
    uint8_t  bus;
    uint8_t  addr;
    uint16_t pid;
    uint8_t  family;
    char     name[MAX_LENGTH_HARDWARE_NAME];
    uint8_t  hw_type;
    uint8_t  hw_ver;
    uint8_t  fw[3];
    uint8_t  bl[3];
    uint32_t serial;
    uint32_t dev_id[PEAKUSB_MAX_CAN];
    int      can_count;
    uint8_t  usb_tx_seq;
    uint16_t usb_ts16;
    uint32_t usb_ts_hi;
    uint32_t usbpro_ts_low;
    uint32_t usbpro_ts_hi;
    uint8_t  ep_cmd_out;
    uint8_t  ep_cmd_in;
    uint8_t  ep_data_in;
    uint8_t  ep_data_out[PEAKUSB_MAX_CAN];
    int      claimed;
    int      rx_running;
    int      rx_pending;    /* transfers still owned by libusb */
    pthread_t rx_thread;
    pthread_mutex_t cmd_lock;
    struct libusb_transfer *rx_xfer[PEAKUSB_RX_URBS];
    uint8_t *rx_buf[PEAKUSB_RX_URBS];
    uint8_t  frag[256];
    int      frag_len;
    int      frag_need;
    struct pcan_channel ch[PEAKUSB_MAX_CAN];
};

struct pcan_handle {
    TPCANHandle         handle;
    struct pcan_device *dev;
    int                 can_idx;
    int                 present;
};

void pcan_log(const char *fmt, ...);

int  pcan_usb_global_init(void);
void pcan_usb_global_exit(void);
int  pcan_usb_refresh(void);
int  pcan_usb_handle_count(void);
TPCANHandle pcan_usb_handle_at(int idx);
struct pcan_handle *pcan_usb_lookup(TPCANHandle h);
struct pcan_channel *pcan_usb_channel(TPCANHandle h);
int  pcan_usb_open_channel(struct pcan_channel *ch);
void pcan_usb_close_channel(struct pcan_channel *ch);
void pcan_usb_release_device(struct pcan_device *d);
int  pcan_usb_bus_on(struct pcan_channel *ch);
int  pcan_usb_bus_off(struct pcan_channel *ch);
int  pcan_usb_configure(struct pcan_channel *ch);
int  pcan_usb_write_msg(struct pcan_channel *ch, const TPCANMsgFD *msg);
int  pcan_usb_tx_throttle(struct pcan_channel *ch, const TPCANMsgFD *msg);
int  pcan_usb_identify(struct pcan_channel *ch, int on);
int  pcan_usb_io_write(struct pcan_channel *ch, uint16_t opcode, uint32_t val);
void *pcan_cmd_add(uint8_t *buf, size_t *len, int ch, int op);
int  pcan_cmd_send(struct pcan_device *d, uint8_t *buf, size_t *len);
int  pcan_xl_configure(struct pcan_channel *ch);
void pcan_xl_decode(struct pcan_device *d, uint8_t *ptr, uint16_t size);
int  pcan_xl_write_msg(struct pcan_channel *ch, const TPCANMsgXL *msg);
int  pcan_xl_queue_init(struct pcan_channel *ch);
void pcan_xl_queue_free(struct pcan_channel *ch);
int  pcan_xl_queue_pop(struct pcan_channel *ch, TPCANMsgXL *m, uint64_t *ts);
uint32_t pcan_usb_bitrate(const struct pcan_channel *ch, int data_phase);
void pcan_usb_reset_queue(struct pcan_channel *ch);

int  pcan_queue_init(struct rx_queue *q, uint32_t cap);
void pcan_queue_free(struct rx_queue *q);
void pcan_queue_clear(struct rx_queue *q);
int  pcan_queue_push(struct rx_queue *q, const TPCANMsgFD *m, uint64_t ts,
                     int *was_empty);
int  pcan_queue_pop(struct rx_queue *q, TPCANMsgFD *m, uint64_t *ts);

int  pcan_parse_bitrate_fd(const char *s, struct pcan_bittiming *bt);
int  pcan_btr_from_sja1000(uint16_t btr0btr1, struct pcan_bittiming *bt);
void pcan_event_signal(struct pcan_channel *ch);
void pcan_event_clear_if_empty(struct pcan_channel *ch);

#endif
