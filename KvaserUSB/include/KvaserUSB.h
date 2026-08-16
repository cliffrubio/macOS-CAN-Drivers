/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 *
 * KvaserUSB: macOS user-space driver for Kvaser USB CAN adapters.
 *
 * The wire protocols are ported from Kvaser's linuxcan sources:
 *   leaf/ + usbcanII/ ("filo"/"helios" commands) and mhydra/ ("hydra").
 * This is not a wrapper around Kvaser's closed-source CANlib.
 *
 *   Copyright (C) 2017-2023  Kvaser AB, Molndal, Sweden (linuxcan sources)
 *   Copyright (C) 2026  Cliff Rubio (macOS port)
 */
#ifndef KVASER_USB_H
#define KVASER_USB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KVASERUSB_LIB_VERSION "1.0.2"

#if defined(__GNUC__) || defined(__clang__)
#define KV_API __attribute__((visibility("default")))
#else
#define KV_API
#endif

/* Return codes. Every entry point returns KV_OK (0) or a negative KV_ERR_*. */
#define KV_OK              0
#define KV_ERR_NO_DEVICE  -1   /* no adapter behind that handle */
#define KV_ERR_BUSY       -2   /* claimed by another process, or already open */
#define KV_ERR_IO         -3   /* USB transfer failed */
#define KV_ERR_TIMEOUT    -4   /* firmware did not answer a command */
#define KV_ERR_NOT_OPEN   -5   /* channel is not open */
#define KV_ERR_EMPTY      -6   /* receive queue is empty (not an error) */
#define KV_ERR_BUS_OFF    -7   /* controller is bus-off, transmit refused */
#define KV_ERR_PARAM      -8   /* bad argument or unsupported option */
#define KV_ERR_NO_DRIVER  -9   /* libusb could not be initialised */
#define KV_ERR_TX_FULL   -10   /* adapter's transmit window is full, retry */

/* kv_msg.flags uses the same bit layout as the PEAK MSGTYPE flags, so the
 * two drivers in this repository can share frame-handling code. */
#define KV_FLAG_RTR       0x01  /* remote request */
#define KV_FLAG_EXTENDED  0x02  /* 29-bit identifier */
#define KV_FLAG_FD        0x04  /* CAN FD frame */
#define KV_FLAG_BRS       0x08  /* FD bit-rate switch */
#define KV_FLAG_ESI       0x10  /* FD error-state indicator */
#define KV_FLAG_ECHO      0x20  /* loopback of a frame we transmitted */
#define KV_FLAG_ERROR     0x40  /* error frame, see the data[] layout below */
#define KV_FLAG_STATUS    0x80  /* bus-status change */

/* An error frame (KV_FLAG_ERROR) carries everything the adapter reports:
 *   data[0] = receive error counter        data[1] = transmit error counter
 *   data[2] = controller bus status (KV_BUSSTAT_*)
 *   data[3] = error factor reported by the firmware (0 if not provided) */
#define KV_BUSSTAT_RESET          0x01  /* controller is in reset / bus-off requested */
#define KV_BUSSTAT_ERROR_PASSIVE  0x20  /* an error counter passed 127 */
#define KV_BUSSTAT_BUSOFF         0x40  /* controller is bus-off */

/* Firmware capability bits, as reported by kv_capabilities(). Read these
 * rather than inferring from the product table, which is only a fallback. */
#define KV_CAP_CAN_FD       0x0001  /* CAN FD, ISO */
#define KV_CAP_CAN_FD_NONISO 0x0002 /* CAN FD, non-ISO */
#define KV_CAP_SILENT       0x0004  /* listen-only mode */
#define KV_CAP_ERR_COUNTERS 0x0008  /* readable error counters */
#define KV_CAP_BUS_STATS    0x0010  /* bus load statistics */
#define KV_CAP_ERR_FRAMES   0x0020  /* error frame reporting */
#define KV_CAP_SINGLE_SHOT  0x0040  /* single-shot transmit */

/* Upper bound for a kv_scan() result array. */
#define KV_MAX_CHANNELS   32

/* One CAN channel of one adapter, as returned by kv_scan(). */
typedef struct {
    uint32_t handle;         /* bus << 16 | address << 8 | channel index */
    char     name[64];       /* product name, with " CANn" for multi-channel */
    char     product[64];    /* product name without the channel suffix */
    uint16_t pid;            /* USB product ID */
    uint8_t  channel;        /* 0-based channel index within the adapter */
    uint8_t  channel_count;  /* channels this adapter has */
    uint8_t  fd_capable;     /* 1 if the adapter supports CAN FD */
    uint8_t  available;      /* 1 if the channel can be opened */
    uint32_t serial;         /* 0 until the adapter has been opened once */
} kv_channel_info;

typedef struct {
    uint32_t id;             /* 11- or 29-bit identifier */
    uint8_t  flags;          /* KV_FLAG_* */
    uint8_t  dlc;            /* raw DLC (0..15) */
    uint8_t  len;            /* payload bytes actually present in data[] */
    uint8_t  data[64];
    uint64_t ts_us;          /* device timestamp, microseconds */
} kv_msg;

typedef struct {
    int bitrate;             /* arbitration bit rate, bit/s (default 500000) */
    int data_bitrate;        /* FD data bit rate, bit/s (default 2000000) */
    int can_fd;              /* 1 to open in CAN FD mode (hydra adapters) */
    int listen_only;         /* 1 for silent mode, no ACKs */
    int sample_point;        /* sample point in percent (default 80) */

    /* Explicit bit timing, for bus configurations the automatic calculation
     * cannot reach. Leave at 0 to derive the segments from bitrate and
     * sample_point. When tseg1 and tseg2 are both set they are sent to the
     * firmware as given, in time quanta; sjw falls back to 1 if left 0.
     * The prescaler follows from them: the firmware divides its CAN clock
     * by bitrate * (1 + tseg1 + tseg2). */
    int tseg1, tseg2, sjw;                   /* arbitration phase */
    int data_tseg1, data_tseg2, data_sjw;    /* FD data phase */
} kv_open_opts;

/* Enumerate channels of every attached Kvaser adapter.
 * Returns the number written to out[], or a negative KV_ERR_*. */
KV_API int  kv_scan(kv_channel_info *out, int max);

/* Open one channel and go bus-on. opts may be NULL for 500 kbit/s classic CAN. */
KV_API int  kv_open(uint32_t handle, const kv_open_opts *opts);
KV_API void kv_close(uint32_t handle);

/* Non-blocking. Returns KV_ERR_EMPTY when no frame is queued. */
KV_API int  kv_read(uint32_t handle, kv_msg *msg);
KV_API int  kv_write(uint32_t handle, const kv_msg *msg);

/* Latest controller state reported by the adapter. Any pointer may be NULL. */
KV_API int  kv_status(uint32_t handle, int *bus_off, uint8_t *tec, uint8_t *rec);

/* Bus-off then bus-on again with the settings the channel was opened with. */
KV_API int  kv_reset(uint32_t handle);

KV_API int  kv_device_info(uint32_t handle, char *hw, int hw_len,
                           char *fw, int fw_len, uint32_t *serial);

/* Bus parameters the controller is actually running, read back from the
 * device rather than echoed from what was requested. Any pointer may be NULL. */
KV_API int  kv_get_busparams(uint32_t handle, int *bitrate, int *tseg1,
                             int *tseg2, int *sjw);

/* Firmware capability bits (KV_CAP_*), queried from the device. Falls back to
 * the product table when the firmware does not answer. */
KV_API int  kv_capabilities(uint32_t handle, uint32_t *caps);

/* Bus load in tenths of a percent (0..1000), averaged by the adapter over its
 * own sampling interval. KV_ERR_PARAM if the device does not report it. */
KV_API int  kv_bus_load(uint32_t handle, int *per_mille);

/* Discard whatever the adapter has queued for transmission on this channel.
 * The host-side receive queue is untouched; use kv_reset() for that. */
KV_API int  kv_flush_tx(uint32_t handle);

/* Frames handed to the adapter that it has not yet acknowledged. Transmit
 * refuses with KV_ERR_TX_FULL once this reaches the firmware's window, so a
 * caller that ignores the return value cannot silently lose frames. */
KV_API int  kv_tx_outstanding(uint32_t handle, int *count);

/* Periodic transmit. The adapter sends the frame itself on the given
 * interval, so a heartbeat or stimulus keeps running without the host.
 * buf selects one of the adapter's buffers; interval_us of 0 stops it.
 * kv_auto_tx_count() reports how many buffers the device has. */
KV_API int  kv_auto_tx(uint32_t handle, int buf, const kv_msg *msg,
                       uint32_t interval_us);
KV_API int  kv_auto_tx_stop(uint32_t handle, int buf);
KV_API int  kv_auto_tx_count(uint32_t handle, int *buffers);

/* Minimum gap the adapter leaves between frames it transmits, in
 * microseconds. This is enforced by the firmware, unlike a host-side delay. */
KV_API int  kv_set_tx_interval(uint32_t handle, uint32_t interval_us);

/* Licence bits the device carries. Kvaser gates some higher-tier firmware
 * features on these; the meaning of each bit is Kvaser's. */
KV_API int  kv_license(uint32_t handle, uint32_t *mask, uint32_t *kvaser_mask);

/* Hardware-specific I/O ports, on the devices that have them. */
KV_API int  kv_io_port_read(uint32_t handle, int port, uint32_t *value);
KV_API int  kv_io_port_write(uint32_t handle, int port, uint32_t value);

/* Silent mode as the firmware reports it, rather than as it was requested. */
KV_API int  kv_get_driver_mode(uint32_t handle, int *listen_only);

/* CAN transceiver fitted to a channel. The type is Kvaser's own numbering:
 * high-speed, low-speed, single-wire and so on. Any pointer may be NULL. */
KV_API int  kv_transceiver(uint32_t handle, uint32_t *capabilities,
                           int *type, int *status);

/* CAN controller chip behind a channel, and that channel's capability word
 * as the firmware reports it. */
KV_API int  kv_interface_info(uint32_t handle, uint32_t *capabilities,
                              int *chip_type, int *chip_subtype);

/* Manufacturing detail: the PCB identifier string the device carries.
 * pcb_id must have room for 25 bytes. */
KV_API int  kv_card_info2(uint32_t handle, char *pcb_id, int pcb_id_len);

/* Hardware-specific device mode, on adapters that have one. */
KV_API int  kv_get_device_mode(uint32_t handle, int *mode);
KV_API int  kv_set_device_mode(uint32_t handle, int mode);

/* Bit timing expressed in time quanta rather than as a bit rate, which is
 * how newer firmware prefers to report it. Any pointer may be NULL. */
KV_API int  kv_get_busparams_tq(uint32_t handle, int *prop, int *phase1,
                                int *phase2, int *sjw, int *brp);

/* Sound the buzzer, on devices that have one. freq in Hz, duration in
 * milliseconds; a duration of 0 stops it. */
KV_API int  kv_beep(uint32_t handle, int freq_hz, int duration_ms);

KV_API const char *kv_library_version(void);
KV_API const char *kv_strerror(int err);

#ifdef __cplusplus
}
#endif

#endif /* KVASER_USB_H */
