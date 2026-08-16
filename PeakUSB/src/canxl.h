/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * CAN XL wire format for the PCAN-USB XL (product ID 0x0030).
 * Definitions adapted from peak-linux-driver 9.2.0, pcanxl_core_user.h and
 * pcanxl_usb_fw.h.
 *
 * XL reuses the uCAN USB framing: records carry a 16-bit size and an 8-bit
 * type, and several receive types are shared with the FD family outright. The
 * frame itself is a different shape: up to 2048 payload bytes, a priority ID
 * rather than an arbitration ID, plus VCID, SDT and an acceptance field.
 */
#ifndef PEAKUSB_CANXL_H
#define PEAKUSB_CANXL_H

#include <stdint.h>

#define PCAN_USBXL_PRODUCT_ID       0x0030

/* Record types. The CC/FD receive types are shared with the uCAN family. */
#define CANXL_TX_MSG_CCFD           0x30
#define CANXL_TX_PAUSE              0x32
#define CANXL_TX_MSG_XL             0x33
#define CANXL_RX_MSG_XL             0x05
#define CANXL_RX_ERR_CNT_DEC        0x06
#define CANXL_RX_ERR_NOTIF          0x07
#define CANXL_RX_PROT_EXCEPT        0x08
#define CANXL_RX_OVERLOAD           0x09
#define CANXL_RX_BUSLOAD2           0x0b
#define CANXL_RX_OVERRUN            0x21

/* Timing commands: XL programs three phases independently. */
#define CANXL_CMD_TIMING_NOMINAL    0x44
#define CANXL_CMD_TIMING_FD         0x45
#define CANXL_CMD_TIMING_XL         0x46
#define CANXL_CMD_PWM_CFG_XL        0x47
#define CANXL_CMD_FD_OPTS           0x55
#define CANXL_CMD_XL_OPTS           0x56

#define CANXL_USB_CLK_160MHZ        0x6

/* Message flags. XLF distinguishes an XL frame from a CC/FD one. */
#define CANXL_MSG_XLF               0x0100
#define CANXL_MSG_SEC               0x0200
#define CANXL_MSG_RRS               0x0400

/* sjw_tseg2_brp packs three fields into one 32-bit word. */
#define CANXL_SJW_SHIFT             0
#define CANXL_TSEG2_SHIFT           10
#define CANXL_BRP_SHIFT             20
#define CANXL_FIELD_MASK            0x3FFu

#pragma pack(push, 1)

struct canxl_tx_hdr {
    uint16_t size;              /* multiple of 4 */
    uint8_t  type;
    uint8_t  channel;
};

struct canxl_rx_hdr {
    uint16_t size;
    uint8_t  type;
    uint8_t  channel;
    uint64_t timestamp;         /* nanoseconds */
};

/* Classic and FD frames, on an XL adapter. */
struct canxl_msg_fd {
    uint16_t size;
    uint8_t  type;
    uint8_t  channel;
    uint64_t tag;
    uint8_t  dlc;               /* DLC in the upper four bits */
    uint8_t  client;
    uint16_t flags;
    uint32_t id;
    uint8_t  d[];
};

/* A CAN XL frame. pid_rrs_dlc_sec_sdt packs the priority ID, the RRS and SEC
 * flags, the 11-bit DLC and the service data type into one word. */
struct canxl_msg_xl {
    uint16_t size;
    uint8_t  type;
    uint8_t  channel;
    uint64_t tag;
    uint8_t  vcid;
    uint8_t  client;
    uint16_t flags;
    uint32_t pid_rrs_dlc_sec_sdt;
    uint32_t af;
    uint8_t  d[];
};

struct canxl_timing {
    uint16_t opcode_channel;
    uint16_t tseg1;
    uint32_t sjw_tseg2_brp;
};

#pragma pack(pop)

/* Field accessors for pid_rrs_dlc_sec_sdt. The DLC is a byte count minus one
 * for XL frames, unlike the CC/FD table lookup. */
#define CANXL_PID(v)        ((v) & 0x7FFu)
#define CANXL_SDT(v)        (((v) >> 24) & 0xFFu)
#define CANXL_DLC_GET(v)    ((((v) >> 11) & 0x7FFu) + 1u)
#define CANXL_PACK(pid, dlc, sdt) \
    (((uint32_t)(pid) & 0x7FFu) | \
     ((((uint32_t)(dlc) - 1u) & 0x7FFu) << 11) | \
     (((uint32_t)(sdt) & 0xFFu) << 24))

#define CANXL_TIMING_PACK(sjw, tseg2, brp) \
    ((((uint32_t)(sjw)   - 1u) & CANXL_FIELD_MASK) << CANXL_SJW_SHIFT | \
     (((uint32_t)(tseg2) - 1u) & CANXL_FIELD_MASK) << CANXL_TSEG2_SHIFT | \
     (((uint32_t)(brp)   - 1u) & CANXL_FIELD_MASK) << CANXL_BRP_SHIFT)

#endif /* PEAKUSB_CANXL_H */
