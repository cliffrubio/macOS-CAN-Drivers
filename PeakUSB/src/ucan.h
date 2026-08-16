/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * uCAN / PCAN-USB (Pro) FD protocol definitions
 * Adapted from PEAK peak-linux-driver 9.2.0 (GPL-2.0)
 */
#ifndef PEAKUSB_UCAN_H
#define PEAKUSB_UCAN_H

#include <stdint.h>

#define PCAN_USB_VENDOR_ID          0x0c72
#define PCAN_USB_PRODUCT_ID         0x000c
#define PCAN_USBPRO_PRODUCT_ID      0x000d
#define PCAN_USBPROFD_PRODUCT_ID    0x0011
#define PCAN_USBFD_PRODUCT_ID       0x0012
#define PCAN_USBCHIP_PRODUCT_ID     0x0013
#define PCAN_USBX6_PRODUCT_ID       0x0014

#define USB_VENDOR_REQUEST_INFO     0
#define USB_VENDOR_REQUEST_ZERO     1
#define USB_VENDOR_REQUEST_FKT      2
#define USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED  5

#define PCAN_USBFD_FWINFO_REQ       1
#define PCAN_USBFD_TYPE_EXT         2

#define CANFD_CMD_NOP               0x000
#define CANFD_CMD_RESET_MODE        0x001
#define CANFD_CMD_NORMAL_MODE       0x002
#define CANFD_CMD_LISTEN_ONLY_MODE  0x003
#define CANFD_CMD_TIMING_SLOW       0x004
#define CANFD_CMD_TIMING_FAST       0x005
#define CANFD_CMD_SET_STD_FILTER    0x006
#define CANFD_CMD_TX_ABORT          0x009
#define CANFD_CMD_WR_ERR_CNT        0x00a
#define CANFD_CMD_SET_EN_OPTION     0x00b
#define CANFD_CMD_CLR_DIS_OPTION    0x00c
#define CANFD_CMD_RX_BARRIER        0x010
#define CANFD_CMD_END_OF_COLLECTION 0x3ff

#define CANFD_MSG_CAN_RX            0x0001
#define CANFD_MSG_ERROR             0x0002
#define CANFD_MSG_STATUS            0x0003
#define CANFD_MSG_BUSLOAD           0x0004
#define CANFD_MSG_CAN_TX            0x1000
#define CANFD_USB_MSG_CALIBRATION   0x100
#define CANFD_USB_MSG_OVERRUN       0x101

#define CANFD_CMD_OPCODE_CHANNEL(c, o)  ((uint16_t)(((c) << 12) | ((o) & 0x3ff)))

#define CANFD_OPTION_ERROR          0x0001
#define CANFD_OPTION_BUSLOAD        0x0002
#define CANFD_OPTION_ISO_MODE       0x0004
#define CANFD_OPTION_20AB_MODE      0x0010

#define CANFD_USB_OPTION_CALIBRATION 0x8000
#define CANFD_USB_CMD_CLK_SET       0x80
#define CANFD_USB_CMD_LED_SET       0x86
/* PCAN-USB Chip digital I/O. These are accepted only by that device; other
 * members of the FD family answer with an error or ignore them. */
#define CANFD_USB_CMD_DPIN_CFG_SET  0x8d
#define CANFD_USB_CMD_DPIN_CFG_REQ  0xca
#define CANFD_USB_CMD_DPIN_CFG_RSP  0xcb
#define CANFD_USB_CMD_DPIN_VAL_SET  0x8e
#define CANFD_USB_CMD_DPIN_VAL_REQ  0xcc
#define CANFD_USB_CMD_DPIN_VAL_RSP  0xcd
#define CANFD_USB_CMD_DPIN_SET_HIGH 0x8f
#define CANFD_USB_CMD_DPIN_SET_LOW  0x90
#define CANFD_USB_CLK_80MHZ         0x0
#define CANFD_USB_LED_DEV           0x00
#define CANFD_USB_LED_OFF           0x04

#define CANFD_WRERRCNT_TE           0x4000
#define CANFD_WRERRCNT_RE           0x8000
#define CANFD_FLTSTD_ROW_IDX_BITS   6
#define CANFD_TX_ABORT_FLUSH        0x0001

#define CANFD_MSG_RTR               0x01
#define CANFD_MSG_EXT_ID            0x02
#define CANFD_MSG_HW_SRR            0x04
#define CANFD_MSG_SINGLE_SHOT       0x08
#define CANFD_MSG_EXT_DATA_LEN      0x10
#define CANFD_MSG_BITRATE_SWITCH    0x20
#define CANFD_MSG_ERROR_STATE_IND   0x40
#define CANFD_MSG_API_SRR           0x80

#define CANFD_MSG_CHANNEL(m)        ((m)->channel_dlc & 0x0f)
#define CANFD_MSG_DLC(m)            ((m)->channel_dlc >> 4)
#define CANFD_MSG_CHANNEL_DLC(c, d) ((uint8_t)(((c) & 0xf) | ((d) << 4)))

#define CANFD_STMSG_CHANNEL(e)      ((e)->channel_p_w_b & 0x0f)
#define CANFD_STMSG_PASSIVE(e)      ((e)->channel_p_w_b & 0x20)
#define CANFD_STMSG_WARNING(e)      ((e)->channel_p_w_b & 0x40)
#define CANFD_STMSG_BUSOFF(e)       ((e)->channel_p_w_b & 0x80)
#define CANFD_ERMSG_CHANNEL(e)      ((e)->channel_type_d & 0x0f)
#define CANFD_USB_OVMSG_CHANNEL(o)  ((o)->channel & 0xf)

#define PCAN_USBFD_VENDOR_INFO_SIZE_V234  36

#pragma pack(push, 1)

struct pcan_usbfd_fw_info {
    uint16_t size_of;
    uint16_t type;
    uint8_t  hw_type;
    uint8_t  bl_version[3];
    uint8_t  hw_version;
    uint8_t  fw_version[3];
    uint32_t dev_id[2];
    uint32_t ser_no;
    uint32_t flags;
    uint8_t  cmd_out_ep;
    uint8_t  cmd_in_ep;
    uint8_t  data_out_ep[2];
    uint8_t  data_in_ep;
    uint8_t  can_bus_count;
    uint8_t  instance_idx;
    uint8_t  dummy;
    uint8_t  guid[16];
    uint8_t  fw_version_adv[32];
};

struct canfd_command {
    uint16_t opcode_channel;
    uint16_t args[3];
};

struct canfd_timing_slow {
    uint16_t opcode_channel;
    uint8_t  ewl;
    uint8_t  sjw_t;
    uint8_t  tseg2;
    uint8_t  tseg1;
    uint16_t brp;
};

struct canfd_timing_fast {
    uint16_t opcode_channel;
    uint8_t  unused;
    uint8_t  sjw;
    uint8_t  tseg2;
    uint8_t  tseg1;
    uint16_t brp;
};

struct canfd_std_filter {
    uint16_t opcode_channel;
    uint8_t  unused;
    uint8_t  idx;
    uint32_t mask;
};

struct canfd_wr_err_cnt {
    uint16_t opcode_channel;
    uint16_t sel_mask;
    uint8_t  tx_counter;
    uint8_t  rx_counter;
    uint16_t unused;
};

struct canfd_option {
    uint16_t opcode_channel;
    uint16_t mask;
    uint16_t unused;
    uint16_t ext_mask;
};

struct canfd_tx_abort {
    uint16_t opcode_channel;
    uint16_t flags;
    uint32_t unused;
};

struct ucan_usb_clock {
    uint16_t opcode_channel;
    uint8_t  mode;
    uint8_t  unused[5];
};

/* Digital I/O control record: opcode, then the 32-bit pin mask or value. */
struct ucan_usb_io_ctrl {
    uint16_t opcode;
    uint16_t reserved;
    uint32_t io_val;
};

struct ucan_usb_led {
    uint16_t opcode_channel;
    uint8_t  mode;
    uint8_t  unused[5];
};

struct canfd_msg {
    uint16_t size;
    uint16_t type;
    uint32_t ts_low;
    uint32_t ts_high;
};

struct canfd_rx_msg {
    uint16_t size;
    uint16_t type;
    uint32_t ts_low;
    uint32_t ts_high;
    uint32_t tag_low;
    uint32_t tag_high;
    uint8_t  channel_dlc;
    uint8_t  client;
    uint16_t flags;
    uint32_t can_id;
    uint8_t  d[64];
};

struct canfd_error_msg {
    uint16_t size;
    uint16_t type;
    uint32_t ts_low;
    uint32_t ts_high;
    uint8_t  channel_type_d;
    uint8_t  code_g;
    uint8_t  tx_err_cnt;
    uint8_t  rx_err_cnt;
};

struct canfd_status_msg {
    uint16_t size;
    uint16_t type;
    uint32_t ts_low;
    uint32_t ts_high;
    uint8_t  channel_p_w_b;
    uint8_t  unused[3];
};

struct ucan_usb_ovr_msg {
    uint16_t size;
    uint16_t type;
    uint32_t ts_low;
    uint32_t ts_high;
    uint8_t  channel;
    uint8_t  unused[3];
};

struct canfd_tx_msg {
    uint16_t size;
    uint16_t type;
    uint32_t tag_low;
    uint32_t tag_high;
    uint8_t  channel_dlc;
    uint8_t  client;
    uint16_t flags;
    uint32_t can_id;
    uint8_t  d[64];
};

#pragma pack(pop)

static inline uint8_t pcan_dlc2len(uint8_t dlc)
{
    static const uint8_t t[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
    };
    return t[dlc & 0x0f];
}

static inline uint8_t pcan_len2dlc(uint8_t len)
{
    if (len <= 8)
        return len;
    if (len <= 12)
        return 9;
    if (len <= 16)
        return 10;
    if (len <= 20)
        return 11;
    if (len <= 24)
        return 12;
    if (len <= 32)
        return 13;
    if (len <= 48)
        return 14;
    return 15;
}

static inline uint32_t ucan_align4(uint32_t n)
{
    return (n + 3u) & ~3u;
}

#endif
