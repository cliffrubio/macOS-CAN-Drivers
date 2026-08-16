/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Classic PCAN-USB (0x000c) and PCAN-USB Pro (0x000d) protocol
 * Adapted from PEAK peak-linux-driver 9.2.0 (GPL-2.0)
 */
#ifndef PEAKUSB_PEAK_USB_H
#define PEAKUSB_PEAK_USB_H

#include <stdint.h>

/* --- classic PCAN-USB 0x000c --- */
#define PCAN_USB_GET                    0x01
#define PCAN_USB_SET                    0x02
#define PCAN_USB_CMD_BITRATE            1
#define PCAN_USB_CMD_BUS                3
#define PCAN_USB_CMD_DEVID              4
#define PCAN_USB_CMD_SN                 6
#define PCAN_USB_CMD_ERR_FR             11
#define PCAN_USB_SET_SILENT_MODE        3
#define PCAN_USB_FORMAT_CODE            0x02
#define PCAN_USB_CMD_LEN                16
#define PCAN_USB_PKT_SIZE               64

#define STLN_WITH_TIMESTAMP             0x80
#define STLN_INTERNAL_DATA              0x40
#define STLN_EXTENDED_ID                0x20
#define STLN_RTR                        0x10
#define STLN_DATA_LENGTH                0x0F

#define PCAN_USB_XMT_BUFFER_FULL        0x01
#define PCAN_USB_CAN_RECEIVE_OVERRUN    0x02
#define PCAN_USB_BUS_LIGHT              0x04
#define PCAN_USB_BUS_HEAVY              0x08
#define PCAN_USB_BUS_OFF                0x10
#define PCAN_USB_QUEUE_OVERRUN          0x40
#define PCAN_USB_QUEUE_XMT_FULL         0x80

#define PCAN_USB_ERR_ECC                0x01
#define PCAN_USB_ERR_RXERR              0x02
#define PCAN_USB_ERR_TXERR              0x04
#define PCAN_USB_ERR_RXERR_CNT          0x08
#define PCAN_USB_ERR_TXERR_CNT          0x10

#define PCAN_USB_TS_SCALE_MULTIPLIER    42667ull
#define PCAN_USB_TS_SCALE_DIVISOR       1000ull

#define USB_VENDOR_REQUEST_wVALUE_INFO_BOOTLOADER  0
#define USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE    1

/* --- PCAN-USB Pro 0x000d records --- */
#define USBPRO_RX_8                     0x80
#define USBPRO_RX_4                     0x81
#define USBPRO_RX_0                     0x82
#define USBPRO_RTR_RX                   0x83
#define USBPRO_STATUS_ERROR_RX          0x84
#define USBPRO_CALIBRATION_RX           0x85
#define USBPRO_BUSLOAD_RX               0x86
#define USBPRO_TX_8                     0x41
#define USBPRO_TX_4                     0x42
#define USBPRO_TX_0                     0x43
#define USBPRO_FKT_SETBAUDRATE          0x02
#define USBPRO_FKT_SETCANBUSACTIVATE    0x04
#define USBPRO_FKT_SETSILENTMODE        0x05
#define USBPRO_FKT_SETFILTERMODE        0x0a
#define USBPRO_FKT_SETGET_CALIBRATION   0x10
#define USBPRO_FKT_SET_CANLED           0x1C

#define USBPRO_RTR                      0x01
#define USBPRO_EXT                      0x02
#define USBPRO_STATUS_ERROR_S           0x0001
#define USBPRO_STATUS_BUS_S             0x0002
#define USBPRO_STATUS_OVERRUN_S         0x0004
#define USBPRO_STATUS_QOVERRUN_S        0x0008
#define USBPRO_LED_DEVICE               0x00
#define USBPRO_LED_BLINK_FAST           0x01
#define USBPRO_LED_BLINK_SLOW           0x02
#define USBPRO_LED_ON                   0x03
#define USBPRO_LED_OFF                  0x04
#define USBPRO_SYSCLK_HZ                56000000u

#pragma pack(push, 1)

struct pcan_usb_param {
    uint8_t Function;
    uint8_t Number;
    uint8_t Param[14];
};

struct pcan_usbpro_bootloader_info {
    uint32_t ctrl_type;
    uint8_t  version[4];
    uint8_t  day, month, year, dummy;
    uint32_t serial_num_high;
    uint32_t serial_num_low;
    uint32_t hw_type;
    uint32_t hw_rev;
};

struct pcan_usbpro_ext_firmware_info {
    uint32_t ctrl_type;
    uint8_t  version[4];
    uint8_t  day, month, year, dummy;
    uint32_t fw_type;
};

struct pcan_usbpro_canmsg_rx {
    uint8_t  data_type;
    uint8_t  client;
    uint8_t  flags;
    uint8_t  len;
    uint32_t timestamp32;
    uint32_t id;
    uint8_t  data[8];
};

struct pcan_usbpro_status_error_rx {
    uint8_t  data_type;
    uint8_t  channel;
    uint16_t status;
    uint32_t timestamp32;
    uint32_t error_frame;
};

struct pcan_usbpro_canmsg_tx {
    uint8_t  data_type;
    uint8_t  client;
    uint8_t  flags;
    uint8_t  len;
    uint32_t id;
    uint8_t  data[8];
};

struct pcan_usbpro_baudrate {
    uint8_t  data_type;
    uint8_t  channel;
    uint16_t dummy;
    uint32_t CCBT;
};

struct pcan_usbpro_u16 {
    uint8_t  data_type;
    uint8_t  channel;
    uint16_t value;
};

struct pcan_usbpro_calibration {
    uint8_t  data_type;
    uint8_t  dummy;
    uint16_t mode;
};

struct pcan_usbpro_set_can_led {
    uint8_t  data_type;
    uint8_t  channel;
    uint16_t mode;
    uint32_t timeout;
};

#pragma pack(pop)

#endif
