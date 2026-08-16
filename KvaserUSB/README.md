# KvaserUSB

A user-space driver for Kvaser USB CAN adapters on macOS. The public header
is [`include/KvaserUSB.h`](include/KvaserUSB.h); the built library is
`libKvaserUSB.1.dylib`.

This is a port of the USB protocols in Kvaser's `linuxcan` 5.52.563
(BSD-3-Clause OR GPL-2.0), driven from user space with
[libusb](https://libusb.info/). It is **not** a wrapper around Kvaser's
CANlib, and it needs no kernel or system extension.

It is also unrelated to MacCAN's
[KvaserCAN-Library](https://github.com/mac-can/KvaserCAN-Library), which
drives the same hardware on macOS behind a different interface. The two share
no code and no symbol names, so both can be installed at once. See
[Relationship to MacCAN KvaserCAN-Library](../README.md#relationship-to-maccan-kvasercan-library).

## Supported adapters

| Family | linuxcan source | Devices | CAN FD |
| --- | --- | --- | --- |
| helios | `usbcanII/` | USBcan, VCI-2, USBcan II, Memorator | no |
| filo | `leaf/` | Leaf Light, Leaf Light v2, Leaf Professional, Leaf SemiPro, USBcan Light 2xHS, BlackBird SemiPro | no |
| hydra | `mhydra/` | Leaf Pro HS v2, Leaf v3, USBcan Pro (2x/4x/5x), U100 / U100P / U100S, Memorator v2, Hybrid CAN/LIN, Eagle, BlackBird v2 | usually |

Protocol family and CAN FD support are not the same question. Every filo
adapter is classic CAN, but the hydra family has at least one classic-only
member (USBcan Light 4xHS), so treat FD as a per-device property: read
`fd_capable` from `kv_scan()` rather than inferring it from the family.

The full product-ID table is [`src/ids.c`](src/ids.c), under vendor ID
`0x0BFD`. The channel count listed there is a fallback; the real one is read
from the card when it is opened. PCI/PCIe cards that `linuxcan` also supports
(`pcican`, `pciefd`, and so on) are not USB and are out of scope.

## Build

```bash
brew install libusb
make
```

This produces, in `build/`:

- `libKvaserUSB.1.dylib` and the `libKvaserUSB.dylib` symlink
- `kvaser_ls`, lists adapters and channels
- `kvaser_recv <handle> [bitrate] [data-bitrate]`, dumps traffic
- `kvaser_send <handle> <id> [bytes...]`, transmits one frame

```bash
./build/kvaser_ls
#   handle=0x00020500  Kvaser Leaf Pro HS v2  pid=0x0107  CAN1/1  FD

./build/kvaser_recv 0x00020500 500000              # classic CAN
./build/kvaser_recv 0x00020500 500000 2000000      # CAN FD, 500 k / 2 M
./build/kvaser_send 0x00020500 123 11 22 33
KVASERUSB_DEBUG=1 ./build/kvaser_ls                # trace USB activity
```

A handle encodes where the channel is: `usb_bus << 16 | usb_address << 8 |
channel_index`. It changes when the adapter moves to another port, so call
`kv_scan()` (or `kvaser_ls`) rather than hard-coding one.

## Install

```bash
sudo make install     # /usr/local/{lib,include,bin}
sudo make uninstall
```

## Using the API

```c
#include <KvaserUSB.h>

kv_channel_info ch[KV_MAX_CHANNELS];
int n = kv_scan(ch, KV_MAX_CHANNELS);

kv_open_opts opts = { .bitrate = 500000, .sample_point = 80 };
if (kv_open(ch[0].handle, &opts) == KV_OK) {
    kv_msg m;
    while (kv_read(ch[0].handle, &m) == KV_OK)
        /* m.id, m.flags, m.data[0..m.len-1], m.ts_us */;
    kv_close(ch[0].handle);
}
```

Link with `-lKvaserUSB`. `kv_read()` is a non-blocking poll of the receive
queue: it returns `KV_ERR_EMPTY` when nothing is waiting, so sleep briefly
between empty reads. For CAN FD, set `opts.can_fd = 1` and `opts.data_bitrate`
on a hydra adapter; `kv_open()` returns `KV_ERR_PARAM` if the hardware cannot
do FD.

Bit timing is derived from `bitrate` and `sample_point` for the common rates.
For bus configurations that calculation cannot reach, set `opts.tseg1`,
`opts.tseg2` and `opts.sjw` (and their `data_` counterparts for the FD data
phase) to send explicit segment values, in time quanta, to the firmware.

Every entry point returns `KV_OK` or a negative `KV_ERR_*`, which
`kv_strerror()` turns into a message.

## Source map

| File | Contents |
| --- | --- |
| `src/usb.c` | libusb transport, discovery, receive thread, command/response rendezvous, public API |
| `src/leaf.c` | filo command protocol (Leaf family, classic CAN) |
| `src/helios.c` | helios command protocol (USBcan II generation) |
| `src/hydra.c` | hydra command protocol, HE routing, CAN FD |
| `src/ids.c` | product-ID table |
| `src/queue.c` | receive ring, DLC/length and bit-timing helpers |

## Status

The filo path (Leaf, USBcan II) and the hydra path (Leaf Pro v2, U100,
USBcan Pro v2) work for enumeration, bus on/off, receive and transmit.
Points worth knowing about:

- Timestamps (`ts_us`) are decoded on both paths from the adapter's 48-bit
  high-resolution clock, scaled by the timer frequency the firmware reports
  (16/24/32 MHz on filo, 24/80 MHz on hydra). They count from the adapter's
  own epoch and are not synchronised to the host clock.
- Error frames arrive as `KV_FLAG_ERROR` with `data[0..3]` = receive error
  counter, transmit error counter, controller bus status (`KV_BUSSTAT_*`)
  and the firmware's error factor. The hardware does not report per-error
  detail (bit/stuff/CRC), so neither can the driver.
- Bit timing is automatic for the common rates; explicit `tseg1`/`tseg2`/
  `sjw` values can be passed through `kv_open_opts` for anything else.

## Not implemented

- t-script environment variables, and the Memorator logging and file
  commands. Those are a filesystem protocol in their own right, for a device
  class this driver does not otherwise address.
- PCI and PCIe cards, which are out of scope for a USB driver.

## License

Dual licensed **BSD-3-Clause OR GPL-2.0**, matching Kvaser's `linuxcan`.
See [LICENSE](LICENSE). Kvaser AB is not affiliated with this project.
