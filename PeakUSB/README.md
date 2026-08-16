# PeakUSB

A user-space, PCAN-Basic compatible driver for PEAK PCAN-USB adapters on
macOS. The public header is [`include/PeakUSB.h`](include/PeakUSB.h); the
built library is `libPeakUSB.1.dylib`.

The USB protocols are adapted from PEAK's `peak-linux-driver` 9.2.0 (GPL-2.0)
and driven from user space with [libusb](https://libusb.info/). No kernel or
system extension is involved.

## Supported adapters

| Device | PID | Channels | Protocol |
| --- | --- | --- | --- |
| PCAN-USB | `0x000c` | 1 | classic 16-byte commands |
| PCAN-USB Pro | `0x000d` | **2** | USB Pro records |
| PCAN-USB FD | `0x0012` | 1 | uCAN |
| PCAN-USB Pro FD | `0x0011` | **2** | uCAN |
| PCAN-Chip USB | `0x0013` | 1 | uCAN |
| PCAN-USB X6 | `0x0014` | 2 per USB interface | uCAN |
| PCAN-USB XL | `0x0030` | 2 | CAN XL |

PCAN-USB and PCAN-USB Pro are CAN 2.0 hardware, so `CAN_InitializeFD()`
returns `PCAN_ERROR_ILLOPERATION` for them. Use `CAN_Initialize()` with an
SJA1000 `BTR0BTR1` value such as `PCAN_BAUD_500K`.

Channels are handed out as `PCAN_USBBUS1` to `PCAN_USBBUS8` (`0x51` to `0x58`) in
USB bus/address order, so an adapter keeps its handle as long as it stays in
the same port. Both channels of a Pro / Pro FD are opened from **one**
process; two applications cannot share one adapter (see the note on USB
claims in the [top-level README](../README.md#limits-worth-knowing)).

## Build

```bash
brew install libusb
make
```

This produces, in `build/`:

- `libPeakUSB.1.dylib` and the `libPeakUSB.dylib` symlink
- `pcan_ls`, lists adapters and channels
- `pcan_recv [handle] [bitrate]`, dumps traffic
- `pcan_send [-b bitrate] [handle] <id> [bytes...]`, transmits one frame

```bash
./build/pcan_ls
./build/pcan_recv 0x51                       # CAN FD, 1 M nominal / 2 M data
./build/pcan_recv 0x51 500000                # classic CAN at 500 kbit/s
./build/pcan_send 0x51 123 11 22 33 44
PEAKUSB_DEBUG=1 ./build/pcan_ls              # trace USB activity
```

## Install

```bash
sudo make install                 # /usr/local/{lib,include,bin}
sudo make install-compat          # + libPCBUSB.dylib -> libPeakUSB.1.dylib
```

The compatibility link is what makes existing macOS tooling pick this driver
up: SavvyCAN's Qt `peakcan` plugin and python-can both search for
`libPCBUSB.dylib` by name, the library published as
[MacCAN PCBUSB](https://github.com/mac-can/PCBUSB-Library) by UV Software.

`install-compat` therefore takes over a filename that belongs to another
project. This library is not PCBUSB and shares no code with it, so anything
that misbehaves afterwards is a bug here, not there. Before running it, move
your existing `libPCBUSB.dylib` out of the way as a real copy, not a symlink,
since a symlink will simply follow the file you are replacing:

```bash
sudo cp /usr/local/lib/libPCBUSB.dylib /usr/local/lib/libPCBUSB.orig.dylib
```

- **SavvyCAN**: choose *QT SerialBus > peakcan*. A Pro FD shows `usb0` and
  `usb1`; open both from the same SavvyCAN window. An empty port list means
  the compatibility link is missing.
- **python-can**: `can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=500000)`.

Uninstall with `sudo make uninstall`.

## Using the API

```c
#include <PeakUSB.h>

TPCANStatus st = CAN_InitializeFD(PCAN_USBBUS1,
    (TPCANBitrateFD)"f_clock=80000000,"
    "nom_brp=10,nom_tseg1=5,nom_tseg2=2,nom_sjw=1,"
    "data_brp=4,data_tseg1=7,data_tseg2=2,data_sjw=1");   /* 1 M / 2 M */

int evfd;
CAN_GetValue(PCAN_USBBUS1, PCAN_RECEIVE_EVENT, &evfd, sizeof(evfd));
/* poll(evfd) then drain with CAN_ReadFD() until PCAN_ERROR_QRCVEMPTY */

CAN_Uninitialize(PCAN_USBBUS1);
```

Link with `-lPeakUSB`. Implemented entry points:

`CAN_Initialize`, `CAN_InitializeFD`, `CAN_Uninitialize`, `CAN_Reset`,
`CAN_GetStatus`, `CAN_Read`, `CAN_ReadFD`, `CAN_Write`, `CAN_WriteFD`,
`CAN_FilterMessages`, `CAN_GetValue`, `CAN_SetValue`, `CAN_GetErrorText`,
`CAN_LookUpChannel`, `CAN_InitializeXL`, `CAN_ReadXL`, `CAN_WriteXL`.

`PCAN_RECEIVE_EVENT` returns a pipe file descriptor for `poll()`/`select()`,
rather than the Windows event handle of the original API.

## Source map

| File | Contents |
| --- | --- |
| `src/usb.c` | libusb transport: discovery, endpoints, receive thread, per-family decode and transmit |
| `src/pcanbasic.c` | the PCAN-Basic entry points on top of it |
| `src/bitrate.c` | `f_clock=...` string parsing and SJA1000 `BTR0BTR1` conversion |
| `src/queue.c` | per-channel receive ring |
| `src/canxl.c` | CAN XL frames, timing and queue (PCAN-USB XL) |
| `src/canxl.h` | CAN XL wire format |
| `src/ucan.h` | uCAN protocol definitions (USB FD family) |
| `src/peak_usb.h` | classic PCAN-USB and USB Pro record definitions |

## Not implemented

- Trace files. The `PCAN_TRACE_*` and `PCAN_LOG_*` parameters answer rather
  than erroring, but report the feature as off; `PEAKUSB_DEBUG=1` traces to
  stderr instead.
- `PCAN_DEVICE_GUID`, and the LAN parameters (`PCAN_IP_ADDRESS`,
  `PCAN_LAN_CHANNEL_DIRECTION`, `PCAN_LAN_SERVICE_STATUS`), which describe
  Ethernet gateways rather than USB adapters.

## License

GPL-2.0-or-later. See [LICENSE](LICENSE).

[`include/PeakUSB.h`](include/PeakUSB.h) is derived from PEAK's own
`PCANBasic.h`, distributed with `peak-linux-driver` under LGPL-2.1.
Section 3 of the LGPL allows a copy to be used under the ordinary GPL, which
is the basis on which this library relicenses it. The API constants, structure
layouts and function signatures are PEAK's; the implementation behind them is
not. PEAK-System Technik GmbH is not affiliated with this project.
