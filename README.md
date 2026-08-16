# macOS CAN Drivers

User-space drivers for USB CAN / CAN FD adapters on macOS.

Two libraries live here. Both talk to the hardware directly over
[libusb](https://libusb.info/), so there is no kernel extension, no system
extension and nothing to approve in System Settings. You build them, you
load them, and you can read every byte that goes to the adapter.

| Directory | Library | Hardware | API | License |
| --- | --- | --- | --- | --- |
| [`PeakUSB/`](PeakUSB) | `libPeakUSB.dylib` | PEAK PCAN-USB family | PCAN-Basic compatible | GPL-2.0-or-later |
| [`KvaserUSB/`](KvaserUSB) | `libKvaserUSB.dylib` | Kvaser Leaf / USBcan / Hydra family | small C API (`kv_*`) | BSD-3-Clause OR GPL-2.0 |

The wire protocols are ported from the vendors' own Linux drivers, PEAK's
`peak-linux-driver` 9.2.0 under the GPL and Kvaser's `linuxcan` under BSD or
GPL. Nothing here is reverse engineered from, or wrapped around, a macOS
library whose sources are not public.

## Why

macOS is the weak spot of an otherwise well-served ecosystem. Both vendors
publish their Linux drivers in source form and maintain libraries for Windows,
but neither ships a macOS driver, so what exists on this platform comes from
third parties.

For PEAK hardware the widely used option ships as a binary, so its limitations
cannot be fixed from the outside. For Kvaser there is an alternative whose
sources are public, MacCAN's KvaserCAN-Library. This driver takes a different
shape from it, and the two sections below say how it relates to each.

Because these two drivers are ports of the vendors' own Linux sources, a
firmware quirk or a new product ID is a patch you can write yourself instead
of a support ticket you wait on.

## Relationship to MacCAN PCBUSB

Credit where it is due: the reason PEAK adapters work on macOS at all is
[MacCAN PCBUSB](https://github.com/mac-can/PCBUSB-Library) by Uwe Vogt of
UV Software. It got there first, and it established `libPCBUSB.dylib` as the
filename that SavvyCAN's `peakcan` plugin and python-can go looking for. The
`install-compat` target in this repository works only because of that
convention.

`PeakUSB` is an independent implementation, not a fork of PCBUSB. Its
protocol layer is ported from PEAK's GPL `peak-linux-driver` and driven from
user space with libusb, and its public header is derived from PEAK's own
`PCANBasic.h`, which ships with that same driver under LGPL-2.1 and whose
section 3 permits use under the GPL. Both sources are PEAK's. No part of it
comes from PCBUSB, which is proprietary and could not be used here.

What the two do share is the `CAN_*` entry points, because that is PEAK's API
and implementing it is the whole point. That shared surface is what lets this
library stand in for PCBUSB.

That substitution deserves a warning. If you run `install-compat`, a library
named `libPCBUSB.dylib` on your system is no longer PCBUSB, and any bug you
hit is ours, not UV Software's. Report it here. Keep your original
`libPCBUSB.dylib` somewhere you can put it back from, and check that any
backup you make is a real copy rather than a symlink into the file you are
about to replace.

The specific problems that prompted this work were seen with the 0.13 binary
build of PCBUSB:

- only CAN1 of a PCAN-USB Pro / Pro FD was reachable, leaving the second bus
  invisible;
- receive frames went missing because USB IN transfers were not kept in
  flight, so traffic often showed up only after an unplug and replug;
- `PCAN_ATTACHED_CHANNELS` was not implemented, so Qt 6 applications such as
  SavvyCAN could not enumerate anything.

Later PCBUSB releases may well have addressed some of these. The list is a
record of why this driver was written, not a running comparison.

## Relationship to MacCAN KvaserCAN-Library

MacCAN also publishes
[KvaserCAN-Library](https://github.com/mac-can/KvaserCAN-Library), dual
licensed BSD-2-Clause OR GPL-3.0-or-later, which drives Kvaser adapters on
macOS. Nothing in the section above carries over: unlike PeakUSB and PCBUSB,
these two have nothing in common.

`KvaserUSB` is not a fork of it and contains none of its code. Both drivers
necessarily speak the same wire protocols, because Kvaser's firmware defines
them and there is only one way to talk to the hardware, but this one takes
them from `linuxcan`. KvaserCAN-Library was not read, copied from or
consulted.

They do not share an interface either. KvaserCAN-Library exposes CAN API V3
as a C++ class library with a C wrapper, `libKvaserCAN` and `libUVCANKVL`;
this one exposes a set of `kv_*` C functions from `libKvaserUSB.dylib`. No
function name, header name or library name is common to the two. So there is
no compatibility shim here of the kind `install-compat` provides for PCBUSB,
and none is needed, because nothing looks for the other's filename. Install
both if you want; they will not collide.

The licences differ, and in one direction they do not mix. `KvaserUSB` is
BSD-3-Clause OR GPL-2.0, matching `linuxcan`. KvaserCAN-Library offers
GPL-3.0-or-later, which GPL-2.0-only code cannot absorb. Its BSD-2-Clause
option would permit reuse, but none has taken place, and keeping the
provenance of this driver traceable to `linuxcan` alone is deliberate.

What differs in practice is coverage. KvaserCAN-Library handles the Leaf and
Mhydra families and reaches channel CAN1 of a multi-channel adapter. This
driver adds the helios family for the USBcan II generation, and opens every
channel of an adapter rather than the first.

## Requirements

- macOS on Apple Silicon or Intel
- Xcode command line tools (`xcode-select --install`)
- `libusb`, via `brew install libusb`

## Build

```bash
make            # builds both drivers
make -C PeakUSB # or just one of them
```

Everything generated lands in `PeakUSB/build/` and `KvaserUSB/build/`:
the `.dylib`, and the `pcan_*` / `kvaser_*` command line tools.

The build targets whatever architecture the host runs, so on Apple Silicon
you get arm64 and on an Intel Mac you get x86_64. There is nothing
architecture-specific in the sources; both were developed and tested on
Apple Silicon.

A universal build is possible, but only if the libusb you link against is
itself universal. Homebrew ships a single-architecture build, so this fails
with a "found architecture arm64, required architecture x86_64" link error
unless you point at a universal libusb yourself:

```bash
make ARCHS="arm64 x86_64"
```

## Try it

```bash
./PeakUSB/build/pcan_ls
# PCAN USB channels: 2
#   handle=0x51  PCAN-USB Pro FD     CAN1  id=2164195328  FD       available
#   handle=0x52  PCAN-USB Pro FD     CAN2  id=2164195328  FD       available

./PeakUSB/build/pcan_recv 0x51                  # CAN FD, 1 M / 2 M
./PeakUSB/build/pcan_send 0x51 123 11 22 33
```

```bash
./KvaserUSB/build/kvaser_ls
./KvaserUSB/build/kvaser_recv 0x00020500 500000
./KvaserUSB/build/kvaser_send 0x00020500 123 11 22 33
```

Set `PEAKUSB_DEBUG=1` or `KVASERUSB_DEBUG=1` to trace USB activity on stderr.

## Install

```bash
sudo make install                      # both, into /usr/local
sudo make -C PeakUSB install-compat    # + the libPCBUSB.dylib compatibility link
```

`install-compat` matters for anything that hard-codes PEAK's macOS library
name. SavvyCAN's `peakcan` plugin and python-can both look for
`libPCBUSB.dylib` and never find `libPeakUSB.dylib` on their own:

```python
import can
bus = can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=500000)
```

Use `PREFIX=` to install somewhere else, `DESTDIR=` when staging a package:

```bash
make install PREFIX=$HOME/.local
```

## How they work

Both libraries follow the same shape, because both vendor protocols do:

- **Discovery** enumerates the USB bus by vendor ID and matches the product ID
  against a table of known adapters, then asks the firmware for the real
  channel count, serial number and endpoint layout.
- **Receive** keeps 8 bulk IN transfers of 4 KiB permanently in flight, served
  by one libusb event thread per adapter. Completed buffers are split into
  protocol records. A URB can end mid-record, so both drivers carry a
  fragment across callbacks. Decoded frames are pushed into a per-channel
  ring of 65536 messages, which drops the oldest frame when it overflows.
- **Transmit** is a synchronous bulk write on the channel's OUT endpoint.
- **Commands** (bit timing, bus on/off, silent mode, LEDs) go out on a
  separate endpoint; where the firmware answers, the caller blocks on a
  condition variable until the receive thread hands over the matching reply.

`PeakUSB` additionally exposes a pipe file descriptor through
`PCAN_RECEIVE_EVENT`, so applications can `poll()` for traffic instead of
spinning. `KvaserUSB` has no equivalent yet, so `kv_read()` is a non-blocking
poll of the queue.

## Limits worth knowing

- **One process per adapter.** Claiming a USB interface is exclusive, so two
  applications cannot share one adapter. Both channels of a two-channel
  adapter can still be opened from within a single process. This is the same
  constraint as a single usbfs claim on Linux.
- If another program already holds the adapter, opening fails with
  `PCAN_ERROR_HWINUSE` / `KV_ERR_BUSY`. Quit the other program first.
- Classic PCAN-USB and PCAN-USB Pro are CAN 2.0 hardware: `CAN_InitializeFD`
  returns `PCAN_ERROR_ILLOPERATION` for them, as it should.
- Timestamps come from the adapter's own clock. They are monotonic and
  microsecond-scaled, but they are not synchronised to the host clock.
- Only USB adapters are covered. The PCI/PCIe cards that `linuxcan` supports
  are out of scope.

## Repository layout

```text
PeakUSB/          PCAN-Basic compatible driver
  include/        public header, the PCAN-Basic API
  src/            libusb transport, uCAN + classic + USB Pro protocols
  tools/          pcan_ls, pcan_recv, pcan_send
KvaserUSB/        Kvaser driver
  include/        public header, the kv_* API
  src/            libusb transport, filo (leaf) + hydra protocols
  tools/          kvaser_ls, kvaser_recv, kvaser_send
```

Each driver is versioned and released on its own, with its own changelog:
[PeakUSB](PeakUSB/CHANGELOG.md) and [KvaserUSB](KvaserUSB/CHANGELOG.md).

## License

The two drivers carry the licenses of the vendor sources they are ported
from, so they are licensed separately:

- `PeakUSB/` is **GPL-2.0-or-later**, see [PeakUSB/LICENSE](PeakUSB/LICENSE).
  The PCAN-Basic function and constant names are used for compatibility with
  existing applications.
- `KvaserUSB/` is **BSD-3-Clause OR GPL-2.0**, the same dual license as
  `linuxcan`, see [KvaserUSB/LICENSE](KvaserUSB/LICENSE).

Every source file carries an SPDX identifier. PEAK, PCAN, Kvaser and their
product names are trademarks of their respective owners; this project is not
affiliated with or endorsed by either company.
