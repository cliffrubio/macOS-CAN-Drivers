# Changelog

Notable changes to KvaserUSB. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the version
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

PeakUSB is versioned separately, see [its changelog](../PeakUSB/CHANGELOG.md).

## [1.0.2] - 2026-08-16

### Added

- Transmit flow control. The firmware acknowledges each frame it has sent, so
  the driver now counts what the adapter still holds and `kv_write()` returns
  the new `KV_ERR_TX_FULL` once that reaches the firmware's window, rather
  than handing over frames the adapter will discard without telling anyone.
  The window comes from `maxOutstandingTx` in the software-info reply, with a
  conservative default when the firmware does not report one.
  `kv_tx_outstanding()` exposes the current count.

- `kv_capabilities()`, which asks the firmware what it supports instead of
  inferring it from the product table. The table remains the fallback for
  devices that do not answer, but it is no longer the only source: it needed a
  hand-written exception for the USBcan Light 4xHS precisely because a static
  list cannot be right for every unit.

- `kv_get_busparams()`, reading back the bit timing the controller is actually
  running rather than echoing what was requested. Worth having: testing the
  PEAK driver showed hardware silently refusing timings that were arithmetically
  valid, and the same class of surprise is likely here.

- `kv_bus_load()`, the adapter's own bus-load measurement in tenths of a
  percent, and `kv_flush_tx()`, which discards whatever the adapter has queued
  for transmission and clears the transmit window with it.

- Firmware error events (`CMD_ERROR_EVENT`) and fatal firmware errors
  (`CMD_FATAL_ERROR`) are decoded and delivered as error frames, distinct from
  the CAN bus errors already reported.

- The **helios** command set, restoring the original USBcan, VCI-2, USBcan II
  and Memorator (product IDs 2 to 5). These were removed in 1.0.1 because the
  driver only spoke filo, and the two share command numbers without sharing
  layouts: helios `cmdRxCanMessage` is 20 bytes with the frame at offset 4,
  where filo is 24 with it at offset 10, so every frame failed the length
  check. Its timestamp is also a 16-bit tick whose high half arrives
  separately in `CMD_CLOCK_OVERFLOW_EVENT`, rather than filo's 48-bit counter.

  These adapters also acknowledge transmits, so the flow control added in this
  release covers them as well as the hydra family.

- Periodic transmit: `kv_auto_tx()`, `kv_auto_tx_stop()` and
  `kv_auto_tx_count()`. The adapter repeats a frame on its own timer, so a
  heartbeat keeps running even if the host stalls.

- `kv_set_tx_interval()`, the minimum gap the firmware leaves between frames
  it transmits. Enforced by the adapter rather than by the host, so unlike a
  host-side delay it survives a stalled caller.

- Device and channel detail the firmware can report: `kv_transceiver()` for
  the fitted transceiver, `kv_interface_info()` for the CAN controller chip,
  `kv_card_info2()` for the PCB identifier, `kv_get_busparams_tq()` for the
  timing expressed in time quanta rather than as a bit rate, and
  `kv_get_device_mode()` / `kv_set_device_mode()`. `kv_beep()` sounds the
  buzzer on devices that have one.

- `kv_license()` for the device's licence bits, `kv_io_port_read()` and
  `kv_io_port_write()` for the hardware-specific I/O ports on devices that
  have them, and `kv_get_driver_mode()`, which reports silent mode as the
  firmware sees it rather than as it was requested.

## [1.0.1] - 2026-08-16

### Added

- Optional explicit bit timing in `kv_open_opts`: `tseg1`, `tseg2`, `sjw` for
  the arbitration phase and `data_tseg1`, `data_tseg2`, `data_sjw` for the FD
  data phase. Left at zero the segments are derived from `bitrate` and
  `sample_point` exactly as before, so existing code is unaffected. This
  reaches bus configurations the automatic calculation cannot, because it
  works from a fixed quanta count per bit-rate band.
- `KV_BUSSTAT_RESET`, `KV_BUSSTAT_ERROR_PASSIVE` and `KV_BUSSTAT_BUSOFF` for
  reading the controller status now carried in error frames.
- Firmware version on filo adapters, parsed from the software-info response
  that was previously requested and then discarded. `kv_device_info()`
  reported 0.0.0 for these devices before.

### Fixed

- Hydra adapters now timestamp received frames. Both hydra receive paths left
  `ts_us` at zero: the 48-bit counter in classic records and the 64-bit FPGA
  timestamp in CAN FD records are now decoded.
- Filo timestamps were wrong, not merely absent. Raw device ticks were stored
  as if they were microseconds, making them 16 to 32 times too fast. Both
  families now scale ticks by the high-resolution timer frequency the firmware
  reports (16/24/32 MHz on filo, 24/80 MHz on hydra).
- Hydra adapters produced no error frames at all; `CMD_CAN_ERROR_EVENT` was
  not handled. It now delivers an error frame and updates the counters that
  `kv_status()` reports.
- Error frames carry the full set of values the firmware provides. `data[]`
  was receive and transmit error counters only, and is now those two plus the
  controller bus status and the firmware's error factor.
- Bounded every field read in both protocol decoders by the command's own
  length. The hydra CAN FD path sized its payload copy from the DLC alone, so
  a record claiming DLC 15 copied 64 bytes out of a 32-byte command. Five filo
  handlers cast the buffer to structures of 14 to 32 bytes and dereferenced
  fields before checking the length, which the splitter only guarantees to be
  2 bytes or more. Each could read past the end of a 4 KiB receive buffer.
- A hydra command split across two USB transfers with fewer than 8 bytes in
  the first was discarded instead of carried over, because the length field
  sits at bytes 4 and 5 and the splitter refused to buffer anything shorter
  than that. The frame was lost and the stream desynchronised until it
  happened to realign. Short tails are now carried across transfers, and the
  reassembly path waits for the 8 bytes it needs before reading the length.

- The original USBcan, VCI-2, USBcan II and Memorator (product IDs 2 to 5)
  were listed as supported but could never work. `linuxcan` drives them from
  `usbcanII/`, which speaks a third command set whose records differ from the
  filo ones this driver implements, so every received frame failed the length
  check and was discarded. They have been removed from the ID table until a
  back end for that protocol exists.
- Bit timing produced fractional prescalers at common CAN FD data rates, which
  the firmware either rejects or honours as some other bit rate. The number of
  time quanta per bit is now chosen so the adapter's CAN clock divides exactly,
  and that clock is read from the device rather than assumed. The
  resynchronisation jump width scales with the bit length instead of always
  being one quantum.
- A command reply woke the waiting caller before the values it carried had
  been stored, so a caller could resume and read the previous contents. The
  payload is now applied first and the waiter woken last.
- A reply arriving in the moment between sending a request and registering
  interest in the answer was discarded, stalling the caller for the full
  one-second timeout; requests now register interest before transmitting. A
  reply that landed exactly as the deadline expired was also thrown away.
- `kv_scan`, `kv_read` and `kv_write` crashed instead of returning an error
  when passed a NULL pointer.
- A card that failed to initialise left the USB interface claimed and the
  receive thread running, so the adapter stayed unusable by any other process
  and a retry silently skipped initialisation and reported success. The claim
  is now released when initialisation fails.

## [1.0.0] - 2026-08-16

First public release. A user-space driver for Kvaser USB adapters with a small
C API, using libusb with no kernel or system extension.

### Added

- Ten `kv_*` entry points covering enumeration, open and close, non-blocking
  receive, transmit, controller status, reset and device information.
- 61 product IDs across the two protocol families: filo for the classic-CAN
  Leaf and USBcan II adapters, and hydra for the newer Leaf Pro,
  Leaf v3, USBcan Pro, U100 and Memorator v2 hardware.
- CAN FD on the adapters that support it, with a separate data bit rate.
- Command line tools `kvaser_ls`, `kvaser_recv` and `kvaser_send`.
- Eight bulk IN transfers of 4 KiB kept permanently in flight, feeding a
  per-channel ring of 65536 messages that drops the oldest frame on overflow.
- `KVASERUSB_DEBUG=1` traces USB activity on stderr.

### Known limitations

- Claiming a USB interface is exclusive, so one adapter cannot be shared
  between two processes.
- Timestamps come from each adapter's own clock and are not synchronised to
  the host clock.
- Neither protocol carries per-error detail (bit, stuff, CRC), so the driver
  reports error counters and bus status but cannot report the error kind.
- Only USB hardware is covered; the PCI and PCIe cards that `linuxcan` also
  supports are out of scope.
