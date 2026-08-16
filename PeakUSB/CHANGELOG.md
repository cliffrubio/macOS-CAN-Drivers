# Changelog

Notable changes to PeakUSB. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the version
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

KvaserUSB is versioned separately, see [its changelog](../KvaserUSB/CHANGELOG.md).

## [1.0.2] - 2026-08-16

### Added

- CAN XL, for the PCAN-USB XL (product ID `0x0030`). `CAN_InitializeXL`,
  `CAN_ReadXL` and `CAN_WriteXL` complete the seventeen entry points PEAK
  declares, along with `TPCANMsgXL` and its 2048-byte payload. XL frames get
  their own receive queue: at that payload size a ring as deep as the CC/FD
  one would need 137 MB, so it is 256 frames deep and sized for burst
  absorption rather than long buffering.

- Eleven PCAN-Basic parameters that were previously unimplemented:
  `PCAN_ALLOW_ECHO_FRAMES`, `PCAN_INTERFRAME_DELAY`,
  `PCAN_CHANNEL_IDENTIFYING`, `PCAN_BUSSPEED_NOMINAL`, `PCAN_BUSSPEED_FD`,
  `PCAN_BITRATE_INFO_CC`, `PCAN_DEVICE_PART_NUMBER`,
  `PCAN_HARD_RESET_STATUS`, `PCAN_BITRATE_ADAPTING`, `PCAN_5VOLTS_POWER`.
  `PCAN_BITRATE_INFO_CC` was missing from the header entirely.

  Two are worth singling out. `PCAN_INTERFRAME_DELAY` feeds the transmit
  throttle, so frames are paced at the requested rate instead of at wire
  speed. `PCAN_ALLOW_ECHO_FRAMES` asks the adapter to return a copy of each
  frame it transmits: measured on a PCAN-USB Pro FD, the flag is accepted and
  reads back, but that firmware returns no echo frames, so callers should not
  depend on it without checking.

- The digital I/O parameters, `PCAN_IO_DIGITAL_CONFIGURATION`,
  `PCAN_IO_DIGITAL_VALUE`, `PCAN_IO_DIGITAL_SET` and `PCAN_IO_DIGITAL_CLEAR`.
  These reach the wire only on the PCAN-USB Chip, which is the only device
  with the pins; every other adapter returns `PCAN_ERROR_ILLOPERATION`.

- `PCAN_DEVICE_GUID`, derived from the serial number, product and channel so
  it is stable for a given adapter and distinct between its channels. PEAK
  has no GUID on the wire either; it synthesises one the same way.

- `PCAN_BITRATE_INFO_XL` and `PCAN_BUSSPEED_XL`, which were missing from the
  header, and `PCAN_IO_ANALOG_VALUE`. The analog read reports
  `PCAN_ERROR_ILLPARAMTYPE` on the one device that has the pin, because the
  firmware answers on a command endpoint this driver does not poll for
  replies; reporting that is better than returning a fabricated value.

- The logging and tracing parameters now answer instead of being rejected.
  This driver has no trace-file machinery, so they report the feature as off,
  accept a request to turn it off, and route `PCAN_LOG_TEXT` to the same
  stderr channel as `PEAKUSB_DEBUG`. A caller probing for the feature gets a
  definite answer rather than `PCAN_ERROR_ILLPARAMTYPE`.

## [1.0.1] - 2026-08-16

### Added

- Transmit backpressure. The adapter buffers outgoing frames and, once that
  buffer is full, discards them without reporting anything, so an application
  writing faster than the bus can carry lost frames with no way to detect it.
  `CAN_Write` and `CAN_WriteFD` now return `PCAN_ERROR_XMTFULL` instead, which
  applications are expected to handle by retrying. The limit is modelled from
  the configured bit rate rather than a fixed frame count, so it does not
  depend on any particular adapter's buffer depth, and it does not trigger for
  an application transmitting at or below the bus rate. Measured on a
  PCAN-USB Pro FD at 500 kbit/s: an unthrottled burst of 200 frames delivered
  124; with backpressure and retry it delivers 197.

### Fixed

- Classic PCAN-USB dropped most received frames. The adapter emits fixed
  64-byte IN packets and one bulk completion aggregates many of them, each
  with its own record-count header; the decoder read a single header at the
  start of the whole transfer, so everything after the first packet was
  discarded.
- Every USB Pro transmit was four bytes too long: the record length came from
  the receive structure, which carries a timestamp the transmit record does
  not have.
- The library is now safe to call from more than one thread. The device list,
  the handle map and every channel field were reachable without a lock while
  `pcan_usb_refresh()` could free and rebuild devices, so two threads doing
  ordinary work could hit a use-after-free. All entry points now serialise.
- Freeing a device tore down its queues while the receive thread was still
  running, whenever no channel happened to be marked initialised - which is
  the state left by an open that failed after claiming the interface.
- Applications waiting on `PCAN_RECEIVE_EVENT` could hang forever. Whether to
  signal the pipe was decided from a queue count sampled outside the lock that
  guarded the insertion, and once a wakeup was missed every later frame missed
  it too.
- `CAN_GetErrorText` reported "No error" for every bus condition, including
  bus-off, because the bus bits were masked away before the lookup.
- Classic timestamps jumped backwards by about 2.8 seconds on each 16-bit
  rollover, and the adapter's periodic clock-sync record was ignored, so a
  quiet bus lost the wrap entirely. USB Pro timestamps wrapped to zero every
  71 minutes.
- Classic internal records for the analog, bus-load and clock-sync functions
  were skipped using the length nibble, which does not describe them; a packet
  carrying one alongside a frame was parsed from the wrong offset thereafter.
- Classic bus-error records were requested from the adapter and then thrown
  away, so those channels never produced an error frame and their error
  counters stayed at zero.
- Classic bus status was reported one level too severe: a light condition came
  back as heavy and a heavy condition as error-passive. Error-passive is now
  derived from the error counters, as the hardware intends.
- Bus-off is preceded by a short drain delay, so frames handed over
  immediately before closing are not aborted inside the adapter.
- `CAN_InitializeFD` accepted bit-timing values that overflowed the hardware
  registers and clock frequencies the hardware does not have, silently
  running the channel at the wrong rate while reporting success. Bit-rate
  strings containing spaces, which PEAK documents as valid and uses in its own
  examples, were rejected.
- `CAN_GetValue` filled part of a caller's DWORD and still reported success
  when the buffer was too small, leaving the rest uninitialised.
- `CAN_SetValue` accepted a NULL or zero-length buffer and applied the value
  zero, quietly disabling whichever feature was addressed.
- `CAN_LookUpChannel` returned an arbitrary channel for criteria it does not
  implement, and `PCAN_CHANNEL_VERSION` never reported the device version
  because the library-version case shadowed it.
- A failed `CAN_Initialize` on a busy channel had already overwritten that
  channel's stored bit timing, so it then reported a rate it was not using.
- `pcan_send -b` with no value transmitted a frame with a garbage identifier
  instead of reporting the missing argument, and a non-numeric identifier was
  silently sent as zero. `pcan_recv` looped forever printing uninitialised
  stack if a read failed for any reason other than an empty queue.

- Bounded every field read in the uCAN record decoder by the record's own
  length. The decoder trusted the record type to imply the layout and read
  fixed offsets regardless of how many bytes the firmware actually sent:
  - The 12-byte common header, including the timestamp, was read from records
    the splitter only guaranteed to be 4 bytes long.
  - `CANFD_MSG_CAN_RX` checked for 20 bytes, but its fixed part runs to offset
    28, so the flags and CAN identifier were read past the record.
  - The payload length came from the DLC alone, so a record claiming DLC 15
    copied 64 bytes out of a record that may have carried none. It is now
    clamped to the bytes the record actually contains.
  - `CANFD_MSG_STATUS`, `CANFD_MSG_ERROR` and `CANFD_USB_MSG_OVERRUN` had no
    length check at all before dereferencing their 16-byte structures.

  Each of these could read past the end of a 4 KiB receive buffer on a
  truncated or corrupt record.

## [1.0.0] - 2026-08-16

First public release. A PCAN-Basic compatible, user-space driver for PEAK
PCAN-USB adapters, using libusb with no kernel or system extension.

### Added

- All fourteen `CAN_*` entry points of the PCAN-Basic API: `CAN_Initialize`,
  `CAN_InitializeFD`, `CAN_Uninitialize`, `CAN_Reset`, `CAN_GetStatus`,
  `CAN_Read`, `CAN_ReadFD`, `CAN_Write`, `CAN_WriteFD`, `CAN_FilterMessages`,
  `CAN_GetValue`, `CAN_SetValue`, `CAN_GetErrorText` and `CAN_LookUpChannel`.
- Six adapter models across three wire protocols: classic PCAN-USB, the
  PCAN-USB Pro record format, and uCAN for the CAN FD family.
- Both channels of two-channel adapters (Pro and Pro FD) reachable from a
  single process, addressed as `PCAN_USBBUS1` upward.
- `PCAN_ATTACHED_CHANNELS` and `PCAN_ATTACHED_CHANNELS_COUNT`, so Qt 6
  applications such as SavvyCAN can enumerate adapters.
- `PCAN_RECEIVE_EVENT` returns a pipe file descriptor usable with `poll()`
  or `select()`, in place of the Windows event handle of the original API.
- `make install-compat` installs a `libPCBUSB.dylib` symlink for applications
  that look for PEAK's macOS library by that name.
- Command line tools `pcan_ls`, `pcan_recv` and `pcan_send`.
- Eight bulk IN transfers of 4 KiB kept permanently in flight, feeding a
  per-channel ring of 65536 messages that drops the oldest frame on overflow.
- `PEAKUSB_DEBUG=1` traces USB activity on stderr.

### Known limitations

- Claiming a USB interface is exclusive, so one adapter cannot be shared
  between two processes.
- Classic PCAN-USB and PCAN-USB Pro are CAN 2.0 hardware; `CAN_InitializeFD`
  returns `PCAN_ERROR_ILLOPERATION` for them.
- Timestamps come from the adapter's own clock and are not synchronised to
  the host clock.
- Only USB hardware is covered; the PCI and PCIe cards that PEAK's Linux
  driver also supports are out of scope.
