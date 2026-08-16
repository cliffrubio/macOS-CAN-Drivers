# Security policy

## Reporting a vulnerability

Please report security issues privately through GitHub's
[private vulnerability reporting](https://github.com/cliffrubio/macOS-CAN-Drivers/security/advisories/new)
rather than opening a public issue.

Expect an acknowledgement within a week. This is a spare-time project, so
please do not expect a same-day response.

## What is in scope

Both drivers parse data that arrives from a USB device over bulk endpoints,
and both hand buffers to callers through a C API. Reports that matter most:

- reads or writes outside a buffer in the receive path, including malformed,
  truncated or hostile USB records
- anything reachable by attaching a device that claims a supported vendor and
  product ID
- misuse of a caller-supplied buffer or length in the public API of either
  library

A malicious or faulty USB adapter is a realistic threat here: the drivers run
in user space, but they will happily parse whatever a device sends them.

## What is not in scope

- Needing physical access to plug in a device is not by itself a
  vulnerability; it is the precondition for using the driver at all.
- An adapter being unusable while another process holds it. Claiming a USB
  interface is exclusive by design.
- Bugs in libusb, in macOS, or in applications that load these libraries.

## Supported versions

Only the most recent release is supported. There are no maintenance branches.
