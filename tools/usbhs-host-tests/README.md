# CH32V4 USBHS host tests

Run from the HAL repository root after regenerating the sibling `../ch32-data`
PAC:

```sh
cargo test --manifest-path tools/usbhs-host-tests/Cargo.toml \
  --target x86_64-unknown-linux-gnu -- --test-threads=1
```

`--target x86_64-unknown-linux-gnu` is the Linux target of the reference
command; no CI job currently runs this harness. It is a plain `std` crate, so
omit `--target` to run natively on macOS (all five pre-existing harnesses
passed this way on 2026-11-03).

The production driver, its endpoint/control helpers, register geometry tests,
and the generated USBHS/RCC register types are compiled directly from their
source paths. The wrapper substitutes RAM-backed registers, clock/reset/IRQ
plumbing and GPIO methods. It models endpoint DONE/NAK W0C, the read-only
TRANSFER summary bit, DMA packet copies and endpoint enable/response checks.
Do not run the tests concurrently: the model intentionally shares the same
singleton state as the hardware driver.

The lifecycle tests exercise both instances, complete 512-byte packets across
all seven endpoint pairs, cancellation, reset, endpoint disable/STALL, control
data/status/address handling, suspend/resume and actual waker invocation. A
SETUP injection after the ISR's first RX snapshot checks that a newly arriving
SETUP cannot be cleared as ordinary OUT data. Additional cases cover malformed
SETUP recovery, duplicate OUT toggles and overflow reporting.

The model does not validate USB electrical behavior, enumeration by a real
host, PHY/DMA timing, isochronous scheduling, remote wakeup or USB compliance.
The `hal_usbhs` example and its CDC echo script supply the separate board test.
