# CH32V4 Ethernet host tests

After generating the sibling `../ch32-data` PAC, run from the HAL root:

```sh
cargo test --manifest-path tools/eth-host-tests/Cargo.toml \
  --target x86_64-unknown-linux-gnu -- --test-threads=1
```

`--target x86_64-unknown-linux-gnu` is the Linux target of the reference
command; no CI job currently runs this harness. It is a plain `std` crate, so
omit `--target` to run natively on macOS (all five pre-existing harnesses
passed this way on 2026-11-03).

The harness compiles the production Ethernet driver/rings and the generated
V4x7 Ethernet/RCC register types directly. The register wrapper is shared with
the SDIO harness. It substitutes RAM for MMIO, models SMI page selection and
PHY responses, reset completion, PHY_CR reads returning zero, DMA W1C flags,
intentional DMA stops and clock/interrupt gating. Test buffers are leaked owned
allocations to satisfy the production API's static resource lifetime; tests
emulate DMA descriptor and packet updates through full host pointers.

Run tests serially: registers, PHY state and interrupt wakers are singletons.
The 16 tests cover initialization and delayed PHY readiness, clock boundaries,
invalid configuration, reset/SMI failures, descriptor chains, ownership barriers,
TX padding and queue exhaustion, RX FCS length and invalid frame rejection,
actual interrupt wakeups, PHY bit 11, link changes, token drop/callback unwind,
explicit fault recovery, failed recovery and late interrupts after Drop. The
initialization test also rejects writes to reserved DMA/MMC control bits.

This model does not execute a real DMA engine, validate bus ordering/timing on
silicon, negotiate a physical Ethernet link or calibrate PHY startup delays.
MAC speed/FES behavior needs hardware confirmation: the RM bit table calls FES
read-only, while EVT writes it. The implementation uses the integrated PHY's
resolved speed and writes only duplex/enables. The RM marks DMASR/DMAIER bit 13
reserved even though an EBS description mentions it and EVT retains generic FBE
definitions; the driver does not enable or acknowledge that reserved interrupt.
Unexpected process-stop interrupts require explicit `recover()`.

`examples/ch32v467/hal_eth` and `eth_echo_test.py` provide the separate physical
test. No real PHY identification or frame exchange has passed on hardware yet.
