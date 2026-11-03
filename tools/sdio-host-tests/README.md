# CH32V4 SDIO host model

Run from the HAL repository root after generating the sibling `../ch32-data`
PAC:

```sh
cargo test --manifest-path tools/sdio-host-tests/Cargo.toml \
  --target x86_64-unknown-linux-gnu -- --test-threads=1
```

`--target x86_64-unknown-linux-gnu` is the Linux target of the reference
command; no CI job currently runs this harness. It is a plain `std` crate, so
omit `--target` to run natively on macOS (all five pre-existing harnesses
passed this way on 2026-11-03).

The harness compiles the production V4x7 SDIO and DMA implementations and the
generated SDIO/DMA register types directly. RAM registers substitute for MMIO;
the wrapper models command responses, write-one-to-clear flags, FIFO/DMA data,
clock gating and interrupt delivery. Tests must run serially because these
registers and driver state are shared singletons. The 32-bit DMA address model
reconstructs the upper address bits from the current test thread's stack;
transfer buffers must remain on that stack, in the same 4 GiB address window.
Heap and static DMA buffers are not supported by this model.

Tests cover SDSC byte addressing, SDHC block addressing, 1/4-bit configuration,
high-speed selection, register/FIFO byte order, maximum SDXC capacity, divider
limits, single/multiple block transfers, DMA/SDIO completion ordering and actual
waker invocation. Fault injection checks command/CRC/timeout/data/DMA errors,
incomplete DMA, late errors, cancellation during power-up, transfer and card
programming, invalid requests and reinitialization. Checked DMA transfers return
an error through `has_error`; ordinary DMA transfers retain their panic on a
transfer error. The production DMA ring-buffer tests run here as well.

The model does not validate physical SD bus timing, pull-ups, signal integrity,
real card compatibility or actual DMA bus arbitration. All modeled writes stay
in RAM. The `hal_sdio` board example is read-only; no successful hardware test
has been recorded yet.
