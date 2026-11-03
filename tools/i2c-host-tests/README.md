# CH32V4 I2C host tests

Run the production I2C driver and V4 DMA backend against generated PAC register
types with modeled I2C bus events and DMA completion:

```sh
CARGO_TARGET_DIR=/tmp/ch32v4-i2c-host-target cargo test \
  --manifest-path tools/i2c-host-tests/Cargo.toml \
  --target x86_64-unknown-linux-gnu -- --test-threads=1
```

`--target x86_64-unknown-linux-gnu` is the Linux CI reference. The harness
is a plain `std` crate and also runs natively on macOS without `--target`
(all five pre-existing harnesses passed this way on 2026-11-03).

The generated local PAC must exist at `../ch32-data/build/ch32-metapac`.
The build script reads the current production I2C source, resolves its existing
unit tests and attaches the lifecycle tests without maintaining a driver copy.
GPIO, RCC and interrupt plumbing are replaced by host fixtures. PAC accesses
model START/STOP, ADDR clearing, ACK/POS receive tails, DMA buffer movement and
error status. The shared model requires serial tests.

Coverage includes blocking/async transactions, DMA receive prefixes and CPU
tails, scattered reads, repeated START in both directions, zero-length writes,
simultaneous peripheral errors and DMA completion, cancellation, timeout,
recovery and cleanup. DMA1 CH6 is TX and CH7 is RX.

The model completes DMA on request enable and cannot establish real request
routing, IRQ latency, electrical timing, ACK/NACK waveforms or device
interoperability. Hardware acceptance still requires an external I2C target
and observation of bus timing; `hal_i2c` provides an optional 24C02 read test.
