# CH32V4 SPI host tests

Run the production SPI driver against generated PAC register types with a
modeled CH32V4x7 status register:

```sh
CARGO_TARGET_DIR=/tmp/ch32v4-spi-host-target cargo test \
  --manifest-path tools/spi-host-tests/Cargo.toml -- --test-threads=1
```

`--target x86_64-unknown-linux-gnu` is only the Linux CI reference. The harness
is a plain `std` crate with no chip-level PAC or link sections, so it runs
natively on macOS too (omit `--target`).

The build script reads the current `src/spi.rs` and attaches the lifecycle tests
without maintaining a driver copy. GPIO, RCC, interrupt and DMA-channel plumbing
are replaced by host fixtures. The model encodes the CH32V467 measurement that
`STATR.TXE` reads 0 while `SPE=0`; once `SPE=1`, `TXE` follows it. `BSY` is
controlled by the test.

Coverage:

- `blocking_flush` and async `flush` with `SPE=0` return immediately;
- the first `blocking_write` goes through `prepare()` -> flush before enabling
  `SPE`, which is the path that deadlocked `hal_spi`;
- with `SPE=1`, both flushes still poll `TXE && !BSY`, and the async one stays
  pending until `BSY` clears.

The model requires serial tests. It does not establish transfer timing, IRQ
latency or pin waveforms; `hal_spi` on the board remains the hardware
acceptance test.
