# CH32V4 Flash host tests

Run the production V4x7 flash driver against generated PAC register types with a
modeled status register:

```sh
CARGO_TARGET_DIR=/tmp/ch32v4-flash-host-target cargo test \
  --manifest-path tools/flash-host-tests/Cargo.toml \
  --target x86_64-unknown-linux-gnu -- --test-threads=1
```

`--target x86_64-unknown-linux-gnu` is the Linux target of the reference
command; no CI job currently runs this harness. It is a plain `std` crate and
runs natively on macOS too (omit `--target`).

The generated local PAC must exist at `../ch32-data/build/ch32-metapac`; the
harness reads `flash_v4x7.rs` from that tree, which is not pinned to the HAL's
Git PAC revision.

The build script reads the current `src/flash/mod.rs` and `src/flash/v4x7.rs`.
It has no generated chip PAC, so it substitutes the CH32V467WEU6 geometry for
`FLASH_SIZE`/`WRITE_SIZE` (992 KiB application flash, 256-byte fast-program
page), and drops the three ELF-only `#[link_section]` attributes so the file can
be compiled into a Mach-O host binary.

The model encodes the state sequence measured on CH32V467 fast page
programming: `BSY` is set from `FTPG` until `PGSTRT` completes, while `WRBSY`
only marks a word entering the page buffer. Coverage:

- `wait_write_buffer` accepts `BSY=1, WRBSY=0`, and `wait_idle` still times out
  on the same status;
- `program_page` completes a full 256-byte load while `BSY` stays set — the
  `flash-write-test` `code=21` (`Timeout`) regression;
- a `WRBSY` stall that starts at `FTPG` makes `program_page` report
  `Error::Timeout`; the test also asserts that the load was entered, so it
  cannot pass by timing out in the initial idle wait.

The tests take a process-wide lock, so `--test-threads=1` is no longer required,
but remains the documented invocation.

The driver's `Flash::blocking_write` targets the absolute address
`0x0800_0000`, so the tests call the same `program_page` routine with a RAM
buffer instead of the public wrapper. Because the production code stores words
straight into the flash address space and not through a PAC register, the model
never observes those stores and cannot reproduce a per-store `WRBSY` pulse; it
only drives the status flags around `FTPG`/`PGSTRT`. For the same reason the
read-back byte comparison assumes a little-endian host (macOS on both
architectures and x86_64 Linux).

`operation()` checks execution-region safety only under
`cfg(target_arch = "riscv32")`, and the harness appends these lifecycle tests
to the production driver, so `validate_execution_address` is not covered here.

Erase timing, option bytes and real flash endurance remain board-only checks
(`hal_flash --features flash-write-test`).
