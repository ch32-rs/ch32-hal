# CH32V4 flash host tests

Run the production V4x7 flash driver against generated PAC register types with a
modeled status register:

```sh
CARGO_TARGET_DIR=/tmp/ch32v4-flash-host-target cargo test \
  --manifest-path tools/flash-host-tests/Cargo.toml -- --test-threads=1
```

`--target x86_64-unknown-linux-gnu` is only the Linux CI reference. The harness
is a plain `std` crate and runs natively on macOS too (omit `--target`).

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
- a stalled `WRBSY` still produces `Error::Timeout`.

The driver's `Flash::blocking_write` targets the absolute address `0x0800_0000`,
so the tests call the same `program_page` routine with a RAM buffer instead of
the public wrapper. Erase timing, option bytes and real flash endurance remain
board-only checks (`hal_flash --features flash-write-test`).
