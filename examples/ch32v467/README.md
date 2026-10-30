# CH32V467 first bring-up

Minimal **PAC-only** examples, not a claim of CH32V467 HAL support. RCC, DMA,
Flash and other V4x7 blocks differ from V3; no `ch32_hal::init()` or Embassy is
used. The local generated PAC is taken directly from
`../../../ch32-data/build/ch32-metapac` (relative to this directory).

## Build

Run from this directory with a recent Rust toolchain:

```sh
rustup target add riscv32imc-unknown-none-elf
cargo build --release --bins
```

The default part is **CH32V467WEU6**, with the board LED on **PB12**.
Select exactly one part for another package:

```sh
cargo build --release --bins --no-default-features --features ch32v467ret6
cargo build --release --bins --no-default-features --features ch32v467vet6
```

The stock RV32IMC target deliberately uses neither native CAS, Xwchc, nor a
hard-float ABI. `qingke-rt/v4f` provides core startup and the PAC provides the
device vectors and memory layout. No custom linker memory map is maintained.
The PAC's default `c512_r200` split is 512 KiB primary flash + 200 KiB SRAM;
another 480 KiB flash follows it. The generated linker script merges the flash
regions into a 992 KiB boot alias at `0x00000000`; the stack top is `0x20032000`.
The chip option-byte split must match. Do not change option bytes blindly.

## Examples and observation

- `blinky`: the board LED on PB12 toggles every nominal 500 ms (1 Hz square
  wave). Logs clock setup and each heartbeat over **defmt RTT**, not SDI.
  PB12 is a 2 MHz push-pull output; either LED polarity blinks at the same rate.
- `rtt`: no GPIO outputs; logs before clock init and repeats clock/heartbeat
  information every nominal second. Replaces the former `sdi_print` example.
- `cpuid`: no GPIO outputs; prints a one-shot CPU identity report over defmt
  RTT after a 500 ms attachment window, following the CH585 example's output
  style. Includes `mvendorid`, `marchid`, `mimpid`, `mhartid`, `misa`, raw
  hexadecimal values, CSR presence, WCH-style identity fields, XLEN and ISA
  extension bits (including F/V). Illegal CSR reads during a probe report
  `present=false`; unrelated exceptions halt. Zero is not treated as absent.
  Decoded fields do not identify the package, and advertised ISA bits do not
  prove instruction behavior or safe atomics. It does not enable F/V or change
  the target ABI. The report is not repeated; re-run/reset for another report.
- All expose ELF symbols `BOOT_STAGE` (1 = entered init, 2 = clock/SysTick
  configured) and `HEARTBEAT` for debugger inspection without a console.

`defmt.x` retains the log metadata, `defmt-rtt` provides `_SEGGER_RTT` in RAM,
and `qingke/critical-section-impl` serializes the single-core logger. RTT does
not depend on the SDI DATA0/DATA1 addresses. `disable-blocking-mode` keeps
execution moving if the host disconnects; a full buffer can lose log data.
The panic handler is still `panic-halt`: Rust panics halt without panic text.

### probe-rs runner and local V4x7 support

The runner is `probe-rs run --chip CH32V467WEU6`, with `DEFMT_LOG=info` by
default. No flashing is done by `cargo build`.

The local `../../../probe-rs` checkout now recognizes WCH-Link family `0xA6`
and includes `CH32V4X7_Series.yaml`: six V407/V467 packages, each with 200 KiB
or 136 KiB RAM, plus generated V4x7 Flash algorithms. The local build was installed with `cargo install --path probe-rs-tools
--bin probe-rs --force --locked`; PATH now resolves to
`/Users/mono/.cargo/bin/probe-rs`, and the default Cargo runner can use it.
To explicitly select the checkout build instead, use this runner override:

```sh
export CARGO_TARGET_RISCV32IMC_UNKNOWN_NONE_ELF_RUNNER="$(cd ../../../probe-rs && pwd)/target/release/probe-rs run --chip CH32V467WEU6"
# Flashes, starts the firmware and decodes defmt RTT (not yet hardware-validated).
cargo run --release --bin blinky
# Logging-only test; also flashes.
cargo run --release --bin rtt
# CPU identity report; also flashes. No GPIO outputs.
cargo run --release --bin cpuid
# For an already programmed matching ELF; no flashing/reset.
../../../probe-rs/target/release/probe-rs attach --chip CH32V467WEU6 target/riscv32imc-unknown-none-elf/release/rtt
```

Read-only hardware checks on 2026-10-28 passed with the local release binary:
`info --verbose --protocol jtag` reports family `0xA6`, Chip ID `0x46740011`;
explicit-chip reads confirm that ID and USER at `0x1FFFF802` = `0xBF` (200 KiB
RAM, matching the default linker layout). Omitting `--chip` identifies WEU6
but the subsequent connection times out over USB; keep the explicit chip.
Flash programming, live RTT and PB12 behavior remain unverified. V4x7 is not
opted into WCH-Link's automatic unprotect-on-attach path.

SDI did not work on the user's board; an address/handshake mismatch is a
hypothesis, not a diagnosed cause. These examples no longer touch SDI registers.

The examples explicitly select **HSI 20 MHz, HCLK /1** and use a polled SysTick.
They do not need an external crystal, configure PLL/PSRAM, or enable peripheral
interrupts. Timing is nominal until measured on hardware, including log overhead.

## Next steps

1. Hardware-validate the generated V4x7 Flash algorithms; investigate the
   auto-detection reconnect timeout (use explicit `--chip` meanwhile).
2. Verify RTT, PB12 LED, option-byte RAM split and measured HSI frequency.
3. Port the V4x7 RCC/GPIO HAL path, then add USART and interrupt/DMA tests.

Building these examples only validates compilation and linking, not hardware
startup or full peripheral support.

## Local validation (2026-10-28)

- `cpuid` passes release build and Clippy `-D warnings` for WEU6, RET6 and VET6,
  plus rustfmt. Its CSR values, exception recovery and live RTT report have
  not been hardware-validated; no measured CPU identity is asserted here.
- After regenerating the PAC with V4x7 SysTick 64-bit/high-word access views,
  both RTT binaries build and link for RET6, VET6 and WEU6; all three release
  Clippy configurations pass with `-D warnings`. Rustfmt checks also pass.
  The earlier missing `cnth()` / `cmph()` build error is resolved without
  changing the example's split-word writes while SysTick is stopped.
- Both ELFs contain `.defmt` metadata and `_SEGGER_RTT` at `0x20000008`;
  heartbeat markers are in RAM and stack top remains `0x20032000`.
- `llvm-size` text/data/bss: `blinky` 2206/56/1040 bytes;
  `rtt` 2102/56/1040 bytes (including the default 1024-byte RTT buffer).
- The installed PATH tool now supports V467. After installation, target
  lookup and `info --verbose --protocol jtag` both passed.
- No on-board RTT testing or flashing has been performed by the agent.
