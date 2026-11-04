# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

`ch32-hal` is a single-crate Embassy-style HAL for WCH's 32-bit RISC-V
microcontrollers (CH32V/F/X/L/H/CH641/CH643). One crate supports the whole
series through Cargo features that map onto `ch32-metapac` chip features.

Peripheral register access comes from `ch32-metapac`, generated from the
`ch32-rs/ch32-data` repo. Do not hand-edit generated PAC code — change the
ch32-data YAML and regenerate (see `ch32-data-gen-workflow` in local memory).

## Build, test, flash

Nightly Rust is required (`rustup default nightly`, plus `rust-src` for the
custom JSON targets that need `build-std`).

Build the library for one chip:

```sh
cargo build --features <chip>,embassy,rt,memory-x,defmt --no-default-features \
  --target riscv32imc-unknown-none-elf
```

Targets vary by core: most chips use `riscv32imc-unknown-none-elf`; `ch641` and
`ch32v006` use `riscv32i-unknown-none-elf`; the small `ec`/`emc`/`imfc` cores
and the H4 dual-core use per-example JSON targets under `examples/*`. The
reference matrix is `.github/workflows/build.yml`.

Build the on-target examples:

```sh
cd examples/<name> && cargo build --release
cd examples/ch32h417 && cargo xtask build   # dual-core, both V3F and V5F images
```

Run a host-side driver test (plain `std`, runs natively on macOS/Linux):

```sh
cargo test --manifest-path tools/flash-host-tests/Cargo.toml -- --test-threads=1
```

Flash / observe the H4 dual-core examples:

```sh
cd examples/ch32h417
cargo xtask flash --example cpuid --dual-core
cargo xtask report               # read the second core's mailbox afterwards
```

There is no root workspace or `cargo xtask`; `examples/ch32h417/xtask` is the
only xtask and is specific to the dual-core chip.

## Architecture

- **Metapac, not hand-written PAC.** `src/lib.rs` re-exports
  `ch32_metapac as pac`. Chip/family differences are expressed as Cargo
  features on `ch32-metapac`, and `build.rs` reads
  `ch32_metapac::metadata::METADATA` to emit per-chip/peripheral cfg flags
  (`rcc_v1`, `rcc_v4x7`, `afio_h4`, `peri_*`, …) and generated impls.
- **RCC dispatch.** `src/rcc/mod.rs` selects a family implementation by cfg:
  `rcc_v1`, `rcc_v003`, `rcc_v00x`, `rcc_v3`, `rcc_v3_d8c`, `rcc_v4x7`,
  `rcc_l1`, `rcc_x0`, `rcc_ch641`, `rcc_h4`. Each has its own `src/rcc/*.rs`
  (or `src/rcc/h4/`). `clocks()` is a cached global refreshed by
  `rcc::init`/`rcc::refresh`.
- **Pin traits are generated.** `build.rs` has a `(kind, signal) → trait`
  table and emits `pin_trait_impl!` / `mco_pin_impl!` from the metapac pin
  tables. Two pin mux shapes coexist, selected by `afio_h4`: legacy PCFR remap
  (every other family) vs H4 per-pin AFR.
- **Embassy.** Async drivers are behind the `embassy` feature; `ch32-hal` is
  the Embassy driver crate for these chips. Time driver is per-family
  (SysTick on most, explicit `time-driver-timN` on H4).
- **Dual-core H4.** `examples/ch32h417` is special: two crates (`v3f`, `v5f`)
  built separately, sharing one `Mailbox<T>` in `SRAM_SHARED`. The V5F half is
  metapac-only — never `ch32-hal`/`embassy` — because both cores share one RCC
  and the HAL `init()` would fight over it. See `AGENTS.md` (local-only) for
  the SDI-print, I-cache, and observation discipline details.

## Style

Committed files (`src/`, `examples/`, `tools/`, `README.md`) are **English**;
Chinese appears only in `local-docs/` and local-only memory/`AGENTS.md`.

- Comments: sentence case, trailing period; backticks around registers,
  symbols, APIs (`STATR.TXE`, `SPE=0`, `wait_write_buffer()`).
- `//!` for module heads, `///` for public API, `//` for in-body explanation.
- RM references: `RM §3.3.5.5`, `RM §3.4.6–3.4.9`, `RM table N` — no `RM 32.5.6`
  or `RM chapter 25`.
- Wrap comments/docs at ~80 columns; explain *why*, not what the code does.
- Commit: Conventional Commits `type(scope): lower-case summary`
  (`feat`/`fix`/`docs`/`test`/`chore`), scope like `ch32v4`, `spi`, `flash`,
  body wrapped at ~72 columns.
- Chinese local-docs: full-width punctuation, space between CJK and Latin/digits,
  `I²C` in prose (ASCII `I2C_EV` for identifiers). Headings sentence case; dates
  in parens `(2026-11-04)`.

## Testing strategy

Current state and the direction it should move in:

- **CI only compiles** (`.github/workflows/build.yml` runs `cargo build` for a
  chip matrix and for the examples). There is no `cargo test`, no host-test job,
  no HIL.
- The dead `src/` `#[cfg(test)]` modules and `*_tests.rs` files were removed
  (2026-11-04): they needed a real PAC and CI never ran them.
- Only two host harnesses remain: `tools/flash-host-tests` and
  `tools/spi-host-tests`. They compile the real driver source against a
  hand-written mock register model to pin down the two boundary bugs fixed on
  2026-11-03 (flash `WRBSY` stall, SPI `TXE` while `SPE=0`). Shared scaffolding
  is `tools/host-tests-common/{common_word.rs,build_helpers.rs}`; each harness
  exposes its model through a `record_read`/`record_write` seam and appends its
  `lifecycle` tests in `build.rs`. The five protocol harnesses
  (i2c/sdio/usbhs/i3c/eth) were deleted as low-ROI: their models were large and
  duplicated the on-board `examples/ch32v467` coverage.
- Examples share the `ch32v467_examples::report_*!` macros instead of
  per-binary marker statics.

Direction (updated 2026-11-04):

1. Add a host `cargo test -- --test-threads=1` CI job for the two harnesses.
2. Eventually move to HIL (embassy's `tests/stm32` + `teleprobe` style); deferred
   — only a single dev board is available.

## Dev-time notes

- **Local `ch32-data` path is fine for now.** The host harnesses read the PAC
  from `../../../../ch32-data/build/ch32-metapac/…`, a cross-repo relative path.
  This is an accepted artifact of being mid-development; do not spend effort
  removing it yet.
- **Do not run `cargo fmt` across the whole crate.** The committed tree is not
  fully `rustfmt`-clean; a broad `cargo fmt` reformats unrelated files. Format
  only the files you touched (`rustfmt.toml` sets `max_width = 120`).
- `AGENTS.md` and `local-docs/` are local-only and excluded via
  `.git/info/exclude` (never `.gitignore`), so the exclusion itself stays out
  of the repo. They hold hardware observations and deferred work.
- **Full-series rule.** Before designing any shared API, scan
  `ch32-metapac`/`ch32-data` across all RCC/peripheral versions first; never
  assume the current chip is the general case.
