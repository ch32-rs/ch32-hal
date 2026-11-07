# CH32H417 examples

CH32H417 is one chip with **two RISC-V cores**, so these examples are organised
per chip rather than per core:

```
ch32h417/
├── v3f/     control core, hart 0 — riscv32imafc, flash 0x00000000…0x0000FFFF
├── v5f/     second core,  hart 1 — riscv32imafbc, flash 0x00010000…
├── ipc/     the shared mailbox, consumed by both crates
├── xtask/   build / flash / run / merge / report driver
└── out/     merged images (generated)
```

`v3f/` and `v5f/` are separate crates: different ISA (`imafc` vs `imafbc`),
different ABI, different `memory.x`. They share one flash chip and one mailbox
in shared RAM, which `xtask` handles.

## Two kinds of example

Reset only starts the **V3F**. The V5F stays put until the V3F writes its wake
register (`PFIC_WAKEIP1`), so examples come in exactly two shapes, which
`xtask` derives from the files that exist. A `v5f/src/bin/NAME.rs` without a
`v3f` half is rejected.

| Kind | Files | Examples |
|---|---|---|
| **v3f-only** | `v3f/src/bin/NAME.rs` | `adc_check`, `blinky`, `bme280_blocking`, `bus_check`, `can_async_check`, `can_check`, `dac_check`, `delay_systick`, `dma2_check`, `dma_m2m`, `embassy_systick`, `esig_check`, `exti_check`, `flash_check`, `i2c_scan`, `launcher`, `rcc_delay_check`, `rng_check`, `rtc_check`, `rtc_hse_check`, `sdi_print`, `sdio_check`, `sdio_exfat`, `sdio_fat`, `spi_check`, `timer_check` |
| **dual-core** | both `src/bin/NAME.rs` | `atomics`, `bme280`, `cpuid`, `dualcore`, `hello`, `pingpong`, `sdi_cpuid` |

In a dual-core example the V3F half brings the chip up (clocks, GPIO),
hands over to hart 1, and reports. The V5F half depends on `ch32-hal` but never
calls `ch32_hal::init`: that re-programs the PLL and then blocks on
`CFGR0.SWS == PLL`, which would switch SYSCLK out from under the core that is
running. It calls `ch32_hal::init_secondary` instead, which only reads the clock
tree the boot core already programmed back into `rcc::clocks()`.
`ch32h417_v5f::pac` re-exports the HAL's metapac. Each image has its own
non-blocking defmt RTT buffer. The two cores still must not both use one peripheral: each
image has its own `Peripherals` singleton, so neither can tell that the other
already took it.

## Commands

```sh
cd examples/ch32h417

cargo xtask build  --example cpuid                 # boot core only (the default)
cargo xtask build  --example cpuid --dual-core     # both of its halves
cargo xtask build  --example cpuid --jump-v5f      # jumper + cpuid's V5F half
cargo xtask build                                  # everything

cargo xtask flash  --example dualcore --dual-core  # build + write both, no console
cargo xtask run    --example dualcore --dual-core  # same, plus the V3F RTT console
cargo xtask flash  --example cpuid --jump-v5f      # jump firmware + V5F payload
cargo xtask flash  --jump-v5f                      # reflash the jumper alone

cargo xtask merge  --example dualcore --dual-core  # out/dualcore.bin + .hex
cargo xtask report --example dualcore              # decode that example's mailbox
```

| Layout | Boot core (`v3f`) | Second core (`v5f`) |
|---|---|---|
| *(default)* | the named example's half | left exactly as it is |
| `--dual-core` | the named example's half | the named example's half |
| `--jump-v5f` | the generic jumper, `launcher` | the named example's half |

There is no V5F-only mode: reset only starts the boot core, so a V5F payload
would have nothing to run. `--no-build` skips the build step, `--out DIR` moves
the merged artifacts. Merging needs both images, so it asks for `--dual-core`
or `--jump-v5f`. Flashing writes the boot core first with `--no-run` and the
last image with a plain write, because the final write is the one that resets
and runs the chip.

## Reading results back

Any `wlink` inspection halts both cores. For dual-core execution results the
examples publish into the shared mailbox:

```sh
cargo xtask flash --example cpuid --dual-core
cargo xtask report --example cpuid
```

Leave a gap between reads: a burst of `wlink` requests keeps both harts halted,
so counters look frozen. `run` uses probe-rs and shows V3F RTT output. For multiple
probes, set `PROBE_RS_PROBE=VID:PID:SERIAL`. Dual-core RTT attach has not been
validated for continuous V5F execution; use the mailbox to verify hart 1.

## The examples

- **`blinky`**, **`sdi_print`** — v3f-only. `sdi_print` is the RTT console demo (legacy name):
  `cargo xtask run --example sdi_print`.
- **`i2c_scan`**, **`bme280_blocking`** — v3f-only, I2C1 on PB6 = SCL, PB7 = SDA
  (AF4). The boot core reads the sensor itself.
- **`adc_check`**, **`bus_check`**, **`can_async_check`**, **`can_check`**,
  **`dac_check`**, **`dma2_check`**, **`esig_check`**, **`exti_check`**,
  **`flash_check`**, **`rng_check`**, **`rtc_check`**, **`rtc_hse_check`**,
  **`spi_check`**, **`sdio_check`**, **`timer_check`** — v3f-only self-checks
  for the H4-only drivers. Each
  publishes a `stage` and a `failures` count into the mailbox (plus
  driver-specific fields); read them with
  `cargo xtask report --example NAME`. `can_check` runs CAN1 on PA11/PA12 and
  CAN2 on PB12/PB13 in silent loopback, so no transceiver is needed;
  `can_async_check` repeats it through the async driver and needs the
  `embassy` feature (xtask builds it with that feature). `bus_check` covers
  the USART/SPI/I2C instances that have no other check. `rtc_check` runs the
  RTC from LSI and then LSE, exercising the second/alarm interrupts and the
  retargeting guard; `rtc_hse_check` boots the HSE recipe and uses HSE/512.
- **`dma_m2m`**, **`delay_systick`**, **`embassy_systick`** — bring-up checks
  for the DMA/DMAMUX path, the blocking SysTick delay and the Embassy time
  driver.
- **`bme280`** — the same sensor read by hart 1 instead. The V5F half calls
  `ch32_hal::init_secondary` (which reads the clock tree back rather than
  touching it) and brings up I2C1 itself, then stores each sample in the
  mailbox; the boot core only prints it. `report` prints the
  sensor state (`ok` once the chip answered) and the last temperature, pressure
  and humidity.
- **`rcc_delay_check`** — v3f-only regression check for the default 25 MHz HSE at
  the 400 MHz preset: cached V3F/V5F frequencies, RCC refresh, and SysTick
  compare values for rounded-up nanosecond delays, zero delay, and exact
  microsecond/millisecond delays. It does not wake hart 1 or print.
  `report` reads its own payload: `stage` (1 = init entered, 2 = init returned,
  3 = checks passed), `failures` (**0** means success), `v3f_hz` / `v5f_hz`
  (**100000000 / 400000000**) and `heartbeat` (one-second counter). Read once per
  fresh flash.
- **`cpuid`** — the handover example. The V3F half runs `hal::init()` (RCC, flash
  latency, AFIO/GPIO, EXTI), writes hart 1's entry to the PFIC wake register and
  parks in `wfi`; the V5F half probes its own Machine-mode CSRs into the mailbox.
  `mhartid = 1` is what shows the values came from the second core.
- **`launcher`** — the same handover with nothing else, used by `--jump-v5f`: it
  brings the shared blocks up, wakes hart 1 at the address `v5f/memory.x` links
  it to, and parks the V3F. Not an embassy application — no executor, and
  `hal::init()` only touches the blocks the V5F halves depend on.
- **`sdi_cpuid`** — the legacy-named CPU-ID example. Each core logs to its own
  RTT control block, and V3F also logs the V5F values published in the mailbox.
  `cargo xtask run` attaches to the V3F ELF; use the V5F ELF for its separate
  RTT stream. Both loggers use non-blocking mode.
- **`atomics`** — both cores `fetch_add` the same shared word `CAS_INCREMENTS`
  (1,000,000) times each, so the total must be exactly 2,000,000. Anything less
  means updates were lost. `report` prints the total, the done mask and the
  verdict. Both target JSONs set `"atomic-cas": true` and both crates enable
  `qingke/unsafe-trust-wch-atomics`; a critical section that masks one hart's
  interrupts cannot serialise two cores.
- **`pingpong`** — hart 1 probes its CPU-ID CSRs into the mailbox, the boot core
  prints what hart 1 reported, and from then on the two exchange one round per
  second: hart 1 increments `ping`, the boot core stores `pong`, and hart 1
  waits for the answer before the next round. Read the counters with `report`
  after a `flash`.
- **`dualcore`**, **`hello`** — the minimal handover and the first shared-memory
  exchange.

## Flash / RAM partition

The same split as the WCH CSDK (`Ld/V3F/Link_v3f.ld`, `Ld/V5F/Link_v5f.ld`):

| Region | Owner | Notes |
|---|---|---|
| `0x00000000` + 64K | V3F load image (flash) | `v3f/memory.x`; copied to RAM at startup |
| `0x20100000` + 448K | V3F code, data, bss | zero-wait shared SRAM |
| `0x20170000` + 32K | V3F stack | zero-wait shared SRAM |
| `0x00010000` + 960K−64K | V5F image (flash) | `v5f/memory.x` |
| `0x20178000` + 32K | shared region (`ipc::Mailbox`) | declared by both `memory.x`; defined by the V3F image only |

Either image declares its own range, so an oversized build fails to link
instead of overwriting the other image; `xtask merge` checks the sizes again.

The boot core runs entirely from the shared block. Hart 0 has no usable
instruction cache, Code FLASH is non-zero-wait (RM §1.2.1), and hart 1's ITCM
costs hart 0 two HCLK waits — while the 512 KB shared block is zero-wait for
both cores. `v3f/link.x` folds all of `.text`/`.rodata` into `.highcode` (the
`qingke-rt` `highcode` feature), so only `.init` and `.handle_reset` stay in
flash and the startup copies the rest into RAM. Measured with `eth_bench`: a
1M-iteration integer loop drops from 1.12 s to 87 ms, and TCP throughput from
5.0 to 93 Mbit/s up, 92 Mbit/s down. The 64 KB flash slot is now a load-image
limit, not an execution one.

The V5F executes from flash in place and has an instruction cache, so it does
not need the copy; the CSDK's ITCM copy for hart 1 is not implemented here, so
keep V5F halves small.

### Mailbox

One shared region, `SRAM_SHARED` (`0x20178000` + 32K), holding one shared
object: `ch32h417_ipc::Mailbox<T>`, a layout-version word plus the example's own
`#[repr(C)]` payload. The payload types live in `ipc/src/protocols.rs` — one per
example, so an example no longer reserves words for protocols it never runs, and
two examples no longer share a field. The boot core **defines** the object; the
second core never places one:

```
v3f image   .shared (NOLOAD) -> MAILBOX @ 0x20178000   (defined here, and only here)
v5f image   no .shared, no MAILBOX symbol -> ipc::at(MAILBOX_ADDR)
```

The address is not negotiated between the images. `shared.x` puts `.shared` at
the start of `SRAM_SHARED`, both `memory.x` files pin that region at
`0x20178000`, and `ch32h417_ipc::MAILBOX_ADDR` names the same place, so moving
the region means changing all three. `cargo xtask report --example NAME` decodes
whichever protocol that example stores there; the read length is
`size_of::<Mailbox<ThatProtocol>>()`. Because the region is `NOLOAD` it costs no
flash and nothing zeroes it at boot, so it survives one core resetting while the
other runs. The boot core clears it before handing over, so `report` describes
the current run. The version word versions the container only, so two
independently flashed halves can notice they disagree about it.

## Notes

- **Privilege.** `qingke-rt` ≥ 0.8.2 leaves `main` in Machine mode, which is what
  lets the V3F read `misa`, `mhartid` and friends directly. The `u-mode` feature
  restores WCH's startup behaviour (User mode), where only the `URW` CSRs
  `gintenr`/`intsyscr` remain reachable.
- **Flash writes are not always complete.** `wlink flash` can report success
  while only part of the image landed, so `xtask` reads every `--no-run` image
  back and repeats the write if it did not. The final write resets and runs the
  chip, so it cannot be checked the same way — if a dual-core example only shows
  V3F activity, flash it again before suspecting the firmware.
- **Merged images.** `out/NAME.bin` is the full flash image (`0xFF` padding
  between the two payloads); `out/NAME.hex` is the same image as records, which
  only cover the payloads. Use the merged image to publish and the two-ELF
  `xtask flash` path to iterate.
- **`xwchc`.** Both target specs (`v3f/riscv32imafc-unknown-none-elf.json`,
  `v5f/riscv32imafbc-unknown-none-elf.json`) enable it: WCH's compressed byte and
  halfword loads and stores, which QingKe V3F and V5F both implement (QingKe V3
  manual, table 1-1). It only changes codegen, so the HAL's CI builds against the
  stock targets and leaves it to the examples.
- **Hart 1's instruction cache is off out of reset** (`cache_strtg_ctlr`, CSR
  `0xBC2`, `ic_disable = 1`), so every V5F half starts with
  `ch32h417_v5f::cache::enable_icache()`. Hart 0 has no usable instruction cache,
  so the V3F image runs from the shared SRAM (`v3f/link.x`, `highcode`).

## SD card file-system examples

`sdio_fat` uses the first MBR partition when it is FAT16/FAT32; `sdio_exfat`
uses the first MBR partition when it is exFAT. Both use PB11/PB10 and PE8–PE11
with DMA1 channel 1. They request 2.5 MHz: higher SDIO clock settings on the
current H417 board and card are not yet reliable. These are blocking examples.

Each run creates a new file named `H4TST00.TXT` through `H4TST99.TXT`, writes
1300 bytes, remounts, and verifies the contents. Existing filenames are skipped;
the test file is left on the card. Do not run these examples against a mounted
card shared with another host.

```text
cargo xtask flash --example sdio_exfat
cargo xtask report --example sdio_exfat
```

For either example, `stage = 7`, `error = 0`, and `bytes verified = 1300`
indicate success; `test file number = 1` means `H4TST01.TXT`. A nonzero error
or an earlier stage means the file-system operation did not complete. The
FAT16/FAT32 example requires a FAT16/FAT32 partition; the exFAT example does
not format the card.
