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
| **v3f-only** | `v3f/src/bin/NAME.rs` | `blinky`, `i2c_scan`, `bme280_blocking`, `sdi_print`, `launcher`, `rcc_delay_check` |
| **dual-core** | both `src/bin/NAME.rs` | `atomics`, `bme280`, `cpuid`, `dualcore`, `hello`, `pingpong`, `sdi_cpuid` |

In a dual-core example the V3F half brings the chip up (clocks, GPIO, SDI),
hands over to hart 1, and reports. The V5F half is written against **metapac
only** — no `ch32-hal`, no embassy: `Peripherals::take()` is a single-use
singleton shared by both cores, and `rcc::init()` re-programs the PLL and then
blocks on `CFGR0.SWS == PLL`, which would switch SYSCLK out from under the core
that is running. `v5f/` depends on `ch32-metapac` directly (`ch32h417_v5f::pac`)
and prints through `v5f/src/sdi.rs`. A V5F half that wants HAL drivers needs the
secondary-core entry point tracked in `docs/backlog.md`.

## Commands

```sh
cd examples/ch32h417

cargo xtask build  --example cpuid                 # boot core only (the default)
cargo xtask build  --example cpuid --dual-core     # both of its halves
cargo xtask build  --example cpuid --jump-v5f      # jumper + cpuid's V5F half
cargo xtask build                                  # everything

cargo xtask flash  --example dualcore --dual-core  # build + write both, no console
cargo xtask run    --example dualcore --dual-core  # same, plus the SDI console
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

**Do not watch a dual-core example through the console.** Any `wlink` request
that inspects the chip halts both cores, and `run` (`--watch-serial`) stays
attached for as long as the console runs, so hart 1 makes no progress meanwhile.
The examples publish into the shared mailbox instead:

```sh
cargo xtask flash --example cpuid --dual-core
cargo xtask report --example cpuid
```

Leave a gap between reads: a burst of `wlink` requests keeps both harts halted,
so counters look frozen. `run` is fine for a v3f-only example, or for seeing the
boot core's own lines.

## The examples

- **`blinky`**, **`sdi_print`** — v3f-only. `sdi_print` is the SDI console demo:
  `cargo xtask run --example sdi_print`.
- **`i2c_scan`**, **`bme280_blocking`** — v3f-only, I2C1 on PB6 = SCL, PB7 = SDA
  (AF4). The boot core reads the sensor itself.
- **`bme280`** — the same sensor read by hart 1 instead. The V5F half brings up
  I2C1 through metapac (no HAL, so it does not touch the clock tree) and stores
  each sample in the mailbox; the boot core only prints it. `report` prints the
  sensor state (`ok` once the chip answered) and the last temperature, pressure
  and humidity.
- **`rcc_delay_check`** — v3f-only regression check for the default 25 MHz HSE at
  the 400 MHz preset: cached V3F/V5F frequencies, RCC refresh, and SysTick
  compare values for rounded-up nanosecond delays, zero delay, and exact
  microsecond/millisecond delays. It does not wake hart 1 or print over SDI.
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
- **`sdi_cpuid`** — `cpuid`, but the boot core prints hart 1's CSR block over SDI.
  Flash with no console first (`cargo xtask flash --example sdi_cpuid
  --dual-core`), then attach without reset (`wlink sdi-print enable`) and read
  the link's virtual serial port (`/dev/cu.usbmodem*`). `run` attaches at boot
  and shows only the boot core.
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
| `0x00000000` + 64K | V3F image | `v3f/memory.x` |
| `0x00010000` + 960K−64K | V5F image | `v5f/memory.x` |
| `0x20178000` + 32K | shared region (`ipc::Mailbox`) | declared by both `memory.x`; defined by the V3F image only |

Either image declares its own range, so an oversized build fails to link
instead of overwriting the other image; `xtask merge` checks the sizes again.
The V5F currently executes from flash in place — the CSDK's copy of V5F `.text`
to ITCM is not implemented here, so keep V5F halves small.

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
- **Hart 1's instruction cache is off out of reset** (`cache_strtg_ctlr`, CSR
  `0xBC2`, `ic_disable = 1`), so every V5F half starts with
  `ch32h417_v5f::cache::enable_icache()`. Hart 0 has no usable instruction cache;
  its speed path is running code from RAM (see `docs/backlog.md`).
