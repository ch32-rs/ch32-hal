# CH32H417 examples

CH32H417 is one chip with **two RISC-V cores**, so these examples are organised
per chip rather than per core:

```
ch32h417/
├── v3f/     control core, hart 0 — riscv32imafc, flash 0x00000000…0x0000FFFF
├── v5f/     second core,  hart 1 — riscv32imafbc, flash 0x00010000…
├── xtask/   build / flash / run / merge driver
└── out/     merged images (generated)
```

`v3f/` and `v5f/` are separate crates because the two cores cannot share a
Cargo target: different ISA (`imafc` vs `imafbc`), different ABI, and different
`memory.x`. What they do share is one flash chip and one mailbox in shared RAM,
which is what `xtask` handles.

## Two kinds of example

Reset only starts the **V3F**. The V5F stays put until the V3F writes its wake
register (`PFIC_WAKEIP1`), so a V5F-only image has no entry point at all.
Examples therefore come in exactly two shapes, which `xtask` derives from the
files that exist:

| Kind | Files | Examples |
|---|---|---|
| **v3f-only** | `v3f/src/bin/NAME.rs` | `blinky`, `i2c_scan`, `bme280_blocking`, `sdi_print`, `launcher` |
| **dual-core** | `v3f/src/bin/NAME.rs` + `v5f/src/bin/NAME.rs` | `atomics`, `cpuid`, `dualcore`, `hello`, `pingpong`, `sdi_cpuid` |

In a dual-core example the V3F half brings the chip up (clocks, GPIO, SDI),
hands over to hart 1, and reports; the V5F half is deliberately small — it runs
from flash with no HAL initialisation of its own and writes its results to the
shared mailbox or drives a pin. A `v5f/src/bin/NAME.rs` without a `v3f` half is
rejected by `xtask`.

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
cargo xtask report                     # decode the mailbox the examples publish
```

Two things are selected: **which example**, and **what to program**:

| Layout | Boot core (`v3f`) | Second core (`v5f`) |
|---|---|---|
| *(default)* | the named example's half | left exactly as it is |
| `--dual-core` | the named example's half | the named example's half |
| `--jump-v5f` | the generic jumper, `launcher` | the named example's half |

The default is the boot core alone because reset only ever starts that core: a
V5F payload is written when you ask for it, and there is deliberately no V5F-only
mode — it would have nothing to run. `--no-build` skips the build step and
`--out DIR` moves the merged artifacts. Merging needs both images, so it asks for
`--dual-core` or `--jump-v5f`.

Flashing writes the boot core first with `--no-run` and the last image with a
plain write, because the final write is the one that resets and runs the chip:
only then are both images in place.

### Reading results back

A console cannot be used to watch a dual-core example: every `wlink` request
pauses the cores while it is attached, and `--watch-serial` stays attached for as
long as the console runs, so the V5F half makes no progress meanwhile. The
examples therefore publish their results into the shared mailbox, and `report`
reads that back:

```sh
cargo xtask flash --example cpuid
cargo xtask report
```

### Handing over to hart 1

`cpuid` is the handover example: the V3F half runs the whole bring-up —
`hal::init()` programmes the clock tree (RCC, flash latency, AFIO/GPIO, EXTI) —
then writes hart 1's entry to the PFIC wake register and parks in `wfi`. The V5F
half probes **its own** Machine-mode CSRs and stores them; `report` prints them,
and `mhartid = 1` is what shows the values came from the second core rather than
from the boot core's probe.

```sh
cargo xtask flash --example cpuid --dual-core   # hand over, let hart 1 probe itself
cargo xtask report                              # read hart 1's block back out
```

`launcher` is the same handover with nothing else: a generic V3F-only image that
brings the shared blocks up, wakes hart 1 at the address `v5f/memory.x` links it
to, and parks the V3F in `wfi`. `--jump-v5f` puts that jumper on the boot core
while programming a V5F payload next to it, so a V5F half needs no V3F
counterpart of its own:

```sh
cargo xtask flash --example cpuid --jump-v5f   # jumper + cpuid's V5F half
cargo xtask flash --jump-v5f                   # reflash the jumper alone
```

It is deliberately not an embassy application — no executor, and `hal::init()`
only touches the blocks the V5F halves depend on.

### Writing a V5F half

Hart 1 is written against **metapac only** — no `ch32-hal`, no embassy:

- the boot core owns every global block (there is one RCC register file for both
  harts) and `ch32-hal`'s `init()` assumes exactly that: `Peripherals::take()` is
  a single-use singleton shared by both cores, and `rcc::init()` re-programs the
  PLL and then blocks on `CFGR0.SWS == PLL`, which would switch SYSCLK out from
  under the core that is running;
- so `v5f/` depends on `ch32-metapac` directly, re-exported as
  `ch32h417_v5f::pac`, and reaches printf through `v5f/src/sdi.rs`
  (`sdi_println!`, plus a lossy `sdi_try_println!` that cannot stall hart 1);
- `qingke-rt` still supplies the entry point, the stack and the vector table.

`v5f/build.rs` ships this crate's `memory.x` **and** the `device.x` hook — qingke-rt's
`link.x` includes both, and a crate with no HAL and no svd2rust device crate has
to provide the latter itself.

A V5F half that wants actual HAL *drivers* (GPIO, SPI, …) would need the
secondary-core entry point tracked in `docs/backlog.md`; today's examples drive
registers directly instead.

## Flash / RAM partition

The same split as the WCH CSDK (`EXAM/…/Common/Ld/V3F/Link_v3f.ld` and
`Ld/V5F/Link_v5f.ld`):

| Region | Owner | Notes |
|---|---|---|
| `0x00000000` + 64K | V3F image | `v3f/memory.x` |
| `0x00010000` + 960K−64K | V5F image | `v5f/memory.x`, code copied to ITCM by the CSDK |
| `0x20178000` + 32K | shared region (`ipc::Mailbox`) | declared by both `memory.x`; defined by the V3F image only |

Either image declares its own range, so an oversized build fails to link
instead of overwriting the other image; `xtask merge` checks the sizes again
while assembling the merged image.

### Cross-core atomics (`atomics`)

Both cores `fetch_add` the *same* shared word, concurrently, `CAS_INCREMENTS`
(1,000,000) times each — so the total must be exactly 2,000,000. Anything less
means updates were lost, i.e. the atomics were emulated rather than performed by
the A extension. That matters because the only fallback available here is a
critical section that masks *one* hart's interrupts (`qingke`'s
`SingleHartCriticalSection`), which cannot serialise two cores.

```text
cargo xtask flash --example atomics --dual-core   # both cores hammer, then stop
cargo xtask report                                 # total, done mask, verdict
cargo xtask run   --example atomics --dual-core    # prints the verdict over SDI
```

Both target JSONs therefore set `"atomic-cas": true` and both crates enable
`qingke/unsafe-trust-wch-atomics` — the feature qingke gates behind a warning
that its atomics were "most likely broken" *as tested on QingKe V4*. On this
H417 the litmus above passes (2,000,000 with no lost updates), which is what
makes the feature warranted rather than hopeful. Note the exchange takes several
seconds: hart 1 runs this loop from flash and the test's `delay`-free pacing is
whatever the cores' clocks give.

### Shared-memory ping-pong (`pingpong`)

The end-to-end demo of the mechanism above: the boot core clears the mailbox and
hands over, hart 1 probes its own CPU-ID CSRs into it, the boot core prints what
hart 1 reported — the only thing crossing between the cores is shared memory —
and from then on the two exchange one round per second. Hart 1 increments
`ping`; the boot core answers by storing `pong`, and hart 1 only starts the next
round once it has seen the answer, so the exchange cannot run away from the
printer.

```text
cargo xtask flash --example pingpong --dual-core   # rounds run (~0.7/s, paced by delay_ms(1000))
cargo xtask report                                  # ping/pong, plus hart 1's CPU-ID block
cargo xtask run   --example pingpong --dual-core    # console: see hart 1's report printed
```

The two views are mutually exclusive, for the reason in the note below: with the
console attached hart 1 is held, so the counters freeze — and the boot core's
half prints through `try_println!` (the bounded variant) precisely so a missing
console cannot stall the rounds. Read the counters back after a `flash`, leaving
a gap: *any* `wlink` request halts both harts, so a burst of reads keeps them
halted and the counters look frozen.

### Hart 1's instruction cache

The datasheet's "core 1" memory block — 32 KB instruction cache, 128K ITCM,
256K DTCM — is hart 1, the same resources the CSDK's `Link_v5f.ld` builds its
code and data into. That cache is **disabled out of reset**
(`cache_strtg_ctlr`/`cstrcr`, CSR `0xBC2`, `ic_disable = 1`), so a
flash-resident V5F image fetches every instruction from flash until software
turns it on. Every V5F half here therefore starts with
`ch32h417_v5f::cache::enable_icache()`.

Measured on a CH32H417 with the `atomics` workload (2 × 1,000,000 contended
increments): 798,047 reached in the first four seconds with the cache off, and
the whole run finished inside that same window with it on. The CSR reads
`0x0f000003` before and `0x0f000001` after (bit 1 cleared; the `[27:24]` region
permits were already set).

Hart 0 has no usable cache: its copy of the register reads `0x00000003` and
ignores writes (QingKe's instruction cache is the V4J variant). The boot core's
speed path is running code from RAM instead — which is exactly what the CSDK's
`RAM_CODE` in the 512K shared block does — not caching.

### Mailbox and shared region

There is exactly **one** shared region — `SRAM_SHARED` (`0x20178000` + 32K), the
tail of the 512K block the chip shares between the two harts, declared with the
same address in both `memory.x`. Inside it lives exactly **one** shared object,
the `ch32h417-ipc` crate's `Mailbox` (`#[repr(C)]`, fields documented there):
console token, the two liveness counters, and the V5F's CSR report block.

The boot core **defines** it; the second core **consumes its address**:

```
v3f image   .shared (NOLOAD) -> ipc::MAILBOX @ 0x20178000   (defined here, and only here)
xtask       post-processes the V3F ELF (llvm-nm -S) -> out/layout.txt
v5f image   no .shared at all -> ipc::at(MAILBOX_ADDR) from the generated layout
```

Because the region is `NOLOAD` it costs no flash, nothing zeroes it at boot (so
it survives one core resetting while the other runs), and the boot core clears it
before handing over — `cargo xtask report` then describes the current run rather
than whatever an earlier example left behind. The layout file also carries a
version word, so two independently flashed halves can notice they disagree.

## Notes

- **Privilege.** `qingke-rt` ≥ 0.8.2 leaves `main` in Machine mode, which is
  what lets the V3F read `misa`, `mhartid` and friends directly. The `u-mode`
  feature restores WCH's startup behaviour (User mode), where only the `URW`
  CSRs `gintenr`/`intsyscr` remain reachable.
- **Reading the target.** Every `wlink` request that inspects the chip (`dump`,
  `regs`, `status`) pauses the cores while it is attached, and the SDI console
  (`flash --enable-sdi-print --watch-serial`) keeps wlink attached for as long
  as it runs. So a dual-core example's V5F half only makes progress while
  nothing is observing the chip: use `cargo xtask flash` (plain write, then
  reset and run) and read the mailbox afterwards — `cargo xtask run` opens the
  console and therefore shows the V3F half only.
- **Flash writes are not always complete.** `wlink flash` has been seen to
  report success while only part of the image reached flash, and to hang on a
  write for minutes. `xtask` therefore reads every `--no-run` image back and
  repeats the write if it did not land:

  ```text
  v3f: read-back failed on attempt 1
  retrying ...
  v3f verified (7036 bytes)
  ```

  Repeating the write is what fixes it — a request that follows a killed one
  hangs once and then goes through. If two attempts both fail, `xtask` also
  resets the chip and re-cycles the link's protocol mode (`mode-switch --dap`,
  `--rv`) before the last attempt, which is cheap but only anecdotally
  effective. The final write is the one that resets and runs the chip, so it
  cannot be checked the same way without pausing it — if a dual-core example
  only shows V3F activity, flash it again before suspecting the firmware.
- **Merged images.** `out/NAME.bin` is the full flash image (`0xFF` padding
  between the two payloads); `out/NAME.hex` is the same image as records, which
  only cover the payloads. `wlink` accepts either and writes the whole address
  range; both program in a few seconds on a healthy link, so use the merged
  image to publish and the two-ELF `xtask flash` path to iterate.
- **V5F code placement.** The V5F currently executes from flash in place; the
  CSDK copies V5F `.text` to ITCM before running. That model is not implemented
  here yet, so keep V5F halves small (CSR reads and mailbox stores, not
  `core::fmt`).
