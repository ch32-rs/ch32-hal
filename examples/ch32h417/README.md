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
| **dual-core** | `v3f/src/bin/NAME.rs` + `v5f/src/bin/NAME.rs` | `cpuid`, `dualcore`, `hello`, `sdi_cpuid` |

In a dual-core example the V3F half brings the chip up (clocks, GPIO, SDI),
hands over to hart 1, and reports; the V5F half is deliberately small — it runs
from flash with no HAL initialisation of its own and writes its results to the
shared mailbox or drives a pin. A `v5f/src/bin/NAME.rs` without a `v3f` half is
rejected by `xtask`.

## Commands

```sh
cd examples/ch32h417

cargo xtask build --example cpuid      # both cores of a dual-core example
cargo xtask build --example i2c_scan   # a v3f-only example
cargo xtask build                      # everything

cargo xtask flash --example dualcore   # build + write both images, no console
cargo xtask run   --example dualcore   # same, plus the SDI console
cargo xtask flash --example launcher --v3f-only   # boot core alone

cargo xtask merge  --example dualcore  # out/dualcore.bin + out/dualcore.hex
cargo xtask report                     # decode the mailbox the examples publish
```

There are two things to select, matching the two kinds of example: **which
example**, and whether a dual-core example is written **whole** (both cores) or
only on the **boot core** (`--v3f-only`, which leaves the V5F region as it is).
There is deliberately no V5F-only mode — reset never starts hart 1, so it would
have nothing to run. `--no-build` skips the build step and `--out DIR` moves the
merged artifacts.

Flashing a dual-core example writes the V3F image first with `--no-run` and the
V5F image last, because the final write is the one that resets and runs the chip:
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
cargo xtask flash --example cpuid   # hand over, let hart 1 probe itself
cargo xtask report                  # read hart 1's block back out
```

`launcher` is the same handover with nothing else: a generic V3F image that brings
the shared blocks up, wakes hart 1 at the address `v5f/memory.x` links it to, and
parks the V3F in `wfi`. With `--v3f-only` it starts whatever V5F payload is
already in flash, including one built by hand:

```sh
cargo xtask flash --example launcher --v3f-only   # start the existing V5F image
```

It is deliberately not an embassy application — no executor, and `hal::init()`
only touches the blocks the V5F halves depend on.

## Flash / RAM partition

The same split as the WCH CSDK (`EXAM/…/Common/Ld/V3F/Link_v3f.ld` and
`Ld/V5F/Link_v5f.ld`):

| Region | Owner | Notes |
|---|---|---|
| `0x00000000` + 64K | V3F image | `v3f/memory.x` |
| `0x00010000` + 960K−64K | V5F image | `v5f/memory.x`, code copied to ITCM by the CSDK |
| `0x20178000` + 32K | shared RAM mailbox | declared by both `memory.x` |

Either image declares its own range, so an oversized build fails to link
instead of overwriting the other image; `xtask merge` checks the sizes again
while assembling the merged image.

### Mailbox map

```
0x20178000  sdi_cpuid: console token
0x20178004  sdi_cpuid: V5F tick counter
0x20178008  dualcore:  0xDEADBEEF liveness marker written by the V5F
0x2017800C  dualcore:  V3F loop counter
0x20178010… cpuid:     V5F CSR report block (see v3f/src/bin/cpuid.rs)
```

`cargo xtask report` decodes all of it — the console is not usable for this, as
the next note explains. The V3F half of `cpuid` clears the mailbox before the
handover, so the report always describes the current run rather than whatever an
earlier example left in shared RAM.

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
