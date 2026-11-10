# CH32H417 as a homogeneous dual core — feasibility spike

Status: **spike done and API v1 implemented (`ch32_hal::multicore`), verified on
the bench board. Remaining gaps: hart-1 interrupts/time, cross-core
`critical-section`, checked peripheral ownership.**
Worktree branch: `dual-core-homogeneous`. Examples:
`examples/ch32h417/v3f/src/bin/homogeneous.rs` (mechanism) and
`.../multicore.rs` (API).

Question: can CH32H417 run one image on both harts, the way
`embassy_rp::multicore::spawn_core1` runs RP2040 core 1 — same `.text`, same
`.data`/`.bss`, second hart with its own stack and entry — instead of building
two images (V3F at flash `0x0000_0000`, V5F at `0x0001_0000`)?

Short answer: **yes for the mechanism, no for the API yet.** The second hart
boots into a 1 KiB-aligned trampoline inside the boot image, sets `gp`/`sp`,
and runs the same Rust code and HAL drivers. What is missing is the library
layer: a per-hart `Peripherals` split, a cross-core `critical-section`, a
per-hart time driver, and an audit of shared peripheral registers.

## What the spike does

`v3f/src/bin/homogeneous.rs`, built as the **only** image (`cargo xtask flash
--example homogeneous`, default `--v3f-only` layout):

1. Hart 0 runs `_start` as usual: copies `.highcode` (all of `.text`/`.rodata`)
   into the zero-wait shared block at `0x2010_0000`, clears `.bss`, takes
   `Peripherals`, runs `hal::init`.
2. Hart 0 clears an observation block in `.shared` (fixed at `0x2017_8000`),
   stores the wake address, does a `fence`, and calls
   `qingke::pfic::wake_other_core(entry)`.
3. The wake address is `_hart1_trampoline`, placed by `v3f/link.x` inside
   `.highcode` with `. = ALIGN(1024)`. `PFIC_WAKEIP[1]` keeps only bits
   [31:10], so the entry **must** be 1 KiB aligned; the trampoline ends up at
   `0x2010_0800` in this build.
4. The trampoline sets `gp` to the shared `__global_pointer$`, sets `sp` to a
   static 8 KiB stack in `.bss`, enables `mstatus.FS`, installs a tiny trap
   stub, does `fence rw, rw`, and calls `hart1_main` — a normal Rust `fn` in
   the same image.
5. Hart 1 reads `mhartid`/`PFIC_SCTLR`, takes the singleton with
   `Peripherals::steal()` (hart 0 already owns it — `take()` would panic),
   drives PF0 through `hal::gpio::Output`, and paces itself with
   `hal::delay::Delay`.

The only linker change is the 1 KiB-aligned `.hart1_entry` slot in
`v3f/link.x`. No new dependency, no second image, no `init_secondary`.

## Measured on the bench board (WCH-LinkE `1a86:8010:D7708F065C7F`)

RCC recipe `with_sysclk_400m_v5f_400m_v3f_100m_hsi()`:

| Observation | Value |
|---|---|
| Wake entry | `0x20100800` (1 KiB aligned, shared SRAM, inside `.highcode`) |
| Hart 1 `mhartid` / `HartId::current()` | `1` / `C1` |
| Hart 1 `sp` | `0x20103150` (inside `.bss`, the static stack) |
| Hart 1 `mcause` | `0` (no trap over minutes of running) |
| `Delay.delay_ms(1000)` on hart 1 | `400000172` `mcycle` ticks = 1.000 s at 400 MHz |
| `h0_seq` / `h1_seq` over 10 s | +10 / +10 |
| `Peripherals::steal()` + `Output::new(PF0)` on hart 1 | works, LED1 blinks |
| Atomic `fetch_add` on a shared `.shared` word, both harts | works (matches the earlier 2×1 M litmus) |

Notes from this run:

- **`wlink dump` leaves hart 0 halted after the first read while hart 1 keeps
  running.** Read the observation block **once**; a frozen `h0_seq` after that
  is the probe, not the image. This is the mirror image of the SDI-console
  behaviour in `AGENTS.md` (attach pauses hart 1). It matters for every future
  dual-core test on this board.
- `mcycle` is readable on V5F and counts the 400 MHz core clock. On V3F
  `csrr mcycle` reads 0 (no counter), so cycle accounting is hart-1-only.
- With the default `Config::with_hsi()` recipe, `Delay.delay_ms(1000)` on
  **both** harts takes ≈ 0.25 s while `rcc::clocks()` reports 25 MHz. Both
  harts behave identically, so this is a clock-reporting bug in the default
  recipe (or the counters' actual source clock), not a multicore one — worth
  its own issue.
- `probe-rs 0.32` identifies the part (`CH32H417QEU6`, NVM tables, RAM
  regions) but reports **one** RISC-V core, so it cannot observe hart 1.
  `wlink dump` is the only working observation channel here.

## Mapping to `embassy_rp::multicore`

| embassy-rp | CH32H417 spike | Gap |
|---|---|---|
| `Stack<N>` static | static `Hart1Stack` in `.bss` | none |
| `spawn_core1(core1, stack, closure)` | `wake_other_core(_hart1_trampoline)` + statics | needs a `Core1` token / safe API |
| bootrom sets SP, jumps to `core1_startup` | trampoline sets `sp`/`gp` itself | entry must be 1 KiB aligned → linker slot |
| FIFO handshake before core 1 runs | `fence` + `SETEVENT`; hart 0 polls an ack flag | handshake code to write |
| shared `.data`/`.bss`, core 0 initialises once | same | none |
| `critical-section` impl is multicore-safe | `SingleHartCriticalSection` | **blocker** |
| `Peri<'static, CORE1>` ownership split | `Peripherals::steal()` | **blocker** |
| time driver per core | SysTick0 driver only; V5F uses `SysTick1` (vector 13) that `qingke-rt` does not expose | **blocker** |

## Blockers, in the order they will bite

1. **`Peripherals` is a single-image singleton — partly solved.**
   `ch32_hal::init_secondary()` still calls `Peripherals::take()` and panics
   when hart 0 already took it (`src/lib.rs:335`). API v1 sidesteps it by
   passing peripherals *by move* into the closure: Rust prevents moving the
   same field twice, but nothing stops hart 0 from keeping a peripheral whose
   registers hart 1 also touches. A `Peripherals::split()` /
   `Peri<'static, CORE>` token would make that explicit; `init_secondary`
   should get a variant that does not `take()`.
2. **Hart 1 has no interrupt setup.** The API leaves `mtvec` untouched and
   interrupts disabled. An interrupt-driven executor or `embassy-time` on
   hart 1 needs a per-hart `mtvec` pointing at the shared vector table plus
   the V5F `corecfgr`/`intsyscr` values (the spike deliberately does not call
   qingke-rt's V3F `_setup_interrupts` on V5F). Remember IRQs above 31 are
   delivered to exactly one core, chosen by whoever enables them.
3. **`critical-section` is local-only.** `qingke`'s `SingleHartCriticalSection`
   masks interrupt state; it does not serialise harts. Anything both harts
   touch (`embassy-sync`'s `CriticalSectionRawMutex`, the mailbox, tracing)
   needs an `amoswap`-based spinlock impl. The A extension and CAS are
   proven working (`unsafe-trust-wch-atomics`, 2×1 M increments), so this is
   implementable in `qingke`, not just desirable.
4. **Embassy time is hart-0 only.** `src/embassy/mod.rs`: the H4 SysTick driver
   is compiled for hart 0; V5F's counter interrupts through `SysTick1`
   (vector 13), which `qingke-rt` does not expose, so V5F images use a TIM
   driver. A dual-core embassy runner needs either `SysTick1` support in
   `qingke`/`qingke-rt` plus a per-hart driver instance, or a TIM driver
   instance per core.
5. **Shared peripheral registers are not concurrency-safe.**
   Example found by reading the code: `Delay` clears its flag with
   `SYSTICK.sr().modify(...)` (`src/delay/impl_systick_v3f_v5f.rs`). `SR` is
   plain read-write (`systick_v3f_v5f.rs`, `Access::ReadWrite`), so that is a
   read-modify-write on a register both counters share: one hart can write
   back a stale copy of the other's `CNTIF` bit and drop it. The same pattern
   exists in GPIO/AFIO/EXTI/RCC configuration. Either each register gets a
   single owning hart, or access goes through a cross-core lock.
6. **Tracing is single-producer.** `defmt-rtt` and SDI have one buffer/fifo;
   both harts printing concurrently corrupts it. Hart 1 should share hart 0's
   logger through a lock or stay silent.
7. **Flash programming with both harts live.** `xtask` currently flashes two
   images; with one image there is one write, but any future runtime
   reflash/OTA has to park hart 1 (it may be executing from flash if the
   image ever moves out of the shared block).

## Memory partition for the homogeneous image

```
flash (load image, 960 KiB, no second slot)
  0x0000_0000 .. 0x000F_0000   one ELF: .init + .handle_reset (flash) plus
                               the load images of .highcode/.data
  0x0800_0000 ..               same window through the USR alias
     the old 64 KiB V3F slot and the 0x0001_0000 V5F slot disappear;
     wlink writes this image through the alias as today

shared SRAM, zero-wait for both cores (512 KiB)
  0x2010_0000  .highcode      all .text/.rodata, copied from flash at boot
               .data/.bss     both harts see the same instance
               hart1 stack    static in .bss (8 KiB in the spike), or a
                              dedicated .hart1_stack section if it should
                              leave the shared block
  0x2017_0000 + 32 KiB        hart 0 stack (existing STACK region)
  0x2017_8000 + 32 KiB        cross-core mailbox (unchanged, both maps)

hart-1 private memory (optional)
  0x200A_0000 + 128 KiB ITCM  only useful for hart-1-only code; a single
                              shared .text cannot be placed here and in SRAM
  0x200C_0000 + 256 KiB DTCM  good place for hart-1 stack/hot data via a
                              dedicated section; first 512 B reserved for the
                              hardware push/pop area
```

Why the shared block: it is the only region that is zero-wait for **both**
cores. V3F has no I-cache (flash ≈ 25 MHz-equivalent), and V5F's I-cache does
not help code that has to be shared anyway. One copy in shared SRAM keeps the
"same program" property, and the 448 KiB the V3F examples already use is
enough for the current images (the spike image is 5.7 KiB; the Ethernet image
is the largest, still inside 448 KiB today).

## API v1 — `ch32_hal::multicore`

`src/multicore.rs`, gated `#[cfg(all(ch32h4, not(qingke_v5f)))]` (boot core only):

```rust
pub struct Stack<const N: usize>;              // aligned 16, `const fn new()`

/// Start `entry` on hart 1; returns after hart 1 has moved the closure out
/// of hart 0's frame (`embassy-rp`'s ack, for the same reason).
pub fn spawn_core1<F, const N: usize>(stack: &'static mut Stack<N>, entry: F)
where F: FnOnce() + Send + 'static;

pub fn hart_id() -> HartId;
pub struct SpinLock<T>;                        // amoswap + local IRQ mask
```

- The trampoline lives in `.text.ch32_multicore_entry` with `.balign 1024`;
  the assembler's section alignment makes the linker place it 1 KiB aligned,
  so **no linker-script change is needed** for the API (the spike's
  `.hart1_entry` slot in `v3f/link.x` predates this).
- `spawn_core1` stores the stack top, a pointer to `ManuallyDrop<F>` on hart
  0's stack, and a type-erased `extern "C" fn(*mut ()) -> !`; `fence(Release)`;
  `wake_other_core`; then spins on the ack with a bounded retry.
- The trampoline sets `gp` from `__global_pointer$`, loads `sp` from the
  static, `fence rw, rw`, enables `mstatus.FS`, and calls back into Rust.
- Peripherals are handed over **by move** (`let pf0 = p.PF0;` then capture
  `pf0`), so there is no `steal()` in user code and no double owner.
- `SpinLock<T>` is the stopgap for shared state until a real cross-core
  `critical-section` exists: `compare_exchange` on the lock plus
  `critical_section::with` around it so a same-hart interrupt handler cannot
  deadlock the thread it interrupted.

Verified with `multicore.rs` (default clock recipe, 3 consecutive flashes):
`h1_hart = 1`, `h1_done = 1`, and `SpinLock` value **200000 exactly** after
100 000 `with(|v| *v += 1)` on each hart — no lost updates across three runs.
Both harts ran `Delay` and GPIO through the same `.text`/`.data`.

Constraints the implementation has to encode:

- entry symbol in a 1 KiB-aligned section (`PFIC_WAKEIP` ignores [9:0]);
- `gp`/`sp` set before any Rust code, no `_start` on hart 1 (never re-copy
  `.data` or clear `.bss`);
- `fence rw, rw` before `SETEVENT` and after entry; ack flag so hart 0 can
  wait for the stack/entry publication;
- a cross-core `critical-section` impl before anything in 2 is used;
- per-hart peripheral ownership, not `steal()` in user code.

## Open questions for the deeper research pass

- Litmus test on this silicon: store data → `fence` → store flag; does the
  second hart always see the data? And does `SETEVENT` itself order prior
  stores (the spike adds an explicit fence, the current mailbox convention
  does not)?
- Interrupt routing: `init_secondary` already documents that an IRQ above 31
  is delivered to exactly one core and enabling assigns it to the enabling
  core. Both harts programming `PFIC` needs a policy.
- Cache: V5F I-cache is enabled by the existing V5F examples. If code in the
  shared block is ever modified or reloaded, `fence.i` + cache invalidation is
  needed; if a D-cache is ever enabled, the shared region has to be marked
  uncacheable (`cache_pmp_ovr`) or maintained explicitly.
- Does hart 1 need the V5F-specific `corecfgr` / `intsyscr` values, or is
  qingke-rt's V3F `_setup_interrupts` enough? The spike does not call it (no
  interrupts on hart 1); an embassy runner will have to.
- Per-hart time: `SysTick1` (vector 13) vs a TIM per core, and whether
  `embassy-time` can be instantiated twice (its driver state is a single
  static today).
- `probe-rs` cannot see hart 1 (one core in its description). Check whether a
  newer probe-rs or a custom `wch` debug sequence can, since the wlink
  halt/resume asymmetry makes automated dual-core tests painful.

## Recommended order

1. **Finish the API (no dependency on the lock or the time driver):** hart-1
   interrupt bring-up (per-hart `mtvec`, V5F `corecfgr`/`intsyscr`, vector
   table shared with hart 0), a `Peripherals::split()` / `Core1` token, and an
   `init_secondary` that does not `take()`. This is what makes the API usable
   for an `embassy_executor` runner, and it is the test bed for the next step.
2. **Cross-core `critical-section`** (upstream `qingke`): `CriticalSectionRawMutex`
   and any shared logger/mailbox are unsound without it; `SpinLock` is a
   stopgap in user code, not a replacement. Needs hart 1 running interrupts to
   test the "handler vs thread on the same hart" path.
3. **Per-hart time driver:** `SysTick1` (vector 13) in `qingke-rt` plus
   per-hart `embassy-time` state, or one TIM per core. Only this unlocks
   `embassy_time` tasks on hart 1; an executor with polling tasks already
   works today.

Both 2 and 3 depend on step 1, not the other way round — that is why the API
comes first.
