# CH32H417 as a homogeneous dual core — feasibility spike

Status: **spike done, API v1 implemented (`ch32_hal::multicore`), hart 1 runs
its own interrupt setup + `embassy_executor`, and cross-core
`critical-section` is implemented on a qingke branch — all verified on the
bench board. Cross-core `embassy_time` works when hart 0 also schedules, but a
hart-1-only timer does not fire yet. Remaining gaps: that time bug, V5F-specific
core setup, checked peripheral ownership, and getting the qingke change
released.**
Worktree branch: `dual-core-homogeneous`. Examples:
`examples/ch32h417/v3f/src/bin/homogeneous.rs` (mechanism),
`.../multicore.rs` (API + lock litmus), `.../multicore_embassy.rs` (one
executor per hart + hart-1 interrupt).

Question: can CH32H417 run one image on both harts, the way
`embassy_rp::multicore::spawn_core1` runs RP2040 core 1 — same `.text`, same
`.data`/`.bss`, second hart with its own stack and entry — instead of building
two images (V3F at flash `0x0000_0000`, V5F at `0x0001_0000`)?

Short answer: **yes.** The second hart boots into a 1 KiB-aligned trampoline
inside the boot image, sets `gp`/`sp`, installs the shared vector table, and
runs the same Rust code, HAL drivers and an `embassy_executor`. A cross-core
`critical-section` is implemented on a qingke branch. What is still missing is
a per-hart time driver, V5F-specific core setup, an audit of shared peripheral
registers, and a released qingke with the new feature.

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

## Cross-core `critical-section` (qingke branch)

`~/Repos/qingke`, branch `dual-core-critical-section`, commit `9b855be`:

```toml
# qingke/Cargo.toml
dual-core-critical-section = ["critical-section-impl"]
```

`src/critical_section_impl.rs` now has two variants selected by that feature.
The dual-hart one masks `MIE`/`MPIE` on the calling hart (as before), then
takes a shared `AtomicBool` with `compare_exchange`, tracked per hart with a
depth counter, and releases in reverse order. Mechanism notes:

- Interrupts are masked **before** the spinlock, so an ISR on the same hart
  cannot spin on a lock its own thread holds.
- Nesting is per hart, so `critical_section::with` inside `with` is safe; the
  other hart still sees the lock held.
- Releasing the lock before unmasking avoids a woken handler observing it
  still held.
- The lock state is in `.bss`, so this is only correct in the single-image
  model. Two separately flashed images get two independent locks and no
  exclusion.

To reproduce: check out that qingke branch, add
`[patch.crates-io] qingke = { path = "~/Repos/qingke" }` to
`examples/ch32h417/v3f/Cargo.toml`, add

```toml
qingke = { version = "0.8.4", features = [..., "dual-core-critical-section"] }
```

and build with `--features dual-core-critical-section` (the v3f feature of the
same name turns on the example half; it does not depend on the qingke feature,
because that would not resolve against the released 0.8.4). Without the qingke
patch the example still builds but `cs_value` will be short — that is the
demonstration of why the single-hart section is not enough.

Result: 2 × 100 000 increments gave `lock_value = cs_value = 200000` on four
consecutive flashes (three before, one after the cfg refactor), with both
harts advancing.

## Cross-core `embassy_time`: root cause and fix

Symptom: when hart 1 is the only `embassy_time` user, `Timer::after` never
completes; when hart 0 also schedules, both harts' timers work.

Measured with temporary counters inside the driver plus `wlink dump` of
`0xE000F000`:

| Check | Value |
|---|---|
| hart 1 → `schedule_next` entries | 1 |
| `set_alarm` returned "armed" | 1 |
| `SysTick` ISR entries | **0** |
| `set_alarm` "deadline already passed" | 0 |
| `SYSTICK.CMP(0)` after arming | `0x80000000` (still `init`'s half-period value) |
| raw hart-1 store `0x12345678` to `CMP0` | does not stick |
| hart 1 read of `CNT0` | live (6.64 M while the register read 8.22 M a moment later) |

So **the V5F can read counter 0 but writes to counter 0's control/compare
registers are dropped**. `set_alarm` from hart 1 arms nothing; the comparator
stays at the half-period maintenance value, and hart-0 timers work only because
hart 0's own `schedule_wake` programs the comparator.

The fix, in order of importance:

1. **Counter 0 is hart-0-only.** `schedule_next()`/`set_alarm()` must run on
   hart 0. From hart 1, `schedule_wake` only inserts into the shared queue.
   It must also not touch `SR`: that register *is* shared, and clearing
   `CNTIF0` from hart 1 would eat hart 0's pending compare.
2. **Short re-arm tick.** `init` arms the maintenance alarm at ~1 ms instead
   of half a wrap, and the `SysTick` ISR calls `schedule_next()` on every
   tick. A wake inserted by hart 1 is then picked up within one tick (worst
   case added latency = tick period; once re-armed, the deadline is exact).
   Cost: ~1 kHz idle interrupt on hart 0; tunable (250 µs – 2 ms).
3. **`Driver::now()` stays as it is**: hart-1 reads of `CNT0` are valid.
4. **Regression test**: an example where *only* hart 1 schedules a `Timer`
   (hart 0 just yields) and the counter must advance; plus a check that
   `CMP0` changes when the alarm is armed from hart 0.

Rejected alternatives:

- **Cross-core doorbell** (hart 1 pokes hart 0 instead of a tick): PFIC
  enable/pending is core-local (`pend_interrupt` from hart 1 delivers to
  hart 1), and `SEV` only wakes `WFI`/`WFE` without running driver code. No
  usable primitive today.
- **Per-hart driver on counter 1 / `SysTick1`**: needs `qingke-rt` vector 13
  and two global `embassy-time` drivers, which the `embassy-time-driver` API
  does not support. Not the fix for `embassy_time`.
- **Move the timebase to hart 1**: mirrors the same problem for hart 0.

Blocking `Delay` is unaffected: it uses the calling hart's own counter (0 on
hart 0, 1 on hart 1), so it never writes across the domains.

## Blockers, in the order they will bite

1. **`Peripherals` is a single-image singleton — partly solved.**
   `ch32_hal::init_secondary()` still calls `Peripherals::take()` and panics
   when hart 0 already took it (`src/lib.rs:335`). API v1 sidesteps it by
   passing peripherals *by move* into the closure: Rust prevents moving the
   same field twice, but nothing stops hart 0 from keeping a peripheral whose
   registers hart 1 also touches. A `Peripherals::split()` /
   `Peri<'static, CORE>` token would make that explicit; `init_secondary`
   should get a variant that does not `take()`.
2. **Hart 1's interrupt setup is the boot core's, not V5F's — works, but
   incomplete.** `multicore` calls qingke-rt's `_setup_interrupts`, which
   writes the V3F `corecfgr` value (`0x123703E1`); the vendor's V5F startup
   also sets V5F-only bits (`NLP_EN`, `GHR_EN`, `LSU_DUAL`). Interrupt
   delivery and the hardware stack work on the bench, but the V5F-specific
   configuration belongs in `qingke-rt` (a `v5f` branch of the startup) before
   this is considered finished. Also remember IRQs above 31 are delivered to
   exactly one core, chosen by whoever enables them; `qingke::pfic::enable_interrupt`
   writes the whole `IENR` word, so it must only be used by a hart that owns
   that bank.
3. **`critical-section` needs qingke's new dual-hart feature — implemented,
   not released.** `SingleHartCriticalSection` masks interrupt state and does
   not serialise harts, so `CriticalSectionRawMutex` (and therefore
   `embassy-sync`), the shared logger and the mailbox are unsound without it.
   Implemented on the `dual-core-critical-section` branch of the local qingke
   checkout (`~/Repos/qingke`, commit `9b855be`): the critical section masks
   the calling hart's interrupts, then takes an `amoswap` spinlock shared by
   both harts, with per-hart nesting depth so a `with` inside a `with` does
   not self-deadlock. Measured on the bench (see below): 2 × 100 000
   `critical_section::with` increments of a shared `u32` gave **200000
   exactly** on 4 runs. Caveats: it relies on both harts executing the same
   image (the lock lives in `.bss`), it deadlocks if a hart is halted by the
   debugger while holding it, and it only makes sense for short sections.
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

Before the closure runs, hart 1 calls `qingke-rt`'s `_setup_interrupts`
(same call hart 0's `_start` makes): shared `mtvec`, hardware push/pop stack,
`mstatus.MIE`, `corecfgr`/`intsyscr`/`inestcr`. The closure therefore starts
with interrupts on.

Measured (default clock recipe unless noted):

- `multicore.rs`, 3 consecutive flashes: `h1_hart = 1`, `h1_done = 1`, and
  `SpinLock` value **200000 exactly** after 100 000 `with(|v| *v += 1)` on
  each hart — no lost updates. Both harts ran `Delay` and GPIO through the
  same `.text`/`.data`.
- `multicore_embassy.rs`: an `embassy_executor::Executor` on each hart; both
  cooperative tasks advance (`h0_seq` 411 k → 3.2 M while `h1_seq` 411 k →
  4.6 M over 5 s), and the self-wake path works.
- The same example pends `IPC_CH0` (interrupt 16) on hart 1 only. `h1_irqs`
  reads exactly **1** and stays 1: the shared vector table dispatched to
  hart 1, the handler ran, `unpend_interrupt` cleared it with no storm.
  Hart 0's interrupts were untouched.
- Cross-core `embassy_time`: with hart 0 also scheduling, hart 0's and
  hart 1's `Timer::after(500 ms)` counters advance in lockstep (6 → 47 in 5 s,
  the 4×-fast default HSI recipe), i.e. hart 1 tasks are woken by hart 0's
  SysTick ISR. **But** in two runs where hart 1 was the *only* user of
  `embassy_time`, `h1_timer_ticks` stayed 0: the alarm never fired. Root cause
  and fix are below ("Cross-core `embassy_time`: root cause and fix"): the V5F
  cannot program counter 0's comparator.

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

1. **Done — API v1 + hart-1 interrupts + executor.** `spawn_core1`, `Stack`,
   `hart_id`, `SpinLock`, `_setup_interrupts` on hart 1, and
   `multicore_embassy`. Still open inside this step: V5F-specific
   `corecfgr`/`intsyscr` (today hart 1 gets the V3F values),
   `Peripherals::split()` / `Core1` token, and an `init_secondary` variant
   that does not `take()`.
2. **Done in qingke, waiting for release / upstream review — cross-core
   `critical-section`.** Feature `dual-core-critical-section` on the qingke
   branch, verified with `multicore` (`--features dual-core-critical-section`
   against that checkout). Next actions: upstream the qingke change, cut a
   release, then make the H4 chip features enable it (and drop the HAL
   `SpinLock` from the docs as the recommended path). The "handler vs thread
   on the same hart" nesting path is covered by the depth counter; it has not
   been stress-tested with a real IRQ storm yet — that is worth adding when
   the API grows interrupt-driven drivers.
3. **Next — implement the cross-core time fix** specified above (hart-0-only
   arming + ~1 ms re-arm tick + hart-1-only regression test). No qingke-rt
   change and no per-hart driver are needed for `embassy_time`.
