# Task: CH32H417 — `Delay.delay_ms` is 4× too fast with the default HSI clock recipe

Repo: `ch32-rs/ch32-hal` (local checkout `~/Repos/ch32-hal`; board on the bench).
Chip: CH32H417QEU6, `0x4170052d`, WCH-LinkE probe `1a86:8010:D7708F065C7F`
(three probes are attached — pin the right one).

This is independent of any multicore work: it reproduces on hart 0 of a plain
default-config image.

## Symptom

With the default clock config

```rust
let _p = ch32_hal::init(ch32_hal::Config::default()); // == Config::with_hsi()
```

`ch32_hal::delay::Delay.delay_ms(1000)` returns after **≈250 ms** of wall-clock
time (4× too fast), while `ch32_hal::rcc::clocks()` reports 25 MHz for
`sysclk`, `hclk`, `v3f`, `v5f`.

With

```rust
config.rcc = ch32_hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hsi();
```

the same call is **accurate**: hart 1 measures `mcycle` = 400 000 172 for a
1 000 ms delay (1.000 s at the reported 400 MHz), and a 1 s heartbeat counts
10 per 10 s on both harts.

Measured on `examples/ch32h417/v3f/src/bin/multicore.rs` (default config,
`delay_ms(500)`): the counter advances ~4×/s instead of 2×/s. This is not a
dual-core bug — hart 0 alone shows it.

## Where to look

- `src/delay/impl_systick_v3f_v5f.rs`
  - `current_hart()` → `(index, rcc::clocks().v3f.0)`
  - `wait_chunk()` programs `SYSTICK.cmp(index) = cycles`, `cnt = 0`,
    `ctlr(index).stclk = HCLK`, `mode = UPCOUNT`, then waits on
    `SR.CNTIF<index>`; the module header says RM §4.6 clocks *both* counters
    from the V3F HCLK domain.
- `src/rcc/h4/mod.rs` (`Config::with_hsi`, `with_sysclk_400m_…`),
  `src/rcc/h4/measure.rs`, `src/rcc/clock_tree_reference.rs` — how `clocks()`
  is computed vs what the recipe writes.
- `examples/ch32h417/v3f/src/bin/delay_systick.rs` — claims one tick per
  wall-clock second with the default config; check whether that was ever
  verified against a wall clock.

## Hypotheses (verify, do not assume)

1. `with_hsi()` leaves (or the chip reset default gives) V3F HCLK ≈ 100 MHz
   while `clocks()` reports 25 MHz, so the SysTick counters tick 4× faster
   than the delay assumes.
2. The counters clock from the V3F **core** clock, not HCLK, and the core
   clock is 4× SYSCLK in the HSI recipe.
3. The HSI recipe does not program the clock tree it reports (e.g. leaves the
   PLL/mux at a reset state), so `clocks()` computes a tree that is not on
   silicon.
4. `cycles()`/`wait_chunk()` math bug (less likely: the 400/400/100 recipe is
   accurate with the same code).

## How to measure without trusting `clocks()`

- Flash a firmware that increments a RAM counter once per `delay_ms(1000)`
  on hart 0 only (no console). Read it **once** with `wlink dump` after a
  host-side `sleep 30`: ≈30 increments means correct, ≈120 means 4× fast.
  Read once only — on this board a `wlink dump` leaves hart 0 halted after the
  first read (hart 1 keeps running), so repeated reads freeze the counter.
- Dump the RCC registers (`CFGR0/CFGR1/PLLCFGR/…`) with `wlink dump` and
  compare against RM §4.6 and the WCH CSDK's `SetSYSCLK_*` sequence.
- `csrr mcycle` reads 0 on V3F (hart 0) and counts the core clock on V5F
  (400 000 000/s at 400 MHz). Treat that as a separate question; if V3F
  genuinely has no `mcycle`, note it and do not use it as a V3F reference.

## Constraints

- Build/flash from `examples/ch32h417`:
  `WLINK_PROBE=1a86:8010:D7708F065C7F cargo xtask flash --example <name>`.
  Do not use `--watch-serial` while hart 1 matters; it pauses hart 1.
- `wlink flash` occasionally under-writes silently; `xtask` verifies with
  `--no-run`, keep using `xtask`.
- Do not change the default config's *behaviour* for existing examples
  without checking the wall-clock result; many examples use it.
- Leave the multicore branch alone; fix this on a normal branch off `main`.

## Deliverable

- Root cause with register-level evidence (which clock the counters actually
  use, and why `clocks()` disagrees).
- Minimal fix (delay, clocks computation, or recipe — whichever is actually
  wrong; document the choice).
- A regression check that does not depend on the broken clock: e.g. a bench
  script/example that counts `delay_ms(1000)` iterations over a host-side
  sleep and asserts the count is within tolerance, or a comparison against a
  second independent timebase.
- A short note in the repo (docs) and an issue/PR summary, including the
  separate `mcycle`-on-V3F observation if it is unrelated.
