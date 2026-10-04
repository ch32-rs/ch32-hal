# Verifying QingKe hardware atomics (V3F / V5F on CH32H4)

## Why this doc

The `qingke` crate currently refuses to build whenever rustc reports
`target_has_atomic = "8"` unless the user opts in with the
`unsafe-trust-wch-atomics` feature
([qingke 75dbd95][qingke-guard]). The guard was added in 2024-11 after
[ch32-rs/ch32-hal#59][issue-59] documented a USB FS lock-up on V307
(QingKe V4) traced to an `embassy_executor` task showing
`TaskHeader.state == 3` (atomic-flagged "enqueued") while the run queue
was empty — switching `embassy_executor` from its atomic-queue
implementation to the critical-section queue made the lock-up
disappear.

The evidence is **indirect** and **V4-specific**. CH32H4's V3F and V5F
harts are newer cores; the errata may or may not still apply. We
currently dodge the question with `+forced-atomics` in our target
JSON, which makes the compiler synthesise atomics via critical
sections — correct but slower, and pointless on hardware that doesn't
actually need it.

This doc captures a verification plan we want to run before flipping
H4 examples to real hardware atomics.

[qingke-guard]: https://github.com/ch32-rs/qingke/commit/75dbd9539d5abf66f24435b66da3a02bb251dde6
[issue-59]: https://github.com/ch32-rs/ch32-hal/issues/59

## What we want to know

1. **V3F (hart 0) single-hart atomics** — does an LR / SC sequence
   correctly retry after an ISR preempts it? Failure mode: lost
   increments.
2. **V5F (hart 1) single-hart atomics** — same question.
3. **Inter-hart atomic ordering** — do `Release` stores on one hart
   become visible to `Acquire` loads on the other? Failure modes:
   stale loads (store-buffer not flushed), reordered observations
   (Dekker / IRIW litmus).
4. **Inter-hart `compare_exchange` correctness** — does an LR on
   hart 0 see the SC of hart 1 break its reservation? Failure mode:
   both harts CAS succeed on the same word (lost update).

## Test setup

### A. Single-hart "ISR steals reservation" test (V3F, V5F separately)

```rust
static COUNTER: AtomicU32 = AtomicU32::new(0);

#[ch32_hal::entry]
fn main() -> ! {
    // Run on V3F. SysTick → ISR. SysTick set to fire every N μs.
    setup_systick(/* every 100 μs */);
    let mut expected: u64 = 0;

    loop {
        // Tight increment loop; should retry SC on ISR-induced reservation loss.
        COUNTER.fetch_add(1, Ordering::SeqCst);
        expected += 1;

        if expected % 1_000_000 == 0 {
            let actual = COUNTER.load(Ordering::SeqCst) as u64
                       + ISR_COUNT.load(Ordering::SeqCst) as u64;
            // (ISR_COUNT is incremented from the ISR; total should equal
            // expected + isr_iters)
            println!("expected={expected}, observed={actual}, drift={}", expected as i64 - actual as i64);
        }
    }
}

#[interrupt]
fn SysTick() {
    ISR_COUNT.fetch_add(1, Ordering::SeqCst);
    COUNTER.fetch_add(1, Ordering::SeqCst);  // tight contention with main
    clear_systick_irq();
}
```

Run for ≥ 10 minutes. Acceptance: drift should be `0`. Any positive
drift means main-loop SC was acknowledged into `COUNTER` from atomic
state but the result didn't land, or the ISR-side increment was lost.

### B. Dual-hart Dekker (V3F ↔ V5F)

```rust
static X: AtomicU32 = AtomicU32::new(0);
static Y: AtomicU32 = AtomicU32::new(0);
static X_SAW_ZERO: AtomicU32 = AtomicU32::new(0);
static Y_SAW_ZERO: AtomicU32 = AtomicU32::new(0);

// hart 0 (V3F)
loop {
    X.store(1, Ordering::Release);
    if Y.load(Ordering::Acquire) == 0 {
        Y_SAW_ZERO.fetch_add(1, Ordering::Relaxed);
    }
    // reset for next round (using a barrier on the other hart)
    X.store(0, Ordering::Release);
    while Y.load(Ordering::Acquire) != 0 {}
}

// hart 1 (V5F): mirror
loop {
    Y.store(1, Ordering::Release);
    if X.load(Ordering::Acquire) == 0 {
        X_SAW_ZERO.fetch_add(1, Ordering::Relaxed);
    }
    Y.store(0, Ordering::Release);
    while X.load(Ordering::Acquire) != 0 {}
}
```

SC consistency says at most one of the two can see the other as
unset on any given iteration. Both reaching nonzero (i.e. both
counters increment by 1) per iteration **must not** happen. Drive
the loop for ≥ 1M iterations and assert
`X_SAW_ZERO + Y_SAW_ZERO ≤ iterations` (no overlap).

If both counters incremented on the *same* iteration we'd need a
shared iteration marker; the cleaner thing is to assert each
counter alone stays ≤ iterations. A violation means the
release/acquire fences are not flushing the store buffer between
harts.

### C. Dual-hart CAS race

```rust
static SLOT: AtomicU32 = AtomicU32::new(0);
static WINNERS_V3F: AtomicU32 = AtomicU32::new(0);
static WINNERS_V5F: AtomicU32 = AtomicU32::new(0);

// both harts
loop {
    if SLOT.compare_exchange(0, MY_HART, AcqRel, Relaxed).is_ok() {
        MY_HART_WINNERS.fetch_add(1, Relaxed);
        // ... critical section ...
        SLOT.store(0, Release);
    }
}
```

After N iterations on each hart, `winners_v3f + winners_v5f` should
roughly equal total CAS attempts × hit-rate. The key invariant:
**both harts should never observe `SLOT == 0` and both succeed in the
same window** — if you see signs of double-entry (e.g. side-effect
counter going up twice in one CAS-claimed critical section), the LR
reservation isn't being broken across cores.

A simpler probe: have each winner increment a per-hart counter, then
periodically print SLOT, WINNERS_V3F, WINNERS_V5F. Validate
WINNERS_V3F + WINNERS_V5F equals the number of times SLOT cycled
0 → MY_HART → 0.

## Mechanical wiring

Where to drop these in:

```
examples/ch32h417/src/bin/litmus_atomic_isr.rs       — test A
examples/ch32h417/src/bin/litmus_atomic_dekker.rs    — test B (needs V5F bring-up)
examples/ch32h417/src/bin/litmus_atomic_cas_race.rs  — test C (same)
```

Build matrix:

| feature combo | atomic impl |
|---|---|
| default (target JSON has `+forced-atomics`) | critical-section synthesised (baseline — should never lose) |
| `unsafe-trust-wch-atomics` + target JSON with real `+a`, `atomic-cas: true` | hardware LR/SC (the thing under test) |

Run both, diff results. If hardware-atomic build matches the
baseline over multi-hour runs, we have justification to flip the
target JSON to real atomics (no more `+forced-atomics`) and add an
H4-specific feature override in the qingke crate.

## Constraints / what we won't measure here

- **Single-iteration crash repros** — none of these tests catches a
  "one-shot" atomic miss; they're all statistical. Running for
  ≥ 1 hour at high contention is the bar.
- **Cache coherence corner cases** — H4 V5F has an ICache; if there's
  a data-cache coherence bug (extremely unlikely on a 2-core SoC of
  this size, but worth noting), these tests don't directly exercise
  it. Add `cache_invalidate` if doing memory-mapped peripheral
  ordering.
- **TSO vs sequential consistency distinction** — RISC-V RVWMO
  permits some reorderings that x86 TSO forbids. Tests B / C use
  `Release` / `Acquire` explicitly to match RVWMO and shouldn't
  produce false positives on a *correct* implementation.

## Triggering the work

This isn't a blocker for the current H4 mux + RCC + GPIO + USART
foundation PR. Flag it as a separate task:

- [ ] Build the three litmus binaries.
- [ ] Run each for ≥ 1 hour on EVT board (V3F-only first; V5F bring-up
      is its own PR).
- [ ] Cross-reference findings with ch32-rs/ch32-hal#59 — if H4 is
      clean, add an issue documenting that result so future maintainers
      know V3F/V5F is OK.
- [ ] If H4 is clean: add `cfg(ch32h4)` feature override in qingke's
      atomic guard, drop `+forced-atomics` from H4 example target JSON.
- [ ] If H4 still misbehaves: file an errata report with WCH,
      document in `extracting-ch32-register-data` skill.

## See also

- ch32-rs/ch32-hal#59 — original V4 atomic lock-up evidence.
- qingke `75dbd95` — the compile-error guard.
- RISC-V RVWMO specification (RISC-V Privileged Spec, chapter "Memory
  Model") for what release / acquire is allowed to do.
- WCH QingKe V5 IP manual §8 (PFIC + memory model) for any V5-specific
  ordering guarantees.
