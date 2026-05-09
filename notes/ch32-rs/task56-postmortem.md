# task #56 postmortem — fnGetClockCBs link.x pin retirement

**Period**: 2026-05-06 — 2026-05-09
**Owner**: @Lucy
**Collaborators**: @Andelf (delegation + final approval), @Vega (code execution + reviews), @Cindy (hardware gates)
**Final outcome**: link.x `_ebss=0x20001c78` + `KEEP(*(.fnGetClockCBs))` pin permanently retired. Production mechanism: single `write_volatile(addr_of_mut!(fnGetClockCBs), 0x420B_000A)` in `ble_ip_core_init`.
**Iron Law #38 three-condition gate**:
- Step B discriminator (`d056863`, +4 KB pad): warm cba=43, power-cycle cba=32, scrub-then-reset cba=54 — all PASS.
- Terminal regression (`d50907a`, frozen, MD5 `3e69b556`): warm cba=39, power-cycle cba=45, scrub-then-reset cba=52 — all PASS.

---

## TL;DR

ROM IRQ reads `*(volatile u32*)0x20001c78` as a fn ptr but **is NULL-tolerant** —
falls back to internal handler (`clockGetHSEValue` at 0x420B_000A) when the slot
is NULL. For the **frozen binary layout**, `_ebss` naturally covers 0x20001c78
so qingke-rt startup BSS clear gives NULL → ROM fallback runs cleanly →
`ble_ip_core_init`'s explicit symbol-resolved write installs the same fn ptr the
ROM would have used → BLE TX works. The link.x address pin was archaeology, not
a hardware contract; we proved this by force-shifting the symbol +4 KB via a
BSS pad and confirming the system still works.

The journey to that one-paragraph answer took **3 days, 5+ failed hypotheses,
4 hardware gate cycles, and 1 missed orphan-section bug**. This document is
the postmortem.

---

## Core issue causes (root-cause list)

The five recurring failure modes in this saga, in priority order:

### 1. Hypothesis stacking without empirical falsification

For weeks the working model was "ROM auto-installs `0x420B000A` at 0x20001c78
on cold boot". That premise was **assumed**, not tested. It survived because:
- The slot **was** observed at `0x420B000A` after warm reboots.
- The slot **was** observed at `0x420B000A` after WCH-Link 3V3 power-cycles.
- Both observations were actually warm-boot SRAM persistence (cause #5 below),
  not ROM auto-install.

Falsification took one cold-boot test (`baf71de`, Cindy 2026-05-08): post-run
slot=NULL, cba=0. The "ROM auto-installs" wording sat in 4 source files and
notes for months because nobody had built a binary that *forced* the cold-boot
path with the slot guaranteed-NULL at boot.

**Lesson**: any claim about ROM/silicon behavior must be reduced to a falsifiable
prediction with a hardware test that distinguishes it from competing models.
Iron Law #38 (three-condition gate) is the structural fix: cold + warm + scrub
forces every assumption to defend itself against deterministic-NULL boot.

### 2. Static analysis ≠ semantic equivalence

Task #56 v5 (commit `5528b5f`) replaced the ROM `0x420B000A` tick fn with a
RISC-V `cycle` CSR read (`fallback_clock`). nm + disasm verified the code change
was structurally well-formed: `bb_irq_lib_handler` indirected through a static
slot vs. directly calling `cycle` CSR; both paths terminated correctly; both
returned a u32. **Static analysis was sufficient to prove the code compiled and
linked, but could not detect that the value semantics were 60,000× off**
(96 MHz CPU clock ≠ 1600 Hz BLE timing contract per WCH manual §3.3).

Air-visible cba=0 caught this within 60 seconds when we finally ran the binary
on hardware — but only after we had already **committed** v5 based on static
verification.

**Lesson**: any change to a value flowing into BLE ABI words (`gBleIPPara[*]`,
`gBleLlPara[*]`, the fnGetClockCBs slot) requires a hardware air-visible cba≥5
gate **before commit** — Iron Law #38, codified specifically because of this
incident.

### 3. Mistaking layout coincidence for hardware contract

The address `0x20001c78` was originally observed in **a specific WCH reference
build's BSS layout** with a specific `BLE_MEMHEAP_SIZE` and a specific set of
`.c` files. The ch32-hal port copied that address verbatim into link.x as a hard
pin, justified as "WCH places it there → ROM expects it there".

C ground-truth experiment (2026-05-09, `notes/ch32-rs/c-ground-truth-fnGetClockCBs.md`)
showed:
- `fnGetClockCBs` is a **COMMON** symbol in libwchble.a (size 4, type C).
- All `tmos_hw.o` references use `R_RISCV_PCREL_HI20` relocations.
- WCH ships EVT examples with `BLE_MEMHEAP_SIZE` from 4096–10240; each lands
  `fnGetClockCBs` at a different address; all work.
- Our own EVT Broadcaster build with default 6144 puts it at **0x20002420**,
  not 0x20001c78.

The pin was archaeology: a fingerprint of one historical binary, not a hardware
constraint.

**Lesson**: when porting from a reference C build, distinguish between
"observed in the reference binary" and "required by the hardware". The
discriminator is whether changing the layout breaks the chip — which can only
be answered by deliberately changing the layout and gating it on hardware.
Step B (the 4 KB BSS pad force-shift) was that experiment.

### 4. Confusing two consumer paths into one mental model

The fnGetClockCBs slot has **two different consumers**:
- **Userland Rust `bb_irq_lib_handler` .L7 path** reads via PCREL_HI20 to the
  linker-placed symbol address (whatever it happens to be).
- **ROM IRQ handler** reads the absolute physical word at 0x20001c78
  unconditionally, regardless of where the Rust symbol lands.

Multiple hypothesis iterations over-generalized in one direction or the other:
- **H3 ("PCREL only")**: predicted minimal binary should work because Step1
  wrote to symbol@0x200007c0. **Falsified** by minimal FAIL — ROM read
  0x20001c78=garbage and crashed.
- **H2a ("absolute only, requires non-NULL")**: predicted Step B should FAIL
  because 0x20001c78 would be NULL. **Falsified** by Step B PASS — ROM tolerated
  NULL via internal fallback.
- **H4 (correct, dual-path with NULL-tolerant abs)**: ROM IRQ reads abs, falls
  back on NULL; userland reads PCREL. Both paths are real, both must be served
  in the production write.

The frozen binary path "accidentally" satisfies both consumers: in frozen
layout, the symbol IS at 0x20001c78, so the symbol-resolved write IS the
absolute write. This is the layout that production uses today.

**Lesson**: when a system has multiple consumers of the same data, enumerate
them explicitly. Don't infer a unified mental model from the symptoms of any
one consumer.

### 5. Warm-boot SRAM persistence masking cold-boot bugs

The CH32V208 SRAM retains state across resets and even WCH-Link 3V3
power-cycles for several seconds (long enough that wlink scripts running back
to back observe the previous binary's RAM content). This **silently propped up**
multiple bad code paths:
- Phase 2a's "boundary trick" (0x20001c78 outside `_ebss`, never zeroed)
  worked because warm boot retained whatever was last written there.
- Step 3 FAIL was actually **two** separate bugs combined: (a) pin removed,
  (b) Step1 missing — but (a)+(b) only manifested as cba=0 because the warm
  SRAM at the OLD pinned address still held a stale `0x420B_000A` from a
  previous run; the NEW symbol address was untouched.
- "ROM auto-installs" (cause #1) was a misreading of warm SRAM retention —
  every test had observed `0x420B_000A` in the slot because every prior test
  put it there.

The cure was the **scrub gate**: explicitly write `0xDEADBEEF` to the target
address, then reset, then test. If `cba` still PASSes (Step B scrub: cba=54),
the system does not depend on warm retention. If it fails, you've either
exposed a SRAM-persistence dependency or, when combined with code changes,
disambiguated which mechanism actually carries the value.

**Lesson**: every hardware gate must include a scrub condition. Iron Law #38
codifies this as one of its three required conditions.

### 6 (bonus). Premature retirement of working safety nets

Step 3 (`baf71de`) removed the link.x pin **without** simultaneously adding a
guarantee that the slot would receive a non-NULL value via another mechanism.
The mental model was "ROM auto-installs", so the slot was assumed safe regardless
of layout — but that assumption was wrong (cause #1).

The structural fix is to never retire a safety net unilaterally — always pair
"remove old mechanism" with "verify replacement is in place AND gates pass".
Step 1 (`52ed8dc`, explicit write) should have been merged BEFORE Step 3
(`baf71de`, pin removal); doing them simultaneously without ordering trapped
us into the v3/Step 3 FAIL.

---

## Full timeline

### Pre-task #56 baseline (Phase 2a, ≤ 2026-05-07)
- 5 of 6 BSS-contract symbols already de-pinned (Iron Law #34 v5 first version).
- `fnGetClockCBs` left pinned at 0x20001c78 via boundary trick (`_ebss` =
  0x20001c78, `KEEP(*(.fnGetClockCBs))` outside zeroed range).
- "ROM auto-installs" model accepted; warm-boot tests PASS (cba ≥ 35).

### Task #56 attempt v5 (`5528b5f`, 2026-05-08 ~21:00)
- Goal: drop the 4-byte slot entirely, inline `fallback_clock()` (RISC-V cycle
  CSR) at the .L7 indirection site.
- Static verification: nm + disasm OK; cargo build clean.
- Hardware gate (Cindy, 2026-05-08 21:35): cba=0/30s, RSSI nothing.
- Bisect: pre-v5 (`9185cf9`) PASS 28/30s; v5 (`5528b5f`) FAIL 0/30s; revert
  (`be4db43`) PASS 35/30s.
- Root cause: `ble_tx_adv_ch37.rs:1940` writes `gBleIPPara[0] = 0x60` (bit5+bit6),
  so .L7 is LIVE on every IRQ. Cycle CSR ~96 MHz vs 1600 Hz contract = 60,000×
  scale error → BLE scheduler timing collapses.
- v5 preserved on `bad/task56-v5-air-regression`; `feat/ble` reverted to
  `be4db43`.

### Iron Law #38 proposed (2026-05-08 23:15)
- Vega-endorsed wording: any change to BLE ABI symbols requires hardware
  cba≥5 gate before commit + match WCH manual's 1600 Hz / 625 µs contract.

### Task #56 option (b) hardware gate (`baf71de`, 2026-05-08 23:00)
- Goal: zero-init the slot inside `_ebss`, rely on assumed "ROM auto-install".
- Cindy warm gate: cba=0, post-run slot=NULL.
- **Falsifies "ROM auto-installs"**. Vega reverts to Phase 2a (`dbae274`).

### Step 1 — explicit write (`52ed8dc`, 2026-05-08 23:48)
- `ble_ip_core_init` writes `0x420B_000A` to fnGetClockCBs explicitly.
- Cindy gate: warm cba=21 PASS, cold-ish (3V3 power-cycle) cba=39 PASS.
- Strict pull-plug cold not yet tested (caveat: SRAM self-discharge >5 s).

### Step 3 — remove pin (`baf71de` retry → `78f0493` → `0665bc1`)
- Goal: drop link.x pin entirely, let LLD place symbol freely.
- `78f0493` (Vega): pin removed. The code review step (Vega + Lucy)
  **missed the orphan-section implication** — `.fnGetClockCBs` is no longer
  in any input section matched by the BSS rule, so LLD treats it as orphan
  and emits it as a loadable section in flash → 536 MB objcopy.
- Andelf called out the miss (`040fbf42`); Lucy owned the review failure
  (`c8168443`).
- `0665bc1` (Vega fix): adds `*(.fnGetClockCBs .fnGetClockCBs.*)` to BSS NOLOAD
  block. Build size restored.
- Cindy SRAM-scrub gate (Cindy 2026-05-09 00:05): cba=0, abc=0. Pre-scan
  `0x20001c78=0xDEADBEEF`; post-scan `0x200007c0=0x420B000A` (Step1 wrote
  symbol), `0x20001c78=0xDEADBEEF` (untouched).
- Conclusion at the time: **non-Rust consumer reads abs 0x20001c78 directly**
  (later refined to H4: NULL-tolerant abs). Iron Law #34 v5 fnGetClockCBs
  exception proposed; Iron Law #39 (proposed) "must pin" — both later
  refined/retired.

### Phase 2c — pin removal (`10d8b91`, 2026-05-09 ~09:30)
- Vega: link.x pin removed, Step 1 retained.
- frozen binary natural address: 0x20001c78 (coincides with historical pin).
- minimal binary natural address: 0x200007c0 (`_ebss=0x200007c4`).
- Cindy frozen warm sanity: cba=41 PASS (later confirmed valid despite
  ELF/bin worktree drift mishap).
- Cindy minimal warm: cba=0 FAIL (`0x20001c78=0x0085f913` SRAM garbage).

### Step B — discriminator (`d056863`, 2026-05-09 09:35)
- Vega: 4 KB BSS pad in `ble_tx_adv_ch37.rs` shifts fnGetClockCBs to
  0x20002c78. `_ebss=0x20002c7c` (covers 0x20001c78).
- Cindy three-condition gate (Iron Law #38):
  - warm 30s: cba=43, `0x20002c78=0x420B000A`, `0x20001c78=0x00000000`
  - WCH-Link 3V3 power-cycle 30s: cba=32, same
  - scrub `0xDEADBEEF`→reset 30s: cba=54, same
- All PASS. **Confirms H4 model**.

### double-write hedge (`c09d701`, 2026-05-09 ~09:40)
- Provisional: also write `0x420B_000A` to absolute `0x20001c78`.
- Discussion concluded: not needed for frozen path (single symbol-resolved
  write is sufficient because frozen layout puts the symbol at 0x20001c78
  anyway, and Step B PASS proved NULL-tolerance even when it doesn't).
- Kept in commit history as minimal-path hedge reference.

### Cleanup (executed 2026-05-09 morning, Andelf-approved)
- C1 (`d50907a`, Vega): drop `_PHASE2C_AXIS_PAD` + drop double-write + bb.rs
  wording cleanup. Regression binary `/tmp/phase2c_final/frozen.bin`
  MD5 `3e69b556`.
- C2 (`5e0330d`, Vega): types.rs 3-section doc rewrite (`PfnGetSysClock`,
  `BleClockConfig.get_clock_value`, Iron Law #35 module doc).
- C3 (Lucy, this commit): `lib-dependency-removal.md` Iron Law revisions +
  wording sweep + this postmortem document.
- Cindy terminal regression on `d50907a` frozen: warm cba=39, power-cycle
  cba=45, scrub-then-reset cba=52 — three-condition gate ALL PASS.

---

## Hypothesis ledger

| ID | Hypothesis | Status | What killed it |
|----|------------|--------|----------------|
| H1 | ROM auto-installs `0x420B_000A` at 0x20001c78 on cold boot | FALSIFIED 2026-05-08 | `baf71de` cold gate: post-run slot=NULL, cba=0 |
| H2a | ROM hard-binds abs 0x20001c78 AND requires non-NULL | FALSIFIED 2026-05-09 | Step B PASS with `0x20001c78=NULL` |
| H3 | Consumer reads via PCREL only (any symbol address works) | PARTIALLY FALSIFIED | Minimal FAIL: PCREL would read symbol@0x200007c0=valid → should PASS, but actually FAILed |
| H4 | ROM IRQ reads abs 0x20001c78 + NULL-tolerant fallback; userland reads PCREL | CURRENT | Explains every data point; surviving hypothesis |
| H39 (proposed) | fnGetClockCBs slot must be pinned at 0x20001c78 (hardware-fixed) | RETIRED 2026-05-09 | Step B PASS with symbol@0x20002c78 |

---

## Iron Law evolution

| Iron Law | Status | Change driver |
|----------|--------|---------------|
| #27 (`gBleIPPara[0]=0x60` at init) | unchanged | v7-probe pre-existing |
| #34 v5 (ROM is RAM-layout-agnostic for 6 BSS symbols) | revoked-and-restored | revoked 2026-05-09 00:05 with fnGetClockCBs exception; reaffirmed-without-exception 2026-05-09 09:48 after Step B PASS |
| #34 v5-final (this saga's wording) | new wording | ROM IRQ reads abs but NULL-tolerant; userland reads PCREL; frozen layout makes single symbol-resolved write sufficient |
| #35 (35a/35b split) | new | 35a: fnGetClockCBs returns raw HSE counter (used by .L7 → ip+0x1c); 35b: pfnTimerCBs returns 1600 Hz tick (used by TMOS_GetSystemClock via clockGetTickValve) |
| #38 (3-condition hardware gate) | new | response to v5 regression; required for any change touching BLE ABI words |
| #39 (proposed) | retired | Step B PASS rules out hardware-fixed-address constraint |

---

## Engineering lessons

1. **Hardware gates beat static analysis** — and when in doubt, **add another
   condition** (scrub > power-cycle > warm). Iron Law #38 codifies this.
2. **Distinguish layout coincidence from hardware contract** — the test is to
   *break the layout deliberately* (Step B pad). If hardware still works, the
   constraint was archaeology.
3. **Enumerate consumers explicitly** — when a value flows through multiple
   paths, list each consumer separately. The dual-path H4 model only became
   obvious once we wrote it down as two distinct read sites.
4. **Warm SRAM is a debugging adversary** — every hardware test must include a
   scrub condition. Without scrub, you cannot tell whether the system depends
   on persistence.
5. **Don't retire safety nets unilaterally** — pair "remove old mechanism"
   with "verify replacement is in place AND gates pass" in a single ordering.
   Step1 must precede Step3.
6. **Preserve every bad commit** — `bad/task56-v5-air-regression` (5528b5f)
   was the empirical anchor for the "static analysis ≠ semantic equivalence"
   lesson. Discarding it would have cost us the regression-test data.
7. **Iron Laws should be falsifiable** — when a candidate Iron Law is
   proposed, it must include the test that would refute it. Iron Law #39
   (proposed) was retired specifically because Step B was a designed-to-refute
   experiment.

---

## What's out of scope for #56 (follow-up tasks)

### Minimal binary FAIL
- Symptom: `cba=0`, `0x20001c78` holds SRAM garbage, ROM IRQ jalrs to garbage.
- Root cause: `_ebss=0x200007c4` doesn't cover 0x20001c78, so BSS clear leaves
  it untouched; warm SRAM persistence + Step1's symbol-resolved write to
  `0x200007c0` doesn't help the absolute consumer.
- Remediation candidates (not yet decided):
  - (a) Extend `_ebss` past 0x20001c78 via a dummy `.bss` symbol or linker
    PROVIDE.
  - (b) Add an explicit absolute write `write_volatile(0x20001c78 as *mut u32,
    0x420B_000A)` for minimal binary only (cargo feature gate).
  - (c) Resolve via the same mechanism as frozen but with a different layout
    strategy (e.g. don't have a "minimal" variant for production at all).
- Out of #56 scope per Vega-Lucy alignment (msg=20a0b102) and Andelf approval.

### B1 silicon-version gating
- `0x420B_000A` is ROM `clockGetHSEValue`'s address on CH32V208WBU6 B1 silicon.
- Other silicon revisions may relocate this fn.
- Recommendation: cargo feature gate (`ch32v208wbu6-b1`) controlling the
  constant; gate-fail at compile time if not set.

### Pull-plug strict cold boot
- `baf71de` and Step B used WCH-Link 3V3 power-cycle. Strict pull-plug cold
  (USB unplug + bench supply off) was deferred (Cindy noted SRAM self-discharge
  >5 s caveat).
- Worth one round of confirmation on the cleaned commit before we declare
  task #56 strictly done.

---

## Appendix: artifacts

- `notes/ch32-rs/c-ground-truth-fnGetClockCBs.md` — symbolic + empirical proof
  from libwchble.a (COMMON, PCREL_HI20, MEMHEAP-shift experiment)
- `notes/ch32-rs/tmos-timerinit-disasm.md` — fnGetClockCBs vs pfnTimerCBs ABI
  distinction; clockGetTickValve hardcoded `1600`
- `notes/ch32-rs/lib-dependency-removal.md` — full saga, Iron Laws #1–#38
- `notes/ch32-rs/task56-closure-draft.md` — code/doc diff plan executed in
  C1/C2/C3
- `/tmp/phase2c_d056863_clean/` — Step B binaries + ELFs (md5 `fdaf2354…`,
  `bb795da2…`)
- `/tmp/wlink_dump_phase2c_d056863_padded_*` — three-condition gate dump logs
- `/tmp/ble_scan_phase2c_d056863_padded_*` — three-condition gate scan logs
- `bad/task56-v5-air-regression` (`5528b5f`) — preserved cycle-CSR regression
  for future reference

