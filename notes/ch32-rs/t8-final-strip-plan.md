# T8 Final Strip Plan — `-lwchble` removal (Task #33)

**Status:** DRAFT (doc-only). Andelf approval gate before any patch.
**Author:** Lucy
**Date:** 2026-05-06 ~07:58
**Prerequisite:** Task #35 Phase D RF gate GREEN + nRF external verify (Cindy in flight on #41).
**Implementation owner (after approval):** Vega
**Cross-check / Iron-Law owner:** Lucy

---

## 0. Context

After #35 Phase C (commit `d57ed50`), `gBleLlPara` is the last large COMMON BSS migration. The remaining lib dependency is:

- One 4-byte COMMON BSS scalar: `fnGetClockCBs` (clock callback table base — never invoked in our path; safe at zero per Vega Phase A).
- Anchor-only externs (`BLE_IPCoreInit`, `LLE_IRQSubHandler`, `BB_IRQLibHandler`, `llAdvertise*`) whose lib bodies are ALREADY GC'd by `--gc-sections`; removing them is purely cosmetic.
- 4 size-neutral compensator pads in `.rodata` (`_T4_PAD`, `_T5_PAD`, `_T6_PAD`, `_T7_PAD`) totaling **29,028 B** that pad the binary back to T2 baseline 51,588 B. They become dead weight once `-lwchble` is gone.

T8 collapses all of the above in one shot, *under a hard re-gate*, and removes the static-library link line.

> **NOT in T8 scope:** further BLE feature work, register doc work, or any TX-path logic change. T8 is a *strip* operation, not a refactor.

---

## 1. Pre-flight gates (all must be GREEN before patching)

| Gate | Source | Required state |
|------|--------|----------------|
| G1 | #35 Phase D | 3×60s `cba ≥ 52`, `abc = 0` on `/tmp/ble_tx_adv_ch37_task35_phaseC_20260506.bin` |
| G2 | nRF external | Andelf manual capture confirms ADV channel 37 packets observed |
| G3 | #35 closed | Task moved to `done` |
| G4 | This doc approved | Andelf "可以开始 T8" signal in #ch32-rs or DM |

If any of G1-G4 is missing, **do not start T8**. There is no workaround.

---

## 2. Code changes

All paths relative to `/Users/mono/Elec/WCH/ch32-hal/examples/ch32v208/`.

### 2.1 `build.rs` — remove the lib link line

Delete L6-7:

```rust
// REMOVE:
println!("cargo:rustc-link-search=/Users/mono/Elec/WCH/CH32V20xEVT-2.31/EXAM/BLE/LIB");
println!("cargo:rustc-link-lib=static=wchble");
```

Replace the comment block at L5-11 with a one-line history note:

```rust
// T8 (2026-05-06): -lwchble removed. All lib COMMON BSS migrated to Rust strong
// symbols (#34 gBleIPPara, #35 gBleLlPara, T8 fnGetClockCBs). All anchored
// externs were already GC'd by --gc-sections; their decl block also removed.
```

Keep the inline linker-script verbatim (incl. `*(.tx_buf_aligned ...)` and `_T4_PAD`/`_T7_PAD` historical notes — those are LLVM/linker invariants we still rely on).

> **Linker-script rule of thumb:** the `.bss (NOLOAD) : ALIGN(4) { ... *(.sbss .sbss.* .bss .bss.*) }` wildcard already covers every `*.bss.<name>` we add, so no script edit is required for new section-isolated symbols.

### 2.2 `src/bin/ble_tx_adv_ch37.rs` — symbol surgery

#### (a) Add `fnGetClockCBs` Rust strong BSS stub (typed `Option<fn ptr>`, Andelf Q2 Option A)

Place IMMEDIATELY after `gBleIPPara` (around L205) so the COMMON-BSS-replacement symbols are co-located in source order. Vega Phase A confirmed: never read on our path; `None` (= null pointer, byte-equivalent to lib's `u32 = 0`) is safe.

C-side signature (forensic from `ble-bb-irq-handler-cross-check.md` + `rf-tx-abi.md`):
- `(*fnGetClockCBs)()` — single-deref function pointer
- Returns `u32` (clock tick / time-now)
- Lib reports size 4 B (single fn ptr, not array)
- Path C reaches it only via `gBleIPPara[0] bit6` set; on our path bit6 is always 0 → zero calls

```rust
// Phase D+1 T8 (2026-05-06): fnGetClockCBs → typed Rust strong BSS, last lib COMMON holdout.
// C-side: u32 fn-ptr `(*fnGetClockCBs)() -> u32` clock-tick callback. Vega Phase A:
// no read site on Path C ADV TX (gBleIPPara[0] bit6 never set → zero call).
// Stale-RAM hazard: NONE — `None` is byte-equivalent to lib `u32 = 0`.
// GlobalMerge isolation: #[link_section=".bss.fnGetClockCBs"] mandatory per
// retroactive Iron Law (any BSS migration MUST carry a unique link_section).
// Future scan/connect paths: replace with `Some(my_clock_fn)` returning tick-now.
// Andelf 2026-05-06 10:29 picked Option A (typed None, byte-equivalent to u32=0).
#[no_mangle]
#[link_section = ".bss.fnGetClockCBs"]
pub static mut fnGetClockCBs: Option<unsafe extern "C" fn() -> u32> = None; // 4B — byte-equivalent to lib `u32 = 0`
```

> **Note on type vs byte representation:** `Option<unsafe extern "C" fn() -> u32>` uses Rust's null-pointer optimization → `None` is the all-zero bit pattern, identical to the C-side `u32 = 0`. Self-check #1 (nm strong `B`) is unchanged; self-check #4 (TX_BUF mod16) is unchanged; ISR canary #5 is unchanged. The type is *purely documentation* at the binary level.

#### (b) Add defensive `#[link_section]` to `ble`

Current L142-143:

```rust
#[no_mangle]
pub static mut ble: [u32; 16] = [0; 16];
```

Replace with:

```rust
// T8 retroactive fix (2026-05-06): add link_section per the post-#35 Iron Law.
// `ble` was migrated in T2 before the GlobalMerge mechanism was understood.
// Without isolation, future BSS additions could fold `ble` into a MergedGlobals
// aggregate and shift TX_BUF / ISR cycle count. Defensive consistency only —
// no observed regression today; prevents latent breakage in future tasks.
#[no_mangle]
#[link_section = ".bss.ble"]
pub static mut ble: [u32; 16] = [0; 16]; // 64B (lib size confirmed)
```

#### (c) Delete the `extern "C"` block

Current L59-76 — entire block goes:

```rust
// REMOVE — all bodies were GC'd by T4-T7 anchor removal and --gc-sections.
extern "C" {
    fn BLE_IPCoreInit();
    fn LLE_IRQSubHandler();
    fn BB_IRQLibHandler();
    fn llAdvertiseCreateCore();
    fn llAdvertiseSet();
    fn llAdvertiseStart();
    fn llAdvTraverseallChannel();
}
```

Before removal, run a `Grep` for each symbol to verify NO live call sites:

```
Grep "BLE_IPCoreInit|LLE_IRQSubHandler|BB_IRQLibHandler|llAdvertiseCreateCore|llAdvertiseSet|llAdvertiseStart|llAdvTraverseallChannel" src/bin/ble_tx_adv_ch37.rs
```

Expected: 0 hits outside the extern block. If any hit appears, **STOP**, document, escalate — do not silently delete a live call.

(The comments in the historical `Phase D+1 T4/T5/T6/T7` blocks reference these names; those are *comments* and stay.)

#### (d) Delete all 5 compensator pads (Andelf 2026-05-06 08:13: "尽可能不留")

Remove ALL pads — `_T3_PAD` included per Andelf Q1 decision (target = clean baseline, zero pad pollution):

- `_T3_PAD` (4 B) — GlobalMerge align probe
- `_T4_PAD` (752 B)
- `_T5_PAD` (544 B)
- `_T6_PAD` (828 B)
- `_T7_PAD` (26904 B)

Total strip: **29,032 B**.

Replace the deleted block with a single history note:

```rust
// T8 (2026-05-06): _T3_PAD..._T7_PAD removed (29,032 B compensator total).
// They padded BIN back to 51,588 B while -lwchble was still linked but its
// bodies were GC-removed. With -lwchble gone, the deficit and the pad both
// disappear; BIN drops to T8 target. Andelf 2026-05-06: target = clean
// baseline, no pad residue. If alignment regresses (TX_BUF mod16 ≠ 0),
// fall back to §6 bisect-3a (re-add _T3_PAD ±4 B as the minimal probe).
```

### 2.3 `Cargo.toml` / `.cargo/config.toml`

No changes expected. Verify:
- No `[target.'cfg(...)']` block references `wchble`.
- No `rustflags` linker arg references `EXAM/BLE/LIB`.

```
Grep -r "wchble|EXAM/BLE/LIB" Cargo.toml .cargo/
```

If hits found, fold into the same patch.

---

## 3. Pre-implementation link-probe protocol

Before committing the patch, verify the link is *clean*. Two-step probe:

### 3.1 Cold link probe (with all changes applied)

```bash
cargo +nightly build -Z build-std=core --release --bin ble_tx_adv_ch37 \
  --target riscv32imac-unknown-none-elf 2>&1 | tee /tmp/t8-link-probe.log
```

Expected outcomes:
1. **GREEN:** Build succeeds. Proceed to §4.
2. **One unresolved symbol:** Investigate; add Rust strong stub if forensic-safe (Vega-style Phase A required).
3. **Multiple unresolved symbols:** STOP. The dependency surface is wider than this plan accounted for. Re-open Phase A scope.

### 3.2 nm cross-check

```bash
$RISCV_NM target/riscv32imac-unknown-none-elf/release/ble_tx_adv_ch37 \
  | grep -E ' [BD] ' \
  | grep -E 'gBleIPPara|gBleLlPara|gPaControl|dtmFlag|fnGetClockCBs|ble$|gptr(BB|LLE|AES|RFEND)Reg' \
  | sort
```

Expected: every symbol shows `B` (BSS) or `D` (data), strong, with addresses inside `_sbss .. _ebss`. If any symbol shows `C` (COMMON), the migration is incomplete.

### 3.3 BIN size capture

```bash
$RISCV_OBJCOPY -O binary target/.../ble_tx_adv_ch37 /tmp/t8-probe.bin
ls -l /tmp/t8-probe.bin
```

Capture the size; it is an input to §4.

---

## 4. Self-check matrix (must all PASS before flashing)

| # | Check | Expected | Tool |
|---|-------|----------|------|
| 1 | All migrated BSS symbols `B` strong | `gBleIPPara`, `gBleLlPara`, `ble`, `gPaControl`, `dtmFlag`, `fnGetClockCBs` all `B` | nm |
| 2 | All migrated symbols inside `_sbss.._ebss` | addr ≥ `_sbss` and < `_ebss` | nm |
| 3 | Each migrated symbol has its own `.bss.<name>` section | 6 distinct sections | objdump -h |
| 4 | TX_BUF address mod16 == 0 | `0x........0` | nm \| grep TX_BUF |
| 5 | ISR `__qingke_rt_BB` size | **≤ 262 B** (cycle-count parity with last green) | objdump -d |
| 6 | gBleIPPara[0] explicit init runs before BB-IRQ enable | source review | Grep |
| 7 | No COMMON (`C`) symbols remain in nm output | 0 matches | nm \| grep ' C ' |
| 8 | `wchble` not present in build artifacts | 0 hits | strings \| grep wchble |
| 9 | BIN size delta vs T7-baseline | **-29,032 B floor** (smaller acceptable if all canaries pass; pad-only arithmetic is the minimum). If TX_BUF mod16 ≠ 0, fall back to §6 bisect-3a re-add `_T3_PAD` 4 B. Retroactive note 2026-05-06 10:40: actual T8 impl `80defc9` measured -29,072 B (extra 40 B from Vega's bonus cleanup of 3 compile-time dead `extern` else-branch call sites — legitimate, all canaries passed) | objcopy + ls |
| 10 | No reference to deleted extern names in `.text` | 0 hits | objdump -d \| grep |

> **Iron Law #22 reminder:** ISR length is the canary. A drop below 262 B does *not* mean "smaller is better" — it means GlobalMerge folded a new symbol and the timing window has shifted. If check #5 shows < 262 B, treat it as a regression until proven otherwise.

---

## 5. RF final gate protocol

Triple-redundant. Cindy or whoever owns the RF gate runs all three.

| Gate | Form | Pass criterion |
|------|------|----------------|
| F1 | 3 × 60 s `cba` after fresh POR | every round `cba ≥ 52`, `abc = 0` |
| F2 | 1 × 5 min stable `cba` (continuous) | mean `cba ≥ 52`, no `abc` event, no UART stall |
| F3 | nRF Connect / sniffer external | ADV packets visible on channel 37, expected payload prefix |

Failure of *any* gate ⇒ revert to `bad/t8-attempt-N` branch (§6) and re-open T8.

---

## 6. Rollback / never-discard protocol

Per `feedback_never_discard_code`:

1. Before flashing the patch, push the WIP branch to `wip/t8-attempt-1`.
2. If any §4 or §5 gate fails:
   - `git branch bad/t8-attempt-1 HEAD` (preserve current state)
   - `git reset --hard <pre-T8 SHA>` (only after the bad branch is created and pushed)
   - Append a postmortem to `notes/ch32-rs/lib-dependency-removal.md` with: failed gate, raw data, hypothesised mechanism, next-attempt diff strategy.
3. Bisect order for re-attempt (smallest blast radius first):
   - 3a. Re-add `_T3_PAD` 4 B only (minimal align probe). If TX_BUF mod16 ≠ 0 was the failure, this is the cheapest rescue. (most likely first signal)
   - 3b. Restore `_T4_PAD..._T7_PAD` (re-add 29,028 B). If this rescues, the issue is BIN-size / vector / boot. (low probability)
   - 3c. Re-add `extern "C"` block + `_KEEP_*` anchors. If this rescues, a stale call site was hidden by the original anchor. (medium probability)
   - 3d. Re-link `-lwchble` (single line in build.rs). If this rescues, an undeclared lib symbol is being silently linked. (high-info)
   - 3e. Final: revert `fnGetClockCBs` Rust stub. If this rescues, Phase A forensic missed a write site → re-do forensic.

Each bisect step is a separate commit; never bundle rollback steps.

---

## 7. Iron-Law application checklist

For every Iron Law, state how T8 satisfies (or explicitly waives) it.

| Law | Statement (short) | T8 application |
|-----|-------------------|----------------|
| #22 | BSS layout shift breaks BLE timing | All 6 migrated BSS symbols carry `#[link_section=".bss.<name>"]`; ISR size canary in §4 #5. |
| #27 | Stale-RAM hidden dependency | Only `fnGetClockCBs` is new BSS; Vega Phase A confirms zero is safe. Other migrated symbols already passed #34/#35 gates. |
| #28 | LLVM branch-hoisting hazard | No control-flow change in T8; pure declaration/section/build edits. |
| #29 | Cargo target mismatch | `riscv32imac-unknown-none-elf` only; verify with `cargo metadata` and `.cargo/config.toml`. |
| #30 | UART accidental timing pad | No UART change in T8. |
| #31 | `_ebss` warm-reset hazard | All migrated symbols sit inside `_sbss.._ebss` (verified §4 #2); warm-reset clears them via startup zero-init. |
| Retro § | Any BSS migration MUST have unique `#[link_section]` | `fnGetClockCBs` complies; `ble` retro-fixed in §2.2(b). |

---

## 8. Sequencing / handoff

1. **Lucy:** This doc → Andelf review.
2. **Andelf:** Approve OR request changes (in `#ch32-rs` thread of #33 or in DM).
3. **Lucy:** Open task #33 thread, post the approved plan summary, claim #33 (after #35 closes).
4. **Vega:** Implement §2.1-2.3 in a single commit on a fresh branch `t8-strip-lwchble`. Push WIP. Run §3 link probe.
5. **Vega:** Post `nm`, BIN size, ISR disasm in #33 thread.
6. **Lucy:** Cross-check §4 self-check matrix from Vega's outputs.
7. **Cindy:** Run §5 RF gate (F1+F2+F3 sequence).
8. **Andelf:** External nRF verify.
9. **Lucy:** If all pass, merge, close #33, archive plan to `lib-dependency-removal.md` history.
10. **Failure path:** §6 rollback protocol.

Estimated wall-clock: 60-120 min for §3-§7 once approval lands. RF gate (§5) is the long pole.

---

## 9. Open questions / risks (for Andelf)

- **Q1 [RESOLVED 2026-05-06 08:13]:** Should `_T3_PAD` (4 B) also be removed in T8?
  - **Andelf decision: YES — "尽可能不留 _T3_PAD"** (target = clean baseline, zero pad residue).
  - Plan §2.2(d) updated to strip all 5 pads (total -29,032 B).
  - Fallback if TX_BUF mod16 regresses: §6 bisect-3a re-adds `_T3_PAD` ±4 B as the minimal align probe.
- **Q2 [RESOLVED 2026-05-06 10:29]:** Should `fnGetClockCBs` be a TODO stub or a real Rust impl?
  - **Andelf decision: Option A — typed `Option<unsafe extern "C" fn() -> u32> = None`** (byte-equivalent to lib `u32 = 0`, no behavior change vs current plan, type self-documents intent, future scan/connect path extends via `Some(...)` one line).
  - Plan §2.2(a) updated with the typed declaration and forensic recap.
- **Q3 [RESOLVED 2026-05-06 08:59]:** Andelf delegated to Lucy; **default ✅ stands** — `_KEEP_RUST_IP_CORE_INIT` not stripped in T8 (out of scope, removing risks Rust DCE on `ble_ip_core_init`).

### Post-T8 future direction (Andelf 2026-05-06 08:13)
> "路径清晰后,剥离 1:1 复制的 C 内存偏移逻辑,改状态变量"

Once `-lwchble` is gone and the TX path is clean, refactor the byte-offset writes (e.g. `gBleLlPara[N]`, `gBleIPPara[N]`) into typed Rust state variables / structs. Out of T8 scope; tracked separately (likely #37 register doc upstream or new task).

---

## 10. Sign-off block (filled at execution time)

```
Plan approved by:    @Andelf 2026-05-06 ~10:30 ("可以开始 T8")
Implemented by:      @Vega initial attempt 80defc9; final form @ f27c394 (attempt-15 boundary mode)
Link-probe (§3):     PASS  (cargo build green for all attempts 9-15)
Self-check (§4):     PASS  (attempt-15 binary BYTE-IDENTICAL to bisect-3g.bin = 22,556 B)
RF gate F1:          PASS  (3×60s, median cba=83, abc=0)
RF gate F2:          PASS  (5-min stable, cba=468 total, mean=93.6, abc=0, no UART stall)
RF gate F3:          PENDING (Andelf nRF Connect external verify — non-blocking)
#42 closed by:       @Vega 2026-05-06 17:18 (in_review → done, Andelf approved 17:16:24)
#33 closed by:       (pending plan doc + Iron Laws lock + #37 register doc commit)
```

---

## 11. T8 Implementation Forensic (post-execution, 2026-05-06)

The original plan (§2.2(a)) proposed a typed `Option<unsafe extern "C" fn() -> u32>` Rust strong stub for `fnGetClockCBs`, byte-equivalent to lib `u32 = 0`. Vega's initial implementation at `80defc9` followed that recipe and **failed RF gate** (cba=0). Attempts 9-15 explored the failure mode and converged on a fundamentally different approach (boundary mode, `_ebss = fnGetClockCBs = 0x20001c78`) that turned out to be byte-identical to the `bisect-3g` baseline.

### 11.1 Attempt timeline

| Attempt | SHA / branch | Approach | Result |
|---------|--------------|----------|--------|
| 9 | (Vega WIP) | Plan §2.2(a) verbatim — typed `Option<fn> = None` at `.bss.fnGetClockCBs` (zero-init by startup) | FAIL cba=0 |
| 10-13 | (Vega WIP) | Various GlobalMerge / link_section / address-pinning permutations to force `fnGetClockCBs` to land at `0x20001c78` while still being startup-zeroed | All FAIL cba=0 |
| 14 | `bad/t8-attempt-14-clock-fn` (`26d3f8e`) | User-supplied Rust fn `rust_get_chip_clock_hz` returning `hal::rcc::clocks().hclk.0 = 96_000_000`, written via `core::ptr::write_volatile` into `fnGetClockCBs` before `ble_hw_preamble()` | FAIL cba=0 — ROM accepts the write but BLE init fails. ROM `fnGetClockCBs` contract is **not** "return HCLK Hz" but a private ROM-only protocol. |
| 15 | `bad/t8-attempt-15-ebss-boundary` (`f27c394`) | **Boundary mode**: linker places `_ebss = address_of(fnGetClockCBs) = 0x20001c78`. Slot is OUTSIDE the qingke-rt startup zero-init range (`_sbss .. _ebss` exclusive), so its value at boot is whatever ROM/SRAM left there. ROM auto-manages. | PASS — F1 R1/R2/R3 (median cba=83), F2 5-min stable (cba=468 total, mean=93.6, abc=0, no UART stall). Binary BYTE-IDENTICAL to bisect-3g baseline (22,556 B). |

### 11.2 Why the original plan failed

The plan assumed `fnGetClockCBs = 0` (NULL) was safe because Vega Phase A confirmed "no read site on Path C ADV TX". That was true for the **lib path** (where the lib's startup also zeroed the slot, and the lib then wrote `0x420B000A` into it during BLE init). It was **not** true once `-lwchble` was stripped — the lib write disappeared, leaving the slot at NULL post-startup, and the ROM's BLE init then dispatched into NULL → fault → broken HSE fallback path → cba=0.

Phase A's "no read site" forensic missed the fact that the read site lives **inside ROM** (at `0x420B0000+`), not in any library or our Rust code. The static analysis surface was incomplete by construction; only hardware confirmed the gap.

### 11.3 ROM behavior model (definitive, T8 attempt-11..15)

Empirical truth table for the value at `0x20001c78` at the start of ROM-side BLE init:

| Boot value | ROM behavior | RF outcome |
|------------|--------------|------------|
| `0x00000000` (NULL — startup-zeroed) | Falls back to internal HSE path (broken on this MCU revision) | FAIL cba=0 |
| `0x420B000A` (ROM default fn ptr) | Dispatches directly into the ROM fn | PASS cba ≥ 52 |
| Non-zero, non-flash garbage (e.g. random SRAM `0xcbb5ae96`) | ROM detects "not flash range", auto-installs `0x420B000A`, then dispatches | PASS — same as ROM-default case |
| User-supplied valid Rust fn ptr returning `hclk_hz` (attempt-14) | ROM accepts the write, dispatches into the Rust fn — but the contract is private | FAIL cba=0 — Rust fn returning HCLK Hz is the wrong semantic |

Cold-boot SRAM signature note: SRAM cells come up with random transistor-determined bits at power-on, not zeros. A cold-boot dump showing e.g. `0xcbb5ae96` is a **valid** cold-boot signature; the test for "this is fresh SRAM" is "value is **not** `0x420B000A`", not "value is `0x00000000`".

### 11.4 0x420B0000 ROM region — permanent black box (triple-path probe)

Three independent probe paths, all 2026-05-06, all confirming the region is read-blocked:

**Path 1 — Debugger halt + wlink dump (SBA backdoor, Cindy)**:
- `wlink halt; wlink dump 0x420B0000 256` → all zeros.
- `wlink halt; wlink dump 0x420B0000 4096` → all zeros.
- `wlink halt; wlink dump 0x420A0000 256` / `0x420C0000` / `0x42100000` → all zeros / non-readable.

**Path 2 — CPU running-mode `lw` via main bus, BLE clock OFF** (Andelf 17:26 proposed, Vega built `probe/rom-running-read` branch `dd7d567` BIN `/tmp/ble_tx_adv_ch37_probe_rom_running_read.bin` 7,644 B; Cindy flashed + RTT capture 17:48):
- `core::ptr::read_volatile` on `0x420B0000..0x420B00FC` (64 words) → all `0x00000000`.
- Spot-sample `0x420B0008 / 0x420B000C / 0x420B0010` (fnGetClockCBs body neighborhood) → all `0x00000000`.
- Probe reached `=== ROM probe end ===` and entered idle `loop {}` → **no hardfault, no load access fault**.
- Logs preserved: `/tmp/sdi_probe_rom_running_read_20260506_1748_rom_running_read.log`, `/tmp/wlink_probe_rom_running_read_20260506_1748_rom_running_read.log`.

**Path 3 — CPU running-mode `lw` via main bus, BLE clock ON** (Andelf 18:13 hypothesis: "是不是需要启用 BLE 外设先才能读到？"; Vega built `probe/rom-clock-enabled` branch `bf8f661` BIN `/tmp/ble_tx_adv_ch37_probe_rom_clock_enabled.bin` 7,792 B = +148 B vs v1 for the `ble_hw_preamble()` body; Cindy flashed + RTT capture 18:55):
- Identical probe loop as Path 2, but with `unsafe { hal::ble::ble_hw_preamble(); }` called **before** the loop. `ble_hw_preamble` performs:
  - HSE calibration + HSE on
  - `RCC_AHBPCENR @ 0x4002_1014 |= 0x00030040` (bits 6/16/17 = CRC/BLEC/BLES — full BLE peripheral clock enable)
  - LSI on
  - It does **not** dispatch into the ROM (no `jalr 0x420B000A`), keeping BLE init fully isolated from the probe.
- Result: `0x420B0000..0x420B00FC` (64 words) → all `0x00000000`. Spot-sample `0x420B0008 / 0x420B000C / 0x420B0010` → all `0x00000000`. Reached `=== ROM probe v2 end ===`, no hardfault.
- Logs preserved: `/tmp/sdi_probe_rom_clock_enabled_20260506_1855_rom_clock_enabled.log`, `/tmp/wlink_probe_rom_clock_enabled_20260506_1855_rom_clock_enabled.log`.

**Conclusion**: all three independent probe paths agree — `0x420B0000..` is **execute-only memory** via a static soft-block bus-matrix policy:
- Instruction fetch (`jalr` to `0x420B000A`) is **allowed** — BLE init successfully dispatches into the ROM fn and works (cba ≥ 52 in attempt-15).
- Data load (`lw` from any address) is **silently returned as zero**, on the debugger SBA path, on the CPU main-bus path with BLE clock disabled, and on the CPU main-bus path with BLE clock fully enabled. **Symmetric across clock state** → the soft-block is a static bus-matrix property, not a runtime clock-gated feature.
- The bus matrix does NOT raise an access fault for data reads — it returns a zero word. Softer than hardfault but functionally identical for us (no contents readable).

The CH32V208 reference manual marks this address space as "reserved" with no further description. All three plausible read paths now confirmed closed → **permanent black box**, no path to disassemble or reverse-engineer the ROM's `fnGetClockCBs` body.

Direction: **never** write a non-conformant fn (e.g. one returning Hz) to `fnGetClockCBs`. The boundary mode (Path A in Iron Law #35) lets ROM auto-manage.

The `probe/rom-running-read` (`dd7d567`) and `probe/rom-clock-enabled` (`bf8f661`) branches are preserved as evidence — never-discard policy, do not delete.

---

## 12. Iron Laws (locked T8 close, 2026-05-06)

These laws are derived directly from §11 forensic and the bisect-3g baseline match. They are now **invariants** for the CH32V208 BLE example — any future BSS / linker-script edit must preserve them or the BLE TX path breaks silently.

### Iron Law #34 — `_ebss` boundary at `fnGetClockCBs`

**Statement**: The linker symbol `_ebss` must be placed AT `address_of(fnGetClockCBs) = 0x20001c78`, before any allocation of the `fnGetClockCBs` slot.

**Rationale**: The qingke-rt startup BSS-zero loop zeroes `[_sbss, _ebss)` exclusive. Placing `_ebss` AT the `fnGetClockCBs` address ensures the slot is **excluded** from zero-init, preserving whatever value (ROM-default `0x420B000A` after warm reset, or random SRAM at cold boot) the slot holds at boot. ROM then auto-manages it correctly per §11.3.

**Linker recipe** (`build.rs` link.x, current): see `examples/ch32v208/build.rs`:

```ld
.bss (NOLOAD) : ALIGN(4) {
    PROVIDE(_sbss = .);
    /* … other BSS … */
    . = 0x20001c78;
    PROVIDE(_ebss = .);              /* boundary BEFORE fnGetClockCBs */
    KEEP(*(.fnGetClockCBs));         /* fnGetClockCBs at 0x20001c78, NOT startup-zeroed */
} >RAM
```

**Verification**: `nm` shows `_ebss = 0x20001c78` and `fnGetClockCBs = 0x20001c78`. Binary byte-identical to bisect-3g.bin baseline (22,556 B).

### Iron Law #35 — `fnGetClockCBs` must conform to the `pfnGetSysClock` contract (RTC tick counter, LSI/2 ≈ 16 KHz, 32-bit wrap) — never write Hz

**Statement (revised 2026-05-06 after wchble.h discovery)**: Any value at `0x20001c78` that is dispatched by the BLE controller ROM as `fnGetClockCBs` must satisfy the documented `pfnGetSysClock` contract from `wchble.h` L77:

```c
typedef uint32_t (*pfnGetSysClock)(void);   // returns RTC tick counter
```

Reference C usage (`HAL/RTC.c` L80-96): `conf.getClockValue = RTC_GetCounter` — the function returns the current 32-bit RTC counter sampled from a clock source running at `CAB_LSIFQ / 2` (the LSI/2 timing clock, approximately **16 KHz**, with 32-bit wrap). This is **not** a system clock frequency in Hz.

**The two safe paths** (T8 attempt-15 = path A):
- **Path A — Boundary mode (preferred, used in attempt-15)**: Place `_ebss = address_of(fnGetClockCBs) = 0x20001c78` so qingke-rt startup BSS-zero leaves the slot uninitialized. On warm reboot the slot retains the ROM-default `0x420B000A`; on cold boot the slot holds random SRAM bits, and the ROM auto-detects "not flash range" and re-installs `0x420B000A` itself. Rust does not write the slot. The ROM-side fn at `0x420B000A` implements the `pfnGetSysClock` contract internally.
- **Path B — Conforming Rust fn (untested at T8 close)**: Install a Rust strong symbol whose fn body returns the RTC counter — e.g. `RTC->CNTL` 32-bit read or a HAL wrapper that mirrors the C `RTC_GetCounter` semantic. This path was **NOT** validated as part of T8; attempt it only with full F1/F2/F3 RF gate coverage.

**The forbidden path** (attempt-14 falsified):
- Writing a fn that returns HCLK in Hz (e.g. `hal::rcc::clocks().hclk.0 = 96_000_000`) violates the contract — the ROM expects a counter, not a frequency. Result: cba=0, BLE FAIL.

**Rationale — why we can't easily ship Path B at T8 close**:
- §11.4 dual-path ROM probe (debugger SBA + CPU running-mode `probe/rom-running-read` `dd7d567`) confirms `0x420B0000..` reads as all zeros under both paths; the bus matrix soft-blocks data loads (silent zero return). Instruction fetch is allowed. Therefore the ROM's internal `pfnGetSysClock` body cannot be disassembled to verify our Rust mirror against it.
- §11.3 attempt-14 demonstrated that returning a wrong value (Hz) fails silently — no compile-time, link-time, or static check catches it. Only RF gate F1/F2/F3 per attempt validates conformance.
- Path A bypasses this risk entirely by deferring to the ROM's own `pfnGetSysClock` implementation, which by construction conforms to the contract the ROM expects.

**Verification (Path A enforcement)**: `Grep "fnGetClockCBs" src/` should show only the strong-symbol declaration (placed in the `.fnGetClockCBs` section by `#[link_section = ".fnGetClockCBs"]`) and **no** `write_volatile(0x20001c78, …)` or `fnGetClockCBs = …` assignment sites. The Rust storage symbol exists only to satisfy the linker; its initial value never reaches the slot at runtime (the slot is outside `_ebss` and is not zeroed).

**Future direction**: If a feature requires a Rust-controlled clock callback (e.g. for LSI calibration tracking), the proper path is to mirror `bleClockConfig_t` (wchble.h L112-119) and pass it through whatever ROM-blessed init API exists (TBD — currently the lib uses `TMOS_TimerInit(&conf)` per `HAL/RTC.c`). Task #43 lays the groundwork by adding the `BleClockConfig` Rust mirror.

### Iron Law #36 — `_ebss > 0x20001c78` is forbidden

**Statement**: Any BSS migration that pushes `_ebss` past `0x20001c78` is a regression. The BSS-zero loop would then either zero the `fnGetClockCBs` slot (NULL → broken HSE fallback) or shift the slot off its ROM-pinned address (ROM dispatches into wrong memory).

**Rationale**: The ROM has a hardcoded address `0x20001c78` for `fnGetClockCBs`. Any future migration adding more BSS (e.g. extra structs, more lib COMMON replacements) must allocate space BEFORE `0x20001c78` in the link script, not after. The `KEEP(*(.fnGetClockCBs))` placement at exactly `_ebss` is non-negotiable.

**Self-check**: `nm | grep _ebss` must show `0x20001c78`. Any `_ebss > 0x20001c78` is an immediate STOP / re-check.

### Cross-references to existing Iron Laws

- **Iron Law #22** (ISR length canary `__qingke_rt_BB ≤ 262 B`) — still active. T8 attempt-15 verified.
- **Iron Law #23** (TX_BUF mod16 == 0 hardware DMA strict requirement) — still active. T8 attempt-15 verified via `.tx_buf_aligned` section + `ALIGN(16)`.
- **Iron Law #27** (LLE+0x08 = 0x8000 PHY pre-arm — see ble-registers.md 2026-05-05) — orthogonal to T8.
- **Iron Law #31** (`_ebss` warm-reset hazard) — partially superseded by #34/#36. The original #31 phrased the hazard generically; #34/#36 nail down the exact address.

---

## 13. T8 close marker (2026-05-06)

- ✅ `-lwchble` removed — `examples/ch32v208/build.rs` no longer emits the static-library link flags. Confirmed in current source (lines 14-15, commented out with T8 note).
- ✅ All required lib COMMON BSS migrated:
  - `gBleIPPara` @ `0x20000758` (#34, Phase D+1).
  - `gBleLlPara` / `ble` @ `0x20001858` (#35, Phase D+1).
  - `fnGetClockCBs` @ `0x20001c78` (T8 boundary mode).
- ✅ All 5 compensator pads (`_T3..._T7_PAD`) deleted (-29,072 B vs T7 baseline; extra 40 B from Vega's bonus cleanup of 3 dead `extern` else-branch call sites).
- ✅ `extern "C"` block deleted (anchor names already GC'd by `--gc-sections`).
- ✅ Self-check matrix (§4) PASS for attempt-15 (`f27c394`).
- ✅ RF gates F1 + F2 PASS.
- ✅ Binary byte-identical to bisect-3g.bin baseline (22,556 B).
- ✅ ROM black-box probe complete via dual-path (Cindy 2026-05-06): debugger SBA `wlink halt + dump` + CPU running-mode `lw` (`probe/rom-running-read` `dd7d567`) both return all zeros. Region is execute-only (jalr allowed, lw soft-blocked).
- ✅ #42 closed (Vega 2026-05-06 17:18, Andelf approved 17:16:24).
- ✅ F3 Andelf nRF Connect external verify — PASS (2026-05-06 18:45 DM `f20f8ea6`): nRF Connect + CoreBluetooth dual-端 visible on `f27c394`.
- ✅ Felix final dry-run gate locked at `f27c394` (50/0/0 PASS, 17:44).
- ❌ Fast-forward merge to `main` — **CANCELLED** (Andelf 2026-05-06 18:45 DM `f20f8ea6`): "main 先不回归，先继续在我们的 ble 分支前进". T8 final state ships on the `feat/ble-phy-init` ble feature branch, not `main`.
- ⏳ T8 doc commit (this file + `notes/ch32-rs/ble-registers.md`) onto `feat/ble-phy-init` after ff from `bad/t8-attempt-15-ebss-boundary` (10 T8 commits added linearly, no content loss; intermediate `t8-attempt-*` branches preserved as never-discard refs).
- ✅ Probe v2 (BLE clock-enabled ROM read at `0x420B0000..`) per Andelf 18:13 hypothesis — Vega built `probe/rom-clock-enabled` (`bf8f661`, BIN +148 B vs v1 for `ble_hw_preamble()` body), Cindy flashed + RTT capture 18:55: all `0x00000000`, no hardfault. Result: clock-on / clock-off symmetric → soft-block is a static bus-matrix property; Iron Law #35 dual-path rationale upgraded to triple-path. See §11.4.
- ⏳ Close #33 umbrella after this doc + Iron Laws #34/#35/#36 + #37 register doc commit (this commit closes the loop on the doc side).
