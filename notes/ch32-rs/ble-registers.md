# CH32V208 BLE Peripheral Registers — Living Document

**Status**: Active living document. Last update 2026-05-06 (T8 close: ROM region + ROM-pinned SRAM contracts added).
**Owner**: Lucy (task #37)
**Sources**: `ch32-hal/src/ble/{bb,lle,rfend,regint}.rs` + libwchble.a V1.40 disassembly + live hardware dump + #34 forensic probe results + T8 attempt-9..15 forensic

## Recent Findings

- **2026-05-06 (T8 close)**: `libwchble.a` fully stripped (`-lwchble` removed from `build.rs`). All required lib COMMON BSS migrated to Rust strong symbols at ROM-pinned RAM addresses: `gBleIPPara` @ `0x20000758` (#34), `gBleLlPara`/`ble` @ `0x20001858` (#35), `fnGetClockCBs` @ `0x20001c78` (T8). T8 attempt-15 (`f27c394`) byte-identical to bisect-3g baseline (22,556 B), F1 + F2 + F3 (Andelf nRF Connect + CoreBluetooth) all PASS. New facts documented this update:
  - **0x420B0000 ROM region** confirmed opaque **execute-only** via triple-path probe (2026-05-06):
    - *Path 1*: `wlink halt + dump 0x420B0000 (256B + 4 KB) + boundaries (0x420A/0x420C/0x4210)` all return 0/X (debugger SBA backdoor).
    - *Path 2*: CPU running-mode `core::ptr::read_volatile` with **BLE clock OFF**, branch `probe/rom-running-read` (`dd7d567`, 7,644 B) — returns `0x00000000` for `0x420B0000..0x420B00FC` and spot samples `0x420B0008/000C/0010` — **no hardfault**, soft-block silent-zero return on the main bus.
    - *Path 3*: CPU running-mode `core::ptr::read_volatile` with **BLE clock ON** (`ble_hw_preamble()` called before the probe loop = HSE on + `RCC_AHBPCENR @ 0x4002_1014 |= 0x00030040` BLE bits + LSI on, NO ROM dispatch), branch `probe/rom-clock-enabled` (`bf8f661`, 7,792 B = +148 B for the preamble body) — same result: all `0x00000000`, no hardfault. Logs: `/tmp/sdi_probe_rom_clock_enabled_20260506_1855_rom_clock_enabled.log`, `/tmp/wlink_probe_rom_clock_enabled_20260506_1855_rom_clock_enabled.log`.
    - **Conclusion**: clock-on / clock-off symmetric → soft-block is a **static bus-matrix property**, not a runtime clock-gated feature. Instruction fetch (`jalr 0x420B000A`) is allowed (BLE init works); data load is uniformly read-as-zero on all three paths. Houses the BLE controller default `fnGetClockCBs` fn at `0x420B000A`. See new **Block 5** below.
  - **`fnGetClockCBs @ 0x20001c78` ROM behavior model** (definitive, T8 attempts 11-15 forensic):
    - `= 0x00000000 (NULL)` → ROM falls back to broken HSE path → cba=0 (FAIL).
    - `= 0x420B000A (ROM default)` → ROM calls directly → BLE clock OK (PASS).
    - `= non-zero non-flash garbage` → ROM auto-installs `0x420B000A` → PASS (this is the cold/random-SRAM case).
    - `= user-supplied valid Rust fn ptr returning hclk Hz` → ROM accepts the write but BLE FAILS (cba=0). The ROM contract is **not** "return HCLK Hz" — `wchble.h` L77 documents `pfnGetSysClock` as returning a 32-bit RTC tick counter (LSI/2 ≈ 16 KHz, 32-bit wrap), per `HAL/RTC.c` reference: `conf.getClockValue = RTC_GetCounter`.
  - **Iron Law candidates locked**: #34 boundary `_ebss = fnGetClockCBs = 0x20001c78`; #35 `fnGetClockCBs` must conform to `pfnGetSysClock` contract — Path A (boundary mode, current attempt-15) Rust does not write the slot, ROM auto-installs its conforming fn; Path B (Rust fn returning RTC counter) untested; **forbidden**: writing a Rust fn returning Hz; #36 forbid `_ebss > 0x20001c78`. See `t8-final-strip-plan.md` §Iron Laws.
  - **Public BLE library types found in wchble.h** (Vega 18:25 discovery): `pfnGetSysClock` typedef + `bleClockConfig_t` (sizeof=16: getClockValue + ClockMaxCount=u32 + ClockFrequency=u16 + ClockAccuracy=u16 + irqEnable=u8) + `bleConfig_t` (~30 fields incl. 7 fn-ptr CBs). Triggered task #43 (BLE Rust types align with wchble.h C types) — see Public BLE Library Types section below + cross-ref `t8-final-strip-plan.md` §13.
- **2026-05-05**: `LLE+0x08 = 0x8000` re-classified from "active-scan TX-mode" to **"PHY pre-arm for ADV TX"**. Required explicit write in `ble_ip_core_init` for Path C. probe-A confirmation: forced `gBleIPPara[0]=0x60` (triggers ISR `.L6` bit5 → `LLE+0x08=0x8000` write) + `ip20=0` → cba=39 R1 ✓. T7 baseline pass had this write happening serendipitously via stale-RAM-triggered ISR scan-mode. See `lib-dependency-removal.md` #34 forensic + Iron Law #27.

## Confidence Legend

| Tag | Meaning |
|-----|---------|
| ✅ | Asm-confirmed (libwchble.a disasm) **and** hardware-validated (live dump or Cindy gate) |
| 🔍 | Inferred from code/asm but not yet hardware-validated |
| ❓ | Unknown — register is touched but bit-level intent is undecoded |

## Address Map (top level)

| Block | Base | WCH name | Crate name | Role |
|-------|------|----------|------------|------|
| BB | `0x40024100` | `gptrBBReg` *(see naming note)* | `BB_BASE` (bb.rs), `BB_BASE_REG` (regint.rs) | Baseband CTRL/GO/MODE/CFG/IRQ STATUS |
| LLE | `0x40024200` | `gptrLLEReg` | `LLE_BASE` (lle.rs, regint.rs) | Link-layer TIMING/STATE/DMA buf/timer/IRQ |
| RFEND | `0x40025000` | `gptrRFENDReg` | `RFEND_BASE` (rfend.rs), `RFEND_CAL_BASE` (regint.rs) | RF analog (PLL/VCO/filter/bias) + calibration tables |
| AES | `0x40024300` | `gptrAESReg` | `WCH_AESR` (bb.rs ISR) | AES engine — only +0x04 cleanup observed |

### Naming caveat — gptrBBReg ↔ gptrLLEReg

`bb.rs::bb_irq_lib_handler` carries a comment claiming WCH's `gptrBBReg` is at `0x40024100` but its **role** is timer/IRQ/DMA (= what we call lle_*), and `gptrLLEReg` at `0x40024200` is CTRL/GO (= what we call bb_*). Reality from `bb_dev_init` / `lle_dev_init` actually uses each base for both ranges of features:

- `0x40024100` writes both CTRL/GO bits at `+0x00` **and** IRQ STATUS at `+0x38` (bb.rs is the only writer of either).
- `0x40024200` writes timing/state/timer/DMA buf (lle.rs).

The "swapped naming" warning is best read as: WCH header names confused, but **we always reference by physical address**, so the comment is informational only. If someone refactors and tries to rename, do NOT trust the WCH header — read the asm.

---

## Block 1 — BB @ `0x40024100` (gptrBBReg)

Used by: `bb.rs::bb_dev_init`, `bb.rs::bb_irq_lib_handler`, `regint.rs::ble_reg_init` (pre/post analog gate via `bb_rmw`).

| Offset | Width | Bits / Value | Role | Conf |
|--------|-------|--------------|------|------|
| `+0x00` | u32 | bit12 | BB CTRL — analog gate (BLE_RegInit pre-init: `clear[13:12], set bit12`) | ✅ asm L71150 |
| | | bits[8:7] | BB CTRL — analog clears (pre/post-init: `clear[8:7]`, post sets bit7) | ✅ asm L71151,71177 |
| | | bit8 | BB CTRL — analog enable pre-init (`set bit8`) | ✅ asm L71152 |
| | | bit23 | BB CTRL — baseband enable (permanent) | ✅ asm L69623 (lui 0x800) |
| | | bit28 | BB CTRL — hardware enable (permanent, 0x10000000) | ✅ asm |
| `+0x20` | u32 | `0x00090083` | BB MODE register | ✅ asm (lui 0x90; addi 131) |
| `+0x2C` | u32 | `0x80010EC8 \| (rf_flag<<25)` | BB CFG — bits[30:25]=rf_flag, base 0x80010EC8 | ✅ asm + hw dump (`0x92010EC8` for rf_flag=0x09) |
| | | bits[30:25] | PHY mode bits (BB_RF_FLAG_1M=0x09: bit0=1M PHY, bit3=?) | 🔍 hw value confirmed; bit3 meaning TBD |
| `+0x34` | u32 | `0x000001D0` (464) | BB TIMING register | ✅ asm |
| `+0x38` | u32 | bit4 | IRQ STATUS — `.L4` cleanup; sets `gBleIPPara[4]=1` (re-arm) | ✅ asm L91127 |
| | | bit5 | IRQ STATUS — paired with bit6 in W1C mask `0x60` (PLL-related) | ✅ asm |
| | | bit6 | **PLL-ready** — triggers `.L6` TX advance path; W1C with bit5 (`0x60`) | ✅ asm + hw |
| | | bit7 | IRQ STATUS — `.L8` cleanup; sets `gBleIPPara[4]=1` (re-arm) | ✅ asm |
| | | (other) | Other IRQ STATUS bits — unknown (not touched by any decoded path) | ❓ |

**Notes**:
- `+0x38` is **W1C** (write-1-to-clear). Reading `+0x38` returns live status; writing `0x60` clears bits 5+6, etc.
- bit23 / bit28 of `+0x00` are NOT self-clearing strobes — WCH leaves them set permanently. (Earlier code wrongly treated bit23 as a strobe and inserted a `println!` between bit23 and bit28 writes, which appeared to "clear" bit23 — pure timing artifact.)
- IRQ status bits 5 / 6 / 7 in `bb_irq_lib_handler` map to `gBleIPPara[4]` flags `0x40` / armed-bit clear / re-arm.

---

## Block 2 — LLE @ `0x40024200` (gptrLLEReg)

Used by: `lle.rs::lle_dev_init` (timing init), `lle.rs::lle_read_state/lle_clear_irq`, `regint.rs::ble_reg_init` (settle timer + countdown), `bb.rs::bb_irq_lib_handler` (TX advance writes via `WCH_LLER`).

| Offset | Width | Bits / Value | Role | Conf |
|--------|-------|--------------|------|------|
| `+0x08` | u32 | (read) | IRQ STATUS — live event bits | ✅ asm + hw |
| | | (write) | IRQ STATUS W1C — clear pending bits | ✅ |
| | | `0x8000` | **PHY pre-arm for ADV TX** (was annotated "active-scan TX-mode write" — incorrect; see note below). REQUIRED before first TX fire. | ✅ asm + 2026-05-05 probe-A confirmation |
| | | `0x2000` | PHY state advance `0x33 → 0x37` (TX fire) — written in `.L6` TX advance | ✅ asm + hw |
| | | `0xFFFF` | Initial clear-all-IRQ in `lle_dev_init` (W1C all bits) | ✅ asm |
| `+0x0C` | u32 | `0xF00F` | IRQ MASK — bits[15:12] + bits[3:0] | ✅ asm + hw |
| | | (any) | BLE_RegInit saves and zeros this for the duration of calibration, then restores | ✅ asm L71134-71136 |
| `+0x14` | u32 | `140` | TIMING0 | ✅ asm + hw dump |
| `+0x1C` | u32 | `93` | STATE_MACHINE — `ConnRxWait` (no connection, RX wait) | ✅ asm |
| | | `97` | STATE_MACHINE — `ConnTxPrep` | ✅ asm |
| | | `101` | STATE_MACHINE — `ConnAckWait` | ✅ asm |
| | | `105` | STATE_MACHINE — `ConnEventClosing` | ✅ asm |
| | | `107` | STATE_MACHINE — `SleepPrep` | ✅ asm |
| | | `108` | STATE_MACHINE — `Sleep` (default after `lle_dev_init`) | ✅ asm + hw |
| `+0x24` | u32 | `140` | TIMING2 | ✅ asm + hw |
| `+0x2C` | u32 | `60`  | TIMING3 | ✅ asm + hw |
| `+0x34` | u32 | `140` | TIMING4 | ✅ asm + hw |
| `+0x3C` | u32 | `60`  | TIMING5 | ✅ asm + hw |
| `+0x44` | u32 | `140` | TIMING6 | ✅ asm + hw |
| `+0x4C` | u32 | `108` | TIMING7 | ✅ asm + hw |
| `+0x50` | u32 | `93` (during cal) / `0` (post) | Settle timer used during BLE_RegInit | ✅ asm L71168, L71190 |
| `+0x64` | u32 | `6000` | Countdown timer for `RFEND_WaitTune` (decrements during BLE events) | ✅ asm |
| | | `gBleIPPara[16..19]` | Written in `.L6` TX advance — = 776 for ADV TX | ✅ asm |
| `+0x6c` | u32 | `gBleIPPara[20..23] << 1` | Active-scan offset (set in `.L6` bit5 path) | ✅ asm |
| `+0x74` | u32 | `&LLE_DMA_BUF` | DMA buffer base (= `gBleIPPara[36]` MEMAddr) | ✅ asm + LLE-state-machine empirical |

**Notes**:
- `+0x08` is split-semantic: reads return live IRQ status; writes are W1C. Don't interpret a read value as ACCESS_ADDR.
- **`+0x08 = 0x8000` PHY pre-arm — re-evaluated 2026-05-05** (#34 forensic): was previously labeled "active-scan TX-mode write" because lib only wrote it in `bb_irq_lib_handler` `.L6` bit5 path (the scan-mode branch). probe-A (force `gBleIPPara[0]=0x60` + `ip20=0`) recovered cba=39 from cba=0 → confirms the **`0x8000` write is the missing essential pre-arm step for ADV TX**, not specifically a scan-mode artifact. Path C must explicitly write `LLE+0x08 = 0x8000` once in `ble_ip_core_init` (or pre-GO sequence) — independent of `gBleIPPara[0]` ISR-trigger gymnastics. **`LLE+0x6c` is NOT essential** (probe-A had `ip20=0` → `+0x6c=0` and still TX-decodable).
- `+0x64` does not decrement in standalone calibration (no active BLE events). In RFEND_WaitTune the SW polls bit26/bit25 of RFEND+0x90 instead, and `+0x64==0` is only a safety-net exit.
- Without a non-zero `+0x74` the LLE state machine refuses to fire any trigger → blocks RFEND PLL calibration. Confirmed empirically 2026-05-01.

---

## Block 3 — RFEND @ `0x40025000` (gptrRFENDReg)

Used by: `rfend.rs::rfend_dev_init/rfend_reset` (analog config), `regint.rs::*` (calibration trigger + result tables).

⚠️ **Historical bug**: This block was originally coded as `0x40024300` (= AES block) — see `rfend.rs` header. Fixed 2026-05-01 after `ip.o` BLE_IPCoreInit reloc decode confirmed `gptrRFENDReg = 0x40025000`.

### 3a. Analog initialization (rfend.rs)

| Offset | Bits | Action | Role | Conf |
|--------|------|--------|------|------|
| `+0x0C` | u32 | `0x1101` → `0` → `0x1101` (100 nops between) | CTRL reset sequence (sets bits 12+8+0) | ✅ hw + asm |
| `+0x28` | u32 | `= 0x480` | CFG0 (default) | ✅ asm + hw |
| `+0x2C` | bits[22:20] set | RMW set | PLL/VCO config #1 | ✅ asm L75348 (B1 fix) |
| | bits[26:24] clr | RMW clear | PLL/VCO config #2 | ✅ asm |
| | bits[13:12] set bit13 | RMW | PLL/VCO config #3 | ✅ asm |
| | bits[17:16] set bit17 | RMW | PLL/VCO config #4 | ✅ asm |
| `+0x30` | bits[3:0] clr | RMW clear | Loop filter #1 | ✅ asm L75368 (B2 fix) |
| | bits[7:4] clr | RMW clear | Loop filter #2 | ✅ asm |
| | bits[10:8] clr | RMW clear | Loop filter #3 | ✅ asm |
| | bits[22:20] set | RMW set | Loop filter #4 | ✅ asm |
| | bits[30:28] = `0b101` | RMW set | Loop filter mode | ✅ asm |
| `+0x38` | bits[15:12] = `8` | RMW | CFG5 #1 | ✅ asm |
| | bits[26:24] set bit25 | RMW | CFG5 #2 | ✅ asm |
| | bit30=1, bit31=0 | RMW | CFG5 PLL select (A5 fix: was bits[30:29]=`11`) | ✅ asm |
| | bits[25:19] clr, bit20 set | RMW | PLL enable (B3 fix: bit-mask was wrong before) | ✅ asm L75385 |
| | bit31 = 1 | RMW set | PLL enable strobe | ✅ asm |
| `+0x48` | bits[30:28] set bit29 | RMW | RF0 #1 | ✅ asm |
| | bits[26:24] = `0b100` | RMW | RF0 #2 | ✅ asm |
| | bits[3:0] = `9` | RMW | RF0 #3 | ✅ asm |
| | bits[18:16] = `0` | RMW (A1 fix: clear mask `0x7000` not `0xF000` — bit19 must NOT be cleared) | RF0 #4 | ✅ asm L75274 |
| | bit31 = 1 | RMW set | RF0 enable | ✅ asm |
| `+0x4C` | bits[2:0] = `3` | RMW | RF1 bias #1 | ✅ asm |
| | bits[6:4] = `3` | RMW | RF1 bias #2 | ✅ asm |
| | bits[10:8] = `3` | RMW | RF1 bias #3 | ✅ asm |
| | bit24 = 0 | RMW | RF1 #4 | ✅ asm |
| | bit25 = 1 | RMW (A2 fix: was bit21 — `0x02000000` not `0x00200000`) | RF1 enable | ✅ asm L75316 |
| `+0x50` | bit14 = 1 | RMW (A3 fix: was bit12 — `0x4000` not `0x1000`) | RF2 #1 | ✅ asm L75331 |
| `+0x54` | bits[3:0] = 12 | RMW | RF2b #1 (A4 fix: previously misplaced at +0x50) | ✅ asm |
| | bit7 = 1 | RMW | RF2b #2 | ✅ asm |
| | bit12 = 0 | RMW | RF2b #3 | ✅ asm |
| `+0x3C` | bits[15:12] = `8` | RMW | (See above — same as `+0x38` row in early text; verify offset) | ✅ asm |
| | bits[26:24] | RMW | CFG5 cont. | ✅ |
| | `0xE000_0000` clr, `0x4000_0000` set | RMW (A5 fix: clear mask was `0x6000`) | CFG5 PLL strobe | ✅ asm |

(Note: A1-A5 / B1-B3 are bug-fix labels from Lucy's full asm audit on 2026-05-01.)

### 3b. Calibration triggers / paths (regint.rs)

| Offset | Bit | Role | Conf |
|--------|-----|------|------|
| `+0x04` | bit0 | TX_tune_trigger (assert/deassert pulse for PLL measurement) | ✅ asm |
| | bit4 | TX_cal_mode (set during TX cal, cleared post) | ✅ asm |
| | bit8 | TXF_enable (RFEND_TXFtune sets; cleared post-cal Bug #3) | ✅ asm |
| | bit12 | RX_filter_mode (set during RX filter cal; cleared post Bug #3) | ✅ asm |
| | bit16 | RX_ADC_config (cleared then set in RFEND_RXAdc) | ✅ asm |
| `+0x08` | bit16 | RX_ADC path enable | ✅ asm |
| | bit17 | TX cal path enable (set in pre-init mask `0x0033_0000`) | ✅ asm |
| | bit20, bit21 | TX/RX PLL pre-enable (pre-init mask `0x0033_0000`) | ✅ asm |
| | bit21 | RX_filter path enable (set in RFEND_RXFilter) | ✅ asm |
| `+0x0C` | bit4 | RX_filter_strobe (RFEND_RXFilter: high → 4 nops → low → high again) | ✅ asm |
| | bit8 | ADC_ref_strobe (RFEND_RXAdc latch) | ✅ asm |
| `+0x28` | bit12 | TX_tune_mode_final (set after CO/GA cal complete) | ✅ asm L71173 |
| `+0x2C` | bit4 | post_cal_enable (set after cal) | ✅ asm L71174 |
| `+0x38` | bits[15:8] | freq_code (BF00=2401, D300=2440, E700=2480) | ✅ asm + hw |
| | bits[5:0] | nCO2440 — stored CO anchor for default channel comp | ✅ asm |
| | bits[30:24] | nGA2440 — stored GA anchor for default channel comp | ✅ asm |
| `+0x50` | bit16 | RX_filter_valid (cleared at start of RX filter cal, set post) | ✅ asm |
| | bits[4:0] | RX_filter_result (latched copy from `+0x9C` post-cal) | ✅ asm |
| `+0x58` | bit16 | RX_ADC_ref_enable (cleared then set during RFEND_RXAdc) | ✅ asm |
| `+0x90` | bits[5:0] | CO_result (read after PLL lock) | ✅ asm + hw |
| | bit25 | tune_done (PLL locked, CO+GA valid) — second-check by RFEND_WaitTune | ✅ asm + hw |
| | bit26 | tune_active (set first; double-check semantic) | ✅ asm + hw |
| | (read side-effect) | Reading `+0x90` latches GA into `+0x94` | ✅ empirical 2026-05-01 |
| `+0x94` | bits[16:10] | GA_result (latched by `+0x90` read) | ✅ asm + hw |
| `+0x9C` | bit8 | RX_filter_done (RFEND_RXFilter polls this bit) | ✅ asm |
| | bits[4:0] | RX_filter_result | ✅ asm |

### 3c. Calibration result tables (regint.rs `rfend_tx_ctune`)

All tables nibble-packed (4 bits per channel × 8 channels per word).

| Offset | Words | Channels | Encoding | Conf |
|--------|-------|----------|----------|------|
| `+0xA0..+0xB0` | 5 | ch0..ch39 | CO1[ch] = `delta_low * (39-ch) / 39` (monotone decreasing, ch0=delta_low → ch39=0) | ✅ asm + hw |
| `+0xB4..+0xC4` | 5 | ch0..ch39 | CO2[ch] = `delta_high * (ch+1) / 40` (monotone increasing, ch0≈0 → ch39=delta_high) | ✅ asm + hw |
| `+0xC8` nibble 0-1 | — | ch40, ch41 | CO2 overflow (guard band) | ✅ asm |
| `+0xC8` nibble 2-7 | — | GA1[0..5] | `(delta_ga * (10-n) / delta_high \| 8) & 0xF` | ✅ asm |
| `+0xCC` nibble 0-3 | — | GA1[6..9] | `(delta_ga * (4-n) / delta_high \| 8) & 0xF` | ✅ asm |
| `+0xCC` nibble 4 | — | (separator) | always 0 | ✅ asm |
| `+0xCC` nibble 5-7 | — | GA2[0..2] | `delta_ga2 * (n+1) / delta_low & 0xF` (signed nibble OK) | ✅ asm |
| `+0xD0` nibble 0-6 | — | GA2[3..9] | `delta_ga2 * (n+4) / delta_low & 0xF` | ✅ asm |
| `+0xD0` nibble 7 | — | (clear) | always 0 | ✅ asm |

**Frequency codes** (RFEND+0x38 bits[15:8]):
- `0xBF00` = 2401 MHz (one step below BLE ch0 = 2402)
- `0xD300` = 2440 MHz (BLE ch19, band midpoint — anchor)
- `0xE700` = 2480 MHz (BLE ch39)

**Live dump** (post `dtm.elf` run, 2026-05-01): `0x40025000+0xA0` word0 = `0x45555556` (ch0=6, ch7=4) ✓ matches `delta_low=6` formula.

---

## Block 4 — AES @ `0x40024300` (gptrAESReg)

Used by: `bb.rs::bb_irq_lib_handler` `.L9` cleanup only.

| Offset | Bit | Role | Conf |
|--------|-----|------|------|
| `+0x04` | bit0 | AES op cleanup phase 2 (cleared after bit1) | ✅ asm L91127 (`.L9`) |
| | bit1 | AES op cleanup phase 1 (cleared first if set) | ✅ asm |
| | (other) | All other AES registers — not yet decoded | ❓ |

**Notes**:
- `bb_irq_lib_handler` only checks bit1 set, then performs sequential clear bit1 → reload → clear bit0. The two-step semantic suggests these are paired status/ack bits but exact AES operation is undecoded for the ADV TX path.

---

## Block 5 — BLE Controller On-Chip ROM @ `0x420B0000` (opaque)

**Confidence**: ❓ region presence inferred from the default value of `fnGetClockCBs` (`0x420B000A`, in the WCH ROM range `0x4_xxxx_xxxx`). Contents are **execute-only** — not readable via wlink halt+dump.

| Address (range) | Width | Role | Conf |
|-----------------|-------|------|------|
| `0x420B0000` | 4 KB+ | BLE controller ROM — houses default `fnGetClockCBs` callback fn (entry point at `0x420B000A`, thumb-style low-bit-set) | ❓ inferred |
| `0x420A_0000`..`0x4210_0000` boundaries | — | All return zeros / non-readable when probed via wlink halt+dump | ✅ 2026-05-06 hw probe (Cindy) |

**Live probe — both paths confirm read-as-zero (2026-05-06, Cindy)**:

*Path 1 — Debugger halt + wlink dump (debugger SBA backdoor)*:
- Halt CPU then `wlink dump 0x420B0000 256` → all zeros.
- Halt CPU then `wlink dump 0x420B0000 4096` → all zeros.
- Halt CPU then `wlink dump 0x420A0000 / 0x420C0000 / 0x42100000 256` → all zeros / non-readable.

*Path 2 — CPU running-mode `lw` via main-bus (probe/rom-running-read branch `dd7d567`, BIN `/tmp/ble_tx_adv_ch37_probe_rom_running_read.bin` 7,644 B)*:
- `0x420B0000..0x420B00FC` (64 words via `core::ptr::read_volatile`) → all `0x00000000`.
- Spot-sample `0x420B0008 / 0x420B000C / 0x420B0010` → all `0x00000000`.
- Probe reached `=== ROM probe end ===` and entered idle `loop {}` → **no hardfault, no load access fault**.
- Logs: `/tmp/sdi_probe_rom_running_read_20260506_1748_rom_running_read.log`, `/tmp/wlink_probe_rom_running_read_20260506_1748_rom_running_read.log`.

**Conclusion**: 0x420B0000 region implements **execute-only memory** via a soft-block bus policy:
- Instruction fetch (`jalr` to `0x420B000A`) is **allowed** — BLE init successfully dispatches into the ROM `fnGetClockCBs` default fn and works.
- Data load (`lw` from any address in the region) is **silently returned as zero**, both via the debugger SBA backdoor *and* via the CPU main bus.
- The bus matrix does NOT raise an access fault for data reads — it returns a zero word. This is a softer security mechanism than hardfault but functionally identical for our purposes (we cannot read the contents).

The CH32V208 reference manual marks this address space as "reserved" with no further description. Both possible read paths now confirmed closed → **permanent black box**, no path to disassemble or reverse-engineer the ROM's `fnGetClockCBs` body.

**Practical implication**: `fnGetClockCBs @ 0x20001c78` (RAM) holds a fn ptr that ROM dispatches into. The default value `0x420B000A` points into this opaque ROM. We must let ROM auto-manage this slot rather than supplying a Rust replacement (see "ROM-pinned SRAM data structures" below + Iron Law #35).

---

## ROM-Pinned SRAM Data Structures

These are not peripheral MMIO, but RAM-resident data structures whose addresses are **hardcoded inside the BLE controller ROM**. Any Rust port must place the matching symbols at these exact addresses or BLE init silently misbehaves.

| Address | Size | Symbol (WCH name) | Owner | Conf |
|---------|------|-------------------|-------|------|
| `0x20000758` | varies (≥ ~256 B) | `gBleIPPara` (BLE IP parameter block, 32-bit fields incl. `[0]` mode flags, `[4]` ISR re-arm flags, `[16..19]` LLE+0x64 source = 776 for ADV TX, `[20..23]` LLE+0x6c source, `[36]` LLE_DMA buf MEMAddr) | Rust strong symbol (Phase D+1 #34) | ✅ hw boundary confirmed |
| `0x20001858` | varies (≥ ~1 KB est.) | `gBleLlPara` (a.k.a. `ble`, link-layer parameter block) | Rust strong symbol (Phase D+1 #35) | ✅ hw boundary confirmed |
| `0x20001c78` | 4 B (fn ptr) | `fnGetClockCBs` (BLE clock-source callback) | ROM-managed (boundary mode — see contract below) | ✅ Iron Law #34/#35/#36 |

### `fnGetClockCBs` callback contract (T8 final, 2026-05-06)

The `fnGetClockCBs` slot at `0x20001c78` is a 4-byte fn-pointer that the BLE controller ROM reads during BLE init and dispatches into. T8 attempts 11-15 fully characterized its ROM behavior:

| RAM value at 0x20001c78 (boot) | ROM behavior | BLE outcome |
|--------------------------------|--------------|-------------|
| `0x00000000` (NULL — startup-zeroed) | ROM falls back to internal HSE path (broken on this MCU revision) | **FAIL** — cba=0 |
| `0x420B000A` (ROM default fn ptr) | ROM dispatches to its own ROM fn | **PASS** — cba ≥ 52 |
| Non-zero, non-flash garbage (e.g. random SRAM `0xcbb5ae96` at cold boot) | ROM detects "not flash range", auto-installs `0x420B000A` | **PASS** — same as ROM-default case |
| User-supplied valid Rust fn ptr returning `hclk_hz` (e.g. attempt-14) | ROM accepts the write, dispatches into the Rust fn — but the ROM contract is **not** "return HCLK Hz" | **FAIL** — cba=0 (attempt-14 disqualified) |

**Linker placement (Path B, T8 attempt-15)** — `examples/ch32v208/build.rs` link.x:

```ld
.bss (NOLOAD) : ALIGN(4) {
    PROVIDE(_sbss = .);
    /* … other BSS … */
    . = 0x20001c78;
    PROVIDE(_ebss = .);                  /* boundary BEFORE fnGetClockCBs */
    KEEP(*(.fnGetClockCBs));             /* fnGetClockCBs at 0x20001c78, NOT startup-zeroed */
} >RAM
```

**Key insight**: `_ebss` is placed AT `0x20001c78` so the qingke-rt startup BSS-zero loop (`_sbss .. _ebss` exclusive) does **not** zero the `fnGetClockCBs` slot. Whatever value the slot holds at boot is preserved into ROM init — either the ROM-default `0x420B000A` (warm boot after prior BLE init) or random uninitialized SRAM (cold boot, e.g. `0xcbb5ae96`). In both cases the ROM auto-manages the slot correctly.

**Iron Laws** (locked T8 close):
- **#34** — `_ebss = address_of(fnGetClockCBs) = 0x20001c78` (boundary).
- **#35** — `fnGetClockCBs` must conform to the `pfnGetSysClock` contract (returns RTC tick counter, LSI/2 ≈ 16 KHz, 32-bit wrap — see `wchble.h` L77 + `HAL/RTC.c` L80-96 `RTC_GetCounter`). T8 attempt-15 ships **Path A boundary mode** (Rust does not write the slot; ROM auto-installs its own conforming fn at `0x420B000A`). **Forbidden**: writing a Rust fn that returns Hz (attempt-14 falsified, cba=0). Path B (Rust fn returning RTC counter) is theoretically conformant but untested at T8 close — requires full F1/F2/F3 per attempt to validate.
- **#36** — `_ebss > 0x20001c78` is forbidden (would zero or shift the slot).

See `t8-final-strip-plan.md` §Iron Laws and `lib-dependency-removal.md` for full T8 attempt-1..15 forensic.

---

## Public BLE Library Types (wchble.h mirror)

> **Status: PRE-DRAFT (2026-05-06)** — landing with task #43 Phase A code (Vega owner). This section mirrors the public C types in `CH32V20xEVT-2.31/EXAM/BLE/LIB/wchble.h` so Rust callers can interact with the BLE controller library through the documented ABI rather than ad-hoc `u32` arrays. Layouts cross-checked against wchble.h L70-119; sizes verified by `core::mem::size_of` const asserts in Rust.
>
> **Why this section exists**: T8 attempts 11-14 partially reinvented these types (`Option<unsafe extern "C" fn() -> u32>` for `fnGetClockCBs` etc.) without realizing wchble.h already defines the canonical typedefs. Task #43 reconciles ad-hoc Rust types in `src/ble/{mod,bb,lle,rfend,regint,adv,listener}.rs` with the C public ABI. `tx adv` baseline (attempt-15 `f27c394`) does not change.

### Function-pointer typedefs (wchble.h L67-77)

| C name | C signature | Rust mirror | Notes |
|--------|-------------|-------------|-------|
| `pTaskEventHandlerFn` | `tmosEvents (*)(tmosTaskID, tmosEvents)` | `unsafe extern "C" fn(TmosTaskId, TmosEvents) -> TmosEvents` | TMOS task event dispatcher; `tmosTaskID = u8`, `tmosEvents = u16` |
| `pfnSrandCB` | `void (*)(uint32_t seed)` | `unsafe extern "C" fn(seed: u32)` | Seed RNG |
| `pfnIdleCB` | `void (*)(uint32_t delta)` | `unsafe extern "C" fn(delta: u32)` | Enter idle for `delta` clock counts |
| `pfnTempSampleCB` | `uint32_t (*)(void)` | `unsafe extern "C" fn() -> u32` | Temperature sample (for LSI calibration) |
| `pfnLSICalibrationCB` | `uint32_t (*)(void)` | `unsafe extern "C" fn() -> u32` | LSI calibration |
| `pfnLibStatusErrorCB` | `void (*)(uint8_t status, uint32_t param, uint16_t dataLen)` | `unsafe extern "C" fn(status: u8, param: u32, data_len: u16)` | Library status / error |
| `pfnFlashReadCB` | `uint32_t (*)(uint32_t addr, uint32_t num, uint32_t *pBuf)` | `unsafe extern "C" fn(addr: u32, num: u32, p_buf: *mut u32) -> u32` | Read flash for SNV |
| `pfnFlashWriteCB` | `uint32_t (*)(uint32_t addr, uint32_t num, uint32_t *pBuf)` | `unsafe extern "C" fn(addr: u32, num: u32, p_buf: *const u32) -> u32` | Write flash for SNV |
| `pfnGetSysClock` | `uint32_t (*)(void)` | `unsafe extern "C" fn() -> u32` | **Returns RTC tick counter (LSI/2 ≈ 16 KHz, 32-bit wrap) — NOT Hz**. See `fnGetClockCBs` callback contract above. |

### `bleClockConfig_t` (wchble.h L112-119, sizeof=16, alignof=4)

```c
typedef struct tag_ble_clock_config {
    pfnGetSysClock getClockValue;      // fn ptr — 4 B
    uint32_t       ClockMaxCount;      // 4 B — typically 0xFFFFFFFF (32-bit wrap)
    uint16_t       ClockFrequency;     // 2 B — Hz (e.g. CAB_LSIFQ/2 ≈ 16000)
    uint16_t       ClockAccuracy;      // 2 B — ppm (typically 100 or 1000)
    uint8_t        irqEnable;          // 1 B + 3 B padding — reserved
} bleClockConfig_t;                    // total = 16 B
```

Rust mirror (`#[repr(C)]`):
```rust
#[repr(C)]
pub struct BleClockConfig {
    pub get_clock_value: Option<PfnGetSysClock>,  // NPO → 4 B (None == NULL fn ptr)
    pub clock_max_count: u32,
    pub clock_frequency: u16,
    pub clock_accuracy:  u16,
    pub irq_enable:      u8,
}
const _: () = assert!(core::mem::size_of::<BleClockConfig>() == 16);
const _: () = assert!(core::mem::align_of::<BleClockConfig>() == 4);
```

Reference init in C (HAL/RTC.c L80-96):
```c
RCC_RTCCLKCmd(ENABLE);
RTC_WaitForLastTask();
RTC_SetPrescaler(1);
RTC_SetCounter(0);
conf.ClockAccuracy  = CLK_OSC32K ? 1000 : 100;
conf.ClockFrequency = CAB_LSIFQ / 2;
conf.ClockMaxCount  = 0xFFFFFFFF;
conf.getClockValue  = RTC_GetCounter;     // ← assigns the fn ptr that ends up at 0x20001c78
state = TMOS_TimerInit(&conf);
```

### `bleConfig_t` (wchble.h L80-109, ~30 fields)

Larger config struct; fields:
- `MEMAddr: u32` — library memory start (RAM allocation for BLE lib state)
- `MEMLen: u16` — library memory size
- `SNVAddr: u32` — SNV flash start (NULL → bonding info not saved)
- `SNVBlock: u16`, `SNVNum: u8` — SNV layout
- `BufNumber: u8`, `BufMaxLen: u16` — HCI packet buffering
- `TxNumEvent: u8`, `RxNumEvent: u8`, `TxPower: u8`, `ConnectNumber: u8`
- `WindowWidening: u8`, `WaitWindow: u8`
- `MacAddr: [u8; 6]` — little-endian
- `ClockFrequency: u16` (Hz), `ClockAccuracy: u16` (ppm) — same encoding as `bleClockConfig_t`
- 7 fn-ptr CBs: `srandCB`, `idleCB`, `tsCB`, `rcCB`, `staCB`, `readFlashCB`, `writeFlashCB`

Phase A (task #43) may defer `bleConfig_t` mirror to Phase B if no `src/ble/*.rs` ad-hoc usage requires it for the typedef-pass alone.

### Cross-references

- Source of truth: `/Users/mono/Elec/WCH/CH32V20xEVT-2.31/EXAM/BLE/LIB/wchble.h` L67-119
- Reference init site: `CH32V20xEVT-2.31/EXAM/BLE/HAL/RTC.c` L80-96 (`TMOS_TimerInit(&conf)`)
- Iron Law #34/#35/#36 (above) — `fnGetClockCBs` slot must NOT be Rust-written post-init; Rust storage at `_ebss` boundary.
- Task #43 (`#ch32-rs:c2413069`) — type alignment owner Vega, review Lucy, hardware Cindy.

---

## Open Questions / TODOs

1. ❓ **BB CFG bit3 of rf_flag** — bit0=1M PHY confirmed; bit3 set in our hw value (`rf_flag=0x09`) but exact role TBD.
2. ❓ **BB+0x38 bits other than 4/5/6/7** — IRQ STATUS likely has more sources (RX path, ACK, error). Map when RX path lands.
3. ❓ **AES block beyond +0x04** — full register map TBD (only matters when AES-CCM landed).
4. 🔍 **LLE+0x14/+0x24/+0x34/+0x44 ALL = 140** — these are 4 separate timing slots, hardware-dump matched. Whether each maps to a different LL stage (preamble / access addr / payload / CRC) is plausible but unconfirmed.
5. 🔍 **RFEND+0x2C bits[13:12] / bits[17:16] semantic** — set bit13 / set bit17 at init suggests "PLL band select" but raw asm gives no name.
6. 🔍 **RFEND+0x30 bits[30:28] = `0b101`** — likely "loop filter bandwidth mode"; verify by comparing with WCH SDK header constants if/when found.
7. ❓ **`fnGetClockCBs` ROM contract** — the function at `0x420B000A` is execute-only ROM and its calling convention / return value semantic is private. Empirically it is **not** "return HCLK Hz" (attempt-14 falsified that hypothesis). The ROM slot is permanently treated as a black box (Iron Law #35); deeper decoding only possible if WCH publishes a header or via a side-channel (e.g. JTAG instruction trace, if available).
8. 🔍 **`gBleIPPara` / `gBleLlPara` field maps** — partial fields decoded via `bb_irq_lib_handler` and `ble_ip_core_init` asm paths, but full struct layouts (sizes, all field roles) are not yet documented as a single table. Migrate from C-style byte-offset writes to typed Rust state vars (Andelf post-T8 direction) will require this map.

## Update protocol

When you decode a new bit/offset:
1. Add row(s) in the appropriate block table.
2. Tag confidence (✅ / 🔍 / ❓).
3. Cite source (asm line / hw dump / commit / cross-check).
4. If a previous entry is invalidated, **mark it ❌ and append the corrected row** — do NOT silently delete (preserves audit trail like the rfend.rs A1-A5 / B1-B3 fix lineage).

## Cross-references

- `notes/ch32-rs/lib-dependency-removal.md` — main T0-T8 removal plan and Iron Laws (§Iron Law #23 explains TX_BUF mod16=0 hardware DMA strict requirement).
- `notes/ch32-rs/t8-final-strip-plan.md` — T8 final strip plan + Iron Laws #34/#35/#36 + ROM black-box conclusion + attempt-9..15 forensic.
- `notes/ch32-rs/ble-bb-irq-handler-cross-check.md` — full `bb_irq_lib_handler` decode (asm L91127 walkthrough).
- `notes/ch32-rs/ble-ipcoreinit-cross-check.md` — `BLE_IPCoreInit` reloc decode that fixed RFEND_BASE.
- `elec-docs/ble-reverse-docs/04-rfend-registers.md` — earliest reverse pass on RFEND.
- `elec-docs/ble-reverse-docs/05-lle-engine.md` — LLE state machine encoding source.
- `elec-docs/ble-reverse-docs/24-rf-phy-supplement.md` — BB / RF PHY mode flags.
