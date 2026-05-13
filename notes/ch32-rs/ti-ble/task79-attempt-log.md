# Task #79 attempt log — SCAN_RSP Step 2 baseline control → RX-only → warm-kick smoke

Status: control gate in progress (2026-05-14)
Branch: `feat/ble-scan-rsp-step2` on main repo `/Users/mono/Elec/WCH/ch32-hal`
HEAD: `7f04104` + gate patch (see §3)

---

## §1. Baseline selection history

### Candidate: 7f04104 + ch32-data 1b6b0d4
Selected in spec (scan-rsp-implementation-spec.md §2). Air-validated control claim was based on a
1-frame Segment B capture — insufficient.

**P1 (2026-05-11)**: `7f04104` produces 0 ADV / 65.5s after power cycle. Root cause: working
SysTick timeout (2ms) causes `rx_turnaround_capture_ch37` to run mid-TX → `bb_write(0x08, 0x2000)`
(BLE_LLE.access_addr) clobbers in-progress TX → device silent.

**Sniffer confirmation (2026-05-14)**: Three captures, 20309 total BLE ADV-AA records, target
frames = 0. Ambient BLE captured normally → sniffer is working; device genuinely silent.
Tags: `inconclusive/scan-rsp-step2-control-air-blocked-20260514`,
      `inconclusive/scan-rsp-step2-control-no-air-sniffer-restored-20260514`

### Why rx_turnaround fires: df26837 vs 7f04104

| Commit | mcycle behavior | probe-wait exit | rx_turnaround timing | Air result |
|--------|----------------|-----------------|----------------------|------------|
| df26837 | always 0 → `0 < 192_000` always true → waits until ip4==1 | ip4==1 (TX done) | post-TX | 17.1 Hz ✓ |
| 7f04104 | SysTick CNTL, real 2ms timeout | timeout (ip4≠1) | mid-TX | 0 Hz ✗ |

df26837..7f04104 = **1 commit only** (the SysTick fix). No bisect needed; regression commit is
7f04104 itself.

---

## §2. Options considered

| # | Description | Decision |
|---|-------------|----------|
| v4a | probe-wait gate `ip4==0x80 && dt<240_000`, keep rx_turnaround | FAIL — W1C 0xF00F transient bit13 clear kills TX |
| v4b | same + delete rx_turnaround entirely | FAIL — missing LLE[8]+LLE[100]+gBleIPPara[4] re-arm, 0 ADV |
| α bisect df26837..7f04104 | find regression commit | MOOT — only 1 commit in range |
| C1 fork df26837 | use broken-mcycle baseline | REJECTED — loses SysTick fine timer for T_IFS gate |
| **Option 2 (current)** | gate rx_turnaround on `irq==1` in 7f04104 | **IN PROGRESS** |
| A1 surgical | replace rx_turnaround with libwchble ll_tx_wait_finish mode=1 full sequence | task #80, post-task-79-PASS |
| Path A | HW autonomous pivot | deferred |

---

## §3. Current patch (option 2)

File: `examples/ch32v208/src/bin/ble_peripheral_phase1_adv.rs`
Change: gate `rx_turnaround_capture_ch37` call on `irq == 1` at call site (~line 2074).

`irq` = ip4 as u32 (3rd return value from `adv_tx_burst_ch37`).
`irq == 1` means `gBleIPPara[4]` became 1 within 2ms (TX done path).
`irq != 1` means 2ms SysTick timeout (rx_turnaround skipped).

```rust
// Before (unconditional, caused mid-TX clobber):
let rx_snap = rx_turnaround_capture_ch37(adv_channel);

// After (gated):
let rx_snap = if irq == 1 {
    rx_turnaround_capture_ch37(adv_channel)
} else {
    None
};
```

Note: SDI print removed from skip path — SDI µs overhead in timing-sensitive path violates
Andelf directive (msg 8ab182ec). Use pcap first; add a ring/counter only after the air result needs
local diagnosis.

### Attempt result (2026-05-14 02:07)

- Build: PASS, `/tmp/task79_gate_rx_turnaround_noskipprint_build_20260514_0207.log`
- Flash: PASS, `/tmp/task79_gate_rx_turnaround_noskipprint_flash_20260514_0207.log`
- Pcap: `/tmp/task79_scan_rsp_step2/gate_rx_turnaround_noskipprint_sniff60.pcapng`
- Pcap raw evidence: ambient BLE ADV-AA = 17886, target AdvA bytes = 0, `Simple` name bytes = 0.
- Result: FAIL — target remains air-silent with the gate patch.

Candidate causes to prove next:

1. `ip4==1` may be rare under this path, so RX-turnaround almost never runs while another TX suppressor remains active.
2. Existing task/SDI prints around the ADV path may still perturb timing.
3. The current partial RX-prep state machine may have a pre-GO or IRQ-side suppressor independent of the timeout-path helper call.
4. Full `libwchble` mode=1 alignment from task #80 may be required before Step 2 can proceed.

Tags to apply on result:
- PASS: `probe-control/scan-rsp-step2-gate-patch-pass-YYYYMMDD`
- FAIL: `bad/scan-rsp-step2-gate-patch-fail-{symptom}-YYYYMMDD`

---

## §4. Control gate pass criteria

Per scan-rsp-implementation-spec.md §6.1:
- target AdvA `C2:21:43:65:87:12` visible ≥1 frame in 60s pcap
- serial shows firmware alive
- sniffer used today: `/dev/cu.usbmodem21101`

---

## §5. Register naming reminder (GF-2)

In this example file:
- `bb_write(N, v)` → physical `0x40024200 + N` = WCH gptrLLEReg (BLE_LLE)
- `lle_write(N, v)` → physical `0x40024100 + N` = WCH gptrBBReg (BLE_BB)

**SWAPPED from WCH naming.** See notes/ch32-rs/ground-facts.md.

---

## §6. Pending after control gate PASS

1. RX-only gate: add SCAN_REQ parser, log `SCAN_REQ_ACCEPT`, verify pcap+serial match
2. SCAN_RSP warm-kick gate: enable warm kick, pcap sees target SCAN_RSP
3. OQ3 T_IFS on-chip trace: SysTick CNTL ring at SCAN_REQ_ACCEPT + SCAN_RSP_KICK
4. task #80: A1 libwchble mode=1 alignment (post step 2 PASS)

SDI timing note: any `hal::println!` inside the SCAN_REQ→SCAN_RSP turnaround window (T_IFS
150 µs) will perturb timing. Replace with counter/ring before T_IFS gate runs.
