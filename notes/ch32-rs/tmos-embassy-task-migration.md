# CH32 BLE TMOS/EVT/libwchble task model to Embassy migration plan

Date: 2026-05-10

## Scope

This note turns the current PeripheralRole discussion into an implementation plan. The goal is a Rust-only CH32 BLE peripheral stack that keeps the useful TMOS/EVT task model semantics while using Embassy for Rust scheduling.

Inputs:

- WCH EVT Peripheral example: `CH32V20xEVT-2.31/EXAM/BLE/Peripheral/APP/peripheral.c`
- WCH BLE ABI: `CH32V20xEVT-2.31/EXAM/BLE/LIB/wchble.h`
- Existing Rust reference bridge: `ch58x-hal/src/ble/mod.rs`, `ch58x-hal/examples/ble-peripheral.rs`
- Current ch32-hal BLE work: `src/ble/*`, `examples/ch32v208/src/bin/ble_rx_listener.rs`, `examples/ch32v208/src/bin/ble_peripheral_phase1_adv.rs`
- Prior local research: `notes/ch32-rs/peripheral-role-embassy-migration.md`
- Embassy docs: executor, time, sync primitives

Unavailable local references:

- `notes/ch32-rs/wch-ble-manual.md`
- `notes/ch32-rs/tmos-timerinit-disasm.md`

Those names appear in prior discussion, but they are absent from this worktree. This note therefore uses EVT source, `wchble.h`, ch32-hal source, ch58x-hal source, and official Embassy docs as the evidence base.

Additional elec-docs references found after review:

- `/Users/mono/Elec/elec-docs/WCHBLE.md`
- `/Users/mono/Elec/elec-docs/ble-reverse-docs/07-tmos-internals.md`

`WCHBLE.md` contains the PDF-derived TMOS API section, including `TMOS_TimerInit`, `tmos_start_task`, `tmos_start_reload_task`, `TMOS_GetSystemClock`, `TMOS_SystemProcess`, and message APIs. `07-tmos-internals.md` contains the reverse-engineered task table, event bitmask, timer node, message header, and TMOS dispatch model. These two elec-docs files cover the missing manual/disassembly facts needed for this plan.

## Decision

Use a three-layer runtime model:

1. **Radio hard-real-time layer**: BB/LLE/RF ISR and hardware timers own T_IFS, RX window, connection-event anchor, and sub-millisecond radio deadlines. ISR work stays bounded: read/clear flags, arm hardware, copy small snapshots, signal an Embassy task.
2. **LL controller task layer**: Embassy task owns advertising, CONNECT_IND decode, connection context, channel hop, SN/NESN/MD, supervision, and packet queues. ISR wakes it with `Signal` or `Channel`.
3. **Host/app task layer**: Embassy tasks model TMOS events for GAPRole/GATTServApp/SimpleProfile semantics. Timers use `Timer::at` or a generation-counter timer wrapper. Event messages use bounded channels.

This gives us TMOS-compatible control flow in direct Rust. The direct Rust stack can still follow EVT ordering and event names.

Confidence: high for the task/event mapping, high for Embassy primitive mapping, medium for the exact LL radio handoff details. The LL handoff still needs `ll_advertise_legacy_rx`-level register evidence and hardware gates.

## Embassy properties relevant to BLE

Embassy executor:

- Tasks are statically allocated by `#[embassy_executor::task]`; runtime allocation stays out of the scheduler path.
- Thread-mode executor polls async tasks cooperatively; a task runs until it reaches `.await`.
- Woken tasks are polled individually; idle executor sleeps.
- `InterruptExecutor` exists for higher-priority async work, but the current CH32 path should first use plain IRQ handlers plus `Signal`/`Channel`. Radio register writes inside an async interrupt executor add complexity before the LL path is stable.

Evidence: official Embassy executor docs state that embedded tasks are statically allocated, include an integrated timer queue, and support multiple executor instances including interrupt-mode executors. See <https://docs.embassy.dev/struct.InterruptExecutor.html>.

Embassy time:

- `embassy_time::Timer` is a future that expires at an `Instant`; `Timer::after` and `Timer::at` cover one-shot sleeps.
- `Ticker` is suitable for non-radio periodic app events.
- The CH32 SysTick driver uses HCLK/8 and Embassy's global time driver. At HCLK 144 MHz, the counter input is 18 MHz; with the usual 1 MHz tick configuration, Embassy time has 1 us tick granularity.
- Use `Timer::at(next)` for periodic GAP/app timers to avoid cumulative drift.
- Use BB/LLE hardware timers for T_IFS, connection event windows, and exact radio anchors.

Evidence: official Embassy time docs describe a global time driver, 64-bit ticks, `Timer`, and `Ticker`. See <https://docs.embassy.dev/embassy-time/git/default/index.html> and <https://docs.embassy.dev/embassy-time/git/default/struct.Timer.html>. ch32-hal implements the global driver in `src/embassy/time_driver_systick.rs`.

Clock boundary: `tmos_start_task` and `TMOS_GetSystemClock` use TMOS logical ticks where one tick is 625 us. This is the `pfnTimerCBs`/TMOS path. It is separate from the low-level `fnGetClockCBs` raw counter path used by the BLE controller. The wrapper below deliberately models the TMOS logical timer unit and should not be reused for raw LL clock callbacks.

Embassy sync:

- `Signal<CriticalSectionRawMutex, T>` is a single-slot ISR-to-task wake path. It is appropriate for "latest state" notifications, e.g. `LlIrqSummary`.
- `Channel<CriticalSectionRawMutex, T, N>` is appropriate for ordered event queues, e.g. host messages and ATT packets.
- `CriticalSectionRawMutex` is safe for state shared between tasks and interrupts.

Evidence: official Embassy sync docs describe `Signal` as a single-slot primitive and `CriticalSectionRawMutex` as safe across executors and interrupts. See <https://docs.embassy.dev/embassy-sync/0.6.1/default/signal/struct.Signal.html> and <https://docs.embassy.dev/embassy-sync/git/default/blocking_mutex/raw/struct.CriticalSectionRawMutex.html>.

## TMOS model from WCH EVT

TMOS gives the WCH host stack a small cooperative scheduler:

- `tmosTaskID = uint8_t`
- `tmosEvents = uint16_t`
- `tmosTimer = uint32_t`
- `SYS_EVENT_MSG = 0x8000`
- `SYSTEM_TIME_MICROSEN = 625`
- `TMOS_ProcessEventRegister(cb)` registers a task event function.
- `tmos_set_event(task, event)` ORs a bit into a task's pending event mask.
- `tmos_start_task(task, event, time)` sets the event after `time` ticks.
- `tmos_start_reload_task(task, event, time)` is a periodic timer.
- `tmos_stop_task(task, event)` cancels a pending timer.
- `tmos_msg_send(task, msg)` enqueues a message and sets `SYS_EVENT_MSG`.
- `TMOS_SystemProcess()` dispatches pending task event masks through registered callbacks.

EVT `Peripheral_ProcessEvent()` uses this event set:

| Event | EVT action |
|---|---|
| `SYS_EVENT_MSG` | `tmos_msg_receive`, dispatch GAP/GATT message, `tmos_msg_deallocate` |
| `SBP_START_DEVICE_EVT` | `GAPRole_PeripheralStartDevice(...)` |
| `SBP_PERIODIC_EVT` | reschedule timer and send SimpleProfile char4 notification |
| `SBP_PARAM_UPDATE_EVT` | `GAPRole_PeripheralConnParamUpdateReq(...)` |
| `SBP_PHY_UPDATE_EVT` | `GAPRole_UpdatePHY(...)` |
| `SBP_READ_RSSI_EVT` | `GAPRole_ReadRssiCmd(...)` and reschedule |

The EVT Peripheral init sequence:

1. `Peripheral_TaskID = TMOS_ProcessEventRegister(Peripheral_ProcessEvent)`
2. configure GAPRole advertising enable, adv data, scan response, min/max interval
3. configure GAP params: adv interval min/max, scan request notify
4. configure GAPBondMgr
5. add GAP/GATT/DeviceInfo/SimpleProfile services
6. set GAP device name and SimpleProfile characteristic defaults
7. register callbacks
8. `tmos_set_event(Peripheral_TaskID, SBP_START_DEVICE_EVT)`

On `GAP_LINK_ESTABLISHED_EVENT`, EVT fills `peripheralConnList`, resets MTU, and starts periodic, param-update, and RSSI timers. The WCH header defines `gapEstLinkReqEvent_t` with connection handle, interval, latency, timeout, role, peer address, and clock accuracy.

## Existing bridge evidence

`ch58x-hal` shows a practical Embassy/TMOS bridge:

- `hal::ble::init()` calls WCH `BLE_LibInit`, `TMOS_TimerInit`, registers a HAL TMOS task, starts HAL register init events.
- HAL TMOS task receives `SYS_EVENT_MSG`, wraps the raw pointer as `TmosEvent`, publishes it through `PubSubChannel<CriticalSectionRawMutex, ...>`, and deallocates the message in `Drop`.
- The example runs:
  - WCH BLE init
  - `GAPRole_PeripheralInit()`
  - `peripheral_init()`
  - `spawner.spawn(peripheral(task_id, sub))`
  - an async loop calling `TMOS_SystemProcess()` every 300 us

That bridge validates the scheduling shape. The CH32 target uses the same shape as a reference, then replaces WCH FFI internals with direct Rust LL/host logic.

Bridge confidence: high for Embassy/TMOS coexistence. ch58x-hal still calls WCH BLE FFI, so it validates scheduling and message delivery shape rather than validating the Rust-only LL implementation.

## Rust mapping

| TMOS/EVT concept | Rust/Embassy mapping |
|---|---|
| `TMOS_ProcessEventRegister(cb)` | static `BleTaskId` enum plus per-layer async task or handler table |
| task pending event mask | critical-section bitmask plus `Signal` wake |
| `tmos_set_event(task, event)` | OR event bit and `signal(task)` |
| `tmos_clear_event(task, event)` | clear event bit under critical section |
| `tmos_start_task(task, event, ticks)` | spawn/update timer entry; `Timer::at(deadline)` then set event |
| `tmos_start_reload_task` | `Ticker` or timer loop using absolute deadlines |
| `tmos_stop_task` | generation counter or cancel token; stale timer completions are ignored |
| `SYS_EVENT_MSG` | bounded `Channel<BleMsg>` plus event bit |
| `tmos_msg_receive` | drain one message from the channel |
| `tmos_msg_deallocate` | Rust ownership drop |
| `TMOS_SystemProcess` | explicit `ble_host_task` loop that drains events/messages |
| GAPRole state callback | `BleHostEvent::Gap(GapEvent::StateChanged(...))` |
| `GAP_LINK_ESTABLISHED_EVENT` | `BleHostEvent::Gap(GapEvent::LinkEstablished(ConnectionInfo))` |

Recommended internal types:

```rust
enum BleTaskId {
    Ll,
    L2cap,
    Gatt,
    GapRole,
    App,
}

struct TaskEvents {
    bits: Mutex<CriticalSectionRawMutex, Cell<u16>>,
    wake: Signal<CriticalSectionRawMutex, ()>,
}

enum BleMsg {
    Gap(GapEvent),
    Gatt(GattEvent),
    Att(AttEvent),
    L2cap(L2capEvent),
    App(AppEvent),
}

struct TimerEntry {
    generation: u32,
    task: BleTaskId,
    event: u16,
    deadline: embassy_time::Instant,
}
```

The timer wrapper should preserve WCH's 625 us unit at the compatibility boundary:

```rust
fn tmos_ticks_to_duration(ticks: u32) -> Duration {
    Duration::from_micros(ticks as u64 * 625)
}
```

The event-drain loop must drain all bits from the bitmask before waiting on the single-slot `Signal`:

```rust
loop {
    let bits = events.take();
    if bits == 0 {
        events.wake.wait().await;
        continue;
    }
    for bit in BitIter::new(bits) {
        dispatch(bit).await;
    }
}
```

For `tmos_clear_event`, use a critical-section read-clear operation or a `take()` processing model. Clearing after a handler returns can erase an event set by an ISR during the handler.

## Proposed runtime topology

### 1. IRQ layer

Inputs:

- BB IRQ
- LLE IRQ
- optional SysTick/Embassy time driver

Responsibilities:

- clear/ack hardware flags
- copy tiny snapshots from DMA buffers
- arm one-shot BB/LLE hardware timers for radio windows
- signal `ble_ll_task`
- write deferred trace records into a fixed ring buffer

Rules:

- Hot path uses deferred trace ring records.
- Loops are bounded.
- Static storage only.
- Avoid full RFEND mode transitions inside ISR until the register sequence is proven stable.

### 2. `ble_ll_task`

Responsibilities:

- advertising state machine
- ADV channel scheduling and advDelay
- SCAN_REQ/SCAN_RSP handling
- CONNECT_IND parse
- connection context creation
- connection event scheduling
- LL data PDU queues

Scheduling:

- Advertising interval can use Embassy `Timer::at` for Phase 1 and Phase 2 probing.
- Connection-event anchor and RX/TX sub-windows need BB/LLE hardware timer assistance.
- ISR gives `LlIrqEvent` to this task; this task decides next state.

### 3. `ble_host_task`

Responsibilities:

- TMOS-compatible event bit processing
- GAPRole state machine
- L2CAP/ATT/GATT dispatch
- app callbacks

Execution:

- Drain `SYS_EVENT_MSG`-equivalent messages first.
- Process one event bit at a time in EVT order for easier diffing against `Peripheral_ProcessEvent`.
- Use `Timer::at` for periodic app timers.

### 4. app task

The first app mirrors EVT SimpleProfile:

- device name: `Simple Peripheral`
- primary service UUID `0xFFF0`
- char1 read/write
- char3 write
- char4 notify + CCCD
- char5/auth paths deferred

## PeripheralRole event flow in Rust

### Startup

1. hardware BLE init and metapac register setup
2. init BLE runtime statics and queues
3. spawn `ble_ll_task`
4. spawn `ble_host_task`
5. host task configures GAPRole/GATT/SimpleProfile state
6. host sets `SBP_START_DEVICE_EVT`
7. GAPRole event starts connectable advertising

### Connect

1. LL task sends connectable `ADV_IND`.
2. Radio captures `CONNECT_IND`.
3. LL parses payload:
   - InitA
   - AdvA
   - access address
   - CRC init
   - window size/offset
   - interval
   - latency
   - timeout
   - channel map
   - hop and SCA
4. LL constructs `ConnectionContext`.
5. LL schedules first connection anchor.
6. LL publishes `GapEvent::LinkEstablished`.
7. Host task updates link database and starts periodic/param/RSSI timers.

### Disconnect

1. LL supervision timeout or LL_TERMINATE_IND produces `GapEvent::LinkTerminated`.
2. Host clears link state and stops periodic timers.
3. GAPRole restarts advertising.

## Implementation phases

### Phase A — runtime skeleton

#### Phase A0 — preserve probe state and freeze diagnostics

Deliverables:

- keep task #68 ad-hoc probe changes as diagnostic worktree state or a `bad/` branch
- record the latest ADV/CONNECT_IND evidence and register observations
- keep the production branch based on the last review-passed commit

Deliverables:

#### Phase A1 — add library runtime skeleton

- `src/ble/runtime.rs` or `src/ble/host.rs`
- `BleTaskId`, `TaskEvents`, `BleMsg`, `BleTimer`
- `ble_ll_task` and `ble_host_task` skeletons
- deferred trace ring for IRQ diagnostics

Gate:

- build
- existing `ble_tx_adv_ch37` and `ble_rx_listener` still pass their known gates
- deferred trace ring in BB/LLE ISR hot path
- merge gate: each baseline example needs 5/5 PASS; 4/5 expands to n=10; any example <=3/5 is treated as a skeleton regression until disproven.

### Phase B — connectable advertising as an LL task

Deliverables:

- move Phase 1 ADV event scheduling into `ble_ll_task`
- ADV_IND + SCAN_RSP per-channel event
- Android-visible Flags `0x06`
- 3-channel advertising
- advDelay 0..10 ms

Gate:

- macOS CoreBluetooth sees `Simple`
- Android scanner sees `Simple`
- automated scan gate stores SDI and macOS logs

### Phase C — CONNECT_IND capture and context

Deliverables:

- EVT-aligned TX to RX turnaround in ISR/hardware timer path
- `ConnectIndParams` parser
- `ConnectionContext` allocation
- `GapEvent::LinkEstablished` delivered to host task

Gate:

- macOS script initiates connection
- SDI/ring trace contains CONNECT_IND fields with AdvA match
- host task logs LinkEstablished and connection interval/timeout

### Phase D — one connection event engine

Deliverables:

- access address / CRC init setup
- channel map + hop
- event counter
- empty PDU keepalive
- supervision timeout

Gate:

- phone stays connected for 30 seconds
- disconnect returns to advertising

### Phase E — ATT/GATT SimpleProfile

Deliverables:

- L2CAP fixed channel demux
- ATT MTU 23
- primary service discovery
- characteristic discovery
- read/write/write-command
- CCCD and notification path

Gate:

- phone app reads and writes SimpleProfile characteristic
- notification observed on subscribed central

## Current task #68 implication

The CONNECT_IND probe has produced useful register-level evidence, but the remaining fixes are now inside a larger scheduler problem:

- BB IRQ full handler is required for visible advertising.
- Main-loop capture after `BB+0x00=2` is unreliable because the BB IRQ path owns the state transition.
- Mode bit restoration around TX/RX is subtle (`LLE+0x00 bits[8:7]` and `bits[13:12]`).
- SDI in the hot path changes timing and must move to a deferred ring.

The next code step should move from ad-hoc probe logic to the Phase A/B LL task skeleton, then reimplement CONNECT_IND capture inside that structured runtime. This keeps future GATT/GAP work from inheriting probe-only control flow.

### Hardware evidence to carry into Phase C

The task #68 probe state was preserved on branch `bad/task68-rx-probe-diagnostic-20260510-131708` before starting the Phase A skeleton.

Facts from the probe:

- `gBleIPPara[4]` state machine follows `0x80` (TX armed) -> `0xC0` (`.L6` TX fire) -> `1` (bit4/7 TX done) -> `0x80` (re-arm).
- TX done is surfaced through BB status bit4/bit7 in the full BB IRQ path.
- `BLE_LLE.ctrl()` / LLE+0x00 mode bits are split: bits[8:7] represent TX mode, bits[13:12] represent RX mode. Enabling both mode groups at once can leave LLE state stuck at `108`.
- `adv_tx_burst` already prepares several RX inputs for the turnaround path: `BB+0x74` DMA buffer, `LLE+0x08` ADV access address, `LLE+0x04` CRC init, and `LLE+0x00 bits[5:0]` channel.
- The next CONNECT_IND attempt should be an ISR one-shot: save `ctrl`/RFEND path state, clear bits[8:7], set bits[13:12]=01, arm RX, wait about 600 us, snapshot DMA to a deferred buffer, then restore saved TX mode state before the next ADV event.

Scope contract: this document is a scheme and migration boundary. Runtime skeleton, CONNECT_IND capture, and GAP/GATT implementation each need their own task and gate.

## Verification strategy

Every phase needs an automated, repeatable gate:

1. `cargo build --release --bin ...`
2. flash with the reset mode learned from task #65
3. run macOS CoreBluetooth scan/connect script
4. collect SDI log
5. collect deferred ring trace
6. classify:
   - ADV visible
   - SCAN_RSP observed
   - CONNECT_IND observed
   - LinkEstablished event emitted
   - connection event maintained

Cold-start RF behavior needs multi-run statistics. Use at least 5 cold runs before calling a radio behavior deterministic.

Baseline gates by example:

| Example | Build gate | Hardware gate |
|---|---|---|
| `ble_tx_adv_ch37` | `cargo build --release --bin ble_tx_adv_ch37` | CoreBluetooth scan for `cba`, warm + WCH-Link 3V3 power-cycle + SRAM scrub→reset, cba >= 5/30s |
| `ble_rx_listener` | `cargo build --release --bin ble_rx_listener` | SDI heartbeat `frames>0`, `stuck=0`, warm + power-cycle + scrub→reset when judging regressions |
| `ble_peripheral_phase1_adv` | `cargo build --release --bin ble_peripheral_phase1_adv` | macOS scan sees `Simple`, Android scan sees `Simple`, CoreBluetooth connect attempt produces CONNECT_IND or classified RX snapshot |

Iron Law #38 applies to changes that touch BLE init, LL/RF registers, RAM contracts, and ISR scheduling: run warm, power-cycle, and scrub→reset conditions before closing the task.

Suggested automation artifacts:

- keep `tools/ble_phase1_run_gate.sh` as the Phase 1 peripheral gate
- add a `--scan-only` mode for ADV visibility
- add a `--connect` mode for CONNECT_IND
- write SDI hot-path records into a fixed ring and print after the radio window
- save every gate under `/tmp/task<N>_<phase>_<timestamp>_{sdi,mac}.log`

## Open questions

1. Exact EVT/libwchble `ll_advertise_legacy_rx` register sequence for connectable ADV remains the highest-value reverse-engineering target.
2. BB IRQ bits for RX done in connectable advertising need a focused trace with deferred logging.
3. Connection-event timer ownership needs confirmation: which parts are BB hardware timer reloads and which parts can be Embassy `Timer::at`.
4. Connection-event anchor priority needs validation. Hardware timers own the anchor, but `ble_ll_task` may eventually need `InterruptExecutor` priority if thread-mode scheduling latency becomes visible.
5. `ConnectionContext` ownership starts as a single-link `ble_ll_task`-owned object. Multi-link support will need an indexed pool, probably `Mutex<[Option<ConnectionContext>; N]>` or an equivalent lock-free pool.
6. Security/GAPBondMgr stays outside the first Rust-only milestone.

## Evidence table

| Claim | Evidence | Tier | Confidence |
|---|---|---|---|
| TMOS event tasks are 16-bit event masks and a cooperative dispatcher | `wchble.h` typedefs and APIs; EVT `Peripheral_ProcessEvent()`; elec-docs `07-tmos-internals.md` | primary source + reverse docs | high |
| WCH PeripheralRole startup uses GAPRole params, GAP params, services, callbacks, then `SBP_START_DEVICE_EVT` | EVT `Peripheral_Init()` | primary source | high |
| Link established callback updates connection list and starts periodic/param/RSSI timers | EVT `Peripheral_LinkEstablished()` | primary source | high |
| TMOS public API uses 625 us units for `tmos_start_task` and `TMOS_GetSystemClock` | elec-docs `WCHBLE.md` API section 8.1; `wchble.h` `SYSTEM_TIME_MICROSEN` | PDF-derived docs + header | high |
| TMOS task/event/message internals are task table + event bitmask + timer list + message queue | elec-docs `07-tmos-internals.md` | reverse docs | medium-high |
| ch58x-hal can bridge WCH TMOS into Embassy via `PubSubChannel` and periodic `TMOS_SystemProcess()` | `ch58x-hal/src/ble/mod.rs`, `examples/ble-peripheral.rs` | local source | high |
| Embassy `Signal`/`Channel` fit ISR-to-task and task queue delivery | Embassy sync docs + current `ble_rx_listener` `Signal` use | official docs + local source | high |
| Embassy timers fit GAP/app timers | Embassy time docs + ch32-hal SysTick driver | official docs + local source | high |
| BB/LLE hardware timers should own radio windows | Current task #68 timing failures and BLE timing requirements | hardware evidence + protocol constraint | medium |
| Phase A/B skeleton should precede deeper GATT/GAP implementation | task #68 probe instability plus EVT layered model | engineering inference | medium |

## Recommended next action

Create the Phase A runtime skeleton and move Phase 1 connectable ADV into it. Keep task #68 probe changes as diagnostic worktree state until the skeleton can reproduce current ADV visibility. Then reattempt CONNECT_IND capture through the structured LL task and ISR event path.
