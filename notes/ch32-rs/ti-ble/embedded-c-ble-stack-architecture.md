# Embassy BLE Stack Logic-Port Architecture Plan

Status: planning draft v3.2, 2026-05-18. Review updates from Vega
`2560f004` / `56b55bd3`, Lucy `9c0acdfc` / `74939ed8`, and their
second-pass nits are incorporated.

Purpose: collect the existing CH32V208 BLE research material and turn it into an
architecture plan for a Rust/Embassy-owned BLE stack. TI/WCH/TMOS/EVT material is
used as behavioral source input. The implementation target is native Rust modules:
Embassy tasks, `Signal`/`Channel`, static state machines, explicit ISR boundaries,
and Rust-owned Link Layer / Host logic.

Design lock from Andelf, 2026-05-17:

1. Vendor C examples, WCH binary libraries, and ROM hooks are reference material.
2. The stack should port the protocol and scheduler logic into our codebase.
3. The runtime should fit the current Embassy framework.
4. FFI/lib calls can exist as probe or comparison scaffolding, while production
   behavior should live in Rust-owned modules.

Target: one legacy BLE peripheral first. The first production slice should support
ADV_IND, SCAN_REQ to SCAN_RSP, CONNECT_IND accept, one peripheral connection,
ATT/GATT SimpleProfile-style services, and notifications. Multi-connection,
encryption, bonding, extended advertising, and LE data length updates are follow-up
milestones.

## 1. Source Document Inventory

### 1.1 Local planning and validation docs

| Doc | Use in this plan |
| --- | --- |
| `README.md` | Canonical retrieval index and current BLE locks. |
| `ble-followup-roadmap.md` | Route from ADV and SCAN_RSP to scheduler and connection work. |
| `time-event-model.md` | TMOS event dispatch, `LL_ProcessEvent`, 625 us tick model, event-close semantics. |
| `peripheral-flow-deep-dive.md` | WCH Peripheral and TI Simple Peripheral semantic map. |
| `evt-broadcaster-ti-flow.md` | EVT Broadcaster and Peripheral comparison; cold ADV vs SCAN_RSP warm path correction. |
| `scan-rsp-implementation-spec.md` | SCAN_RSP warm turnaround contract and hardware gates. |
| `connectable-adv-baseline-spec.md` | Legacy `ADV_IND` proof path before SCAN_RSP and connection work. |
| `task79-attempt-log.md` | Canonical pitfall ledger for failed RF/LL attempts. |
| `../physical-debugging-guide.md` | Flashing, reset boundary, Wlink/OpenOCD/GDB, and pcap evidence procedures. |

### 1.2 Local reverse-engineering docs

| Doc | Use in this plan |
| --- | --- |
| `../libwchble-tmos-ll-disasm.md` | BB/LLE IRQ dispatch, W1C rules, `ll_tx_wait_finish`, RX completion setters. |
| `../ble-registers.md` | BB/LLE/RFEND address map and register semantics. |
| `../peripheral-role-embassy-migration.md` | EVT PeripheralRole app flow and TMOS timer/task contracts. |
| `../tmos-embassy-task-migration.md` | TMOS task/event/message model and Rust/Embassy mapping. |
| `../lib-dependency-removal.md` | Prior lib removal decisions and ROM/lib boundary findings. |

### 1.3 LL disassembly notes (in-tree, behavioral reference only)

These focused notes were originally produced in Lucy's agent workspace and
migrated into this directory under task #90 (`#ch32-rs:decce4f6`,
branch `lucy/disasm-migration`). Each note carries a provenance footer
(origin path, migration date, responsible agent, and the
behavioral-reference-only boundary). Treat them as observed-behavior
references — implementation branches may cite them for control-flow
provenance but must not link `libwchble` symbols or call ROM helpers as
a production fallback (see §2.5 anti-import boundary):

| In-tree doc | Required facts |
| --- | --- |
| [`ll-advertise-tx-disasm.md`](./ll-advertise-tx-disasm.md) | Cold ADV TX sequence, PDU stamping, channel writes. |
| [`ll-tx-completion-disasm.md`](./ll-tx-completion-disasm.md) | TX done, RX wait boundaries, event close. |
| [`ll-adv-scheduler-disasm.md`](./ll-adv-scheduler-disasm.md) | ADV scheduler, channel hop, interval/random delay. |
| [`ll-process-event-minipass.md`](./ll-process-event-minipass.md) | `LL_ProcessEvent` event-mask dispatch table. |
| [`ll-rx-ingress-disasm.md`](./ll-rx-ingress-disasm.md) | SCAN_REQ and CONNECT_IND split. |
| [`ll-phy-preflight-disasm.md`](./ll-phy-preflight-disasm.md) | PHY setup, warm SCAN_RSP kick, RF constants. |
| [`ll-slave-init-disasm.md`](./ll-slave-init-disasm.md) | CONNECT_IND first hop and slave init. |
| [`ll-advertise-filter-disasm.md`](./ll-advertise-filter-disasm.md) | Filter policy, resolving-list mutation, connect/scan provenance. |

## 2. Design Goals and Boundaries

### 2.1 Goals

1. Reimplement the useful TI/WCH/TMOS control flow as Rust/Embassy logic:
   cooperative event dispatch, event masks, bounded message queues, timers,
   explicit state machines, and deterministic ISR boundaries.
2. Keep the protocol source of truth in Rust modules:
   advertising, SCAN_RSP, connection events, L2CAP, ATT, GATT, and GAP role.
3. Centralize radio ownership so ADV, SCAN_RSP, connection events, and ATT/GATT
   payloads share one scheduler and one hardware arbitration point.
4. Keep all protocol transitions observable through pcap, trace rings, event logs,
   and small hardware-state snapshots.

### 2.2 Reference-to-implementation mapping

| Reference concept | Rust/Embassy implementation concept |
| --- | --- |
| `TMOS_ProcessEventRegister` | static task registry or enum-dispatch task table. |
| `tmos_set_event` | atomic OR with `AtomicU16::fetch_or(Ordering::Release)` plus `Signal`. |
| `SYS_EVENT_MSG` | per-task bounded `Channel<...Msg, N>` (`LlMsg` / `L2capMsg` / `AttMsg` / `GapMsg` / `AppMsg`) and task event bit. |
| `tmos_start_task` | Embassy timer adapter using 625 us logical ticks. |
| `LL_ProcessEvent` | `ll_task()` event-bit dispatcher. |
| `ll_advertise_process` | Rust `AdvEvent` synchronous state machine inside LL task. |
| `ll_tx_wait_finish` | Rust radio wait helper with BB/LLE completion sources. |
| `GAPRole_Peripheral*` | Rust `GapPeripheral` state machine and callbacks. |
| `GATTServApp_*` | Rust ATT server plus static GATT database. |

### 2.3 First supported feature set

| Layer | First milestone |
| --- | --- |
| Radio HAL | BB/LLE/RFEND init, IRQ status handling, PHY TX/RX setup, calibration sanity. |
| Link Layer | Legacy advertising, SCAN_REQ/SCAN_RSP, CONNECT_IND accept, one connection. |
| L2CAP | Fixed channels: ATT `0x0004`, LE signaling `0x0005`. |
| ATT | MTU 23, read, read by type, read by group type, write request, write command. |
| GATT | Generic Access, Generic Attribute, and one SimpleProfile-like service. |
| GAP role | Peripheral role: start advertising, connected, disconnected, restart advertising. |
| App | Simple event callbacks and one notification path. |

### 2.4 Deferred feature set

| Feature | Reason |
| --- | --- |
| Pairing, bonding, encryption | Requires SM, key storage, and security policy. |
| Multi-connection | First connection engine should prove one link and stable timing first. |
| Extended advertising | Current evidence base is legacy ADV focused. |
| Data length update and 2M/Coded PHY | First link can use LE 1M and MTU 23. |
| GATT long writes and prepare/execute write | Adds storage and transaction complexity after basic ATT works. |

### 2.5 Anti-import boundary

These vendor behaviors stay out of the first Rust/Embassy implementation:

1. closed-source WCH scan-list / resolving-list internals.
2. closed-source WCH SM/SMP and bonding implementation.
3. vendor RNG / crypto callback hooks as opaque runtime dependencies.
4. direct libwchble or ROM function calls as production fallbacks.
5. undocumented global mutable state that cannot be mapped to a named Rust state
   machine field.

## 3. Proposed Rust/Embassy Layering

```text
examples/ch32v208/ble_peripheral_embassy.rs
  |
src/ble/stack/profile/
  |
src/ble/stack/host/{gap,gatt,att,l2cap}
  |
src/ble/stack/ll/{adv,scan_rsp,conn,filter,radio}
  |
src/ble/stack/osal/{event,timer,msg,task}
  |
src/ble/stack/hw/{regs,irq} + existing src/ble/{bb,lle,rfend}
  |
ch32v208 hardware
```

### 3.1 Module layout

Recommended Rust module split:

```text
src/ble/stack/
  mod.rs
  config.rs
  types.rs

  osal/
    mod.rs
    event.rs
    timer.rs
    msg.rs
    task.rs

  hw/
    mod.rs
    irq.rs
    regs/
      mod.rs
      bb.rs
      lle.rs
      rfend.rs

  ll/
    mod.rs
    radio.rs
    adv.rs
    scan_rsp.rs
    conn.rs
    filter.rs
    channel_map.rs

  host/
    mod.rs
    l2cap.rs
    att.rs
    gatt.rs
    gatt_db.rs
    gap_peripheral.rs
    hci.rs          # placeholder, deferred until one peripheral link works

  profile/
    mod.rs
    simple_profile.rs

  trace/
    mod.rs
    rings.rs
    dump.rs
    schema.rs
```

Rules:

1. `hw/regs/` owns raw BB/LLE/RFEND read/write helpers and W1C mask helpers.
   It has no scheduler policy.
2. `ll/` owns BLE controller state and all radio scheduling decisions.
3. `host/` owns L2CAP, ATT, GATT, GAP role, and app-visible events.
4. `profile/` owns service attributes and profile callbacks.
5. `osal/` owns the TMOS-like behavior implemented with Embassy primitives.
6. `trace/` owns diagnostic ring schemas, dump layouts, and detectability tags.
7. Cross-layer calls go downward through explicit APIs and upward through messages.
8. `ll/mod.rs` is the public aggregator for LL APIs; internal items use
   `pub(crate)` or narrower visibility.

`ll/radio.rs` owns sequencing policy: cold vs warm kick selection, radio-op
arbitration, IRQ-summary consumption, and state-machine routing. Control-register
writers should require a `RadioOwnerToken` whose constructor is private to
`ll/radio.rs`; this makes accidental BB/LLE/RFEND control writes from host/app
code a compile-time error.

`osal/task.rs` owns task IDs and dispatch entry points. `osal/event.rs` owns the
event-bit storage and wake primitive. Embassy task spawning stays in the example
or top-level stack init, not inside `osal/`.

### 3.2 Cross-layer message dictionary

Initial message vocabulary:

| Direction | Message | Payload |
| --- | --- | --- |
| GAP -> LL | `StartAdv` | advertising params, adv data handle, scan response handle. |
| GAP -> LL | `StopAdv` | reason. |
| GAP -> LL | `Disconnect` | connection handle, reason. |
| LL -> GAP | `LinkEstablished` | handle, peer address, interval, latency, timeout. |
| LL -> GAP | `LinkTerminated` | handle, reason. |
| LL -> Host | `L2capDataIn` | handle, payload handle. |
| Host -> LL | `L2capDataOut` | handle, payload handle. |
| ATT/GATT -> App | `AttrRead` / `AttrWrite` / `NotifyComplete` | handle, status, payload handle when needed. |

Control messages go through per-task channels. Packet payload bytes live in static
pools and move by handle.

## 4. Embassy Execution Model

### 4.1 Cooperative event engine

Use a TMOS-like event engine implemented with Embassy:

```rust
use core::sync::atomic::{AtomicU16, Ordering};

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum BleTaskId {
    Ll,
    L2cap,
    Att,
    Gap,
    App,
}

pub type BleEvents = u16;

pub const EVT_SYS_MSG: BleEvents = 0x8000;

pub struct TaskEvents {
    bits: AtomicU16,
    wake: Signal<CriticalSectionRawMutex, ()>,
}

impl TaskEvents {
    pub const fn new() -> Self {
        Self {
            bits: AtomicU16::new(0),
            wake: Signal::new(),
        }
    }
}

static TASK_EVENT_TABLE: [TaskEvents; 5] = [
    TaskEvents::new(),
    TaskEvents::new(),
    TaskEvents::new(),
    TaskEvents::new(),
    TaskEvents::new(),
];

pub fn task_events(task: BleTaskId) -> &'static TaskEvents {
    &TASK_EVENT_TABLE[task as usize]
}

pub fn set_event(task: BleTaskId, event: BleEvents) {
    let task = task_events(task);
    task.bits.fetch_or(event, Ordering::Release);
    task.wake.signal(());
}

pub async fn run_task_loop(task: BleTaskId) -> ! {
    let events = task_events(task);
    loop {
        let bits = events.bits.swap(0, Ordering::Acquire);
        if bits == 0 {
            events.wake.wait().await;
            continue;
        }
        dispatch_bits(task, bits).await;
    }
}
```

`AtomicU16` is used for task event bits because TMOS event state is one word and
ISR paths should avoid a global critical-section enter/exit on every event OR.
The release/acquire pair preserves writes that precede the event signal. New bits
set during `dispatch_bits()` remain in the atomic word and are drained by the next
loop iteration.

The synchronization chain is explicit: a producer updates shared summaries, ORs
the task bits with `fetch_or(Ordering::Release)`, and then calls `Signal::signal`.
`Signal::wait().await` provides the wake edge; after it returns, the task loops
back to `swap(0, Ordering::Acquire)` before dispatching. Dispatch never reads a
cached post-wait bit value directly, so spurious wakes and coalesced wakes still
flow through the same atomic drain path.

The runtime shape:

```rust
#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    board_init();
    ble_hw_init();
    ble_stack_init();

    spawner.spawn(ll_task()).unwrap();
    spawner.spawn(l2cap_task()).unwrap();
    spawner.spawn(att_task()).unwrap();
    spawner.spawn(gap_task()).unwrap();
    spawner.spawn(app_task()).unwrap();

    core::future::pending().await
}
```

Event engine invariants:

1. Event bits are ORed with `fetch_or(Ordering::Release)` before `Signal`.
2. Each task drains all currently set bits with `swap(0, Ordering::Acquire)`
   before awaiting the next `Signal`.
3. Timer expiry sets event bits.
4. Message delivery sets `EVT_SYS_MSG`.
5. Long protocol work is split by state-machine boundaries and event bits.
6. Radio-critical sub-millisecond timing stays in LL/radio helpers and hardware
   timers; app and host tasks never busy-wait on radio state.

### 4.2 Task split

| Task | Event examples | Owner |
| --- | --- | --- |
| `ll_task` | ADV start, ADV next event, radio IRQ summary, connection event. | Link Layer controller. |
| `l2cap_task` | RX ACL data, TX complete, signaling request. | L2CAP. |
| `att_task` | ATT request received, response ready, notification ready. | ATT/GATT server. |
| `gap_task` | start peripheral, link established, link terminated, param update. | GAP role. |
| `app_task` | profile changed, periodic notify, app command. | Application/profile. |

### 4.3 Message queue

Use one bounded Embassy channel per task:

```rust
static LL_MSG_CH: Channel<CriticalSectionRawMutex, LlMsg, 8> = Channel::new();
static L2CAP_MSG_CH: Channel<CriticalSectionRawMutex, L2capMsg, 8> = Channel::new();
static ATT_MSG_CH: Channel<CriticalSectionRawMutex, AttMsg, 8> = Channel::new();
static GAP_MSG_CH: Channel<CriticalSectionRawMutex, GapMsg, 8> = Channel::new();
static APP_MSG_CH: Channel<CriticalSectionRawMutex, AppMsg, 8> = Channel::new();
```

Runtime routing uses typed per-task message enums (`LlMsg`, `L2capMsg`,
`AttMsg`, `GapMsg`, `AppMsg`). If a unified message enum is useful for trace
output, keep it in `trace/schema.rs` as `BleTraceMsg` so runtime routing remains
per-task.

Memory rules:

1. Avoid heap allocation in the first milestone.
2. Control messages use per-task bounded channels.
3. ACL/ATT payload bytes use fixed packet pools; messages carry payload handles.
4. Queue overflow uses `try_send`, increments a counter, and sets a fault event.
5. Channel capacity is sized from a documented burst budget before implementation.
   Start at 8 control messages per task; raise only with a RAM budget update.
6. ISR paths use `Signal` or `try_send` only.
7. ISR-to-task data uses a separate small ring or `Signal` payload summary.

## 5. Interrupt Handling Architecture

### 5.1 IRQ inputs

Current CH32V208 BLE evidence uses:

| IRQ | Role |
| --- | --- |
| BB IRQ 63 | BB status, TX advance markers, W1C `BB+0x38`. |
| LLE IRQ 64 | LLE status, RX/TX completion flags, W1C `LLE+0x08`. |

Both IRQ paths are hard real-time entry points. They perform bounded work,
acknowledge hardware, capture compact evidence, and wake the LL task.

### 5.2 ISR contract

Recommended ISR steps:

1. Read raw status registers at IRQ entry.
2. Copy raw status into an IRQ summary struct or small ring.
3. Acknowledge W1C bits exactly once.
4. Update minimal volatile flags required by tight hardware waits.
5. Signal `ll_task`.
6. Return.

Hard rules:

1. ISR paths may update summary flags and acknowledge W1C status registers.
2. ISR paths must not perform read-modify-write on radio control registers:
   BB control range, LLE control range, or RFEND control range.
3. Radio control RMW belongs to `ll/radio.rs` and requires `RadioOwnerToken`.
4. ISR paths must not call `send().await`; they use `Signal`, `try_send`, or an
   IRQ ring push with explicit overflow counters.

Example shape:

```rust
#[no_mangle]
unsafe extern "C" fn __qingke_rt_BB() {
    let statr = bb::read_statr();
    let lle_irq = lle::read_irq_status();
    let ack = decode_bb_ack_mask(statr);

    IRQ_RING.push(IrqSample::bb(statr, lle_irq, fine_time_now()));

    if ack != 0 {
        bb::write_statr_w1c(ack);
    }

    LL_IRQ_FLAGS.update_from_bb(statr);
    LL_IRQ_SIGNAL.signal(());
}

#[no_mangle]
unsafe extern "C" fn __qingke_rt_LLE() {
    let irq = lle::read_irq_status();
    let ack = decode_lle_ack_mask(irq);

    IRQ_RING.push(IrqSample::lle(irq, fine_time_now()));

    if ack != 0 {
        lle::write_irq_w1c(ack);
    }

    LL_IRQ_FLAGS.update_from_lle(irq);
    LL_IRQ_SIGNAL.signal(());
}
```

`LL_IRQ_SIGNAL` is reserved for LL-internal radio wait helpers such as
`tx_wait_done()`. Task-level dispatch still uses `task_events(BleTaskId::Ll)`:
an IRQ that needs the LL event loop sets an LL event bit and wakes the task event
table entry.

### 5.3 W1C rule

W1C status-source fields require pre-W1C capture. A post-W1C dump loses the
source bit that explains the IRQ.

Applied rules:

1. BB `+0x38` and LLE `+0x08` status fields are captured before W1C.
2. Any diagnostic ring column derived from W1C status is filled at IRQ entry.
3. Protocol state reads can happen later, while W1C source truth belongs at IRQ
   entry.
4. Multi-symbol host dumps use one continuous halt window with `--no-detach`
   per PF-21 in `task88-phase3/sectionX_h_final.md`.

### 5.4 Diagnostic ring schema

Diagnostic rings used for mechanism discrimination must declare the detectability
class for every column:

```rust
pub enum Detectability {
    WithinRunPattern,
    CrossRunComparisonOnly,
    IrqEntryPreW1cOnly,
    InvariantControl,
}

pub struct TraceColumnMeta {
    pub name: &'static str,
    pub source: TraceSource,
    pub detectability: Detectability,
    pub reset_label_required: ResetLabel,
}
```

`TraceSource` and `ResetLabel` are defined in `trace/schema.rs`.

Rules:

1. W1C status-source columns use `IrqEntryPreW1cOnly`.
2. F4-style post-IRQ RFEND stable-state columns use `CrossRunComparisonOnly`.
3. R10-like cal table controls use `InvariantControl`.
4. ADV event counters and IRQ-ring sequence numbers are examples of
   `WithinRunPattern`.
5. Phase reports must state whether a signal is within-run-detectable or requires
   Run-A vs Run-B cross-run comparison.
6. `IrqEntryPreW1cOnly` is hard detectability: missing the IRQ-entry capture
   permanently destroys that signal for the run. The other classes can usually
   be sampled again in a rerun or cross-run pair.

### 5.5 ISR and protocol boundary

Protocol parsing stays in Embassy tasks. The LL task owns:

1. SCAN_REQ parse.
2. CONNECT_IND parse.
3. connection context allocation.
4. ATT/L2CAP data dispatch.
5. GAP/app messages.

The ISR may execute tiny hardware handoff operations when WCH timing requires
sub-millisecond work. Those operations need named helpers and paired pcap/trace
evidence.

## 6. Global Radio and TX Scheduler

### 6.1 Single radio owner

The stack should expose one internal radio operation API:

```rust
pub enum RadioOpKind {
    AdvTx,
    AdvRxWindow,
    ScanRspTx,
    ConnEvent,
}

pub struct RadioOp {
    pub kind: RadioOpKind,
    pub channel: u8,
    pub deadline_625us: u32,
    pub tx_payload: Option<PayloadHandle>,
    pub rx_pool: Option<RxPoolId>,
    pub conn_init_first_anchor_us: Option<u32>,
}

pub fn radio_request(op: RadioOp) -> Result<(), RadioBusy>;
pub fn radio_cancel_current();
pub fn radio_process_event();
```

Only the LL radio scheduler writes BB/LLE/RFEND TX/RX control registers. Host and
app layers request protocol actions through LL APIs and messages.

RX buffers are owned by the radio scheduler static pool. Callers never lend stack
buffers across an `await` point. On RX completion, LL transfers a `PayloadHandle`
to L2CAP/ATT or returns the buffer to the pool.

`PayloadHandle` owns a checked-out static pool slot. `Drop` returns the buffer
to its source pool; upper layers should call explicit `release()` before long
`await` paths when deterministic pool pressure matters.

`ll_tx_wait_finish` migration plan:

1. Phase 2/3 can use a bounded synchronous poll helper for bring-up.
2. The long-term implementation is `async fn tx_wait_done()` awakened by BB/LLE
   IRQ summaries.
3. Phase 4 should retire busy-polling from normal SCAN_RSP/connection paths.

### 6.2 Advertising event sequence

Legacy advertising event shape:

```text
adv_event_start
  ch37 ADV_IND TX
  ch37 RX window
    SCAN_REQ -> SCAN_RSP warm TX
    CONNECT_IND -> connection transition
  ch38 ADV_IND TX
  ch38 RX window
  ch39 ADV_IND TX
  ch39 RX window
adv_event_closed(remaining_us)
```

First implementation can use deterministic `[37, 38, 39]` channel order and the
known 625 us time base.

### 6.3 Cold ADV TX vs warm SCAN_RSP TX

Current evidence separates two hardware kicks:

| Path | Hardware shape | Use |
| --- | --- | --- |
| Cold ADV TX | `LLE[0] = 2` | Initial/per-channel ADV transmit. |
| Warm SCAN_RSP TX | `BB[0] |= 0x800000`, `BB[44] &= ~0x3` | Fast RX-to-TX response after SCAN_REQ. |

Source anchors: `../libwchble-tmos-ll-disasm.md` and the SCAN_RSP warm path
notes referenced from `scan-rsp-implementation-spec.md`.

These paths should be separate Rust helpers:

```rust
pub fn kick_adv_tx(pdu: &AdvPdu, channel: u8);
pub fn kick_scan_rsp_tx(pdu: &ScanRspPdu, channel: u8);
```

The separation keeps cold ADV behavior and SCAN_RSP warm-turnaround behavior
reviewable as distinct state-machine edges.

### 6.4 TX queue ownership

Use per-layer queues that converge at LL:

| Producer | Queue item | Scheduler action |
| --- | --- | --- |
| GAP role | start/stop advertising | enable/disable ADV state machine. |
| ATT/GATT | notification data | enqueue connection data PDU. |
| L2CAP | signaling response | enqueue connection data PDU. |
| LL control | empty PDU, LL control PDU | schedule inside connection event. |

Connection event TX priority:

1. Required LL control PDU.
2. ATT/L2CAP data PDU.
3. Empty PDU keepalive.

Advertising event priority:

1. In-flight SCAN_RSP or CONNECT_IND transition.
2. Current channel close.
3. Next channel.
4. event close and interval re-arm.

## 7. State Machines

### 7.1 GAP peripheral role

```text
GapIdle
  -> GapAdvertising
  -> GapConnecting
  -> GapConnected
  -> GapDisconnecting
  -> GapAdvertising
```

Events:

| Event | Action |
| --- | --- |
| `GapStartDevice` | configure adv set, start LL advertising. |
| `GapScanReq` | notify app if enabled. |
| `GapLinkEstablished` | allocate linkDB entry, stop advertising, start app timers. |
| `GapLinkTerminated` | stop link timers, restart advertising by policy. |
| `GapParamUpdateReq` | enqueue LE signaling response or defer. |

### 7.2 Advertising LL state

```text
AdvDisabled
  -> AdvEnabled
  -> AdvEventStart
  -> AdvTx
  -> AdvRxWindow
  -> AdvScanRspTx
  -> AdvConnectIndRx
  -> AdvEventClosed
```

Important fields:

```rust
pub struct AdvContext {
    pub enabled: bool,
    pub own_addr: [u8; 6],
    pub adv_data: [u8; 31],
    pub adv_data_len: u8,
    pub scan_rsp_data: [u8; 31],
    pub scan_rsp_len: u8,
    pub interval_625us: u16,
    pub channel_map: u8,
    pub current_channel: u8,
    pub next_event_tick: u32,
    pub event_counter: u32,
}
```

### 7.3 Connection LL state

```text
ConnUnused
  -> ConnInitPending
  -> ConnActive
  -> ConnTerminating
  -> ConnUnused
```

Minimum fields:

```rust
pub struct ConnContext {
    pub handle: u16,
    pub access_addr: u32,
    pub crc_init: u32,
    pub channel_map: [u8; 5],
    pub hop_increment: u8,
    pub conn_interval_1250us: u16,
    pub slave_latency: u16,
    pub supervision_timeout_10ms: u16,
    pub event_counter: u16,
    pub last_unmapped_channel: u8,
    pub sn: bool,
    pub nesn: bool,
    pub md: bool,
    pub anchor_tick_625us: u32,
}
```

First connection scope:

1. Accept one central.
2. Schedule connection events from the CONNECT_IND parameters.
3. Send and receive empty PDUs.
4. Carry ATT data PDU when queues are ready.
5. Enforce supervision timeout and return to advertising after disconnect.

The first anchor calculation must preserve CONNECT_IND `transmitWindowOffset` and
`transmitWindowSize`/delay semantics. `RadioOp::conn_init_first_anchor_us` is the
handoff field from CONNECT_IND parsing into the radio scheduler.

### 7.4 ATT/GATT state

ATT server can be table-driven:

```rust
pub struct GattAttr {
    pub handle: u16,
    pub uuid: Uuid,
    pub permissions: AttrPerm,
    pub value: AttrValue,
}

pub enum AttrValue {
    Const(&'static [u8]),
    Mutable(&'static Mutex<CriticalSectionRawMutex, [u8; 32]>),
    Callback(AttrCallbacks),
}
```

First GATT database:

1. Generic Access service.
2. Generic Attribute service.
3. SimpleProfile service UUID `0xFFF0`.
4. char1 read/write.
5. char3 write.
6. char4 notify plus CCCD.

`host/gatt.rs` owns runtime request handling against this table. `host/gatt_db.rs`
owns static attribute schema and generated handles.

## 8. Timer and Clock Model

### 8.1 Units

| Unit | Use |
| --- | --- |
| 625 us tick | TMOS-compatible event timers and ADV interval accounting. |
| SysTick fine counter | measurement and debug timestamps. |
| BB/LLE hardware timer | T_IFS, RX window, connection event anchor, hardware waits. |
| Pcap timestamp | final air-side protocol evidence. |

### 8.2 Embassy timer adapter

Use a small adapter layer for TMOS-like 625 us timers:

```rust
pub fn ticks_625us_to_duration(ticks: u32) -> Duration {
    Duration::from_micros(ticks as u64 * 625)
}

pub struct BleTimerEntry {
    pub generation: u32,
    pub task: BleTaskId,
    pub event: BleEvents,
    pub deadline_tick_625us: u32,
    pub reload_period_tick_625us: Option<u32>,
}
```

Rules:

1. Timer expiry sets event bits.
2. Reload timers use absolute next deadlines to avoid cumulative drift.
3. Stop increments generation. A timer completion carries `(handle, generation)`;
   it fires the event only when the stored generation still matches.
4. The backing Embassy time driver instance is TBD for CH32V208; Phase 1 must
   record whether it uses SysTick or a TIMx instance.
5. Radio-critical timing uses hardware timers and LL state, then reports into the
   event engine at safe boundaries.

## 9. Implementation Phases

### Phase 0 - Documentation and Rust skeleton

Deliverables:

1. Start migration of required external disassembly notes into this repo. This
   can run in parallel with the skeleton work.
2. Create `src/ble/stack/` skeleton with modules listed in §3.
3. Define common types, event masks, and compile-time config.
4. Add feature flag `ble-embassy-stack`, default off.
5. Add host-side tests for pure state-machine functions where `std` is available.
6. Isolate pure logic from CH32 target code with `#[cfg(test)]` or host-compatible
   modules so event/timer tests run on the host.

Gate:

1. Existing BLE examples still compile with `ble-embassy-stack` disabled.
2. New `ble_peripheral_embassy` skeleton binary compiles with
   `--features ble-embassy-stack`.
3. No WCH BLE library symbols are required by the new modules.
4. Phase 0 skeleton work does not depend on completing the disassembly-note
   migration. Phase 2 radio implementation may cite a disassembly fact only
   after that note has been migrated and provenanced in-repo.

### Phase 1 - Embassy event engine and timers

Deliverables:

1. Task event table.
2. Event bit operations.
3. Bounded message channels.
4. 625 us timer adapter.
5. Unit tests on event/timer behavior.

Gate:

1. Synthetic tasks can set, clear, and re-arm events.
2. Timer generation counters prevent stale event firing.
3. Queue overflow is deterministic and counted.

### Phase 2 - Radio HAL and IRQ boundary

Deliverables:

1. BB/LLE/RFEND register access layer.
2. BB/LLE IRQ handlers with pre-W1C capture.
3. IRQ-to-LL summary ring.
4. `--no-detach` host dump recipe baked into debug tooling.

Gate:

1. Hardware init reaches known idle LLE state.
2. IRQ capture ring shows monotonic entries.
3. W1C status-source data is captured before clear.
4. Single-shot cold ADV TX can be kicked and observed before scheduler loop work.

### Phase 3 - Stable ADV_IND with global scheduler

Deliverables:

1. `AdvContext`.
2. ADV_IND PDU builder.
3. Per-channel cold TX path.
4. event close and interval re-arm.

Gate:

1. 60s pcap sees stable target ADV_IND.
2. pcap header type is legacy `ADV_IND`.
3. phone scanner sees expected name/service data.

Bring-up sequence:

1. verify BB/LLE/RFEND init and IRQ ring monotonicity.
2. prove one manual cold ADV TX packet.
3. connect the event engine to per-channel ADV TX.
4. run the 60s ADV_IND gate.

### Phase 4 - SCAN_REQ and SCAN_RSP warm turnaround

Deliverables:

1. RX window after ADV TX.
2. SCAN_REQ parser and address filter.
3. SCAN_RSP PDU builder.
4. warm TX kick helper.

Gate:

1. pcap sees SCAN_REQ from scanner and matching SCAN_RSP from target.
2. turnaround timing is measured around T_IFS.
3. cold ADV TX stays stable for 60s with SCAN_RSP enabled.

### Phase 5 - CONNECT_IND first hop

Deliverables:

1. CONNECT_IND parser.
2. connection context allocation.
3. first connection event timer.
4. empty PDU keepalive.

Gate:

1. pcap shows CONNECT_IND accepted.
2. firmware reports link established.
3. link remains active for at least 30 seconds with empty PDUs.
4. disconnect returns to advertising.

### Phase 6 - L2CAP, ATT, GATT, GAP role

Deliverables:

1. L2CAP fixed channel demux.
2. ATT server core.
3. GATT database and SimpleProfile service.
4. GAP role events and app callbacks.
5. notification path.

Gate:

1. phone connects.
2. service discovery returns expected attributes.
3. read/write operations work.
4. notifications are delivered after CCCD enable.
5. disconnect/reconnect loop is stable.

### Phase 7 - Hardening

Deliverables:

1. reset-boundary matrix.
2. pcap endpoint parser.
3. env-control device workflow.
4. fault counters and debug dump schema.
5. code size and RAM budget report.
6. critical-section and atomic-operation timing report against T_IFS budget.

Gate:

1. power-cycle runs and reset-only runs are labeled separately.
2. failures produce useful counters and trace data.
3. stack survives repeated connect/disconnect cycles.

## 10. Validation and Evidence Rules

### 10.1 Evidence ladder

| Evidence | Use |
| --- | --- |
| build and static analysis | catches API and layout mistakes. |
| on-chip trace ring | proves ISR/event ordering. |
| Wlink memory dump | captures raw state under a continuous halt window. |
| pcap | primary air-side protocol evidence. |
| phone scanner/app | user-visible integration evidence. |
| env-control device | sniffer-side artifact detection. |

Cross-run methodology:

1. Use Run-0 for baseline/regression count.
2. Use Run-A for pcap-confirmed active-state dump.
3. Use Run-B for pcap-confirmed silent-state dump.
4. Mechanism discrimination that uses post-IRQ stable-state columns compares
   Run-A vs Run-B values; within-run uniformity is baseline behavior.
5. Endpoint and silent-transition claims use pcap timestamps as the canonical
   source. Bleak HIT windows can gate a run but should not be the endpoint source.

### 10.2 Reset labels

Use the physical debugging guide labels:

| Label | Meaning |
| --- | --- |
| `POWER_CYCLE` | Vcc off for at least 2s, Vcc on, then flash/run. |
| `RESET_ONLY` | `wlink flash` or `wlink reset` with Vcc continuously applied. |
| `SESSION_NEW` | First run after a fresh operator/session boundary. |
| `SESSION_WARM` | Later run in the same Vcc/session window. |
| `INCONCLUSIVE-reset-boundary` | Result is useful as exploration and requires power-cycle confirmation. |

The reset and session labels follow `../physical-debugging-guide.md`. Phase 7 must
separate `RESET_ONLY` cold-session and warm-session behavior before promoting any
reset-domain inference.

### 10.3 BLE air gates

Minimum pcap checks:

1. AdvA matches target.
2. PDU type matches expected phase.
3. channel sequence is plausible.
4. scan response follows scan request for the same AdvA.
5. connection request is decoded with expected access address, CRC init, hop, and
   interval.
6. env-control keeps broadcasting when used to prove a target-side stop.
7. silent-state claims include env-control or pcap evidence that the sniffer kept
   observing other devices after the target stopped.

### 10.4 Known pitfalls to carry forward

| Pitfall | Rule |
| --- | --- |
| W1C status captured after clear | Capture W1C-source status at IRQ entry. |
| `wlink dump` default detach | Use `wlink halt --no-detach` and per-dump `--no-detach`. |
| macOS/Bleak short-window dips | Use wider windows and pcap timestamp confirmation. |
| cold ADV evidence applied to SCAN_RSP | Keep cold TX and warm RX-to-TX proofs separate. |
| within-run F4 uniformity interpreted as silent signature | Treat post-IRQ stable-state sampling as baseline; use cross-run comparison. |

Full pitfall ledgers:

1. `task79-attempt-log.md` for SCAN_RSP Step 2 and earlier RF bring-up pitfalls.
2. `task88-phase3/sectionX_h_final.md` for PF-21/PF-22 and F4 RESET_ONLY closure.

## 11. Open Design Decisions

| Decision | Current recommendation |
| --- | --- |
| Public API boundary | Start with direct Rust GAP role APIs. Add HCI facade after one peripheral link works. |
| IRQ nesting and atomicity | Keep ISR shared counters protected by critical section or atomic increment; document PFIC priority. |
| Timer source | Use 625 us event ticks for stack timers, hardware timers for radio-critical windows, SysTick for measurement. |
| Memory allocation | Static pools and bounded Embassy channels for first milestone. |
| Security | Defer SM/bonding; return explicit unsupported/security-required responses where needed. |
| GATT database generation | Start with a static Rust table; codegen can follow after API stabilizes. |
| Embassy task granularity | Start with LL/L2CAP/ATT/GAP/App tasks; merge tasks only after latency and stack usage are measured. |
| AFIO / AF mux upstream changes | PR#177 affects peripheral pin traits, while BLE BB/LLE/RFEND MMIO stack work is decoupled. Re-check before adding debug UART/SPI/CAN integration. |

## 12. Immediate Next Actions

1. Review this plan with Andelf and lock the Rust/Embassy module skeleton.
2. Migrate Lucy external disassembly notes that the Rust implementation will cite
   directly.
3. Create `src/ble/stack/` skeleton and host-testable event engine.
4. Implement the Embassy event engine before new radio code.
5. Implement IRQ capture and radio HAL with no host stack dependencies.
6. Reproduce stable ADV_IND with the global scheduler.
7. Add SCAN_REQ/SCAN_RSP, then CONNECT_IND first hop.

The first code milestone should be a buildable Embassy event engine plus
state-machine types. Radio MMIO writes should begin after the IRQ/event boundary
review is accepted.
