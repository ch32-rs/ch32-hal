# Peripheral Flow Deep Dive: WCH CH32V208 vs TI BLE5-Stack

## Purpose

This note maps the full peripheral example path from application boot to advertising, scan response, connection, and event callbacks. The goal is to turn TI BLE5-Stack source/header material and WCH `libwchble.a` disassembly into design input for a pure Rust BLE stack.

The TI material provides semantic structure for most Host, HCI, OSAL, profile, and app paths. WCH uses a closely related naming and layering model, and many TI ROM/header contracts have concrete RISC-V object implementations in WCH `libwchble.a`. The practical workflow is:

1. Read TI source/header for intent and state-machine semantics.
2. Use WCH headers and ROM jump table for ABI shape.
3. Use WCH `.a` disassembly for CH32V208-specific hardware landing points.
4. Reimplement the required behavior in Rust, with TI/WCH used as reference material.

## Evidence Tiers

| Tier | Source | How to use it |
| --- | --- | --- |
| TI source | `simplelink-lowpower-f2-sdk-main/source/ti/ble5stack/{host,hci,osal,...}` and app examples | Semantic reference for Host/HCI/OSAL/app state flow. |
| TI headers / ROM JT | `source/ti/ble5stack/inc`, `source/ti/ble5stack/rom/...` | API contract and function boundary reference when controller code is in ROM. |
| WCH headers / ROM JT | `CH32V20xEVT-2.31/EXAM/BLE/LIB/wchble*.h` | ABI contract used by CH32V208 examples. |
| WCH disassembly | `/tmp/wchble_a/*.dr` from `libwchble.a` | Ground truth for WCH object-code behavior and hardware/register touch points. |
| WCH examples | `EXAM/BLE/Peripheral` | Application-level integration and public callback usage. |

## WCH Peripheral Flow

### 1. Boot and BLE stack init

WCH `Peripheral` enters BLE through `peripheral_main.c`:

- `SystemCoreClockUpdate()`
- `Delay_Init()`
- `WCHBLE_Init()`
- `HAL_Init()`
- `GAPRole_PeripheralInit()`
- `Peripheral_Init()`
- `Main_Circulation()`

Source anchor: `peripheral_main.c:53-65`.

This is the first boundary for a Rust stack: hardware/clock bring-up, BLE controller bring-up, Host/role bring-up, application task registration, and a scheduler loop.

### 2. Application role and GATT setup

`Peripheral_Init()` registers the app task and sets the role/profile state:

- `Peripheral_TaskID = TMOS_ProcessEventRegister(Peripheral_ProcessEvent)`.
- `GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, TRUE)`.
- `GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, scanRspData)`.
- `GAPRole_SetParameter(GAPROLE_ADVERT_DATA, advertData)`.
- `GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN/MAX, advInt)`.
- `GAP_SetParamValue(TGAP_ADV_SCAN_REQ_NOTIFY, ENABLE)`.
- Bond manager parameters.
- `GGS_AddService`, `GATTServApp_AddService`, `DevInfo_AddService`, `SimpleProfile_AddService`.
- `SimpleProfile_RegisterAppCBs`.
- `GAPRole_BroadcasterSetCB`.
- `tmos_set_event(Peripheral_TaskID, SBP_START_DEVICE_EVT)`.

Source anchor: `peripheral.c:207-285`.

Rust design equivalent:

- `BlePeripheralApp` task registration.
- `AdvSet` state with enabled flag, legacy adv data, scan response data, interval, and scan-request notification mask.
- GATT database registration separated from controller radio state.
- App callback queue for scan request, connection state, GATT, RSSI, and periodic work.

### 3. App scheduler entry

`Peripheral_ProcessEvent()` is the WCH application dispatcher:

- `SYS_EVENT_MSG` drains `tmos_msg_receive()` and calls `Peripheral_ProcessTMOSMsg()`.
- `SBP_START_DEVICE_EVT` calls `GAPRole_PeripheralStartDevice(Peripheral_TaskID, &Peripheral_BondMgrCBs, &Peripheral_PeripheralCBs)`.
- Periodic, parameter update, PHY update, and RSSI events are timer/event bits.

Source anchor: `peripheral.c:317-385`.

This aligns with TI's app queue/event loop. Rust should keep app callbacks out of the IRQ path and route them through an internal event queue.

### 4. GAP messages, scan-request notice, and connection state

WCH application messages:

- `Peripheral_ProcessGAPMsg()` handles `GAP_SCAN_REQUEST_EVENT` and `GAP_PHY_UPDATE_EVENT`.
- `Peripheral_ProcessTMOSMsg()` dispatches `GAP_MSG_EVENT` and `GATT_MSG_EVENT`.
- `peripheralStateNotificationCB()` handles role states: started, advertising, connected, connected advertising, waiting, error.
- `Peripheral_LinkEstablished()` starts app timers for periodic work, connection parameter update, and RSSI.
- `Peripheral_LinkTerminated()` stops timers and re-enables advertising through `GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, TRUE)`.

Source anchors: `peripheral.c:396-416`, `428-650`.

Rust design equivalent:

- `scan_req_received(scanner_addr)` event.
- `role_state_changed(new_state, reason)` event.
- `link_established(conn_params)` event.
- `link_terminated(reason)` event with advertising restart policy owned by the role/app layer.

## WCH LL / Scheduler Findings

### 1. Event-register functions are function-pointer setters

WCH exposes:

- `LL_ConnectEventRegister(pfnEventCB connEventCB)` in `wchble.h:2984`, ROM JT slot 26.
- `LL_AdvertiseEventRegister(pfnEventCB advEventCB)` in `wchble.h:2993`, ROM JT slot 135.

Disassembly shows both are three-instruction setters:

```asm
LL_AdvertiseEventRegister:
  auipc a5, %hi(pfnAdvertiseEventCBs)
  sw    a0, %lo(...)(a5)
  ret

LL_ConnectEventRegister:
  auipc a5, %hi(pfnConnectEventCBs)
  sw    a0, %lo(...)(a5)
  ret
```

Disassembly anchors: `/tmp/wchble_a/ll_advertise.dr:5240`, `/tmp/wchble_a/ll_connect.dr:21`.

`LL_Init` clears both callbacks during init, which matches boot-once registration semantics. Source anchor: `/tmp/wchble_a/ll.dr:255`.

Rust design equivalent:

- Internal hooks can use once-cell/static callback semantics.
- Hooks should be state-machine exit points, not public dependency on WCH ROM.

### 2. Advertising event-close hook

`ll_advertise_event_closed()` is the WCH advertising event boundary:

- Clears `gBleLlPara + 0x7c`.
- Writes event status byte at event offset 11.
- Loads `pfnAdvertiseEventCBs`.
- If callback exists, calls `tmos_get_task_timer(gTmosPara[3], 1)`.
- Multiplies remaining timer ticks by 625.
- Tail-calls `cb(timeUs)`.

Disassembly anchor: `/tmp/wchble_a/ll_advertise.dr:594`.

Important semantic correction: `timeUs` is a remaining-time value derived from the TMOS task timer, expressed in microseconds. It is a scheduler-budget hint around the next advertising event, not an absolute timestamp.

The event-close function is a multi-path convergence point. It is called from status-close, scan/receive processing, advertising terminated paths, and `LL_AdvertiseToStandby`. The Rust LL should have a single `adv_event_closed(time_us_remaining)` transition point with all relevant close paths converging there.

### 3. Advertising runtime dispatcher

Key WCH LL functions from `ll_advertise.o`:

| Function | Role |
| --- | --- |
| `LL_AdvertiseEnalbe` | Registers LL task if needed, initializes LL, installs advertise function pointers, and enables advertising machinery. |
| `llAdvertiseCreateCore` | Creates/initializes advertising core state. |
| `llAdvertiseSet` | Applies advertising-set parameters/data into LL state. |
| `llAdvertiseStart` | Starts advertising schedule. |
| `ll_advertise_tx` | TX-side advertising action. |
| `ll_advertise_process` | Central advertising state dispatcher. |
| `ll_advertise_legacy_rx` | RX-side legacy advertising receive path, including scan request and connect indication handling. |
| `ll_advertise_generated_scan_rsp` | Generates scan response payload/action. |
| `ll_advertise_to_connection_state` | Transitions from advertising into connection state. |
| `ll_advertise_event_closed` | Advertising event boundary and callback dispatch. |

Disassembly anchors:

- `LL_AdvertiseEnalbe`: `/tmp/wchble_a/ll_advertise.dr:5251`.
- `ll_advertise_process`: `/tmp/wchble_a/ll_advertise.dr:3777`.
- `ll_advertise_legacy_rx`: `/tmp/wchble_a/ll_advertise.dr:3613`.
- `ll_advertise_event_closed`: `/tmp/wchble_a/ll_advertise.dr:594`.

`ll_advertise_legacy_rx` is the key scan/connection ingress point. It parses packet type and address, runs `ll_advertise_filter`, then branches to connect or scan-response actions.

Rust design equivalent:

- `AdvSet::start()`.
- `adv_tx()`.
- `adv_rx_legacy(packet) -> ScanRequest | ConnectInd | Ignore`.
- `generate_scan_rsp()`.
- `adv_to_connection()`.
- `adv_event_closed()`.

### 4. Connection event hook and TMOS handoff

WCH connect path has a separate event-chain:

- `ll_set_connect_event()` installs `llProcessConnectEvent` as `pfnConnectHandle` and registers it via `TMOS_IrqProcessRegister`.
- `llProcessConnectEvent()` runs from the IRQ-to-TMOS process path and calls lower-level device/event-start helpers.
- `LL_ConnectFreeTmosPrioritID()` loads `pfnConnectEventCBs`, gates on single-connection state (`gBleLlPara + 0xb4 == 1`), calculates a time budget using connection fields, `bleClock_t`, and `fnGetClockCBs`, then calls the registered callback.

Disassembly anchors:

- `ll_set_connect_event`: `/tmp/wchble_a/ll_connect.dr:3363`.
- `llProcessConnectEvent`: `/tmp/wchble_a/ll_connect.dr:2020`.
- `LL_ConnectFreeTmosPrioritID`: `/tmp/wchble_a/ll_connect.dr:3489`.

Rust design equivalent:

- `connect_event_irq_entry()` should only snapshot and schedule.
- `connect_event_process()` runs the state machine outside the hard IRQ context.
- `connect_event_boundary(time_us_remaining)` fires at the state-machine exit.
- Initial Rust scope can match WCH's single-peripheral/single-connection behavior.

## TI Simple Peripheral Counterpart

### 1. App init and event loop

TI `SimplePeripheral_init()` performs the same high-level work as WCH `Peripheral_Init()`:

- `ICall_registerApp(&selfEntity, &syncEvent)`.
- App message queue construction.
- `GGS_SetParameter(GGS_DEVICE_NAME_ATT, ...)`.
- `GAP_SetParamValue(...)`.
- Bond manager setup.
- `GGS_AddService`, `GATTServApp_AddService`, `DevInfo_AddService`, `SimpleProfile_AddService`.
- `SimpleProfile_RegisterAppCBs`.
- `GAPBondMgr_Register`.
- `GAP_RegisterForMsgs`, `GATT_RegisterForMsgs`.
- HCI data-length defaults.
- `GATT_InitClient`.
- `GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, &pRandomAddress)`.

Source anchor: `simple_peripheral.c:452-590`.

The TI app loop waits on `Event_pend`, fetches BLE service messages with `ICall_fetchServiceMsg`, calls `SimplePeripheral_processStackMsg`, and drains app queue messages. Source anchor: `simple_peripheral.c:599-660`.

This is structurally the same as WCH `TMOS_SystemProcess()` plus task event bits and message queue.

### 2. Advertising setup after device init

TI starts advertising after `GAP_DEVICE_INIT_DONE_EVENT`:

- `GapAdv_create(&SimplePeripheral_advCallback, &advParams1, &advHandleLegacy)`.
- `GapAdv_loadByHandle(... GAP_ADV_DATA_TYPE_ADV ...)`.
- `GapAdv_loadByHandle(... GAP_ADV_DATA_TYPE_SCAN_RSP ...)`.
- `GapAdv_setEventMask(...)`.
- `GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0)`.

Source anchor: `simple_peripheral.c:922-979`.

TI advertising APIs are declared in `gap_advertiser.h`:

- `GapAdv_create`: line 699.
- `GapAdv_enable`: line 759.
- `GapAdv_setEventMask`: line 798.
- `GapAdv_loadByHandle`: around line 968.

Rust design equivalent:

- `AdvSet::create(params, callback_mask)`.
- `AdvSet::load_adv_data`.
- `AdvSet::load_scan_rsp_data`.
- `AdvSet::enable`.

### 3. TI advertising callback and app-context processing

TI `SimplePeripheral_advCallback()` allocates event data and enqueues an app message. `SimplePeripheral_processAdvEvent()` handles the advertising event in app context:

- `GAP_EVT_ADV_START_AFTER_ENABLE`.
- `GAP_EVT_ADV_END_AFTER_DISABLE`.
- `GAP_EVT_ADV_START`.
- `GAP_EVT_ADV_END`.
- `GAP_EVT_ADV_SET_TERMINATED`.
- `GAP_EVT_SCAN_REQ_RECEIVED`.
- `GAP_EVT_INSUFFICIENT_MEMORY`.

Source anchors: `simple_peripheral.c:1286-1359`.

This is the strongest model for Rust callback discipline: radio/LL event callbacks should enqueue or signal; app-level work should run in a task context.

### 4. TI connection events

TI handles link events in `SimplePeripheral_processGapMessage()`:

- `GAP_LINK_ESTABLISHED_EVENT`: add connection, start periodic clock, continue/stop advertising based on connection capacity.
- `GAP_LINK_TERMINATED_EVENT`: remove connection, stop periodic clock when needed, re-enable advertising.

Source anchors: `simple_peripheral.c:999-1060`.

WCH performs the same policy in `peripheralStateNotificationCB()`, `Peripheral_LinkEstablished()`, and `Peripheral_LinkTerminated()`.

## Cross-Vendor Mapping Table

| Concept | TI path | WCH path | Rust LL design target |
| --- | --- | --- | --- |
| App scheduler registration | `ICall_registerApp`, `Event_pend`, `ICall_fetchServiceMsg` | `TMOS_ProcessEventRegister`, `TMOS_SystemProcess`, `tmos_msg_receive` | Internal executor/task queue with app messages. |
| Peripheral role init | `GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, ...)` | `GAPRole_PeripheralInit`, `GAPRole_PeripheralStartDevice` | `PeripheralRole::start()` state machine. |
| Advertising set creation | `GapAdv_create` | `LL_AdvertiseEnalbe`, `llAdvertiseCreateCore`, `llAdvertiseSet` | `AdvSet::create` and state allocation. |
| Adv data load | `GapAdv_loadByHandle(... ADV ...)` | `GAPRole_SetParameter(GAPROLE_ADVERT_DATA)`, `GAP_UpdateAdvertisingData` | Adv data buffer ownership + serialized PDU builder. |
| Scan-rsp data load | `GapAdv_loadByHandle(... SCAN_RSP ...)` | `GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA)`, `ll_advertise_generated_scan_rsp` | Separate scan-rsp buffer and T_IFS-constrained response path. |
| Adv enable | `GapAdv_enable` | `GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED)`, `llAdvertiseStart` | Enable flag + scheduler enqueue. |
| Adv event callback | `SimplePeripheral_advCallback` -> app queue | `LL_AdvertiseEventRegister` -> `ll_advertise_event_closed` | Internal `adv_event_closed(time_us_remaining)`. |
| Scan request notification | `GAP_EVT_SCAN_REQ_RECEIVED` | `GAP_SCAN_REQUEST_EVENT`, `HCI_LE_ScanRequestReceivedEvent`, `ll_advertise_legacy_rx` | `scan_req_received` event emitted from LL RX path. |
| Connection transition | `GAP_LINK_ESTABLISHED_EVENT` | `ll_advertise_to_connection_state`, `GAP_LINK_ESTABLISHED_EVENT` | `adv_to_connection` transition and app event. |
| Connection event boundary | TI connection event reports / callbacks | `ll_set_connect_event`, `llProcessConnectEvent`, `LL_ConnectFreeTmosPrioritID` | IRQ snapshot -> task state machine -> `connect_event_boundary`. |
| Time base | TI Clock/ICall/LL timing | `bleClock_t`, `fnGetClockCBs`, TMOS tick × 625us | BLE controller clock abstraction independent of generic SysTick. |

## Implications for Pure Rust BLE Stack

### 1. Build the Host/Role shape from TI/WCH, then attach WCH hardware later

TI gives enough source to model the upper stack:

- App event loop and message queue.
- GAP/GATT/SM/Bond manager call structure.
- Advertising set lifecycle.
- Advertising event callback model.
- Scan request notification model.
- Connection state policy.

WCH disassembly supplies the CH32V208-specific lower half:

- Real advertising process functions.
- Real event-close boundaries.
- Real TMOS/LL handoff.
- Real hardware register writes in `ll_advertise_tx`, `ll_advertise_generated_scan_rsp`, and connection paths.

### 2. Rust controller needs explicit event-boundary hooks

The Rust LL should define internal hooks:

- `adv_event_closed(time_us_remaining)`.
- `scan_req_received(scanner_addr, channel, rssi?)`.
- `scan_rsp_sent(...)`.
- `connect_ind_received(...)`.
- `connect_event_boundary(time_us_remaining)`.

These are state-machine outputs. They should feed a queue/signal into the app/task layer.

### 3. IRQ work and app work need a hard split

Both TI and WCH route radio events toward a scheduler/app context:

- TI: callback -> app queue -> `SimplePeripheral_processAdvEvent`.
- WCH: LL/TMOS path -> GAP role messages -> `Peripheral_ProcessEvent`.
- WCH connect path: IRQ registration -> `llProcessConnectEvent` -> boundary callback.

Rust should use IRQ handlers for register snapshots, W1C, buffer handoff, and signal posting. PDU parsing, scan response policy, GATT, and app callbacks should run in task context unless the BLE timing requirement forces a bounded LL fast path.

### 4. Advertising should be modeled as an `AdvSet`

The shape should include:

- Legacy/extended mode.
- Own address and AdvA.
- Advertising data.
- Scan response data.
- Interval min/max and randomization policy.
- Channel map 37/38/39.
- Filter policy.
- Event mask.
- Enabled/running state.
- Event boundary and close reasons.

TI's `GapAdv_*` APIs are a good semantic shape. WCH's `GAPRole_*` API is older-role style but maps to the same internal needs.

### 5. Scan response is a first-class timing path

WCH has `ll_advertise_legacy_rx` and `ll_advertise_generated_scan_rsp`; TI has `GAP_EVT_SCAN_REQ_RECEIVED` and explicit scan-rsp data loading. The Rust design should treat scan response as:

1. RX path detects valid SCAN_REQ for our AdvA.
2. Filter policy passes.
3. Controller prepares SCAN_RSP within BLE timing constraints.
4. App-level scan-request notification is emitted separately from the real-time response.

The app notification is post-fact; the radio response path must be controller-owned.

## Focused Follow-Up Documents

This deep dive identifies several narrower reverse-engineering documents:

1. `gaprole-to-ll-callgraph.md`: `GAPRole_PeripheralStartDevice` -> GAP -> HCI -> LL advertise enable path.
2. `ll-advertise-tx-disasm.md`: `ll_advertise_tx`, `ll_advertise_generated_scan_rsp`, and hardware register write sequence. Companion doc currently lives in Lucy's workspace at `/Users/mono/.slock/agents/997bdbdb-791a-40d4-a4f5-25dc90f2ed08/notes/ch32-rs/ti-ble/ll-advertise-tx-disasm.md`.
3. `tmos-osal-equivalence.md`: TI OSAL event/message/timer semantics vs WCH TMOS symbols.
4. `ble-timebase.md`: `bleClock_t`, `fnGetClockCBs`, TMOS tick × 625us, and Rust BLE clock abstraction.
5. `scan-rsp-turnaround.md`: SCAN_REQ parse/filter/response path across TI, WCH disasm, and Rust design.

## Open Questions

- Exact WCH GAPRole -> GAP -> HCI -> LL call chain needs focused disassembly; public names and app behavior are clear, but the internal call graph remains partially inferred.
- `ll_advertise_tx` and `ll_advertise_generated_scan_rsp` hardware writes are covered by Lucy's companion doc; keep this file focused on full-flow architecture and Rust design targets.
- TI controller LL internals are mostly headers/ROM contracts; WCH controller LL has object code we can disassemble. The combined view is enough for shape and many state boundaries, while bit-level CH32 behavior still requires WCH-specific disassembly and board traces.
- The Rust BLE stack should use TI/WCH references as architecture anchors while keeping runtime implementation independent of WCH ROM hooks.
