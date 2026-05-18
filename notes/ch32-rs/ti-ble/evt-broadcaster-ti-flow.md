# EVT Broadcaster + TI BLE Advertising Flow

Status: review-ready, 2026-05-14.

Purpose: compile the WCH EVT Broadcaster and Peripheral examples, reverse the linked ELFs enough to identify the advertising and SCAN_RSP critical paths, and compare them with TI BLE stack flow. This doc is scoped to legacy advertising / SCAN_RSP path decisions for task #79/#80.

## 1. Build Artifact

EVT source:

- EVT root: `/Users/mono/Elec/WCH/CH32V20xEVT-2.31`
- Example: `EXAM/BLE/Broadcaster`
- BLE library: `EXAM/BLE/LIB/libwchble.a`
- Linker script: `EXAM/BLE/HAL/Link.ld`

Rebuild artifact directory:

```text
/tmp/ch32_evt_broadcaster_rebuild_20260514_072434/
/tmp/ch32_evt_peripheral_rebuild_20260514_0733/
```

Key files:

- `Broadcaster.elf`
- `Broadcaster.bin`
- `Broadcaster.hex`
- `Broadcaster.map`
- `Broadcaster.full.dump`
- `Broadcaster.symtab.txt`
- `key-functions.dump`
- `gaprole-key.dump`

Build command was reconstructed manually with Homebrew `riscv64-unknown-elf-gcc`. The successful arch flags were:

```text
-march=rv32imac_zicsr_zifencei -mabi=ilp32
```

Build result:

```text
Broadcaster:
text=134548 data=284 bss=11364 dec=146196
FLASH 134832 B / 448 KB = 29.39%
RAM   64 KB / 64 KB = 100.00%

Peripheral:
text=161888 data=1132 bss=11432 dec=174452
FLASH 163020 B / 448 KB = 35.54%
RAM   64 KB / 64 KB = 100.00%
```

Build caveat: the local GCC warns that `interrupt("WCH-Interrupt-fast")` is ignored. This ELF is valid for static reverse/reference work. Use WCH's intended toolchain for air validation binaries.

## 2. Source-Level WCH Flow

`broadcaster_main.c`:

```text
main
  SystemCoreClockUpdate
  Delay_Init
  USART_Printf_Init
  WCHBLE_Init
  HAL_Init
  GAPRole_BroadcasterInit
  Broadcaster_Init
  Main_Circulation
    while true: TMOS_SystemProcess()
```

Source anchors:

- `Broadcaster/APP/broadcaster_main.c:52-65`
- `Broadcaster/APP/broadcaster_main.c:37-42`

`Broadcaster_Init()` sets the application-side role parameters:

```text
Broadcaster_TaskID = TMOS_ProcessEventRegister(Broadcaster_ProcessEvent)
GAPROLE_ADVERT_ENABLED = TRUE
GAPROLE_ADV_EVENT_TYPE = GAP_ADTYPE_ADV_NONCONN_IND
GAPROLE_SCAN_RSP_DATA = scanRspData
GAPROLE_ADVERT_DATA = advertData
TGAP_DISC_ADV_INT_MIN/MAX = 160  ; 100 ms, 625 us units
tmos_set_event(Broadcaster_TaskID, SBP_START_DEVICE_EVT)
```

Source anchors:

- `Broadcaster/APP/broadcaster.c:130-155`
- `Broadcaster/APP/broadcaster.c:171-190`

Important scope note: EVT Broadcaster sets `GAP_ADTYPE_ADV_NONCONN_IND`. It configures `SCAN_RSP_DATA`, yet the protocol path is non-connectable / non-scannable advertising. Use Broadcaster for ADV TX/init/hot-loop reference. Use EVT Peripheral for SCAN_REQ -> SCAN_RSP.

EVT Peripheral source config:

```text
DEFAULT_ADVERTISING_INTERVAL = 80       ; 50 ms, 625 us units
GAPROLE_ADVERT_ENABLED = TRUE
GAPROLE_SCAN_RSP_DATA = scanRspData
GAPROLE_ADVERT_DATA = advertData
TGAP_DISC_ADV_INT_MIN/MAX = 80
TGAP_ADV_SCAN_REQ_NOTIFY = ENABLE
SBP_START_DEVICE_EVT -> GAPRole_PeripheralStartDevice(...)
```

Source anchors:

- `Peripheral/APP/peripheral.c:45`
- `Peripheral/APP/peripheral.c:218-234`
- `Peripheral/APP/peripheral.c:335-339`
- `Peripheral/APP/peripheral.c:397-406`

## 3. Linked ELF Critical Path

The linked ELF includes the application, HAL, GAP/GATT/SM, and `libwchble.a` LL code. Relevant symbols are present:

```text
WCHBLE_Init                         0x034ea
HAL_Init                            0x03626
GAP_MakeDiscoverable                0x09b72
LL_ProcessEvent                     0x125f4
LL_Init                             0x12718
ll_tx_wait_finish                   0x022d4
ll_advertise_tx                     0x13dd6
llAdvTraverseallChannel             0x14322
llAdvertiseStart                    0x14bcc
LL_AdvertiseEnalbe                  0x14f68
BLE_SetPHYTxMode                    0x173ba
GAPRole_BroadcasterProcessEvent     0x1b75a
GAPRole_BroadcasterStartDevice      0x1baf2
GAPRole_SetParameter                0x1d70e
GAPRole_BroadcasterInit             0x1da28
GAP_DeviceInit                      0x1e91c
BB_IRQHandler                       0x032c8 -> BB_IRQLibHandler
```

### 3.1 Init Path

`WCHBLE_Init` installs the LLE IRQ handler location and calls `BLE_LibInit`. It also enables AHB peripheral clock and touches PFIC-ish registers after BLE init. Anchor: `key-functions.dump:1-105`.

`HAL_Init` registers `HAL_ProcessEvent`, initializes time, and starts a HAL periodic task. Anchor: `key-functions.dump:107-124`.

`GAPRole_BroadcasterInit` is the stack-side role initializer:

```text
GAPRole_BroadcasterInit
  LL_AdvertiseEnalbe
    TMOS_ProcessEventRegister(LL_ProcessEvent)
    LL_Init
    gBleLlPara[0x68] = llAdvertiseCreateCore
    gBleLlPara[0x6c] = llAdvertiseSet
    gBleLlPara[0x70] = llAdvertiseStart
    gBleLlPara[0x74] = llAdvTraverseallChannel
  TMOS_ProcessEventRegister(L2CAP_ProcessEvent); L2CAP_Init
  TMOS_ProcessEventRegister(GAP_ProcessEvent); GAP_Init
  TMOS_ProcessEventRegister(GATT_ProcessEvent); GATT_Init
  TMOS_ProcessEventRegister(SM_ProcessEvent); SM_Init
  TMOS_ProcessEventRegister(GAPBondMgr_ProcessEvent); GAPBondMgr_Init
  TMOS_ProcessEventRegister(GATTServApp_ProcessEvent); GATTServApp_Init
  GAPRole_BroadcasterEnable
```

Anchors:

- `gaprole-key.dump:363-394`
- `key-functions.dump:273-302`

### 3.2 Application Start to GAP Discoverable

Application event path:

```text
Broadcaster_ProcessEvent(SBP_START_DEVICE_EVT)
  GAPRole_BroadcasterStartDevice(&Broadcaster_BroadcasterCBs)
    stores pGapRoles_AppCGs
    GAP_DeviceInit(profileRole = broadcaster)
```

Anchor: `key-functions.dump:125-143`.

GAP role internal event path:

```text
GAPRole_BroadcasterProcessEvent(mask 0x0001)
  if gapRole_AdvEnabled:
    build discoverable parameter block from:
      gapRole_AdvEventType
      gapRole_AdvDirectAddr
      gapRole_AdvDirectType
      gapRole_AdvChanMap
      gapRole_AdvFilterPolicy
    GAP_MakeDiscoverable(gapRole_TaskID, params)
```

Anchor: `Broadcaster.full.dump:1b9ae-1ba1e`.

`GAP_MakeDiscoverable` allocates `pGapAdvertState`, maps GAP event type to internal advertising state, calls `gapSetAdvParams`, then starts the LL task with `tmos_start_task(..., event mask 0x0001, 1600)`. Anchor: `key-functions.dump:144-272`.

`gapSetAdvParams` reads GAP param values and calls `HCI_LE_SetExtendedAdvertisingParametersCmd`, which jumps into `API_LE_SetExtendedAdvertisingParametersV2Cmd`. Anchor: `gaprole-key.dump:13-109`.

Advertising data and scan-response data paths are immediate when the GAP advertising state exists:

```text
GAP_SetAdvertisingAdvData -> HCI_LE_SetExtendedAdvertisingDataCmd
GAP_SetAdvertisingRspData -> HCI_LE_SetExtendedScanResponseDataCmd
```

Anchors:

- `gaprole-key.dump:754-854`
- `gaprole-key.dump:855-946`

### 3.3 LL Event Dispatch

`LL_AdvertiseEnalbe` installs the ADV vtable under `gBleLlPara` and registers `LL_ProcessEvent`. `LL_ProcessEvent` then dispatches TMOS event masks through vtable slots:

```text
mask 0x0001 / bit 0 -> gBleLlPara[0x70] -> llAdvertiseStart
mask 0x0002 / bit 1 -> gBleLlPara[0x74] -> llAdvTraverseallChannel
```

This matches the earlier `ll-process-event-minipass.md` result and the linked Broadcaster ELF.

`llAdvertiseStart`:

- gates on active `adv_state.byte_12 == 1`
- sets `gBleLlPara[0x64] = adv_state`
- sets `gBleLlPara[0x7c] = 1`
- computes channel order in `adv_state[10/25/26]`
- arms the next event timer with `tmos_start_task`
- calls `ll_advertise_tx` for the first channel.

Anchor: `key-functions.dump:402-493` and `key-functions.dump:493-760`.

`llAdvTraverseallChannel`:

- loads `gBleLlPara[0x64]`
- checks active flag
- optionally calls `ll_advertise_event_closed` when `gBleLlPara[0x7c] == 4`
- tail-calls `ll_advertise_tx`.

Anchor: `key-functions.dump:llAdvTraverseallChannel`.

### 3.4 LL TX Hot Path

`ll_advertise_tx` is the canonical hot-loop ADV TX entry. It does all per-channel register writes itself:

```text
ll_advertise_tx
  if gBleLlPara[168] busy: return
  phy_status_clear(2)
  LLE[100] = 160
  BB[44] mode field = 1
  BB[0] direction bits clear/set
  RFEND[8] |= 0x330000
  LLE[80] = 0x5a
  RFEND[44] &= ~2
  channel index -> BB[0]/BB[44]
  assemble PDU in adv_state[76]
  stamp byte_11
  BB[112] = pdu_ptr
  Access Address 0x8e89bed6 -> BB[8]
  CRC init 0x555555 -> BB[4]
```

Then it has two observable cold-TX variants:

1. Fast path in `ll_advertise_tx`:

```text
wait while LLE[100] != 0
BLE_SetPHYTxMode(mode, len)
TMOS_SysRegister(ll_advertise_process)
BB[0] |= 0x800000
BB[44] &= ~3
LLE[0] = 2
```

Anchor: `key-functions.dump:ll_advertise_tx` around `13fca-14012`.

2. Tail-call/helper path in `ll_tx_wait_finish(mode=0)`:

```text
wait while LLE[100] != 0
gBleIPPara[2] = 0
BLE_SetPHYTxMode(mode, len)
BB[0] |= 0x800000
BB[44] &= ~3
gBleIPPara[5] = 0
gBleIPPara[1] = 0
LLE[0] = 2
wait until gBleIPPara[2].b0 || gBleIPPara[3].b0 || LLE[100] == 0
```

Anchor: `key-functions.dump:ll_tx_wait_finish`.

`BLE_SetPHYTxMode` itself is config-only. It sets PHY/RF mode bits, writes `LLE[8] = 0x2000`, writes `gBleIPPara[4] = 0x80`, computes `LLE[100]`, and restores `LLE[12]`. It does not write `LLE[0]=2`. Anchor: `key-functions.dump:BLE_SetPHYTxMode`.

### 3.5 SCAN_REQ -> SCAN_RSP Path from EVT Peripheral

EVT Peripheral linked ELF confirms the SCAN_RSP path:

```text
ll_advertise_legacy_rx
  len check
  pdu_type accept: (type - 3) & 0xfd == 0  ; admits SCAN_REQ=3 and CONNECT_IND=5
  AdvA check
  ll_advertise_filter
  if pdu_type == 5:
    ll_advertise_to_connection_state(...)
  else:
    generated SCAN_RSP
    BLE_SetPHYTxMode(mode, len)
    ll_tx_wait_finish(mode=3, 0, 0)
    ble_ll_hw_api_shut()
```

Peripheral artifact anchors:

- `/tmp/ch32_evt_peripheral_rebuild_20260514_0733/peripheral-rsp-key.dump:166-263`
- `/tmp/ch32_evt_peripheral_rebuild_20260514_0733/peripheral-rsp-key.dump:264-306`
- `/tmp/ch32_evt_peripheral_rebuild_20260514_0733/peripheral-rsp-key.dump:307-403`

`ll_advertise_generated_scan_rsp` only prepares the PDU and stamps `adv_state.byte_11 = 0x93`:

```text
adv_state[17] = scan_rsp_len + 6
pdu[0] = 4                         ; SCAN_RSP PDU type
pdu[1] = adv_state[17]
pdu[2..7] = AdvA
pdu[8..] = scanRspData
adv_state[11] = 0x93
```

`BLE_SetPHYTxMode` stays config-only. The warm turnaround kick belongs to `ll_tx_wait_finish(mode=3)`:

```text
BB[0] |= 0x800000
BB[44] &= ~3
wait until gBleIPPara[2].b0 || gBleIPPara[3].b0 || LLE[100] == 0
```

This is the exact SCAN_RSP implementation target. The Rust broadcaster being air-visible only proves cold ADV TX. It does not prove the warm RX->TX path.

### 3.6 ISR Shape

The linked EVT `BB_IRQHandler` is a direct jump to `BB_IRQLibHandler`:

```text
BB_IRQHandler -> BB_IRQLibHandler
```

Anchor: `key-functions.dump:BB_IRQHandler`.

This matters for the Rust probe path: EVT's BB ISR is the library handler. The current Rust/example path has a partial ISR and partial PATHC/RX-prep model; the linked EVT reference keeps handler semantics inside `libwchble.a`.

## 4. TI BLE Reference Flow

TI sources provide a useful host/GAP/HCI model. They do not expose the hardware-level CC26xx LL in the same way WCH's `libwchble.a` ELF does for CH32V208.

TI legacy HCI APIs map advertising commands to LL functions:

```text
HCI_LE_SetAdvParamCmd      -> MAP_LL_SetAdvParam
HCI_LE_SetAdvDataCmd       -> MAP_LL_SetAdvData
HCI_LE_SetScanRspDataCmd   -> MAP_LL_SetScanRspData
HCI_LE_SetAdvEnableCmd     -> MAP_LL_SetAdvControl
```

Source anchors:

- `source/ti/ble5stack/hci/cc26xx/hci.c:1935-1960`
- `source/ti/ble5stack/hci/cc26xx/hci.c:1972-1984`
- `source/ti/ble5stack/hci/cc26xx/hci.c:2043-2058`
- `source/ti/ble5stack/hci/cc26xx/hci.c:2070-2083`

TI's newer ICall shim uses `GapAdv_enable(...)` for the legacy advertising handle and explicitly notes that `HCI_EXT_AdvEventNoticeCmd(...)` should be called before each `GAP_MakeDiscoverable` when application ADV-end events are desired.

Source anchors:

- `source/ti/ble5stack/icall/app/icall_hci_tl.c:6960-6979`
- `source/ti/ble5stack/inc/hci.h:3860-3895`

Useful TI/WCH alignment:

| Layer | WCH EVT Broadcaster | TI BLE reference |
| --- | --- | --- |
| App profile | `Broadcaster_Init`, `GAPRole_SetParameter`, `GAPRole_BroadcasterStartDevice` | App/role config around GAP params |
| GAP discoverable | `GAP_MakeDiscoverable` | Legacy `GAP_MakeDiscoverable` / modern `GapAdv_enable` |
| HCI command surface | `HCI_LE_SetExtendedAdvertising*` in linked WCH lib | `HCI_LE_SetAdv*`, `HCI_LE_SetScanRspDataCmd`, `HCI_LE_SetAdvEnableCmd` |
| LL boundary | `LL_SetAdvControl`, `llAdvertiseStart`, `ll_advertise_tx` | `MAP_LL_SetAdv*`, `MAP_LL_SetAdvControl` |
| Event callback | `LL_AdvertiseEventRegister` / `ll_advertise_event_closed(timeUs)` in WCH RE docs | `HCI_EXT_AdvEventNoticeCmd(taskID, event)` |
| Hardware landing | WCH linked ELF / `libwchble.a` disassembly | controller internals hidden |

## 5. Implications for Path A

The linked EVT Broadcaster supports Path A for cold ADV TX; the linked EVT Peripheral is the direct reference for SCAN_RSP:

- The application hot loop is `TMOS_SystemProcess()`.
- The GAP/LL path uses TMOS event masks and vtable dispatch.
- `LL_AdvertiseEnalbe` installs ADV vtable entries once.
- `ll_advertise_tx` is the per-channel hot path and performs the repeated full TX register sequence.
- The hot TX path uses `BLE_SetPHYTxMode` + TX kick/wait; it does not run PATHC W1C / IRQ re-enable as a per-iteration block.
- `BB_IRQHandler` remains the library ISR, so the EVT reference includes hidden handler state transitions that the Rust example currently approximates.
- SCAN_RSP is a warm RX->TX path: `ll_advertise_legacy_rx -> ll_advertise_generated_scan_rsp -> BLE_SetPHYTxMode -> ll_tx_wait_finish(mode=3)`.
- The next Rust implementation should target the warm path directly after preserving the working cold ADV broadcaster baseline.

Practical decision for task #79/#80:

1. Treat EVT Broadcaster as the primary runnable reference for legacy ADV flow.
2. Treat TI BLE as a semantic reference for GAP/HCI/event-notice shape.
3. Use WCH linked ELF / `libwchble.a` disassembly as the source of truth for CH32V208 BB/LLE/RFEND register order.
4. Move PATHC/IRQ bring-up into a once-at-init path and keep cold ADV TX close to `ll_advertise_tx` / `ll_tx_wait_finish(mode=0)`: wait `LLE[100]`, configure PHY, kick TX, then use the lib-style completion sources.
5. Implement SCAN_RSP from the EVT Peripheral warm path: parse SCAN_REQ, generate SCAN_RSP PDU, run `BLE_SetPHYTxMode`, and fire `ll_tx_wait_finish(mode=3)` warm kick.

## 6. Follow-Up Checks

Before coding Path A, confirm two details in the Rust example:

- Which existing block corresponds to `BLE_SetPHYTxMode(mode, len)` and whether it already writes `LLE[8]=0x2000`, `gBleIPPara[4]=0x80`, and `LLE[100]`.
- Whether the Rust BB ISR path can be kept out of the first Path A smoke test by preserving EVT-like library ISR behavior assumptions, or whether a minimal ISR mirror is required for completion flags.

The next code experiment should be a single architecture variant recorded in `notes/ch32-rs/ti-ble/task79-attempt-log.md`: PATHC once-at-init, hot loop per-channel TX only.
