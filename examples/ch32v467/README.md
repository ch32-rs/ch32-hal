# CH32V467 first bring-up

The original examples are **PAC-only**. `hal_init` additionally exercises the
initial HAL support: HSI clocks, bus dividers, GPIO clock/reset setup and optional
Embassy driver initialization. Peripheral drivers are still being migrated.
The HAL and examples use the same pinned Git PAC revision, `e150d53`, generated
from ch32-data `844640b` (#78). Firmware builds do not require sibling PAC or
qingke checkouts; qingke/qingke-rt are resolved from crates.io.

## Build

Run from this directory with a recent Rust toolchain:

```sh
rustup target add riscv32imc-unknown-none-elf
cargo build --release --bins
```

The default part is **CH32V467WEU6**, with the board LED on **PE8/PE9**.
Select exactly one part for another package:

```sh
cargo build --release --bins --no-default-features --features ch32v467ret6
cargo build --release --bins --no-default-features --features ch32v467vet6
```

### HAL stages 0–1

The migration plan and validation record are in
[`../../local-docs/ch32v4-migration.md`](../../local-docs/ch32v4-migration.md).
The Git PAC revision is fixed in both Cargo manifests until the corrected
V4x7 metadata is released. HAL dependencies require qingke/qingke-rt 0.8.3
or newer for V3V. The host models under `tools/` still require the generated
sibling PAC as described in their READMEs.

```sh
cargo run --release --bin hal_init --features hal
cargo run --release --bin hal_init --features hal,clock-dividers
cargo run --release --bin hal_init --features embassy
```

`hal_init` calls `ch32_hal::init()` once, checks the RCC clock cache, HSI status,
GPIO/AFIO clock enables and `rcc::refresh(None)`, then reports
`HAL_INIT_RESULT=PASS` over RTT and repeats a heartbeat. With `clock-dividers`,
SYSCLK/HCLK/PCLK1/PCLK2 are 20/10/2.5/5 MHz; otherwise all four are 20 MHz.
It has no GPIO outputs. `HAL_INIT_RESULT` is 0 before entry, 1 while checking,
2 on success, or 3 on a failed check.

The blocking variant uses the HAL SysTick delay. With `embassy`, it instead
checks SysTick configuration and uses a CPU spin loop for the heartbeat;
executor, Timer accuracy, rollover and sleep/wake validation belong to stage 3.
Do not use the blocking `Delay` while Embassy owns SysTick.

HAL defaults are disabled here; opt into GPIO EXTI with `exti`. TIM-based time
drivers still reject V4x7 until ported. DMA and other unported drivers are not exposed. HSE/PLL
configuration is not available, and `refresh` rejects those clock sources.

### HAL stage 2: GPIO and EXTI

Build locally without programming the board:

```sh
cargo build --release --bin hal_blinky --features hal
cargo build --release --bin hal_exti --features exti
```

`hal_blinky` alternates PE8 and PE9 through HAL GPIO outputs, using a nominal
100 ms blocking delay. It checks both output latch and pad readback after each
transition and reports `GPIO_OUT_RESULT=PASS` after 20 transitions. The debugger
symbol is 0 before entry, 1 while running, 2 on success, and 3 on failure. With
`embassy`, it uses a CPU spin loop; executor-based blinking uses `hal_embassy`.
Output readback passed on WEU6; LED appearance and waveform accuracy require
visual or instrument verification.

For `hal_exti`, connect **PA0 to PB1, PB8 and PC13** as one signal net (three
jumper branches). Check that these pins are exposed and free of external drivers
on your board before running. The program initially floats PA0 and tests each
input's pull-up/down individually, restoring floating mode between tests. It
then drives PA0 and waits on all three inputs concurrently, covering EXTI1,
EXTI9_5 and EXTI15_10 and GPIO B/C routing. Eight rounds check rising/falling/any
edges, actual IRQ wake callbacks, future cancellation, stale pending clearance
and rearming. Each edge wait has a bounded polling timeout.

This example polls futures directly and does not use an executor or Timer. It
reports `GPIO_EXTI_RESULT=PASS` over RTT; the debugger symbol is 0 before entry,
1 while running, 2 on success, and 3 on a failed check. Failures halt. A missing
jumper can also fail the earlier pull/routing checks. Hardware acceptance is
pending; building does not verify electrical behavior or interrupt delivery.

After preparing the wiring, these commands program and run the examples:

```sh
cargo run --release --bin hal_blinky --features hal
cargo run --release --bin hal_exti --features exti
```

USART1's split remap fields (PCFR1 bit 2 and PCFR2 bit 26) are implemented and
covered by local register-model checks. USART transfers remain a later stage.

### HAL stage 3: Embassy

```sh
cargo build --release --bin hal_embassy --features embassy
cargo run --release --bin hal_embassy --features embassy
cargo run --release --bin hal_embassy --features embassy,clock-dividers
```

No input wiring is needed. The example owns SysTick, places its 32-bit counter
near rollover before application timers exist, checks monotonic time, then runs
three tasks with 100/150/230 ms timers for 16 iterations each. Two tasks drive
PE8/PE9. It checks alarms across rollover, cancelled and past deadlines, short
deadlines, and eight WFI waits with interrupts masked. RTT reports
`EMBASSY_RESULT=PASS`, followed by a continuous timer-based heartbeat. The result
symbol uses 0/1/2/3 for not entered/running/pass/fail. The rollover setup is
test-only; applications must not reset or rewrite the running time counter.

V4x7 has a **32-bit CNT and 32-bit CMP**, with reserved words at 0x0c and 0x14.
The pinned PAC no longer exposes invented high-word or 64-bit views. A dedicated
driver extends the counter in software and keeps maintenance interrupts active
even without user timers. Interrupts must not remain masked for half a wrap
(about 859 seconds at the default clock). Time conversion supports the
noninteger counter/tick ratios at HCLK=20 MHz and 10 MHz.

Both clock configurations passed on WEU6 on 2026-11-01, including IRQ wakeup and
WFI. Host-observed RTT heartbeat intervals were approximately one second;
calibrated oscillator accuracy and sleep current were not measured. GPIO input
and external EXTI validation remain deferred until wiring is available.

The stock RV32IMC target is extended in `.cargo/config.toml` with
`-C target-feature=+xwchc,+zve64x,+zvl64b,+zvbb`. It retains the soft-float
ILP32 ABI and does not enable native CAS. These vector features match WCH's
`EVT/EXAM/CPU/AccuracyDelayInTick/AccuracyDelayInTick.wvproj` in the
[CH32V407/CH32V467 SDK](https://github.com/openwch/ch32v407_ch32v467).
Use the embedded integer vector subset rather than `+v`, which also implies
floating-point vector capabilities and a minimum VLEN of 128 bits. This board
reports VLEN = 64 bits and no scalar F extension.

The current rustc nightly (1.101.0-nightly, 2026-10-17, LLVM 23.1.4) warns that
`xwchc` is an unknown/unstable Rust target feature, but passes it to LLVM.
The ELF architecture attribute includes `xwchc2p2`, and disassembly and the
board test below confirm that it is effective. A future rustc may reject this
flag; keep this toolchain available when reproducing the test. `xwchc` is not
exposed through Rust's `cfg(target_feature)`, so the ISA example checks vector
features at compile time and tests WCH compressed instructions at runtime.

qingke-rt 0.8.3 from crates.io provides V3V core startup via `qingke-rt/v3v`,
and the PAC provides the device vectors and memory layout.
No custom linker memory map is maintained.
The PAC's default `c512_r200` split is 512 KiB primary flash + 200 KiB SRAM;
another 480 KiB flash follows it. The generated linker script merges the flash
regions into a 992 KiB boot alias at `0x00000000`; the stack top is `0x20032000`.
The chip option-byte split must match. Do not change option bytes blindly.

## Examples and observation

- `blinky`: the board LEDs on PE8/PE9 alternate every nominal 100 ms (5 Hz
  per LED). Logs clock setup and each heartbeat over **defmt RTT**, not SDI.
  PE8/PE9 are 2 MHz push-pull outputs driven at opposite levels.
- `rtt`: no GPIO outputs; logs before clock init and repeats clock/heartbeat
  information every nominal second. Replaces the former `sdi_print` example.
- `cpuid`: no GPIO outputs; prints a one-shot CPU identity report over defmt
  RTT after a 500 ms attachment window, following the CH585 example's output
  style. Includes `mvendorid`, `marchid`, `mimpid`, `mhartid`, `misa`, raw
  hexadecimal values, CSR presence, WCH-style identity fields, XLEN and ISA
  extension bits (including F/V). Illegal CSR reads during a probe report
  `present=false`; unrelated exceptions halt. Zero is not treated as absent.
  Decoded fields do not identify the package, and advertised ISA bits do not
  prove instruction behavior or safe atomics. The example only reads CSRs;
  `qingke-rt/v3v` enables vector state (mstatus.VS = Dirty) during startup.
  The report is not repeated; re-run/reset for another report.
- `isa`: tests WCH compressed byte/halfword loads and stores, plus separate
  Rust volatile-copy functions for disassembly. Tests `vsetvli`, vector loads,
  `vadd.vv`, stores and Zvbb `vror.vi` with 8/16/32/64-bit elements. Each case
  processes 19 elements in variable-length chunks and compares every result
  against scalar wrapping addition and rotation, including overflow and tails.
  Logs failures and synchronous traps over RTT. `ISA_RESULT` is 0 before the
  test, 1 while running, 2 for PASS, or 3 for failure/trap. No GPIO outputs.
- All expose ELF symbols `BOOT_STAGE` (1 = entered init, 2 = clock/SysTick
  configured) and `HEARTBEAT` for debugger inspection without a console.

`defmt.x` retains the log metadata, `defmt-rtt` provides `_SEGGER_RTT` in RAM,
and `qingke/critical-section-impl` serializes the single-core logger. RTT does
not depend on the SDI DATA0/DATA1 addresses. `disable-blocking-mode` keeps
execution moving if the host disconnects; a full buffer can lose log data.
The panic handler is still `panic-halt`: Rust panics halt without panic text.

### probe-rs runner and local V4x7 support

The runner is `probe-rs run --chip CH32V467WEU6`, with `DEFMT_LOG=info` by
default. No flashing is done by `cargo build`.

The local `../../../probe-rs` checkout now recognizes WCH-Link family `0xA6`
and includes `CH32V4X7_Series.yaml`: six V407/V467 packages, each with 200 KiB
or 136 KiB RAM, plus generated V4x7 Flash algorithms. The local build was
installed with `cargo install --path probe-rs-tools --bin probe-rs --force
--locked`; the installed `probe-rs` in `~/.cargo/bin` is what the default
Cargo runner resolves from PATH.
To explicitly select the checkout build instead, use this runner override:

```sh
export CARGO_TARGET_RISCV32IMC_UNKNOWN_NONE_ELF_RUNNER="$(cd ../../../probe-rs && pwd)/target/release/probe-rs run --chip CH32V467WEU6"
# Flashes, starts the firmware and decodes defmt RTT (not yet hardware-validated).
cargo run --release --bin blinky
# Logging-only test; also flashes.
cargo run --release --bin rtt
# CPU identity report; also flashes. No GPIO outputs.
cargo run --release --bin cpuid
# Instruction behavior checks; also flashes. No GPIO outputs.
cargo run --release --bin isa
# For an already programmed matching ELF; no flashing/reset.
../../../probe-rs/target/release/probe-rs attach --chip CH32V467WEU6 target/riscv32imc-unknown-none-elf/release/rtt
```

Read-only hardware checks on 2026-10-28 passed with the local release binary:
`info --verbose --protocol jtag` reports family `0xA6`, Chip ID `0x46740011`;
explicit-chip reads confirm that ID and USER at `0x1FFFF802` = `0xBF` (200 KiB
RAM, matching the default linker layout). Omitting `--chip` identifies WEU6
but the subsequent connection times out over USB; keep the explicit chip.
Flash programming and live RTT passed on 2026-10-31; PE8/PE9 behavior remains
unverified. V4x7 is not
opted into WCH-Link's automatic unprotect-on-attach path.

## Board status after recovery (2026-11-03)

The earlier low-HCLK firmware left the probe unable to attach (`81 55 01 01`).
`wlink erase --method power-off --chip CH32V4X7` recovered the board without
touching option bytes, and the supported examples were re-run (USBHS, EXTI and
the natural-wrap variant were not). Timing-sensitive results were collected by
flashing (`probe-rs download`), resetting and reading RAM markers afterwards,
because a live RTT/debug attach halts the core.

- PAC-only `isa`, `cpuid`, `rtt`, and HAL `hal_init`, `hal_blinky`, `hal_rcc`
  (9 HSI + 14 HSE configurations), `hal_embassy`, `hal_time_scale` (default,
  PLL, AHB/512) and `hal_systick_stress` (512 rollovers, default and divided;
  the negative control reports the expected `FAIL code=13`) passed.
- `hal_dma` (24 channels, 216 copies), `hal_usart` (10/10 TX), `hal_timer`
  (24 cases), `hal_i2c` (configuration/cleanup), `hal_spi` (24 cases),
  `hal_dac`, `hal_rng`, `hal_adc`, `hal_can` (72-frame loopback), `hal_i3c` and
  `hal_flash` (read-only plus `flash-write-test`) passed.
- `hal_eth` initialized (PHY ID `0x73719011`) but had no cable; `hal_sdio`
  correctly reported no card response.
- Two driver bugs were found and fixed: `SPI::blocking_flush` polled `TXE`
  while `SPE=0` (first `hal_spi` write deadlocked), and fast page programming
  waited for `BSY|WRBSY` instead of `WRBSY` (`flash-write-test` timed out).
  Host models in `tools/spi-host-tests` and `tools/flash-host-tests` now cover
  both boundaries.

Still requiring hardware or equipment: PE8/PE9 LED observation, the natural
32-bit SysTick wrap, GPIO input/EXTI wiring, USART/SPI loopback wiring, USBHS
enumeration/echo, an external I2C EEPROM, external I3C/CAN/SD/Ethernet peers,
option-byte RAM split and measured HSI frequency, and analog measurements.

SDI did not work on the user's board; an address/handshake mismatch is a
hypothesis, not a diagnosed cause. These examples no longer touch SDI registers.

The PAC-only examples explicitly select **HSI 20 MHz, HCLK /1** and use a polled SysTick.
They do not need an external crystal, configure PLL/PSRAM, or enable peripheral
interrupts. Timing is nominal until measured on hardware, including log overhead.

## USBHS device / CDC echo

```sh
cargo run --release --bin hal_usbhs --features usbhs
cargo run --release --bin hal_usbhs --features usbhs,clock-pll
cargo run --release --bin hal_usbhs --features usbhs,clock-hse
cargo run --release --bin hal_usbhs --features usbhs2
python3 usbhs_echo_test.py /dev/ttyACM0
```

The default uses USBHS1: PA12 = D+, PA11 = D−. `usbhs2` selects USBHS2:
PB7 = D+, PB6 = D−, available on VET6/WEU6, not RET6. Connect the matching
USB data connector and a common ground. The WCH-Link debug connection does not
provide a USB host connection to these pins. Do not use both instances on the
same pair of wires. USBHS2 uses its own interrupt, bus gate and UTMI gate.

The example uses **full-speed USB** with 64-byte CDC bulk packets. HSI feeds the
shared 480 MHz USBHS PLL; `clock-pll` additionally runs HCLK at 200 MHz.
`clock-hse` requires a 25 MHz crystal and instead supplies the USBHS PLL from HSE.
HSI operation still needs physical enumeration and stability testing; use the
actual board crystal configuration when assessing USB frequency accuracy.
The demonstration VID/PID is `c0de:cafe`, matching the existing USB examples;
it is not a production allocation.

Open the enumerated CDC port with DTR asserted. The Linux test script has no
third-party dependencies and checks binary payloads spanning 1–4096 bytes,
including 63/64/65-byte boundaries and all byte values. It transfers both ways
concurrently and fails on mismatches, disconnects or timeout. Expect
`PASS: 120 exchanges, 62160 bytes matched` with the default ten rounds.
The firmware adds an IN ZLP after a full 64-byte packet so exact packet-sized
echoes are delivered without waiting for another packet.

Debugger symbols: `USBHS_STAGE` = 1 initializing, 2 waiting for CDC connection,
3 connected, 4 echoed data. `USBHS_ECHO_PACKETS`, `USBHS_ECHO_BYTES` and
`USBHS_ERRORS` record successful packets/bytes and buffer overflow errors.
Stage 4 alone is not a complete pass: compare host data, exercise bus reset,
close/reopen and reconnect, and check that the error counter remains zero.

The driver borrows aligned internal SRAM for the bus/endpoint lifetimes. EP0
reserves 64 bytes; each non-control direction allocates its own packet buffer.
Physical endpoints 1–7 are exposed; paired IN/OUT directions must have equal
maximum packet sizes because hardware shares the length register. Canceling
an IN write may leave its owned packet queued; the next write waits for that
packet before copying new data. Reset, disable and new control SETUP invalidate
pending transfers. The driver does not detect VBUS removal; physical cable
removal cannot currently produce a `PowerRemoved` event.

High-speed packet allocation and register operation are covered by the host
model, but this CDC example intentionally uses full speed. High-speed PHY,
isochronous timing, remote wakeup and full-speed fallback need hardware tests.
Embassy USB 0.6 does not supply an other-speed configuration descriptor, so
switching only the driver's speed is insufficient for a complete dual-speed
device. Endpoint aliases 9–15, burst mode, LPM and USB host mode remain pending.

The reproducible USBHS model lives in `../../tools/usbhs-host-tests`; its README
lists the host test command and model boundaries. All 27 USBHS checks passed,
alongside the 16-case library matrix and six CDC example build configurations.
The library and example builds and host model tests passed; **enumeration and
echo have not yet run on the board**. The probe connection was recovered on
2026-11-03, but no USB host has been attached to the CDC port.

## SysTick rollover stress test (2026-11-01)

```sh
cargo run --release --bin hal_systick_stress --features embassy
cargo run --release --bin hal_systick_stress --features embassy,clock-dividers
cargo run --release --bin hal_systick_stress --features embassy,clock-pll,stress-natural-wrap
```

This test uses TIM2 as a reference counter and watchdog; it requires no input
wiring. Each run exercises 512 accelerated SysTick rollovers, alternating normal
interrupt handling with 50/500/2000 us of masked interrupts across the boundary.
It checks that the ISR rearms CMP before allowing the foreground to read Embassy
time again, so foreground reads cannot hide a missed rollover. It also checks the
software epoch, deadlines and a two-second relative time measurement.

Three standalone runs per clock configuration passed all 3072 rollovers. Measured
3 ms waits were 3036–3098 us at HCLK 20 MHz and 3076–3152 us at HCLK 10 MHz.
These include ISR/executor latency and are observed samples, not worst-case bounds.
TIM2 and SysTick share HSI, so this does not calibrate oscillator accuracy. A
natural 32-bit wrap and very long interrupt masking remain separate tests.
`clock-pll,stress-natural-wrap` adds a complete natural counter period at
HCLK = 200 MHz (about 172 seconds), after the accelerated tests. During that
period, foreground code uses TIM2 and waits for the SysTick ISR to rearm CMP
before polling the Embassy timer or reading Embassy time again. This additional
test has been built but has not yet passed on hardware.

For a negative control, add `stress-disable-systick-irq`: the watchdog must report
`FAIL code=13`. Reflash a normal configuration afterward. With no continuous RTT
connection, inspect `SYSTICK_STRESS_RESULT` (1 running, 2 passed, 10–18 failed) and
`SYSTICK_STRESS_STATS` (rounds, min/max wait, TIM2 elapsed, Embassy elapsed; us).
Allow five seconds after reset before attaching to read the result. Continuous
debugger access can introduce much larger latency or lose short TIM2 reference
periods; the reference uses the full 65.536 ms counter period to reduce this risk.

`hal_time_scale --features embassy` checks the conversion of stopped hardware
counter values against a wide integer reference. Add `clock-pll` for 200 MHz
HCLK or `clock-slow` for AHB /512. The latter has a fractional SysTick frequency;
the driver preserves the source/divider ratio instead of truncating it to Hz.
The example restores HSI /1 after checking, allowing the debugger to reconnect.
After the 2026-11-03 recovery, the default, PLL and AHB/512 configurations
passed on hardware, as did `hal_systick_stress` with the default and divided
clocks (512 rollovers each) and the `stress-disable-systick-irq` negative
control (`FAIL code=13`). The `stress-natural-wrap` variant (about 172 seconds)
has not been run yet.

During the earlier slow test, HCLK stayed at about 39 kHz and the probe could
not attach. The version that restores HSI /1 was flashed after a power-off
erase, and the board has been recovered as described above.

## DMA migration check (2026-11-01)

```sh
cargo build --release --bin hal_dma --features embassy
cargo run --release --bin hal_dma --features embassy
```

The example tests all 13 DMA1 and 11 DMA2 channels, including both register
windows. It performs 216 copies across byte/halfword/word widths, checks byte
lengths 1/2/3/31/256/65535 and buffer guards, then cancels a transfer without a
peripheral request and reuses that channel. Each copy has a two-second timeout.
No signal wiring is needed. The large buffers use about 133 kB of RAM; retain
the default 200 KiB SRAM layout for this example.

Inspect `DMA_RESULT` (1 running, 2 passed, 10 timeout, 11 content mismatch,
12 guard corruption, 13 unexpected peripheral request), `DMA_CASES`,
`DMA_CHANNELS_PASSED` and `DMA_LAST` (channel index, word count, word bytes).
The hardware transfer-error handler stops the channel and panics; it does not
return a recoverable error from `Transfer`.

Release linking and 13 host tests passed, covering register windows and the
transfer state machine with RAM-backed PAC registers. `hal_dma` then passed on
hardware on 2026-11-03 (24 channels, 216 copies). Interrupt-timing margins were
not separately measured.

## USART migration check (2026-11-01)

```sh
cargo build --release --bin hal_usart --features embassy
cargo build --release --bin hal_usart --features embassy,clock-dividers
cargo build --release --bin hal_usart --features embassy,clock-pll
```

The example drives the default TX pins of all ten instances. No input wiring is
required for this mode. It checks blocking transmit pacing and BRR, asynchronous
DMA writes, TC interrupts, write/flush cancellation and subsequent reuse. Baud
rates are 9600, 115200 and the lesser of 1 Mbps or PCLK/16. USART1 uses PCLK2;
the other instances use PCLK1.

| USART | TX / RX pins | TX / RX DMA channels |
| --- | --- | --- |
| 1 | PA9 / PA10 | DMA1_CH4 / DMA1_CH5 |
| 2 | PA2 / PA3 | DMA1_CH7 / DMA1_CH6 |
| 3 | PB10 / PB11 | DMA1_CH2 / DMA1_CH3 |
| 4 | PC10 / PC11 | DMA2_CH5 / DMA2_CH3 |
| 5 | PC12 / PD2 | DMA2_CH4 / DMA2_CH2 |
| 6 | PC0 / PC1 | DMA2_CH6 / DMA2_CH7 |
| 7 | PC2 / PC3 | DMA2_CH8 / DMA2_CH9 |
| 8 | PC4 / PC5 | DMA2_CH10 / DMA2_CH11 |
| 9 | PB1 / PB2 | DMA1_CH8 / DMA1_CH9 |
| 10 | PE0 / PE1 | DMA1_CH10 / DMA1_CH11 |

For V4x7, async TX-only constructors take an explicit USART interrupt binding
before the DMA channel, as shown in `hal_usart`. This ensures `flush().await`
can be woken by TC. Empty writes succeed; buffers longer than 65535 bytes return
`Error::BufferTooLong`. DMA transfer faults retain the DMA driver's panic policy.

After input wiring becomes available, connect PA9 to PA10 and build with
`embassy,usart-loopback` to add USART1 RX tests. The optional test covers repeated
full transfers, IDLE lengths, cancelling/restarting reception and keeping TX
active after dropping the split RX half. The unused `check_rx_mappings` function
only type-checks all ten RX pin/DMA combinations; it performs no board testing.

For timing checks, flash with explicit `--chip CH32V467WEU6`, reset and allow at
least ten seconds of autonomous execution before attaching to read RAM. Continuous
RTT/debug access can distort the short cancellation windows. Inspect
`USART_RESULT` (1 running, 2 passed, 10–22 TX failure, 30–37 loopback failure),
`USART_CHANNELS_PASSED` and `USART_SNAPSHOTS` (PCLK Hz, BRR, CTLR1, CTLR3).

All four build modes, including PLL+APB dividers+loopback, link successfully.
Eighteen host tests cover UART configuration and asynchronous state with the
existing DMA tests. These results do not establish physical TX waveforms or RX
correctness. The default TX test passed on hardware on 2026-11-03 (10/10
channels); it does not verify reception, and its timer shares the system clock.
RX and loopback still require PA9–PA10 wiring.

## Next steps

1. Expand hardware coverage of the generated V4x7 Flash algorithms (endurance,
   worst-case erase timing); investigate the auto-detection reconnect timeout
   (use explicit `--chip` meanwhile).
2. Verify PE8/PE9 LED, option-byte RAM split and measured HSI frequency.
3. Run the remaining board tests that need wiring or external peers: USART
   loopback, SPI loopback, I2C EEPROM, external I3C target/CAN/SD/Ethernet, and
   the natural SysTick wrap. The time-driver, DMA and default USART regressions
   passed on 2026-11-03.
   HSE/PLL RCC passed 9 HSI-based and 14 HSE-enabled clock configurations.
   GPIO input and external
   EXTI tests remain pending until wiring is available; HAL output and Embassy
   timers/rollover/WFI have passed the stage 2–3 tests above.

Building these examples only validates compilation and linking, not hardware
startup or full peripheral support.

## Hardware validation (2026-10-31)

On the attached CH32V467WEU6, `cargo run --release --bin isa` flashed and ran
successfully with the configured target features. RTT reported:

```text
mstatus=0x80001e88, VS=3, VLEN=64 bits
Xwchc byte/halfword load/store + Rust copies: pass=true
Zve64x vadd + Zvbb vror e8: PASS (19 elements, 3 chunks)
Zve64x vadd + Zvbb vror e16: PASS (19 elements, 5 chunks)
Zve64x vadd + Zvbb vror e32: PASS (19 elements, 10 chunks)
Zve64x vadd + Zvbb vror e64: PASS (19 elements, 19 chunks)
ISA RESULT: PASS
```

`llvm-readelf-21 -A target/riscv32imc-unknown-none-elf/release/isa` confirms
`zve64x1p0_zvkb1p0_zvl32b1p0_zvl64b1p0_xwchc2p2` (alongside other inherited
features). Inspect code generation with:

```sh
llvm-objdump-21 -d --mattr=+xwchc,+zve64x,+zvl64b,+zvbb \
  target/riscv32imc-unknown-none-elf/release/isa
```

LLVM 21 prints compressed WCH instructions using ordinary `lbu`/`sb`/`sh`
mnemonics; their two-byte encodings distinguish them. `isa_copy_byte` contains
`218c` (load) and `a10c` (store); `isa_copy_halfword` contains `a10e` (store).
The explicit assembly also executes compressed `lhu`. These checks validate
the exercised instructions, not every instruction in each enabled extension,
automatic vectorization, vector context switching, or atomic correctness.

## HAL I2C validation

Reproduce the host-side I2C/DMA regression from the repository root:

```sh
CARGO_TARGET_DIR=/tmp/ch32v4-i2c-host-target cargo test \
  --manifest-path tools/i2c-host-tests/Cargo.toml \
  --target x86_64-unknown-linux-gnu -- --test-threads=1
```

These tests compile the current production driver and generated local PAC. They
model bus events and DMA completion and do not replace external-device testing.
`--target x86_64-unknown-linux-gnu` is the Linux target of the reference
command (no CI job currently runs these harnesses); the harness also runs
natively on macOS without it. SPI and Flash have equivalent host models in
`tools/spi-host-tests` and `tools/flash-host-tests`, covering the two boundary
bugs fixed on 2026-11-03.

`cargo run --release --bin hal_i2c --features embassy` checks the default PB6/PB7
mapping and PB8/PB9 remap at 100kHz and both 400kHz duty ratios. Add
`clock-pll,clock-dividers` for the high clock/APB prescaler case.

The default run performs address-only probes at 0x55 and exercises cancellation
cleanup. `I2C_PROBES` records 0 for ACK, 1 for NACK and 2 for timeout. A timeout on
unwired pins does not validate communication. `I2C_RESULT=2` establishes only
configuration and cleanup checks; values 10–30 identify failures. Timing register
snapshots are in `I2C_SNAPSHOTS`.

For external-device validation, connect a 3.3V 24C02 EEPROM at address 0x50 with
SCL=PB6, SDA=PB7, common ground and suitable external pull-ups, then add
`i2c-eeprom`. This reads a baseline and compares blocking and async DMA reads,
1/2/3-byte tails, adjacent read buffers, repeated START and cancellation/reuse.
It writes the EEPROM address pointer only; it does not write nonvolatile data.
`I2C_DEVICE_RESULT=2` marks completion of those checks. Measure SCL/SDA with a
scope or logic analyzer for electrical timing and ACK verification.

Both modes have built and linked locally. The default configuration passed its
address-probe and cleanup checks on hardware on 2026-11-03; external-device
(`i2c-eeprom`) testing remains pending.

## HAL timer/PWM validation

`cargo run --release --bin hal_timer --features embassy` exercises all eight
timers at 50, 1000 and 10000Hz using update interrupts and an Embassy time window.
Add `clock-pll,clock-dividers` to exercise both APB timer clock domains after
prescaling. The test also configures all six PWM timers in up, down and center
aligned modes and checks 0%, 50% and 100% duty register values.

Afterward TIM1 drives PE9/PE8 as CH1/CH1N (full remap), 1kHz, 50% duty, with a
requested dead time of 20 timer input clock ticks. Inspect both outputs with a
scope to verify frequency, polarity and dead time. Register readback alone does
not establish electrical behavior.

RAM markers: `TIMER_RESULT` is 1 while running, 2 after passing, or 10–16 on
failure; `TIMER_CASES` should reach 24; `TIMER_SNAPSHOTS` contains frequency,
PSC, ARR and the last interrupt count for each timer. Allow at least 4 seconds
without continuous debugger polling before inspecting results.

The example has built and linked locally. The 24 timer/PWM register cases passed
on hardware on 2026-11-03; PE9/PE8 waveform and dead-time verification still
needs a scope.

## HAL SPI validation

From this example directory:

```sh
cargo run --release --bin hal_spi --features embassy
cargo run --release --bin hal_spi --features embassy,clock-pll,clock-dividers,spi-loopback
```

The default requires no input wiring. It transmits through SPI1 PA5/PA7, SPI2
PB13/PB15, and SPI3 PB3/PB5. Each instance exercises all four modes, both bit
orders, 8/16-bit blocking and DMA writes, frequency changes, and cancellation
followed by reuse. Check `SPI_RESULT` (1 running, 2 passed, 10–30 failed),
`SPI_CASES` (24 on completion), and `SPI_SNAPSHOTS` (control, status, actual Hz).
This checks transmitter operation and peripheral state; received data needs a
physical loopback or device.

For `spi-loopback`, connect SPI1 PA7 (MOSI) to PA6 (MISO). The test compares
blocking byte and DMA halfword transfers, unequal buffer lengths, and dummy
reads in all modes and both bit orders. It additionally checks completion of
both receive-only constructors; MOSI is disconnected in those constructors,
so their received values are intentionally not checked. `SPI_LOOPBACK_RESULT`
is 0 when skipped, 1 running, and 2 passed. The example manages no CS pin;
do not leave an external device selected during this loopback test.

Both configurations build and link. `hal_spi` passed its 24 configuration and
lifecycle cases on hardware on 2026-11-03, after fixing `blocking_flush` (it
polled `TXE` while `SPE=0`, hanging the first write). The loopback cases remain
opt-in (`spi-loopback`) and still require MOSI–MISO wiring. Host register-model
tests pass, including cancellation, error IRQ wakeup, flush and 16-bit dummy
configuration; these do not establish physical signal timing or actual DMA data
integrity.

## HAL DAC and RNG validation

From this example directory:

```sh
cargo run --release --bin hal_dac --features embassy
cargo run --release --bin hal_dac --features embassy,clock-pll,clock-dividers
cargo run --release --bin hal_rng --features embassy
cargo run --release --bin hal_rng --features embassy,clock-pll,rng-usbhs-pll
```

`hal_dac` uses PA4/PA5 and requires no input wiring. It compares digital DOR
readback for 8-bit and both 12-bit formats, dual software triggering, split
channel lifetime, TIM6-triggered DMA2_CH3/CH4 transfers, circular cancellation,
reuse, and triangle variation. Inspect `DAC_RESULT` (1 running, 2 passed,
10–24 failed), `DAC_CASES` (3 scalar format cases), `DAC_SNAPSHOTS`, and
`DAC_WAVE_RANGE`. DMA completion leaves the channel enabled so the final holding
register value can reach the output on the next trigger; the test waits two
extra trigger periods before checking it. Scalar left-aligned values are
unshifted 12-bit samples, while left-aligned DMA buffers contain samples shifted
into bits 15:4. Measure PA4/PA5 separately to establish analog output accuracy.

`hal_rng` reads 32 words, a 31-byte buffer, and verifies interrupt cleanup and
drop/recreate. Inspect `RNG_RESULT` (1 running, 2 passed, 10–16 failed),
`RNG_SAMPLES`, and `RNG_SOURCE_HZ`. The dropped single-poll read may complete
before cancellation on real hardware; deterministic pending cancellation is
covered by the host event model. This acquisition test does not assess entropy
quality. `rng-usbhs-pll` selects the 480 MHz USBHS PLL source; without it, RNG
uses SYSCLK, independently of the AHB divider. Its PLL mux is shared with I2S.
RNG initialization does not wait for entropy, and fallible read APIs report seed
or clock errors. `rand_core` infallible APIs panic on a hardware failure.

Default and PLL configurations build and link. The host suite passes DAC
lifetime/DMA cancellation and RNG fault/wakeup/cancellation checks. Both
examples passed their on-board checks on 2026-11-03.

## ADC migration check (2026-11-01)

```sh
cargo run --release --bin hal_adc --features embassy
cargo run --release --bin hal_adc --features embassy,clock-pll
cargo run --release --bin hal_adc --features embassy,clock-pll,clock-dividers,adc-usbhs-pll
```

The default example needs no wiring. It checks ADC1 Vref and temperature reads,
concurrent ADC1/ADC2 reads through the shared ADC1_2 interrupt, ADC1 Vref/temperature
scan DMA on DMA1_CH12, ADC2 PA0 DMA on DMA1_CH13, cancellation/reuse and independent
clock lifetimes. PA0 is allowed to float: its result only checks the digital path
and does not validate analog accuracy. `ADC_SAMPLES` contains four initial reads
followed by 32 interleaved Vref/temperature scan frames.

`ADC_RESULT` is 1 while running, 2 on success, and 10–23 on failure.
`ADC_CLOCK_HZ` records the ADC kernel frequency. `ADC_CANCEL_PENDING` records
whether the cancellation exercise actually observed a pending transfer.
Async reads use a 500 ms timeout; timeout cancellation stops conversion and DMA.

The RCC ADC clock can select HCLK or USBHS PLL /1–32. Both paths also pass through
the APB2 prescaler and ADC /2/4/6/8 prescaler, matching the EVT clock calculation.
The driver validates the 30 MHz clock limit, the 2 MS/s conversion limit and the
temperature sensor's recommended 17.1 us acquisition time. At 30 MHz, use a sample
time of at least 2.5 cycles. The PLL example uses APB2 /2 to keep the longest sample
time sufficient for the temperature sensor. Internal reference voltage is nominal
1.2 V; the example does not claim factory-calibrated voltage or temperature.

For an optional analog loopback, connect PA4 to PA0 and add `adc-loopback`.
This checks DAC outputs 512, 2048 and 3584 against ADC1, allowing 100 counts of
error. Do not enable it without that connection. Accurate analog characterization
still requires an external reference and measurement equipment.

The driver currently supports independent regular conversion; injected groups,
dual-ADC modes, watchdog and hardware trigger APIs remain future work.
Host register/event tests and builds pass. The example passed its on-board
digital checks on 2026-11-03; the optional PA4–PA0 analog loopback was not wired.

## Flash migration check (2026-11-01)

```sh
cargo run --release --bin hal_flash --features hal
```

The default example only reads Flash and checks invalid ranges and empty requests.
It records `FLASH_RESULT` (1 running, 2 pass, 10–28 failure), `FLASH_SNAPSHOTS`,
`FLASH_TEST_RANGE` and `FLASH_TEST_CASES`. No erase or program command is issued.

For an explicitly destructive storage test:

```sh
cargo run --release --bin hal_flash --features hal,flash-write-test
cargo run --release --bin hal_flash --features embassy,flash-write-test,clock-pll
```

`flash-write-test` erases existing contents of the final 32 KiB,
`0x080F0000..0x080F8000`. It enables `memory-split-usr` and reserves that region
with `flash-test.x`; the ELF contains a NOLOAD reservation with no download bytes.
Code remains in zero-wait Flash. Custom applications must also ensure erase/program
functions and their callees execute from zero-wait Flash, as required by the RM.

V4x7 supports 256-byte fast programming, 16-bit standard programming, 4 KiB sector
erase and 32 KiB fast block erase. Raw erased halfwords read as `0xE339`.
The raw driver therefore implements `ReadNorFlash` only. `into_nor_flash(from, to)`
creates an explicit data partition whose XOR encoding normalizes erased bytes to
`0xFF`, allowing the `NorFlash` contract. This representation changes stored bytes
and is unsuitable for firmware images. Erase before reprogramming; the driver does
not provide multiple-write guarantees or track physical programming history.

The optional test checks both erase sizes, both programming modes, unaligned source
buffers, readback verification, repeated-program rejection and the encoded partition.
It checks protection and clock snapshots without changing option bytes, protection
or RAM/code allocation. Interrupts are masked during each physical operation;
the datasheet allows up to 200 ms for a block erase. A busy timeout leaves controller
registers untouched: retain the driver and retry `recover()` once hardware is idle.
`Config::poll_limit` counts register polls, not elapsed time.

The combined host suite passes 104 tests, and the 16-configuration library matrix
passes. Read-only, write-test and Embassy/200 MHz PLL builds link successfully.
ELF inspection confirms the reserved range and zero-wait operation entrypoints.
Read-only and `flash-write-test` flows passed on hardware on 2026-11-03, after
fixing the fast-page busy wait (it waited for `BSY|WRBSY` instead of `WRBSY`).
Endurance and worst-case erase timing remain untested.

## CAN migration check (2026-11-01)

```sh
cargo run --release --bin hal_can --features embassy
cargo run --release --bin hal_can --features embassy,clock-pll,clock-dividers
```

The example uses CAN1 SilentLoopback on PA11/PA12, so no transceiver or wiring is
needed. It first checks blocking loopback at 125/500/1000 kbit/s, then 72 standard
and extended data/remote frames with payload lengths 0–8. Exact filters route
standard frames to FIFO0 and extended frames to FIFO1 through banks 0 and 13.
It also checks rejected identifiers, invalid filter banks, canceled pending
receives, reuse, FIFO overrun reporting and clock release when the driver drops.

`CAN_RESULT` is 1 while running, 2 on success and 10–26 on failure.
`CAN_FRAMES` counts standard data, extended data, standard remote and extended
remote frames. `CAN_SNAPSHOTS` contains PCLK1, BTIMR, filter activation, filter
FIFO assignment, interrupt enables and error status.

The dedicated V4x7 API supports both receive FIFOs, all 14 filter banks and
16/32-bit list/mask filters. `Filter::exact` matches ID format and optionally RTR;
raw `Filter::registers` exposes the RM filter representation for other layouts.
Bitrate selection requires an exact clock ratio and uses all ten prescaler bits.
Standard TS1/TS2/SJW ranges follow EVT; extended long-TS1, time-triggered mode and
CAN FD are not enabled. This chip implements Classical CAN 2.0A/B.

Both blocking and async transmission wait for a completion result. Nonblocking
`try_transmit` only submits a frame and returns a mailbox; its result is valid
until that mailbox is reused or the controller is dropped. Pending frames are
never replaced. The embedded-can blocking/nb traits receive from FIFO0; explicit
receive methods select either FIFO. Wrap async operations in `with_timeout` as
needed. Blocking `Config::poll_limit` counts register polls rather than time.
Canceling transmit requests abort only for the future's own mailbox; it cannot
withdraw a frame already sent on the bus. Receive cancellation disables its
interrupt sources. An overrun reports loss while leaving queued frames readable.

The combined host suite passes 128 tests, including CAN frame/timing/filter and
controller lifecycle/interrupt/cancellation checks. All 16 library configurations
pass, and both example configurations build and link. The 72-frame internal
loopback passed on hardware on 2026-11-03. Real bus timing, arbitration,
transceivers and electrical quality still need an external CAN setup.

## HAL SDIO validation

```sh
cargo run --release --bin hal_sdio --features embassy
cargo run --release --bin hal_sdio --features embassy,sdio-4bit
cargo run --release --bin hal_sdio --features embassy,sdio-4bit,sdio-remap
cargo run --release --bin hal_sdio --features embassy,sdio-4bit,clock-pll,sdio-high-speed
```

Use a powered 3.3 V SD socket and common ground. Connect CK to PC12, CMD to PD2,
and D0 to PC8. Four-bit mode additionally uses PC9/D1, PC10/D2 and PC11/D3.
`sdio-remap` moves D0/D1 to PB14/PB15; CK, CMD and D2/D3 stay fixed. CMD and data
lines require external pull-ups as specified by the card/socket design. The
driver owns DMA2 channel 4. GPIO input test wiring is not needed for this test.

The example only reads: it initializes the card, reads SD Status and block 0,
compares blocking and asynchronous reads, compares a two-block read with single
reads, and cancels an asynchronous read if it is still pending on its first poll.
After cancellation it checks DMA shutdown, reinitializes and repeats block 0.
It never writes to the card. No card response is reported separately from a
passing read test.

Default HSI gives HCLK 20 MHz. For this example only, `clock-pll` selects HCLK
100 MHz (AHB /4): the eight-bit SDIO divider cannot produce the identification
clock at HCLK 200 MHz. The driver rejects that clock configuration with
`BadClock`. `clock-dividers` changes only APB clocks to check the HCLK kernel
source. The requested transfer clock is a maximum: 25 MHz by default, 50 MHz
with `sdio-high-speed`, subject to card capability and the available divider.
`sdio-bypass` allows bypass with hardware flow control; it is disabled by default.

ELF symbols: `SDIO_RESULT` is 1 while running, 2 for no card response, 3 for a
passing read test and 10–26 for a failed check. `SDIO_CARD_INFO` contains the low
and high block-count words, transfer clock in Hz, RCA and high-speed selection.
`SDIO_FIRST_BYTES` contains the first 64 bytes of block 0; `SDIO_CANCELLED`
records whether the cancellation path was exercised. The example ran on hardware
on 2026-11-03 and reported `SDIO_RESULT=2` (no card response), as expected with
no card inserted. Card read, high-speed and cancellation paths remain untested.

## HAL Ethernet validation

```sh
cargo run --release --bin hal_eth --features eth
cargo run --release --bin hal_eth --features eth,clock-pll
cargo run --release --bin hal_eth --features eth,clock-dividers
```

This example requires a 25 MHz HSE crystal and enables ETH PLL. Default HCLK is
20 MHz; `clock-pll` selects 200 MHz, and `clock-dividers` changes only APB clocks.
The driver uses the integrated 10/100M PHY and dedicated differential pins,
with a board Ethernet jack/magnetics circuit. It needs no external RMII GPIO
connections. Optional PHY GPIO LEDs are disabled, preserving PE8/PE9 for other
examples. The example uses the local MAC address `02:00:00:00:00:47`; use a unique
address when running multiple boards on the same network.

The driver implements `embassy-net-driver` with software checksums and static
DMA resources. This example tests raw frames without IP configuration. On a
Linux peer attached to the board's Ethernet network, run:

```sh
sudo python3 eth_echo_test.py eth0
```

Replace `eth0` with the wired interface. The script needs root or CAP_NET_RAW.
It verifies 100 echoes over payload lengths 46/47/128/512/1500 bytes, including
per-frame sequence and data checks. The example echoes only EtherType 0x88b5
frames addressed to its MAC with the `CH32V4-ECHO` prefix. Real frame exchange
has not yet been tested; the script's echo, timeout and corruption paths passed
a host socket model.

`ETH_RESULT`: 1 entering init, 2 initialized with no link, 3 link up, 4 an echo
queued, 10 init failure, 11 runtime error. Result 4 alone is not proof of delivery;
the peer script must verify the returned frame. `ETH_PHY_ID` records the PHY ID;
`ETH_STATS` contains received frames, queued echoes, TX errors, RX errors and DMA
errors. No cable is required for the first PHY identification check. On
2026-11-03 initialization succeeded with PHY ID `0x73719011`; frame exchange was
not tested because no cable was attached.

The implementation follows the RM's reduced DMA register map and PHY interrupt
at bit 11. FES access and fatal bus-error bit 13 have conflicting RM/EVT details;
see `tools/eth-host-tests/README.md`. Physical 10/100M negotiation, half duplex,
reset timing and recovery remain hardware acceptance items. Hardware checksum
offload, PTP, wake-on-LAN, VLAN/jumbo frames and a TCP/IP example are not included.

## HAL I3C validation

`hal_i3c` uses the independent V4x7 I3C controller. Default SCL/SDA are
PB10/PB11; `i3c-remap` selects PB4/PB5 (check debug pin ownership before use).

```sh
cargo build --release --bin hal_i3c --features embassy
cargo build --release --bin hal_i3c --features embassy,clock-dividers
cargo build --release --bin hal_i3c --features embassy,i3c-remap
cargo build --release --bin hal_i3c --features embassy,i3c-dma
```

The default program checks timing/control register readback, controller address,
reinitialization, interrupt masks and clock gating on Drop. Its raw timing values
are register fixtures, not a qualified communication profile. It issues no START
or CCC, and `I3C_RESULT=PASS` only reports those register/lifecycle checks.

For a real I3C target, first review timings for the actual HCLK, target and bus.
Set the build environment variables `I3C_TIMINGR0` and `I3C_TIMINGR1` to the
reviewed raw register values (decimal or `0x` hex), then run:

```sh
cargo run --release --bin hal_i3c --features embassy,i3c-daa
```

There is deliberately no default communication timing profile. Missing variables
cause the device test to fail before controller construction. TIMINGR2 is zero
(no optional stalls). Wire common ground, SCL and SDA to a native I3C target and
use the pull-up arrangement specified by the target/board. An I2C-only target
cannot exercise ENTDAA.

The opt-in device test broadcasts RSTDAA, assigns 0x30–0x33 to at most four
targets, records PID/BCR/DCR, and checks each PID using direct GETPID. RSTDAA
changes all I3C target dynamic addresses on that bus. Prepare a dedicated test
bus with no other active controller. Debugger markers are `I3C_DEVICE_RESULT`
(0 not run, 1 started, 2 passed), `I3C_TARGET_COUNT` and `I3C_TARGETS`.
Failed operations currently panic with the driver error; a device PASS is
separate from the register PASS.

The backend supports blocking and Embassy FIFO transfers, single-direction legacy
I2C via both embedded-hal traits, SDR private transfers, ordinary broadcast/direct
CCC and ENTDAA. RM 19.4.1 explicitly requires MEND=1 for legacy I2C messages.
Legacy transactions changing direction, including write_read, return
`UnsupportedTransaction` before bus activity; they are not split into separate
STOP-delimited transactions. Adjacent operations in one direction are merged.
Repeated starts are supported for private transfers and direct CCC.
Zero-length I2C or
private operations are rejected; zero-length broadcast CCC is supported. Direct
CCC currently requires a nonempty payload and no defining byte. RSTACT,
IBI/hot-join delivery and automatic physical bus recovery are not implemented.

With `i3c-dma`, `I3c::new_with_dma` owns DMA1_CH4 for TX and DMA1_CH5 for RX.
Private and supported legacy transactions with at most one buffer per direction
use byte DMA. Scattered transactions, CCC and ENTDAA retain the FIFO backend.
Both data channels are armed before control words are queued; success requires
zero remaining DMA counts and FCF, with peripheral/DMA errors taking priority.
The normal `i3c-dma` example still performs only register/lifecycle checks.

For an actual data-path test, prepare a native I3C **echo fixture** that supports
RSTDAA/ENTDAA/GETPID and returns the previous private write on a subsequent private
read, including a repeated START between write and read. The fixture must handle
1/7/8/9/31/257-byte payloads. With reviewed `I3C_TIMINGR0` / `I3C_TIMINGR1`, run:

```sh
cargo run --release --bin hal_i3c --features embassy,i3c-dma-loopback
```

This opt-in test writes arbitrary payloads to the first assigned target. Use the
dedicated echo fixture rather than a sensor or other application device.
`I3C_DMA_RESULT=2` indicates that separate write/read and repeated-start echo
checks passed and that both DMA channels and requests stopped afterward.
Communication failure returns the driver error. Neither successful echo nor
the host model establishes a maximum bus speed. Measure SCL/SDA and exercise
representative interrupt load before qualifying a timing profile.

The FIFO backend has only eight-byte data FIFOs. Neither the host model nor a
successful build establishes a sustainable bus speed or interrupt latency.
DMA also needs validation with a logic analyzer and representative interrupt
load before claiming high-rate SDR support. Legacy I2C does not promise
clock-stretching compatibility. Failed/cancelled frames require explicit
`reinitialize()` after restoring an idle physical bus; this method does not
clear a stuck target or reset its dynamic address. ENTDAA output is provisional
on error, including capacity exhaustion. Supplied addresses must be unused on
the existing bus; the driver only checks reservations, duplicates and its own
address.

The register/lifecycle check passed on hardware on 2026-11-03. No external
I3C target or DMA echo fixture has been tested yet.

## Local validation (2026-10-28)

Historical record: the SysTick high-word definitions described below were
disproved on 2026-11-01. V4x7 has a 32-bit CNT/CMP; the PAC and current examples
have been corrected. Current hardware results supersede the earlier untested status.

- `cpuid` passes release build and Clippy `-D warnings` for WEU6, RET6 and VET6,
  plus rustfmt. Its CSR values, exception recovery and live RTT report have
  not been hardware-validated; no measured CPU identity is asserted here.
- After regenerating the PAC with V4x7 SysTick 64-bit/high-word access views,
  both RTT binaries build and link for RET6, VET6 and WEU6; all three release
  Clippy configurations pass with `-D warnings`. Rustfmt checks also pass.
  The earlier missing `cnth()` / `cmph()` build error is resolved without
  changing the example's split-word writes while SysTick is stopped.
- Both ELFs contain `.defmt` metadata and `_SEGGER_RTT` at `0x20000008`;
  heartbeat markers are in RAM and stack top remains `0x20032000`.
- `llvm-size` text/data/bss: `blinky` 2206/56/1040 bytes;
  `rtt` 2102/56/1040 bytes (including the default 1024-byte RTT buffer).
- The installed PATH tool now supports V467. After installation, target
  lookup and `info --verbose --protocol jtag` both passed.
- No on-board RTT testing or flashing has been performed by the agent.
