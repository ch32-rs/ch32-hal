//! The example protocols carried in [`crate::Mailbox`].
//!
//! Each is the payload of exactly the examples that share it, so an example no
//! longer reserves words for protocols it never runs. Both harts include this
//! module, and the field order is the ABI between them — and what `xtask report`
//! decodes — so a field is added at the end or not at all.
//!
//! Nothing here is hardware. The constants and the `#[repr(C)]` layouts are the
//! whole contract, which is why they live with the mailbox rather than inside
//! one hart's crate.

use core::sync::atomic::{AtomicU32, Ordering};

use crate::Protocol;

/// No payload at all.
///
/// For an image that defines a mailbox only so it can clear one, and never
/// reads a field — the launcher, which starts whatever V5F image is already
/// programmed and so cannot know that image's layout.
pub struct Empty;

impl Protocol for Empty {
    const ZERO: Self = Self;
}

/// `dualcore`: hart 1 writes [`DUALCORE_MARKER`] once it has run.
pub const DUALCORE_MARKER: u32 = 0xDEAD_BEEF;

/// `dualcore`: hart 1 ran, the boot core counts.
#[repr(C)]
pub struct Dualcore {
    pub marker: AtomicU32,
    pub counter: AtomicU32,
}

impl Protocol for Dualcore {
    const ZERO: Self = Self {
        marker: AtomicU32::new(0),
        counter: AtomicU32::new(0),
    };
}

/// `delay_systick`: the boot core's one-second ticks, and hart 1's own.
#[repr(C)]
pub struct DelaySystick {
    /// The boot core, driven by `hal::delay::Delay`.
    pub v3f_ticks: AtomicU32,
    /// Hart 1, driven by its counted spin.
    pub v5f_ticks: AtomicU32,
}

impl Protocol for DelaySystick {
    const ZERO: Self = Self {
        v3f_ticks: AtomicU32::new(0),
        v5f_ticks: AtomicU32::new(0),
    };
}

/// `rcc_delay_check`: how far the boot core's self-check got.
///
/// `stage` is 1 once init is entered, 2 once it returns, 3 once every check
/// passed. `failures` counts the checks that did not.
#[repr(C)]
pub struct RccDelayCheck {
    pub stage: AtomicU32,
    pub failures: AtomicU32,
    /// The V3F core frequency the clock tree reported, in Hz.
    pub v3f_hz: AtomicU32,
    /// The V5F core frequency the clock tree reported, in Hz.
    pub v5f_hz: AtomicU32,
    /// One-second heartbeat, so a report shows the loop is still running.
    pub heartbeat: AtomicU32,
}

impl Protocol for RccDelayCheck {
    const ZERO: Self = Self {
        stage: AtomicU32::new(0),
        failures: AtomicU32::new(0),
        v3f_hz: AtomicU32::new(0),
        v5f_hz: AtomicU32::new(0),
        heartbeat: AtomicU32::new(0),
    };
}

/// `embassy_systick`: the boot core's Embassy timer against a free-running TIM2.
///
/// `stage` is 1 once the executor starts, 2 once the measured wait passes.
/// `error_us` is the TIM2-measured length of an Embassy 100 ms wait, minus
/// 100 ms. `ticks` advances once a second from `embassy_time`.
#[repr(C)]
pub struct EmbassySystick {
    pub stage: AtomicU32,
    pub error_us: AtomicU32,
    pub ticks: AtomicU32,
}

impl Protocol for EmbassySystick {
    const ZERO: Self = Self {
        stage: AtomicU32::new(0),
        error_us: AtomicU32::new(0),
        ticks: AtomicU32::new(0),
    };
}

/// Increments each core performs in the `atomics` example, on the *same* word.
pub const CAS_INCREMENTS: u32 = 1_000_000;

/// `atomics`: `done` bits, one per core.
pub const CAS_DONE_V3F: u32 = 1 << 0;
/// `atomics`: hart 1 has finished its increments.
pub const CAS_DONE_V5F: u32 = 1 << 1;

/// `atomics`: the contended word and the per-core completion flags.
#[repr(C)]
pub struct Atomics {
    /// Both cores `fetch_add` this. The total must equal `2 * CAS_INCREMENTS`
    /// for the hardware atomics to be usable across cores.
    pub total: AtomicU32,
    /// [`CAS_DONE_V3F`] / [`CAS_DONE_V5F`], set once a core has finished.
    pub done: AtomicU32,
}

impl Protocol for Atomics {
    const ZERO: Self = Self {
        total: AtomicU32::new(0),
        done: AtomicU32::new(0),
    };
}

/// Number of Machine-mode CSRs hart 1 probes into [`CpuidBlock`].
pub const CPUID_CSRS: usize = 14;

/// Names of those CSRs, in the order hart 1 writes them. The examples print
/// them; `xtask report` labels them.
pub const CPUID_CSR_NAMES: [&str; CPUID_CSRS] = [
    "mvendorid",
    "marchid",
    "mimpid",
    "mhartid",
    "misa",
    "mstatus",
    "mtvec",
    "mepc",
    "mcause",
    "corecfgr",
    "intsyscr",
    "gintenr",
    "cache_strtg_ctlr",
    "cache_pmp_ovr",
];

/// Index of `misa` in [`CPUID_CSR_NAMES`].
pub const CPUID_MISA_INDEX: usize = 4;

/// Written into [`CpuidBlock::done`] once the whole block is in place.
pub const CPUID_DONE: u32 = 0xC0DE_0001;

/// Hart 1's own Machine-mode CSRs, as published through the mailbox.
#[repr(C)]
pub struct CpuidBlock {
    /// Bit `i` set when `values[i]` could be read.
    pub present: AtomicU32,
    /// In [`CPUID_CSR_NAMES`] order.
    pub values: [AtomicU32; CPUID_CSRS],
    /// ISA features the V5F image was built for.
    pub buildcfg: AtomicU32,
    /// [`CPUID_DONE`] when the block is complete.
    pub done: AtomicU32,
    /// How many CSRs hart 1 has probed, for diagnosing a hang.
    pub progress: AtomicU32,
}

impl CpuidBlock {
    const ZERO: Self = Self {
        present: AtomicU32::new(0),
        values: [const { AtomicU32::new(0) }; CPUID_CSRS],
        buildcfg: AtomicU32::new(0),
        done: AtomicU32::new(0),
        progress: AtomicU32::new(0),
    };

    /// Read [`Self::values`] into a plain array.
    pub fn snapshot(&self) -> [u32; CPUID_CSRS] {
        let mut values = [0u32; CPUID_CSRS];
        for (out, slot) in values.iter_mut().zip(&self.values) {
            *out = slot.load(Ordering::Relaxed);
        }
        values
    }
}

/// `cpuid`: hart 1's CSR block and nothing else.
#[repr(C)]
pub struct Cpuid {
    pub cpuid: CpuidBlock,
}

impl Protocol for Cpuid {
    const ZERO: Self = Self {
        cpuid: CpuidBlock::ZERO,
    };
}

/// `pingpong`: hart 1's CSR block, then the round counters.
///
/// `ping` is hart 1's counter, incremented once per answered round. `pong` is
/// the boot core's echo — equal to `ping` once it has answered, so `ping !=
/// pong` means a round is outstanding.
#[repr(C)]
pub struct Pingpong {
    pub cpuid: CpuidBlock,
    pub ping: AtomicU32,
    pub pong: AtomicU32,
}

impl Protocol for Pingpong {
    const ZERO: Self = Self {
        cpuid: CpuidBlock::ZERO,
        ping: AtomicU32::new(0),
        pong: AtomicU32::new(0),
    };
}

/// `sdi_cpuid`: the console token and hart 1's liveness counter, then its CSRs.
///
/// `token` is 1 while hart 1 owns the SDI console and 0 when the boot core does.
#[repr(C)]
pub struct SdiCpuid {
    pub token: AtomicU32,
    pub ticks: AtomicU32,
    pub cpuid: CpuidBlock,
}

impl Protocol for SdiCpuid {
    const ZERO: Self = Self {
        token: AtomicU32::new(0),
        ticks: AtomicU32::new(0),
        cpuid: CpuidBlock::ZERO,
    };
}

/// `bme280`: hart 1 has not started the sensor yet.
pub const BME280_IDLE: u32 = 0;
/// `bme280`: hart 1 is bringing the sensor up.
pub const BME280_INIT: u32 = 1;
/// `bme280`: the sensor answered and is a BME280 (humidity present).
pub const BME280_OK: u32 = 2;
/// `bme280`: the sensor answered but is a BMP280 (no humidity).
pub const BME280_BMP: u32 = 3;
/// `bme280`: the sensor did not answer, or init failed.
pub const BME280_FAIL: u32 = 4;

/// `seq` with no sample published yet.
pub const BME280_NO_SAMPLE: u32 = 0;

/// `bme280`: hart 1's sensor state and its latest sample.
#[repr(C)]
pub struct Bme280 {
    /// One of the `BME280_*` constants.
    pub status: AtomicU32,
    /// Hart 1's sample counter. [`BME280_NO_SAMPLE`] until the first reading is
    /// published; the boot core prints a sample when this changes.
    pub seq: AtomicU32,
    /// Temperature in 1/100 °C, as `edrv_bme280` reports it.
    pub temperature: AtomicU32,
    /// Pressure in 1/100 Pa.
    pub pressure: AtomicU32,
    /// Relative humidity in 1/100 %RH (0 on a BMP280).
    pub humidity: AtomicU32,
}

impl Protocol for Bme280 {
    const ZERO: Self = Self {
        status: AtomicU32::new(0),
        seq: AtomicU32::new(0),
        temperature: AtomicU32::new(0),
        pressure: AtomicU32::new(0),
        humidity: AtomicU32::new(0),
    };
}

/// `spi_check`: the boot core's SPI1 blocking self-check.
///
/// `stage` is 1 once init returns, 2 once every mode and bit order ran, and 3
/// once none of them failed. `failures` counts failed checks and `cases` the
/// completed transfers. `ctlr1` and `hscr` are the registers read back after
/// the last configuration. `loopback` is 1 only when the `spi-loopback`
/// feature ran with PA7 jumpered to PA6 and the received bytes matched.
#[repr(C)]
pub struct SpiCheck {
    pub stage: AtomicU32,
    pub failures: AtomicU32,
    pub cases: AtomicU32,
    pub ctlr1: AtomicU32,
    pub hscr: AtomicU32,
    pub loopback: AtomicU32,
}

impl Protocol for SpiCheck {
    const ZERO: Self = Self {
        stage: AtomicU32::new(0),
        failures: AtomicU32::new(0),
        cases: AtomicU32::new(0),
        ctlr1: AtomicU32::new(0),
        hscr: AtomicU32::new(0),
        loopback: AtomicU32::new(0),
    };
}

/// `rng_check`: the boot core's RNG blocking self-check.
///
/// `stage` is 1 once init returns and 2 once the sample passed. `failures`
/// counts failed checks, `words` the accepted 32-bit samples, and `checksum`
/// their XOR, which is nonzero for a live generator.
#[repr(C)]
pub struct RngCheck {
    pub stage: AtomicU32,
    pub failures: AtomicU32,
    pub words: AtomicU32,
    pub checksum: AtomicU32,
}

impl Protocol for RngCheck {
    const ZERO: Self = Self {
        stage: AtomicU32::new(0),
        failures: AtomicU32::new(0),
        words: AtomicU32::new(0),
        checksum: AtomicU32::new(0),
    };
}

/// `adc_check`: the boot core's ADC1 blocking self-check.
///
/// `stage` is 1 once init returned and 2 once calibration and the three reads
/// completed. `failures` counts failed checks; `pa1`, `vref` and `temperature`
/// are the raw 12-bit results. The analog accuracy of the first two is not
/// judged, only that the conversion path produced an in-range value.
#[repr(C)]
pub struct AdcCheck {
    pub stage: AtomicU32,
    pub failures: AtomicU32,
    pub pa1: AtomicU32,
    pub vref: AtomicU32,
    pub temperature: AtomicU32,
}

impl Protocol for AdcCheck {
    const ZERO: Self = Self {
        stage: AtomicU32::new(0),
        failures: AtomicU32::new(0),
        pa1: AtomicU32::new(0),
        vref: AtomicU32::new(0),
        temperature: AtomicU32::new(0),
    };
}

/// `dac_check`: the boot core's DAC self-check.
///
/// `stage` is 1 once init returned and 2 once every write was read back.
/// `failures` counts failed checks; `dor1` and `dor2` are the two output
/// registers after the last software-triggered write.
#[repr(C)]
pub struct DacCheck {
    pub stage: AtomicU32,
    pub failures: AtomicU32,
    pub dor1: AtomicU32,
    pub dor2: AtomicU32,
}

impl Protocol for DacCheck {
    const ZERO: Self = Self {
        stage: AtomicU32::new(0),
        failures: AtomicU32::new(0),
        dor1: AtomicU32::new(0),
        dor2: AtomicU32::new(0),
    };
}

/// `flash_check`: the boot core's FPEC self-check.
///
/// `stage` is 1 once init returned and 2 once every erase/program case ran.
/// `failures` counts failed checks and `checks` the cases attempted.
/// `erased_word` is the word read back after a sector erase, `readback` the
/// first word after a fast page write. `geometry` packs the sector and block
/// sizes as `(block << 16) | sector` in 256-byte units.
#[repr(C)]
pub struct FlashCheck {
    pub stage: AtomicU32,
    pub failures: AtomicU32,
    pub checks: AtomicU32,
    pub capacity: AtomicU32,
    pub geometry: AtomicU32,
    pub erased_word: AtomicU32,
    pub readback: AtomicU32,
}

impl Protocol for FlashCheck {
    const ZERO: Self = Self {
        stage: AtomicU32::new(0),
        failures: AtomicU32::new(0),
        checks: AtomicU32::new(0),
        capacity: AtomicU32::new(0),
        geometry: AtomicU32::new(0),
        erased_word: AtomicU32::new(0),
        readback: AtomicU32::new(0),
    };
}

/// `sdio_check`: the boot core's SDIO self-check.
///
/// `stage` is 1 once the controller was constructed and 2 once card init was
/// attempted. `card` is 0 for no card, 1 for SDSC and 2 for SDHC/SDXC.
/// `error` carries the init error code (`0` = none, `1` = timeout, `2` =
/// other). `blocks`, `clock` and `first_word` describe a successfully
/// initialized card; `status` is the raw `STA` register at the end.
#[repr(C)]
pub struct SdioCheck {
    pub stage: AtomicU32,
    pub failures: AtomicU32,
    pub checks: AtomicU32,
    pub card: AtomicU32,
    pub error: AtomicU32,
    pub blocks: AtomicU32,
    pub clock: AtomicU32,
    pub first_word: AtomicU32,
    pub status: AtomicU32,
}

impl Protocol for SdioCheck {
    const ZERO: Self = Self {
        stage: AtomicU32::new(0),
        failures: AtomicU32::new(0),
        checks: AtomicU32::new(0),
        card: AtomicU32::new(0),
        error: AtomicU32::new(0),
        blocks: AtomicU32::new(0),
        clock: AtomicU32::new(0),
        first_word: AtomicU32::new(0),
        status: AtomicU32::new(0),
    };
}
