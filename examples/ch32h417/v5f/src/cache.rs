//! Hart 1's own instruction cache (QingKe V5 manual §8.2, CSR `cache_strtg_ctlr`).
//!
//! The core has a 32 KB, 2-way, 8-byte-line I-cache (`meminfo` reports
//! `Icache_datasize = 100b`), but **`ic_disable` resets to 1**, i.e. the cache is
//! off until software turns it on. With it off every fetch of this image's
//! flash-resident code goes to flash, which is why hart 1 looks an order of
//! magnitude slower than hart 0 unless it enables the cache itself.
//!
//! The three region permits (`ic_code/mem0/mem1/sram_strtg`) reset to 1, so
//! clearing `ic_disable` is all the setup this image needs; the flash window
//! (`0x0000_0000-0x1fff_ffff`) is already cacheable.
//!
//! This cache is hart 1's, and the datasheet's "core 1" block (32 KB I-cache,
//! 128K ITCM, 256K DTCM) describes this core — the same resources the CSDK's
//! `Link_v5f.ld` places its code and data in. Hart 0 has **no** usable cache: its
//! copy of the same register reads `0x00000003` and ignores writes (QingKe's
//! I-cache is the V4J variant), which is why the boot core has to run hot code
//! from RAM instead (see the CSDK's `RAM_CODE`).

/// `ic_disable` in `cache_strtg_ctlr` (0xBC2): 0 enables instruction caching.
const IC_DISABLE: u32 = 1 << 1;

/// `csrr` with a literal CSR number — `csrr`/`csrw` take the CSR as an
/// immediate, so a runtime value cannot be used.
macro_rules! csrr {
    ($csr:literal) => {{
        let value: u32;
        unsafe { core::arch::asm!(concat!("csrr {0}, ", $csr), out(reg) value) };
        value
    }};
}

/// `csrw` with a literal CSR number.
macro_rules! csrw {
    ($csr:literal, $value:expr) => {{
        let value: u32 = $value;
        unsafe { core::arch::asm!(concat!("csrw ", $csr, ", {0}"), in(reg) value) };
    }};
}

/// Instruction cache size in bytes from `meminfo`, or 0 when the core reports
/// none.
pub fn icache_size() -> u32 {
    const SIZES: [u32; 8] = [0, 4, 8, 16, 32, 64, 128, 256];
    SIZES[((csrr!("0xfc0") >> 2) & 0b111) as usize] * 1024
}

/// Whether instruction caching is currently enabled (`ic_disable == 0`).
pub fn icache_enabled() -> bool {
    csrr!("0xbc2") & IC_DISABLE == 0
}

/// Invalidate the whole instruction cache (`opcache_ctlr`, opcode 00, index
/// mode).
pub fn invalidate_icache() {
    csrw!("0xbd0", 0);
}

/// Turn instruction caching on. Checks `icache_enabled` afterwards, so a core
/// that refuses the write cannot silently keep running uncached.
pub fn enable_icache() {
    let current = csrr!("0xbc2");
    if current & IC_DISABLE != 0 {
        csrw!("0xbc2", current & !IC_DISABLE);
        debug_assert!(icache_enabled(), "instruction cache did not turn on");
    }
}

/// Turn instruction caching off (for measuring its effect).
pub fn disable_icache() {
    csrw!("0xbc2", csrr!("0xbc2") | IC_DISABLE);
}
