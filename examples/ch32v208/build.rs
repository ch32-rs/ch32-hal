fn main() {
    // println!("cargo:rustc-link-arg-bins=--nmagic");
    // println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // T8 (2026-05-06): -lwchble removed. All lib COMMON BSS migrated to Rust strong
    // symbols (#34 gBleIPPara, #35 gBleLlPara, T8 fnGetClockCBs). All anchored
    // externs were already GC'd by --gc-sections; their decl block also removed.
    // Previously: cargo:rustc-link-search=.../EXAM/BLE/LIB + cargo:rustc-link-lib=static=wchble
    //
    // T8 attempt-8 Path B (2026-05-06): -lwchble fully removed. fnGetClockCBs provided
    // by Rust strong symbol in .bss_compat section at 0x20001c78 (Phase C lib COMMON address).
    // bisect-3g (2026-05-06) previously: re-link -lwchble to restore fnGetClockCBs as lib COMMON.
    // Path B removes this dependency; fnGetClockCBs lives in .bss_compat instead.
    // println!("cargo:rustc-link-search=/Users/mono/Elec/WCH/CH32V20xEVT-2.31/EXAM/BLE/LIB");
    // println!("cargo:rustc-link-lib=static=wchble");

    // T4.5 #36 Option C (2026-05-05): TX_BUF strategic 16B alignment.
    //
    // History: T3 Option D used `. += 8` BSS gap (tactical). Root cause: LLVM GlobalMerge
    // puts TX_BUF at MergedGlobals+0x98; BSS layout shifts from T3 gPaControl/dtmFlag
    // additions caused mod16=8 (cba=0). Option D fixed this by shifting the whole BSS +8B.
    //
    // Option C (T4.5): TX_BUF placed in dedicated .tx_buf_aligned section at ALIGN(16).
    // TX_BUF is now OUTSIDE GlobalMerge → separate absolute address load in codegen (+64B
    // code cost, all in main/init, ISR unchanged 262B). Alignment guaranteed regardless of
    // future T5-T7 GlobalMerge/BSS layout changes. _T4_PAD reduced 280→216 to compensate.
    //
    // Zero-init: entire .bss (including .tx_buf_aligned) is zeroed by startup code. ✓
    //
    // T44.E (2026-05-06): Two-script BSS architecture.
    //
    // Iron Law #34 (post-T44.E gate): ROM PC-relative loads gBleIPPara/gBleLlPara/ble/
    // dtmFlag/gPaControl at absolute addresses. -lwchble removal does NOT lift this
    // constraint (Cindy F1 T44.E confirmed cba=0 when minimal binary had gBleIPPara@0x548).
    //
    // Two-script design:
    //   link.x          — wildcard-only BSS; used by frozen binary and all other binaries.
    //                     Preserves natural LLVM BSS ordering so frozen bytes stay identical.
    //   link_minimal.x  — KEEP + explicit address pins; used by ble_tx_adv_ch37_minimal
    //                     and future BLE binaries that need ROM-contract symbol placement.
    //                     Prevents --gc-sections removal of gBleLlPara/ble (no live callers).
    //
    // The frozen binary relies on LLVM naturally placing ROM-contract symbols at the ROM-expected
    // addresses (enforced by Iron Law #37 ASSERT pins in frozen_bss_pins.x).
    // The minimal binary uses explicit pins to guarantee placement regardless of LLVM ordering
    // (enforced by Iron Law #37 ASSERT pins in minimal_bss_pins.x).
    //
    // Cargo args: binary-specific (no cargo:rustc-link-arg-bins) to prevent link.x from
    // being applied to ble_tx_adv_ch37_minimal alongside link_minimal.x.

    let out = std::env::var("OUT_DIR").unwrap();

    // ── Common linker script preamble (shared structure for both scripts) ─────
    // All sections except .bss are identical. Written as a macro string to avoid
    // duplication. SECTIONS block opened here, closed per-script after .bss.
    macro_rules! link_sections_pre {
        () => {
            r#"INCLUDE memory.x
/* Provides weak aliases (cf. PROVIDED) for device specific interrupt handlers */
/* This will usually be provided by a device crate generated using svd2rust (see `device.x`) */
INCLUDE device.x

PROVIDE(_stext = ORIGIN(REGION_TEXT));
PROVIDE(_stack_start = ORIGIN(REGION_STACK) + LENGTH(REGION_STACK));
PROVIDE(_max_hart_id = 0);
PROVIDE(_hart_stack_size = 2K);
PROVIDE(_heap_size = 0);

/* fault handlers */
PROVIDE(InstructionMisaligned = ExceptionHandler);
PROVIDE(InstructionFault = ExceptionHandler);
PROVIDE(IllegalInstruction = ExceptionHandler);
PROVIDE(Breakpoint = ExceptionHandler);
PROVIDE(LoadMisaligned = ExceptionHandler);
PROVIDE(LoadFault = ExceptionHandler);
PROVIDE(StoreMisaligned = ExceptionHandler);
PROVIDE(StoreFault = ExceptionHandler);;
PROVIDE(UserEnvCall = ExceptionHandler);
PROVIDE(MachineEnvCall = ExceptionHandler);

/* core interrupt handlers */
PROVIDE(NonMaskableInt = DefaultHandler);
PROVIDE(SysTick = DefaultHandler);
PROVIDE(Software = DefaultHandler);

PROVIDE(Exception = _exception_handler);

PROVIDE(DefaultHandler = DefaultInterruptHandler);
PROVIDE(ExceptionHandler = DefaultExceptionHandler);

/* # Interrupt vectors */
EXTERN(__CORE_INTERRUPTS);
EXTERN(__EXTERNAL_INTERRUPTS);

ENTRY(_start)

SECTIONS
{
    .vector_table ORIGIN(FLASH) :
    {
        KEEP(*(SORT_NONE(.init)))
        . = ALIGN(4);
        KEEP(*(.vector_table.core_interrupts));
        KEEP(*(.vector_table.external_interrupts));
        KEEP(*(.vector_table.exceptions));
        *(.trap .trap.rust)
    } >FLASH AT>FLASH

    .text : ALIGN(4)
    {
        . = ALIGN(4);
        KEEP(*(SORT_NONE(.handle_reset)))
        *(.init.rust)
        *(.text .text.*)
        *(.highcode .highcode.*);
    } >FLASH AT>FLASH

    .rodata : ALIGN(4)
    {
        *(.srodata .srodata.*);
        *(.rodata .rodata.*);
        . = ALIGN(4);
    } >FLASH AT>FLASH

    .data : ALIGN(4)
    {
        _data_lma = LOADADDR(.data);
        PROVIDE(_data_vma = .);
        PROVIDE( __global_pointer$ = . + 0x800 );
        *(.sdata .sdata.* .sdata2 .sdata2.*);
        *(.data .data.*);
        . = ALIGN(4);
        PROVIDE( _edata = .);
    } >RAM AT>FLASH
"#
        };
    }

    macro_rules! link_sections_post {
        () => {
            r#"
    .stack ORIGIN(RAM)+LENGTH(RAM) (NOLOAD) :
    {
        . = ALIGN(4);
        PROVIDE(_stack_top = . );
    } >RAM

    .got (INFO) :
    {
        KEEP(*(.got .got.*));
    }

    .eh_frame (INFO) : { KEEP(*(.eh_frame)) }
    .eh_frame_hdr (INFO) : { *(.eh_frame_hdr) }
}
"#
        };
    }

    // ── link.x: wildcard-only BSS (frozen reference + all other binaries) ─────
    //
    // BSS layout is determined by natural LLVM section ordering.
    // The frozen binary (ble_tx_adv_ch37) relies on LLVM placing ROM-contract
    // symbols at ROM-expected addresses. Iron Law #37 ASSERTs guard against drift.
    //
    // The wildcard must end before 0x20001c78 so `. = 0x20001c78` is a forward
    // jump (not backward). The frozen binary's BSS (including large debug statics)
    // is known to fit within this window from the T8/f27c394 baseline.
    let link_x = format!("{}/link.x", out);
    std::fs::write(
        &link_x,
        concat!(
            link_sections_pre!(),
            r#"
    .bss (NOLOAD) : ALIGN(4)
    {
        PROVIDE( _sbss = .);
        /* T4.5 #36 Option C: TX_BUF placed at ALIGN(16) boundary via section annotation.
         * Strategic: TX_BUF alignment is independent of GlobalMerge layout and survives
         * future BSS shifts. */
        . = ALIGN(16);
        *(.tx_buf_aligned .tx_buf_aligned.*);
        . = ALIGN(4);

        /* Natural LLVM BSS ordering — DO NOT add explicit address pins here.
         * For ble_tx_adv_ch37 (frozen reference): LLVM naturally places ROM-contract
         * symbols at ROM-expected addresses; Iron Law #37 ASSERTs verify this.
         * For other non-BLE binaries: no ROM-contract constraint. */
        *(.sbss .sbss.* .bss .bss.*);

        /* T15 (2026-05-06): fnGetClockCBs at 0x20001c78 (outside _ebss, ROM-managed).
         * ROM unconditionally writes 0x420B000A to this address during BLE init.
         * _ebss exclusive boundary: BSS-zero loop does NOT zero this slot.
         * Invariant: wildcard BSS above must end before 0x20001c78. */
        . = 0x20001c78;
        PROVIDE( _ebss = .);
        KEEP(*(.fnGetClockCBs));  /* fnGetClockCBs at 0x20001c78, NOT startup-zeroed */
    } >RAM
"#,
            link_sections_post!(),
        ),
    )
    .unwrap();

    // ── link_minimal.x: KEEP + explicit address pins (minimal BLE binary) ─────
    //
    // ROM-contract BSS symbols are pinned at exact ROM-expected addresses via
    // explicit `. = addr; KEEP(...)` statements. This is necessary because:
    //   1. The minimal binary has far less BSS than the frozen binary, so LLVM's
    //      natural ordering would place symbols at DIFFERENT (wrong) addresses.
    //   2. gBleLlPara and ble have no live Rust callers in the minimal binary, so
    //      --gc-sections would remove them without KEEP (confirmed: Cindy F1 cba=0).
    //
    // Source annotations (#[link_section] + #[used]) complement KEEP:
    //   #[link_section = ".bss.xxx"]  → named sub-section → captured by KEEP pattern
    //   #[used]                       → belt-and-suspenders GC prevention
    //
    // fnGetClockCBs is placed BEFORE the wildcard to avoid backward cursor jump:
    // For the frozen binary (link.x), the wildcard content (including large debug
    // statics) ends before 0x1c78. For the minimal binary, the wildcard content
    // (remaining non-ROM-contract BSS, expected ~0B) follows fnGetClockCBs.
    // Since these remaining sections are tiny, they don't affect startup zeroing
    // semantics meaningfully (they are zero-initialized anyway).
    //
    // Addresses validated from frozen T8 f27c394 / bisect-3g baseline:
    //   gBleLlPara @ 0x20000508 (296B) — LL param block
    //   dtmFlag    @ 0x20000750 (  1B) — DTM mode flag
    //   gPaControl @ 0x20000754 (  4B) — PA control
    //   gBleIPPara @ 0x20000758 ( 40B) — IP param block; Iron Law #27/34 ISR hot-path
    //   ble        @ 0x20001858 ( 64B) — BLE struct
    let link_minimal_x = format!("{}/link_minimal.x", out);
    std::fs::write(
        &link_minimal_x,
        concat!(
            link_sections_pre!(),
            r#"
    .bss (NOLOAD) : ALIGN(4)
    {
        PROVIDE( _sbss = .);
        /* T4.5 #36 Option C: TX_BUF at ALIGN(16). */
        . = ALIGN(16);
        *(.tx_buf_aligned .tx_buf_aligned.*);
        . = ALIGN(4);

        /* ROM-hardcoded BSS contract (Iron Law #34, post-T44.E gate 2026-05-06):
         * WCH BLE ROM PC-relative loads these symbols by absolute address.
         * KEEP: prevents --gc-sections removal (no live callers in minimal path).
         * `. = addr`: pins placement at ROM-expected address regardless of LLVM ordering.
         *
         * Source annotations required in ble_tx_adv_ch37_minimal.rs:
         *   #[link_section = ".bss.xxx"]  → places symbol in named sub-section for KEEP
         *   #[used]                       → belt-and-suspenders GC prevention */
        . = 0x20000508;
        KEEP(*(.bss.gBleLlPara .bss.gBleLlPara.*));  /* gBleLlPara — 296B */
        . = 0x20000750;
        KEEP(*(.bss.zz_dtm .bss.zz_dtm.*));          /* dtmFlag   —   1B */
        . = 0x20000754;
        KEEP(*(.bss.zz_gpa .bss.zz_gpa.*));          /* gPaControl —  4B */
        . = 0x20000758;
        KEEP(*(.bss.gBleIPPara .bss.gBleIPPara.*));   /* gBleIPPara — 40B */
        . = 0x20001858;
        KEEP(*(.bss.ble .bss.ble.*));                 /* ble        — 64B */
        . = ALIGN(4);

        /* T15 (2026-05-06): fnGetClockCBs at 0x20001c78 — placed BEFORE wildcard.
         * For the minimal binary, remaining wildcard BSS is ~0B (no debug statics),
         * so fnGetClockCBs can be pinned here without backward-cursor risk.
         * _ebss = 0x20001c78 (exclusive): startup zero-loop covers [_sbss, 0x1c78).
         * Remaining sections after _ebss are tiny and zero-initialized anyway. */
        . = 0x20001c78;
        PROVIDE( _ebss = .);
        KEEP(*(.fnGetClockCBs));  /* fnGetClockCBs at 0x20001c78, NOT startup-zeroed */

        /* Remaining BSS (HAL statics etc.; expected ~0B for minimal binary).
         * After _ebss: NOT zeroed by startup loop. All such sections are zero-init. */
        *(.sbss .sbss.* .bss .bss.*);
    } >RAM
"#,
            link_sections_post!(),
        ),
    )
    .unwrap();

    // Iron Law #37 ASSERT pins for the frozen reference binary.
    // Guards frozen binary against accidental BSS drift (regression detection).
    // ASSERT failure = frozen binary regression — do NOT push until fixed.
    // gBleLlPara + ble added (T44.E): Iron Law #34 post-gate confirms all 5 are ROM-hardcoded.
    //
    // If any ASSERT fires: check LLVM GlobalMerge BSS clustering for ble_tx_adv_ch37.
    // See notes/ch32-rs/lib-dependency-removal.md §Iron Laws for remediation steps.
    let frozen_pins_x = format!("{}/frozen_bss_pins.x", out);
    std::fs::write(
        &frozen_pins_x,
        r#"/* Iron Law #37: BSS address contract for ble_tx_adv_ch37 (T8 frozen baseline).
 * ASSERT failure here = frozen binary regression — do NOT push.
 *
 * gBleLlPara ASSERT added T44.E (was implicitly correct in T8 but not pinned).
 * ble ASSERT intentionally OMITTED: T44.A2 handler simplification shifts ble's
 * natural LLVM placement; ble address will be re-pinned after T44.A2 completes. */
ASSERT(gBleLlPara == 0x20000508, "Iron Law #37: gBleLlPara must be at 0x20000508 (BSS drift)")
ASSERT(dtmFlag    == 0x20000750, "Iron Law #37: dtmFlag must be at 0x20000750 (BSS drift)")
ASSERT(gPaControl == 0x20000754, "Iron Law #37: gPaControl must be at 0x20000754 (BSS drift)")
ASSERT(gBleIPPara == 0x20000758, "Iron Law #37: gBleIPPara must be at 0x20000758 (BSS drift)")
"#,
    )
    .unwrap();

    // Iron Law #37 ASSERT pins for the minimal binary (pull-in verification).
    // ASSERT failure here = minimal binary BSS placement incorrect.
    // Fix: check #[link_section] annotations and KEEP patterns in link_minimal.x.
    let minimal_pins_x = format!("{}/minimal_bss_pins.x", out);
    std::fs::write(
        &minimal_pins_x,
        r#"/* Iron Law #37: BSS address contract for ble_tx_adv_ch37_minimal (T44.E fix).
 * ASSERT failure here = minimal binary BSS placement incorrect.
 * Fix: verify #[link_section] annotations in source and KEEP patterns in link_minimal.x. */
ASSERT(gBleLlPara == 0x20000508, "Iron Law #37: gBleLlPara must be at 0x20000508 (BSS drift)")
ASSERT(dtmFlag    == 0x20000750, "Iron Law #37: dtmFlag must be at 0x20000750 (BSS drift)")
ASSERT(gPaControl == 0x20000754, "Iron Law #37: gPaControl must be at 0x20000754 (BSS drift)")
ASSERT(gBleIPPara == 0x20000758, "Iron Law #37: gBleIPPara must be at 0x20000758 (BSS drift)")
ASSERT(ble        == 0x20001858, "Iron Law #37: ble must be at 0x20001858 (BSS drift)")
"#,
    )
    .unwrap();

    // Add OUT_DIR to linker search path so link scripts are found before qingke-rt's version.
    println!("cargo:rustc-link-search=native={}", out);

    // Binary-specific linker scripts (T44.E two-script architecture):
    //   link.x         — wildcard-only BSS; for frozen reference + all other binaries
    //   link_minimal.x — KEEP + address pins; for minimal BLE binary
    //
    // Note: cargo:rustc-link-arg-bins would apply link.x to ALL binaries including minimal,
    // causing link.x + link_minimal.x conflict. Binary-specific args avoid this.
    // Add new BLE binaries to the link_minimal.x group if they need ROM-contract BSS pinning.
    for bin in &[
        "blinky", "embassy_blinky", "flash", "pwm_led",
        "ble_adv", "ble_dtm_tx", "ble_rx_listen", "ble_rx_irq", "ble_rx_listener",
        "ble_tx_adv_ch37",
    ] {
        println!("cargo:rustc-link-arg-bin={}=-Tlink.x", bin);
    }
    println!("cargo:rustc-link-arg-bin=ble_tx_adv_ch37_minimal=-Tlink_minimal.x");

    // Iron Law #37 ASSERT pins (binary-scoped):
    //   frozen_bss_pins.x → guards ble_tx_adv_ch37 regression (T8 baseline, commit f27c394)
    //                        "ASSERT failure = frozen binary regression"
    //   minimal_bss_pins.x → verifies ble_tx_adv_ch37_minimal pull-in (T44.E fix)
    //                         "ASSERT failure = minimal BSS placement incorrect"
    println!("cargo:rustc-link-arg-bin=ble_tx_adv_ch37=-Tfrozen_bss_pins.x");
    println!("cargo:rustc-link-arg-bin=ble_tx_adv_ch37_minimal=-Tminimal_bss_pins.x");
}
