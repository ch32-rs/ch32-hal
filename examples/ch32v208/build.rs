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
    let out = std::env::var("OUT_DIR").unwrap();
    let link_x = format!("{}/link.x", out);
    std::fs::write(
        &link_x,
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

    .bss (NOLOAD) : ALIGN(4)
    {
        PROVIDE( _sbss = .);
        /* T4.5 #36 Option C: TX_BUF placed at ALIGN(16) boundary via section annotation.
         * Strategic: TX_BUF alignment is independent of GlobalMerge layout and survives
         * T5-T7 BSS shifts. Replaces Option D (`. += 8` tactical gap). */
        . = ALIGN(16);
        *(.tx_buf_aligned .tx_buf_aligned.*);
        . = ALIGN(4);
        *(.sbss .sbss.* .bss .bss.*);
        /* T15 (2026-05-06): _ebss = 0x20001c78, matching bisect-3g layout EXACTLY.
         * fnGetClockCBs placed AT _ebss boundary — BSS-zero loop (_sbss.._ebss exclusive)
         * does NOT zero fnGetClockCBs. ROM writes 0x420B000A (valid fn ptr) to this address
         * during BLE init; retaining that value enables correct BLE clock Hz.
         * Rust strong symbol provides linker definition (no -lwchble needed).
         *
         * Layout: _ebss = 0x20001c78; fnGetClockCBs at 0x20001c78 (NOT startup-zeroed)
         * Expected: binary byte-identical to bisect-3g (_ebss immediate 0x1c78 = 0x45 restored).
         *
         * gBleIPPara @ 0x20000758 ✓, ble @ 0x20001858 ✓ */
        . = 0x20001c78;
        PROVIDE( _ebss = .);
        KEEP(*(.fnGetClockCBs));  /* fnGetClockCBs at 0x20001c78, NOT startup-zeroed (ROM manages) */
    } >RAM

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

/* Iron Law #37 (D-pre, 2026-05-06): explicit address pins for ROM-expected BSS layout.
 * Addresses confirmed from T8/f27c394 attempt-15 (byte-identical to bisect-3g).
 * Link fails with a clear message if any symbol drifts — prevents silent GlobalMerge
 * regressions like the Iron Law #34 violation caught in task #43 Phase C (Cindy 2026-05-06).
 *
 * dtmFlag    @ 0x20000750 — 1B DTM mode flag (init-only, not in ISR hot path)
 * gPaControl @ 0x20000754 — 4B PA control (CH32V208 integrated PA; init-only)
 * gBleIPPara @ 0x20000758 — 40B BLE IP parameter block (ISR hot path; MUST NOT drift)
 *
 * If any ASSERT fires: check LLVM GlobalMerge BSS clustering for the example binary.
 * See notes/ch32-rs/lib-dependency-removal.md §Iron Laws for remediation steps.
 */
ASSERT(dtmFlag    == 0x20000750, "Iron Law #37: dtmFlag must be at 0x20000750 (BSS drift)")
ASSERT(gPaControl == 0x20000754, "Iron Law #37: gPaControl must be at 0x20000754 (BSS drift)")
ASSERT(gBleIPPara == 0x20000758, "Iron Law #37: gBleIPPara must be at 0x20000758 (BSS drift)")
"#,
    )
    .unwrap();
    // Add OUT_DIR to linker search path so `link.x` is found before qingke-rt's version.
    println!("cargo:rustc-link-search=native={}", out);
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
}
