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
        PROVIDE( _ebss = .);
    } >RAM

    /* T8 attempt-8 (2026-05-06): Path B — fnGetClockCBs forced to Phase C address.
     * 0x20001c78 = Phase C lib COMMON address (at _ebss boundary, outside startup-zero range).
     * Root cause: the 4B BSS shift from fnGetClockCBs inside _ebss moves gBleIPPara from
     * 0x20000758 (Phase C, ROM-expected?) to 0x2000075c (+4B) → RF failure.
     * By forcing fnGetClockCBs to 0x20001c78 (outside _ebss), gBleIPPara stays at 0x20000758.
     *
     * Layout within this section (8B total):
     *   +0x00: fnGetClockCBs   (4B, .fnGetClockCBs sub-section — placed FIRST = at 0x20001c78)
     *   +0x04: FNGETCLOCKCBS_CALL_COUNT (4B, .bss_compat sub-section — probe counter)
     *
     * WARNING: NOT zeroed by startup BSS-zero (_sbss.._ebss range).
     * Safe because: ADV-only path (gBleIPPara[0]=0x60 bit6=0) → Path C unreachable
     * → fnGetClockCBs is never called from any linked function in the RF gate run.
     * Active probe: Rust sets fnGetClockCBs = &rust_fnGetClockCBs_probe in main() before
     * BLE init. If ROM/lib deref's it, FNGETCLOCKCBS_CALL_COUNT increments.
     * Cold-reset: RAM = 0 (safe NULL default before main() sets probe fn ptr).
     * Warm-reset: main() resets count to 0 and sets probe fn ptr before BLE init. */
    .bss_compat 0x20001c78 (NOLOAD) : {
        KEEP(*(.fnGetClockCBs));            /* fnGetClockCBs MUST be at 0x20001c78 */
        KEEP(*(.bss_compat .bss_compat.*)); /* CALL_COUNT follows at 0x20001c7c */
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
"#,
    )
    .unwrap();
    // Add OUT_DIR to linker search path so `link.x` is found before qingke-rt's version.
    println!("cargo:rustc-link-search=native={}", out);
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
}
