fn main() {
    // T8 (2026-05-06): -lwchble removed. All lib COMMON BSS migrated to Rust strong
    // symbols (#34 gBleIPPara, #35 gBleLlPara, T8 fnGetClockCBs). All anchored
    // externs were already GC'd by --gc-sections; their decl block also removed.
    //
    // T4.5 #36 Option C (2026-05-05): TX_BUF strategic 16B alignment.
    // TX_BUF placed in dedicated .tx_buf_aligned section at ALIGN(16). Independent of
    // GlobalMerge layout. DMA-alignment requirement, NOT a BSS pin.
    //
    // Phase 2a (2026-05-08): single-script BSS architecture.
    //
    // Iron Law #34 (v5 final, 2026-05-08 — ROM hex disassembly + Cindy 06:28 RAM dump
    // + frozen drift cba=72/61 PASS): ROM is RAM-layout-agnostic for the 6 BSS-contract
    // symbols. ROM uses its private 0x20003000-0x200036FF workspace; host BSS placement
    // is host-only concern. Therefore link_minimal.x (KEEP+pin block) and the binary-
    // specific ASSERT scripts (frozen_bss_pins.x, minimal_bss_pins.x) are dropped.
    //
    // The minimal binary's symbols are retained against --gc-sections by `#[no_mangle]`
    // + `#[used]` on the 6 BSS-contract statics — these attributes are independent of
    // section pinning and continue to apply.
    //
    // fnGetClockCBs @ 0x20001c78 (2026-05-08, task #56): the slot is no longer
    // a runtime fn-ptr indirection (bb_irq_lib_handler now calls fallback_clock
    // directly). The 4-byte placeholder is preserved at 0x20001c78 INSIDE the
    // [_sbss, _ebss) zero-init range so that startup BSS-clear keeps the byte
    // at 0 every reset (cold and warm). If the chip silicon ROM hardcodes a
    // read at 0x20001c78, NULL triggers its auto-install fallback path —
    // deterministic regardless of cold/warm SRAM state. Compare to the prior
    // "boundary trick" (Phase 2a) where _ebss=0x20001c78 left the slot OUTSIDE
    // zero-init range, depending on warm-boot SRAM retention.

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
         * DMA-alignment requirement, NOT a BSS pin. */
        . = ALIGN(16);
        *(.tx_buf_aligned .tx_buf_aligned.*);
        . = ALIGN(4);

        /* Natural LLVM BSS ordering — no explicit address pins (Iron Law #34 v5 final:
         * ROM is RAM-layout-agnostic for the 6 BSS-contract symbols). */
        *(.sbss .sbss.* .bss .bss.*);

        /* fnGetClockCBs @ 0x20001c78 (task #56, 2026-05-08): 4-byte placeholder
         * INSIDE [_sbss, _ebss) so startup BSS-clear zeroes it on every reset.
         * Chip silicon ROM (if it hardcodes a read at 0x20001c78) sees NULL and
         * triggers its auto-install fallback path → cold + warm boot deterministic.
         * The slot is no longer a runtime indirection — bb_irq_lib_handler calls
         * crate::ble::fallback_clock() directly. Invariant: wildcard BSS above
         * must end at or before 0x20001c78. */
        . = 0x20001c78;
        KEEP(*(.fnGetClockCBs));
        . = ALIGN(4);
        PROVIDE( _ebss = .);
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

    // Add OUT_DIR to linker search path so link.x is found before qingke-rt's version.
    println!("cargo:rustc-link-search=native={}", out);

    // Single linker script for all binaries (Phase 2a + task #56, 2026-05-08).
    // Iron Law #34 v5: no binary needs ROM-contract address pins on the 5 BSS symbols.
    // fnGetClockCBs is preserved at 0x20001c78 INSIDE _ebss as a zero-init placeholder
    // (chip ROM auto-install fallback safety); runtime indirection retired (task #56).
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
}
