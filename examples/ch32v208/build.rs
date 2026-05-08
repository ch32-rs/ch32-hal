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
    // fnGetClockCBs @ 0x20001c78 is kept pinned for now (caveat §6 in
    // notes/ch32-rs/phase2-bss-pin-removal-design.md). Phase 2b separately validates
    // its removal. Until 2b lands, link.x continues to PROVIDE(_ebss=0x20001c78) and
    // KEEP the .fnGetClockCBs section.

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

        /* fnGetClockCBs @ 0x20001c78 — HARD ROM ABI ADDRESS (Step 3 hardware-confirmed,
         * 2026-05-09). ROM reads this exact physical address unconditionally regardless
         * of where our symbol is placed. Poisoning 0x20001c78 → cba=0 even when
         * our Rust symbol at 0x200007c0 held the correct 0x420B000A value.
         * The pin is a real hardware requirement, not a historical artifact.
         * ble_ip_core_init writes 0x420B000A here explicitly (= TMOS_TimerInit(0)
         * equivalent per PDF §8.1.1). ROM does NOT write this slot itself (baf71de). */
        . = 0x20001c78;
        PROVIDE( _ebss = .);
        KEEP(*(.fnGetClockCBs));  /* fnGetClockCBs at 0x20001c78, NOT startup-zeroed */
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

    // Single linker script for all binaries (Phase 2a, 2026-05-08).
    // Iron Law #34 v5: no binary needs ROM-contract address pins on the 5 BSS symbols.
    // fnGetClockCBs is still pinned at 0x20001c78 inside link.x (Phase 2b caveat).
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
}
