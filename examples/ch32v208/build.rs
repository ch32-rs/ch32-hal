fn main() {
    // println!("cargo:rustc-link-arg-bins=--nmagic");
    // println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // Path C: link the WCH BLE library for BLE_IPCoreInit and the BB IRQ sub-handler.
    println!("cargo:rustc-link-search=/Users/mono/Elec/WCH/CH32V20xEVT-2.31/EXAM/BLE/LIB");
    println!("cargo:rustc-link-lib=static=wchble");
    println!("cargo:rustc-link-arg=--undefined=BB_IRQLibHandler");

    // T3 Option D (2026-05-04): TX_BUF must be 16B-aligned (hardware DMA requirement).
    //
    // Root cause: T3 adds gPaControl(4B)+dtmFlag(1B)+3B align = +8B before MergedGlobals.180.
    // In T2 baseline TX_BUF was at MergedGlobals+0x98 = mod16=0 (cba=57 ✓).
    // In T3PA (1fb675f) MergedGlobals shifted +8B → TX_BUF at mod16=8 (cba=0 ✗).
    //
    // Fix: inject `. += 8` at the top of the .bss section in a custom link.x.
    // This shifts ALL BSS content (TRACE_BUF, MergedGlobals.180, …) by +8B uniformly.
    // TX_BUF stays at offset 0x98 inside MergedGlobals; base shifts +8B → mod16=0. ✓
    // No Rust type change → LLVM codegen identical to T3PA → ISR=262B, BIN=51588B. ✓
    //
    // Zero-init: entire .bss (including the 8B gap) is zeroed by startup code. ✓
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
        /* T3 Option D: +8B gap shifts MergedGlobals.180 base by +8B so TX_BUF
         * (offset 0x98 in MergedGlobals) moves from mod16=8 to mod16=0.
         * Startup code zeros _sbss.._ebss, covering the gap. */
        . += 8;
        *(.sbss .sbss.* .bss .bss.*);
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
    // Add OUT_DIR to linker search path so `link.x` is found before qingke-rt's version.
    println!("cargo:rustc-link-search=native={}", out);
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
}
