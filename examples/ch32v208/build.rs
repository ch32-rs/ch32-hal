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
    // Iron Law #34 (v5 final, 2026-05-08): ROM is RAM-layout-agnostic for the 6
    // BSS-contract symbols. ROM uses its private 0x20003000-0x200036FF workspace.
    //
    // Phase 2c (2026-05-09): fnGetClockCBs address pin removed.
    // C ground truth (Lucy, 2026-05-09): fnGetClockCBs is a COMMON symbol with
    // PCREL relocations in libwchble.a — linker resolves its address freely.
    // WCH EVT Broadcaster nm shows fnGetClockCBs at 0x20002420 (not 0x20001c78)
    // and shifting 1:1 with MEM_BUF size. 0x20001c78 was a BSS layout coincidence
    // from a specific historical binary, not a ROM/hardware hardcoded address.
    //
    // Step 3 FAIL (0665bc1) was caused by removing the pin WITHOUT keeping the
    // explicit write — symbol moved to new address but slot was NULL → cba=0.
    // Root cause was the missing write, not a hardcoded physical address in ROM.
    //
    // Production mechanism (Step 1, 52ed8dc): ble_ip_core_init writes 0x420B_000A
    // to fnGetClockCBs via addr_of_mut!(fnGetClockCBs) — symbol-resolved, works at
    // any linker-assigned address. fnGetClockCBs is startup-zeroed (normal BSS),
    // then explicitly written before any ROM BLE call.

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
        /* fnGetClockCBs: COMMON/PCREL symbol, linker-placed at natural BSS address.
         * Must be listed here explicitly to stay in NOLOAD (orphan sections become
         * loadable, causing 536MB objcopy — see 78f0493 post-mortem).
         * ble_ip_core_init writes 0x420B_000A via addr_of_mut!(fnGetClockCBs) after
         * startup zeroing. No address pin required (Phase 2c, 2026-05-09). */
        *(.fnGetClockCBs .fnGetClockCBs.*);

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

    // Single linker script for all binaries (Phase 2a, 2026-05-08).
    // Phase 2c (2026-05-09): fnGetClockCBs address pin removed. All 6 BSS-contract
    // symbols now linker-placed without address constraints.
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
}
