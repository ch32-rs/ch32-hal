/*
 * V3F link script: run the boot core out of the zero-wait shared SRAM.
 *
 * A copy of `qingke-rt`'s `link-highcode.x` with one change: `.text` and
 * `.rodata` are folded into `.highcode`, so the whole program — smoltcp and
 * every other dependency included — is copied from flash to RAM at startup
 * and executed there. Hart 0 has no instruction cache and Code FLASH is
 * non-zero-wait (RM §1.2.1), so code fetched from flash runs at ~2.6 MIPS
 * against ~1 IPC from the shared block; see `memory.x`.
 *
 * What stays in flash is only what runs before the copy exists: `.init` (the
 * boot alias jump) and `.handle_reset` with its copy loops. `.highcode`
 * starts with the interrupt vector tables, because `qingke-rt` points
 * `mtvec` at `_highcode_vma_start`.
 *
 * The script ships from this crate's build script, whose link search path
 * takes precedence over `qingke-rt`'s, so the `highcode` feature's own
 * (`.text` in flash) layout is replaced by this one.
 */
INCLUDE memory.x
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

/* PROVIDE(__EXTERNAL_INTERRUPTS = __DEFAULT_EXTERNAL_INTERRUPTS);*/

/* # Interrupt vectors */
EXTERN(__CORE_INTERRUPTS);
EXTERN(__EXTERNAL_INTERRUPTS); /* `static` variable similar to `__EXCEPTIONS` */

ENTRY(_start)

SECTIONS
{
    /* init jump opcode */
    .init :
    {
        . = ALIGN(4);
        KEEP(*(SORT_NONE(.init)))
        . = ALIGN(4);
    } >FLASH AT>FLASH

    /* Copied to RAM by the startup code; executed from there. */
    .highcode : ALIGN(4)
    {
        _highcode_lma = LOADADDR(.highcode);
        PROVIDE(_highcode_vma_start = .);
        LONG(_start); /* Placeholder for the first vector */
        KEEP(*(.vector_table.core_interrupts));
        KEEP(*(.vector_table.external_interrupts));
        KEEP(*(.vector_table.exceptions));
        *(.trap .trap.rust)
        *(.highcode);
        *(.highcode.*);
        *(.text .text.*)
        *(.gnu.linkonce.t.*)
        *(.rodata .rodata.*);
        *(.srodata .srodata.*);
        . = ALIGN(4);
        PROVIDE(_highcode_vma_end = .);
    } >RAM AT>FLASH

    /* Runs from flash, before `.highcode` has been copied. */
    .text : ALIGN(4)
    {
        . = ALIGN(4);
        KEEP(*(SORT_NONE(.handle_reset)))
        *(.init.rust)
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
        *(.sbss .sbss.* .bss .bss.*);
        PROVIDE( _ebss = .);
    } >RAM

    .uninit (NOLOAD) : ALIGN(4)
    {
        *(.uninit .uninit.*);
    } >RAM

    /* Align `__sheap` and `_stack_end` pointers to 4 bytes */
    . = ALIGN(4);

    /* Place the heap start and stack end at the end of allocated RAM */
    PROVIDE(__sheap = .);
    PROVIDE(_stack_end = .);

    .got (INFO) :
    {
        KEEP(*(.got .got.*));
    }

    .eh_frame (INFO) : { KEEP(*(.eh_frame)) }
    .eh_frame_hdr (INFO) : { *(.eh_frame_hdr) }
}

INCLUDE assert-align.x
