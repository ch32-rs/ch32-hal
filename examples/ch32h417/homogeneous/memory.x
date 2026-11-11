/*
 * CH32H417QEU6 (QingKe V3F boot core) single-image memory map.
 *
 * One program runs on both harts: hart 0 executes `_start` and copies
 * `.highcode` into the zero-wait shared block; hart 1 is started from the same
 * image by `ch32_hal::multicore` and runs the same `.text`. Because there is no
 * second image, the whole 960 KiB of flash belongs to this image — unlike
 * `v3f/memory.x`, which stops at 64 KiB so the V5F image can sit at
 * 0x0001_0000.
 *
 *   CODE  0x0000_0000 + 960 KiB   load image (also reachable via 0x0800_0000)
 *   RAM   0x2010_0000 + 448 KiB   zero-wait code/data/bss for both harts
 *   STACK 0x2017_0000 +  32 KiB   hart 0 stack
 *   SRAM_SHARED 0x2017_8000 + 32 KiB  cross-core mailbox / observation block
 *
 * Hart 1's stack is a `static` in `.bss` (inside RAM). ITCM/DTCM are not used:
 * a single `.text` can only live in one place, and the shared block is the only
 * region that is zero-wait for both cores.
 */
MEMORY
{
    CODE        (rx)  : ORIGIN = 0x00000000, LENGTH = 960K
    USR         (rwx) : ORIGIN = 0x08000000, LENGTH = 960K
    SYS         (rwx) : ORIGIN = 0x1FFF0000, LENGTH =   56K
    VND         (r)   : ORIGIN = 0x1FFFF700, LENGTH =    256
    OPT         (rw)  : ORIGIN = 0x1FFFF800, LENGTH =    256
    RAM         (rwx) : ORIGIN = 0x20100000, LENGTH = 448K
    STACK       (rwx) : ORIGIN = 0x20170000, LENGTH =  32K
    SRAM_SHARED (rwx) : ORIGIN = 0x20178000, LENGTH =  32K
}

REGION_ALIAS("FLASH", CODE);
REGION_ALIAS("REGION_TEXT", CODE);
REGION_ALIAS("REGION_RODATA", CODE);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", STACK);
