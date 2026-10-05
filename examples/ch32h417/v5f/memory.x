/*
 * CH32H417QEU6 (QingKe V5F / second core) linker memory map.
 *
 * Mirrors the WCH CSDK's dual-core partition
 * (`CH32H417EVT/EXAM/.../Common/Ld/V5F/Link_v5f.ld`), so the two images tile
 * flash instead of overlapping:
 *
 *   V3F : flash 0x00000000 + 64K,  RAM = ITCM
 *   V5F : flash 0x00010000 + ...,  RAM = DTCM
 *   both: RAM_SHARED 0x20178000 + 32K — cross-core mailbox, the same
 *         address in both linker maps
 *
 * `FLASH` ORIGIN is also the wake entry: the boot core passes this address
 * to `qingke::pfic::wake_other_core()`. The CSDK does the same through its
 * per-project `Core_V5F_StartAddr` macro, which always equals this ORIGIN.
 *
 * The first 512 bytes of DTCM are left unused because the QingKe V5
 * hardware push/pop (interrupt hardware stack) lives there; the CSDK
 * scripts reserve the same area. `wlink` maps this image's 0x00010000-based
 * sections onto flash 0x08010000.
 */
MEMORY
{
    FLASH : ORIGIN = 0x00010000, LENGTH = 960K - 64K
    RAM   : ORIGIN = 0x200C0200, LENGTH = 256K - 512 /* DTCM after the HW push/pop area */
    SRAM_SHARED (rwx) : ORIGIN = 0x20178000, LENGTH = 32K /* cross-core mailbox */
}

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);
