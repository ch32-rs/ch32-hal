/*
 * Places the cross-core mailbox (the single `#[link_section = ".shared"]` item in
 * `ipc/src/lib.rs`) at the start of SRAM_SHARED, the region both harts declare at
 * the same address in their `memory.x`.
 *
 * qingke-rt's link.x puts `.uninit` in `RAM`, i.e. the core's private memory, so
 * a shared variable needs its own output section. NOLOAD: the section costs no
 * flash and nothing zeroes it at startup, which is what a mailbox wants (it must
 * survive a reset of one core while the other keeps running).
 *
 * Only one such item may exist per image — the section is laid out in
 * declaration order, so a second one would shift the offsets on one side.
 */
SECTIONS
{
    .shared (NOLOAD) : ALIGN(4)
    {
        KEEP(*(.shared .shared.*));
    } >SRAM_SHARED
}
