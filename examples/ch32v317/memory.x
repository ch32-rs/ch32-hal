/*
 * This memory layout file will make the examples working
 * with an out-of-the-box ROM/RAM Split Configuration
 *
 * The current configuration can be checked with
 *   $wchisp config
 *   [INFO] Opening USB device #0
 *   [INFO] Current config registers: a55aff0000ff00ffffffffff00020300feed31e566e495b7
 *   RDPR_USER: 0x00FF5AA5
 *     [7:0]   RDPR 0xA5 (0b10100101)
 *       `- Unprotected
 *     [16:16] IWDG_SW 0x1 (0b1)
 *       `- IWDG enabled by the software, and disabled by hardware
 *     [17:17] STOP_RST 0x1 (0b1)
 *       `- Disable
 *     [18:18] STANDBY_RST 0x1 (0b1)
 *       `- Disable, entering standby-mode without RST
 *     [23:22] SRAM_CODE_MODE 0x3 (0b11)
 *       `- CODE-228KB + RAM-32KB
 *   DATA: 0xFF00FF00
 *     [7:0]   DATA0 0x0 (0b0)
 *     [23:16] DATA1 0x0 (0b0)
 *   WRP: 0xFFFFFFFF
 *     `- Unprotected
 */

MEMORY
{
    CODE   (rx)  : ORIGIN = 0x00000000, LENGTH =  228K /* USR boot alias */
    USR    (rwx) : ORIGIN = 0x08000000, LENGTH =  480K
    SYS    (rwx) : ORIGIN = 0x1FFF8000, LENGTH =   28K
    VND    (r)   : ORIGIN = 0x1FFFF700, LENGTH =   256
    OPT    (rw)  : ORIGIN = 0x1FFFF800, LENGTH =   128
    RAM    (rwx) : ORIGIN = 0x20000000, LENGTH =   32K
}

REGION_ALIAS("FLASH", CODE);

REGION_ALIAS("REGION_TEXT", CODE);
REGION_ALIAS("REGION_RODATA", CODE);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);
