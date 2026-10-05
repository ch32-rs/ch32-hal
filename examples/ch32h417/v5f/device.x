/*
 * qingke-rt's link.x does `INCLUDE device.x`, expecting the hooks an svd2rust
 * device crate would add (interrupt vector aliases, PROVIDE()d symbols).
 *
 * The V5F crate has no HAL and no device crate — it drives metapac directly —
 * so there is nothing to add here. The file exists so the linker script
 * resolves; keep it empty rather than deleting it.
 */
