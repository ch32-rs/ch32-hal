# ch32-hal

[![Demo Code Github Actions][badge-actions]][actions-build]

[badge-actions]: https://img.shields.io/github/actions/workflow/status/ch32-rs/ch32-hal/build.yml?style=for-the-badge&label=Demo%20Code%20Build
[actions-build]: https://github.com/ch32-rs/ch32-hal/actions/workflows/build.yml

Rust HAL(Hardware Abstraction Layer) crate for WCH's 32-bit RISC-V microcontrollers.

> **Note**
> This project is under development. While it's usable for experimentation and testing,
> it may not be fully stable for production environments.
> We welcome user feedback and encourage reporting any issues you encounter to help improve the hal crate.

This HAL crates is the [Embassy](https://github.com/embassy-rs/embassy) framework driver for WCH's 32-bit RISC-V microcontrollers.

This HAL crates uses the metapac approach to support multiple chips in the same crate.
The metapac is maintained in the [ch32-rs/ch32-data](https://github.com/ch32-rs/ch32-data) repository, published as a crate `ch32-metapac`.

Keypoints:

- Embassy support
- All-in-one metapac for peripheral register access, check [ch32-data](https://github.com/ch32-rs/ch32-data) for more
- All-in-one HAL crate, no need to create a new crate for each chip
- Async drivers, with async/await support, DMA support
- Write once, run on all supported chips(should be)

## Supported Devices and Peripherals

Currently, supported chips are listed in `Cargo.toml` as feature flags,
others should work if you are careful as most peripherals are similar enough.

For a full list of chip capabilities and peripherals, check the [ch32-data](https://github.com/ch32-rs/ch32-data) repository.

| Family     | V2/V3 | V1  | V003 | V00X | X0   | L1   | CH641 | CH643 | H4‡   |
| ---------- | ----- | --- | ---- | ---- | ---- | ---- | ----- | ----- | ---- |
| Embassy    | ✅    | ✅  | ✅   | ✅   | ✅   | ✅   | ✅    | ✅    | ✅    |
| RCC        | ✅    | ✅  | ✅   | ✅   | ✅   | ✅   | ✅    | ✅    | ✅    |
| GPIO       | ✅    | ✅  | ✅   | ✅   | ✅   | ✅   | ✅    | ✅    | ✅    |
| EXTI\*     | ✅    | ✅  | ✅   | ✅   | ✅   | ✅   | ✅    | ✅    | ✅    |
| UART\*     | ✅    | ✅  | ✅   | ❓   | ✅   | ✅   | ❓    | ❓    | ✅    |
| SPI\*      | ✅    | ✅  | ✅   | ❓   | ✅   | ✅   | N/A   | ❓    | ✅\*  |
| I2C        | ✅    | ✅  | ✅   | ❓   | ❓   | ❓   | ❓    | ❓    | ✅    |
| ADC        | ✅    | ✅  | ✅   | ✅   | ✅   | ✅   | ✅    | ✅    | ✅    |
| Timer(PWM) | ✅    | ✅  | ✅   | ❓   | ✅   | ✅   | ✅    | ✅    | ✅    |
| IWDG       | ✅    | N/A | N/A  | ❓   | N/A  | N/A  | N/A   | N/A   | ❓    |
| WWDG       | ✅    | N/A | N/A  | ❓   | N/A  | N/A  | N/A   | N/A   | ❓    |
| CRC        | ✅    | N/A | N/A  | N/A  | N/A  | N/A  | N/A   | N/A   | ❓    |
| USBD       | ✅\*  | N/A | N/A  | N/A  | N/A  | N/A  | N/A   | N/A   | N/A  |
| USB/OTG FS | ✅\*  | N/A | N/A  | N/A  | ✅\* | ✅\* | N/A   | N/A   | ✅\*  |
| USB HS     | ✅\*  | N/A | N/A  | N/A  | N/A  | N/A  | N/A   | N/A   | ✅\*  |
| USB PD     | N/A   | N/A | N/A  | N/A  | ✅\* | ✅\* | ❓    | ❓    | ❌    |
| ETH\*      | ✅†   | N/A | N/A  | N/A  | N/A  | N/A  | N/A   | N/A   | ✅†   |
| CAN\*      | ✅    | N/A | N/A  | N/A  | N/A  | ✅   | N/A   | N/A   | ✅    |

- ✅ : Expected to work
- ❌ : Not implemented
- ❓ : Not tested
- `*` marks the async driver (EXTI is also opt-in via the `exti` Cargo feature)
- `†` ETH: the CH32V208 10M MAC+PHY and the H4 internal 10/100 PHY are
  validated; the V305/V307 Synopsys DWC MAC is in progress (PR #157), and the
  H4 RGMII/Gigabit path is not board-tested.
- `‡` H4 is the dual-core CH32H417 (QingKe V3F + V5F). USB FS is the OTG-v2
  device block (`usbfs_h4`, reusing the `otg_fs` driver); USB HS (`usbhs_h4`)
  shares the V30x/V4x7 driver and is validated at High and Full Speed. USB 3.0
  (`usbss`) has an experimental link-layer framework only, not a validated
  device stack. USB PD is compiled out because the H4 register layout
  differs. I2C is flagged `version: v3` in the H4 metadata, so the V3 driver
  applies as-is. DMA is not in the table; the H4 DMAMUX is driven from
  request numbers in metadata. The H4 Embassy time driver is
  `time_driver_systick_h4`, so no `time-driver-timN` feature is needed.
- N/A: Not available

**V00X** covers CH32V002 / CH32V004 / CH32V005 / CH32V006 / CH32V007 and CH32M007.

I2C also has slave support (blocking, interrupt and DMA); see
[`examples/ch32l103/src/bin/i2c_slave.rs`](examples/ch32l103/src/bin/i2c_slave.rs),
validated on the CH32L103 against a CH32V203 master.

Dual-core examples for the CH32H417 (separate V3F and V5F images, a shared
mailbox, `xtask` for flash/report) live in [`examples/ch32h417`](examples/ch32h417).

### Notes

- For USB OTGFS and HS, look at the respective `mod.rs` to understand what is / is not tested.

### Important: ROM/RAM Split Configuration (CH32V2/V3)

> **Warning**
> All CH32V2 and CH32V3 series chips support configurable ROM/RAM split. The configuration is stored in
> `FLASH_OBR.RAM_CODE_MOD` register (read-only at runtime, can only be modified via external tools).
>
> If your program crashes immediately after flashing (e.g., Store Access Fault at stack addresses),
> your chip's ROM/RAM configuration likely doesn't match the linker script.

**Symptoms:**

- Program crashes with `mcause=0x7` (Store/AMO access fault)
- Stack pointer (`sp`) points to invalid memory region
- No output or immediate crash after reset

**Solution:**
Use WCH's official tool (WCHISPTool) to configure the ROM/RAM split to match the default values defined in [ch32-data](https://github.com/ch32-rs/ch32-data).

The linker script expects the **default RAM size** for each chip. Check your chip's YAML definition in ch32-data for the expected memory layout.

### TODOs

This section lists some key items that are not implemented yet. And should be noted when using this crate.

- PLL2 for CH32V3
- DMA2 for CH32V3 (requires special handling of high DMA channels)

### Coming New Chips - Help Wanted

- CH645, USB HUB, SerDes (V4C)
- CH564, USBHS, 100M Ethernet (V4J)

## Built with ch32-hal ✨

This is a list for awesome projects that are built using ch32-hal

- [Hackoween 2024 badge](https://github.com/rappet/hackoween-badge)

## Minimum supported Rust version(MSRV)

This project is developed with a recent **nightly** version of Rust compiler. And is expected to work with beta versions of Rust.

Feel free to change this if you did some testing with some version of Rust.

## Contributing

All kinds of contributions are welcome.

- Share your project at [Discussions](https://github.com/ch32-rs/ch32-hal/discussions)
  - if your project is an open-source project, consider adding it to the list above
- README and Documentation, including doc comments in code
- Writing demo code for peripherals
- Revising the peripheral definitions at [ch32-data](https://github.com/ch32-rs/ch32-data)
- Adding new peripheral drivers - This is difficult to make it compatible with all chips, but you can try.
- ...

## License

This project is licensed under the MIT or Apache-2.0 license, at your option.
