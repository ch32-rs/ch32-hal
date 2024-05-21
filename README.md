# ch32-hal

[![Demo Code Github Actions][badge-actions]][actions-build]

[badge-actions]: https://img.shields.io/github/actions/workflow/status/ch32-rs/ch32-hal/build.yml?style=for-the-badge&label=Demo%20Code%20Build
[actions-build]: https://github.com/ch32-rs/ch32-hal/actions/workflows/build.yml

Rust HAL(Hardware Abstraction Layer) crate for WCH's 32-bit RISC-V microcontrollers.

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

| Family | Status | Embassy | RCC | GPIO | UART*| SPI*| I2C | ADC | Timer(PWM) | EXTI*| RTC | DMA*| Delay | Others |
|--------|--------|---------|-----|------|------|-----|-----|-----|------------|------|-----|-----|-------| ------ |
| V2/V3  |        | ✅      | ✅  | ✅   | ✅   | ✅  | ✅  | ✅  | ✅         | ✅   |     | ✅  |       | RNG, SDIO |
| V1     |        | ✅      | ✅  | ✅   | ✅   | ✅  | ✅  | ✅  | ✅         | ❓   |     | ❓  | ✅    | |
| V0     |        | ✅      | ✅  | ✅   | ✅   | ✅  | ✅  | ✅  | ✅         | ❓   |     | ❓  | ✅    | |
| X0     |        | ✅      | ✅  | ✅   | ✅   | ✅  | ❓  | ✅  | ✅         | ✅   |     | ✅  |       | |
| L1     |        | ✅      | ✅  | ✅   | ✅   | ✅  | ❓  | ✅  | ✅         | ❓   |     | ❓  |       | |
| CH641  |        | ✅      | ✅  | ✅   | ❓   | N/A | ❓  | ✅  | ✅         | ❓   |     | ❓  | ✅    | ISP |
| CH643  | TODO   |         |     |      |      |     |     |     |            |      |     |     |       | |

- ✅ : Expected to work
- ❓ : Not tested
- `*` marks the async driver
- TODO: I haven't got a dev board yet, help-wanted
- N/A: Not available

### TODOs

This section lists some key items that are not implemented yet. And should be noted when using this crate.

- PLL2 for CH32V3
- DMA2 for CH32V3 (requires special handling of high DMA channels)

### Coming New Chips - Help Wanted

- CH32V002 / CH32V004 / CH32V005 / CH32V006 / CH32V007 / CH32M007, Qingke V2C
- CH645, USB HUB, SerDes (V4C)
- CH564, USBHS, 100M Ethernet (V4J)

## Built with ch32-hal ✨

This is a list for awesome projects that are built using ch32-hal

- ... not yet :(

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
