# ch32-hal

WIP hal crate for WCH's 32-bit RISC-V microcontrollers.

This hal crates is the [embassy](https://github.com/embassy-rs/embassy) driver for WCH's 32-bit RISC-V microcontrollers.

This hal crates uses the metapac approach to support multiple chips in the same crate.
The metapac is maintained in the [ch32-rs/ch32-data](https://github.com/ch32-rs/ch32-data) repository, published as a crate `ch32-metapac`.

Currently, supported chips are listed in `Cargo.toml` as feature flags,
others should work if you are careful as most peripherals are similar enough.

Keypoints:

- embassy support
- metapac approach, check [ch32-data](https://github.com/ch32-rs/ch32-data)
- all in one crate, no need to create a new crate for each chip
- async drivers

## Supported Devices and Peripherals

| Family | Status | Embassy | RCC | GPIO | UART | SPI | I2C | ADC | Timer(PWM) | EXTI* | RNG |
|--------|--------|---------|-----|------|------|-----|-----|-----|------------|-------|-----|
| V2/V3  |        | ✅      | ✅  | ✅   | ✅   | ✅  | ✅  | ✅  |            | ✅    | ✅  |
| V1     |        | ✅      | ✅  | ✅   | ✅   | ✅  | ✅  | ✅  |            | ✅    |     |
| V0     |        | ✅      | ✅  | ✅   | ✅   | ✅  | ✅  | ❓  |            | ✅    |     |
| X0     |        |         | ✅  | ✅   | ✅   | ✅  |     | ✅  | ✅         | ✅    |     |
| L0     |        |         |     |      |      |     |     |     |            |       |     |
| CH641  | N/A    |         |     |      |      |     |     |     |            |       |     |
| CH643  | N/A    |         |     |      |      |     |     |     |            |       |     |
| CH645  | N/A    |         |     |      |      |     |     |     |            |       |     |

- ✅ : Works
- ❓ : Not tested
- `*` marks the async driver

## Minimum supported Rust version(MSRV)

This project is developed with a recent **nightly** version of Rust compiler. And is expected to work with beta versions of Rust.

Feel free to change this if you did some testing with some version of Rust.

## Contributing

TODO. All kinds of contributions are welcome.

## License

This project is licensed under the MIT or Apache-2.0 license, at your option.
