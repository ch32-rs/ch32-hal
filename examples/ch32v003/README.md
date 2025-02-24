# ch32v003-examples

## Boards

### nanoCH32V003

Link: [wuxx/nanoCH32V003](https://github.com/wuxx/nanoCH32V003)

- MCU: CH32V003F4U6
- LED: PD6

### LineDog CH32V003J4M6 Board

Link: [OSHWHUB](https://oshwhub.com/andelf/linedog)

- MCU: CH32V003J4M6
- LED: PA2, PC4
- No HSE crystal

### LineDog v2 CH32V003F4P6 Board


- BUZZ: PC4 (TIM1CH4)
- WS2812: PA2

## Running Examples

### using rust-nightly for building

call `rustup install nightly` and `rustup override set nightly` to build this project with rust nightly,
which allows to compilation for riscv32ec using the [json-file](riscv32ec-unknown-none-elf.json) provided.

You might need to install the sources for core with `rustup component add rust-src`

### Using [wlink](https://github.com/ch32-rs/wlink)
- Install wlink using the installation instructions: https://github.com/ch32-rs/wlink?tab=readme-ov-file#install

- Edit the [`.cargo/config.toml`](.cargo/config.toml) file so the runner is `wlink`. This may already be the default runner.

- Build and run the [blinky](src/bin/blinky.rs) example with `cargo run --release --bin blinky`.

### Using [probe-rs](https://probe.rs/)

- Install probe-rs using the installation instructions: https://probe.rs/docs/getting-started/installation/
    - If you are on a Linux based system, you may have to add udev rules to allow probe-rs access to the WCH-Link debugger. https://probe.rs/docs/getting-started/probe-setup/#linux%3A-udev-rules

- Edit the [`.cargo/config.toml`](.cargo/config.toml) file so the runner is `probe-rs run --chip ch32v003`.

- Build and run the [blinky](src/bin/blinky.rs) example with `cargo run --release --bin blinky`.
