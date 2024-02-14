# ch32-hal
WIP hal crate for ch32[vxl]\d{3} MCUs

This hal crates supports [embassy](https://github.com/embassy-rs/embassy) for CH32V[vxl]\d{3} series of microcontrollers.

Currently, supports only CH32V307, others should work if you are careful as most peripherals are similar enough.
Most will later be split off into a common peripheral crate.

## TODOs

- [x] Embassy timer driver using Systick
- [x] SDI Debug
- [x] GPIO
    - [x] Interrupts (async)
- [x] UART
    - [x] Half-duplex
    - [ ] UART async (DMA)
- [x] SPI
    - [x] blocking API
    - [ ] async API
- [ ] OPA
- [ ] I2C
- [ ] USB
- More to come
