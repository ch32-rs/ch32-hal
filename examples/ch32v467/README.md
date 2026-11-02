# CH32V467 HAL bring-up

The HAL and examples share Git PAC revision `e150d53`, generated from ch32-data
`844640b` (#78). qingke/qingke-rt use crates.io version 0.8.3.

Default chip: CH32V467WEU6. Board LED outputs: PE8/PE9.

From this directory:

```sh
cargo build --release --bin hal_init --features hal
cargo build --release --bin hal_embassy --features embassy
```

Available HAL examples in this commit:

- `hal_init`
- `hal_blinky`
- `hal_exti`
- `hal_embassy`
- `hal_systick_stress`
- `hal_rcc`
- `hal_time_scale`
- `hal_dma`
- `hal_usart`
- `hal_timer`
- `hal_i2c`
- `hal_spi`

Driver builds and host tests do not establish hardware acceptance. GPIO input
requires external wiring. Natural SysTick rollover and peripheral board tests
remain pending as recorded in the migration plan.
