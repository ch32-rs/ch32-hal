# CH32X035

Button PC3, active low. Pulled down or floating.

## Examples

| File | What it shows |
|---|---|
| `awu` | auto-wakeup timer, blocking and async |
| `blinky` | GPIO output on the user LED |
| `blinky_pwm` | timer PWM on an LED pin |
| `exti` | external interrupt on the button |
| `flash` | reading and writing the option bytes area |
| `i2c_oled` | SSD1306 over I2C |
| `sdi_print` | `println!` over the single-wire debug interface |
| `spi-lcd-st7735` | SPI display |
| `uart_echo` | USART4 echo |
| `uart_pms7003_async` | USART4 receive with DMA |
| `pioc_rainbow` | WS2812 panel through the PIOC, `PC18` |

`pioc_rainbow` needs the HAL's manual `pioc` feature, which the crate already
enables, and a firmware image in the PIOC program SRAM at `0x2000_4000`. Two
consequences the example handles:

- `memory-x` puts the stack on top of that window, so `pioc.x` moves
  `_stack_start` below it. Only that binary gets the fragment.
- `Config::default()` divides HSI by six, so the example asks for
  `SYSCLK_FREQ_48MHZ_HSI`; the image is assembled for a 48 MHz bit period.

`release_swd_pins: true` in the `Config` frees the RVSWD pins for the PIOC.
The debugger cannot attach afterwards, so reset the chip into its system
bootloader to program it again.
