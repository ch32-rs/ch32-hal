#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32_hal::gpio::Output;
use hal::delay::Delay;
use qingke_rt::highcode;

pub const RGB_UNIT_MAX: f32 = 255.0;
pub const HUE_MAX: f32 = 360.0;
pub const PERCENT_MAX: f32 = 100.0;
pub const RATIO_MAX: f32 = 1.0;
pub const ALL_MIN: f32 = 0.0;

const ONE: f32 = 1.0;
const TWO: f32 = 2.0;
const SIX: f32 = 6.0;

const ONE_THIRD: f32 = ONE / 3.0;
const TWO_THIRD: f32 = TWO / 3.0;

pub fn bound(r: f32, entire: f32) -> f32 {
    let mut n = r;
    loop {
        let less = n < ALL_MIN;
        let bigger = n > entire;
        if !less && !bigger {
            break n;
        }
        if less {
            n += entire;
        } else {
            n -= entire;
        }
    }
}

pub fn bound_ratio(r: f32) -> f32 {
    bound(r, RATIO_MAX)
}

fn calc_rgb_unit(unit: f32, temp1: f32, temp2: f32) -> f32 {
    let mut result = temp2;
    if SIX * unit < ONE {
        result = temp2 + (temp1 - temp2) * SIX * unit
    } else if TWO * unit < ONE {
        result = temp1
    } else if 3.0 * unit < TWO {
        result = temp2 + (temp1 - temp2) * (TWO_THIRD - unit) * SIX
    }
    result * RGB_UNIT_MAX
}

/// hsl: [1.0, 1.0, 1.0] -> [255.0, 255.0, 255.0]
pub fn hsl_to_rgb(hsl: [f32; 3]) -> [f32; 3] {
    let [h, s, l]: [f32; 3] = hsl;
    if s == 0.0 {
        let unit = RGB_UNIT_MAX * l;
        return [unit, unit, unit];
    }

    let temp1 = if l < 0.5 { l * (ONE + s) } else { l + s - l * s };

    let temp2 = TWO * l - temp1;
    let hue = h;

    let temp_r = bound_ratio(hue + ONE_THIRD);
    let temp_g = bound_ratio(hue);
    let temp_b = bound_ratio(hue - ONE_THIRD);

    let r = calc_rgb_unit(temp_r, temp1, temp2);
    let g = calc_rgb_unit(temp_g, temp1, temp2);
    let b = calc_rgb_unit(temp_b, temp1, temp2);
    [r, g, b]
}

struct WS2812 {
    pin: Output<'static>,
}

impl WS2812 {
    pub fn new(pin: Output<'static>) -> Self {
        Self { pin }
    }

    pub fn reset(&mut self) {
        self.pin.set_low();
        Delay.delay_us(50);
    }

    // Tune values to get 800kHz, with 33% and 66% duty cycles.
    #[highcode]
    pub fn test(&mut self) -> ! {
        loop {
            self.pin.set_high();
            qingke::riscv::asm::delay(18);
            self.pin.set_low();
            qingke::riscv::asm::delay(7);
        }
    }

    #[highcode]
    pub fn set_color(&mut self, color: u32) {
        for i in (0..24).rev() {
            if color & (1 << i) == 0 {
                self.pin.set_high();
                qingke::riscv::asm::delay(8);
                self.pin.set_low();
                qingke::riscv::asm::delay(16);
            } else {
                self.pin.set_high();
                qingke::riscv::asm::delay(18);
                self.pin.set_low();
                qingke::riscv::asm::delay(7);
            }
        }

        self.pin.set_low();
        qingke::riscv::asm::delay(8 * 50);
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();

    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);

    let ws2812 = Output::new(p.PA2, hal::gpio::Level::Low, Default::default());

    let mut ws2812 = WS2812::new(ws2812);

    ws2812.reset();

    match (cfg!(feature = "ws2812-timing-test"), cfg!(feature = "ws2812-off")) {
        // Timing-test takes precedence when both features are enabled.
        (true, _) => ws2812.test(),
        (false, true) => loop {
            ws2812.set_color(0x000000);
            hal::println!("WS2812 off");
            Delay.delay_ms(10);
        },
        (false, false) => loop {
            for h in 0..360 {
                let rgb = hsl_to_rgb([h as f32 / 360.0, 0.900, 0.500]);
                // to GRB
                let color = ((rgb[1] as u32) << 16) | ((rgb[0] as u32) << 8) | (rgb[2] as u32);

                ws2812.set_color(color);
                Delay.delay_ms(20);
            }
        },
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    hal::println!("\n\n\n{}", info);

    loop {}
}
