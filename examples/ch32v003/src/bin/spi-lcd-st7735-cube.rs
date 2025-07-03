//! TFT LCD ST7735, 160x80
//!
//! Animated 3D cube

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use embedded_graphics::pixelcolor::raw::ToBytes;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle};
use hal::gpio::{Level, Output, Pin};
use hal::prelude::*;
use hal::spi::Spi;
use hal::{peripherals, println};
use micromath::F32Ext;
use qingke::riscv;

#[derive(Debug, Clone, Copy)]
pub enum Instruction {
    NOP = 0x00,
    SWRESET = 0x01,
    RDDID = 0x04,
    RDDST = 0x09,
    SLPIN = 0x10,
    SLPOUT = 0x11,
    PTLON = 0x12,
    NORON = 0x13,
    INVOFF = 0x20,
    INVON = 0x21,
    DISPOFF = 0x28,
    DISPON = 0x29,
    CASET = 0x2A,
    RASET = 0x2B,
    RAMWR = 0x2C,
    RAMRD = 0x2E,
    PTLAR = 0x30,
    COLMOD = 0x3A,
    MADCTL = 0x36,
    FRMCTR1 = 0xB1,
    FRMCTR2 = 0xB2,
    FRMCTR3 = 0xB3,
    INVCTR = 0xB4,
    DISSET5 = 0xB6,
    PWCTR1 = 0xC0,
    PWCTR2 = 0xC1,
    PWCTR3 = 0xC2,
    PWCTR4 = 0xC3,
    PWCTR5 = 0xC4,
    VMCTR1 = 0xC5,
    RDID1 = 0xDA,
    RDID2 = 0xDB,
    RDID3 = 0xDC,
    RDID4 = 0xDD,
    PWCTR6 = 0xFC,
    GMCTRP1 = 0xE0,
    GMCTRN1 = 0xE1,
}

pub struct ST7735<const WIDTH: u16, const HEIGHT: u16, const OFFSETX: u16, const OFFSETY: u16> {
    spi: Spi<'static, peripherals::SPI1, hal::mode::Blocking>,
    dc: Output<'static>,
    // _marker: core::marker::PhantomData<(OFFSETX, OFFSETY)>,
}

impl<const WIDTH: u16, const HEIGHT: u16, const OFFSETX: u16, const OFFSETY: u16>
    ST7735<WIDTH, HEIGHT, OFFSETX, OFFSETY>
{
    pub fn new(spi: Spi<'static, peripherals::SPI1, hal::mode::Blocking>, dc: Output<'static>) -> Self {
        Self {
            spi,
            dc,
            //   _marker: core::marker::PhantomData,
        }
    }

    pub fn init(&mut self) {
        self.send_command(Instruction::SWRESET);
        //        Delay.delay_ms(20);
        self.send_command(Instruction::SLPOUT);
        //      Delay.delay_ms(200);

        self.send_command_data(Instruction::FRMCTR1, &[0x01, 0x2C, 0x2D]);
        self.send_command_data(Instruction::FRMCTR2, &[0x01, 0x2C, 0x2D]);
        self.send_command_data(Instruction::FRMCTR3, &[0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D]);
        self.send_command_data(Instruction::INVCTR, &[0x07]);
        self.send_command_data(Instruction::PWCTR1, &[0xA2, 0x02, 0x84]);
        self.send_command_data(Instruction::PWCTR2, &[0xC5]);
        self.send_command_data(Instruction::PWCTR3, &[0x0A, 0x00]);
        self.send_command_data(Instruction::PWCTR4, &[0x8A, 0x2A]);
        self.send_command_data(Instruction::PWCTR5, &[0x8A, 0xEE]);
        self.send_command_data(Instruction::VMCTR1, &[0x0E]);
        self.send_command_data(Instruction::INVON, &[]);

        // BITS:
        // MY, MX, MV, ML,
        // RGB, MV, _, _
        self.send_command_data(Instruction::MADCTL, &[0b0110_1000]);

        self.send_command_data(Instruction::COLMOD, &[0x05]); // 16-bit/pixel
        self.send_command_data(Instruction::DISPON, &[]);

        // Delay.delay_ms(200);
    }

    #[inline]
    fn set_update_window(&mut self, x: u16, y: u16, w: u16, h: u16) {
        let ox = OFFSETX + x;
        let oy = OFFSETY + y;

        self.send_command_data(
            Instruction::CASET,
            &[
                (ox >> 8) as u8,
                (ox & 0xFF) as u8,
                ((ox + w - 1) >> 8) as u8,
                ((ox + w - 1) & 0xFF) as u8,
            ],
        );

        self.send_command_data(
            Instruction::RASET,
            &[
                (oy >> 8) as u8,
                (oy & 0xFF) as u8,
                ((oy + h - 1) >> 8) as u8,
                ((oy + h - 1) & 0xFF) as u8,
            ],
        );
    }

    pub fn write_raw_pixel(&mut self, x: u16, y: u16, data: &[u8]) {
        self.set_update_window(x, y, 1, 1);

        self.send_command_data(Instruction::RAMWR, data);
    }

    fn send_command(&mut self, cmd: Instruction) {
        self.dc.set_low();
        self.spi.blocking_write(&[cmd as u8]).unwrap();
    }

    fn send_data(&mut self, data: &[u8]) {
        self.dc.set_high();
        self.spi.blocking_write(data).unwrap();
    }

    fn send_command_data(&mut self, cmd: Instruction, data: &[u8]) {
        self.send_command(cmd);
        self.send_data(data);
    }
}

impl<const WIDTH: u16, const HEIGHT: u16, const OFFSETX: u16, const OFFSETY: u16> OriginDimensions
    for ST7735<WIDTH, HEIGHT, OFFSETX, OFFSETY>
{
    fn size(&self) -> Size {
        Size::new(WIDTH as _, HEIGHT as _)
    }
}

impl<const WIDTH: u16, const HEIGHT: u16, const OFFSETX: u16, const OFFSETY: u16> DrawTarget
    for ST7735<WIDTH, HEIGHT, OFFSETX, OFFSETY>
{
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::prelude::Pixel<Self::Color>>,
    {
        for pixel in pixels {
            let x = pixel.0.x as u16;
            let y = pixel.0.y as u16;
            let color = pixel.1;

            self.write_raw_pixel(x, y, color.to_be_bytes().as_ref());
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.set_update_window(0, 0, WIDTH, HEIGHT);

        self.send_command(Instruction::RAMWR);
        for _ in 0..((WIDTH as u16) * (HEIGHT as u16)) {
            self.send_data(color.to_be_bytes().as_ref());
        }
        Ok(())
    }

    fn fill_contiguous<I>(
        &mut self,
        area: &embedded_graphics::primitives::Rectangle,
        colors: I,
    ) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        self.set_update_window(
            area.top_left.x as u16,
            area.top_left.y as u16,
            area.size.width as u16,
            area.size.height as u16,
        );

        self.send_command(Instruction::RAMWR);
        for color in colors {
            self.send_data(color.to_be_bytes().as_ref());
        }
        Ok(())
    }

    fn fill_solid(
        &mut self,
        area: &embedded_graphics::primitives::Rectangle,
        color: Self::Color,
    ) -> Result<(), Self::Error> {
        self.set_update_window(
            area.top_left.x as u16,
            area.top_left.y as u16,
            area.size.width as u16,
            area.size.height as u16,
        );

        self.send_command(Instruction::RAMWR);
        for _ in 0..(area.size.width * area.size.height) {
            self.send_data(color.to_be_bytes().as_ref());
        }
        Ok(())
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSE;
    let p = hal::init(config);


    // SPI1, remap 0
    let cs = p.PC1;
    let sck = p.PC5;
    let sda = p.PC6;

    let rst = p.PC2;
    let dc = p.PC0;

    // let led = p.PB8;
    let mut led = Output::new(p.PD6, Level::Low, Default::default());

    let mut cs = Output::new(cs, Level::High, Default::default());
    let dc = Output::new(dc.degrade(), Level::High, Default::default());
    let mut rst = Output::new(rst, Level::High, Default::default());

    cs.set_low();

    let mut spi_config = hal::spi::Config::default();
    spi_config.frequency = Hertz::mhz(1);

    // Remap 0
    let spi = Spi::new_blocking_txonly::<0>(p.SPI1, sck, sda, spi_config);

    rst.set_low();
    //    Timer::after_millis(120).await;
    riscv::asm::delay(100000);
    rst.set_high();
    //  Timer::after_millis(20).await;
    riscv::asm::delay(100000);

    let mut display: ST7735<160, 80, 1, 26> = ST7735::new(spi, dc);

    display.init();

    display.clear(Rgb565::BLACK).unwrap();

    Line::new(Point::new(0, 0), Point::new(159, 0))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 1))
        .draw(&mut display)
        .unwrap();
    Line::new(Point::new(0, 0), Point::new(0, 79))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
        .draw(&mut display)
        .unwrap();
    Line::new(Point::new(159, 0), Point::new(159, 79))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLUE, 1))
        .draw(&mut display)
        .unwrap();
    Line::new(Point::new(0, 79), Point::new(159, 79))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(&mut display)
        .unwrap();

    // cube animation
    const POINTS: usize = 8;
    // A 3D cube has 8 points, each point has 3 values (x, y, z)
    let orig_points: [[f32; 3]; POINTS] = [
        [1.0, 1.0, 1.0],
        [1.0, -1.0, 1.0],
        [-1.0, -1.0, 1.0],
        [-1.0, 1.0, 1.0],
        [1.0, 1.0, -1.0],
        [1.0, -1.0, -1.0],
        [-1.0, -1.0, -1.0],
        [-1.0, 1.0, -1.0],
    ];

    // Perspective Projection:
    // distance to project plane
    let mut d = 1.4;

    let z_offset = -3.0; // offset on z axis, leave object

    let cube_size = 30.0;
    let screen_offset_x = 160.0 / 2.0;
    let screen_offset_y = 80.0 / 2.0;
    let mut rotate_angle = 0.0_f32;

    let mut rotated_points = [[0.0; 3]; POINTS];
    let mut projected_points = [[0.0; 2]; POINTS];
    let mut points = [[0_i32; 2]; POINTS];

    let mut inc = true;

    loop {
        // clear
        for ([idx, idy], color) in [
            // use different colors for each connected line
            ([0, 1], Rgb565::BLACK),
            ([1, 2], Rgb565::BLACK),
            ([2, 3], Rgb565::BLACK),
            ([3, 0], Rgb565::BLACK),
            ([4, 5], Rgb565::BLACK),
            ([5, 6], Rgb565::BLACK),
            ([6, 7], Rgb565::BLACK),
            ([7, 4], Rgb565::BLACK),
            ([0, 4], Rgb565::BLACK),
            ([1, 5], Rgb565::BLACK),
            ([2, 6], Rgb565::BLACK),
            ([3, 7], Rgb565::BLACK),
        ]
        .iter()
        {
            Line::new(
                Point::new(points[*idx][0], points[*idx][1]),
                Point::new(points[*idy][0], points[*idy][1]),
            )
            .into_styled(PrimitiveStyle::with_stroke(*color, 1))
            .draw(&mut display)
            .unwrap();
        }

        for (i, [x, y, z]) in orig_points.iter().copied().enumerate() {
            // rotate around y axis
            rotated_points[i][0] = x * rotate_angle.cos() + z * rotate_angle.sin();
            rotated_points[i][1] = y;
            rotated_points[i][2] = -x * rotate_angle.sin() + z * rotate_angle.cos();

            let rx = rotated_points[i][0];
            let ry = rotated_points[i][1];
            let rz = rotated_points[i][2] + z_offset; // offset on z axis, leave object

            /*
            x' = (d * x) / z
            y' = (d * y) / z
             */
            projected_points[i][0] = (d * rx) / rz;
            projected_points[i][1] = (d * ry) / rz;
        }

        for (i, [x, y]) in projected_points.iter().enumerate() {
            let x: f32 = (x * cube_size) + screen_offset_x;
            let y: f32 = (y * cube_size) + screen_offset_y;

            // manually round
            points[i][0] = (x + 0.5) as i32;
            points[i][1] = (y + 0.5) as i32;
        }

        // display.clear(Rgb565::BLACK);
        for ([idx, idy], color) in [
            // use different colors for each connected line
            ([0, 1], Rgb565::CSS_ALICE_BLUE),
            ([1, 2], Rgb565::GREEN),
            ([2, 3], Rgb565::BLUE),
            ([3, 0], Rgb565::CSS_SALMON),
            ([4, 5], Rgb565::YELLOW),
            ([5, 6], Rgb565::CYAN),
            ([6, 7], Rgb565::MAGENTA),
            ([7, 4], Rgb565::CSS_KHAKI),
            ([0, 4], Rgb565::CSS_BEIGE),
            ([1, 5], Rgb565::RED),
            ([2, 6], Rgb565::CSS_CORNFLOWER_BLUE),
            ([3, 7], Rgb565::CSS_SILVER),
        ]
        .iter()
        {
            Line::new(
                Point::new(points[*idx][0], points[*idx][1]),
                Point::new(points[*idy][0], points[*idy][1]),
            )
            .into_styled(PrimitiveStyle::with_stroke(*color, 1))
            .draw(&mut display)
            .unwrap();
        }

        //Text::new("@andelf", Point::new(2, 10), text_style)
        //    .draw(&mut display)
        //    .unwrap();

        //    display.f(&mut delay);
        //Timer::after(Duration::from_micros(20)).await;
        riscv::asm::delay(1000000);
        rotate_angle += 0.05;

        if rotate_angle > 2.0 * 3.1415 {
            rotate_angle = 0.0;
        }
        if inc {
            d += 0.01;
            if d > 2.0 {
                inc = false;
            }
        } else {
            d -= 0.01;
            if d < 0.5 {
                inc = true;
            }
        }
        led.toggle();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // CH32V003 is so resource constrained, adding strings is a bad idea.
    // let _ = println!("\n\n\n{}", info);

    loop {}
}
