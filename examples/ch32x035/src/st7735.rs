use ch32_hal::dma::NoDma;
use ch32_hal::gpio::Output;
use ch32_hal::spi::Spi;
use embassy_time::Delay;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_hal::delay::DelayNs as _;

use crate::hal::peripherals;

pub mod cmds {
    pub const NOP: u8 = 0x00;
    pub const SWRESET: u8 = 0x01;
    pub const RDDID: u8 = 0x04;
    pub const RDDST: u8 = 0x09;
    pub const SLPIN: u8 = 0x10;
    pub const SLPOUT: u8 = 0x11;
    pub const PTLON: u8 = 0x12;
    pub const NORON: u8 = 0x13;
    pub const INVOFF: u8 = 0x20;
    pub const INVON: u8 = 0x21;
    pub const DISPOFF: u8 = 0x28;
    pub const DISPON: u8 = 0x29;
    pub const CASET: u8 = 0x2A;
    pub const RASET: u8 = 0x2B;
    pub const RAMWR: u8 = 0x2C;
    pub const RAMRD: u8 = 0x2E;
    pub const PTLAR: u8 = 0x30;
    pub const COLMOD: u8 = 0x3A;
    pub const MADCTL: u8 = 0x36;
    pub const FRMCTR1: u8 = 0xB1;
    pub const FRMCTR2: u8 = 0xB2;
    pub const FRMCTR3: u8 = 0xB3;
    pub const INVCTR: u8 = 0xB4;
    pub const DISSET5: u8 = 0xB6;
    pub const PWCTR1: u8 = 0xC0;
    pub const PWCTR2: u8 = 0xC1;
    pub const PWCTR3: u8 = 0xC2;
    pub const PWCTR4: u8 = 0xC3;
    pub const PWCTR5: u8 = 0xC4;
    pub const VMCTR1: u8 = 0xC5;
    pub const RDID1: u8 = 0xDA;
    pub const RDID2: u8 = 0xDB;
    pub const RDID3: u8 = 0xDC;
    pub const RDID4: u8 = 0xDD;
    pub const PWCTR6: u8 = 0xFC;
    pub const GMCTRP1: u8 = 0xE0;
    pub const GMCTRN1: u8 = 0xE1;
}

pub struct ST7735<const WIDTH: u16, const HEIGHT: u16, const OFFSETX: u16 = 0, const OFFSETY: u16 = 0> {
    spi: Spi<'static, peripherals::SPI1, NoDma, NoDma>,
    dc: Output<'static>,
}

impl<const WIDTH: u16, const HEIGHT: u16, const OFFSETX: u16, const OFFSETY: u16>
    ST7735<WIDTH, HEIGHT, OFFSETX, OFFSETY>
{
    pub fn new(spi: Spi<'static, peripherals::SPI1, NoDma, NoDma>, dc: Output<'static>) -> Self {
        Self { spi, dc }
    }

    pub fn init(&mut self) {
        self.send_command(cmds::SWRESET);
        Delay.delay_ms(20);
        self.send_command(cmds::SLPOUT);
        Delay.delay_ms(200);

        self.send_command_data(cmds::FRMCTR1, &[0x01, 0x2C, 0x2D]);
        self.send_command_data(cmds::FRMCTR2, &[0x01, 0x2C, 0x2D]);
        self.send_command_data(cmds::FRMCTR3, &[0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D]);
        self.send_command_data(cmds::INVCTR, &[0x07]);
        self.send_command_data(cmds::PWCTR1, &[0xA2, 0x02, 0x84]);
        self.send_command_data(cmds::PWCTR2, &[0xC5]);
        self.send_command_data(cmds::PWCTR3, &[0x0A, 0x00]);
        self.send_command_data(cmds::PWCTR4, &[0x8A, 0x2A]);
        self.send_command_data(cmds::PWCTR5, &[0x8A, 0xEE]);
        self.send_command_data(cmds::VMCTR1, &[0x0E]);

        // Some displays are INVON
        // self.send_command(cmds::INVOFF);
        self.send_command(cmds::INVON);

        // BITS:
        // MY, MX, MV, ML, RGB, MV, _, _
        self.send_command_data(cmds::MADCTL, &[0b0110_100_0]);

        self.send_command_data(cmds::COLMOD, &[0x05]); // 16-bit/pixel
        self.send_command_data(cmds::DISPON, &[]);

        Delay.delay_ms(200);
    }

    #[inline]
    fn set_update_window(&mut self, x: u16, y: u16, w: u16, h: u16) {
        let ox = OFFSETX + x;
        let oy = OFFSETY + y;

        self.send_command_data(
            cmds::CASET,
            &[
                (ox >> 8) as u8,
                (ox & 0xFF) as u8,
                ((ox + w - 1) >> 8) as u8,
                ((ox + w - 1) & 0xFF) as u8,
            ],
        );

        self.send_command_data(
            cmds::RASET,
            &[
                (oy >> 8) as u8,
                (oy & 0xFF) as u8,
                ((oy + h - 1) >> 8) as u8,
                ((oy + h - 1) & 0xFF) as u8,
            ],
        );
    }

    pub fn write_raw_framebuffer(&mut self, data: &[u8]) {
        self.set_update_window(0, 0, WIDTH, HEIGHT);

        self.send_command_data(cmds::RAMWR, data);
    }

    pub fn write_raw_pixel(&mut self, x: u16, y: u16, data: &[u8]) {
        self.set_update_window(x, y, 1, 1);

        self.send_command_data(cmds::RAMWR, data);
    }

    fn send_command(&mut self, cmd: u8) {
        self.dc.set_low();
        self.spi.blocking_write(&[cmd]).unwrap();
    }

    fn send_data(&mut self, data: &[u8]) {
        self.dc.set_high();
        self.spi.blocking_write(data).unwrap();
    }

    fn send_command_data(&mut self, cmd: u8, data: &[u8]) {
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

        self.send_command(cmds::RAMWR);
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

        self.send_command(cmds::RAMWR);
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

        self.send_command(cmds::RAMWR);
        for _ in 0..(area.size.width * area.size.height) {
            self.send_data(color.to_be_bytes().as_ref());
        }
        Ok(())
    }
}
