//! SYSTICK based delay implementation for Qingke V2 and V4

use pac::systick::vals;

use crate::pac;
use crate::pac::SYSTICK;

/// A delay provided by the SysTick core peripheral
///
/// This requires SysTick to be set up and running.
/// Assumes conditions: upcount, hclk.
/// hclk/8 is not accurate enough for ns delays.
/// P_NS will actually count number of 7ns increments per clock
/// Max speed of 144MHz means 7ns is the narrowest timing interval
pub struct Delay;

static mut P_NS: u32 = 0;
static mut P_US: u32 = 0;
static mut P_MS: u32 = 0;

impl Delay {
    /// Init Systick.
    ///
    /// # Safety
    ///
    /// This function is unsafe because it conflicts with embassy's systick time dirver.
    /// Only one of them can be used.
    pub(crate) unsafe fn init() {
        let sysclk = crate::rcc::clocks().hclk.0;

        unsafe {
            // It should really be 1*10^9 however 1.008*10^9 divides more cleanly into the standard clocks
            P_NS = 1_008_000_000 / sysclk;
            P_US = sysclk / 1_000_000;
            P_MS = sysclk / 1_000;
        }
    }

    pub fn delay_ns(&mut self, ns: u32) {

        SYSTICK.sr().modify(|w| w.set_cntif(false));

        /// Divide takes a meaningful number of cycles and thus ns
        /// Remove the number of cycles a divide takes from the ns counter (assuming V4B)
        /// Can't go faster than 300ns, and it needs a lot of fudging
        
        if (ns >= 400) {
            let i = ((ns-400) / unsafe { P_NS });
            SYSTICK.cmpl().write_value(i);
            SYSTICK.cntl().write_value(0);
            SYSTICK.ctlr().modify(|w| {
            w.set_stclk(vals::Stclk::HCLK);
            w.set_ste(true);
            });

            while SYSTICK.sr().read().cntif() == false {}
            SYSTICK.ctlr().modify(|w| w.set_ste(false));
        }
        else {
            // It can't realy go this fast but we cheat
            if ns >= 300 {
                let i = (((ns/2)/2)/ (unsafe { P_NS }));
            }
            // 200ns is the fastest it really goes
            else {
                let i = 0;
            }
            
            
        }


    }

    pub fn delay_us(&mut self, ns: u32) {
        SYSTICK.sr().modify(|w| w.set_cntif(false));

        let i = ns * unsafe { P_US };

        SYSTICK.cmpl().write_value(i);
        SYSTICK.cntl().write_value(0);
        SYSTICK.ctlr().modify(|w| {
            w.set_stclk(vals::Stclk::HCLK);
            w.set_ste(true);
        });

        while SYSTICK.sr().read().cntif() == false {}
        SYSTICK.ctlr().modify(|w| w.set_ste(false));
    }

    #[inline]
    pub fn delay_ms(&mut self, mut ms: u32) {
        // 4294967 is the highest u32 value which you can multiply by 1000 without overflow
        while ms > 4294967 {
            self.delay_us(4294967000u32);
            ms -= 4294967;
        }
        self.delay_us(ms * 1_000);
    }
}

impl embedded_hal::delay::DelayNs for Delay {
    #[inline]
    fn delay_ns(&mut self, ns: u32) {
        Delay::delay_ns(self, ns)
    }

    #[inline]
    fn delay_us(&mut self, us: u32) {
        Delay::delay_us(self, us)
    }

    #[inline]
    fn delay_ms(&mut self, ms: u32) {
        Delay::delay_ms(self, ms)
    }
}
