use std::collections::{BTreeMap, HashMap, HashSet};
use std::fmt::Write;
use std::path::PathBuf;
use std::{env, fs};

use ch32_metapac::metadata::METADATA;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Check chip name feature flags
    let chip_name = match env::vars()
        .map(|(a, _)| a)
        .filter(|x| x.starts_with("CARGO_FEATURE_CH32") || x.starts_with("CARGO_FEATURE_CH6"))
        .get_one()
    {
        Ok(x) => x,
        Err(GetOneError::None) => panic!("No ch32xx/ch6xx Cargo feature enabled"),
        Err(GetOneError::Multiple) => panic!("Multiple ch32xx/ch6xx Cargo features enabled"),
    }
    .strip_prefix("CARGO_FEATURE_")
    .unwrap()
    .to_ascii_lowercase();

    // Add chip name and family cfg flags on the fly
    let (chip_base_name, chip_family) = if chip_name.starts_with("ch32") {
        (chip_name[..8].to_string(), chip_name[..6].to_string())
    } else {
        // On of ch643, ch641
        (chip_name[..5].to_string(), chip_name[..5].to_string())
    };
    println!("cargo:rustc-cfg={}", chip_base_name); // ch32v103, ch32v003, ch32x035, ch643, ch641, etc.
    println!("cargo:rustc-cfg={}", chip_family); // On of ch32x0, ch32v0, ch32v1, ch32v2, ch32v3, ch32l1, ch643, ch641

    // Add Qingke IP core version cfg flags on the fly
    // qingke_v2, qingke_v3, qingke_v4
    let qingke_ver = match &*chip_family {
        "ch32v0" | "ch641" => "qingke_v2",
        "ch32v1" => "qingke_v3",
        "ch32v2" | "ch32v3" | "ch32l1" | "ch643" => "qingke_v4", // v4b, v4c, v4f
        _ => "qingke_v4",
    };
    println!("cargo:rustc-cfg={}", qingke_ver);

    // Add CH32 specific cfg flags: D6, D8, D8C, D8W
    // D(Density), 6(2^6), 8(2^8)
    // C(Connectivity / Interconnectivity)
    // W(Wireless)
    if chip_name.starts_with("ch32") {
        let density = match chip_name.as_bytes()[9] {
            b'6' | b'7' | b'8' => Some("D6"),
            b'b' | b'c' => Some("D8"),
            _ => None, // undocumented, leave it empty
        };
        let subtype = match &chip_name[6..8] {
            "03" => Some(""),  // General purpose
            "05" => Some("C"), // Connectivity
            "07" => Some("C"), // Interconnectivity
            "08" => Some("W"), // Wireless
            _ => None,         // 35: Connectivity of USBPD
        };
        match (density, subtype) {
            (Some(density), Some(subtype)) => {
                println!(
                    "cargo:rustc-cfg={}{}",
                    density.to_ascii_lowercase(),
                    subtype.to_ascii_lowercase()
                );
            }
            _ => (),
        }
    }

    // Add peripheral cfg flags on the fly
    for p in METADATA.peripherals {
        if let Some(r) = &p.registers {
            println!("cargo:rustc-cfg={}", r.kind);
            println!("cargo:rustc-cfg={}_{}", r.kind, r.version);
        }
    }

    let mut gpio_lines = 16;
    match &*chip_family {
        "ch32v0" => {
            gpio_lines = 8;
        }
        "ch32x0" => {
            gpio_lines = 24;
        }
        "ch643" => {
            gpio_lines = 24;
        }
        _ => {}
    }

    // ========
    // Generate singletons
    let mut singletons: Vec<String> = Vec::new();
    for p in METADATA.peripherals {
        if let Some(r) = &p.registers {
            println!("cargo:rustc-cfg=peri_{}", p.name.to_ascii_lowercase());
            match r.kind {
                // Generate singletons per pin, not per port
                "gpio" => {
                    let port_letter: &str = p.name.strip_prefix("GPIO").unwrap();
                    for pin_num in 0..gpio_lines {
                        singletons.push(format!("P{}{}", port_letter, pin_num));
                    }
                }

                // No singleton for these, the HAL handles them specially.
                "exti" => {}

                // We *shouldn't* have singletons for these, but the HAL currently requires
                // singletons, for using with RccPeripheral to enable/disable clocks to them.
                "rcc" => {
                    for pin in p.pins {
                        if pin.signal.starts_with("MCO") {
                            let name = pin.signal.replace('_', "").to_string();
                            if !singletons.contains(&name) {
                                println!("cargo:rustc-cfg={}", name.to_ascii_lowercase());
                                singletons.push(name);
                            }
                        }
                    }
                    singletons.push(p.name.to_string());
                }
                // "dma" => {}

                // For other peripherals, one singleton per peri
                _ => singletons.push(p.name.to_string()),
            }
        }
    }

    // One singleton per EXTI line
    for pin_num in 0..gpio_lines {
        singletons.push(format!("EXTI{}", pin_num));
    }

    // One singleton per DMA channel
    for c in METADATA.dma_channels {
        singletons.push(c.name.to_string());
    }

    let mut pin_set = std::collections::HashSet::new();
    for p in METADATA.peripherals {
        for pin in p.pins {
            pin_set.insert(pin.pin);
        }
    }

    // ========
    // Handle time-driver-XXXX features.

    let time_driver = match env::vars()
        .map(|(a, _)| a)
        .filter(|x| x.starts_with("CARGO_FEATURE_TIME_DRIVER_"))
        .get_one()
    {
        Ok(x) => Some(
            x.strip_prefix("CARGO_FEATURE_TIME_DRIVER_")
                .unwrap()
                .to_ascii_lowercase(),
        ),
        Err(GetOneError::None) => None,
        Err(GetOneError::Multiple) => panic!("Multiple ch32xx Cargo features enabled"),
    };

    let time_driver_singleton = match time_driver.as_ref().map(|x| x.as_ref()) {
        None => "",
        Some("tim1") => "TIM1",
        Some("tim2") => "TIM2",
        Some("tim3") => "TIM3",
        Some("tim4") => "TIM4",
        Some("tim5") => "TIM5",
        // TIM6 and TIM7 are BCTM
        Some("tim8") => "TIM8",
        Some("tim9") => "TIM9",
        Some("tim10") => "TIM10",
        Some("any") => {
            [
                "TIM5", "TIM4", "TIM3", "TIM2", // GP16 / GP32
                "TIM10", "TIM9", "TIM8", "TIM1", //ADV
            ]
            .iter()
            .find(|tim| singletons.contains(&tim.to_string()))
            .expect("time-driver-any requested, but the chip doesn't have a TIMx for time driver")
        }
        _ => panic!("unknown time_driver {:?}", time_driver),
    };

    if !time_driver_singleton.is_empty() {
        println!("cargo:rustc-cfg=time_driver_{}", time_driver_singleton.to_lowercase());
        println!("cargo:rustc-cfg=time_driver_timer");
    }

    // ========
    // Write singletons

    // _generated.rs
    let mut g = TokenStream::new();

    let singleton_tokens: Vec<_> = singletons.iter().map(|s| format_ident!("{}", s)).collect();

    g.extend(quote! {
        crate::peripherals_definition!(#(#singleton_tokens),*);
    });

    let singleton_tokens: Vec<_> = singletons
        .iter()
        // .filter(|s| *s != &time_driver_singleton.to_string())
        .map(|s| format_ident!("{}", s))
        .collect();

    g.extend(quote! {
        crate::peripherals_struct!(#(#singleton_tokens),*);
    });

    // ========
    // Generate interrupt declarations
    let mut irqs = Vec::new();
    for irq in METADATA.interrupts {
        irqs.push(format_ident!("{}", irq.name));
    }

    g.extend(quote! {
        crate::interrupt_mod!(
            #(
                #irqs,
            )*
        );
    });

    // ========
    // Extract the rcc registers
    // let rcc_registers = METADATA
    //     .peripherals
    //     .iter()
    //     .filter_map(|p| p.registers.as_ref())
    //     .find(|r| r.kind == "rcc")
    //     .unwrap();

    // ========
    // Generate RccPeripheral and RemapPeripheral impls
    for p in METADATA.peripherals {
        if !singletons.contains(&p.name.to_string()) {
            continue;
        }
        let pname = format_ident!("{}", p.name);

        if let Some(rcc) = &p.rcc {
            let en = rcc.enable.as_ref().unwrap();

            let rst = match &rcc.reset {
                Some(rst) => {
                    let rst_reg = format_ident!("{}", rst.register.to_ascii_lowercase());
                    let set_rst_field = format_ident!("set_{}", rst.field.to_ascii_lowercase());
                    quote! {
                        crate::pac::RCC.#rst_reg().modify(|w| w.#set_rst_field(true));
                        crate::pac::RCC.#rst_reg().modify(|w| w.#set_rst_field(false));
                    }
                }
                None => TokenStream::new(),
            };

            // let ptype = if let Some(reg) = &p.registers { reg.kind } else { "" };

            let en_reg = format_ident!("{}", en.register.to_ascii_lowercase());
            let set_en_field = format_ident!("set_{}", en.field.to_ascii_lowercase());

            let clk = format_ident!("{}", rcc.bus_clock.to_ascii_lowercase());

            g.extend(quote! {
                impl crate::peripheral::SealedRccPeripheral for peripherals::#pname {
                    fn frequency() -> crate::time::Hertz {
                        crate::rcc::clocks().#clk
                    }
                    fn enable_and_reset_with_cs(_cs: critical_section::CriticalSection) {
                        crate::pac::RCC.#en_reg().modify(|w| w.#set_en_field(true));
                        #rst
                    }
                    fn disable_with_cs(_cs: critical_section::CriticalSection) {
                        crate::pac::RCC.#en_reg().modify(|w| w.#set_en_field(false));
                    }
                }

                impl crate::peripheral::RccPeripheral for peripherals::#pname {}
            });
        }

        if let Some(remap) = &p.remap {
            let remap_reg = format_ident!("{}", remap.register.to_ascii_lowercase());
            let set_remap_field = format_ident!("set_{}", remap.field.to_ascii_lowercase());

            g.extend(quote! {
                impl crate::peripheral::SealedRemapPeripheral for peripherals::#pname {
                    fn set_remap(remap: u8) {
                        crate::pac::AFIO.#remap_reg().modify(|w| w.#set_remap_field(unsafe { core::mem::transmute(remap) }));
                    }
                }

                impl crate::peripheral::RemapPeripheral for peripherals::#pname {}
            });
        }

        // TODO
        if let Some(regs) = &p.registers {
            let kind = regs.kind.to_string();

            if ["i2c"].contains(&&*kind) {
                g.extend(quote! {
                    //
                })
            }
        }
    }

    // ========
    // Generate fns to enable GPIO, DMA in RCC
    for kind in ["dma", "gpio"] {
        let mut gg = TokenStream::new();

        for p in METADATA.peripherals {
            if p.registers.is_some() && p.registers.as_ref().unwrap().kind == kind {
                if let Some(rcc) = &p.rcc {
                    let en = rcc.enable.as_ref().unwrap();
                    let en_reg = format_ident!("{}", en.register.to_ascii_lowercase());
                    let set_en_field = format_ident!("set_{}", en.field.to_ascii_lowercase());

                    gg.extend(quote! {
                        crate::pac::RCC.#en_reg().modify(|w| w.#set_en_field(true));
                    })
                }
            }
        }

        let fname = format_ident!("init_{}", kind);
        g.extend(quote! {
            pub unsafe fn #fname(){
                #gg
            }
        })
    }

    // ========
    // Generate pin_trait_impl!
    let signals: HashMap<_, _> = [
        // (kind, signal) => trait
        (("usart", "TX"), quote!(crate::usart::TxPin)),
        (("usart", "RX"), quote!(crate::usart::RxPin)),
        (("usart", "CTS"), quote!(crate::usart::CtsPin)),
        (("usart", "RTS"), quote!(crate::usart::RtsPin)),
        (("usart", "CK"), quote!(crate::usart::CkPin)),
        (("spi", "MISO"), quote!(crate::spi::MisoPin)),
        (("spi", "SCK"), quote!(crate::spi::SckPin)),
        (("spi", "MOSI"), quote!(crate::spi::MosiPin)),
        (("spi", "NSS"), quote!(crate::spi::CsPin)),
        /*(("spi", "I2S_MCK"), quote!(crate::spi::MckPin)),
        (("spi", "I2S_CK"), quote!(crate::spi::CkPin)),
        (("spi", "I2S_WS"), quote!(crate::spi::WsPin)), */
        (("i2c", "SDA"), quote!(crate::i2c::SdaPin)),
        (("i2c", "SCL"), quote!(crate::i2c::SclPin)),
        (("timer", "CH1"), quote!(crate::timer::Channel1Pin)),
        (("timer", "CH1N"), quote!(crate::timer::Channel1ComplementaryPin)),
        (("timer", "CH2"), quote!(crate::timer::Channel2Pin)),
        (("timer", "CH2N"), quote!(crate::timer::Channel2ComplementaryPin)),
        (("timer", "CH3"), quote!(crate::timer::Channel3Pin)),
        (("timer", "CH3N"), quote!(crate::timer::Channel3ComplementaryPin)),
        (("timer", "CH4"), quote!(crate::timer::Channel4Pin)),
        (("timer", "CH4N"), quote!(crate::timer::Channel4ComplementaryPin)),
        (("timer", "ETR"), quote!(crate::timer::ExternalTriggerPin)),
        (("timer", "BKIN"), quote!(crate::timer::BreakInputPin)),
        // sdio is the sdmmc(v1) in stm32
        (("sdio", "CK"), quote!(crate::sdio::CkPin)),
        (("sdio", "CMD"), quote!(crate::sdio::CmdPin)),
        (("sdio", "D0"), quote!(crate::sdio::D0Pin)),
        (("sdio", "D1"), quote!(crate::sdio::D1Pin)),
        (("sdio", "D2"), quote!(crate::sdio::D2Pin)),
        (("sdio", "D3"), quote!(crate::sdio::D3Pin)),
        (("sdio", "D4"), quote!(crate::sdio::D4Pin)),
        (("sdio", "D5"), quote!(crate::sdio::D5Pin)),
        (("sdio", "D6"), quote!(crate::sdio::D6Pin)),
        (("sdio", "D6"), quote!(crate::sdio::D7Pin)),
        (("sdio", "D8"), quote!(crate::sdio::D8Pin)),
        // otg_fs
        (("otg", "DP"), quote!(crate::otg_fs::DpPin)),
        (("otg", "DM"), quote!(crate::otg_fs::DmPin)),
        // USB is splitted into multiple impls
        (("usbd", "DP"), quote!(crate::usbd::DpPin)),
        (("usbd", "DM"), quote!(crate::usbd::DmPin)),
        (("usbhs", "DP"), quote!(crate::usbhs::DpPin)),
        (("usbhs", "DM"), quote!(crate::usbhs::DmPin)),
        // USBPD, handled by usbpd/mod.rs
        //(("usbpd", "CC1"), quote!(crate::usbpd::Cc1Pin)),
        //(("usbpd", "CC2"), quote!(crate::usbpd::Cc2Pin)),
        (("can", "TX"), quote!(crate::can::TxPin)),
        (("can", "RX"), quote!(crate::can::RxPin)),
    ]
    .into();

    for p in METADATA.peripherals {
        if let Some(regs) = &p.registers {
            for pin in p.pins {
                let key = (regs.kind, pin.signal);

                // singnals and pins
                if let Some(tr) = signals.get(&key) {
                    let peri = format_ident!("{}", p.name);
                    let pin_name = format_ident!("{}", pin.pin);

                    let remap = pin.remap.unwrap_or(0);

                    g.extend(quote! {
                        pin_trait_impl!(#tr, #peri, #pin_name, #remap);
                    });

                    // panic!("{} {}", peri, pin_name);
                }

                // ADC pin is special
                if regs.kind == "adc" {
                    if p.rcc.is_none() {
                        continue;
                    }

                    let peri = format_ident!("{}", p.name);
                    let pin_name = format_ident!("{}", pin.pin);

                    let ch: Option<u8> = if pin.signal.starts_with("IN") {
                        Some(pin.signal.strip_prefix("IN").unwrap().parse().unwrap())
                    } else {
                        None
                    };
                    if let Some(ch) = ch {
                        g.extend(quote! {
                            impl_adc_pin!( #peri, #pin_name, #ch);
                        });
                    }
                }

                // DAC is special
                if regs.kind == "dac" {
                    let peri = format_ident!("{}", p.name);
                    let pin_name = format_ident!("{}", pin.pin);
                    let ch: u8 = pin.signal.strip_prefix("OUT").unwrap().parse().unwrap();

                    g.extend(quote! {
                        impl_dac_pin!( #peri, #pin_name, #ch);
                    })
                }
            }
        }
    }

    // ========
    // Generate dma_trait_impl!

    let signals: HashMap<_, _> = [
        // (kind, signal) => trait
        (("usart", "RX"), quote!(crate::usart::RxDma)),
        (("usart", "TX"), quote!(crate::usart::TxDma)),
        (("spi", "RX"), quote!(crate::spi::RxDma)),
        (("spi", "TX"), quote!(crate::spi::TxDma)),
        (("i2c", "RX"), quote!(crate::i2c::RxDma)),
        (("i2c", "TX"), quote!(crate::i2c::TxDma)),
        (("timer", "CH1"), quote!(crate::timer::Ch1Dma)),
        (("timer", "CH2"), quote!(crate::timer::Ch2Dma)),
        (("timer", "CH3"), quote!(crate::timer::Ch3Dma)),
        (("timer", "CH4"), quote!(crate::timer::Ch4Dma)),
        (("sdio", "SDIO"), quote!(crate::sdio::SdioDma)),
    ]
    .into();

    for p in METADATA.peripherals {
        if let Some(regs) = &p.registers {
            let mut dupe = HashSet::new();
            for ch in p.dma_channels {
                // Some chips have multiple request numbers for the same (peri, signal, channel) combos.
                // Ignore the dupes, picking the first one. Otherwise this causes conflicting trait impls
                let key = (ch.signal, ch.channel);
                if !dupe.insert(key) {
                    continue;
                }

                if let Some(tr) = signals.get(&(regs.kind, ch.signal)) {
                    let peri = format_ident!("{}", p.name);

                    let channel = if let Some(channel) = &ch.channel {
                        // Chip with DMA/BDMA, without DMAMUX
                        let channel = format_ident!("{}", channel);
                        quote!({channel: #channel})
                    } else {
                        unreachable!();
                    };

                    let request = quote!(());

                    g.extend(quote! {
                        dma_trait_impl!(#tr, #peri, #channel, #request);
                    });
                }
            }
        }
    }

    // ========
    // Write peripheral_interrupts module.
    let mut mt = TokenStream::new();
    for p in METADATA.peripherals {
        let mut pt = TokenStream::new();

        for irq in p.interrupts {
            let iname = format_ident!("{}", irq.interrupt);
            let sname = format_ident!("{}", irq.signal);
            pt.extend(quote!(pub type #sname = crate::interrupt::typelevel::#iname;));
        }

        let pname = format_ident!("{}", p.name);
        mt.extend(quote!(pub mod #pname { #pt }));
    }
    g.extend(quote!(#[allow(non_camel_case_types)] pub mod peripheral_interrupts { #mt }));

    // ========
    // Write foreach_foo! macrotables
    let mut peripherals_table: Vec<Vec<String>> = Vec::new();
    let mut interrupts_table: Vec<Vec<String>> = Vec::new();
    let mut pins_table: Vec<Vec<String>> = Vec::new();

    let gpio_base = METADATA.peripherals.iter().find(|p| p.name == "GPIOA").unwrap().address as u32;
    let gpio_stride = 0x400;

    for p in METADATA.peripherals {
        if let Some(regs) = &p.registers {
            if regs.kind == "gpio" {
                let port_letter = p.name.chars().nth(4).unwrap();
                assert_eq!(0, (p.address as u32 - gpio_base) % gpio_stride);
                let port_num = (p.address as u32 - gpio_base) / gpio_stride;

                for pin_num in 0..gpio_lines {
                    let pin_name = format!("P{}{}", port_letter, pin_num);

                    pins_table.push(vec![
                        pin_name.clone(),
                        p.name.to_string(),
                        port_num.to_string(),
                        pin_num.to_string(),
                        format!("EXTI{}", pin_num),
                    ]);
                }
            }

            for irq in p.interrupts {
                let row = vec![
                    p.name.to_string(),
                    regs.kind.to_string(),
                    regs.block.to_string(),
                    irq.signal.to_string(),
                    irq.interrupt.to_ascii_uppercase(),
                ];
                interrupts_table.push(row)
            }

            let row = vec![regs.kind.to_string(), p.name.to_string()];
            peripherals_table.push(row);
        }
    }

    // DMA
    let mut dmas = TokenStream::new();

    for (ch_idx, ch) in METADATA.dma_channels.iter().enumerate() {
        let name = format_ident!("{}", ch.name);
        let idx = ch_idx as u8;
        g.extend(quote!(dma_channel_impl!(#name, #idx);));

        let dma = format_ident!("{}", ch.dma);
        let ch_num = ch.channel as usize;

        let dma_peri = METADATA.peripherals.iter().find(|p| p.name == ch.dma).unwrap();
        let bi = dma_peri.registers.as_ref().unwrap();

        let dma_info = match bi.kind {
            "dma" => quote!(crate::dma::DmaInfo::Dma(crate::pac::#dma)),
            "bdma" => quote!(crate::dma::DmaInfo::Bdma(crate::pac::#dma)),
            "gpdma" => quote!(crate::pac::#dma),
            _ => panic!("bad dma channel kind {}", bi.kind),
        };

        let dmamux = quote!();

        dmas.extend(quote! {
            crate::dma::ChannelInfo {
                dma: #dma_info,
                num: #ch_num,
                #dmamux
            },
        });
    }

    // ========
    // Generate DMA IRQs.

    let mut dma_irqs: BTreeMap<&str, Vec<String>> = BTreeMap::new();

    for p in METADATA.peripherals {
        if let Some(r) = &p.registers {
            if r.kind == "dma" {
                for irq in p.interrupts {
                    let ch_name = format!("{}_{}", p.name, irq.signal);
                    // let ch = METADATA.dma_channels.iter().find(|c| c.name == ch_name).unwrap();

                    dma_irqs.entry(irq.interrupt).or_default().push(ch_name);
                }
            }
        }
    }

    let dma_irqs: TokenStream = dma_irqs
        .iter()
        .map(|(irq, channels)| {
            let irq = format_ident!("{}", irq);

            let channels = channels.iter().map(|c| format_ident!("{}", c));

            quote! {
                #[cfg(feature = "rt")]
                #[qingke_rt::interrupt]
                unsafe fn #irq () {
                    #(
                        <crate::peripherals::#channels as crate::dma::ChannelInterrupt>::on_irq();
                    )*
                }
            }
        })
        .collect();

    g.extend(dma_irqs);

    g.extend(quote! {
        pub(crate) const DMA_CHANNELS: &[crate::dma::ChannelInfo] = &[#dmas];
    });

    // ...

    // _macros.rs
    let mut m = String::new();

    make_table(&mut m, "foreach_peripheral", &peripherals_table);
    // name, kind, block, signal, interrupt
    make_table(&mut m, "foreach_interrupt", &interrupts_table);
    // pin, port, exti
    make_table(&mut m, "foreach_pin", &pins_table);

    // ========
    // Write generated.rs

    let out_file = out_dir.join("_generated.rs").to_string_lossy().to_string();
    fs::write(out_file, g.to_string()).unwrap();

    let out_file = out_dir.join("_macros.rs").to_string_lossy().to_string();
    fs::write(out_file, m).unwrap();

    // =======
    // Write memory.x

    // Put the linker script somewhere the linker can find it.
    //fs::write(out_dir.join("memory.x"), include_bytes!("memory.x")).unwrap();
    //println!("cargo:rustc-link-search={}", out_dir.display());
    //println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rerun-if-changed=build.rs");
}

enum GetOneError {
    None,
    Multiple,
}

trait IteratorExt: Iterator {
    fn get_one(self) -> Result<Self::Item, GetOneError>;
}

impl<T: Iterator> IteratorExt for T {
    fn get_one(mut self) -> Result<Self::Item, GetOneError> {
        match self.next() {
            None => Err(GetOneError::None),
            Some(res) => match self.next() {
                Some(_) => Err(GetOneError::Multiple),
                None => Ok(res),
            },
        }
    }
}

fn make_table(out: &mut String, name: &str, data: &Vec<Vec<String>>) {
    write!(
        out,
        "#[allow(unused)]
macro_rules! {} {{
    ($($pat:tt => $code:tt;)*) => {{
        macro_rules! __{}_inner {{
            $(($pat) => $code;)*
            ($_:tt) => {{}}
        }}
",
        name, name
    )
    .unwrap();

    for row in data {
        writeln!(out, "        __{}_inner!(({}));", name, row.join(",")).unwrap();
    }

    write!(
        out,
        "    }};
}}"
    )
    .unwrap();
}
