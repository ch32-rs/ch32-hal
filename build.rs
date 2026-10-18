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
    println!("cargo:rustc-cfg={}", chip_family); // On of ch32x0, ch32v0, ch32m0, ch32v1, ch32v2, ch32v3, ch32l1, ch643, ch641

    // Add Qingke IP core version cfg flags on the fly
    // qingke_v2, qingke_v3, qingke_v4
    let qingke_ver = match &*chip_family {
        "ch32v0" | "ch32m0" | "ch641" => "qingke_v2",
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

    // `afio` (any chip with an AFIO peripheral) is already emitted by the
    // `r.kind` loop above. `afio_h4` is emitted by the `{kind}_{version}`
    // line whenever the chip's AFIO is `version: h4` — that's the gate that
    // selects the AFR-style per-pin mux path (vs the central PCFR remap path
    // used by every other ch32 AFIO version). Declare both so rustc doesn't
    // warn `unexpected-cfg` on V0/V1/V2/V3/X0/L1/641/643 builds where
    // `afio_h4` is intentionally unset.
    println!("cargo:rustc-check-cfg=cfg(afio)");
    println!("cargo:rustc-check-cfg=cfg(afio_h4)");

    let mut gpio_lines = 16;
    match &*chip_family {
        "ch32v0" | "ch32m0" => {
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
    //
    // One row per (kind, signal). The whole table is built unconditionally and
    // the entries whose driver module this chip does not compile are removed
    // afterwards, so the list of modules is written once instead of wrapping
    // half the table in `if`.
    let h4 = chip_family == "ch32h4";
    let mut pin_signals: HashMap<(&str, &str), TokenStream> = [
        // Compiled on every family, H4 included: `pub mod usart` is ungated
        // (H4's block is `usart_h4`), and the timer and I2C drivers were brought
        // back onto H4 once their register layouts checked out (`i2c` is
        // `version: v3` there).
        (("usart", "TX"), quote!(crate::usart::TxPin)),
        (("usart", "RX"), quote!(crate::usart::RxPin)),
        (("usart", "CTS"), quote!(crate::usart::CtsPin)),
        (("usart", "RTS"), quote!(crate::usart::RtsPin)),
        (("usart", "CK"), quote!(crate::usart::CkPin)),
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
        // I2C on CH32H4 is `version: v3` (same register layout as V2/V3), so
        // the V3 driver applies as-is — keep its pin trait impls.
        (("i2c", "SDA"), quote!(crate::i2c::SdaPin)),
        (("i2c", "SCL"), quote!(crate::i2c::SclPin)),
        // No H4 peripheral has these kinds, so the rows match nothing there and
        // the modules they name are never pulled in. The day one grows an
        // `sdio` block, the module gate in lib.rs has to grow with it.
        (("sdio", "CK"), quote!(crate::sdio::CkPin)),
        (("sdio", "CMD"), quote!(crate::sdio::CmdPin)),
        (("sdio", "D0"), quote!(crate::sdio::D0Pin)),
        (("sdio", "D1"), quote!(crate::sdio::D1Pin)),
        (("sdio", "D2"), quote!(crate::sdio::D2Pin)),
        (("sdio", "D3"), quote!(crate::sdio::D3Pin)),
        (("sdio", "D4"), quote!(crate::sdio::D4Pin)),
        (("sdio", "D5"), quote!(crate::sdio::D5Pin)),
        (("sdio", "D6"), quote!(crate::sdio::D6Pin)),
        (("sdio", "D7"), quote!(crate::sdio::D7Pin)),
        (("sdio", "D8"), quote!(crate::sdio::D8Pin)),
        (("otg", "DP"), quote!(crate::otg_fs::DpPin)),
        (("otg", "DM"), quote!(crate::otg_fs::DmPin)),
        (("usbd", "DP"), quote!(crate::usbd::DpPin)),
        (("usbd", "DM"), quote!(crate::usbd::DmPin)),
        (("spi", "MISO"), quote!(crate::spi::MisoPin)),
        (("spi", "SCK"), quote!(crate::spi::SckPin)),
        (("spi", "MOSI"), quote!(crate::spi::MosiPin)),
        (("spi", "NSS"), quote!(crate::spi::CsPin)),
        (("can", "TX"), quote!(crate::can::TxPin)),
        (("can", "RX"), quote!(crate::can::RxPin)),
        // H4 does have USBHS pins, but its block is `usbhs_h4` while the module
        // is `#[cfg(usbhs_v3)]`, so the rows are removed below on that family.
        (("usbhs", "DP"), quote!(crate::usbhs::DpPin)),
        (("usbhs", "DM"), quote!(crate::usbhs::DmPin)),
    ]
    .into();

    // Modules `lib.rs` compiles out on CH32H4 while the chip still has pins of
    // that kind, so a row naming one would not resolve. SPI and CAN are
    // `version: h4` and those drivers reach into V3 register fields; USBHS is
    // `usbhs_h4` while the module is `#[cfg(usbhs_v3)]`. ADC and DAC are not in
    // this table: their pin macros live inside the gated modules and are
    // emitted below.
    if h4 {
        pin_signals.retain(|key, _| !matches!(key.0, "spi" | "can" | "usbhs"));
    }

    for p in METADATA.peripherals {
        if let Some(regs) = &p.registers {
            for pin in p.pins {
                let key = (regs.kind, pin.signal);

                if let Some(tr) = pin_signals.get(&key) {
                    let peri = format_ident!("{}", p.name);
                    let pin_name = format_ident!("{}", pin.pin);
                    // H4 pins carry `af` (AFR index). Every other family carries
                    // `remap` (PCFR group). Emit each (signal, pin, value) on
                    // its own — do not collapse a pin's other signals.
                    let n = if h4 {
                        pin.af.unwrap_or(0)
                    } else {
                        pin.remap.unwrap_or(0)
                    };

                    g.extend(quote! {
                        pin_trait_impl!(#tr, #peri, #pin_name, #n);
                    });
                }

                // ADC / DAC pin impls reference the `impl_adc_pin!` /
                // `impl_dac_pin!` macros defined inside src/adc.rs and
                // src/dac.rs. Those modules are `not(ch32h4)` (their V3-style
                // register layout doesn't match H4) so the macros aren't in
                // scope — skip the emission too.
                if regs.kind == "adc" && !h4 {
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

                if regs.kind == "dac" && !h4 {
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

    let mut dma_signals: HashMap<_, _> = [
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
        (("dac", "CH1"), quote!(crate::dac::DacDma1)),
        (("dac", "CH2"), quote!(crate::dac::DacDma2)),
    ]
    .into();

    // Same rule as the pin table: drop the rows whose driver module `lib.rs`
    // compiles out on CH32H4. `sdio` stays — no H4 peripheral has that kind, so
    // the row matches nothing and the module is never named.
    if h4 {
        dma_signals.retain(|key, _| !matches!(key.0, "spi" | "dac"));
    }

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

                if let Some(tr) = dma_signals.get(&(regs.kind, ch.signal)) {
                    let peri = format_ident!("{}", p.name);

                    let (channel, request) = if let Some(channel) = &ch.channel {
                        // Chip with DMA/BDMA, without DMAMUX: the channel is
                        // hard-wired to the signal, so the request is empty.
                        let channel = format_ident!("{}", channel);
                        (quote!({channel: #channel}), quote!(()))
                    } else if ch.dmamux.is_some() {
                        // CH32H4: any of the 16 mux channels can carry it, and
                        // the request number selects the source.
                        let request = ch.request.expect("DMAMUX signal without a request number") as u8;
                        (quote!({dmamux: DMAMUX}), quote!(#request))
                    } else {
                        panic!("DMA channel for {} {} has neither a channel nor a DMAMUX", p.name, ch.signal);
                    };

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

    // DMA channel table and per-channel interrupt handlers. H4 uses the same
    // `ChannelInfo` as the other families; its `dma_h4` register block is the
    // same layout under different accessor names, which the driver accounts for.
    {
        let mut dmas = TokenStream::new();
        for (ch_idx, ch) in METADATA.dma_channels.iter().enumerate() {
            let name = format_ident!("{}", ch.name);
            let idx = ch_idx as u8;
            g.extend(quote!(dma_channel_impl!(#name, #idx);));

            // H4 routes every channel through the one DMAMUX, so each channel
            // type is a mux channel. The marker only bounds the signal-trait
            // blanket impls; the mux slot is computed from the controller.
            if h4 {
                g.extend(quote! {
                    impl crate::dma::SealedMuxChannel for crate::peripherals::#name {}
                    impl crate::dma::MuxChannel for crate::peripherals::#name {}
                });
            }

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

        let mut dma_irqs: BTreeMap<&str, Vec<String>> = BTreeMap::new();
        for p in METADATA.peripherals {
            if let Some(r) = &p.registers {
                if r.kind == "dma" {
                    for irq in p.interrupts {
                        let ch_name = format!("{}_{}", p.name, irq.signal);
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
    }

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

