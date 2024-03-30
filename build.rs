use std::collections::HashMap;
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
        .filter(|x| x.starts_with("CARGO_FEATURE_CH32"))
        .get_one()
    {
        Ok(x) => x,
        Err(GetOneError::None) => panic!("No ch32xx Cargo feature enabled"),
        Err(GetOneError::Multiple) => panic!("Multiple ch32xx Cargo features enabled"),
    }
    .strip_prefix("CARGO_FEATURE_")
    .unwrap()
    .to_ascii_lowercase();

    // Add family cfg flags on the fly
    let chip_family = if chip_name.starts_with("ch32") {
        // On of ch32x0, ch32v0, ch32v1, ch32v2, ch32v3, ch32l1
        chip_name[..6].to_string()
    } else {
        // On of ch643, ch641
        chip_name[..4].to_string()
    };
    println!("cargo:rustc-cfg={}", chip_family);

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
                //"dbgmcu" => {}
                //"syscfg" => {}
                //"dma" => {}
                //"bdma" => {}
                //"dmamux" => {}

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

            g.extend(quote! {
                impl crate::peripheral::sealed::RccPeripheral for peripherals::#pname {
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
                impl crate::peripheral::sealed::RemapPeripheral for peripherals::#pname {
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
    ]
    .into();

    for p in METADATA.peripherals {
        if let Some(regs) = &p.registers {
            for pin in p.pins {
                let key = (regs.kind, pin.signal);
                if let Some(tr) = signals.get(&key) {
                    let peri = format_ident!("{}", p.name);
                    let pin_name = format_ident!("{}", pin.pin);

                    let remap = pin.remap.unwrap_or(0);

                    g.extend(quote! {
                        pin_trait_impl!(#tr, #peri, #pin_name, #remap);
                    });

                    // panic!("{} {}", peri, pin_name);
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

            let row = vec![regs.kind.to_string(), p.name.to_string()];
            peripherals_table.push(row);
        }
    }

    // _macros.rs
    let mut m = String::new();

    make_table(&mut m, "foreach_peripheral", &peripherals_table);
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
