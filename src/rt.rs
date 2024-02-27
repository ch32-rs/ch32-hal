use core::arch::{asm, global_asm};

use qingke::register::gintenr;
use qingke::register::mtvec::{self, TrapMode};
use qingke::riscv::register::mcause;

extern "C" {
    fn InstructionMisaligned();
    fn InstructionFault();
    fn IllegalInstruction();
    fn Breakpoint();
    fn LoadMisaligned();
    fn LoadFault();
    fn StoreMisaligned();
    fn StoreFault();
    fn UserEnvCall();
    fn MachineEnvCall();
}

#[doc(hidden)]
#[no_mangle]
pub static __EXCEPTIONS: [Option<unsafe extern "C" fn(&TrapFrame)>; 12] = [
    Some(InstructionMisaligned),
    Some(InstructionFault),
    Some(IllegalInstruction),
    Some(Breakpoint),
    Some(LoadMisaligned),
    Some(LoadFault),
    Some(StoreMisaligned),
    Some(StoreFault),
    Some(UserEnvCall),
    None,
    None,
    Some(MachineEnvCall),
];

extern "C" {
    fn NonMaskableInt();
    fn Software();
}

// Core interrupts
#[doc(hidden)]
#[no_mangle]
pub static __CORE_INTERRUPTS: [Option<unsafe extern "C" fn()>; 16] = [
    None, // __start
    None,
    Some(NonMaskableInt), // 2
    None,
    None,
    None,
    None,
    None,
    None,
    Some(Breakpoint),
    None,
    None,
    None,
    None,
    Some(Software), // 14
    None,
];

extern "C" {
    fn SYS_TICK();
    fn WWDG();            /* Window Watchdog */
    fn PVD();             /* PVD through EXTI Line detect */
    fn TAMPER();          /* TAMPER */
    fn RTC();             /* RTC */
    fn FLASH();           /* Flash */
    fn RCC();             /* RCC */
    fn EXTI0();           /* EXTI Line 0 */
    fn EXTI1();           /* EXTI Line 1 */
    fn EXTI2();           /* EXTI Line 2 */
    fn EXTI3();           /* EXTI Line 3 */
    fn EXTI4();           /* EXTI Line 4 */
    fn DMA1_CHANNEL1();   /* DMA1 CHANNEL 1 */
    fn DMA1_CHANNEL2();   /* DMA1 CHANNEL 2 */
    fn DMA1_CHANNEL3();   /* DMA1 CHANNEL 3 */
    fn DMA1_CHANNEL4();   /* DMA1 CHANNEL 4 */
    fn DMA1_CHANNEL5();   /* DMA1 CHANNEL 5 */
    fn DMA1_CHANNEL6();   /* DMA1 CHANNEL 6 */
    fn DMA1_CHANNEL7();   /* DMA1 CHANNEL 7 */
    fn ADC1_2();          /* ADC1_2 */
    fn USB_HP_CAN1_TX();  /* USB HP and CAN1 TX */
    fn USB_LP_CAN1_RX0(); /* USB LP and CAN1RX0 */
    fn CAN1_RX1();        /* CAN1 RX1 */
    fn CAN1_SCE();        /* CAN1 SCE */
    fn EXTI9_5();         /* EXTI Line 9..5 */
    fn TIM1_BRK();        /* TIM1 Break */
    fn TIM1_UP();         /* TIM1 Update */
    fn TIM1_TRG_COM();    /* TIM1 Trigger and Commutation */
    fn TIM1_CC();         /* TIM1 Capture Compare */
    fn TIM2();            /* TIM2 */
    fn TIM3();            /* TIM3 */
    fn TIM4();            /* TIM4 */
    fn I2C1_EV();         /* I2C1 Event */
    fn I2C1_ER();         /* I2C1 Error */
    fn I2C2_EV();         /* I2C2 Event */
    fn I2C2_ER();         /* I2C2 Error */
    fn SPI1();            /* SPI1 */
    fn SPI2();            /* SPI2 */
    fn USART1();          /* USART1 */
    fn USART2();          /* USART2 */
    fn USART3();          /* USART3 */
    fn EXTI15_10();       /* EXTI Line 15..10 */
    fn RTCAlarm();        /* RTC Alarm through EXTI Line */
    fn USBWakeUp();       /* USB Wakeup from suspend */
    fn TIM8_BRK();        /* TIM8 Break */
    fn TIM8_UP();         /* TIM8 Update */
    fn TIM8_TRG_COM();    /* TIM8 Trigger and Commutation */
    fn TIM8_CC();         /* TIM8 Capture Compare */
    fn RNG();             /* RNG */
    fn FSMC();            /* FSMC */
    fn SDIO();            /* SDIO */
    fn TIM5();            /* TIM5 */
    fn SPI3();            /* SPI3 */
    fn UART4();           /* UART4 */
    fn UART5();           /* UART5 */
    fn TIM6();            /* TIM6 */
    fn TIM7();            /* TIM7 */
    fn DMA2_CHANNEL1();   /* DMA2 CHANNEL 1 */
    fn DMA2_CHANNEL2();   /* DMA2 CHANNEL 2 */
    fn DMA2_CHANNEL3();   /* DMA2 CHANNEL 3 */
    fn DMA2_CHANNEL4();   /* DMA2 CHANNEL 4 */
    fn DMA2_CHANNEL5();   /* DMA2 CHANNEL 5 */
    fn ETH();             /* ETH */
    fn ETH_WKUP();        /* ETH WakeUp */
    fn CAN2_TX();         /* CAN2 TX */
    fn CAN2_RX0();        /* CAN2 RX0 */
    fn CAN2_RX1();        /* CAN2 RX1 */
    fn CAN2_SCE();        /* CAN2 SCE */
    fn OTG_FS();          /* OTGFS */
    fn USBHSWakeup();     /* USBHS Wakeup */
    fn USBHS();           /* USBHS */
    fn DVP();             /* DVP */
    fn UART6();           /* UART6 */
    fn UART7();           /* UART7 */
    fn UART8();           /* UART8 */
    fn TIM9_BRK();        /* TIM9 Break */
    fn TIM9_UP();         /* TIM9 Update */
    fn TIM9_TRG_COM();    /* TIM9 Trigger and Commutation */
    fn TIM9_CC();         /* TIM9 Capture Compare */
    fn TIM10_BRK();       /* TIM10 Break */
    fn TIM10_UP();        /* TIM10 Update */
    fn TIM10_TRG_COM();   /* TIM10 Trigger and Commutation */
    fn TIM10_CC();        /* TIM10 Capture Compare */
    fn DMA2_CHANNEL6();   /* DMA2 CHANNEL 6 */
    fn DMA2_CHANNEL7();   /* DMA2 CHANNEL 7 */
    fn DMA2_CHANNEL8();   /* DMA2 CHANNEL 8 */
    fn DMA2_CHANNEL9();   /* DMA2 CHANNEL 9 */
    fn DMA2_CHANNEL10();  /* DMA2 CHANNEL 10 */
    fn DMA2_CHANNEL11();  /* DMA2 CHANNEL 11 */
}

#[doc(hidden)]
#[no_mangle]
pub static __EXTERNAL_INTERRUPTS: [Option<unsafe extern "C" fn()>; 101] = [
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None, // Some(SYS_TICK),
    None,
    None,
    None,
    Some(WWDG),            /* Window Watchdog */
    Some(PVD),             /* PVD through EXTI Line detect */
    Some(TAMPER),          /* TAMPER */
    Some(RTC),             /* RTC */
    Some(FLASH),           /* Flash */
    Some(RCC),             /* RCC */
    Some(EXTI0),           /* EXTI Line 0 */
    Some(EXTI1),           /* EXTI Line 1 */
    Some(EXTI2),           /* EXTI Line 2 */
    Some(EXTI3),           /* EXTI Line 3 */
    Some(EXTI4),           /* EXTI Line 4 */
    Some(DMA1_CHANNEL1),   /* DMA1 CHANNEL 1 */
    Some(DMA1_CHANNEL2),   /* DMA1 CHANNEL 2 */
    Some(DMA1_CHANNEL3),   /* DMA1 CHANNEL 3 */
    Some(DMA1_CHANNEL4),   /* DMA1 CHANNEL 4 */
    Some(DMA1_CHANNEL5),   /* DMA1 CHANNEL 5 */
    Some(DMA1_CHANNEL6),   /* DMA1 CHANNEL 6 */
    Some(DMA1_CHANNEL7),   /* DMA1 CHANNEL 7 */
    Some(ADC1_2),          /* ADC1_2 */
    Some(USB_HP_CAN1_TX),  /* USB HP and CAN1 TX */
    Some(USB_LP_CAN1_RX0), /* USB LP and CAN1RX0 */
    Some(CAN1_RX1),        /* CAN1 RX1 */
    Some(CAN1_SCE),        /* CAN1 SCE */
    Some(EXTI9_5),         /* EXTI Line 9..5 */
    Some(TIM1_BRK),        /* TIM1 Break */
    Some(TIM1_UP),         /* TIM1 Update */
    Some(TIM1_TRG_COM),    /* TIM1 Trigger and Commutation */
    Some(TIM1_CC),         /* TIM1 Capture Compare */
    Some(TIM2),            /* TIM2 */
    Some(TIM3),            /* TIM3 */
    Some(TIM4),            /* TIM4 */
    Some(I2C1_EV),         /* I2C1 Event */
    Some(I2C1_ER),         /* I2C1 Error */
    Some(I2C2_EV),         /* I2C2 Event */
    Some(I2C2_ER),         /* I2C2 Error */
    Some(SPI1),            /* SPI1 */
    Some(SPI2),            /* SPI2 */
    Some(USART1),          /* USART1 */
    Some(USART2),          /* USART2 */
    Some(USART3),          /* USART3 */
    Some(EXTI15_10),       /* EXTI Line 15..10 */
    Some(RTCAlarm),        /* RTC Alarm through EXTI Line */
    Some(USBWakeUp),       /* USB Wakeup from suspend */
    Some(TIM8_BRK),        /* TIM8 Break */
    Some(TIM8_UP),         /* TIM8 Update */
    Some(TIM8_TRG_COM),    /* TIM8 Trigger and Commutation */
    Some(TIM8_CC),         /* TIM8 Capture Compare */
    Some(RNG),             /* RNG */
    Some(FSMC),            /* FSMC */
    Some(SDIO),            /* SDIO */
    Some(TIM5),            /* TIM5 */
    Some(SPI3),            /* SPI3 */
    Some(UART4),           /* UART4 */
    Some(UART5),           /* UART5 */
    Some(TIM6),            /* TIM6 */
    Some(TIM7),            /* TIM7 */
    Some(DMA2_CHANNEL1),   /* DMA2 CHANNEL 1 */
    Some(DMA2_CHANNEL2),   /* DMA2 CHANNEL 2 */
    Some(DMA2_CHANNEL3),   /* DMA2 CHANNEL 3 */
    Some(DMA2_CHANNEL4),   /* DMA2 CHANNEL 4 */
    Some(DMA2_CHANNEL5),   /* DMA2 CHANNEL 5 */
    Some(ETH),             /* ETH */
    Some(ETH_WKUP),        /* ETH WakeUp */
    Some(CAN2_TX),         /* CAN2 TX */
    Some(CAN2_RX0),        /* CAN2 RX0 */
    Some(CAN2_RX1),        /* CAN2 RX1 */
    Some(CAN2_SCE),        /* CAN2 SCE */
    Some(OTG_FS),          /* OTGFS */
    Some(USBHSWakeup),     /* USBHS Wakeup */
    Some(USBHS),           /* USBHS */
    Some(DVP),             /* DVP */
    Some(UART6),           /* UART6 */
    Some(UART7),           /* UART7 */
    Some(UART8),           /* UART8 */
    Some(TIM9_BRK),        /* TIM9 Break */
    Some(TIM9_UP),         /* TIM9 Update */
    Some(TIM9_TRG_COM),    /* TIM9 Trigger and Commutation */
    Some(TIM9_CC),         /* TIM9 Capture Compare */
    Some(TIM10_BRK),       /* TIM10 Break */
    Some(TIM10_UP),        /* TIM10 Update */
    Some(TIM10_TRG_COM),   /* TIM10 Trigger and Commutation */
    Some(TIM10_CC),        /* TIM10 Capture Compare */
    Some(DMA2_CHANNEL6),   /* DMA2 CHANNEL 6 */
    Some(DMA2_CHANNEL7),   /* DMA2 CHANNEL 7 */
    Some(DMA2_CHANNEL8),   /* DMA2 CHANNEL 8 */
    Some(DMA2_CHANNEL9),   /* DMA2 CHANNEL 9 */
    Some(DMA2_CHANNEL10),  /* DMA2 CHANNEL 10 */
    Some(DMA2_CHANNEL11),  /* DMA2 CHANNEL 11 */
];

#[link_section = ".trap.rust"]
#[export_name = "_ch32v0_star_trap_rust"]
pub unsafe extern "C" fn start_trap_rust(trap_frame: *const TrapFrame) {
    extern "C" {
        fn ExceptionHandler();
        fn DefaultHandler();
    }

    let cause = mcause::read();
    let code = cause.code();

    if cause.is_exception() {
        let trap_frame = &*trap_frame;
        if code < __EXCEPTIONS.len() {
            let h = &__EXCEPTIONS[code];
            if let Some(handler) = h {
                handler(trap_frame);
            } else {
                ExceptionHandler(trap_frame);
            }
        } else {
            ExceptionHandler(trap_frame);
        }
        ExceptionHandler(trap_frame)
    } else if code < __CORE_INTERRUPTS.len() {
        let h = &__CORE_INTERRUPTS[code];
        if let Some(handler) = h {
            handler();
        } else {
            DefaultHandler();
        }
    } else if code < __EXTERNAL_INTERRUPTS.len() {
        let h = &__EXTERNAL_INTERRUPTS[code];
        if let Some(handler) = h {
            handler();
        } else {
            DefaultHandler();
        }
    } else {
        DefaultHandler();
    }
}

// override _start_trap in riscv-rt
global_asm!(
    r#"
        .section .trap, "ax"
        .global _start_trap
    _start_trap:
        addi sp, sp, -4
        sw ra, 0(sp)
        jal _ch32x0_star_trap_rust
        lw ra, 0(sp)
        addi sp, sp, 4
        mret
    "#
);

#[no_mangle]
#[export_name = "_setup_interrupts"]
unsafe fn ch32v3_setup_interrupts() {
    extern "C" {
        fn _start_trap();
    }

    crate::debug::SDIPrint::enable();
    // enable hardware stack push
    asm!(
        "
        li t0, 0x1f
        csrw 0xbc0, t0
        li t0, 0x3
        csrw 0x804, t0
        "
    );
    mtvec::write(_start_trap as usize, TrapMode::Direct);
    gintenr::set_enable();
}
