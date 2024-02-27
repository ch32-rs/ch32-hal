/* CH32V307 */
MEMORY
{
	FLASH : ORIGIN = 0x00000000, LENGTH = 256k
	RAM : ORIGIN = 0x20000000, LENGTH = 64k
}

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);

/* fault handlers */

PROVIDE(InstructionMisaligned = ExceptionHandler);
PROVIDE(InstructionFault = ExceptionHandler);
PROVIDE(IllegalInstruction = ExceptionHandler);
PROVIDE(Breakpoint = ExceptionHandler);
PROVIDE(LoadMisaligned = ExceptionHandler);
PROVIDE(LoadFault = ExceptionHandler);
PROVIDE(StoreMisaligned = ExceptionHandler);
PROVIDE(StoreFault = ExceptionHandler);;
PROVIDE(UserEnvCall = ExceptionHandler);
PROVIDE(SupervisorEnvCall = ExceptionHandler);
PROVIDE(MachineEnvCall = ExceptionHandler);
PROVIDE(InstructionPageFault = ExceptionHandler);
PROVIDE(LoadPageFault = ExceptionHandler);
PROVIDE(StorePageFault = ExceptionHandler);

/* core interrupt handlers */

PROVIDE(NonMaskableInt = DefaultHandler);
PROVIDE(Software = DefaultHandler);

/* external interrupt handlers */

PROVIDE(WWDG = DefaultHandler);
PROVIDE(PVD = DefaultHandler);
PROVIDE(TAMPER = DefaultHandler);
PROVIDE(RTC = DefaultHandler);
PROVIDE(FLASH = DefaultHandler);
PROVIDE(RCC = DefaultHandler);
PROVIDE(EXTI0 = DefaultHandler);
PROVIDE(EXTI1 = DefaultHandler);
PROVIDE(EXTI2 = DefaultHandler);
PROVIDE(EXTI3 = DefaultHandler);
PROVIDE(EXTI4 = DefaultHandler);
PROVIDE(DMA1_CHANNEL1 = DefaultHandler);
PROVIDE(DMA1_CHANNEL2 = DefaultHandler);
PROVIDE(DMA1_CHANNEL3 = DefaultHandler);
PROVIDE(DMA1_CHANNEL4 = DefaultHandler);
PROVIDE(DMA1_CHANNEL5 = DefaultHandler);
PROVIDE(DMA1_CHANNEL6 = DefaultHandler);
PROVIDE(DMA1_CHANNEL7 = DefaultHandler);
PROVIDE(ADC = DefaultHandler);
PROVIDE(USB_HP_CAN1_TX = DefaultHandler);
PROVIDE(USB_LP_CAN1_RX0 = DefaultHandler);
PROVIDE(CAN1_RX1 = DefaultHandler);
PROVIDE(CAN1_SCE = DefaultHandler);
PROVIDE(EXTI9_5 = DefaultHandler);
PROVIDE(TIM1_BRK = DefaultHandler);
PROVIDE(TIM1_UP_ = DefaultHandler);
PROVIDE(TIM1_TRG_COM = DefaultHandler);
PROVIDE(TIM1_CC = DefaultHandler);
PROVIDE(TIM2 = DefaultHandler);
PROVIDE(TIM3 = DefaultHandler);
PROVIDE(TIM4 = DefaultHandler);
PROVIDE(I2C1_EV = DefaultHandler);
PROVIDE(I2C1_ER = DefaultHandler);
PROVIDE(I2C2_EV = DefaultHandler);
PROVIDE(I2C2_ER = DefaultHandler);
PROVIDE(SPI1 = DefaultHandler);
PROVIDE(SPI2 = DefaultHandler);
PROVIDE(USART1 = DefaultHandler);
PROVIDE(USART2 = DefaultHandler);
PROVIDE(USART3 = DefaultHandler);
PROVIDE(EXTI15_10 = DefaultHandler);
PROVIDE(RTCALARM = DefaultHandler);
PROVIDE(USBWAKE_UP = DefaultHandler);
PROVIDE(TIM8_BRK = DefaultHandler);
PROVIDE(TIM8_UP_ = DefaultHandler);
PROVIDE(TIM8_TRG_COM = DefaultHandler);
PROVIDE(TIM8_CC = DefaultHandler);
PROVIDE(RNG = DefaultHandler);
PROVIDE(FSMC = DefaultHandler);
PROVIDE(SDIO = DefaultHandler);
PROVIDE(TIM5 = DefaultHandler);
PROVIDE(SPI3 = DefaultHandler);
PROVIDE(UART4 = DefaultHandler);
PROVIDE(UART5 = DefaultHandler);
PROVIDE(TIM6 = DefaultHandler);
PROVIDE(TIM7 = DefaultHandler);
PROVIDE(DMA2_CHANNEL1 = DefaultHandler);
PROVIDE(DMA2_CHANNEL2 = DefaultHandler);
PROVIDE(DMA2_CHANNEL3 = DefaultHandler);
PROVIDE(DMA2_CHANNEL4 = DefaultHandler);
PROVIDE(DMA2_CHANNEL5 = DefaultHandler);
PROVIDE(ETH = DefaultHandler);
PROVIDE(ETH_WKUP = DefaultHandler);
PROVIDE(CAN2_TX = DefaultHandler);
PROVIDE(CAN2_RX0 = DefaultHandler);
PROVIDE(CAN2_RX1 = DefaultHandler);
PROVIDE(CAN2_SCE = DefaultHandler);
PROVIDE(OTG_FS = DefaultHandler);
PROVIDE(USBHSWAKEUP = DefaultHandler);
PROVIDE(USBHS = DefaultHandler);
PROVIDE(DVP = DefaultHandler);
PROVIDE(UART6 = DefaultHandler);
PROVIDE(UART7 = DefaultHandler);
PROVIDE(UART8 = DefaultHandler);
PROVIDE(TIM9_BRK = DefaultHandler);
PROVIDE(TIM9_UP_ = DefaultHandler);
PROVIDE(TIM9_TRG_COM = DefaultHandler);
PROVIDE(TIM9_CC = DefaultHandler);
PROVIDE(TIM10_BRK = DefaultHandler);
PROVIDE(TIM10_UP_ = DefaultHandler);
PROVIDE(TIM10_TRG_COM = DefaultHandler);
PROVIDE(TIM10_CC = DefaultHandler);
PROVIDE(DMA2_CHANNEL6 = DefaultHandler);
PROVIDE(DMA2_CHANNEL7 = DefaultHandler);
PROVIDE(DMA2_CHANNEL8 = DefaultHandler);
PROVIDE(DMA2_CHANNEL9 = DefaultHandler);
PROVIDE(DMA2_CHANNEL10 = DefaultHandler);
PROVIDE(DMA2_CHANNEL11 = DefaultHandler);