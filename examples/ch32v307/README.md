# ch32v307-examples

## Boards

### YD-CH32V307VCT6

8MHz XTAL, 32.768KHz XTAL

- PA15, Red LED, active high
- PB4, Blue LED, active high
- PB3, User button, active low

FT24C32(I2C2 with pull up)

- SCL: PB10
- SDA: PB11

W25Q16(SPI1)

- CS: PA2
- DO(IO1): PA6
- CLK: PA5
- DI(IO0): PA7

Micro SD Card / TFCard (SDIO)

- DAT0: PC8 (requires soldering the jumper on the back of the board)
- DAT1: PC9 (requires soldering the jumper on the back of the board)
- DAT2: PC10
- DAT3: PC11
- CLK: PC12
- CMD: PD2

- TFSW: PD7 (requires soldering the jumper on the back of the board)

Ethernet  (requires soldering the jumper on the back of the board)

- PC0, LINK LED
- PC1, ACT LED
- PC7, RXN
- PC6, RXP
- PC9, TXN
- PC8, TXP
