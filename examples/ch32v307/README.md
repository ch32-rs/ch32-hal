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

## Target

[`riscv32imfc-unknown-none-elf.json`](riscv32imfc-unknown-none-elf.json) enables `xwchc`, WCH's compressed byte and halfword loads and stores. QingKe V4F implements the extension (QingKe V3 manual, table 1-1).

## USBHS isochronous loopback

`src/bin/usb_hs_iso.rs` is a vendor-class device with one interface:

- alt setting 0: idle, no endpoints
- alt setting 1: isochronous OUT + IN, 64-byte packets, one packet per microframe at high speed

OUT packets are drained by a task (`read()`, 16-slot ring in the interrupt); IN packets are
built in the SOF callback (`iso_in_fill()`) and carry the IN sequence, the last OUT sequence
seen, the microframe number, the OUT packet count and the OUT ring's overrun count. A host
with isochronous transfer support (for example Linux with libusb; macOS does not support
isochronous transfers through libusb) can set alt setting 1 and check that IN sequences never
repeat and that the echoed OUT sequence only moves forward.

## USBHS interrupt endpoint echo

`src/bin/usb_hs_int.rs` is a vendor-class device with an interrupt OUT (0x01) and an
interrupt IN (0x81) endpoint, 64-byte packets. The task reads one OUT packet and writes it
back on the IN endpoint. A host using libusb (macOS or Linux both support interrupt
transfers) can send 64-byte, short and zero-length packets and compare the echo; this
exercises the interrupt arm/ready path, NAK flow control and the hardware toggle handling.
