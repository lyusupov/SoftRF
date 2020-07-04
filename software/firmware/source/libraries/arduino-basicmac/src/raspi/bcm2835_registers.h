/*
  bcm2835_registers.h - provides AVR-like acces to the peripheral hardware registers
  Copyright (c) 2015 Hristo Gochkov.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef BCM2835_REGISTERS_H
#define BCM2835_REGISTERS_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>

/*
00000000-1bffffff  System RAM
00008000-005727b3  Kernel code
00598000-0067c3df  Kernel data
20000000-20000fff  bcm2708_vcio
20003000-20003fff  bcm2708_systemtimer
20006000-20006fff  bcm2708_usb
20007000-20007fff  bcm2708_dma
20100000-201000ff  bcm2708_powerman.0
20101098-2010109a  bcm2708-i2s.0
20200000-20200fff  bcm2708_gpio  *
20201000-20201fff  uart-pl011    *
20203000-20203020  bcm2708-i2s.0
20204000-202040ff  bcm2708_spi.0 *
20205000-202050ff  bcm2708_i2c.0 *
20300000-203000ff  bcm2708_sdhci.0
20300000-203000ff  mmc0
20804000-208040ff  bcm2708_i2c.1 *
20980000-2099ffff  bcm2708_usb
*/

//memory locations for all peripherals

#define PI1_BCMBASE 0x20000000
#define PI2_BCMBASE 0x3f000000

#define BCM2835_BASE PI1_BCMBASE

#define BCM2835_ST_BASE     (BCM2835_BASE | 0x00003000) /* System Timer */
#define BCM2835_IRQ_BASE    (BCM2835_BASE | 0x0000B000) /* Interrupt control block */
#define BCM2835_PM_BASE     (BCM2835_BASE | 0x00100000) /* Power Management, Reset controller and Watchdog registers */
#define BCM2835_CM_BASE     (BCM2835_BASE | 0x00101000) /* GPIO Clock Registers */
#define BCM2835_GPIO_BASE   (BCM2835_BASE | 0x00200000) /* GPIO */
#define BCM2835_UART0_BASE  (BCM2835_BASE | 0x00201000) /* UART0 */
#define BCM2835_PCM_BASE    (BCM2835_BASE | 0x00203000) /* I2S */
#define BCM2835_SPI0_BASE   (BCM2835_BASE | 0x00204000) /* SPI0 */
#define BCM2835_BSC0_BASE   (BCM2835_BASE | 0x00205000) /* BSC0 I2C/TWI (I2C0)*/
#define BCM2835_PWM_BASE    (BCM2835_BASE | 0x0020C000) /* PWM Registers */
#define BCM2835_BSCS_BASE   (BCM2835_BASE | 0x00214000) /* BSC/SPI Slave Registers */
#define BCM2835_AUX_BASE    (BCM2835_BASE | 0x00215000) /* AUX Registers (UART1, SPI1, SPI2) */
#define BCM2835_BSC1_BASE   (BCM2835_BASE | 0x00804000) /* BSC1 I2C/TWI (I2C1)*/

//everything below is available after successful init_registers() call

#define BCM2835_CORE_CLK_HZ        250000000  ///< 250 MHz Not true! this is the clock for i2c and spi, uart is 3MHz
#define BCM2835_REG(a, b) *(((volatile uint32_t *)((a) + ((b) / 4))))
#define F_CPU                           (BCM2835_CORE_CLK_HZ)
#define _BV(a) (1 << (a))

// Defines for ST
#define STCS         BCM2835_REG(bcmreg_st, 0x0000) ///< System Timer Control/Status
#define STCLO        BCM2835_REG(bcmreg_st, 0x0004) ///< System Timer Counter Lower 32 bits
#define STCHI        BCM2835_REG(bcmreg_st, 0x0008) ///< System Timer Counter Upper 32 bits
#define STC0         BCM2835_REG(bcmreg_st, 0x000C) ///< System Timer Compare 0
#define STC1         BCM2835_REG(bcmreg_st, 0x0010) ///< System Timer Compare 1
#define STC2         BCM2835_REG(bcmreg_st, 0x0014) ///< System Timer Compare 2
#define STC3         BCM2835_REG(bcmreg_st, 0x0018) ///< System Timer Compare 3

#define STM3          3 //System Timer Match 3
#define STM2          2 //System Timer Match 2
#define STM1          1 //System Timer Match 1
#define STM0          0 //System Timer Match 0

#define STCV      ((uint64_t)(STCLO | ((uint64_t)STCHI << 32)))

// Defines for IRQ
#define IRQPB        BCM2835_REG(bcmreg_irq, 0x0200) ///< IRQ Basic pending
#define IRQP1        BCM2835_REG(bcmreg_irq, 0x0204) ///< IRQ Pending 1
#define IRQP2        BCM2835_REG(bcmreg_irq, 0x0208) ///< IRQ Pending 2
#define FIQ          BCM2835_REG(bcmreg_irq, 0x020C) ///< FIQ Control Register
#define IRQS1        BCM2835_REG(bcmreg_irq, 0x0210) ///< Enable IRQs 1
#define IRQS2        BCM2835_REG(bcmreg_irq, 0x0214) ///< Enable IRQs 2
#define IRQSB        BCM2835_REG(bcmreg_irq, 0x0218) ///< Enable Basic IRQs
#define IRQC1        BCM2835_REG(bcmreg_irq, 0x021C) ///< Disable IRQs 1
#define IRQC2        BCM2835_REG(bcmreg_irq, 0x0220) ///< Disable IRQs 2
#define IRQCB        BCM2835_REG(bcmreg_irq, 0x0224) ///< Disable Basic IRQs

//IRQs Bank 1
#define IRQTIMER0       0
#define IRQTIMER1       1
#define IRQTIMER2       2
#define IRQTIMER3       3
#define IRQAUX          29

//IRQs Bank 2
#define IRQI2CSPISLV    11
#define IRQPWA0         13
#define IRQPWA1         14
#define IRQGPIO0        17
#define IRQGPIO1        18
#define IRQGPIO2        19
#define IRQGPIO3        20
#define IRQVC_I2C       21
#define IRQVC_SPI       22
#define IRQVC_I2SPCM    23
#define IRQVC_UART      25

/// Pad control register offsets from BCM2835_GPIO_PM
#define PADS_0_27       BCM2835_REG(bcmreg_pm, 0x002c) ///< Pad control register for pads 0 to 27
#define PADS_28_45      BCM2835_REG(bcmreg_pm, 0x0030) ///< Pad control register for pads 28 to 45
#define PADS_46_53      BCM2835_REG(bcmreg_pm, 0x0034) ///< Pad control register for pads 46 to 53

/// Pad Control masks
#define PAD_PASSWRD     (0x5A << 24)  ///< Password to enable setting pad mask
#define PAD_SLEWRATEUNL 0x10 ///< Slew rate unlimited
#define PAD_HYS_ENABLED 0x08 ///< Hysteresis enabled
#define PAD_DRIVE_2mA   0x00 ///< 2mA drive current
#define PAD_DRIVE_4mA   0x01 ///< 4mA drive current
#define PAD_DRIVE_6mA   0x02 ///< 6mA drive current
#define PAD_DRIVE_8mA   0x03 ///< 8mA drive current
#define PAD_DRIVE_10mA  0x04 ///< 10mA drive current
#define PAD_DRIVE_12mA  0x05 ///< 12mA drive current
#define PAD_DRIVE_14mA  0x06 ///< 14mA drive current
#define PAD_DRIVE_16mA  0x07 ///< 16mA drive current

// Defines for Clock Manager, word offsets (ie 4 byte multiples)
#define GP0CTL          BCM2835_REG(bcmreg_cm, 0x0070)
#define GP0DIV          BCM2835_REG(bcmreg_cm, 0x0074)
#define GP1CTL          BCM2835_REG(bcmreg_cm, 0x0078)
#define GP1DIV          BCM2835_REG(bcmreg_cm, 0x007C)
#define GP2CTL          BCM2835_REG(bcmreg_cm, 0x0080)
#define GP2DIV          BCM2835_REG(bcmreg_cm, 0x0084)
#define GPPCTL          BCM2835_REG(bcmreg_cm, 0x00A0) //PWM CLK CTRL
#define GPPDIV          BCM2835_REG(bcmreg_cm, 0x00A4) //PWM CLK DIV
#define GPCPASS         (0x5A << 24)

#define GPSRC           0 // 4 bits clock source
#define GPENAB          4 // Enable the clock generator
#define GPKILL          5 // Kill the clock generator
#define GPBUSY          7 // (RO) Clock generator is running
#define GPFLIP          8 // Invert the clock generator output
#define GPMASH          9 // 2 bits MASH control (0-3) integer division/1 stage/2 stage/3 stage MASH

#define GPDIVF          0 // 12 bits fractional part of divisor 0-4095
#define GPDIVI          12 // 12 bits integer part of divisor 0-4095

#define GPSRC_GND       0x00 // 0
#define GPSRC_OSC       0x01 // 19.2 MHz
#define GPSRC_TDBG0     0x02 // 0
#define GPSRC_TDBG1     0x03 // 0
#define GPSRC_PLLA      0x04 // 0
#define GPSRC_PLLC      0x05 // 1000 MHz (changes with overclock settings)
#define GPSRC_PLLD      0x06 // 500 MHz Used by ETH clock
#define GPSRC_HDMI      0x07 // 216 MHz

#define GPF_OSC         19200000
#define GPF_PLLC        1000000000
#define GPF_PLLD        500000000
#define GPF_HDMI        216000000

#define GPDIV2F(c,d)  ((c << 12)/d)
#define GPF2DIV(c,f)  (((c << 12)/f) & 0xFFFFFF)


/// GPIO register offsets from BCM2835_GPIO_BASE. Offsets into the GPIO Peripheral block in bytes per 6.1 Register View
#define GPFSEL0         BCM2835_REG(bcmreg_gpio, 0x0000) ///< GPIO Function Select 0
#define GPFSEL1         BCM2835_REG(bcmreg_gpio, 0x0004) ///< GPIO Function Select 1
#define GPFSEL2         BCM2835_REG(bcmreg_gpio, 0x0008) ///< GPIO Function Select 2
#define GPFSEL3         BCM2835_REG(bcmreg_gpio, 0x000c) ///< GPIO Function Select 3
#define GPFSEL4         BCM2835_REG(bcmreg_gpio, 0x0010) ///< GPIO Function Select 4
#define GPFSEL5         BCM2835_REG(bcmreg_gpio, 0x0014) ///< GPIO Function Select 5
#define GPSET0          BCM2835_REG(bcmreg_gpio, 0x001c) ///< GPIO Pin Output Set 0
#define GPSET1          BCM2835_REG(bcmreg_gpio, 0x0020) ///< GPIO Pin Output Set 1
#define GPCLR0          BCM2835_REG(bcmreg_gpio, 0x0028) ///< GPIO Pin Output Clear 0
#define GPCLR1          BCM2835_REG(bcmreg_gpio, 0x002c) ///< GPIO Pin Output Clear 1
#define GPLEV0          BCM2835_REG(bcmreg_gpio, 0x0034) ///< GPIO Pin Level 0
#define GPLEV1          BCM2835_REG(bcmreg_gpio, 0x0038) ///< GPIO Pin Level 1
#define GPEDS0          BCM2835_REG(bcmreg_gpio, 0x0040) ///< GPIO Pin Event Detect Status 0
#define GPEDS1          BCM2835_REG(bcmreg_gpio, 0x0044) ///< GPIO Pin Event Detect Status 1
#define GPREN0          BCM2835_REG(bcmreg_gpio, 0x004c) ///< GPIO Pin Rising Edge Detect Enable 0
#define GPREN1          BCM2835_REG(bcmreg_gpio, 0x0050) ///< GPIO Pin Rising Edge Detect Enable 1
#define GPFEN0          BCM2835_REG(bcmreg_gpio, 0x0058) ///< GPIO Pin Falling Edge Detect Enable 0
#define GPFEN1          BCM2835_REG(bcmreg_gpio, 0x005c) ///< GPIO Pin Falling Edge Detect Enable 1
#define GPHEN0          BCM2835_REG(bcmreg_gpio, 0x0064) ///< GPIO Pin High Detect Enable 0
#define GPHEN1          BCM2835_REG(bcmreg_gpio, 0x0068) ///< GPIO Pin High Detect Enable 1
#define GPLEN0          BCM2835_REG(bcmreg_gpio, 0x0070) ///< GPIO Pin Low Detect Enable 0
#define GPLEN1          BCM2835_REG(bcmreg_gpio, 0x0074) ///< GPIO Pin Low Detect Enable 1
#define GPAREN0         BCM2835_REG(bcmreg_gpio, 0x007c) ///< GPIO Pin Async. Rising Edge Detect 0
#define GPAREN1         BCM2835_REG(bcmreg_gpio, 0x0080) ///< GPIO Pin Async. Rising Edge Detect 1
#define GPAFEN0         BCM2835_REG(bcmreg_gpio, 0x0088) ///< GPIO Pin Async. Falling Edge Detect 0
#define GPAFEN1         BCM2835_REG(bcmreg_gpio, 0x008c) ///< GPIO Pin Async. Falling Edge Detect 1
#define GPPUD           BCM2835_REG(bcmreg_gpio, 0x0094) ///< GPIO Pin Pull-up/down Enable
#define GPPUDCLK0       BCM2835_REG(bcmreg_gpio, 0x0098) ///< GPIO Pin Pull-up/down Enable Clock 0
#define GPPUDCLK1       BCM2835_REG(bcmreg_gpio, 0x009c) ///< GPIO Pin Pull-up/down Enable Clock 1

#define GPFSEL(p)       BCM2835_REG(bcmreg_gpio + (p/10), 0x0000)
#define GPFSELB(p)      ((p % 10) * 3)
#define GPFSELIN(pin)   do { GPFSEL(pin) &= ~(0x07 << GPFSELB(pin)); GPPUD = 0; } while(0)
#define GPFSELOUT(pin)  do { GPFSEL(pin) = (GPFSEL(pin) & ~(0x07 << GPFSELB(pin))) | (1 << GPFSELB(pin)); } while(0)

#define GPFI        0x00   ///< Input
#define GPFO      0x01   ///< Output
#define GPF0        0x04   ///< Alternate function 0
#define GPF1        0x05   ///< Alternate function 1
#define GPF2        0x06   ///< Alternate function 2
#define GPF3        0x07   ///< Alternate function 3
#define GPF4        0x03   ///< Alternate function 4
#define GPF5        0x02   ///< Alternate function 5

#define GPPO         0x00   ///< Off ? disable pull-up/down
#define GPPD         0x01   ///< Enable Pull Down control
#define GPPU          0x02   ///< Enable Pull Up control

//PWM registers
#define PWMCTL       BCM2835_REG(bcmreg_pwm, 0x0000) // PWM Control
#define PWMSTA       BCM2835_REG(bcmreg_pwm, 0x0004) // PWM Status
#define PWMDMAC      BCM2835_REG(bcmreg_pwm, 0x0008) // PWM DMA Configuration
#define PWMRNG1      BCM2835_REG(bcmreg_pwm, 0x0010) // PWM Channel 1 Range
#define PWMDAT1      BCM2835_REG(bcmreg_pwm, 0x0014) // PWM Channel 1 Data
#define PWMFIF1      BCM2835_REG(bcmreg_pwm, 0x0018) // PWM FIFO Input
#define PWMRNG2      BCM2835_REG(bcmreg_pwm, 0x0020) // PWM Channel 2 Range
#define PWMDAT2      BCM2835_REG(bcmreg_pwm, 0x0024) // PWM Channel 2 Data

//PWMCTL bits
#define PWMMSEN2        15 // Channel 2 M/S Enable
#define PWMUSEF2        13 // Channel 2 Use Fifo
#define PWMPOLA2        12 // Channel 2 Polarity
#define PWMSBIT2        11 // Channel 2 Silence Bit (Defines the state of the output when no transmission takes place)
#define PWMRPTL2        10 // Channel 2 Repeat Last Data (1: Last data in FIFO is transmitted repetedly until FIFO is not empty)
#define PWMMODE2        9  // Channel 2 Mode (0: PWM/ 1: Serial)
#define PWMPWEN2        8  // Channel 2 Enable
#define PWMMSEN1        7  // Channel 1 M/S Enable
#define PWMCLRF1        5  // Clear Fifo
#define PWMUSEF1        5  // Channel 1 Use Fifo
#define PWMPOLA1        4  // Channel 1 Polarity
#define PWMSBIT1        3  // Channel 1 Silence Bit
#define PWMRPTL1        2  // Channel 1 Repeat Last Data
#define PWMMODE1        1  // Channel 1 Mode
#define PWMPWEN1        0  // Channel 1 Enable

//PWMSTA bits
#define PWMSTA4         12 // Channel 4 State
#define PWMSTA3         11 // Channel 3 State
#define PWMSTA2         10 // Channel 2 State
#define PWMSTA1         9 // Channel 1 State
#define PWMBERR         8 // Bus Error Flag
#define PWMGAPO4        7 // Channel 4 Gap Occurred Flag
#define PWMGAPO3        6 // Channel 3 Gap Occurred Flag
#define PWMGAPO2        5 // Channel 2 Gap Occurred Flag
#define PWMGAPO1        4 // Channel 1 Gap Occurred Flag
#define PWMRERR1        3 // Fifo Read Error Flag
#define PWMWERR1        2 // Fifo Write Error Flag
#define PWMEMPT1        1 // Fifo Empty Flag
#define PWMFULL1        0 // Fifo Full Flag

//UART0 register offsets
#define UART0DR        BCM2835_REG(bcmreg_uart0, 0x0000) //Data Register
#define UART0RSRECR      BCM2835_REG(bcmreg_uart0, 0x0004) //receive status/error clear register
#define UART0FR        BCM2835_REG(bcmreg_uart0, 0x0018) //Flag Register
#define UART0IBRD      BCM2835_REG(bcmreg_uart0, 0x0024) //Integer Baud Rate Divisor
#define UART0FBRD      BCM2835_REG(bcmreg_uart0, 0x0028) //Fractional Baud Rate Divisor
#define UART0LCRH      BCM2835_REG(bcmreg_uart0, 0x002C) //Line Control Register
#define UART0CR        BCM2835_REG(bcmreg_uart0, 0x0030) //Control Register
#define UART0IFLS      BCM2835_REG(bcmreg_uart0, 0x0034) //Interupt FIFO Level Select Register
#define UART0IMSC      BCM2835_REG(bcmreg_uart0, 0x0038) //Interupt Mask Set Clear Register
#define UART0RIS      BCM2835_REG(bcmreg_uart0, 0x003C) //Raw Interupt Status Register
#define UART0MIS      BCM2835_REG(bcmreg_uart0, 0x0040) //Masked Interupt Status Register
#define UART0ICR      BCM2835_REG(bcmreg_uart0, 0x0044) //Interupt Clear Register

#define UART0OE        3 //Overrun error
#define UART0BE        2 //Break error
#define UART0PE        1 //Parity error
#define UART0FE        0 //Framing error

#define UART0TXFE       7 //Transmit FIFO empty
#define UART0RXFF       6 //Receive FIFO full
#define UART0TXFF       5 //Transmit FIFO full
#define UART0RXFE       4 //Receive FIFO empty
#define UART0BUSY       3 //UART busy
#define UART0CTS        0 //Clear to send

#define UART0SPS        7 //Stick parity select.
#define UART0WLEN       5 //Word length (2 bits)
#define UART0FEN        4 //Enable FIFOs
#define UART0STP2       3 //Two stop bits select
#define UART0EPS        2 //Even parity select.
#define UART0PEN        1 //Parity enable
#define UART0BRK        0 //Send break

#define UART0WLEN_8BIT  0x03
#define UART0WLEN_7BIT  0x02
#define UART0WLEN_6BIT  0x01
#define UART0WLEN_5BIT  0x00

#define UART0CTSEN      15 //CTS hardware flow control enable
#define UART0RTSEN      14 //RTS hardware flow control enable
#define UART0RTS        11 //Request to send
#define UART0RXE        9  //Receive enable
#define UART0TXE        8  //Transmit enable
#define UART0LBE        7  //Loopback enable
#define UART0EN         0  //UART0 enable

#define UART0RXIFLSEL  3 //Receive interrupt FIFO level select (3 bits)
#define UART0TXIFLSEL  0 //Transmit interrupt FIFO level select (3 bits)

#define UART0IFL_18      0x00 // FIFO interrupt level 1/8
#define UART0IFL_14      0x01 // FIFO interrupt level 1/4
#define UART0IFL_12      0x02 // FIFO interrupt level 1/2
#define UART0IFL_34      0x03 // FIFO interrupt level 3/4
#define UART0IFL_78      0x04 // FIFO interrupt level 7/8

//bits for IMSC, RIS, MIS and ICR registers
#define UART0OEI    10 //Overrun error interrupt
#define UART0BEI    9 //Break error interrupt
#define UART0PEI    8 //Parity error interrupt
#define UART0FEI    7 //Framing error interrupt 
#define UART0RTI    6 //Receive timeout interrupt
#define UART0TXI    5 //Transmit interrupt
#define UART0RXI    4 //Receive interrupt
#define UART0CTSI    1 //CTS interrupt

//BAUDRATE = (3000000/(16 * (DIVI + (DIVF/64))))
// fractional part is 6 bits and integer part is 16 bits
//and after some calculations, it was simplified to the formulas below

//calculators for IBRD and FBRD baudrate registers
#define UART0B2DIVI(b)  (((12000000/b) >> 6) & 0xFFFF)
#define UART0B2DIVF(b)  ((12000000/b) & 0x3F)
#define UART0DIV2B(i,f) (12000000/((i<<6)+f))
#define UART0BITRATE   (UART0DIV2B(UART0IBRD,UART0FBRD))

//PCM Registers
#define PCMCS           BCM2835_REG(bcmreg_pcm, 0x0000) ///< PCM Control and Status
#define PCMFIFO         BCM2835_REG(bcmreg_pcm, 0x0004) ///< PCM FIFO Data
#define PCMMODE         BCM2835_REG(bcmreg_pcm, 0x0008) ///< PCM Mode
#define PCMRXC          BCM2835_REG(bcmreg_pcm, 0x000C) ///< PCM Receive Configuration
#define PCMTXC          BCM2835_REG(bcmreg_pcm, 0x0010) ///< PCM Transmit Configuration
#define PCMDREQ         BCM2835_REG(bcmreg_pcm, 0x0014) ///< PCM DMA Request Level
#define PCMINTEN        BCM2835_REG(bcmreg_pcm, 0x0018) ///< PCM Interrupt Enables
#define PCMINTSTC       BCM2835_REG(bcmreg_pcm, 0x001C) ///< PCM Interrupt Status & Clear
#define PCMGRAY         BCM2835_REG(bcmreg_pcm, 0x0020) ///< PCM Gray Mode Control

//TODO PCM Registers bits

//SPI0 Master Register Offsets
#define SPI0CS          BCM2835_REG(bcmreg_spi0, 0x0000) ///< SPI Master Control and Status
#define SPI0FIFO        BCM2835_REG(bcmreg_spi0, 0x0004) ///< SPI Master TX and RX FIFOs
#define SPI0CLK         BCM2835_REG(bcmreg_spi0, 0x0008) ///< SPI Master Clock Divider
#define SPI0DLEN        BCM2835_REG(bcmreg_spi0, 0x000c) ///< SPI Master Data Length
#define SPI0LTOH        BCM2835_REG(bcmreg_spi0, 0x0010) ///< SPI LOSSI mode TOH
#define SPI0DC          BCM2835_REG(bcmreg_spi0, 0x0014) ///< SPI DMA DREQ Controls

//SPI0CS bits
#define SPI0LEN_LONG    25 // Enable Long data word in Lossi mode if DMA_LEN is set
#define SPI0DMA_LEN     24 // Enable DMA mode in Lossi mode
#define SPI0CSPOL2      23 // Chip Select 2 Polarity
#define SPI0CSPOL1      22 // Chip Select 1 Polarity
#define SPI0CSPOL0      21 // Chip Select 0 Polarity
#define SPI0RXF         20 // RXF - RX FIFO Full
#define SPI0RXR         19 // RXR RX FIFO needs Reading ( full)
#define SPI0TXD         18 // TXD TX FIFO can accept Data
#define SPI0RXD         17 // RXD RX FIFO contains Data
#define SPI0DONE        16 // Done transfer Done
#define SPI0LEN         13 // LEN LoSSI enable
#define SPI0REN         12 // REN Read Enable
#define SPI0ADCS        11 // ADCS Automatically Deassert Chip Select
#define SPI0INTR        10 // INTR Interrupt on RXR
#define SPI0INTD        9 /// INTD Interrupt on Done
#define SPI0DMAEN       8 /// DMAEN DMA Enable
#define SPI0TA          7 /// Transfer Active
#define SPI0CSPOL       6 /// Chip Select Polarity
#define SPI0CLEAR_RX    5 /// Clear FIFO Clear RX 
#define SPI0CLEAR_TX    4 /// Clear FIFO Clear TX 
#define SPI0CPOL        3 /// Clock Polarity
#define SPI0CPHA        2 /// Clock Phase
#define SPI0CSM         0 /// Chip Select

#define SPI0MODE0       0x00
#define SPI0MODE1       (1 << SPI0CPHA)
#define SPI0MODE2       (1 << SPI0CPOL)
#define SPI0MODE3       (SPI0MODE1 | SPI0MODE2)

#define SPI0CS0         0x00 // Chip Select 0
#define SPI0CS1         0x01 // Chip Select 1
#define SPI0CS2         0x02 // Chip Select 0 + 1
#define SPI0CSN         0x03 // No CS, control it yourself

//for setting and getting the SPI0 clock divider
#define SPI0F2DIV(f)    (((F_CPU/f) + 1) & 0xFFFE)
#define SPI0DIV2F(d)    (F_CPU/d)
#define SPI0FREQ    (SPI0CLK?SPI0DIV2F(SPI0CLK):0)

//BSC0 is on pins 0 and 1 and is used for HAT I2C EEPROM
//BSC1 is on pin 2 and pin 3 and is accessible by the Wire library
// Defines for BSC0 and BSC1 I2C Master interfaces
#define BSC0C       BCM2835_REG(bcmreg_bsc0, 0x0000) ///< BSC0 Master Control
#define BSC0S       BCM2835_REG(bcmreg_bsc0, 0x0004) ///< BSC0 Master Status
#define BSC0DLEN    BCM2835_REG(bcmreg_bsc0, 0x0008) ///< BSC0 Master Data Length
#define BSC0A       BCM2835_REG(bcmreg_bsc0, 0x000c) ///< BSC0 Master Slave Address
#define BSC0FIFO    BCM2835_REG(bcmreg_bsc0, 0x0010) ///< BSC0 Master Data FIFO
#define BSC0DIV      BCM2835_REG(bcmreg_bsc0, 0x0014) ///< BSC0 Master Clock Divider
#define BSC0DEL      BCM2835_REG(bcmreg_bsc0, 0x0018) ///< BSC0 Master Data Delay
#define BSC0CLKT    BCM2835_REG(bcmreg_bsc0, 0x001c) ///< BSC0 Master Clock Stretch Timeout

#define BSC1C       BCM2835_REG(bcmreg_bsc1, 0x0000) ///< BSC0 Master Control
#define BSC1S       BCM2835_REG(bcmreg_bsc1, 0x0004) ///< BSC0 Master Status
#define BSC1DLEN    BCM2835_REG(bcmreg_bsc1, 0x0008) ///< BSC0 Master Data Length
#define BSC1A       BCM2835_REG(bcmreg_bsc1, 0x000c) ///< BSC0 Master Slave Address
#define BSC1FIFO    BCM2835_REG(bcmreg_bsc1, 0x0010) ///< BSC0 Master Data FIFO
#define BSC1DIV      BCM2835_REG(bcmreg_bsc1, 0x0014) ///< BSC0 Master Clock Divider
#define BSC1DEL      BCM2835_REG(bcmreg_bsc1, 0x0018) ///< BSC0 Master Data Delay
#define BSC1CLKT    BCM2835_REG(bcmreg_bsc1, 0x001c) ///< BSC0 Master Clock Stretch Timeout

//BSCxC bits
#define BSCI2CEN    15 // I2C Enable, 0 = disabled, 1 = enabled
#define BSCINTR     10 // Interrupt on RX
#define BSCINTT     9 /// Interrupt on TX
#define BSCINTD     8 /// Interrupt on DONE
#define BSCST       7 /// Start transfer, 1 = Start a new transfer
#define BSCCLEAR    4 /// Clear FIFO Clear
#define BSCREAD     0 /// Read transfer

//BSCxS bits
#define BSCCLKT      9 // Clock stretch timeout
#define BSCERR       8 // ACK error
#define BSCRXF       7 // RXF FIFO full, 0 = FIFO is not full, 1 = FIFO is full
#define BSCTXE       6 // TXE FIFO full, 0 = FIFO is not full, 1 = FIFO is full
#define BSCRXD       5 // RXD FIFO contains data
#define BSCTXD       4 // TXD FIFO can accept data
#define BSCRXR       3 // RXR FIFO needs reading (full)
#define BSCTXW       2 // TXW FIFO needs writing (full)
#define BSCDONE      1 // Transfer DONE
#define BSCTA        0 // Transfer Active

//for setting and getting the BSCx clock divider
#define BSCF2DIV(f)     (((150000000/f) + 1) & 0xFFFE)
#define BSCDIV2F(d)     (150000000/d)
#define BSC0FREQ    (BSC0DIV?BSCDIV2F(BSC0DIV):0)
#define BSC1FREQ    (BSC1DIV?BSCDIV2F(BSC1DIV):0)

//BSCS Registers
#define BSCSDR            BCM2835_REG(bcmreg_bscs, 0x0000) ///< Data Register holds (FR << 16) | (RSR << 8) | DATA
#define BSCSRSR           BCM2835_REG(bcmreg_bscs, 0x0004) ///< The operation status register and error clear register
#define BSCSSLV           BCM2835_REG(bcmreg_bscs, 0x0008) ///< The I2C SPI Address Register holds the I2C slave address value (7 bit)
#define BSCSCR            BCM2835_REG(bcmreg_bscs, 0x000C) ///< The Control register is used to configure the I2C or SPI operation
#define BSCSFR            BCM2835_REG(bcmreg_bscs, 0x0010) ///< Flag register
#define BSCSIFLS          BCM2835_REG(bcmreg_bscs, 0x0014) ///< Interrupt fifo level select register
#define BSCSIMSC          BCM2835_REG(bcmreg_bscs, 0x0018) ///< Interupt Mask Set Clear Register
#define BSCSRIS           BCM2835_REG(bcmreg_bscs, 0x001C) ///< Raw Interupt Status Register
#define BSCSMIS           BCM2835_REG(bcmreg_bscs, 0x0020) ///< Masked Interupt Status Register
#define BSCSICR           BCM2835_REG(bcmreg_bscs, 0x0024) ///< Interupt Clear Register
#define BSCSDMACR         BCM2835_REG(bcmreg_bscs, 0x0028) ///< DMA Control Register
#define BSCSTDR           BCM2835_REG(bcmreg_bscs, 0x002C) ///< FIFO Test Data
#define BSCSGPUSTAT       BCM2835_REG(bcmreg_bscs, 0x0030) ///< GPU Status Register
#define BSCSHCTRL         BCM2835_REG(bcmreg_bscs, 0x0034) ///< Host Control Register
#define BSCSDEBUG1        BCM2835_REG(bcmreg_bscs, 0x0038) ///< I2C Debug Register
#define BSCSDEBUG2        BCM2835_REG(bcmreg_bscs, 0x003C) ///< SPI Debug Register

//BSCSRSR bits
#define BSCSOE          0 // RX FIFO is full and a new data character is received
#define BSCSUE          1 // TX FIFO is empty and I2C master attempt to read a data character from I2C slave

//BSCSCR bits
#define BSCSEN          0 // Enable I2C SPI Slave.
#define BSCSSPI         1 // Enabled SPI mode
#define BSCSI2C         2 // Enabled I2C mode
#define BSCSCPHA        3 // CPHA Clock Phase
#define BSCSCPOL        4 // CPOL Clock Polarity
#define BSCSENSTAT      5 // Status register enabled (status register is transferred as a first data character on the I2C bus)
#define BSCSENCTRL      6 // Control register enabled (When enabled the control register is received as a first data character on the I2C bus)
#define BSCSBRK         7 // Stop operation and clear the FIFOs
#define BSCSTXE         8 // Transmit mode enabled
#define BSCSRXE         9 // Receive mode enabled
#define BSCSINV_RXF    10 // Inverse RX status flags
#define BSCSTESTFIFO   11 // TEST FIFO enabled
#define BSCSHOSTCTRLEN 12 // Host Control enabled. allows Host to request GPUSTAT or HCTRL register
#define BSCSINV_TXF    13 // Inverse TX status flags

//BSCSFR bits
#define BSCSTXBUSY     0 // Transmit operation in operation
#define BSCSRXFE       1 // RX FIFO is empty
#define BSCSTXFF       2 // TX FIFO is full
#define BSCSRXFF       3 // RX FIFO is full
#define BSCSTXFE       4 // TX FIFO is empty
#define BSCSRXBUSY     5 // Receive operation in operation
#define BSCSTXFLEVEL   6 // 5 bit Returns the current level of the TX FIFO use
#define BSCSRXFLEVEL   11 // 5 bit Returns the current level of the RX FIFO use

//BSCSIFLS bits
#define BSCSTXIFLSEL   0 // 3 bit TX Interrupt FIFO Level Select
#define BSCSRXIFLSEL   3 // 3 bit RX Interrupt FIFO Level Select

#define BSCSIFL_18     0x00 // Interrupt is triggered when FIFO gets 1/8 full
#define BSCSIFL_14     0x01 // Interrupt is triggered when FIFO gets 1/4 full
#define BSCSIFL_12     0x02 // Interrupt is triggered when FIFO gets 1/2 full
#define BSCSIFL_34     0x03 // Interrupt is triggered when FIFO gets 3/4 full
#define BSCSIFL_78     0x04 // Interrupt is triggered when FIFO gets 7/8 full

//bits for BSCS IMSC, RIS, MIS and ICR registers
#define BSCSRXI        0 // Receive interrupt
#define BSCSTXI        1 // Transmit interrupt
#define BSCSBEI        2 // Break error interrupt 
#define BSCSOEI        3 // Overrun error interrupt

#define BSCSMODE0       0
#define BSCSMODE1       (1 << BSCSCPHA)
#define BSCSMODE2       (1 << BSCSCPOL)
#define BSCSMODE3       (SPISMODE1 | SPISMODE2)

//AUX Registers
#define AUXIRQ        BCM2835_REG(bcmreg_aux, 0x0000) ///< Auxiliary Interrupt status
#define AUXENABLES    BCM2835_REG(bcmreg_aux, 0x0004) ///< Auxiliary enables

//AUX bits
#define AUXUART1        0
#define AUXSPI1         1
#define AUXSPI2         2

//AUX UART1 registers
#define UART1IO        BCM2835_REG(bcmreg_aux, 0x0040) ///< UART1 I/O Data
#define UART1IER    BCM2835_REG(bcmreg_aux, 0x0044) ///< UART1 Interrupt Enable
#define UART1IIR    BCM2835_REG(bcmreg_aux, 0x0048) ///< UART1 Interrupt Identify
#define UART1LCR    BCM2835_REG(bcmreg_aux, 0x004C) ///< UART1 Line Control
#define UART1MCR    BCM2835_REG(bcmreg_aux, 0x0050) ///< UART1 Modem Control
#define UART1LSR    BCM2835_REG(bcmreg_aux, 0x0054) ///< UART1 Line Status
#define UART1MSR    BCM2835_REG(bcmreg_aux, 0x0058) ///< UART1 Modem Status
#define UART1SCRATCH  BCM2835_REG(bcmreg_aux, 0x005C) ///< UART1 Scratch
#define UART1CNTL    BCM2835_REG(bcmreg_aux, 0x0060) ///< UART1 Extra Control
#define UART1STAT    BCM2835_REG(bcmreg_aux, 0x0064) ///< UART1 Extra Status
#define UART1BAUD    BCM2835_REG(bcmreg_aux, 0x0068) ///< UART1 Baudrate

//UART1IER bits
#define UART1TXIE       0
#define UART1RXIE       1

//UART1IIR register
#define UART1NOI        0 //This bit is clear whenever an interrupt is pending
#define UART1TXI        1 //Transmit holding register empty (write 1 to clear TXFIFO)
#define UART1RXI        2 //Receiver holds valid byte (write 1 to clear RXFIFO)

//UART1LCR register
#define UART1WLEN       0 //Data Size (If clear the UART works in 7-bit mode, If set the UART works in 8-bit mode)
#define UART1BREAK      6 //If set the first to Mini UART register give access the the Baudrate register
#define UART1DLAB       7 //If set high the UART1_TX line is pulled low continuously

#define UART1WLEN_7BIT  0x00
#define UART1WLEN_8BIT  0x01

//UART1MCR register
#define UART1RTSC       1 // R/W RTS Control (If clear the UART1_RTS line is high, If set the UART1_RTS line is low)

//UART1CNTL register
#define UART1RXE        0 //If this bit is set the mini UART receiver is enabled
#define UART1TXE        1 //If this bit is set the mini UART transmitter is enabled
#define UART1RTSE       2 //If this bit is set the RTS line will de-assert if the receive FIFO reaches it 'auto flow' level
#define UART1CTSE       3 //If this bit is set the transmitter will stop if the CTS line is de-asserted
#define UART1RTSAFL     4 //These two bits specify at what receiver FIFO level the RTS line is de-asserted in auto-flow mode
#define UART1RTSL       6 //This bit allows one to invert the RTS auto flow operation polarity
#define UART1CTSL       7 //This bit allows one to invert the CTS auto flow operation polarity

#define UART1RTSA_3     0x00 //De-assert RTS when the receive FIFO has 3 empty spaces left
#define UART1RTSA_2     0x01 //De-assert RTS when the receive FIFO has 2 empty spaces left
#define UART1RTSA_1     0x02 //De-assert RTS when the receive FIFO has 1 empty spaces left
#define UART1RTSA_4     0x03 //De-assert RTS when the receive FIFO has 4 empty spaces left

//UART1STAT register
#define UART1RX_READY   0 //If this bit is set the mini UART receive FIFO contains at least 1 symbol
#define UART1TX_READY   1 //If this bit is set the mini UART transmitter FIFO can accept at least one more symbol
#define UART1RX_IDLE    2 //If this bit is set the receiver is idle
#define UART1TX_IDLE    3 //If this bit is set the transmitter is idle
#define UART1OE         4 //This bit is set if there was a receiver overrun
#define UART1TXFF       5 //If this bit is set the mini UART transmitter FIFO is full
#define UART1RTS_GET    6 //This bit shows the status of the UART1_RTS line
#define UART1CTS_GET    7 //This bit shows the status of the UART1_CTS line
#define UART1TXFE       8 //If this bit is set the transmitter FIFO is empty. Thus it can accept 8 symbols
#define UART1TX_DONE    9 //This bit is set if the transmitter is idle and the transmit FIFO is empty
#define UART1RX_LEN     16 //These 4 bits shows how many symbols are stored in the receive FIFO (range 0-8) 
#define UART1TX_LEN     24 //These 4 bits shows how many symbols are stored in the transmit FIFO (range 0-8) 

//Clock calculations
#define UART1B2DIV(b)   ((F_CPU/(8 * b)) - 1) //baud value to register value
#define UART1DIV2B(b)   (F_CPU/(8 * (b + 1))) //register value to baud value
#define UART1BITRATE  (UART1BAUD?UART1DIV2B(UART1BAUD):0)

//AUX SPI1 registers
#define SPI1CNTL0      BCM2835_REG(bcmreg_aux, 0x0080) ///< SPI1 Control 0
#define SPI1CNTL1      BCM2835_REG(bcmreg_aux, 0x0084) ///< SPI1 Control 1
#define SPI1STAT      BCM2835_REG(bcmreg_aux, 0x0088) ///< SPI1 Status
#define SPI1PEEK      BCM2835_REG(bcmreg_aux, 0x008C) ///< SPI1 Peek
#define SPI1IO        BCM2835_REG(bcmreg_aux, 0x00A0) ///< SPI1 Data
#define SPI1TXHOLD      BCM2835_REG(bcmreg_aux, 0x00B0) ///< SPI1 Data
#define SPI2CNTL0      BCM2835_REG(bcmreg_aux, 0x00C0) ///< SPI2 Control 0
#define SPI2CNTL1      BCM2835_REG(bcmreg_aux, 0x00C4) ///< SPI2 Control 1
#define SPI2STAT      BCM2835_REG(bcmreg_aux, 0x00C8) ///< SPI2 Status
#define SPI2PEEK      BCM2835_REG(bcmreg_aux, 0x00CC) ///< SPI2 Peek
#define SPI2IO          BCM2835_REG(bcmreg_aux, 0x00E0) ///< SPI2 Data
#define SPI2TXHOLD      BCM2835_REG(bcmreg_aux, 0x00F0) ///< SPI2 Data

//SPI1CNTL0
#define SPI1DATA_LEN    0 //6 bits Specifies the number of bits to shift when not in VAR_LEN mode
#define SPI1TX_MSB      6 //If 1 the data is shifted out starting with the MS bit
#define SPI1CLK_INVERT  7 //If 1 the 'idle' clock line state is high
#define SPI1TX_RISING   8 //If 1 data is clocked out on the rising edge of the SPI clock
#define SPI1FLUSH_FIFOS 9 //If 1 the receive and transmit FIFOs are held in reset (and thus flushed.)
#define SPI1RX_RISING   10 //If 1 data is clocked in on the rising edge of the SPI clock
#define SPI1SPIE        11 //Enables the SPI interface. Whilst disabled the FIFOs can still be written to or read from
#define SPI1DOUT_HOLD   12 //2 bits Controls the extra DOUT hold time in system clock cycles (0,1,2,3 => 0,1,4,7 clocks)
#define SPI1VAR_LEN     14 //If 1 the SPI takes the shift length and the data from the TX fifo
#define SPI1VAR_CS      15 //If 1 the SPI takes the CS pattern and the data from the TX fifo (ONLY when in VAR_LEN mode)
#define SPI1PIM         16 //If set the SPI input works in post input mode
#define SPI1CSV         17 //3 bits The pattern output on the CS pins when active
#define SPI1SPEED       20 //Sets the SPI clock speed. spi_clk_freq = system_clock_freq/2*(speed+1)

//SPI1CNTL1
#define SPI1KEEPBIT     0 //If 1 the receiver shift register is NOT cleared. Thus new data is concatenated to old data
#define SPI1RX_MSB      1 //If 1 the data is shifted in starting with the MS bit
#define SPI1IDLEIE      6 //If 1 the interrupt line is high when the interface is idle
#define SPI1TXIE        7 //If 1 the interrupt line is high when the transmit FIFO is empty
#define SPI1CS_HOLD     8 //3 bits Additional SPI clock cycles where the CS is high

//SPI1STAT
#define SPI1_LEFT       0 //6 bits The number of bits still to be processed. Starts with 'shift-length' and counts down
#define SPI1_BUSY       6 //Indicates the module is busy transferring data
#define SPI1RX_EMPTY    7 //If 1 the receiver FIFO is empty
#define SPI1TX_EMPTY    8 //If 1 the transmit FIFO is empty
#define SPI1TX_FULL     9 //If 1 the transmit FIFO is full
#define SPI1RX_LEN      16 //8 bits The number of data units in the receive data FIFO
#define SPI1TX_LEN      24 //8 bits The number of data units in the transmit data FIFO

#define SPI1F2DIV(b)    (((F_CPU/(2 * b)) - 1) & 0x0FFF) //baud value to register value
#define SPI1DIV2F(b)    (F_CPU/(2 * (b + 1))) //register value to baud value
#define SPI1FREQ    ((SPI1CNTL0 >> 20)?SPI1DIV2F(SPI1CNTL0 >> 20):0)
#define SPI2FREQ    ((SPI2CNTL0 >> 20)?SPI1DIV2F(SPI2CNTL0 >> 20):0)

#define PIONE_PINMASK    0x0BC6CF93 // GPIO PINS available on first version PI
#define PITWO_PINMASK    0xFBC6CF9C // GPIO PINS available on PI A/B
#define PIPLUS_PINMASK   0x0FFFFFFC // GPIO PINS available on PI A+/B+
#define PINMASKS_LEN     20

const static uint32_t rpi_model_pinmasks[] = {
    0,              // 0x00 Not a board
    0,              // 0x01 Not a board
    PIONE_PINMASK,  // 0x02 Model B Revision 1.0
    PIONE_PINMASK,  // 0x03 Model B Revision 1.0 + ECN0001 (no fuses, D14 removed)
    PITWO_PINMASK,  // 0x04 Model B Revision 2.0 Mounting holes
    PITWO_PINMASK,  // 0x05 Model B Revision 2.0 Mounting holes
    PITWO_PINMASK,  // 0x06 Model B Revision 2.0 Mounting holes
    PIONE_PINMASK,  // 0x07 Model A Mounting holes
    PIONE_PINMASK,  // 0x08 Model A Mounting holes
    PIONE_PINMASK,  // 0x09 Model A Mounting holes
    0,              // 0x0A Not a board
    0,              // 0x0B Not a board
    0,              // 0x0C Not a board
    PITWO_PINMASK,  // 0x0D Model B Revision 2.0 Mounting holes
    PITWO_PINMASK,  // 0x0E Model B Revision 2.0 Mounting holes
    PITWO_PINMASK,  // 0x0F Model B Revision 2.0 Mounting holes
    PIPLUS_PINMASK, // 0x10 Model B+
    0xFFFFFFFF,     // 0x11 Compute Module
    PIPLUS_PINMASK, // 0x12 Model A+
    PIPLUS_PINMASK  // 0x13 Pi 2 Model B
};

//memory maps holding the registers
extern volatile uint32_t *bcmreg_st;
extern volatile uint32_t *bcmreg_irq;
extern volatile uint32_t *bcmreg_pm;
extern volatile uint32_t *bcmreg_cm;
extern volatile uint32_t *bcmreg_gpio;
extern volatile uint32_t *bcmreg_uart0;
extern volatile uint32_t *bcmreg_pcm;
extern volatile uint32_t *bcmreg_spi0;
extern volatile uint32_t *bcmreg_bsc0;
extern volatile uint32_t *bcmreg_pwm;
extern volatile uint32_t *bcmreg_bscs;
extern volatile uint32_t *bcmreg_aux;
extern volatile uint32_t *bcmreg_bsc1;

#ifdef __cplusplus
extern "C" {
#endif

static inline uint32_t border_get(volatile uint32_t reg){
  uint32_t data = *(&reg);
  *(&reg);
  return data;
}

static inline void border_set(volatile uint32_t reg, uint32_t value){
  *(&reg) = value;
  *(&reg) = value;
}

static inline void border_set_mask(volatile uint32_t reg, uint32_t mask){
  *(&reg) |= mask;
  *(&reg) |= mask;
}

static inline void border_clr_mask(volatile uint32_t reg, uint32_t mask){
  *(&reg) &= ~mask;
  *(&reg) &= ~mask;
}

static inline void border_set_bit(volatile uint32_t reg, uint8_t bit){
  border_set_mask(reg, 1 << bit);
}

static inline void border_clr_bit(volatile uint32_t reg, uint8_t bit){
  border_clr_mask(reg, 1 << bit);
}

#ifdef __cplusplus
}
#endif


#endif
