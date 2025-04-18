
/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX            _PINNUM(0, 8) // P0.08
#define SOC_GPIO_PIN_CONS_TX            _PINNUM(0, 6) // P0.06

#define SOC_GPIO_PIN_GNSS_RX            _PINNUM(1, 9) // P1.09
#define SOC_GPIO_PIN_GNSS_TX            _PINNUM(1, 8) // P1.08

#define SOC_GPIO_PIN_GNSS_WKE           _PINNUM(1, 2) // P1.02
#define SOC_GPIO_PIN_GNSS_RST           _PINNUM(1, 5) // P1.05 (REV_2 only)
#define SOC_GPIO_PIN_GNSS_TECHO_PPS     _PINNUM(1, 4) // P1.04

#define SOC_GPIO_PIN_LED                SOC_UNUSED_PIN

#define SOC_GPIO_LED_TECHO_REV_0_GREEN  _PINNUM(0, 13) // P0.13 (Green)
#define SOC_GPIO_LED_TECHO_REV_0_RED    _PINNUM(0, 14) // P0.14 (Red)
#define SOC_GPIO_LED_TECHO_REV_0_BLUE   _PINNUM(0, 15) // P0.15 (Blue)
#define SOC_GPIO_LED_TECHO_REV_1_GREEN  _PINNUM(0, 15)
#define SOC_GPIO_LED_TECHO_REV_1_RED    _PINNUM(0, 13)
#define SOC_GPIO_LED_TECHO_REV_1_BLUE   _PINNUM(0, 14)
#define SOC_GPIO_LED_TECHO_REV_2_GREEN  _PINNUM(1,  1) // P1.01 (Green)
#define SOC_GPIO_LED_TECHO_REV_2_RED    _PINNUM(1,  3) // P1.03 (Red)
#define SOC_GPIO_LED_TECHO_REV_2_BLUE   _PINNUM(0, 14) // P0.14 (Blue)

#define SOC_GPIO_PIN_BATTERY            _PINNUM(0, 4) // P0.04 (AIN2)

#define SOC_GPIO_PIN_RX3                SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TX3                SOC_UNUSED_PIN

/* SPI */
#define SOC_GPIO_PIN_TECHO_REV_0_MOSI   _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_TECHO_REV_1_MOSI   SOC_GPIO_PIN_TECHO_REV_0_MOSI
#define SOC_GPIO_PIN_TECHO_REV_2_MOSI   SOC_GPIO_PIN_TECHO_REV_0_MOSI
#define SOC_GPIO_PIN_TECHO_REV_0_MISO   _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_TECHO_REV_1_MISO   SOC_GPIO_PIN_TECHO_REV_0_MISO
#define SOC_GPIO_PIN_TECHO_REV_2_MISO   SOC_GPIO_PIN_TECHO_REV_0_MISO
#define SOC_GPIO_PIN_TECHO_REV_0_SCK    _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_TECHO_REV_1_SCK    SOC_GPIO_PIN_TECHO_REV_0_SCK
#define SOC_GPIO_PIN_TECHO_REV_2_SCK    SOC_GPIO_PIN_TECHO_REV_0_SCK
#define SOC_GPIO_PIN_SS                 _PINNUM(0, 24) // P0.24

/* NRF905 */
#define SOC_GPIO_PIN_TXE                SOC_UNUSED_PIN
#define SOC_GPIO_PIN_CE                 SOC_UNUSED_PIN
#define SOC_GPIO_PIN_PWR                SOC_UNUSED_PIN

/* SX1262 or SX1276 */
#define SOC_GPIO_PIN_TECHO_REV_0_RST    _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_TECHO_REV_1_RST    SOC_GPIO_PIN_TECHO_REV_0_RST
#define SOC_GPIO_PIN_TECHO_REV_2_RST    SOC_GPIO_PIN_TECHO_REV_0_RST
#define SOC_GPIO_PIN_TECHO_REV_0_DIO0   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TECHO_REV_1_DIO0   _PINNUM(1,  1) // P1.01
#define SOC_GPIO_PIN_TECHO_REV_2_DIO0   _PINNUM(0, 15) // P0.15
#define SOC_GPIO_PIN_DIO1               _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_BUSY               _PINNUM(0, 17) // P0.17

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX           SOC_UNUSED_PIN

/* I2C */
#define SOC_GPIO_PIN_SDA                _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_SCL                _PINNUM(0, 27) // P0.27

/* buttons */
#define SOC_GPIO_PIN_TECHO_REV_0_BUTTON _PINNUM(1, 10) // P1.10
#define SOC_GPIO_PIN_TECHO_REV_1_BUTTON SOC_GPIO_PIN_TECHO_REV_0_BUTTON
#define SOC_GPIO_PIN_TECHO_REV_2_BUTTON SOC_GPIO_PIN_TECHO_REV_0_BUTTON
#define SOC_GPIO_PIN_PAD                _PINNUM(0, 11) // P0.11

/* E-paper */
#define SOC_GPIO_PIN_EPD_MISO           _PINNUM(1,  7) // P1.07
#define SOC_GPIO_PIN_EPD_MOSI           _PINNUM(0, 29) // P0.29
#define SOC_GPIO_PIN_EPD_SCK            _PINNUM(0, 31) // P0.31
#define SOC_GPIO_PIN_EPD_SS             _PINNUM(0, 30) // P0.30
#define SOC_GPIO_PIN_EPD_DC             _PINNUM(0, 28) // P0.28
#define SOC_GPIO_PIN_EPD_RST            _PINNUM(0,  2) // P0.02
#define SOC_GPIO_PIN_EPD_BUSY           _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_EPD_BLGT           _PINNUM(1, 11) // P1.11

/* Power: EINK, RGB, CN1 (, RF) REV_2: FLASH, GNSS, SENSOR */
#define SOC_GPIO_PIN_IO_PWR             _PINNUM(0, 12) // P0.12
/* REV_2 power: RF */
#define SOC_GPIO_PIN_3V3_PWR            _PINNUM(0, 13) // P0.13
/* Modded REV_1 3V3 power */
#define SOC_GPIO_PIN_TECHO_REV_1_3V3_PWR  SOC_GPIO_PIN_TECHO_REV_1_DIO0

/* MX25R1635F SPI flash */
#define SOC_GPIO_PIN_SFL_MOSI           _PINNUM(1, 12) // P1.12
#define SOC_GPIO_PIN_SFL_MISO           _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_SFL_SCK            _PINNUM(1, 14) // P1.14
#define SOC_GPIO_PIN_SFL_SS             _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_SFL_HOLD           _PINNUM(0,  5) // P0.05 (REV_1 and REV_2)
#define SOC_GPIO_PIN_SFL_WP             _PINNUM(0,  7) // P0.07 (REV_1 and REV_2)

/* RTC */
#define SOC_GPIO_PIN_R_INT              _PINNUM(0, 16) // P0.16

/* NFC */
#define SOC_GPIO_PIN_NFC_ANT1           _PINNUM(0,  9) // P0.09
#define SOC_GPIO_PIN_NFC_ANT2           _PINNUM(0, 10) // P0.10
