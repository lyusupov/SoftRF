
/* Peripherals */
#define SOC_GPIO_PIN_CONS_ND_RX   _PINNUM(1,  0) // P1.00
#define SOC_GPIO_PIN_CONS_ND_TX   SOC_UNUSED_PIN // TBD

/* L76K */
#define SOC_GPIO_PIN_GNSS_ND_RX   _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_GNSS_ND_TX   _PINNUM(1,  8) // P1.08

#define SOC_GPIO_PIN_GNSS_ND_PPS  _PINNUM(1,  4) // P1.04
#define SOC_GPIO_PIN_GNSS_ND_WKE  _PINNUM(1,  7) // P1.07
#define SOC_GPIO_PIN_GNSS_ND_RST  _PINNUM(1,  5) // P1.05
#define SOC_GPIO_PIN_GNSS_ND_SW   _PINNUM(1,  2) // P1.02
#define SOC_GPIO_PIN_GNSS_ND_PWR  _PINNUM(0, 12) // P0.12 active HIGH

/* SPI */
#define SOC_GPIO_PIN_ND_MOSI      _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_ND_MISO      _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_ND_SCK       _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_ND_SS        _PINNUM(0, 24) // P0.24

/* SX1262 */
#define SOC_GPIO_PIN_ND_RST       _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_ND_DIO1      _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_ND_DIO3      _PINNUM(0, 21) // P0.21
#define SOC_GPIO_PIN_ND_BUSY      _PINNUM(0, 17) // P0.17
#define SOC_GPIO_PIN_ND_PWR       _PINNUM(0, 13) // P0.13 active HIGH

/* GDEM0213B74 E-paper (250?122) */
#define SOC_GPIO_PIN_EPD_ND_MISO  SOC_UNUSED_PIN // TBD
#define SOC_GPIO_PIN_EPD_ND_MOSI  _PINNUM(0, 29) // P0.29
#define SOC_GPIO_PIN_EPD_ND_SCK   _PINNUM(0, 31) // P0.31
#define SOC_GPIO_PIN_EPD_ND_SS    _PINNUM(0, 30) // P0.30
#define SOC_GPIO_PIN_EPD_ND_DC    _PINNUM(0, 28) // P0.28
#define SOC_GPIO_PIN_EPD_ND_RST   _PINNUM(0,  2) // P0.02
#define SOC_GPIO_PIN_EPD_ND_BUSY  _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_EPD_ND_PWR   _PINNUM(1, 10) // P1.10 active LOW

/* I2C */
#define SOC_GPIO_PIN_ND_SDA       _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_ND_SCL       _PINNUM(0, 27) // P0.27

/* buttons , active LOW */
#define SOC_GPIO_PIN_ND_BUTTON1   _PINNUM(1, 11) // P1.11
#define SOC_GPIO_PIN_ND_BUTTON2   _PINNUM(0,  9) // P0.09 [NFC1]
#define SOC_GPIO_PIN_ND_BUTTON3   _PINNUM(0,  6) // P0.06
#define SOC_GPIO_PIN_ND_BUTTON4   _PINNUM(0,  8) // P0.08
#define SOC_GPIO_PIN_ND_BUTTON5   _PINNUM(0, 10) // P0.10 [NFC2]
#define SOC_GPIO_PIN_ND_BUTTON6   _PINNUM(0, 11) // P0.11

/* LEDs , active LOW */
#define SOC_GPIO_LED_ND_RED       _PINNUM(0, 14) // P0.14
#define SOC_GPIO_LED_ND_BLUE      _PINNUM(1,  1) // P1.01

/* P25Q32 QSPI flash */
#define SOC_GPIO_PIN_SFL_ND_MOSI  _PINNUM(1, 12) // P1.12
#define SOC_GPIO_PIN_SFL_ND_MISO  _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_SFL_ND_SCK   _PINNUM(1, 14) // P1.14
#define SOC_GPIO_PIN_SFL_ND_SS    _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_SFL_ND_HOLD  _PINNUM(0,  5) // P0.05
#define SOC_GPIO_PIN_SFL_ND_WP    _PINNUM(0,  7) // P0.07

/* RTC */
#define SOC_GPIO_PIN_RTC_ND_INT   _PINNUM(0, 16) // P0.16

/* ADC */
#define SOC_GPIO_PIN_ND_BATTERY   _PINNUM(0,  4) // P0.04 (AIN2)
#define SOC_GPIO_PIN_ND_VBAT_EN   _PINNUM(0, 15) // P0.15 active HIGH
