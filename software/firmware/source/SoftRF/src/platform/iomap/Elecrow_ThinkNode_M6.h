
/* Peripherals */
#define SOC_GPIO_PIN_CONS_M6_RX   _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_CONS_M6_TX   _PINNUM(0, 24) // P0.24

/* L76K */
#define SOC_GPIO_PIN_GNSS_M6_RX   _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_GNSS_M6_TX   _PINNUM(0,  2) // P0.02

#define SOC_GPIO_PIN_GNSS_M6_PPS  _PINNUM(0, 31) // P0.31
#define SOC_GPIO_PIN_GNSS_M6_WKE  _PINNUM(0, 30) // P0.30
#define SOC_GPIO_PIN_GNSS_M6_RST  _PINNUM(0, 29) // P0.29
#define SOC_GPIO_PIN_GNSS_M6_EN   _PINNUM(0,  6) // P0.06

/* SPI */
#define SOC_GPIO_PIN_M6_MOSI      _PINNUM(1, 14) // P1.14
#define SOC_GPIO_PIN_M6_MISO      _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_M6_SCK       _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_M6_SS        _PINNUM(1, 12) // P1.12

/* SX1262 */
#define SOC_GPIO_PIN_M6_RST       _PINNUM(1, 10) // P1.10
#define SOC_GPIO_PIN_M6_DIO1      _PINNUM(1,  6) // P1.06
#define SOC_GPIO_PIN_M6_BUSY      _PINNUM(1, 11) // P1.11

/* Peripheral power */
#define SOC_GPIO_PIN_IO_M6_PWR    _PINNUM(0, 27) // P0.27

/* I2C */
#define SOC_GPIO_PIN_M6_SDA       _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_M6_SCL       _PINNUM(0,  8) // P0.08

/* button */
#define SOC_GPIO_PIN_M6_BUTTON    _PINNUM(0, 17) // P0.17

/* LED */
#define SOC_GPIO_LED_M6_RED       _PINNUM(0, 12) // P0.12 active HIGH
#define SOC_GPIO_LED_M6_BLUE      _PINNUM(0,  7) // P0.07 active HIGH

/* MX25R1635F SPI flash */
#define SOC_GPIO_PIN_SFL_M6_MOSI  _PINNUM(1,  1) // P1.01
#define SOC_GPIO_PIN_SFL_M6_MISO  _PINNUM(1,  2) // P1.02
#define SOC_GPIO_PIN_SFL_M6_SCK   _PINNUM(1,  3) // P1.03
#define SOC_GPIO_PIN_SFL_M6_SS    _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_SFL_M6_HOLD  _PINNUM(1,  5) // P1.05
#define SOC_GPIO_PIN_SFL_M6_WP    _PINNUM(1,  4) // P1.04

#define SOC_GPIO_PIN_SFL_M6_EN    _PINNUM(0, 21) // P0.21

/* RTC */
#define SOC_GPIO_PIN_RTC_M6_INT   _PINNUM(0,  4) // P0.04

/* ADC */
#define SOC_GPIO_PIN_M6_BATTERY   _PINNUM(0, 28) // P0.28
#define SOC_GPIO_PIN_M6_ADC_EN    _PINNUM(0, 11) // P0.11 active HIGH

#define SOC_ADC_M6_VOLTAGE_DIV    (1.75F)

/* digital input */
#define SOC_GPIO_PIN_M6_VCHG_SEN  _PINNUM(0, 15) // P0.15 active LOW
#define SOC_GPIO_PIN_M6_VUSB_SEN  _PINNUM(0, 13) // P0.13
