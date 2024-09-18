
/* Peripherals */
#define SOC_GPIO_PIN_CONS_T114_RX   _PINNUM(0,  9) // P0.09 , No NFC
#define SOC_GPIO_PIN_CONS_T114_TX   _PINNUM(0, 10) // P0.10 , No NFC

#define SOC_GPIO_PIN_GNSS_T114_RX   _PINNUM(1,  5) // P1.05
#define SOC_GPIO_PIN_GNSS_T114_TX   _PINNUM(1,  7) // P1.07

#define SOC_GPIO_PIN_GNSS_T114_PPS  _PINNUM(1,  4) // P1.04
#define SOC_GPIO_PIN_GNSS_T114_WKE  _PINNUM(1,  2) // P1.02
#define SOC_GPIO_PIN_GNSS_T114_RST  _PINNUM(1,  6) // P1.06

/* SPI */
#define SOC_GPIO_PIN_T114_MOSI      _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_T114_MISO      _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_T114_SCK       _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_T114_SS        _PINNUM(0, 24) // P0.24

/* SX1262 */
#define SOC_GPIO_PIN_T114_RST       _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_T114_DIO1      _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_T114_BUSY      _PINNUM(0, 17) // P0.17

/* I2C int. (same that T-Echo has) */
#define SOC_GPIO_PIN_T114_SDA_INT   _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_T114_SCL_INT   _PINNUM(0, 27) // P0.27

/* I2C ext. (air pressure sensor) */
#define SOC_GPIO_PIN_T114_SDA_EXT   _PINNUM(0, 16) // P0.16
#define SOC_GPIO_PIN_T114_SCL_EXT   _PINNUM(0, 13) // P0.13

/* button */
#define SOC_GPIO_PIN_T114_BUTTON    _PINNUM(1, 10) // P1.10

/* LED */
#define SOC_GPIO_LED_T114_GREEN     _PINNUM(1,  3) // P1.03
#define SOC_GPIO_PIN_T114_LED       _PINNUM(0, 14) // P0.14 , 2x SK6812

/* ADC */
#define SOC_GPIO_PIN_T114_BATTERY   _PINNUM(0,  4) // P0.04
#define SOC_GPIO_PIN_T114_ADC_EN    _PINNUM(0,  6) // P0.06

#define SOC_ADC_T114_VOLTAGE_DIV    (4.9F) // 390K + 100K voltage divider on VBAT

/* Ext. sensors */
#define SOC_GPIO_PIN_T114_VEXT_EN   _PINNUM(0, 21) // P0.21

/* MX25R1635F SPI flash */
#define SOC_GPIO_PIN_T114_SFL_MOSI  _PINNUM(1, 12) // P1.12
#define SOC_GPIO_PIN_T114_SFL_MISO  _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_T114_SFL_SCK   _PINNUM(1, 14) // P1.14
#define SOC_GPIO_PIN_T114_SFL_SS    _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_T114_SFL_HOLD  _PINNUM(0,  5) // P0.05
#define SOC_GPIO_PIN_T114_SFL_WP    _PINNUM(0,  7) // P0.07

/* TFT */
#define SOC_GPIO_PIN_T114_TFT_MOSI  _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_T114_TFT_MISO  _PINNUM(1, 11) // P1.11 NC
#define SOC_GPIO_PIN_T114_TFT_SCK   _PINNUM(1,  8) // P1.08
#define SOC_GPIO_PIN_T114_TFT_SS    _PINNUM(0, 11) // P0.11
#define SOC_GPIO_PIN_T114_TFT_DC    _PINNUM(0, 12) // P0.12
#define SOC_GPIO_PIN_T114_TFT_RST   _PINNUM(0,  2) // P0.02
#define SOC_GPIO_PIN_T114_TFT_EN    _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_T114_TFT_BLGT  _PINNUM(0, 15) // P0.15

/* RTC */
#define SOC_GPIO_PIN_T114_R_INT     _PINNUM(0,  8) // P0.08
