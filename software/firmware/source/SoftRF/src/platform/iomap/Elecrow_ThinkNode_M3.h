
/* Peripherals */
#define SOC_GPIO_PIN_CONS_M3_RX   SOC_UNUSED_PIN // TBD
#define SOC_GPIO_PIN_CONS_M3_TX   SOC_UNUSED_PIN // TBD

/* L76K */
#define SOC_GPIO_PIN_GNSS_M3_RX   _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_GNSS_M3_TX   _PINNUM(0, 22) // P0.22

#define SOC_GPIO_PIN_GNSS_M3_PPS  SOC_UNUSED_PIN // TBD
#define SOC_GPIO_PIN_GNSS_M3_WKE  _PINNUM(0, 21) // P0.21
#define SOC_GPIO_PIN_GNSS_M3_RST  _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_GNSS_M3_EN   _PINNUM(0, 14) // P0.14 active HIGH

/* SPI */
#define SOC_GPIO_PIN_M3_MOSI      _PINNUM(1, 14) // P1.14
#define SOC_GPIO_PIN_M3_MISO      _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_M3_SCK       _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_M3_SS        _PINNUM(1, 12) // P1.12

/* SX1262 */
#define SOC_GPIO_PIN_M3_RST       _PINNUM(1, 10) // P1.10
#define SOC_GPIO_PIN_M3_DIO1      _PINNUM(1,  8) // P1.08
#define SOC_GPIO_PIN_M3_BUSY      _PINNUM(1, 11) // P1.11

/* I2C */
#define SOC_GPIO_PIN_M3_SDA       _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_M3_SCL       _PINNUM(0, 27) // P0.27

#define SOC_GPIO_PIN_M3_EEPROM_EN _PINNUM(0,  7) // P0.07 active HIGH

/* button */
#define SOC_GPIO_PIN_M3_BUTTON    _PINNUM(0, 12) // P0.12
#define SOC_GPIO_PIN_M3_BUT_EN    _PINNUM(0, 16) // P0.16 active HIGH

/* LED */
#define SOC_GPIO_LED_M3_RED       _PINNUM(1,  1) // P1.01
#define SOC_GPIO_LED_M3_GREEN     _PINNUM(1,  3) // P1.03
#define SOC_GPIO_LED_M3_BLUE      _PINNUM(1,  5) // P1.05
#define SOC_GPIO_LED_M3_RGB_PWR   _PINNUM(0, 29) // P0.29 active HIGH

/* buzzer */
#define SOC_GPIO_PIN_M3_BUZZER    _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_M3_EN1       _PINNUM(1,  4) // P1.04 active HIGH
#define SOC_GPIO_PIN_M3_EN2       _PINNUM(1,  2) // P1.02 active HIGH

/* RTC */
#define SOC_GPIO_PIN_RTC_M3_INT   SOC_UNUSED_PIN // TBD

/* ADC */
#define SOC_GPIO_PIN_M3_BATTERY   _PINNUM(0,  5) // P0.05
#define SOC_GPIO_PIN_M3_ADC_EN    _PINNUM(0, 17) // P0.17 active HIGH

#define SOC_ADC_M3_VOLTAGE_DIV    (1.75F)

/* digital input */
#define SOC_GPIO_PIN_M3_BAT_CHRG  _PINNUM(1,  0) // P1.00
#define SOC_GPIO_PIN_M3_BAT_FULL  _PINNUM(0, 24) // P0.24
#define SOC_GPIO_PIN_M3_VUSB_SEN  _PINNUM(0, 31) // P0.31
#define SOC_GPIO_PIN_M3_VGPS_SEN  SOC_UNUSED_PIN // TBD

/* Sensors (SC7A20H + AHT20) */
#define SOC_GPIO_PIN_M3_ACC_EN    _PINNUM(0,  2) // P0.02 active HIGH ?
#define SOC_GPIO_PIN_M3_TEMP_EN   _PINNUM(0,  3) // P0.03 active HIGH
