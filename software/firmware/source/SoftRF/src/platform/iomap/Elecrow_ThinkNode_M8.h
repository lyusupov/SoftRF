
/* Peripherals */
#define SOC_GPIO_PIN_CONS_M8_RX   _PINNUM(0,  9) // P0.09 , No NFC
#define SOC_GPIO_PIN_CONS_M8_TX   _PINNUM(0, 10) // P0.10 , No NFC

/* L76K */
#define SOC_GPIO_PIN_GNSS_M8_RX   _PINNUM(1,  2) // P1.02
#define SOC_GPIO_PIN_GNSS_M8_TX   _PINNUM(1,  4) // P1.04

#define SOC_GPIO_PIN_GNSS_M8_PPS  _PINNUM(0, 14) // P0.14
#define SOC_GPIO_PIN_GNSS_M8_WKE  _PINNUM(0, 15) // P0.15
#define SOC_GPIO_PIN_GNSS_M8_RST  _PINNUM(0, 17) // P0.17
#define SOC_GPIO_PIN_GNSS_M8_EN   _PINNUM(0, 16) // P0.16 active high

/* SPI */
#define SOC_GPIO_PIN_M8_MOSI      _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_M8_MISO      _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_M8_SCK       _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_M8_SS        _PINNUM(0, 21) // P0.21

/* SX1262 */
#define SOC_GPIO_PIN_M8_RST       _PINNUM(0, 24) // P0.24
#define SOC_GPIO_PIN_M8_DIO1      _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_M8_BUSY      _PINNUM(1,  0) // P1.00
/* PE4259 /CTRL */
#define SOC_GPIO_PIN_M8_ANT_SW    _PINNUM(0, 23) // P0.23

/* E-paper */
#define SOC_GPIO_PIN_EPD_M8_MISO  _PINNUM(0, 11) // P0.11 NC ?
#define SOC_GPIO_PIN_EPD_M8_MOSI  _PINNUM(0, 29) // P0.29
#define SOC_GPIO_PIN_EPD_M8_SCK   _PINNUM(0, 31) // P0.31
#define SOC_GPIO_PIN_EPD_M8_SS    _PINNUM(0, 30) // P0.30
#define SOC_GPIO_PIN_EPD_M8_DC    _PINNUM(0, 28) // P0.28
#define SOC_GPIO_PIN_EPD_M8_RST   _PINNUM(0,  2) // P0.02
#define SOC_GPIO_PIN_EPD_M8_BUSY  _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_EPD_M8_BLGT  _PINNUM(1, 11) // P1.11
#define SOC_GPIO_PIN_EPD_M8_EN    _PINNUM(1, 10) // P1.10

/* I2C (PCF8563 at 0x51, SC7A20 at 0x19) */
#define SOC_GPIO_PIN_M8_SDA       _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_M8_SCL       _PINNUM(0, 27) // P0.27
#define SOC_GPIO_PIN_M8_IIC_EN    _PINNUM(0, 13) // P0.13

/* encoder */
#define SOC_GPIO_PIN_M8_ENC_BTN   _PINNUM(0, 12) // P0.12
#define SOC_GPIO_PIN_M8_ENC_A     _PINNUM(0,  8) // P0.08
#define SOC_GPIO_PIN_M8_ENC_B     _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_M8_ENC_C     _PINNUM(0,  5) // P0.06

/* buzzer */
#define SOC_GPIO_PIN_M8_BUZZER    _PINNUM(1,  1) // P1.01

/* MX25R1635F (?) SPI flash */
#define SOC_GPIO_PIN_SFL_M8_MOSI  _PINNUM(1, 12) // P1.12
#define SOC_GPIO_PIN_SFL_M8_MISO  _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_SFL_M8_SCK   _PINNUM(1, 14) // P1.14
#define SOC_GPIO_PIN_SFL_M8_SS    _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_SFL_M8_HOLD  _PINNUM(0,  5) // P0.05
#define SOC_GPIO_PIN_SFL_M8_WP    _PINNUM(0,  7) // P0.07

/* RTC */
//#define SOC_GPIO_PIN_RTC_M8_INT  TBD

/* ADC */
#define SOC_GPIO_PIN_M8_BATTERY   _PINNUM(0,  4) // P0.04 (AIN2)
#define SOC_GPIO_PIN_M8_ADC_EN    _PINNUM(1,  8) // P1.08

#define SOC_ADC_M8_VOLTAGE_DIV    (1.75F)

/* charger */
#define SOC_GPIO_PIN_M8_CHRG_VUSB _PINNUM(1,  3) // P1.03
#define SOC_GPIO_PIN_M8_CHRG_STAT _PINNUM(1,  5) // P1.05
#define SOC_GPIO_PIN_M8_CHRG_DONE _PINNUM(1,  6) // P1.06
