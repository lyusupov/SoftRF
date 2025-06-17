
/* Peripherals */
#define SOC_GPIO_PIN_CONS_L1_RX   _PINNUM(0,  9) // P0.09 , No NFC
#define SOC_GPIO_PIN_CONS_L1_TX   _PINNUM(0, 10) // P0.10 , No NFC

/* L76K */
#define SOC_GPIO_PIN_GNSS_L1_RX   _PINNUM(0, 27) // P0.27
#define SOC_GPIO_PIN_GNSS_L1_TX   _PINNUM(0, 26) // P0.26

#define SOC_GPIO_PIN_GNSS_L1_PPS  SOC_UNUSED_PIN // TBD
#define SOC_GPIO_PIN_GNSS_L1_WKE  _PINNUM(1,  9) // P1.09

/* SPI */
#define SOC_GPIO_PIN_L1_MOSI      _PINNUM(0, 28) // P0.28
#define SOC_GPIO_PIN_L1_MISO      _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_L1_SCK       _PINNUM(0, 30) // P0.30
#define SOC_GPIO_PIN_L1_SS        _PINNUM(1, 14) // P1.14

/* SX1262 */
#define SOC_GPIO_PIN_L1_RST       _PINNUM(1,  7) // P1.07
#define SOC_GPIO_PIN_L1_DIO1      _PINNUM(0,  7) // P0.07
#define SOC_GPIO_PIN_L1_BUSY      _PINNUM(1, 10) // P1.10
#define SOC_GPIO_PIN_L1_RX_EN     _PINNUM(1,  8) // P1.08

/* 1st I2C bus (SH1106 OLED display) */
#define SOC_GPIO_PIN_L1_OLED_SDA  _PINNUM(0,  6) // P0.06
#define SOC_GPIO_PIN_L1_OLED_SCL  _PINNUM(0,  5) // P0.05

// 2nd I2C bus (ext. sensors)
#define SOC_GPIO_PIN_L1_SDA       _PINNUM(1, 11) // P1.11
#define SOC_GPIO_PIN_L1_SCL       _PINNUM(1, 12) // P1.12

/* buttons */
#define SOC_GPIO_PIN_L1_BUTTON    _PINNUM(0,  8) // P0.08

/* LEDs */
#define SOC_GPIO_LED_L1_GREEN     _PINNUM(1,  1) // P1.01, active HIGH
#define SOC_GPIO_LED_L1_BLUE      _PINNUM(1,  2) // P1.02, active HIGH

/* buzzer */
#define SOC_GPIO_PIN_L1_BUZZER    _PINNUM(1,  0) // P1.00

/* P25Q16SH SPI flash */
#define SOC_GPIO_PIN_SFL_L1_MOSI  _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_SFL_L1_MISO  _PINNUM(0, 24) // P0.24
#define SOC_GPIO_PIN_SFL_L1_SCK   _PINNUM(0, 21) // P0.21
#define SOC_GPIO_PIN_SFL_L1_SS    _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_SFL_L1_HOLD  _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_SFL_L1_WP    _PINNUM(0, 22) // P0.22

/* ADC */
#define SOC_GPIO_PIN_L1_BATTERY   _PINNUM(0, 31) // P0.31

#define SOC_GPIO_PIN_L1_VBAT_EN   _PINNUM(0,  4) // P0.04, active HIGH

/* trackball */
#define SOC_GPIO_PIN_L1_TB_UP     _PINNUM(1,  4) // P1.04
#define SOC_GPIO_PIN_L1_TB_DOWN   _PINNUM(0, 12) // P0.12
#define SOC_GPIO_PIN_L1_TB_LEFT   _PINNUM(0, 11) // P0.11
#define SOC_GPIO_PIN_L1_TB_RIGHT  _PINNUM(1,  3) // P1.03
#define SOC_GPIO_PIN_L1_TB_CENTER _PINNUM(1,  5) // P1.05
