/* https://www.elecrow.com/download/product/CIL12901M/ThinkNode%20M1_LoRa_Meshtastic_Transceiver_DataSheet.pdf */

/* Peripherals */
#define SOC_GPIO_PIN_CONS_M1_RX   _PINNUM(0,  9) // P0.09 , No NFC
#define SOC_GPIO_PIN_CONS_M1_TX   _PINNUM(0, 10) // P0.10 , No NFC

/* L76K */
#define SOC_GPIO_PIN_GNSS_M1_RX   _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_GNSS_M1_TX   _PINNUM(1,  8) // P1.08

#define SOC_GPIO_PIN_GNSS_M1_PPS  SOC_UNUSED_PIN // TBD
#define SOC_GPIO_PIN_GNSS_M1_WKE  _PINNUM(1,  2) // P1.02
#define SOC_GPIO_PIN_GNSS_M1_RST  _PINNUM(1,  5) // P1.05
#define SOC_GPIO_PIN_GNSS_M1_SW   _PINNUM(1,  1) // P1.01

/* SPI */
#define SOC_GPIO_PIN_M1_MOSI      _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_M1_MISO      _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_M1_SCK       _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_M1_SS        _PINNUM(0, 24) // P0.24

/* SX1262 */
#define SOC_GPIO_PIN_M1_RST       _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_M1_DIO1      _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_M1_DIO3      _PINNUM(0, 21) // P0.21
#define SOC_GPIO_PIN_M1_BUSY      _PINNUM(0, 17) // P0.17

/* E-paper */
#define SOC_GPIO_PIN_EPD_M1_MISO  _PINNUM(0, 11) // P0.11 NC ?
#define SOC_GPIO_PIN_EPD_M1_MOSI  _PINNUM(0, 29) // P0.29
#define SOC_GPIO_PIN_EPD_M1_SCK   _PINNUM(0, 31) // P0.31
#define SOC_GPIO_PIN_EPD_M1_SS    _PINNUM(0, 30) // P0.30
#define SOC_GPIO_PIN_EPD_M1_DC    _PINNUM(0, 28) // P0.28
#define SOC_GPIO_PIN_EPD_M1_RST   _PINNUM(0,  2) // P0.02
#define SOC_GPIO_PIN_EPD_M1_BUSY  _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_EPD_M1_BLGT  _PINNUM(1, 11) // P1.11

/* Power: EINK, RGB, RF, FLASH, GNSS, SENSOR */
#define SOC_GPIO_PIN_IO_M1_PWR    _PINNUM(0, 12) // P0.12

/* I2C int. (same that T-Echo has) */
#define SOC_GPIO_PIN_M1_SDA_INT   _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_M1_SCL_INT   _PINNUM(0, 27) // P0.27

/* buttons */
#define SOC_GPIO_PIN_M1_BUTTON1   _PINNUM(1,  7) // P1.07
#define SOC_GPIO_PIN_M1_BUTTON2   _PINNUM(1, 10) // P1.10

/* LED */
#define SOC_GPIO_LED_M1_RED       _PINNUM(1,  6) // P1.06, active HIGH
#define SOC_GPIO_LED_M1_RED_PWR   _PINNUM(1,  4) // P1.04
#define SOC_GPIO_LED_M1_BLUE      _PINNUM(0, 13) // P0.13, active HIGH
/* NC ? */
#define SOC_GPIO_LED_M1_1         _PINNUM(0, 14) // P0.14
#define SOC_GPIO_LED_M1_2         _PINNUM(0, 15) // P0.15

/* buzzer */
#define SOC_GPIO_PIN_M1_BUZZER    _PINNUM(0,  6) // P0.06

/* MX25R1635F (?) or WP25R1635F (?) SPI flash */
#define SOC_GPIO_PIN_SFL_M1_MOSI  _PINNUM(1, 12) // P1.12
#define SOC_GPIO_PIN_SFL_M1_MISO  _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_SFL_M1_SCK   _PINNUM(1, 14) // P1.14
#define SOC_GPIO_PIN_SFL_M1_SS    _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_SFL_M1_HOLD  _PINNUM(0,  5) // P0.05
#define SOC_GPIO_PIN_SFL_M1_WP    _PINNUM(0,  7) // P0.07

/* RTC */
#define SOC_GPIO_PIN_RTC_M1_INT   _PINNUM(0, 16) // P0.16

/* ADC */
#define SOC_GPIO_PIN_M1_BATTERY   _PINNUM(0,  4) // P0.04 (AIN2)

/* digital input */
#define SOC_GPIO_PIN_M1_VBAT_SEN  _PINNUM(0,  8) // P0.08
#define SOC_GPIO_PIN_M1_VUSB_SEN  _PINNUM(1,  3) // P1.03
#define SOC_GPIO_PIN_M1_VGPS_SEN  _PINNUM(1,  1) // P1.01
