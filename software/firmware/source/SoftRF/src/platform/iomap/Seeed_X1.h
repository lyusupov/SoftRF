
/* Peripherals */
#define SOC_GPIO_PIN_CONS_X1_RX    _PINNUM(0, 17) // P0.17
#define SOC_GPIO_PIN_CONS_X1_TX    _PINNUM(0, 16) // P0.16

/* AG3335MN, L1 + L5 */
#define SOC_GPIO_PIN_GNSS_X1_RX    _PINNUM(0, 14) // P0.14
#define SOC_GPIO_PIN_GNSS_X1_TX    _PINNUM(0, 13) // P0.13

#define SOC_GPIO_PIN_GNSS_X1_PPS   _PINNUM(0,  4) // P0.04
#define SOC_GPIO_PIN_GNSS_X1_EN    _PINNUM(1, 11) // P1.11 active HIGH
#define SOC_GPIO_PIN_GNSS_X1_RST   _PINNUM(0,  8) // P0.08 active HIGH
#define SOC_GPIO_PIN_GNSS_X1_VRTC  _PINNUM(1, 13) // P1.13 VRTC EN
#define SOC_GPIO_PIN_GNSS_X1_SINT  _PINNUM(0, 30) // P0.30 SLEEP Interrupt
#define SOC_GPIO_PIN_GNSS_X1_RINT  _PINNUM(0, 29) // P0.29 RTC Interrupt

/* SPI */
#define SOC_GPIO_PIN_X1_MOSI       _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_X1_MISO       _PINNUM(1,  8) // P1.08
#define SOC_GPIO_PIN_X1_SCK        _PINNUM(0, 11) // P0.11
#define SOC_GPIO_PIN_X1_SS         _PINNUM(0, 12) // P0.12

/* LR2021 */
#define SOC_GPIO_PIN_X1_RST        _PINNUM(1, 10) // P1.10
#define SOC_GPIO_PIN_X1_DIO8       _PINNUM(1,  1) // P1.01
#define SOC_GPIO_PIN_X1_BUSY       _PINNUM(0,  7) // P0.07

/* I2C #0 (SPA06, LSM6, BMM350) */
#define SOC_GPIO_PIN_X1_SDA        _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_X1_SCL        _PINNUM(1, 14) // P1.14

/* I2C #1 (YSN8900 RTC) */
#define SOC_GPIO_PIN_X1_RTC_SDA    _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_X1_RTC_SCL    _PINNUM(0, 27) // P0.27
#define SOC_GPIO_PIN_X1_RTC_EN     _PINNUM(0,  9) // P0.09
#define SOC_GPIO_PIN_X1_RTC_INT    _PINNUM(0, 31) // P0.31

/* button */
#define SOC_GPIO_PIN_X1_BUTTON     _PINNUM(0,  6) // P0.06 Key IO, must be configured as input_pulldown

/* LED */
#define SOC_GPIO_LED_X1_RED        _PINNUM(0,  3) // P0.03 active HIGH
#define SOC_GPIO_LED_X1_GREEN      _PINNUM(0, 24) // P0.24 active HIGH
#define SOC_GPIO_LED_X1_BLUE       _PINNUM(0, 28) // P0.28 active HIGH

/* ADC */
#define SOC_GPIO_PIN_X1_BATTERY    _PINNUM(0,  2) // P0.02 Battery
#define SOC_GPIO_PIN_X1_VBAT_EN    _PINNUM(1,  6) // P1.06

#define SOC_ADC_X1_VOLTAGE_DIV     (2.0F) // 100K + 100K voltage divider on VBAT

/* SGM40571 battery charger */
#define SOC_GPIO_PIN_X1_CHG_VUSB   _PINNUM(0,  5) // P0.05
#define SOC_GPIO_PIN_X1_CHG_STATUS _PINNUM(1,  3) // P1.03
#define SOC_GPIO_PIN_X1_CHG_DONE   _PINNUM(1,  4) // P1.04

/* buzzer */
#define SOC_GPIO_PIN_X1_BUZZER     _PINNUM(0, 25) // P0.25

/* DRV2605 */
#define SOC_GPIO_PIN_X1_HAPTIC_EN  _PINNUM(1,  5) // P1.05

// Sensors
#define SOC_GPIO_PIN_X1_INT1       _PINNUM(1,  2) // P1.02 LSM6
#define SOC_GPIO_PIN_X1_INT2       _PINNUM(1, 12) // P1.12 BMM350

/* Power control for buzzer, sensors (SPA06, LSM6, BMM350) and haptic (DRV2605) */
#define SOC_GPIO_PIN_X1_3V3_EN     _PINNUM(1,  7) // P1.07

/* GD25Q64E QSPI flash */
#define SOC_GPIO_PIN_SFL_X1_MOSI   _PINNUM(0, 21) // P0.21
#define SOC_GPIO_PIN_SFL_X1_MISO   _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_SFL_X1_SCK    _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_SFL_X1_SS     _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_SFL_X1_HOLD   _PINNUM(1,  0) // P1.00
#define SOC_GPIO_PIN_SFL_X1_WP     _PINNUM(0, 23) // P0.23

#define SOC_GPIO_PIN_SFL_X1_EN     _PINNUM(0, 15) // P0.15 active HIGH

/* misc. */
#define SOC_GPIO_PIN_X1_MCU_RESET  _PINNUM(0, 18) // P0.18
