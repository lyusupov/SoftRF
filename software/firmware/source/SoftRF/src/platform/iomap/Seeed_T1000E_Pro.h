
/* Peripherals */
#define SOC_GPIO_PIN_CONS_T1KEP_RX    _PINNUM(0, 17) // P0.17
#define SOC_GPIO_PIN_CONS_T1KEP_TX    _PINNUM(0, 16) // P0.16

/* AG3335MN, L1 + L5 */
#define SOC_GPIO_PIN_GNSS_T1KEP_RX    _PINNUM(0, 14) // P0.14
#define SOC_GPIO_PIN_GNSS_T1KEP_TX    _PINNUM(0, 13) // P0.13

#define SOC_GPIO_PIN_GNSS_T1KEP_PPS   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_GNSS_T1KEP_EN    _PINNUM(1, 11) // P1.11 active HIGH
#define SOC_GPIO_PIN_GNSS_T1KEP_RST   _PINNUM(0,  8) // P0.08 active HIGH
#define SOC_GPIO_PIN_GNSS_T1KEP_VRTC  _PINNUM(1, 13) // P1.13 VRTC EN
#define SOC_GPIO_PIN_GNSS_T1KEP_SINT  _PINNUM(0, 30) // P0.30 SLEEP Interrupt
#define SOC_GPIO_PIN_GNSS_T1KEP_RINT  _PINNUM(0, 29) // P0.29 RTC Interrupt

/* SPI */
#define SOC_GPIO_PIN_T1KEP_MOSI       _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_T1KEP_MISO       _PINNUM(1,  8) // P1.08
#define SOC_GPIO_PIN_T1KEP_SCK        _PINNUM(0, 11) // P0.11
#define SOC_GPIO_PIN_T1KEP_SS         _PINNUM(0, 12) // P0.12

/* LR2021 */
#define SOC_GPIO_PIN_T1KEP_RST        _PINNUM(1, 10) // P1.10
#define SOC_GPIO_PIN_T1KEP_DIO8       _PINNUM(1,  1) // P1.01
#define SOC_GPIO_PIN_T1KEP_BUSY       _PINNUM(0,  7) // P0.07

/* I2C */
#define SOC_GPIO_PIN_T1KEP_SDA        _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_T1KEP_SCL        _PINNUM(1, 14) // P1.14

/* button */
#define SOC_GPIO_PIN_T1KEP_BUTTON     _PINNUM(0,  6) // P0.06 Key IO, must be configured as input_pulldown

/* LED */
#define SOC_GPIO_LED_T1KEP_GREEN      _PINNUM(0, 24) // P0.24 active HIGH
#define SOC_GPIO_LED_T1KEP_BLUE       _PINNUM(0, 28) // P0.28 active HIGH

/* ADC */
#define SOC_GPIO_PIN_T1KEP_BATTERY    _PINNUM(0,  2) // P0.02 Battery
#define SOC_GPIO_PIN_T1KEP_VCC        _PINNUM(0,  4) // P0.04 VCC

#define SOC_GPIO_PIN_T1KEP_VBAT_EN    _PINNUM(1,  6) // P1.06

#define SOC_ADC_T1KEP_VOLTAGE_DIV     (2.0F) // 100K + 100K voltage divider on VBAT

/* RTC */
#define SOC_GPIO_PIN_T1KEP_RTC_EN     _PINNUM(0,  9) // P0.09

/* battery charger */
#define SOC_GPIO_PIN_T1KEP_CHG_PWR    _PINNUM(0,  5) // P0.05
#define SOC_GPIO_PIN_T1KEP_CHG_STATUS _PINNUM(1,  3) // P1.03

/* buzzer */
#define SOC_GPIO_PIN_T1KEP_BUZZER     _PINNUM(0, 25) // P0.25

/* DRV2605 */
#define SOC_GPIO_PIN_T1KEP_HAPTIC_EN  _PINNUM(1,  5) // P1.05

/* Sensors */
#define SOC_GPIO_PIN_T1KEP_3V3_EN     _PINNUM(1,  7) // P1.07

/* GD25Q64C QSPI flash */
#define SOC_GPIO_PIN_SFL_T1KEP_MOSI   _PINNUM(0, 21) // P0.21
#define SOC_GPIO_PIN_SFL_T1KEP_MISO   _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_SFL_T1KEP_SCK    _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_SFL_T1KEP_SS     _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_SFL_T1KEP_HOLD   _PINNUM(1,  0) // P1.00
#define SOC_GPIO_PIN_SFL_T1KEP_WP     _PINNUM(0, 23) // P0.23

#define SOC_GPIO_PIN_SFL_T1KEP_EN     _PINNUM(0, 15) // P0.15 active HIGH

/* misc. */
#define SOC_GPIO_PIN_T1KEP_MCU_RESET  _PINNUM(0, 18) // P0.18
