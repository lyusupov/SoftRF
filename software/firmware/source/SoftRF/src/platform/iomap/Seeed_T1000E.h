
/* Peripherals */
#define SOC_GPIO_PIN_CONS_T1000_RX    _PINNUM(0, 17) // P0.17
#define SOC_GPIO_PIN_CONS_T1000_TX    _PINNUM(0, 16) // P0.16

/* AG3335MN */
#define SOC_GPIO_PIN_GNSS_T1000_RX    _PINNUM(0, 14) // P0.14
#define SOC_GPIO_PIN_GNSS_T1000_TX    _PINNUM(0, 13) // P0.13

#define SOC_GPIO_PIN_GNSS_T1000_PPS   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_GNSS_T1000_EN    _PINNUM(1, 11) // P1.11 active HIGH
#define SOC_GPIO_PIN_GNSS_T1000_OUT   _PINNUM(1, 14) // P1.14 RESETB OUT
#define SOC_GPIO_PIN_GNSS_T1000_RST   _PINNUM(1, 15) // P1.15 active HIGH
#define SOC_GPIO_PIN_GNSS_T1000_VRTC  _PINNUM(0,  8) // P0.08 VRTC EN
#define SOC_GPIO_PIN_GNSS_T1000_SINT  _PINNUM(1, 12) // P1.12 SLEEP Interrupt
#define SOC_GPIO_PIN_GNSS_T1000_RINT  _PINNUM(0, 15) // P0.15 RTC Interrupt

/* SPI */
#define SOC_GPIO_PIN_T1000_MOSI       _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_T1000_MISO       _PINNUM(1,  8) // P1.08
#define SOC_GPIO_PIN_T1000_SCK        _PINNUM(0, 11) // P0.11
#define SOC_GPIO_PIN_T1000_SS         _PINNUM(0, 12) // P0.12

/* LR1110 */
#define SOC_GPIO_PIN_T1000_RST        _PINNUM(1, 10) // P1.10
#define SOC_GPIO_PIN_T1000_DIO9       _PINNUM(1,  1) // P1.01
#define SOC_GPIO_PIN_T1000_BUSY       _PINNUM(0,  7) // P0.07

/* I2C */
#define SOC_GPIO_PIN_T1000_SDA        _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_T1000_SCL        _PINNUM(0, 27) // P0.27

/* button */
#define SOC_GPIO_PIN_T1000_BUTTON     _PINNUM(0,  6) // P0.06 Key IO, must be configured as input_pulldown

/* LED */
#define SOC_GPIO_LED_T1000_GREEN      _PINNUM(0, 24) // P0.24 active HIGH
#define SOC_GPIO_LED_T1000_RED        _PINNUM(0,  3) // P0.03 NC?

/* ADC */
#define SOC_GPIO_PIN_T1000_BATTERY    _PINNUM(0,  2) // P0.02 Baterry level dectect
#define SOC_GPIO_PIN_T1000_VCC        _PINNUM(0,  4) // P0.04 VCC voltage dectect
#define SOC_GPIO_PIN_T1000_TEMP       _PINNUM(0, 31) // P0.31 Temperature Sensor ADC input
#define SOC_GPIO_PIN_T1000_LUX        _PINNUM(0, 29) // P0.29 Light Sensor ADC input

#define SOC_ADC_T1000_VOLTAGE_DIV     (2.0F) // 100K + 100K voltage divider on VBAT

/* battery charger */
#define SOC_GPIO_PIN_T1000_CHG_PWR    _PINNUM(0,  5) // P0.05 Charger insert dectect, must be configured as no pullup or pulldown
#define SOC_GPIO_PIN_T1000_CHG_STATUS _PINNUM(1,  3) // P1.03 active LOW
#define SOC_GPIO_PIN_T1000_CHG_DONE   _PINNUM(1,  4) // P1.04 active LOW

/* buzzer */
#define SOC_GPIO_PIN_T1000_BUZZER     _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_T1000_BUZZER_EN  _PINNUM(1,  5) // P1.05

/* QMA6100P */
#define SOC_GPIO_PIN_T1000_ACC_EN     _PINNUM(1,  7) // P1.07 active HIGH
#define SOC_GPIO_PIN_T1000_ACC_INT    _PINNUM(1,  2) // P1.02

/* Sensors */
#define SOC_GPIO_PIN_T1000_3V3_EN     _PINNUM(1,  6) // P1.06

/* GD25Q64EN (or P25Q16H ?) QSPI flash */
#define SOC_GPIO_PIN_SFL_T1000_MOSI   _PINNUM(0, 21) // P0.21
#define SOC_GPIO_PIN_SFL_T1000_MISO   _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_SFL_T1000_SCK    _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_SFL_T1000_SS     _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_SFL_T1000_HOLD   _PINNUM(1,  0) // P1.00
#define SOC_GPIO_PIN_SFL_T1000_WP     _PINNUM(0, 23) // P0.23

#define SOC_GPIO_PIN_SFL_T1000_EN     _PINNUM(1, 13) // P1.13 active HIGH

/* misc. */
#define SOC_GPIO_PIN_T1000_MCU_RESET  _PINNUM(0, 18) // P0.18
