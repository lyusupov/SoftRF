
/* Peripherals */
#define SOC_GPIO_PIN_CONS_T1_RX   0              // N/A
#define SOC_GPIO_PIN_CONS_T1_TX   0              // N/A

// GNSS module ( UC6580 )
#define SOC_GPIO_PIN_GNSS_T1_RX   _PINNUM(0,  8) // P0.08
#define SOC_GPIO_PIN_GNSS_T1_TX   _PINNUM(0,  7) // P0.07

#define SOC_GPIO_PIN_GNSS_T1_PPS  _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_GNSS_T1_EN   _PINNUM(0,  4) // P0.04 active LOW
#define SOC_GPIO_PIN_GNSS_T1_RST  _PINNUM(0, 26) // P0.26

/* SPI */
#define SOC_GPIO_PIN_T1_MOSI      _PINNUM(1, 14) // P1.14
#define SOC_GPIO_PIN_T1_MISO      _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_T1_SCK       _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_T1_SS        _PINNUM(1, 11) // P1.11

/* SX1262, TCXO PWR = 1.8 */
#define SOC_GPIO_PIN_T1_RST       _PINNUM(0,  2) // P0.02
#define SOC_GPIO_PIN_T1_DIO1      _PINNUM(0, 31) // P0.31
#define SOC_GPIO_PIN_T1_BUSY      _PINNUM(0, 29) // P0.29

// TFT 80x160 ST7735
#define SOC_GPIO_PIN_T1_TFT_MOSI   _PINNUM(0, 24) // P0.24
#define SOC_GPIO_PIN_T1_TFT_MISO   _PINNUM(1,  8) // P1.08 NC
#define SOC_GPIO_PIN_T1_TFT_SCK    _PINNUM(1,  0) // P1.00
#define SOC_GPIO_PIN_T1_TFT_SS     _PINNUM(0, 12) // P0.12
#define SOC_GPIO_PIN_T1_TFT_DC     _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_T1_TFT_RST    _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_T1_TFT_BLGT   _PINNUM(0, 15) // P0.15
#define SOC_GPIO_PIN_T1_TFT_EN     _PINNUM(0, 13) // P0.13 active HIGH

/* I2C */
#define SOC_GPIO_PIN_T1_SDA       _PINNUM(1,  3) // P1.03
#define SOC_GPIO_PIN_T1_SCL       _PINNUM(0, 10) // P0.10

// Sensors (ICM42607P IMU and MMC5983MA compass)
#define SOC_GPIO_PIN_T1_SENS_INT1 _PINNUM(1,  1) // P1.01
#define SOC_GPIO_PIN_T1_SENS_INT2 _PINNUM(1,  7) // P1.07
#define SOC_GPIO_PIN_T1_SENS_PWR  _PINNUM(1,  6) // P1.06 active LOW

/* buttons */
#define SOC_GPIO_PIN_T1_BUTTON1   _PINNUM(1, 10) // P1.10
#define SOC_GPIO_PIN_T1_BUTTON2   _PINNUM(0, 14) // P0.14

/* LED */
#define SOC_GPIO_LED_T1_GREEN     _PINNUM(0, 16) // P0.16 active LOW

/* buzzer */
#define SOC_GPIO_PIN_T1_BUZZER    _PINNUM(0,  9) // P0.09
#define SOC_GPIO_PIN_T1_VM_1      _PINNUM(1,  2) // P1.02
#define SOC_GPIO_PIN_T1_VM_2      _PINNUM(1,  5) // P1.05

/* ADC */
#define SOC_GPIO_PIN_T1_BATTERY   _PINNUM(0,  5) // P0.05
#define SOC_GPIO_PIN_T1_ADC_EN    _PINNUM(0, 11) // P0.11 active HIGH

#define SOC_ADC_T1_VOLTAGE_DIV    (4.916F) // 390K + 100K voltage divider on VBAT
