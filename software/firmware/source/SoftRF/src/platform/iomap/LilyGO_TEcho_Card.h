
/* Peripherals */
#define SOC_GPIO_PIN_CONS_TEC_RX   _PINNUM(0,  9) // P0.09 , No NFC
#define SOC_GPIO_PIN_CONS_TEC_TX   _PINNUM(0, 10) // P0.10 , No NFC

/* L76K */
#define SOC_GPIO_PIN_GNSS_TEC_RX   _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_GNSS_TEC_TX   _PINNUM(0, 21) // P0.21

#define SOC_GPIO_PIN_GNSS_TEC_PPS  _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_GNSS_TEC_WKE  _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_GNSS_TEC_PWR  _PINNUM(1, 15) // P1.15 active LOW ?
#define SOC_GPIO_PIN_GNSS_TEC_SW   _PINNUM(0, 29) // P0.29 active HIGH ?

/* SPI */
#define SOC_GPIO_PIN_TEC_MOSI      _PINNUM(0, 15) // P0.15
#define SOC_GPIO_PIN_TEC_MISO      _PINNUM(0, 17) // P0.17
#define SOC_GPIO_PIN_TEC_SCK       _PINNUM(0, 13) // P0.13
#define SOC_GPIO_PIN_TEC_SS        _PINNUM(0, 11) // P0.11

/* SX1262 (AcSiP S68F) */
#define SOC_GPIO_PIN_TEC_RST       _PINNUM(0,  7) // P0.07
#define SOC_GPIO_PIN_TEC_DIO1      _PINNUM(1,  8) // P1.08
#define SOC_GPIO_PIN_TEC_DIO2      _PINNUM(0,  5) // P0.05
#define SOC_GPIO_PIN_TEC_BUSY      _PINNUM(0, 14) // P0.14

#define SOC_GPIO_PIN_TEC_RX_EN     _PINNUM(1,  1) // P1.01 active HIGH
#define SOC_GPIO_PIN_TEC_TX_EN     _PINNUM(0, 27) // P0.27 active HIGH

/* GNSS and RF 3.3V power */
#define SOC_GPIO_PIN_TEC_3V3_EN    _PINNUM(0, 30) // P0.30 active HIGH

/* 1st I2C bus (SSD1315 OLED display, ICM20948 IMU) */
#define SOC_GPIO_PIN_TEC_SDA       _PINNUM(1,  4) // P1.04
#define SOC_GPIO_PIN_TEC_SCL       _PINNUM(1,  2) // P1.02
// Sensor
#define SOC_GPIO_PIN_TEC_INT       _PINNUM(1, 13) // P1.13 ICM20948

/* button(s) */
#define SOC_GPIO_PIN_TEC_BUTTON    _PINNUM(1, 10) // P1.10 active LOW
#define SOC_GPIO_PIN_TEC_BTN1      _PINNUM(0, 24) // P0.24 active LOW

/* NeoPixels */
#define SOC_GPIO_LED_TEC_LED1      _PINNUM(0, 28) // P0.28
#define SOC_GPIO_LED_TEC_LED2      _PINNUM(1,  7) // P1.07
#define SOC_GPIO_LED_TEC_LED3      _PINNUM(1, 12) // P1.12

/* buzzer */
#define SOC_GPIO_PIN_TEC_BUZZER    _PINNUM(1,  6) // P1.06

/* I2S */
#define SOC_GPIO_PIN_I2S_TEC_LRCK  _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_I2S_TEC_BCK   _PINNUM(0, 16) // P0.16
#define SOC_GPIO_PIN_I2S_TEC_DOUT  _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_I2S_TEC_MCK   SOC_UNUSED_PIN
// MAX98375A
#define SOC_GPIO_PIN_I2S_TEC_EN    _PINNUM(1, 11) // P1.11 active HIGH
#define SOC_GPIO_PIN_I2S_TEC_PWR   _PINNUM(0,  3) // P0.03 active HIGH

/* PDM Mic */
#define SOC_GPIO_PIN_PDM_TEC_DATA  _PINNUM(1,  5) // P1.05
#define SOC_GPIO_PIN_PDM_TEC_CLK   _PINNUM(1,  3) // P1.03

/* ZD25WQ32C QSPI flash */
#define SOC_GPIO_PIN_SFL_TEC_MOSI  _PINNUM(0,  6) // P0.06
#define SOC_GPIO_PIN_SFL_TEC_MISO  _PINNUM(0,  8) // P0.08
#define SOC_GPIO_PIN_SFL_TEC_SCK   _PINNUM(0,  4) // P0.04
#define SOC_GPIO_PIN_SFL_TEC_SS    _PINNUM(0, 12) // P0.12
#define SOC_GPIO_PIN_SFL_TEC_HOLD  _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_SFL_TEC_WP    _PINNUM(1,  9) // P1.09

#if !defined(ZD25WQ32C)
// Settings for the Zetta Device ZD25WQ32C 4MiB SPI flash.
// Datasheet: http://en.zettadevice.com/uploads/files/1009/WQ32C/1665301640977617c282766299.pdf
#define ZD25WQ32C                                                              \
  {                                                                            \
    .total_size = (1UL << 22), /* 4 MiB */                                     \
    .start_up_time_us = 12000, .manufacturer_id = 0xba,                        \
    .memory_type = 0x60, .capacity = 0x16, .max_clock_speed_mhz = 104,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }
#endif /* ZD25WQ32C */

/* ADC */
#define SOC_GPIO_PIN_TEC_BATTERY   _PINNUM(0,  2) // P0.02 100K-!00K
#define SOC_GPIO_PIN_TEC_VBAT_EN   _PINNUM(0, 31) // P0.31 active HIGH
