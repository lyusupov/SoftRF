
/* Peripherals */
#define SOC_GPIO_PIN_CONS_TIP_RX   _PINNUM(0,  9) // P0.09 , No NFC
#define SOC_GPIO_PIN_CONS_TIP_TX   _PINNUM(0, 10) // P0.10 , No NFC

/* u-blox MIA-M10Q */
#define SOC_GPIO_PIN_GNSS_TIP_RX   _PINNUM(1, 11) // P1.11
#define SOC_GPIO_PIN_GNSS_TIP_TX   _PINNUM(1, 12) // P1.12

#define SOC_GPIO_PIN_GNSS_TIP_PPS  SOC_UNUSED_PIN // TBD
#define SOC_GPIO_PIN_GNSS_TIP_PWR  _PINNUM(1, 10) // P1.10 active LOW

/* SPI */
#define SOC_GPIO_PIN_TIP_MOSI      _PINNUM(0, 28) // P0.28
#define SOC_GPIO_PIN_TIP_MISO      _PINNUM(0, 30) // P0.30
#define SOC_GPIO_PIN_TIP_SCK       _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_TIP_SS        _PINNUM(1, 14) // P1.14

/* SX1262 (AcSiP S68F) */
#define SOC_GPIO_PIN_TIP_RST       _PINNUM(0,  2) // P0.02
#define SOC_GPIO_PIN_TIP_DIO1      _PINNUM(0, 29) // P0.29
#define SOC_GPIO_PIN_TIP_DIO2      _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_TIP_BUSY      _PINNUM(0, 31) // P0.31

#define SOC_GPIO_PIN_TIP_RX_EN     _PINNUM(1,  7) // P1.07
#define SOC_GPIO_PIN_TIP_TX_EN     _PINNUM(1, 13) // P1.13

/* GNSS and RF 3.3V power */
#define SOC_GPIO_PIN_TIP_3V3_EN    _PINNUM(0, 14) // P0.14 active HIGH

/* 1st I2C bus (SSD1315 OLED display) */
#define SOC_GPIO_PIN_TIP_OLED_SDA  _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_TIP_OLED_SCL  _PINNUM(0, 15) // P0.15

// 2nd I2C bus (ICM20948, SGM41562)
#define SOC_GPIO_PIN_TIP_SDA       _PINNUM(1,  8) // P1.08
#define SOC_GPIO_PIN_TIP_SCL       _PINNUM(0, 11) // P0.11

// Sensors
#define SOC_GPIO_PIN_TIP_INT1      _PINNUM(0,  7) // P0.07 ICM20948
#define SOC_GPIO_PIN_TIP_INT2      _PINNUM(0, 16) // P0.16 SGM41562 ?
// touch pad
#define SOC_GPIO_PIN_TIP_PAD       _PINNUM(1,  4) // P1.04 TTP223
#define SOC_GPIO_PIN_TIP_AHLB      _PINNUM(0, 13) // P0.13 TTP223

/* button */
#define SOC_GPIO_PIN_TIP_BUTTON    _PINNUM(0, 24) // P0.24 active LOW

/* Haptic */
#define SOC_GPIO_PIN_TIP_MOTOR     _PINNUM(0, 22) // P0.22 active HIGH

/* ZD25WQ32C QSPI flash */
#define SOC_GPIO_PIN_SFL_TIP_MOSI  _PINNUM(0,  6) // P0.06
#define SOC_GPIO_PIN_SFL_TIP_MISO  _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_SFL_TIP_SCK   _PINNUM(0,  4) // P0.04
#define SOC_GPIO_PIN_SFL_TIP_SS    _PINNUM(0, 12) // P0.12
#define SOC_GPIO_PIN_SFL_TIP_HOLD  _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_SFL_TIP_WP    _PINNUM(0,  8) // P0.08

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

/* ADC */
#define SOC_GPIO_PIN_TIP_BATTERY   _PINNUM(0,  5) // P0.05 100K-!00K
#define SOC_GPIO_PIN_TIP_VBAT_EN   _PINNUM(0, 25) // P0.25 active HIGH
