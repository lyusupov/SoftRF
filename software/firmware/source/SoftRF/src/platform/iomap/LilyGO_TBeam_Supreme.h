
/* ESP32-S3 section 1 (core) */
#define SOC_GPIO_PIN_S3_CONS_RX         44
#define SOC_GPIO_PIN_S3_CONS_TX         43

// GNSS module
#define SOC_GPIO_PIN_S3_GNSS_RX         9
#define SOC_GPIO_PIN_S3_GNSS_TX         8
#define SOC_GPIO_PIN_S3_GNSS_PPS        6
#define SOC_GPIO_PIN_S3_GNSS_WAKE       7

// USB
#define SOC_GPIO_PIN_S3_USB_DP          20
#define SOC_GPIO_PIN_S3_USB_DN          19

// SX1262 (HPD16A)
#define SOC_GPIO_PIN_S3_MOSI            11
#define SOC_GPIO_PIN_S3_MISO            13
#define SOC_GPIO_PIN_S3_SCK             12
#define SOC_GPIO_PIN_S3_SS              10
#define SOC_GPIO_PIN_S3_RST             5 /* shared with TFT RST (and/or I2C OLED RST) */
#define SOC_GPIO_PIN_S3_BUSY            4 /* shared with HPD13A DIO2 */
// SX1276 (HPD13A)
#define SOC_GPIO_PIN_S3_DIO0            2
#define SOC_GPIO_PIN_S3_DIO1            1
#define SOC_GPIO_PIN_S3_DIO2            4 /* shared with HPD16A BUSY */

/* 2nd I2C bus (PMU, RTC) */
#define SOC_GPIO_PIN_S3_PMU_SDA         42
#define SOC_GPIO_PIN_S3_PMU_SCL         41
#define SOC_GPIO_PIN_S3_PMU_IRQ         40
#define SOC_GPIO_PIN_S3_RTC_IRQ         14

// 32768 Hz crystal
#define SOC_GPIO_PIN_S3_XP              15
#define SOC_GPIO_PIN_S3_XN              16

// button (BOOT)
#define SOC_GPIO_PIN_S3_BUTTON          0 // "strapping" pin (S)

/* ESP32-S3 section 2 (reserved pins) */
// 17,18 - I2C; 33,34,39,(47 ? - DC) - TFT/EINK; 35,36,37,38 - uSD; 2 - SX1276

// 1st I2C bus (OLED display, sensors)
#define SOC_GPIO_PIN_S3_SDA             17
#define SOC_GPIO_PIN_S3_SCL             18

// IMU
#define SOC_GPIO_PIN_S3_IMU_MOSI        35
#define SOC_GPIO_PIN_S3_IMU_MISO        37
#define SOC_GPIO_PIN_S3_IMU_SCK         36
#define SOC_GPIO_PIN_S3_IMU_SS          34
#define SOC_GPIO_PIN_S3_IMU_INT12       33

// microSD
#define SOC_GPIO_PIN_S3_SD_MOSI         35
#define SOC_GPIO_PIN_S3_SD_MISO         37
#define SOC_GPIO_PIN_S3_SD_SCK          36
#define SOC_GPIO_PIN_S3_SD_SS_DK        38
#define SOC_GPIO_PIN_S3_SD_SS_TBEAM     47
