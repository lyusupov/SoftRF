
/* ESP32-S3-MINI-1U-N4 */
#define SOC_GPIO_PIN_T3S3_CONS_RX       39
#define SOC_GPIO_PIN_T3S3_CONS_TX       40

// GNSS module (ext.)
#define SOC_GPIO_PIN_T3S3_GNSS_RX       44
#define SOC_GPIO_PIN_T3S3_GNSS_TX       43
#define SOC_GPIO_PIN_T3S3_GNSS_PPS      38

// USB CDC/JTAG
#define SOC_GPIO_PIN_T3S3_USB_DP        20
#define SOC_GPIO_PIN_T3S3_USB_DN        19

// RADIO SPI
#define SOC_GPIO_PIN_T3S3_MOSI          6
#define SOC_GPIO_PIN_T3S3_MISO          3
#define SOC_GPIO_PIN_T3S3_SCK           5
#define SOC_GPIO_PIN_T3S3_SS            7

// SX1262 or LR1121
#define SOC_GPIO_PIN_T3S3_RST           8
#define SOC_GPIO_PIN_T3S3_BUSY          34
#define SOC_GPIO_PIN_T3S3_DIO1          33 /* V1.3 LR1121 - DIO11 */
#define SOC_GPIO_PIN_T3S3_DIO2          9
#define SOC_GPIO_PIN_T3S3_ANT_RX        21
#define SOC_GPIO_PIN_T3S3_ANT_TX        10

#define SOC_GPIO_PIN_T3S3_3V3EN         35

// 1st I2C bus (ext. sensors)
#define SOC_GPIO_PIN_T3S3_SDA           41 /* 46 ? */
#define SOC_GPIO_PIN_T3S3_SCL           42

/* 2nd I2C bus (OLED display) */
#define SOC_GPIO_PIN_T3S3_OLED_SDA      18
#define SOC_GPIO_PIN_T3S3_OLED_SCL      17

// status LED
#define SOC_GPIO_PIN_T3S3_LED           37

// EPD
#define SOC_GPIO_PIN_T3S3_EPD_MOSI      11
#define SOC_GPIO_PIN_T3S3_EPD_MISO      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_T3S3_EPD_SCK       14
#define SOC_GPIO_PIN_T3S3_EPD_SS        15
#define SOC_GPIO_PIN_T3S3_EPD_DC        16
#define SOC_GPIO_PIN_T3S3_EPD_RST       47
#define SOC_GPIO_PIN_T3S3_EPD_BUSY      48

// microSD
#define SOC_GPIO_PIN_T3S3_SD_MOSI       11
#define SOC_GPIO_PIN_T3S3_SD_MISO       2
#define SOC_GPIO_PIN_T3S3_SD_SCK        14
#define SOC_GPIO_PIN_T3S3_SD_SS         13
#define SOC_GPIO_PIN_T3S3_SD_D1         4
#define SOC_GPIO_PIN_T3S3_SD_D2         12

// ADC
#define SOC_GPIO_PIN_T3S3_BATTERY       1 /* 100K/100K */
#define SOC_GPIO_PIN_T3S3_SOLAR         4 /* V1.3 only, 100K/100K */
