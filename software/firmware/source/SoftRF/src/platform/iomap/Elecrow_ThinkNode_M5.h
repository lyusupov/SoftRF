
/* Peripherals */
#define SOC_GPIO_PIN_M5_CONS_RX     44
#define SOC_GPIO_PIN_M5_CONS_TX     43

/* L76K */
#define SOC_GPIO_PIN_M5_GNSS_RX     19 /* USB D- */
#define SOC_GPIO_PIN_M5_GNSS_TX     20 /* USB D+ */

#define SOC_GPIO_PIN_M5_GNSS_PPS    SOC_UNUSED_PIN // TBD
#define SOC_GPIO_PIN_M5_GNSS_WKE    11
#define SOC_GPIO_PIN_M5_GNSS_RST    13
#define SOC_GPIO_PIN_M5_GNSS_SW     10

/* SPI */
#define SOC_GPIO_PIN_M5_MOSI        15
#define SOC_GPIO_PIN_M5_MISO        7
#define SOC_GPIO_PIN_M5_SCK         16
#define SOC_GPIO_PIN_M5_SS          17

/* SX1262 */
#define SOC_GPIO_PIN_M5_RST         6
#define SOC_GPIO_PIN_M5_DIO1        4
#define SOC_GPIO_PIN_M5_DIO3        46
#define SOC_GPIO_PIN_M5_BUSY        5

/* E-paper */
#define SOC_GPIO_PIN_M5_EPD_MISO    0 // NC, shared with BOOT
#define SOC_GPIO_PIN_M5_EPD_MOSI    45
#define SOC_GPIO_PIN_M5_EPD_SCK     38
#define SOC_GPIO_PIN_M5_EPD_SS      39
#define SOC_GPIO_PIN_M5_EPD_DC      40
#define SOC_GPIO_PIN_M5_EPD_RST     41
#define SOC_GPIO_PIN_M5_EPD_BUSY    42

/* I2C bus 1 */
#define SOC_GPIO_PIN_M5_SDA         2
#define SOC_GPIO_PIN_M5_SCL         1

/* I2C bus 2 (PCA9557) */
#define SOC_GPIO_PIN_M5_SDA2        48
#define SOC_GPIO_PIN_M5_SCL2        47

/* buttons */
#define SOC_GPIO_PIN_M5_BUTTON_1    14
#define SOC_GPIO_PIN_M5_BUTTON_2    21
#define SOC_GPIO_PIN_M5_BUTTON_BOOT 0

/* buzzer */
#define SOC_GPIO_PIN_M5_BUZZER      9

/* RTC */
#define SOC_GPIO_PIN_M5_RTC_INT     3

/* ADC */
#define SOC_GPIO_PIN_M5_BATTERY     8

/* digital input */
#define SOC_GPIO_PIN_M5_VBAT_SEN    18
#define SOC_GPIO_PIN_M5_VUSB_SEN    12
#define SOC_GPIO_PIN_M5_VGPS_SEN    10

/* GPIO expansion (PCA9557) */
#define SOC_EXPIO_LED_M5_BLUE       1 // active HIGH
#define SOC_EXPIO_LED_M5_RED_PWR    2
#define SOC_EXPIO_LED_M5_RED        3 // active HIGH
#define SOC_EXPIO_PIN_M5_IO_EN      4 // active HIGH
#define SOC_EXPIO_PIN_M5_EPD_EN     5 // active HIGH
