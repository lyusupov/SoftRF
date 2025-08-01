
/* Peripherals */
#define SOC_GPIO_PIN_CONS_M5_RX   44
#define SOC_GPIO_PIN_CONS_M5_TX   43

/* L76K */
#define SOC_GPIO_PIN_GNSS_M5_RX   20
#define SOC_GPIO_PIN_GNSS_M5_TX   19

#define SOC_GPIO_PIN_GNSS_M5_PPS  SOC_UNUSED_PIN // TBD
#define SOC_GPIO_PIN_GNSS_M5_WKE  11
#define SOC_GPIO_PIN_GNSS_M5_RST  13
#define SOC_GPIO_PIN_GNSS_M5_SW   10

/* SPI */
#define SOC_GPIO_PIN_M5_MOSI      15
#define SOC_GPIO_PIN_M5_MISO      7
#define SOC_GPIO_PIN_M5_SCK       16
#define SOC_GPIO_PIN_M5_SS        17

/* SX1262 */
#define SOC_GPIO_PIN_M5_RST       6
#define SOC_GPIO_PIN_M5_DIO1      4
#define SOC_GPIO_PIN_M5_DIO3      46
#define SOC_GPIO_PIN_M5_BUSY      5

/* E-paper */
#define SOC_GPIO_PIN_EPD_M5_MISO  SOC_UNUSED_PIN // TBD
#define SOC_GPIO_PIN_EPD_M5_MOSI  45
#define SOC_GPIO_PIN_EPD_M5_SCK   38
#define SOC_GPIO_PIN_EPD_M5_SS    39
#define SOC_GPIO_PIN_EPD_M5_DC    40
#define SOC_GPIO_PIN_EPD_M5_RST   41
#define SOC_GPIO_PIN_EPD_M5_BUSY  42

/* I2C bus 1 */
#define SOC_GPIO_PIN_M5_SDA       2
#define SOC_GPIO_PIN_M5_SCL       1

/* I2C bus 2 (PCA9557) */
#define SOC_GPIO_PIN_M5_SDA2      48
#define SOC_GPIO_PIN_M5_SCL2      47

/* buttons */
#define SOC_GPIO_PIN_M5_BUTTON1   21
#define SOC_GPIO_PIN_M5_BUTTON2   14 /* touch */

/* buzzer */
#define SOC_GPIO_PIN_M5_BUZZER    9

/* RTC */
#define SOC_GPIO_PIN_RTC_M5_INT   3

/* ADC */
#define SOC_GPIO_PIN_M5_BATTERY   8

/* digital input */
#define SOC_GPIO_PIN_M5_VBAT_SEN  18
#define SOC_GPIO_PIN_M5_VUSB_SEN  12
#define SOC_GPIO_PIN_M5_VGPS_SEN  10

/* GPIO expansion (PCA9557) */
#define SOC_GPIO_LED_M5_BLUE      IO1
#define SOC_GPIO_LED_M5_RED_PWR   IO2
#define SOC_GPIO_LED_M5_RED       IO3
#define SOC_GPIO_PIN_IO_M5_EN     IO4 // active HIGH
#define SOC_GPIO_PIN_EPD_M5_EN    IO5 // active HIGH
