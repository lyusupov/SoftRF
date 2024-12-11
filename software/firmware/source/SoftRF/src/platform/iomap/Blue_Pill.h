
/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  PA10
#define SOC_GPIO_PIN_CONS_TX  PA9

#define SOC_GPIO_PIN_GNSS_RX  PA3
#define SOC_GPIO_PIN_GNSS_TX  PA2

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN // PA8

#define SOC_GPIO_PIN_GNSS_PPS PA1
#define SOC_GPIO_PIN_STATUS   LED_GREEN

#define SOC_GPIO_PIN_BUZZER   PB8
#define SOC_GPIO_PIN_BATTERY  PB1

#define SOC_GPIO_PIN_RX3      PB11
#define SOC_GPIO_PIN_TX3      PB10

/* SPI */
#define SOC_GPIO_PIN_MOSI     PA7
#define SOC_GPIO_PIN_MISO     PA6
#define SOC_GPIO_PIN_SCK      PA5
#define SOC_GPIO_PIN_SS       PA4

/* NRF905 */
#define SOC_GPIO_PIN_TXE      PB4
#define SOC_GPIO_PIN_CE       PB3
#define SOC_GPIO_PIN_PWR      PB5

/* SX1276 (RFM95W) */
#define SOC_GPIO_PIN_RST      PB5
#define SOC_GPIO_PIN_DIO0     PB4

/* SX1262 */
#define SOC_GPIO_PIN_BUSY     LMIC_UNUSED_PIN

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX LMIC_UNUSED_PIN

/* I2C */
#define SOC_GPIO_PIN_SDA      PB7
#define SOC_GPIO_PIN_SCL      PB6

/* button */
#define SOC_GPIO_PIN_BUTTON   SOC_UNUSED_PIN
//#define SOC_GPIO_PIN_BUTTON   USER_BTN
