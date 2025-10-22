
/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  PA10
#define SOC_GPIO_PIN_CONS_TX  PA9

#define SOC_GPIO_PIN_GNSS_RX  PA3
#define SOC_GPIO_PIN_GNSS_TX  PA2
#define SOC_GPIO_PIN_GNSS_PPS PB6

#define SOC_GPIO_PIN_LED      PB7 // ext. NeoPixel

#define SOC_GPIO_LED_GREEN    PA0 // active HIGH
#define SOC_GPIO_LED_RED      PA1 // active HIGH
#define SOC_GPIO_PIN_STATUS   SOC_GPIO_LED_GREEN

#define SOC_GPIO_PIN_BUZZER   PB4 // ext. buzzer

#define SOC_GPIO_PIN_BATTERY  PB3

/* I2C */
#define SOC_GPIO_PIN_SDA      PA11
#define SOC_GPIO_PIN_SCL      PA12

/* SPI */
#define SOC_GPIO_PIN_MOSI     PA7
#define SOC_GPIO_PIN_MISO     PA6
#define SOC_GPIO_PIN_SCK      PA5

/* Micro-SD */
#define SOC_GPIO_PIN_SD_SS    PA4

/* OLED */
#define SOC_GPIO_PIN_OLED_SS  PB12
#define SOC_GPIO_PIN_OLED_DC  PA8

/* SX1262 */
#define SOC_GPIO_PIN_SS       LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_RST      LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_BUSY     LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_DIO0     LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_DIO1     LMIC_UNUSED_PIN

/* NRF905 */
#define SOC_GPIO_PIN_TXE      PB8

/* RF antenna switch */
#define SOC_GPIO_ANT_RXTX     PB2 /* TBD */

/* button */
#define SOC_GPIO_PIN_BUTTON   PB5 // ext. button

/* 32768 Hz crystal */
#define SOC_GPIO_PIN_XP       PC14
#define SOC_GPIO_PIN_XN       PC15
