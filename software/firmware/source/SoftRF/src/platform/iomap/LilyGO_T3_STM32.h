
/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  PA10 /* USART1_RX */
#define SOC_GPIO_PIN_CONS_TX  PA9  /* USART1_TX */

#define SOC_GPIO_PIN_GNSS_RX  PA3  /* USART2_RX */
#define SOC_GPIO_PIN_GNSS_TX  PA2  /* USART2_TX */
#define SOC_GPIO_PIN_GNSS_PPS PB5

#define SOC_GPIO_PIN_LED      PA12 // ext. NeoPixel

#define SOC_GPIO_LED_GREEN    PA0  // active HIGH
#define SOC_GPIO_LED_RED      PA1  // active HIGH

#define SOC_GPIO_PIN_STATUS   SOC_GPIO_LED_GREEN
#define SOC_GPIO_RADIO_LED_TX SOC_GPIO_LED_RED
#define SOC_GPIO_RADIO_LED_RX SOC_UNUSED_PIN

#define SOC_GPIO_PIN_BUZZER   PB4  // ext. buzzer

#define SOC_GPIO_PIN_BATTERY  PB3  // 100K + 100K voltage divider on VBAT
#define SOC_GPIO_PIN_SOLAR    PA11 // 100K + 100K voltage divider on SOLAR_IN

/* I2C */
#define SOC_GPIO_PIN_SDA      PB7  /* I2C1_SDA */
#define SOC_GPIO_PIN_SCL      PB8  /* I2C1_SCL */

/* SPI */
#define SOC_GPIO_PIN_MOSI     PA7  /* SPI1_MOSI */
#define SOC_GPIO_PIN_MISO     PA6  /* SPI1_MISO */
#define SOC_GPIO_PIN_SCK      PA5  /* SPI1_SCK  */
#define SOC_GPIO_PIN_SS       SOC_GPIO_PIN_SD_SS

/* Micro-SD */
#define SOC_GPIO_PIN_SD_SS    PA4  /* SPI1_NSS  */

/* OLED */
#define SOC_GPIO_PIN_OLED_SS  PB12 /* SPI2_NSS  */
#define SOC_GPIO_PIN_OLED_DC  PA8

/* SX1262 */
#define SOC_GPIO_PIN_RST      LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_BUSY     LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_DIO0     LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_DIO1     LMIC_UNUSED_PIN
#define SOC_GPIO_TCXO         PB0

/* RF antenna switch */
#define SOC_GPIO_ANT_RX       PB2  /* active HIGH */
#define SOC_GPIO_ANT_TX       PB6  /* active HIGH */

/* button */
#define SOC_GPIO_PIN_BUTTON   PH3  // BOOT0

/* 32768 Hz crystal */
#define SOC_GPIO_PIN_XP       PC14
#define SOC_GPIO_PIN_XN       PC15

/* Spare: PA15 , PC13 */
