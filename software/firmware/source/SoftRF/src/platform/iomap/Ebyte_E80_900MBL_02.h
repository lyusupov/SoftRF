
/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  PA10
#define SOC_GPIO_PIN_CONS_TX  PA9

#define SOC_GPIO_PIN_GNSS_RX  PA1 /* TBD */
#define SOC_GPIO_PIN_GNSS_TX  PA2

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN // PA8

#define SOC_GPIO_PIN_GNSS_PPS PA0 /* TBD */
#define SOC_GPIO_PIN_STATUS   SOC_UNUSED_PIN

#define SOC_GPIO_PIN_BUZZER   PB8
#define SOC_GPIO_PIN_BATTERY  PB3

#define SOC_GPIO_PIN_RX3      PB13
#define SOC_GPIO_PIN_TX3      PB12

/* SPI */
#define SOC_GPIO_PIN_MOSI     PA7
#define SOC_GPIO_PIN_MISO     PA6
#define SOC_GPIO_PIN_SCK      PA5
#define SOC_GPIO_PIN_SS       PA4

/* NRF905 */
#define SOC_GPIO_PIN_TXE      SOC_GPIO_PIN_DIO7
#define SOC_GPIO_PIN_CE       LMIC_UNUSED_PIN /* TBD */
#define SOC_GPIO_PIN_PWR      SOC_GPIO_PIN_RST

/* LR2021 */
#define SOC_GPIO_PIN_RST      PB0
#define SOC_GPIO_PIN_BUSY     PA3
#define SOC_GPIO_PIN_DIO7     PB10
#define SOC_GPIO_PIN_DIO8     PB2
#define SOC_GPIO_PIN_DIO9     PB1

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX LMIC_UNUSED_PIN

/* I2C */
#define SOC_GPIO_PIN_SDA      PB7
#define SOC_GPIO_PIN_SCL      PB6

/* buttons */
#define SOC_GPIO_PIN_BUTTON   PB15 /* KEY 1 */
#define SOC_GPIO_PIN_BTN2     PB14 /* KEY 2 */
