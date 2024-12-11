
#define SOC_GPIO_PIN_CONS_RX  PA3 // UART2_RX
#define SOC_GPIO_PIN_CONS_TX  PA2 // UART2_TX

#define SOC_GPIO_PIN_GNSS_RX  PB7 // UART1_RX
#define SOC_GPIO_PIN_GNSS_TX  PB6 // UART1_TX
#define SOC_GPIO_PIN_GNSS_PPS PB5 // IO1
#define SOC_GPIO_PIN_GNSS_RST PA8 // IO2

#define SOC_GPIO_PIN_STATUS   PA0 // LED1
#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN // PA1 LED2
#define SOC_GPIO_PIN_BATTERY  SOC_UNUSED_PIN
#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN

/* SPI */
#define SOC_GPIO_PIN_MOSI     PA7
#define SOC_GPIO_PIN_MISO     PA6
#define SOC_GPIO_PIN_SCK      PA5
#define SOC_GPIO_PIN_SS       PA4

/* NRF905 */
#define SOC_GPIO_PIN_TXE      PB4
#define SOC_GPIO_PIN_CE       PB3
#define SOC_GPIO_PIN_PWR      PB5

/* SX1262 */
#define SOC_GPIO_PIN_RST      LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_BUSY     LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_DIO0     LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_DIO1     LMIC_UNUSED_PIN

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX PB8 // RXEN

/* RF clock source */
#define SOC_GPIO_PIN_TCXO_OE  PB0

/* RF front end control */
#define SOC_GPIO_PIN_FE       PC13

/* I2C1 */
#define SOC_GPIO_PIN_SDA      PA11
#define SOC_GPIO_PIN_SCL      PA12

/* I2C2 */
#define SOC_GPIO_PIN_SDA2     PA10
#define SOC_GPIO_PIN_SCL2     PA9

/* button */
#define SOC_GPIO_PIN_BUTTON   SOC_UNUSED_PIN

/* 32768 Hz crystal */
#define SOC_GPIO_PIN_XP       PC14
#define SOC_GPIO_PIN_XN       PC15
