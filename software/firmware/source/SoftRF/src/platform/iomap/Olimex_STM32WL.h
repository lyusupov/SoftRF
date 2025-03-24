
#define SOC_GPIO_PIN_CONS_RX  PB7
#define SOC_GPIO_PIN_CONS_TX  PB6

#define SOC_GPIO_PIN_GNSS_RX  PA3
#define SOC_GPIO_PIN_GNSS_TX  PA2
#define SOC_GPIO_PIN_GNSS_PPS PA11

#define SOC_GPIO_PIN_STATUS   PA15
#define SOC_GPIO_PIN_BUZZER   PA12
#define SOC_GPIO_PIN_BATTERY  SOC_UNUSED_PIN /* PA8 */
#define SOC_GPIO_PIN_LED      PB12

/* SPI */
#define SOC_GPIO_PIN_MOSI     PA7 /* E77: PB5 (NC) */
#define SOC_GPIO_PIN_MISO     PB4
#define SOC_GPIO_PIN_SCK      PA5 /* E5 : PB3 (NC) */
#define SOC_GPIO_PIN_SS       PB2

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
#define SOC_GPIO_ANT_RX_OLI   PC13
#define SOC_GPIO_ANT_TX_OLI   PB8

#define SOC_GPIO_ANT_RX_E77   PA7 /* NB: SOC_GPIO_PIN_MOSI = PA7 */
#define SOC_GPIO_ANT_TX_E77   PA6

#define SOC_GPIO_ANT_RX_E5    PA4
#define SOC_GPIO_ANT_TX_E5    PA5 /* NB: SOC_GPIO_PIN_SCK  = PA5 */
#define SOC_GPIO_TCXO_E5      PB0

#define SOC_GPIO_ANT_RX_ST50  PA0
#define SOC_GPIO_ANT_TX_ST50  PA1
#define SOC_GPIO_TCXO_ST50    PB0

#define SOC_GPIO_ANT_RX_3172  PB8
#define SOC_GPIO_ANT_TX_3172  PC13

/* I2C */
#define SOC_GPIO_PIN_SDA      PA10
#define SOC_GPIO_PIN_SCL      PA9

/* button */
#define SOC_GPIO_PIN_BUTTON   PA0 /* NB: SOC_GPIO_ANT_RX_ST50 = PA0 */

/* 32768 Hz crystal */
#define SOC_GPIO_PIN_XP       PC14
#define SOC_GPIO_PIN_XN       PC15
