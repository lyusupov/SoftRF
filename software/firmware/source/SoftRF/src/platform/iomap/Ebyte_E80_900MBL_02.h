
/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  PA10 /* USART1 */
#define SOC_GPIO_PIN_CONS_TX  PA9

#define SOC_GPIO_PIN_GNSS_RX  PB11 /* USART3 */
#define SOC_GPIO_PIN_GNSS_TX  PB10 /* shared with DIO7 */
#define SOC_GPIO_PIN_GNSS_PPS PB9

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN // PA8
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

/* LR2021 */
#define SOC_GPIO_PIN_RST      PB0
#define SOC_GPIO_PIN_BUSY     PA3
#define SOC_GPIO_PIN_DIO7     PB10
#define SOC_GPIO_PIN_DIO8     PB2  /* BOOT 1 */
#define SOC_GPIO_PIN_DIO9     PB1
#define SOC_GPIO_PIN_DIO0     SOC_GPIO_PIN_DIO9

/* NRF905 */
#define SOC_GPIO_PIN_TXE      SOC_GPIO_PIN_BUSY
#define SOC_GPIO_PIN_CE       SOC_GPIO_PIN_DIO9
#define SOC_GPIO_PIN_PWR      SOC_GPIO_PIN_RST

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX LMIC_UNUSED_PIN

/* I2C */
#define SOC_GPIO_PIN_SDA      PB7
#define SOC_GPIO_PIN_SCL      PB6

/* Buttons */
#define SOC_GPIO_PIN_BUTTON   PB15 /* KEY 1 */
#define SOC_GPIO_PIN_BTN2     PB14 /* KEY 2 */

/* 32768 Hz crystal */
#define SOC_GPIO_PIN_XP       PC14
#define SOC_GPIO_PIN_XN       PC15

// USB
#define SOC_GPIO_PIN_USB_DP   PA12
#define SOC_GPIO_PIN_USB_DN   PA11
