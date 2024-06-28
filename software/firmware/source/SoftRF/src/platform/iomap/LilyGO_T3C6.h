/* ESP32-C6 */

/* J6 connector, 2.54mm-10P */

// GNSS module (ext.)
#define SOC_GPIO_PIN_T3C6_GNSS_RX       17 /* U0RXD */
#define SOC_GPIO_PIN_T3C6_GNSS_TX       16 /* U0TXD */
#define SOC_GPIO_PIN_T3C6_GNSS_PPS      19

// I2C (ext.)
#define SOC_GPIO_PIN_T3C6_SDA           8
#define SOC_GPIO_PIN_T3C6_SCL           9

// status LED
#define SOC_GPIO_PIN_T3C6_LED           7

// USB CDC/JTAG
#define SOC_GPIO_PIN_T3C6_USB_DP        13
#define SOC_GPIO_PIN_T3C6_USB_DN        12

/* J5 connector, 2.54mm-10P */

// battery voltage (ADC)
#define SOC_GPIO_PIN_T3C6_BATTERY       2  /* NC */

// spare (LED ring, button or buzzer)
#define SOC_GPIO_PIN_T3C6_NC1           3

// UART 1
#define SOC_GPIO_PIN_T3C6_CONS_RX       4
#define SOC_GPIO_PIN_T3C6_CONS_TX       5

// SPI
#define SOC_GPIO_PIN_T3C6_MOSI          0
#define SOC_GPIO_PIN_T3C6_MISO          1
#define SOC_GPIO_PIN_T3C6_SCK           6
#define SOC_GPIO_PIN_T3C6_SS            18

/* internal wiring for AcSiP S62F */

// SX1262
#define SOC_GPIO_PIN_T3C6_RST           21
#define SOC_GPIO_PIN_T3C6_BUSY          22
#define SOC_GPIO_PIN_T3C6_DIO1          23
#define SOC_GPIO_PIN_T3C6_DIO2          20
#define SOC_GPIO_PIN_T3C6_ANT_RX        15
#define SOC_GPIO_PIN_T3C6_ANT_TX        14
