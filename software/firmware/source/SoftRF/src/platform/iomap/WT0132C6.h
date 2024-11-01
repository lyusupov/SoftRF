
/* ESP32-C6 */
#define SOC_GPIO_PIN_C6_CONS_RX         17
#define SOC_GPIO_PIN_C6_CONS_TX         16

// GNSS module
#define SOC_GPIO_PIN_C6_GNSS_RX         9  /* D3 */
#define SOC_GPIO_PIN_C6_GNSS_TX         22
#define SOC_GPIO_PIN_C6_GNSS_PPS        SOC_UNUSED_PIN // 0

// USB CDC/JTAG
#define SOC_GPIO_PIN_C6_USB_DP          13 /* NC */
#define SOC_GPIO_PIN_C6_USB_DN          12 /* NC */

// SPI
#define SOC_GPIO_PIN_C6_MOSI            6  /* D7 */
#define SOC_GPIO_PIN_C6_MISO            5  /* D6 */
#define SOC_GPIO_PIN_C6_SCK             4  /* D5 */
#define SOC_GPIO_PIN_C6_SS              7  /* D8 */

// NRF905
#define SOC_GPIO_PIN_C6_TXE             2  /* D0 */
#define SOC_GPIO_PIN_C6_CE              8  /* D4 */
#define SOC_GPIO_PIN_C6_PWR             10 /* D2 */

// SX1276
#define SOC_GPIO_PIN_C6_RST             10 /* D2 */
#define SOC_GPIO_PIN_C6_DIO0            2  /* D0 */
#define SOC_GPIO_PIN_C6_SDA             10 /* D2 */
#define SOC_GPIO_PIN_C6_SCL             8  /* D4 */

// LR1121
#define SOC_GPIO_PIN_C6_BUSY            2  /* D0 */
#define SOC_GPIO_PIN_C6_DIO9            8  /* D4 */

// battery voltage (ADC)
#define SOC_GPIO_PIN_C6_BATTERY         1  /* A0 */

// auxillary
#define SOC_GPIO_PIN_C6_BUZZER          21 /* 10 */
#define SOC_GPIO_PIN_C6_STATUS          SOC_UNUSED_PIN


/* WT018684-S5 (ESP32-C2) */
#define SOC_GPIO_PIN_C2_CONS_RX         19
#define SOC_GPIO_PIN_C2_CONS_TX         20

// GNSS module
#define SOC_GPIO_PIN_C2_GNSS_RX         9  /* D3 */
#define SOC_GPIO_PIN_C2_GNSS_TX         18
#define SOC_GPIO_PIN_C2_GNSS_PPS        SOC_UNUSED_PIN // 0

// SPI
#define SOC_GPIO_PIN_C2_MOSI            6  /* D7 */
#define SOC_GPIO_PIN_C2_MISO            5  /* D6 */
#define SOC_GPIO_PIN_C2_SCK             4  /* D5 */
#define SOC_GPIO_PIN_C2_SS              7  /* D8 */

// NRF905
#define SOC_GPIO_PIN_C2_TXE             2  /* D0 */
#define SOC_GPIO_PIN_C2_CE              8  /* D4 */
#define SOC_GPIO_PIN_C2_PWR             10 /* D2 */

// SX1276
#define SOC_GPIO_PIN_C2_RST             10 /* D2 */
#define SOC_GPIO_PIN_C2_DIO0            2  /* D0 */
#define SOC_GPIO_PIN_C2_SDA             10 /* D2 */
#define SOC_GPIO_PIN_C2_SCL             8  /* D4 */

// battery voltage (ADC)
#define SOC_GPIO_PIN_C2_BATTERY         1  /* A0 */

// auxillary
#define SOC_GPIO_PIN_C2_BUZZER          SOC_UNUSED_PIN /* 10 */
#define SOC_GPIO_PIN_C2_STATUS          SOC_UNUSED_PIN
