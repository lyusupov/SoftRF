
/* ESP32-C3 */
#define SOC_GPIO_PIN_C3_CONS_RX         20
#define SOC_GPIO_PIN_C3_CONS_TX         21

// GNSS module
#define SOC_GPIO_PIN_C3_GNSS_RX         9  /* D3 */
#define SOC_GPIO_PIN_C3_GNSS_TX         7
#define SOC_GPIO_PIN_C3_GNSS_PPS        SOC_UNUSED_PIN // 0

// USB CDC/JTAG
#define SOC_GPIO_PIN_C3_USB_DP          19 /* D1 */
#define SOC_GPIO_PIN_C3_USB_DN          18 /* D2 */

// SPI
#define SOC_GPIO_PIN_C3_MOSI            5  /* D7 */
#define SOC_GPIO_PIN_C3_MISO            4  /* D6 */
#define SOC_GPIO_PIN_C3_SCK             3  /* D5 */
#define SOC_GPIO_PIN_C3_SS              8  /* D8 */

// NRF905
#define SOC_GPIO_PIN_C3_TXE             2  /* D0 */
#define SOC_GPIO_PIN_C3_CE              10 /* D4 */
#define SOC_GPIO_PIN_C3_PWR             18 /* D2 */

// SX1276
#define SOC_GPIO_PIN_C3_RST             18 /* D2 */
#define SOC_GPIO_PIN_C3_DIO0            2  /* D0 */
#define SOC_GPIO_PIN_C3_SDA             18 /* D2 */
#define SOC_GPIO_PIN_C3_SCL             10 /* D4 */

// battery voltage (ADC)
#define SOC_GPIO_PIN_C3_BATTERY         1  /* A0 */

// auxillary
#define SOC_GPIO_PIN_C3_BUZZER          6  /* 10 */
#define SOC_GPIO_PIN_C3_STATUS          SOC_UNUSED_PIN
