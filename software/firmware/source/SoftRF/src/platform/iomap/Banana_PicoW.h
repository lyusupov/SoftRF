
/* Console I/O */
#define SOC_GPIO_PIN_BPIPW_CONS_RX      13 // "RX"
#define SOC_GPIO_PIN_BPIPW_CONS_TX      15 // "TX"

/* Waveshare Pico-GPS-L76B (MTK) */
#define SOC_GPIO_PIN_BPIPW_GNSS_RX      44 // D3, H2
#define SOC_GPIO_PIN_BPIPW_GNSS_TX      43 // D9, H1
#define SOC_GPIO_PIN_BPIPW_GNSS_RST     SOC_UNUSED_PIN // NA
#define SOC_GPIO_PIN_BPIPW_GNSS_PPS     1  // R20      (NC by default)
#define SOC_GPIO_PIN_BPIPW_GNSS_SBY     2  // STANDBY  (NC by default)
#define SOC_GPIO_PIN_BPIPW_GNSS_FON     41 // FORCE_ON (NC by default)

/* Waveshare Pico-LoRa-SX1262-868M, SX1262 */
#define SOC_GPIO_PIN_BPIPW_MOSI         38 // D7
#define SOC_GPIO_PIN_BPIPW_MISO         39 // D6
#define SOC_GPIO_PIN_BPIPW_SCK          21 // D5
#define SOC_GPIO_PIN_BPIPW_SS           17 // D8
#define SOC_GPIO_PIN_BPIPW_RST          42 // D2
#define SOC_GPIO_PIN_BPIPW_BUSY         47 // D0
#define SOC_GPIO_PIN_BPIPW_DIO1         5  // D4, may cause conflict with SDA

/* Waveshare Pico-LoRa-SX1262-868M, RF antenna switch */
#define SOC_GPIO_PIN_BPIPW_ANT_RXTX     7  // RXEN

/* Waveshare Pico-Environment-Sensor, BME280 */
#define SOC_GPIO_PIN_BPIPW_SDA          5
#define SOC_GPIO_PIN_BPIPW_SCL          6

// LED
#define SOC_GPIO_PIN_BPIPW_LED          40 // D1
#define SOC_GPIO_PIN_BPIPW_STATUS       46 // green, active HIGH

#define SOC_GPIO_PIN_BPIPW_BUZZER       16 // D10
#define SOC_GPIO_PIN_BPIPW_BATTERY      8  // 18, A0, TBD

// Misc.
#define SOC_GPIO_PIN_BPIPW_NEOPIXEL     48 // WS2812B
#define SOC_GPIO_PIN_BPIPW_CAPACITOR    34 // 1 uF, UF2 boot
