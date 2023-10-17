
// GNSS module
#define SOC_GPIO_PIN_HELTRK_GNSS_RX     33
#define SOC_GPIO_PIN_HELTRK_GNSS_TX     34
#define SOC_GPIO_PIN_HELTRK_GNSS_RST    35
#define SOC_GPIO_PIN_HELTRK_GNSS_PPS    36
#define SOC_GPIO_PIN_HELTRK_GNSS_EN     37 /* active LOW */

// SX1262
#define SOC_GPIO_PIN_HELTRK_MOSI        10
#define SOC_GPIO_PIN_HELTRK_MISO        11
#define SOC_GPIO_PIN_HELTRK_SCK         9
#define SOC_GPIO_PIN_HELTRK_SS          8
#define SOC_GPIO_PIN_HELTRK_RST         12
#define SOC_GPIO_PIN_HELTRK_BUSY        13
#define SOC_GPIO_PIN_HELTRK_DIO1        14

// TFT
#define SOC_GPIO_PIN_HELTRK_TFT_MOSI    42
#define SOC_GPIO_PIN_HELTRK_TFT_MISO    SOC_UNUSED_PIN
#define SOC_GPIO_PIN_HELTRK_TFT_SCK     41
#define SOC_GPIO_PIN_HELTRK_TFT_SS      38
#define SOC_GPIO_PIN_HELTRK_TFT_DC      40
#define SOC_GPIO_PIN_HELTRK_TFT_RST     39
#define SOC_GPIO_PIN_HELTRK_TFT_BL_V03  45 /* V1.0 PCB marking */
#define SOC_GPIO_PIN_HELTRK_TFT_BL_V05  21 /* V1.1 PCB marking */
#define SOC_GPIO_PIN_HELTRK_TFT_EN      46 /* active LOW */

// 1st I2C bus
#define SOC_GPIO_PIN_HELTRK_SDA         6
#define SOC_GPIO_PIN_HELTRK_SCL         7

// LED
#define SOC_GPIO_PIN_HELTRK_LED         18 /* white, active HIGH */

// Misc.
#define SOC_GPIO_PIN_HELTRK_VEXT_EN     3 /* V0.3 - active LOW, V0.5 - HIGH */
#define SOC_GPIO_PIN_HELTRK_ADC_EN      2

// V0.5 only: 32768 Hz crystal
#define SOC_GPIO_PIN_HELTRK_XP          15
#define SOC_GPIO_PIN_HELTRK_XN          16
