/* 
 * V1.1
 * ESP32-S3
 * https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/T-Beam_1W_V1.1.pdf
 */

// L76K GNSS module
#define SOC_GPIO_PIN_1W_GNSS_RX         5
#define SOC_GPIO_PIN_1W_GNSS_TX         6
#define SOC_GPIO_PIN_1W_GNSS_PPS        7
#define SOC_GPIO_PIN_1W_GNSS_WAKE       16

// LR2021 ( XY16E3A_X_P33 )
#define SOC_GPIO_PIN_1W_MOSI            11
#define SOC_GPIO_PIN_1W_MISO            12
#define SOC_GPIO_PIN_1W_SCK             13
#define SOC_GPIO_PIN_1W_SS              15
#define SOC_GPIO_PIN_1W_RST             3
#define SOC_GPIO_PIN_1W_BUSY            38
#define SOC_GPIO_PIN_1W_DIO10           1
#define SOC_GPIO_PIN_1W_DIO11           21
#define SOC_GPIO_PIN_1W_PWR_EN          40

// micro-SD
#define SOC_GPIO_PIN_1W_SD_MOSI         11
#define SOC_GPIO_PIN_1W_SD_MISO         12
#define SOC_GPIO_PIN_1W_SD_SCK          13
#define SOC_GPIO_PIN_1W_SD_SS           10

// I2C bus (SH1106 OLED display, ext. sensors)
#define SOC_GPIO_PIN_1W_SDA             8
#define SOC_GPIO_PIN_1W_SCL             9

// 2nd button
#define SOC_GPIO_PIN_1W_BUTTON_AUX      17

// LED
#define SOC_GPIO_PIN_1W_LED             18

// ADC
#define SOC_GPIO_PIN_1W_BATTERY         4 /* 300K / 150K */
#define SOC_GPIO_PIN_1W_TEMP            14

// Fan
#define SOC_GPIO_PIN_1W_FAN             41
