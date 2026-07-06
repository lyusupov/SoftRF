
// GNSS module
#define SOC_GPIO_PIN_HELTEC_RC32_GNSS_RX     44
#define SOC_GPIO_PIN_HELTEC_RC32_GNSS_TX     43
#define SOC_GPIO_PIN_HELTEC_RC32_GNSS_RST    40
#define SOC_GPIO_PIN_HELTEC_RC32_GNSS_PPS    41
#define SOC_GPIO_PIN_HELTEC_RC32_GNSS_EN     37 /* active HIGH */

// SX1262, TCXO PWR = 1.8 V
#define SOC_GPIO_PIN_HELTEC_RC32_MOSI        12
#define SOC_GPIO_PIN_HELTEC_RC32_MISO        13
#define SOC_GPIO_PIN_HELTEC_RC32_SCK         11
#define SOC_GPIO_PIN_HELTEC_RC32_SS          10
#define SOC_GPIO_PIN_HELTEC_RC32_RST         9
#define SOC_GPIO_PIN_HELTEC_RC32_BUSY        1
#define SOC_GPIO_PIN_HELTEC_RC32_DIO1        14

// TFT 128x220 NV3001B
#define SOC_GPIO_PIN_HELTEC_RC32_TFT_MOSI    38
#define SOC_GPIO_PIN_HELTEC_RC32_TFT_MISO    SOC_UNUSED_PIN
#define SOC_GPIO_PIN_HELTEC_RC32_TFT_SCK     17
#define SOC_GPIO_PIN_HELTEC_RC32_TFT_SS      39
#define SOC_GPIO_PIN_HELTEC_RC32_TFT_DC      16
#define SOC_GPIO_PIN_HELTEC_RC32_TFT_RST     4
#define SOC_GPIO_PIN_HELTEC_RC32_TFT_BL      5  /* active HIGH */
#define SOC_GPIO_PIN_HELTEC_RC32_TFT_EN      6  /* active LOW  */

// I2C bus, TCA6408
#define SOC_GPIO_PIN_HELTEC_RC32_SDA         21
#define SOC_GPIO_PIN_HELTEC_RC32_SCL         18

// Sensor
#define SOC_GPIO_PIN_HELTEC_RC32_SENS_INT    42
#define SOC_GPIO_PIN_HELTEC_RC32_SENS_PWR    46 /* active HIGH */
#define SOC_GPIO_PIN_HELTEC_RC32_SENS_RST    2

// LED
#define SOC_GPIO_PIN_HELTEC_RC32_LED         47

/* buzzer */
#define SOC_GPIO_PIN_HELTEC_RC32_BUZZER      48

/* button */
#define SOC_GPIO_PIN_HELTEC_RC32_BUTTON_BOOT 0

// Misc.
#define SOC_GPIO_PIN_HELTEC_RC32_ADC         7  /* battery voltage ( x 4.9 ) */
#define SOC_GPIO_PIN_HELTEC_RC32_ADC_EN      15 /* active HIGH */
#define SOC_GPIO_PIN_HELTEC_RC32_VEXT_EN     3  /* active HIGH */

// 32768 Hz crystal (TBD)
#define SOC_GPIO_PIN_HELTEC_RC32_XP          15
#define SOC_GPIO_PIN_HELTEC_RC32_XN          16
