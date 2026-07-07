
/* Peripherals */
#define SOC_GPIO_PIN_L2_CONS_RX     44
#define SOC_GPIO_PIN_L2_CONS_TX     43

/* L76K */
#define SOC_GPIO_PIN_L2_GNSS_RX     18
#define SOC_GPIO_PIN_L2_GNSS_TX     17
#define SOC_GPIO_PIN_L2_GNSS_PPS    SOC_UNUSED_PIN // TBD

/* SPI */
#define SOC_GPIO_PIN_L2_MOSI        6
#define SOC_GPIO_PIN_L2_MISO        5
#define SOC_GPIO_PIN_L2_SCK         4
#define SOC_GPIO_PIN_L2_SS          21

/* SX1262 */
#define SOC_GPIO_PIN_L2_RST         7
#define SOC_GPIO_PIN_L2_DIO1        9
#define SOC_GPIO_PIN_L2_BUSY        8

/* I2C bus */
#define SOC_GPIO_PIN_L2_SDA         47
#define SOC_GPIO_PIN_L2_SCL         48

// I2S ES8311 (out) + ES7243E (in)
#define SOC_GPIO_PIN_L2_I2S_LRCK    12
#define SOC_GPIO_PIN_L2_I2S_BCK     11
#define SOC_GPIO_PIN_L2_I2S_MCK     10
#define SOC_GPIO_PIN_L2_I2S_DOUT    16
#define SOC_GPIO_PIN_L2_I2S_DIN     15

/* button */
#define SOC_GPIO_PIN_L2_BUTTON_BOOT 0

/* GPIO expansion (TCA9535) */
#define SOC_EXPIO_L2_BUTTON         0
#define SOC_EXPIO_L2_I2C_INT        1
#define SOC_EXPIO_L2_SD_DET         2
#define SOC_EXPIO_L2_TP_INT         3
#define SOC_EXPIO_L2_TFT_CS         4
#define SOC_EXPIO_L2_TFT_EN         5
#define SOC_EXPIO_L2_TFT_RST        6
#define SOC_EXPIO_L2_GROVE_EN       7
#define SOC_EXPIO_L2_TP_RST         8
#define SOC_EXPIO_L2_GNSS_RST       9
#define SOC_EXPIO_L2_GNSS_WKE       10
#define SOC_EXPIO_L2_OTG_EN         11
#define SOC_EXPIO_L2_PA_EN          12
#define SOC_EXPIO_L2_GNSS_EN        13
#define SOC_EXPIO_L2_TF_EN          14
#define SOC_EXPIO_L2_BAT_EN         15
