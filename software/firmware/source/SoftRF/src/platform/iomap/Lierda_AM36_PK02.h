/* 
 * V01
 * ESP32-S3
 * https://github.com/lierda-iot/esp32_lora_samples/blob/master/docs/L-LRMAM36-FANN4-PK02_SCH_V01.pdf
 */

#define SOC_GPIO_PIN_AM36_CONS_RX       44
#define SOC_GPIO_PIN_AM36_CONS_TX       43

// GNSS module
#define SOC_GPIO_PIN_AM36_GNSS_RX       12
#define SOC_GPIO_PIN_AM36_GNSS_TX       13
#define SOC_GPIO_PIN_AM36_GNSS_PPS      14

// LR2021
#define SOC_GPIO_PIN_AM36_MOSI          42
#define SOC_GPIO_PIN_AM36_MISO          41
#define SOC_GPIO_PIN_AM36_SCK           40
#define SOC_GPIO_PIN_AM36_SS            39
#define SOC_GPIO_PIN_AM36_RST           38
#define SOC_GPIO_PIN_AM36_BUSY          17
#define SOC_GPIO_PIN_AM36_DIO7          21

// I2C #0
#define SOC_GPIO_PIN_AM36_SDA           8
#define SOC_GPIO_PIN_AM36_SCL           9

// I2C #1
#define SOC_GPIO_PIN_AM36_SDA1          11
#define SOC_GPIO_PIN_AM36_SCL1          10

// User button
#define SOC_GPIO_PIN_AM36_BUTTON        4

// NeoPixel
#define SOC_GPIO_PIN_AM36_LED           18

// 32768 Hz crystal (optional)
#define SOC_GPIO_PIN_AM36_XP            15
#define SOC_GPIO_PIN_AM36_XN            16
