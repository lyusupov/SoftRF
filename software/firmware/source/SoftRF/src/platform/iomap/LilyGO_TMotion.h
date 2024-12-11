
/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  PA10
#define SOC_GPIO_PIN_CONS_TX  PA9

#define SOC_GPIO_PIN_GNSS_RX  PC11
#define SOC_GPIO_PIN_GNSS_TX  PC10

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN // PB0

#define SOC_GPIO_PIN_GNSS_RST PB2
#define SOC_GPIO_PIN_GNSS_LS  PC6
#define SOC_GPIO_PIN_GNSS_PPS PB5
#define SOC_GPIO_PIN_STATUS   (hw_info.model == SOFTRF_MODEL_DONGLE ?\
                                PA0 : SOC_UNUSED_PIN)

#define SOC_GPIO_PIN_BUZZER   (hw_info.model == SOFTRF_MODEL_DONGLE ?\
                                PA8 : SOC_UNUSED_PIN)

#define SOC_GPIO_PIN_BATTERY  (hw_info.model == SOFTRF_MODEL_DONGLE   ? PB1 :\
                               hw_info.model == SOFTRF_MODEL_BRACELET ? PC4 :\
                               SOC_UNUSED_PIN)

/* SPI0 */
#define SOC_GPIO_PIN_MOSI     PB15
#define SOC_GPIO_PIN_MISO     PB14
#define SOC_GPIO_PIN_SCK      PB13
#define SOC_GPIO_PIN_SS       PB12

/* NRF905 */
#define SOC_GPIO_PIN_TXE      PB11
#define SOC_GPIO_PIN_CE       PB8  // NC
#define SOC_GPIO_PIN_PWR      PB10

/* SX1276 */
#define SOC_GPIO_PIN_RST      PB10
#define SOC_GPIO_PIN_DIO0     PB11
#define SOC_GPIO_PIN_DIO1     PC13
#define SOC_GPIO_PIN_DIO2     PB9
#define SOC_GPIO_PIN_DIO3     PB4
#define SOC_GPIO_PIN_DIO4     PB3
#define SOC_GPIO_PIN_DIO5     PA15

/* SX1262 */
#define SOC_GPIO_PIN_BUSY     LMIC_UNUSED_PIN

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX PA1 // 1:Rx, 0:Tx

/* RF clock source */
#define SOC_GPIO_PIN_TCXO_OE  PD_7
#define SOC_GPIO_PIN_OSC_SEL  PC1

/* I2C */
#define SOC_GPIO_PIN_SDA      PB7
#define SOC_GPIO_PIN_SCL      PB6

/* SPI1 */
#define SOC_GPIO_SPI1_MOSI    PA7
#define SOC_GPIO_SPI1_MISO    PA6
#define SOC_GPIO_SPI1_SCK     PA5
#define SOC_GPIO_SPI1_SS      PA4

#define TTGO_TIMPULSE_OLED_PIN_RST PA8
#define TTGO_TIMPULSE_GPIO_PAD_OUT PA0
#define TTGO_TIMPULSE_GPIO_PAD_PWR PA2
#define TTGO_TIMPULSE_SENSOR_INT   PB1 // PB0
#define TTGO_TIMPULSE_GPIO_CHRG    PB8
#define TTGO_TIMPULSE_VDD_1V8_EN   PB0 /* HIGH - enable 1.8V power supply        */
#define TTGO_TIMPULSE_GPS_PWR_EN   PA3 /* HIGH - enable GNSS and LS power supply */

/* button */
//#define SOC_GPIO_PIN_BUTTON   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BUTTON   (hw_info.model == SOFTRF_MODEL_BRACELET ? \
                              TTGO_TIMPULSE_GPIO_PAD_OUT : PC4)
//#define SOC_GPIO_PIN_BUTTON   PA3
