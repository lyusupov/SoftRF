/* Minewsemi MX25LE02 Evaluation Kit */

#if defined(ARDUINO_XIAO_NRF54L15_CLEAN)
/* Peripherals */
#define SOC_GPIO_PIN_CONS_MX25_RX       PIN_D0         // D0,  P1.04
#define SOC_GPIO_PIN_CONS_MX25_TX       PIN_D1         // D1,  P1.05

#define SOC_GPIO_PIN_GNSS_MX25_RX       PIN_D7         // D7,  P2.07
#define SOC_GPIO_PIN_GNSS_MX25_TX       PIN_D6         // D6,  P2.08
#define SOC_GPIO_PIN_GNSS_MX25_PPS      PIN_D13        // D13, P2.10

/* SPI */
#define SOC_GPIO_PIN_MX25_MOSI          PIN_D10        // D10, P2.02
#define SOC_GPIO_PIN_MX25_MISO          PIN_D9         // D9,  P2.04
#define SOC_GPIO_PIN_MX25_SCK           PIN_D8         // D8,  P2.01
#define SOC_GPIO_PIN_MX25_SS            PIN_RF_SW_CTL  //      P2.05

/* SX1262 */
#define SOC_GPIO_PIN_MX25_RST           PIN_LED_BUILTIN // D16, P2.00
#define SOC_GPIO_PIN_MX25_DIO1          PIN_BUTTON     // D17, P0.00
#define SOC_GPIO_PIN_MX25_BUSY          _PINNUM(0,  1) // P0.01

/* I2C */
#define SOC_GPIO_PIN_MX25_SDA           PIN_D4         // D4,  P1.10
#define SOC_GPIO_PIN_MX25_SCL           PIN_D5         // D5,  P1.11

/* LEDs */
#define SOC_GPIO_LED_MX25_GREEN         PIN_D11        // D11, P0.03 (Green)
#define SOC_GPIO_LED_MX25_RED           _PINNUM(0,  2) // P0.02 (Red)
#define SOC_GPIO_LED_MX25_BLUE          PIN_D12        // D12, P0.04 (Blue)

/* antenna switch */
#define SOC_GPIO_PIN_MX25_ANT_SW2       PIN_RF_SW      //      P2.03

#define SOC_GPIO_PIN_MX25_STATUS        SOC_GPIO_LED_MX25_GREEN
#define SOC_GPIO_PIN_MX25_BUTTON        PIN_SAMD11_TX  // D19, P1.08
#define SOC_GPIO_PIN_MX25_BUZZER        PIN_D8         // D8,  P2.06
#define SOC_GPIO_PIN_MX25_BATTERY       PIN_A7         // A7,  P1.14

#elif defined(ARDUINO_HOLYIOT_25007_NRF54L15) || \
      defined(ARDUINO_GENERIC_NRF54L15_MODULE_36PIN)
/* Peripherals */
#define SOC_GPIO_PIN_CONS_MX25_RX       P1_04 // +
#define SOC_GPIO_PIN_CONS_MX25_TX       P1_05 // +

#define SOC_GPIO_PIN_GNSS_MX25_RX       P2_07 // +
#define SOC_GPIO_PIN_GNSS_MX25_TX       P2_08 // +
#define SOC_GPIO_PIN_GNSS_MX25_PPS      P2_10 // +

/* SPI */
#define SOC_GPIO_PIN_MX25_MOSI          P2_02 // +
#define SOC_GPIO_PIN_MX25_MISO          P2_04 // +
#define SOC_GPIO_PIN_MX25_SCK           P2_01 // +
#define SOC_GPIO_PIN_MX25_SS            P2_05 // -

/* SX1262 */
#define SOC_GPIO_PIN_MX25_RST           P2_00 // -
#define SOC_GPIO_PIN_MX25_DIO1          P0_00 // -
#define SOC_GPIO_PIN_MX25_BUSY          P0_01 // -

/* I2C */
#define SOC_GPIO_PIN_MX25_SDA           P1_10 // +
#define SOC_GPIO_PIN_MX25_SCL           P1_11 // +

/* LEDs */
#define SOC_GPIO_LED_MX25_GREEN         P0_03 // (Green) -
#define SOC_GPIO_LED_MX25_RED           P0_02 // (Red)   -
#define SOC_GPIO_LED_MX25_BLUE          P0_04 // (Blue)  -

/* antenna switch */
#define SOC_GPIO_PIN_MX25_ANT_SW2       P2_03 // -

#define SOC_GPIO_PIN_MX25_STATUS        SOC_GPIO_LED_MX25_GREEN
#define SOC_GPIO_PIN_MX25_BUTTON        P1_08 // -
#define SOC_GPIO_PIN_MX25_BUZZER        P2_06 // +
#define SOC_GPIO_PIN_MX25_BATTERY       P1_14 // +

#elif defined(ARDUINO_NRF54L15DK_PCA10156) || \
      defined(ARDUINO_XIAO_NRF54LM20A_CLEAN) /* TODO */
/* Peripherals */
#define SOC_GPIO_PIN_CONS_MX25_RX       _PINNUM(1,  4) // P1.04 +
#define SOC_GPIO_PIN_CONS_MX25_TX       _PINNUM(1,  5) // P1.05 +

#define SOC_GPIO_PIN_GNSS_MX25_RX       _PINNUM(2,  7) // P2.07 +
#define SOC_GPIO_PIN_GNSS_MX25_TX       _PINNUM(2,  8) // P2.08 +
#define SOC_GPIO_PIN_GNSS_MX25_PPS      _PINNUM(2, 10) // P2.10 +

/* SPI */
#define SOC_GPIO_PIN_MX25_MOSI          _PINNUM(2,  2) // P2.02 +
#define SOC_GPIO_PIN_MX25_MISO          _PINNUM(2,  4) // P2.04 +
#define SOC_GPIO_PIN_MX25_SCK           _PINNUM(2,  1) // P2.01 +
#define SOC_GPIO_PIN_MX25_SS            _PINNUM(2,  5) // P2.05 -

/* SX1262 */
#define SOC_GPIO_PIN_MX25_RST           _PINNUM(2,  0) // P2.00 -
#define SOC_GPIO_PIN_MX25_DIO1          _PINNUM(0,  0) // P0.00 -
#define SOC_GPIO_PIN_MX25_BUSY          _PINNUM(0,  1) // P0.01 -

/* I2C */
#define SOC_GPIO_PIN_MX25_SDA           _PINNUM(1, 10) // P1.10 +
#define SOC_GPIO_PIN_MX25_SCL           _PINNUM(1, 11) // P1.11 +

/* LEDs */
#define SOC_GPIO_LED_MX25_GREEN         _PINNUM(0,  3) // P0.03 (Green) -
#define SOC_GPIO_LED_MX25_RED           _PINNUM(0,  2) // P0.02 (Red)   -
#define SOC_GPIO_LED_MX25_BLUE          _PINNUM(0,  4) // P0.04 (Blue)  -

/* antenna switch */
#define SOC_GPIO_PIN_MX25_ANT_SW2       _PINNUM(2,  3) // P2.03 -

#define SOC_GPIO_PIN_MX25_STATUS        SOC_GPIO_LED_MX25_GREEN
#define SOC_GPIO_PIN_MX25_BUTTON        _PINNUM(1,  8) // P1.08 -
#define SOC_GPIO_PIN_MX25_BUZZER        _PINNUM(2,  6) // P2.06 +
#define SOC_GPIO_PIN_MX25_BATTERY       _PINNUM(1, 14) // P1.14 +
#endif
