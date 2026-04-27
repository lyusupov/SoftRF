/* Nordic PCA10156 Development Kit */

#if defined(ARDUINO_NRF54L15DK_PCA10156)
/* Peripherals */
#define SOC_GPIO_PIN_CONS_DK_RX         _PINNUM(0,  1) // P0.01
#define SOC_GPIO_PIN_CONS_DK_TX         _PINNUM(0,  0) // P0.00

//#define SOC_GPIO_PIN_GNSS_DK_RX         _PINNUM(1,  5) // P1.05
//#define SOC_GPIO_PIN_GNSS_DK_TX         _PINNUM(1,  4) // P1.04
//#define SOC_GPIO_PIN_GNSS_DK_PPS        _PINNUM(1,  7) // P1.07

/* I2C */
#define SOC_GPIO_PIN_DK_SDA             _PINNUM(1, 10) // shared with LED1
#define SOC_GPIO_PIN_DK_SCL             _PINNUM(1, 11) // P1.11

/* MX25R6435F (Q)SPI flash */
#define SOC_GPIO_PIN_SFL_DK_MOSI        _PINNUM(2,  2) // P2.02
#define SOC_GPIO_PIN_SFL_DK_MISO        _PINNUM(2,  4) // P2.04
#define SOC_GPIO_PIN_SFL_DK_SCK         _PINNUM(2,  1) // P2.01
#define SOC_GPIO_PIN_SFL_DK_SS          _PINNUM(2,  5) // P2.05
#define SOC_GPIO_PIN_SFL_DK_HOLD        _PINNUM(2,  0) // P2.00
#define SOC_GPIO_PIN_SFL_DK_WP          _PINNUM(2,  3) // P2.03

/* Buttons */
#define SOC_GPIO_PIN_DK_BUTTON0         _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_DK_BUTTON1         _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_DK_BUTTON2         _PINNUM(1,  8) // P1.08
#define SOC_GPIO_PIN_DK_BUTTON3         _PINNUM(0,  4) // P0.04

/* L0603G LEDs */
#define SOC_GPIO_PIN_DK_LED0            _PINNUM(2,  9) // P2.09
#define SOC_GPIO_PIN_DK_LED1            _PINNUM(1, 10) // P1.10
#define SOC_GPIO_PIN_DK_LED2            _PINNUM(2,  7) // P2.07
#define SOC_GPIO_PIN_DK_LED3            _PINNUM(1, 14) // P1.14
#endif
