/* https://www.ru-ebyte.com/pdf-down.aspx?id=5534 */

/* CH340 */
#define SOC_GPIO_PIN_EHUB_CONS_RX   44
#define SOC_GPIO_PIN_EHUB_CONS_TX   43

// GNSS module (ext.)
#define SOC_GPIO_PIN_EHUB_GNSS_RX   40 /* TBD */
#define SOC_GPIO_PIN_EHUB_GNSS_TX   41 /* TBD */
#define SOC_GPIO_PIN_EHUB_GNSS_PPS  39 /* TBD */

// E80 (LR1121)
#define SOC_GPIO_PIN_EHUB_MOSI      10
#define SOC_GPIO_PIN_EHUB_MISO      11
#define SOC_GPIO_PIN_EHUB_SCK       9
#define SOC_GPIO_PIN_EHUB_SS        8
#define SOC_GPIO_PIN_EHUB_RST       12
#define SOC_GPIO_PIN_EHUB_BUSY      13
#define SOC_GPIO_PIN_EHUB_DIO9      14

// 1st I2C bus (ext. sensors)
#define SOC_GPIO_PIN_EHUB_SDA       45 /* TBD */
#define SOC_GPIO_PIN_EHUB_SCL       46 /* TBD */

/* 2nd I2C bus (OLED display) */
#define SOC_GPIO_PIN_EHUB_OLED_SDA  18
#define SOC_GPIO_PIN_EHUB_OLED_SCL  17
#define SOC_GPIO_PIN_EHUB_OLED_RST  21
#define SOC_GPIO_PIN_EHUB_OLED_3V3  36

// LED
#define SOC_GPIO_PIN_EHUB_LED       35 /* blue, active HIGH */

// button
#define SOC_GPIO_PIN_EHUB_BUTTON    0

// battery ADC
#define SOC_GPIO_PIN_EHUB_BATTERY   1 /* 390K/100K */
#define SOC_GPIO_PIN_EHUB_VBAT_EN   37

// 32768 Hz crystal
#define SOC_GPIO_PIN_EHUB_XP        15
#define SOC_GPIO_PIN_EHUB_XN        16

// misc.
#define SOC_GPIO_PIN_EHUB_CHRG      34

// pin headers
/* 19 (D-), 20 (D+), 21, 26, 48, 47, 33, 34, 35, 36,  0, RST, 43, 44, 3V3B, 3V3B, VBUS, GND */
/*  7, 6, 5, 4, 3, 2, 1, 38, 39, 40, 41, 42, 45, 46, 37, 3V3A, 3V3A, GND */
