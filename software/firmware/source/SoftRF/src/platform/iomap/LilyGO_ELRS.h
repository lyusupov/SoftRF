/*
 * LilyGO T-Lora Dual LR, a.k.a. LilyGO T-ELRS-LR1121
 * ESP32-PICO-D4
 */

#define SOC_GPIO_PIN_ELRS_GNSS_RX         12 /* NC */
#define SOC_GPIO_PIN_ELRS_GNSS_TX         23 /* NC */

#define SOC_GPIO_PIN_ELRS_MAV_RX          3  /* U0RXD */
#define SOC_GPIO_PIN_ELRS_MAV_TX          1  /* U0TXD */

// SPI
#define SOC_GPIO_PIN_ELRS_MOSI            32
#define SOC_GPIO_PIN_ELRS_MISO            33
#define SOC_GPIO_PIN_ELRS_SCK             25

// LR1121
#if 1
// Radio #1
#define SOC_GPIO_PIN_ELRS_SS              27

#define SOC_GPIO_PIN_ELRS_BUSY            36
#define SOC_GPIO_PIN_ELRS_RST             26
#define SOC_GPIO_PIN_ELRS_DIO9            37

/* AT2401C */
#define SOC_GPIO_PIN_ELRS_HF_TX           14
#define SOC_GPIO_PIN_ELRS_HF_RX           10
#else
// Radio #2
#define SOC_GPIO_PIN_ELRS_SS              13

#define SOC_GPIO_PIN_ELRS_BUSY            39
#define SOC_GPIO_PIN_ELRS_RST             21
#define SOC_GPIO_PIN_ELRS_DIO9            34

/* AT2401C */
#define SOC_GPIO_PIN_ELRS_HF_TX           15
#define SOC_GPIO_PIN_ELRS_HF_RX           9
#endif

// misc.
#define SOC_GPIO_PIN_ELRS_LED             SOC_UNUSED_PIN
#define SOC_GPIO_PIN_ELRS_PIXEL           5
#define SOC_GPIO_PIN_ELRS_BUTTON          0
#define SOC_GPIO_PIN_ELRS_BATTERY         35 /* NC */
