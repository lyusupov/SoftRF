/* https://www.elecrow.com/download/product/CIL13002M/ThinkNode_M2_Meshtastic_Signal_Transceiver_DataSheet.pdf */

/* CH340K */
#define SOC_GPIO_PIN_M2_CONS_RX     44
#define SOC_GPIO_PIN_M2_CONS_TX     43

// GNSS module (ext.)
#define SOC_GPIO_PIN_M2_GNSS_RX     40
#define SOC_GPIO_PIN_M2_GNSS_TX     41
#define SOC_GPIO_PIN_M2_GNSS_PPS    39

// SX1262
#define SOC_GPIO_PIN_M2_MOSI        11
#define SOC_GPIO_PIN_M2_MISO        13
#define SOC_GPIO_PIN_M2_SCK         12
#define SOC_GPIO_PIN_M2_SS          10
#define SOC_GPIO_PIN_M2_RST         21
#define SOC_GPIO_PIN_M2_BUSY        14
#define SOC_GPIO_PIN_M2_DIO1        3
#define SOC_GPIO_PIN_M2_PWR_EN      48

// 1st I2C bus (ext. sensors)
#define SOC_GPIO_PIN_M2_SDA         8
#define SOC_GPIO_PIN_M2_SCL         18

/* 2nd I2C bus (OLED display) */
#define SOC_GPIO_PIN_M2_OLED_SDA    16
#define SOC_GPIO_PIN_M2_OLED_SCL    15
#define SOC_GPIO_PIN_M2_VEXT_EN     46 // OLED, active HIGH

// LED
#define SOC_GPIO_PIN_M2_LED         1 /* active HIGH ? */

// buzzer
#define SOC_GPIO_PIN_M2_BUZZER      5

// buttons
#define SOC_GPIO_PIN_M2_BUTTON_1    4  /* PWR */
#define SOC_GPIO_PIN_M2_BUTTON_2    47 /* FUNCTION */
#define SOC_GPIO_PIN_M2_BUTTON_BOOT 0

// battery ADC
#define SOC_GPIO_PIN_M2_BATTERY     17

// misc.
#define SOC_GPIO_PIN_M2_LED_PWR     6
#define SOC_GPIO_PIN_M2_ADC         42
#define SOC_GPIO_PIN_M2_VUSB        7

// pin header
/* 2, 8, 18, 19 (D-), 20 (D+), 38, 39, 40, 41, 3V3, VBUS, GND */
