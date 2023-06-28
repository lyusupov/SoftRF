
/* LilyGO T-TWR 2.0 ("Plus") */
#define SOC_GPIO_PIN_TWR2_CONS_RX       44
#define SOC_GPIO_PIN_TWR2_CONS_TX       43

// GNSS module
#define SOC_GPIO_PIN_TWR2_GNSS_RX       5
#define SOC_GPIO_PIN_TWR2_GNSS_TX       6
#define SOC_GPIO_PIN_TWR2_GNSS_PPS      7

// SA868 module
#define SOC_GPIO_PIN_TWR2_RADIO_RX      48
#define SOC_GPIO_PIN_TWR2_RADIO_TX      39
#define SOC_GPIO_PIN_TWR2_RADIO_PTT     41
#define SOC_GPIO_PIN_TWR2_RADIO_HL      38
#define SOC_GPIO_PIN_TWR2_RADIO_PD      40

// SPI
#define SOC_GPIO_PIN_TWR2_MOSI          11
#define SOC_GPIO_PIN_TWR2_MISO          13
#define SOC_GPIO_PIN_TWR2_SCK           12
#define SOC_GPIO_PIN_TWR2_SS_SD         10
#define SOC_GPIO_PIN_TWR2_SS_EXT        14

// 1st I2C bus (PMU, OLED display, sensors)
#define SOC_GPIO_PIN_TWR2_SDA           8
#define SOC_GPIO_PIN_TWR2_SCL           9
#define SOC_GPIO_PIN_TWR2_PMU_IRQ       4

#define SOC_GPIO_PIN_TWR2_NEOPIXEL      42

/* LilyGO T-TWR 1.3 */
#define SOC_GPIO_PIN_TWR1_GNSS_RX       17
#define SOC_GPIO_PIN_TWR1_GNSS_TX       16
#define SOC_GPIO_PIN_TWR1_GNSS_PPS      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TWR1_RADIO_RX      48
#define SOC_GPIO_PIN_TWR1_RADIO_TX      47
#define SOC_GPIO_PIN_TWR1_RADIO_PTT     41
#define SOC_GPIO_PIN_TWR1_RADIO_HL      39
#define SOC_GPIO_PIN_TWR1_RADIO_PD      40
#define SOC_GPIO_PIN_TWR1_SDA           13
#define SOC_GPIO_PIN_TWR1_SCL           14
#define SOC_GPIO_PIN_TWR1_MOSI          10
#define SOC_GPIO_PIN_TWR1_MISO          11
#define SOC_GPIO_PIN_TWR1_SCK           12
#define SOC_GPIO_PIN_TWR1_SS            18
#define SOC_GPIO_PIN_TWR1_OLED_PWR_EN   21
#define SOC_GPIO_PIN_TWR1_LED           1  /* green, active LOW */
#define SOC_GPIO_PIN_TWR1_BATTERY       6
#define SOC_GPIO_PIN_TWR1_BUTTON1       0  /* Volume - , BOOT */
#define SOC_GPIO_PIN_TWR1_BUTTON2       3  /* Volume + */
#define SOC_GPIO_PIN_TWR1_BUTTON3       38 /* PTT */
#define SOC_GPIO_PIN_TWR1_ENC_BUTTON    7  /* Encoder */
#define SOC_GPIO_PIN_TWR1_ENC_A         9
#define SOC_GPIO_PIN_TWR1_ENC_B         5
#define SOC_GPIO_PIN_TWR1_4_2V_EN       15 /* NC */
