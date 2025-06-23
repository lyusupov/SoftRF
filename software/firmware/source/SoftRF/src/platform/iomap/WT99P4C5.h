
/* ESP32-P4 */
#define SOC_GPIO_PIN_P4_CONS_RX         38
#define SOC_GPIO_PIN_P4_CONS_TX         37

// GNSS module
#define SOC_GPIO_PIN_P4_GNSS_RX         5
#define SOC_GPIO_PIN_P4_GNSS_TX         4
#define SOC_GPIO_PIN_P4_GNSS_PPS        2

// USB1
#define SOC_GPIO_PIN_P4_USB_DP          25
#define SOC_GPIO_PIN_P4_USB_DN          24

// SPI
#define SOC_GPIO_PIN_P4_MOSI            32
#define SOC_GPIO_PIN_P4_MISO            33
#define SOC_GPIO_PIN_P4_SCK             36 /* strapping pin */
#define SOC_GPIO_PIN_P4_SS              26

// I2C
#define SOC_GPIO_PIN_P4_SDA             7
#define SOC_GPIO_PIN_P4_SCL             8
// touch pad
#define SOC_GPIO_PIN_P4_TP_INT          21
#define SOC_GPIO_PIN_P4_TP_RST          23

// SX12xx
#define SOC_GPIO_PIN_P4_RST             48
#define SOC_GPIO_PIN_P4_DIO             47
#define SOC_GPIO_PIN_P4_BUSY            27

// SDIO 1 - SDMMC
#define SOC_GPIO_PIN_P4_SD_CLK          43
#define SOC_GPIO_PIN_P4_SD_CMD          44
#define SOC_GPIO_PIN_P4_SD_D0           39
#define SOC_GPIO_PIN_P4_SD_D1           40
#define SOC_GPIO_PIN_P4_SD_D2           41
#define SOC_GPIO_PIN_P4_SD_D3           42
#define SOC_GPIO_PIN_P4_SD_DET          45
#define SOC_GPIO_PIN_P4_SD_PWR          46 /* NC ? */

// SDIO 2 - WIFI - ESP32-C5
#define SOC_GPIO_PIN_P4_ESP_HOSTED_CLK  18
#define SOC_GPIO_PIN_P4_ESP_HOSTED_CMD  19
#define SOC_GPIO_PIN_P4_ESP_HOSTED_D0   14
#define SOC_GPIO_PIN_P4_ESP_HOSTED_D1   15
#define SOC_GPIO_PIN_P4_ESP_HOSTED_D2   16
#define SOC_GPIO_PIN_P4_ESP_HOSTED_D3   17
#define SOC_GPIO_PIN_P4_ESP_HOSTED_RST  54 /* C5 EN */
#define SOC_GPIO_PIN_P4_ESP_HOSTED_WKP  6  /* C5 WAKEUP */

// I2S ES8311 + MIC
#define SOC_GPIO_PIN_P4_I2S_LRCK        10
#define SOC_GPIO_PIN_P4_I2S_BCK         12
#define SOC_GPIO_PIN_P4_I2S_MCK         13
#define SOC_GPIO_PIN_P4_I2S_DOUT        11
#define SOC_GPIO_PIN_P4_I2S_DIN         9

// Misc.
#define SOC_GPIO_PIN_P4_485_RW          3
#define SOC_GPIO_PIN_P4_PAMP_CTRL       53

#define SOC_GPIO_PIN_P4_BUTTON          35 /* BOOT, active LOW, strapping pin */
#define SOC_GPIO_PIN_P4_BATTERY         0  /* RSVD, shared with CAM_IO0 */
#define SOC_GPIO_PIN_P4_BUZZER          SOC_UNUSED_PIN
#define SOC_GPIO_PIN_P4_NEOPIXEL        1  /* RSVD, shared with CAM_IO1 */

// Ethernet
#define ETH_PHY_TYPE                    ETH_PHY_IP101
#define SOC_GPIO_PIN_P4_ETH_PHY_MDC     31
#define SOC_GPIO_PIN_P4_ETH_PHY_MDIO    52
#define SOC_GPIO_PIN_P4_ETH_PHY_POWER   51 /* PHY_RSTN */

/* ESP32-C5 */
#define SOC_GPIO_PIN_C5_CONS_RX         12
#define SOC_GPIO_PIN_C5_CONS_TX         11

// ESP HOSTED
#define SOC_GPIO_PIN_C5_SD_CLK          9
#define SOC_GPIO_PIN_C5_SD_CMD          10
#define SOC_GPIO_PIN_C5_SD_D0           8
#define SOC_GPIO_PIN_C5_SD_D1           7
#define SOC_GPIO_PIN_C5_SD_D2           14
#define SOC_GPIO_PIN_C5_SD_D3           13

#define SOC_GPIO_PIN_C5_IO2             2 /* C5 WAKEUP */
#define SOC_GPIO_PIN_C5_IO4             4
#define SOC_GPIO_PIN_C5_IO5             5
#define SOC_GPIO_PIN_C5_IO27            27 /* NC ? */
#define SOC_GPIO_PIN_C5_IO28            28

// GNSS module
#define SOC_GPIO_PIN_C5_GNSS_RX         SOC_GPIO_PIN_C5_IO5
#define SOC_GPIO_PIN_C5_GNSS_TX         SOC_GPIO_PIN_C5_IO4
#define SOC_GPIO_PIN_C5_GNSS_PPS        SOC_GPIO_PIN_C5_IO28 /* TBD */

// SPI
#define SOC_GPIO_PIN_C5_MOSI            SOC_GPIO_PIN_C5_SD_CMD
#define SOC_GPIO_PIN_C5_MISO            SOC_GPIO_PIN_C5_SD_D0
#define SOC_GPIO_PIN_C5_SCK             SOC_GPIO_PIN_C5_SD_CLK
#define SOC_GPIO_PIN_C5_SS              SOC_GPIO_PIN_C5_SD_D3

// SX12xx or LR112x
#define SOC_GPIO_PIN_C5_RST             LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_C5_DIO             SOC_GPIO_PIN_C5_SD_D1
#define SOC_GPIO_PIN_C5_BUSY            SOC_GPIO_PIN_C5_SD_D2

// I2C
#define SOC_GPIO_PIN_C5_SDA             25 /* NC */
#define SOC_GPIO_PIN_C5_SCL             26 /* NC */
