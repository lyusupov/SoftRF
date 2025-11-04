
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
#define SOC_GPIO_PIN_P4_SS              46

// I2C
#define SOC_GPIO_PIN_P4_SDA             7
#define SOC_GPIO_PIN_P4_SCL             8

// SX12xx
#define SOC_GPIO_PIN_P4_RST             48
#define SOC_GPIO_PIN_P4_DIO             47
#define SOC_GPIO_PIN_P4_BUSY            0  /* shared with CAM_IO0 */

// Waveshare ESP32-P4-Module-DEV-KIT with SX1262-L76K HAT
#define SOC_GPIO_PIN_P4_WS_MOSI         3
#define SOC_GPIO_PIN_P4_WS_MISO         2
#define SOC_GPIO_PIN_P4_WS_SCK          0
#define SOC_GPIO_PIN_P4_WS_SS           45
#define SOC_GPIO_PIN_P4_WS_RST          22
#define SOC_GPIO_PIN_P4_WS_DIO          46
#define SOC_GPIO_PIN_P4_WS_BUSY         27
#define SOC_GPIO_PIN_P4_WS_GNSS_RX      38
#define SOC_GPIO_PIN_P4_WS_GNSS_TX      37
#define SOC_GPIO_PIN_P4_WS_GNSS_PPS     SOC_UNUSED_PIN

// Elecrow CrowPanel ESP32-P4 7.0 inch HMI display with optional SX1262 module
#define SOC_GPIO_PIN_P4_EC_MOSI         6
#define SOC_GPIO_PIN_P4_EC_MISO         7
#define SOC_GPIO_PIN_P4_EC_SCK          8
#define SOC_GPIO_PIN_P4_EC_SS           10
#define SOC_GPIO_PIN_P4_EC_RST          54
#define SOC_GPIO_PIN_P4_EC_DIO          53
#define SOC_GPIO_PIN_P4_EC_BUSY         9

// SDIO 1 - SDMMC
#define SOC_GPIO_PIN_P4_SD_CLK          43
#define SOC_GPIO_PIN_P4_SD_CMD          44
#define SOC_GPIO_PIN_P4_SD_D0           39
#define SOC_GPIO_PIN_P4_SD_D1           40
#define SOC_GPIO_PIN_P4_SD_D2           41
#define SOC_GPIO_PIN_P4_SD_D3           42
#define SOC_GPIO_PIN_P4_SD_DET          45
#define SOC_GPIO_PIN_P4_SD_PWR          46 /* NC */

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
#define SOC_GPIO_PIN_P4_I2S_DOUT        9
#define SOC_GPIO_PIN_P4_I2S_DIN         11

// Misc.
#define SOC_GPIO_PIN_P4_485_RW          3
#define SOC_GPIO_PIN_P4_PAMP_CTRL       53

#define SOC_GPIO_PIN_P4_BUTTON          35 /* BOOT, active LOW, strapping pin */
#define SOC_GPIO_PIN_P4_BATTERY         21 /* RSVD, shared with TP_INT */
#define SOC_GPIO_PIN_P4_BUZZER          SOC_UNUSED_PIN
#define SOC_GPIO_PIN_P4_NEOPIXEL        1  /* RSVD, shared with CAM_IO1 */

// Ethernet
#define ETH_PHY_TYPE                    ETH_PHY_IP101
#define SOC_GPIO_PIN_P4_ETH_PHY_MDC     31
#define SOC_GPIO_PIN_P4_ETH_PHY_MDIO    52
#define SOC_GPIO_PIN_P4_ETH_PHY_POWER   51 /* PHY_RSTN */

// 7 inch 1024x600 EK79007 / EK73217 MIPI LCD display
#define SOC_GPIO_PIN_P4_LCD_EN          22 /* NC */
#define SOC_GPIO_PIN_P4_LCD_RST         23 /* shared with TP_RST */
#define SOC_GPIO_PIN_P4_LCD_BLED        20
#define SOC_GPIO_PIN_P4_LCD_UPDN        26
#define SOC_GPIO_PIN_P4_LCD_SHLR        27

// Espressif (or Waveshare)  mapping
#define SOC_GPIO_PIN_P4_WS_LCD_RST      27
#define SOC_GPIO_PIN_P4_WS_LCD_BLED     26

// GT911 I2C touch sensor
#define SOC_GPIO_PIN_P4_TP_INT          21
#define SOC_GPIO_PIN_P4_TP_RST          23 /* shared with LCD_RST */

// SC2336 or OV5647 MIPI CSI camera sensor
#define SOC_GPIO_PIN_P4_CAM_IO0         0  /* shared with RF BUSY  */
#define SOC_GPIO_PIN_P4_CAM_IO1         1  /* shared with NEOPIXEL */

/* ESP32-C5 */
#define SOC_GPIO_PIN_C5_CONS_RX         12
#define SOC_GPIO_PIN_C5_CONS_TX         11

// USB
#define SOC_GPIO_PIN_C5_USB_DP          14
#define SOC_GPIO_PIN_C5_USB_DN          13

// ESP HOSTED
#define SOC_GPIO_PIN_C5_SD_CLK          9
#define SOC_GPIO_PIN_C5_SD_CMD          10
#define SOC_GPIO_PIN_C5_SD_D0           8
#define SOC_GPIO_PIN_C5_SD_D1           7
#define SOC_GPIO_PIN_C5_SD_D2           14 /* D+ */
#define SOC_GPIO_PIN_C5_SD_D3           13 /* D- */

#define SOC_GPIO_PIN_C5_IO2             2  /* C5 WAKEUP */
#define SOC_GPIO_PIN_C5_IO28            28 /* BOOT      */

// GNSS module
#define SOC_GPIO_PIN_C5_GNSS_RX         23
#define SOC_GPIO_PIN_C5_GNSS_TX         24
#define SOC_GPIO_PIN_C5_GNSS_PPS        SOC_GPIO_PIN_C5_IO2

// SPI
#define SOC_GPIO_PIN_C5_MOSI            SOC_GPIO_PIN_C5_SD_CMD
#define SOC_GPIO_PIN_C5_MISO            SOC_GPIO_PIN_C5_SD_D0
#define SOC_GPIO_PIN_C5_SCK             SOC_GPIO_PIN_C5_SD_CLK
#define SOC_GPIO_PIN_C5_SS              SOC_GPIO_PIN_C5_SD_D1

// SX12xx or LR112x
#define SOC_GPIO_PIN_C5_RST             3
#define SOC_GPIO_PIN_C5_DIO             0
#define SOC_GPIO_PIN_C5_BUSY            1

// I2C
#define SOC_GPIO_PIN_C5_SDA             4
#define SOC_GPIO_PIN_C5_SCL             5

// Misc.
#define SOC_GPIO_PIN_C5_BUTTON          SOC_GPIO_PIN_C5_IO28
#define SOC_GPIO_PIN_C5_BATTERY         6
#define SOC_GPIO_PIN_C5_BUZZER          25
#define SOC_GPIO_PIN_C5_NEOPIXEL        26
