
/* ESP32-P4 */
#define SOC_GPIO_PIN_TDP4_CONS_RX         38
#define SOC_GPIO_PIN_TDP4_CONS_TX         37

// GNSS module
#define SOC_GPIO_PIN_TDP4_GNSS_RX         22
#define SOC_GPIO_PIN_TDP4_GNSS_TX         23
#define SOC_GPIO_PIN_TDP4_GNSS_PPS        SOC_UNUSED_PIN

// SPI
#define SOC_GPIO_PIN_TDP4_MOSI            3
#define SOC_GPIO_PIN_TDP4_MISO            4
#define SOC_GPIO_PIN_TDP4_SCK             2
#define SOC_GPIO_PIN_TDP4_SS              24

#define TDP4_IIC_0                        Wire
#define TDP4_IIC_1                        Wire
#define TDP4_IIC_2                        Wire1
#define TDP4_ES8311_IIC                   1

// I2C #0 (ext. BMx280 air pressure sensor)
#define SOC_GPIO_PIN_TDP4_SDA_0           25
#define SOC_GPIO_PIN_TDP4_SCL_0           36

// I2C #1 (XL9535, PCF8563, BQ27220, HI8561, GT9895)
#define SOC_GPIO_PIN_TDP4_SDA_1           7
#define SOC_GPIO_PIN_TDP4_SCL_1           8

// I2C #2 (ES8311, AW86224, SGM38121, ICM20948, OV2710)
#define SOC_GPIO_PIN_TDP4_SDA_2           20
#define SOC_GPIO_PIN_TDP4_SCL_2           21

// SX12xx
#define SOC_GPIO_PIN_TDP4_RST             LMIC_UNUSED_PIN /* XL P16 */
#define SOC_GPIO_PIN_TDP4_DIO             LMIC_UNUSED_PIN /* XL P17 */
#define SOC_GPIO_PIN_TDP4_BUSY            6

// SDIO 1 - SDMMC
#define SOC_GPIO_PIN_TDP4_SD_CLK          43
#define SOC_GPIO_PIN_TDP4_SD_CMD          44
#define SOC_GPIO_PIN_TDP4_SD_D0           39
#define SOC_GPIO_PIN_TDP4_SD_D1           40
#define SOC_GPIO_PIN_TDP4_SD_D2           41
#define SOC_GPIO_PIN_TDP4_SD_D3           42

#define SOC_GPIO_PIN_TDP4_SD_PWR          SOC_UNUSED_PIN /* XL P15 */

// SDIO 2 - WIFI - ESP32-C6
#define SOC_GPIO_PIN_TDP4_ESP_HOSTED_CLK  18
#define SOC_GPIO_PIN_TDP4_ESP_HOSTED_CMD  19
#define SOC_GPIO_PIN_TDP4_ESP_HOSTED_D0   14
#define SOC_GPIO_PIN_TDP4_ESP_HOSTED_D1   15
#define SOC_GPIO_PIN_TDP4_ESP_HOSTED_D2   16
#define SOC_GPIO_PIN_TDP4_ESP_HOSTED_D3   17

#define SOC_GPIO_PIN_TDP4_ESP_HOSTED_RST  SOC_UNUSED_PIN /* XL P14 */
#define SOC_GPIO_PIN_TDP4_ESP_HOSTED_WKP  SOC_UNUSED_PIN /* XL P13 */

// I2S ES8311
#define SOC_GPIO_PIN_TDP4_I2S_LRCK        9
#define SOC_GPIO_PIN_TDP4_I2S_BCK         12
#define SOC_GPIO_PIN_TDP4_I2S_MCK         13
#define SOC_GPIO_PIN_TDP4_I2S_DOUT        10
#define SOC_GPIO_PIN_TDP4_I2S_DIN         11

// touch pad
#define SOC_GPIO_PIN_TDP4_TP_INT          SOC_UNUSED_PIN /* XL P04 */
#define SOC_GPIO_PIN_TDP4_TP_RST          SOC_UNUSED_PIN /* XL P03 */

// XL9535
#define SOC_GPIO_PIN_TDP4_XL9_INT         5

// HI8561
#define SOC_GPIO_PIN_TDP4_BACKLIGHT       51

// Misc.
#define SOC_GPIO_PIN_TDP4_BUTTON          35 /* BOOT, active LOW, strapping pin */

// Ethernet
#define ETH_PHY_TYPE_TDP4                 ETH_PHY_IP101
#define SOC_GPIO_PIN_TDP4_ETH_PHY_MDC     31
#define SOC_GPIO_PIN_TDP4_ETH_PHY_MDIO    52
#define SOC_GPIO_PIN_TDP4_ETH_PHY_RST     -1 /* XL P05 */

/* GPIO expansion (XL9535) */
#define SOC_EXPIO_TDP4_3V3_EN             IO0  // P00
#define SOC_EXPIO_TDP4_RFSW_VCTL          IO1  // P01
#define SOC_EXPIO_TDP4_DSI_RST            IO2  // P02
#define SOC_EXPIO_TDP4_TP_RST             IO3  // P03
#define SOC_EXPIO_TDP4_TP_INT             IO4  // P04
#define SOC_EXPIO_TDP4_ETH_RST            IO5  // P05
#define SOC_EXPIO_TDP4_5V0_EN             IO6  // P06
#define SOC_EXPIO_TDP4_SENS_INT           IO7  // P07

#define SOC_EXPIO_TDP4_VCCA_EN            IO8  // P10
#define SOC_EXPIO_TDP4_GNSS_WKE           IO9  // P11
#define SOC_EXPIO_TDP4_RTC_INT            IO10 // P12
#define SOC_EXPIO_TDP4_SLAVE_WKE          IO11 // P13
#define SOC_EXPIO_TDP4_SLAVE_EN           IO12 // P14
#define SOC_EXPIO_TDP4_SD_EN              IO13 // P15
#define SOC_EXPIO_TDP4_RADIO_RST          IO14 // P16
#define SOC_EXPIO_TDP4_RADIO_DIO          IO15 // P17

/* CON 2.54-2x8 : 25, 26, 27, 32, 33, 36, 53, 54 */
/* CON 1.0-4    : 45, 46 */
/* CON 1.0-4    : 47, 48 */
