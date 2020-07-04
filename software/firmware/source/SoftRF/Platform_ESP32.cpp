/*
 * Platform_ESP32.cpp
 * Copyright (C) 2018-2020 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#if defined(ESP32)

#include <SPI.h>
#include <esp_err.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/efuse_reg.h>
#include <Wire.h>
#include <rom/rtc.h>
#include <rom/spi_flash.h>
#include <flashchips.h>
#include <axp20x.h>
#include <TFT_eSPI.h>

#include "Platform_ESP32.h"
#include "SoCHelper.h"
#include "SoundHelper.h"
#include "EEPROMHelper.h"
#include "RFHelper.h"
#include "WiFiHelper.h"
#include "BluetoothHelper.h"
#include "LEDHelper.h"
#include "BaroHelper.h"
#include "BatteryHelper.h"

#include <battery.h>
#include <U8x8lib.h>

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = SOC_GPIO_PIN_RST,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = SOC_GPIO_PIN_TXE,
    .tcxo = LMIC_UNUSED_PIN,
};

WebServer server ( 80 );

#if defined(USE_NEOPIXELBUS_LIBRARY)
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PIX_NUM, SOC_GPIO_PIN_LED);
#else /* USE_ADAFRUIT_NEO_LIBRARY */
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIX_NUM, SOC_GPIO_PIN_LED,
                              NEO_GRB + NEO_KHZ800);
#endif /* USE_NEOPIXELBUS_LIBRARY */

U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C u8x8_ttgo(TTGO_V2_OLED_PIN_RST,
                                                TTGO_V2_OLED_PIN_SCL,
                                                TTGO_V2_OLED_PIN_SDA);

U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C u8x8_heltec(HELTEC_OLED_PIN_RST,
                                                  HELTEC_OLED_PIN_SCL,
                                                  HELTEC_OLED_PIN_SDA);

AXP20X_Class axp;

static U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C *u8x8 = NULL;
static TFT_eSPI *tft = NULL;

static int esp32_board = ESP32_DEVKIT; /* default */

static portMUX_TYPE GNSS_PPS_mutex = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE PMU_mutex      = portMUX_INITIALIZER_UNLOCKED;
volatile bool PMU_Irq = false;

static bool GPIO_21_22_are_busy = false;

static union {
  uint8_t efuse_mac[6];
  uint64_t chipmacid;
};

static bool OLED_display_probe_status = false;
static bool OLED_display_frontpage = false;
static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
extern uint32_t tx_packets_counter, rx_packets_counter;
extern bool loopTaskWDTEnabled;

const char *OLED_Protocol_ID[] = {
  [RF_PROTOCOL_LEGACY]    = "L",
  [RF_PROTOCOL_OGNTP]     = "O",
  [RF_PROTOCOL_P3I]       = "P",
  [RF_PROTOCOL_ADSB_1090] = "A",
  [RF_PROTOCOL_ADSB_UAT]  = "U",
  [RF_PROTOCOL_FANET]     = "F"
};

const char SoftRF_text[]   = "SoftRF";
const char ID_text[]       = "ID";
const char PROTOCOL_text[] = "PROTOCOL";
const char RX_text[]       = "RX";
const char TX_text[]       = "TX";

static void IRAM_ATTR ESP32_PMU_Interrupt_handler() {
  portENTER_CRITICAL_ISR(&PMU_mutex);
  PMU_Irq = true;
  portEXIT_CRITICAL_ISR(&PMU_mutex);
}

static uint32_t ESP32_getFlashId()
{
  return g_rom_flashchip.device_id;
}

static void ESP32_setup()
{
#if !defined(SOFTRF_ADDRESS)

  esp_err_t ret = ESP_OK;
  uint8_t null_mac[6] = {0};

  ret = esp_efuse_mac_get_custom(efuse_mac);
  if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Get base MAC address from BLK3 of EFUSE error (%s)", esp_err_to_name(ret));
    /* If get custom base MAC address error, the application developer can decide what to do:
     * abort or use the default base MAC address which is stored in BLK0 of EFUSE by doing
     * nothing.
     */

    ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
    chipmacid = ESP.getEfuseMac();
  } else {
    if (memcmp(efuse_mac, null_mac, 6) == 0) {
      ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
      chipmacid = ESP.getEfuseMac();
    }
  }
#endif /* SOFTRF_ADDRESS */

#if ESP32_DISABLE_BROWNOUT_DETECTOR
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

  if (psramFound()) {

    uint32_t flash_id = ESP32_getFlashId();

    /*
     *    Board         |   Module   |  Flash memory IC
     *  ----------------+------------+--------------------
     *  DoIt ESP32      | WROOM      | GIGADEVICE_GD25Q32
     *  TTGO T3  V2.0   | PICO-D4 IC | GIGADEVICE_GD25Q32
     *  TTGO T3  V2.1.6 | PICO-D4 IC | GIGADEVICE_GD25Q32
     *  TTGO T22 V06    |            | WINBOND_NEX_W25Q32_V
     *  TTGO T22 V08    |            | WINBOND_NEX_W25Q32_V
     *  TTGO T22 V11    |            | BOYA_BY25Q32AL
     *  TTGO T8  V1.8   | WROVER     | GIGADEVICE_GD25LQ32
     *  TTGO T5S V1.9   |            | WINBOND_NEX_W25Q32_V
     *  TTGO T5S V2.8   |            | BOYA_BY25Q32AL
     *  TTGO T-Watch    |            | WINBOND_NEX_W25Q128_V
     */

    switch(flash_id)
    {
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25LQ32):
      /* ESP32-WROVER module with ESP32-NODEMCU-ADAPTER */
      hw_info.model = SOFTRF_MODEL_STANDALONE;
      break;
    case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q128_V):
      hw_info.model = SOFTRF_MODEL_SKYWATCH;
      break;
    case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q32_V):
    case MakeFlashId(BOYA_ID, BOYA_BY25Q32AL):
    default:
      hw_info.model = SOFTRF_MODEL_PRIME_MK2;
      break;
    }
  } else {
    uint32_t chip_ver = REG_GET_FIELD(EFUSE_BLK0_RDATA3_REG, EFUSE_RD_CHIP_VER_PKG);
    uint32_t pkg_ver  = chip_ver & 0x7;
    if (pkg_ver == EFUSE_RD_CHIP_VER_PKG_ESP32PICOD4) {
      esp32_board    = ESP32_TTGO_V2_OLED;
      lmic_pins.rst  = SOC_GPIO_PIN_TBEAM_RF_RST_V05;
      lmic_pins.busy = SOC_GPIO_PIN_TBEAM_RF_BUSY_V08;
    }
  }

  ledcSetup(LEDC_CHANNEL_BUZZER, 0, LEDC_RESOLUTION_BUZZER);

  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {
    esp32_board = ESP32_TTGO_T_WATCH;

    Wire1.begin(SOC_GPIO_PIN_TWATCH_SEN_SDA , SOC_GPIO_PIN_TWATCH_SEN_SCL);
    Wire1.beginTransmission(AXP202_SLAVE_ADDRESS);
    if (Wire1.endTransmission() == 0) {

      axp.begin(Wire1, AXP202_SLAVE_ADDRESS);

      axp.enableIRQ(AXP202_ALL_IRQ, AXP202_OFF);
      axp.adc1Enable(0xFF, AXP202_OFF);

      axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);

      axp.setPowerOutPut(AXP202_LDO2, AXP202_ON); // BL
      axp.setPowerOutPut(AXP202_LDO3, AXP202_ON); // S76G (MCU + LoRa)
      axp.setLDO4Voltage(AXP202_LDO4_1800MV);
      axp.setPowerOutPut(AXP202_LDO4, AXP202_ON); // S76G (Sony GNSS)

      pinMode(SOC_GPIO_PIN_TWATCH_PMU_IRQ, INPUT_PULLUP);

      attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_TWATCH_PMU_IRQ),
                      ESP32_PMU_Interrupt_handler, FALLING);

      axp.adc1Enable(AXP202_BATT_VOL_ADC1, AXP202_ON);
      axp.enableIRQ(AXP202_PEK_LONGPRESS_IRQ | AXP202_PEK_SHORTPRESS_IRQ, true);
      axp.clearIRQ();
    }

  } else if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
    esp32_board = ESP32_TTGO_T_BEAM;

    Wire1.begin(TTGO_V2_OLED_PIN_SDA , TTGO_V2_OLED_PIN_SCL);
    Wire1.beginTransmission(AXP192_SLAVE_ADDRESS);
    if (Wire1.endTransmission() == 0) {
      hw_info.revision = 8;

      axp.begin(Wire1, AXP192_SLAVE_ADDRESS);

      axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);

      axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
      axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
      axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
      axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON); // NC
      axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);

      axp.setDCDC1Voltage(3300); //       AXP192 power-on value: 3300
      axp.setLDO2Voltage (3300); // LoRa, AXP192 power-on value: 3300
      axp.setLDO3Voltage (3000); // GPS,  AXP192 power-on value: 2800

      pinMode(SOC_GPIO_PIN_TBEAM_V08_PMU_IRQ, INPUT_PULLUP);

      attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_TBEAM_V08_PMU_IRQ),
                      ESP32_PMU_Interrupt_handler, FALLING);

      axp.enableIRQ(AXP202_PEK_LONGPRESS_IRQ | AXP202_PEK_SHORTPRESS_IRQ, true);
      axp.clearIRQ();
    } else {
      hw_info.revision = 2;
    }
    lmic_pins.rst  = SOC_GPIO_PIN_TBEAM_RF_RST_V05;
    lmic_pins.busy = SOC_GPIO_PIN_TBEAM_RF_BUSY_V08;
  }
}

static void ESP32_loop()
{
  if ((hw_info.model    == SOFTRF_MODEL_PRIME_MK2 &&
       hw_info.revision == 8)                     ||
       hw_info.model    == SOFTRF_MODEL_SKYWATCH) {

    bool is_irq = false;
    bool down = false;

    portENTER_CRITICAL_ISR(&PMU_mutex);
    is_irq = PMU_Irq;
    portEXIT_CRITICAL_ISR(&PMU_mutex);

    if (is_irq) {

      if (axp.readIRQ() == AXP_PASS) {

        if (axp.isPEKLongtPressIRQ()) {
          down = true;
#if 0
          Serial.println(F("Longt Press IRQ"));
          Serial.flush();
#endif
        }
        if (axp.isPEKShortPressIRQ()) {
#if 0
          Serial.println(F("Short Press IRQ"));
          Serial.flush();
#endif
        }

        axp.clearIRQ();
      }

      portENTER_CRITICAL_ISR(&PMU_mutex);
      PMU_Irq = false;
      portEXIT_CRITICAL_ISR(&PMU_mutex);

      if (down) {
        shutdown("  OFF  ");
      }
    }

    if (isTimeToBattery()) {
      if (Battery_voltage() > Battery_threshold()) {
        axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
      } else {
        axp.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
      }
    }
  }
}

static void ESP32_fini()
{
  SPI.end();

  esp_wifi_stop();
  esp_bt_controller_disable();

  if (hw_info.model    == SOFTRF_MODEL_SKYWATCH) {

    axp.setChgLEDMode(AXP20X_LED_OFF);

    axp.setPowerOutPut(AXP202_LDO2, AXP202_OFF); // BL
    axp.setPowerOutPut(AXP202_LDO4, AXP202_OFF); // S76G (Sony GNSS)
    axp.setPowerOutPut(AXP202_LDO3, AXP202_OFF); // S76G (MCU + LoRa)

    delay(20);

    esp_sleep_enable_ext0_wakeup((gpio_num_t) SOC_GPIO_PIN_TWATCH_PMU_IRQ, 0); // 1 = High, 0 = Low

  } else if (hw_info.model    == SOFTRF_MODEL_PRIME_MK2 &&
             hw_info.revision == 8) {

    axp.setChgLEDMode(AXP20X_LED_OFF);

    delay(2000); /* Keep 'OFF' message on OLED for 2 seconds */

    axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);

    delay(20);

    esp_sleep_enable_ext0_wakeup((gpio_num_t) SOC_GPIO_PIN_TBEAM_V08_PMU_IRQ, 0); // 1 = High, 0 = Low
  }

  esp_deep_sleep_start();
}

static void ESP32_reset()
{
  ESP.restart();
}

static uint32_t ESP32_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  uint32_t id = (uint32_t) efuse_mac[5]        | ((uint32_t) efuse_mac[4] << 8) | \
               ((uint32_t) efuse_mac[3] << 16) | ((uint32_t) efuse_mac[2] << 24);

  /* remap address to avoid overlapping with congested FLARM range */
  if (((id & 0x00FFFFFF) >= 0xDD0000) && ((id & 0x00FFFFFF) <= 0xDFFFFF)) {
    id += 0x100000;
  }

  return id;
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif /* SOFTRF_ADDRESS */
}

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static void* ESP32_getResetInfoPtr()
{
  switch (rtc_get_reset_reason(0))
  {
    case POWERON_RESET          : reset_info.reason = REASON_DEFAULT_RST; break;
    case SW_RESET               : reset_info.reason = REASON_SOFT_RESTART; break;
    case OWDT_RESET             : reset_info.reason = REASON_WDT_RST; break;
    case DEEPSLEEP_RESET        : reset_info.reason = REASON_DEEP_SLEEP_AWAKE; break;
    case SDIO_RESET             : reset_info.reason = REASON_EXCEPTION_RST; break;
    case TG0WDT_SYS_RESET       : reset_info.reason = REASON_WDT_RST; break;
    case TG1WDT_SYS_RESET       : reset_info.reason = REASON_WDT_RST; break;
    case RTCWDT_SYS_RESET       : reset_info.reason = REASON_WDT_RST; break;
    case INTRUSION_RESET        : reset_info.reason = REASON_EXCEPTION_RST; break;
    case TGWDT_CPU_RESET        : reset_info.reason = REASON_WDT_RST; break;
    case SW_CPU_RESET           : reset_info.reason = REASON_SOFT_RESTART; break;
    case RTCWDT_CPU_RESET       : reset_info.reason = REASON_WDT_RST; break;
    case EXT_CPU_RESET          : reset_info.reason = REASON_EXT_SYS_RST; break;
    case RTCWDT_BROWN_OUT_RESET : reset_info.reason = REASON_EXT_SYS_RST; break;
    case RTCWDT_RTC_RESET       :
      /* Slow start of GD25LQ32 causes one read fault at boot time with current ESP-IDF */
      if (ESP32_getFlashId() == MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25LQ32))
                                  reset_info.reason = REASON_DEFAULT_RST;
      else
                                  reset_info.reason = REASON_WDT_RST;
                                  break;
    default                     : reset_info.reason = REASON_DEFAULT_RST;
  }

  return (void *) &reset_info;
}

static String ESP32_getResetInfo()
{
  switch (rtc_get_reset_reason(0))
  {
    case POWERON_RESET          : return F("Vbat power on reset");
    case SW_RESET               : return F("Software reset digital core");
    case OWDT_RESET             : return F("Legacy watch dog reset digital core");
    case DEEPSLEEP_RESET        : return F("Deep Sleep reset digital core");
    case SDIO_RESET             : return F("Reset by SLC module, reset digital core");
    case TG0WDT_SYS_RESET       : return F("Timer Group0 Watch dog reset digital core");
    case TG1WDT_SYS_RESET       : return F("Timer Group1 Watch dog reset digital core");
    case RTCWDT_SYS_RESET       : return F("RTC Watch dog Reset digital core");
    case INTRUSION_RESET        : return F("Instrusion tested to reset CPU");
    case TGWDT_CPU_RESET        : return F("Time Group reset CPU");
    case SW_CPU_RESET           : return F("Software reset CPU");
    case RTCWDT_CPU_RESET       : return F("RTC Watch dog Reset CPU");
    case EXT_CPU_RESET          : return F("for APP CPU, reseted by PRO CPU");
    case RTCWDT_BROWN_OUT_RESET : return F("Reset when the vdd voltage is not stable");
    case RTCWDT_RTC_RESET       : return F("RTC Watch dog reset digital core and rtc module");
    default                     : return F("No reset information available");
  }
}

static String ESP32_getResetReason()
{

  switch (rtc_get_reset_reason(0))
  {
    case POWERON_RESET          : return F("POWERON_RESET");
    case SW_RESET               : return F("SW_RESET");
    case OWDT_RESET             : return F("OWDT_RESET");
    case DEEPSLEEP_RESET        : return F("DEEPSLEEP_RESET");
    case SDIO_RESET             : return F("SDIO_RESET");
    case TG0WDT_SYS_RESET       : return F("TG0WDT_SYS_RESET");
    case TG1WDT_SYS_RESET       : return F("TG1WDT_SYS_RESET");
    case RTCWDT_SYS_RESET       : return F("RTCWDT_SYS_RESET");
    case INTRUSION_RESET        : return F("INTRUSION_RESET");
    case TGWDT_CPU_RESET        : return F("TGWDT_CPU_RESET");
    case SW_CPU_RESET           : return F("SW_CPU_RESET");
    case RTCWDT_CPU_RESET       : return F("RTCWDT_CPU_RESET");
    case EXT_CPU_RESET          : return F("EXT_CPU_RESET");
    case RTCWDT_BROWN_OUT_RESET : return F("RTCWDT_BROWN_OUT_RESET");
    case RTCWDT_RTC_RESET       : return F("RTCWDT_RTC_RESET");
    default                     : return F("NO_MEAN");
  }
}

static uint32_t ESP32_getFreeHeap()
{
  return ESP.getFreeHeap();
}

static long ESP32_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

static void ESP32_Sound_test(int var)
{
  if (settings->volume != BUZZER_OFF) {

    ledcAttachPin(SOC_GPIO_PIN_BUZZER, LEDC_CHANNEL_BUZZER);

    ledcWrite(LEDC_CHANNEL_BUZZER, 125); // high volume

    if (var == REASON_DEFAULT_RST ||
        var == REASON_EXT_SYS_RST ||
        var == REASON_SOFT_RESTART) {
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 440);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 640);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 840);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 1040);
    } else if (var == REASON_WDT_RST) {
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 440);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 1040);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 440);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 1040);
    } else {
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 1040);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 840);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 640);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 440);
    }
    delay(600);

    ledcWriteTone(LEDC_CHANNEL_BUZZER, 0); // off

    ledcDetachPin(SOC_GPIO_PIN_BUZZER);
    pinMode(SOC_GPIO_PIN_BUZZER, INPUT_PULLDOWN);
  }
}

static uint32_t ESP32_maxSketchSpace()
{
  return 0x1E0000;
}

static const int8_t ESP32_dB_to_power_level[21] = {
  8,  /* 2    dB, #0 */
  8,  /* 2    dB, #1 */
  8,  /* 2    dB, #2 */
  8,  /* 2    dB, #3 */
  8,  /* 2    dB, #4 */
  20, /* 5    dB, #5 */
  20, /* 5    dB, #6 */
  28, /* 7    dB, #7 */
  28, /* 7    dB, #8 */
  34, /* 8.5  dB, #9 */
  34, /* 8.5  dB, #10 */
  44, /* 11   dB, #11 */
  44, /* 11   dB, #12 */
  52, /* 13   dB, #13 */
  52, /* 13   dB, #14 */
  60, /* 15   dB, #15 */
  60, /* 15   dB, #16 */
  68, /* 17   dB, #17 */
  74, /* 18.5 dB, #18 */
  76, /* 19   dB, #19 */
  78  /* 19.5 dB, #20 */
};

static void ESP32_WiFi_setOutputPower(int dB)
{
  if (dB > 20) {
    dB = 20;
  }

  if (dB < 0) {
    dB = 0;
  }

  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(ESP32_dB_to_power_level[dB]));
}

static IPAddress ESP32_WiFi_get_broadcast()
{
  tcpip_adapter_ip_info_t info;
  IPAddress broadcastIp;

  if (WiFi.getMode() == WIFI_STA) {
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &info);
  } else {
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &info);
  }
  broadcastIp = ~info.netmask.addr | info.ip.addr;

  return broadcastIp;
}

static void ESP32_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  IPAddress ClientIP;
  WiFiMode_t mode = WiFi.getMode();
  int i = 0;

  switch (mode)
  {
  case WIFI_STA:
    ClientIP = ESP32_WiFi_get_broadcast();

    Uni_Udp.beginPacket(ClientIP, port);
    Uni_Udp.write(buf, size);
    Uni_Udp.endPacket();

    break;
  case WIFI_AP:
    wifi_sta_list_t stations;
    ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));

    tcpip_adapter_sta_list_t infoList;
    ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));

    while(i < infoList.num) {
      ClientIP = infoList.sta[i++].ip.addr;

      Uni_Udp.beginPacket(ClientIP, port);
      Uni_Udp.write(buf, size);
      Uni_Udp.endPacket();
    }
    break;
  case WIFI_OFF:
  default:
    break;
  }
}

static void ESP32_WiFiUDP_stopAll()
{
/* not implemented yet */
}

static bool ESP32_WiFi_hostname(String aHostname)
{
  return WiFi.setHostname(aHostname.c_str());
}

static int ESP32_WiFi_clients_count()
{
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
  case WIFI_AP:
    wifi_sta_list_t stations;
    ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));

    tcpip_adapter_sta_list_t infoList;
    ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));

    return infoList.num;
  case WIFI_STA:
  default:
    return -1; /* error */
  }
}

static bool ESP32_EEPROM_begin(size_t size)
{
  return EEPROM.begin(size);
}

static void ESP32_SPI_begin()
{
  if (esp32_board != ESP32_TTGO_T_WATCH) {
    SPI.begin(SOC_GPIO_PIN_SCK, SOC_GPIO_PIN_MISO, SOC_GPIO_PIN_MOSI, SOC_GPIO_PIN_SS);
  } else {
    SPI.begin(SOC_GPIO_PIN_TWATCH_TFT_SCK, SOC_GPIO_PIN_TWATCH_TFT_MISO,
              SOC_GPIO_PIN_TWATCH_TFT_MOSI, -1);
  }
}

static void ESP32_swSer_begin(unsigned long baud)
{
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {

    Serial.print(F("INFO: TTGO T-Beam rev. 0"));
    Serial.print(hw_info.revision);
    Serial.println(F(" is detected."));

    if (hw_info.revision == 8) {
      swSer.begin(baud, SERIAL_IN_BITS, SOC_GPIO_PIN_TBEAM_V08_RX, SOC_GPIO_PIN_TBEAM_V08_TX);
    } else {
      swSer.begin(baud, SERIAL_IN_BITS, SOC_GPIO_PIN_TBEAM_V05_RX, SOC_GPIO_PIN_TBEAM_V05_TX);
    }
  } else {
    if (esp32_board == ESP32_TTGO_T_WATCH) {
      Serial.println(F("INFO: TTGO T-Watch is detected."));
      swSer.begin(baud, SERIAL_IN_BITS, SOC_GPIO_PIN_TWATCH_RX, SOC_GPIO_PIN_TWATCH_TX);
    } else if (esp32_board == ESP32_TTGO_V2_OLED) {
      /* 'Mini' (TTGO T3 + GNSS) */
      Serial.print(F("INFO: TTGO T3 rev. "));
      Serial.print(hw_info.revision);
      Serial.println(F(" is detected."));
      swSer.begin(baud, SERIAL_IN_BITS, TTGO_V2_PIN_GNSS_RX, TTGO_V2_PIN_GNSS_TX);
    } else {
      /* open Standalone's GNSS port */
      swSer.begin(baud, SERIAL_IN_BITS, SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
    }
  }

  /* Default Rx buffer size (256 bytes) is sometimes not big enough */
  // swSer.setRxBufferSize(512);

  /* Need to gather some statistics on variety of flash IC usage */
  Serial.print(F("Flash memory ID: "));
  Serial.println(ESP32_getFlashId(), HEX);
}

static void ESP32_swSer_enableRx(boolean arg)
{

}

static byte ESP32_Display_setup()
{
  byte rval = DISPLAY_NONE;

  if (esp32_board != ESP32_TTGO_T_WATCH) {

    /* SSD1306 I2C OLED probing */
    if (GPIO_21_22_are_busy) {
      Wire1.begin(HELTEC_OLED_PIN_SDA , HELTEC_OLED_PIN_SCL);
      Wire1.beginTransmission(SSD1306_OLED_I2C_ADDR);
      if (Wire1.endTransmission() == 0) {
        u8x8 = &u8x8_heltec;
        esp32_board = ESP32_HELTEC_OLED;
        rval = DISPLAY_OLED_HELTEC;
      }
    } else {
      Wire1.begin(TTGO_V2_OLED_PIN_SDA , TTGO_V2_OLED_PIN_SCL);
      Wire1.beginTransmission(SSD1306_OLED_I2C_ADDR);
      if (Wire1.endTransmission() == 0) {
        u8x8 = &u8x8_ttgo;
        esp32_board = ESP32_TTGO_V2_OLED;

        if (RF_SX12XX_RST_is_connected) {
          hw_info.revision = 16;
        } else {
          hw_info.revision = 11;
        }

        rval = DISPLAY_OLED_TTGO;
      } else {
        if (!(hw_info.model    == SOFTRF_MODEL_PRIME_MK2 &&
              hw_info.revision == 8)) {
          Wire1.begin(HELTEC_OLED_PIN_SDA , HELTEC_OLED_PIN_SCL);
          Wire1.beginTransmission(SSD1306_OLED_I2C_ADDR);
          if (Wire1.endTransmission() == 0) {
            u8x8 = &u8x8_heltec;
            esp32_board = ESP32_HELTEC_OLED;
            rval = DISPLAY_OLED_HELTEC;
          }
        }
      }
    }

    if (u8x8) {
      u8x8->begin();
      u8x8->setFont(u8x8_font_chroma48medium8_r);
      u8x8->clear();
      u8x8->draw2x2String(2, 3, SoftRF_text);
    }

  } else {  /* ESP32_TTGO_T_WATCH */

    ESP32_SPI_begin();

    tft = new TFT_eSPI(LV_HOR_RES, LV_VER_RES);
    tft->init();
    tft->setRotation(0);
    tft->fillScreen(TFT_NAVY);

    ledcAttachPin(SOC_GPIO_PIN_TWATCH_TFT_BL, 1);
    ledcSetup(BACKLIGHT_CHANNEL, 12000, 8);

    for (int level = 0; level < 255; level += 25) {
      ledcWrite(BACKLIGHT_CHANNEL, level);
      delay(100);
    }

    tft->setTextFont(4);
    tft->setTextSize(2);
    tft->setTextColor(TFT_WHITE, TFT_NAVY);

    uint16_t tbw = tft->textWidth(SoftRF_text);
    uint16_t tbh = tft->fontHeight();
    tft->setCursor((tft->width() - tbw)/2, (tft->height() - tbh)/2);
    tft->println(SoftRF_text);

    rval = DISPLAY_TFT_TTGO;
  }

  return rval;
}

static void ESP32_Display_loop()
{
  char buf[16];
  uint32_t disp_value;

  uint16_t tbw;
  uint16_t tbh;

  switch (hw_info.display)
  {
  case DISPLAY_TFT_TTGO:
    if (tft) {
      if (!OLED_display_frontpage) {
        tft->fillScreen(TFT_NAVY);

        tft->setTextFont(2);
        tft->setTextSize(2);
        tft->setTextColor(TFT_WHITE, TFT_NAVY);

        tbw = tft->textWidth(ID_text);
        tbh = tft->fontHeight();

        tft->setCursor(tft->textWidth(" "), tft->height()/6 - tbh);
        tft->print(ID_text);

        tbw = tft->textWidth(PROTOCOL_text);

        tft->setCursor(tft->width() - tbw - tft->textWidth(" "),
                       tft->height()/6 - tbh);
        tft->print(PROTOCOL_text);

        tbw = tft->textWidth(RX_text);
        tbh = tft->fontHeight();

        tft->setCursor(tft->textWidth("   "), tft->height()/2 - tbh);
        tft->print(RX_text);

        tbw = tft->textWidth(TX_text);

        tft->setCursor(tft->width()/2 + tft->textWidth("   "),
                       tft->height()/2 - tbh);
        tft->print(TX_text);

        tft->setTextFont(4);
        tft->setTextSize(2);

        itoa(ThisAircraft.addr & 0xFFFFFF, buf, 16);

        tbw = tft->textWidth(buf);
        tbh = tft->fontHeight();

        tft->setCursor(tft->textWidth(" "), tft->height()/6);
        tft->print(buf);

        tbw = tft->textWidth(OLED_Protocol_ID[ThisAircraft.protocol]);

        tft->setCursor(tft->width() - tbw - tft->textWidth(" "),
                       tft->height()/6);
        tft->print(OLED_Protocol_ID[ThisAircraft.protocol]);

        itoa(rx_packets_counter % 1000, buf, 10);
        tft->setCursor(tft->textWidth(" "), tft->height()/2);
        tft->print(buf);

        itoa(tx_packets_counter % 1000, buf, 10);
        tft->setCursor(tft->width()/2 + tft->textWidth(" "), tft->height()/2);
        tft->print(buf);

        OLED_display_frontpage = true;

      } else { /* OLED_display_frontpage */

        if (rx_packets_counter > prev_rx_packets_counter) {
          disp_value = rx_packets_counter % 1000;
          itoa(disp_value, buf, 10);

          if (disp_value < 10) {
            strcat_P(buf,PSTR("  "));
          } else {
            if (disp_value < 100) {
              strcat_P(buf,PSTR(" "));
            };
          }

          tft->setTextFont(4);
          tft->setTextSize(2);

          tft->setCursor(tft->textWidth(" "), tft->height()/2);
          tft->print(buf);

          prev_rx_packets_counter = rx_packets_counter;
        }
        if (tx_packets_counter > prev_tx_packets_counter) {
          disp_value = tx_packets_counter % 1000;
          itoa(disp_value, buf, 10);

          if (disp_value < 10) {
            strcat_P(buf,PSTR("  "));
          } else {
            if (disp_value < 100) {
              strcat_P(buf,PSTR(" "));
            };
          }

          tft->setTextFont(4);
          tft->setTextSize(2);

          tft->setCursor(tft->width()/2 + tft->textWidth(" "), tft->height()/2);
          tft->print(buf);

          prev_tx_packets_counter = tx_packets_counter;
        }
      }
    }

    break;

  case DISPLAY_OLED_TTGO:
  case DISPLAY_OLED_HELTEC:
    if (u8x8) {
      if (!OLED_display_probe_status) {
        u8x8->clear();

        u8x8->draw2x2String(0, 0, "RADIO");
        u8x8->draw2x2String(14, 0, hw_info.rf   != RF_IC_NONE       ? "+" : "-");
        u8x8->draw2x2String(0, 2, "GNSS");
        u8x8->draw2x2String(14, 2, hw_info.gnss != GNSS_MODULE_NONE ? "+" : "-");
        u8x8->draw2x2String(0, 4, "OLED");
        u8x8->draw2x2String(14, 4, hw_info.display != DISPLAY_NONE  ? "+" : "-");
        u8x8->draw2x2String(0, 6, "BARO");
        u8x8->draw2x2String(14, 6, hw_info.baro != BARO_MODULE_NONE ? "+" : "-");

        delay(3000);

        if (loopTaskWDTEnabled) {
          feedLoopWDT();
        }

        OLED_display_probe_status = true;
      } else if (!OLED_display_frontpage) {

        u8x8->clear();

        u8x8->drawString(1, 1, ID_text);

        itoa(ThisAircraft.addr & 0xFFFFFF, buf, 16);
        u8x8->draw2x2String(0, 2, buf);

        u8x8->drawString(8, 1, PROTOCOL_text);

        u8x8->draw2x2String(14, 2, OLED_Protocol_ID[ThisAircraft.protocol]);

        u8x8->drawString(1, 5, RX_text);

        itoa(rx_packets_counter % 1000, buf, 10);
        u8x8->draw2x2String(0, 6, buf);

        u8x8->drawString(9, 5, TX_text);

        if (settings->txpower == RF_TX_POWER_OFF ) {
          strcpy(buf, "OFF");
        } else {
          itoa(tx_packets_counter % 1000, buf, 10);
        }
        u8x8->draw2x2String(8, 6, buf);

        OLED_display_frontpage = true;

      } else {  /* OLED_display_frontpage */

        if (rx_packets_counter > prev_rx_packets_counter) {
          disp_value = rx_packets_counter % 1000;
          itoa(disp_value, buf, 10);

          if (disp_value < 10) {
            strcat_P(buf,PSTR("  "));
          } else {
            if (disp_value < 100) {
              strcat_P(buf,PSTR(" "));
            };
          }

          u8x8->draw2x2String(0, 6, buf);
          prev_rx_packets_counter = rx_packets_counter;
        }
        if (tx_packets_counter > prev_tx_packets_counter) {
          disp_value = tx_packets_counter % 1000;
          itoa(disp_value, buf, 10);

          if (disp_value < 10) {
            strcat_P(buf,PSTR("  "));
          } else {
            if (disp_value < 100) {
              strcat_P(buf,PSTR(" "));
            };
          }

          u8x8->draw2x2String(8, 6, buf);
          prev_tx_packets_counter = tx_packets_counter;
        }
      }
    }

    break;

  case DISPLAY_NONE:
  default:
    break;
  }
}

static void ESP32_Display_fini(const char *msg)
{
  if (u8x8) {
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    u8x8->clear();
    u8x8->draw2x2String(1, 3, msg);
//    u8x8->noDisplay();
  }
}

static void ESP32_Battery_setup()
{
  if ((hw_info.model    == SOFTRF_MODEL_PRIME_MK2 &&
       hw_info.revision == 8)                     ||
       hw_info.model    == SOFTRF_MODEL_SKYWATCH) {

    /* T-Beam v08 and T-Watch have PMU */

    /* TBD */
  } else {
    calibrate_voltage(hw_info.model == SOFTRF_MODEL_PRIME_MK2 ||
                     (esp32_board == ESP32_TTGO_V2_OLED && hw_info.revision == 16) ?
                      ADC1_GPIO35_CHANNEL : ADC1_GPIO36_CHANNEL);
  }
}

static float ESP32_Battery_voltage()
{
  float voltage = 0.0;

  if ((hw_info.model    == SOFTRF_MODEL_PRIME_MK2 &&
       hw_info.revision == 8)                     ||
       hw_info.model    == SOFTRF_MODEL_SKYWATCH) {

    /* T-Beam v08 and T-Watch have PMU */
    if (axp.isBatteryConnect()) {
      voltage = axp.getBattVoltage();
    }
  } else {
    voltage = (float) read_voltage();

    /* T-Beam v02-v07 and T3 V2.1.6 have voltage divider 100k/100k on board */
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2   ||
       (esp32_board   == ESP32_TTGO_V2_OLED && hw_info.revision == 16)) {
      voltage += voltage;
    }
  }

  return (voltage * 0.001);
}

static void IRAM_ATTR ESP32_GNSS_PPS_Interrupt_handler()
{
  portENTER_CRITICAL_ISR(&GNSS_PPS_mutex);
  PPS_TimeMarker = millis();    /* millis() has IRAM_ATTR */
  portEXIT_CRITICAL_ISR(&GNSS_PPS_mutex);
}

static unsigned long ESP32_get_PPS_TimeMarker()
{
  unsigned long rval;
  portENTER_CRITICAL_ISR(&GNSS_PPS_mutex);
  rval = PPS_TimeMarker;
  portEXIT_CRITICAL_ISR(&GNSS_PPS_mutex);
  return rval;
}

static bool ESP32_Baro_setup()
{
  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    return false;

  } else if (hw_info.model != SOFTRF_MODEL_PRIME_MK2) {

    if ((hw_info.rf != RF_IC_SX1276 && hw_info.rf != RF_IC_SX1262) ||
        RF_SX12XX_RST_is_connected) {
      return false;
    }

#if DEBUG
    Serial.println(F("INFO: RESET pin of SX12xx radio is not connected to MCU."));
#endif

    /* Pre-init 1st ESP32 I2C bus to stick on these pins */
    Wire.begin(SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);

  } else {

    /* Start from 1st I2C bus */
    Wire.begin(SOC_GPIO_PIN_TBEAM_SDA, SOC_GPIO_PIN_TBEAM_SCL);
    if (Baro_probe())
      return true;

    if (hw_info.revision == 2)
      return false;

#if !defined(ENABLE_AHRS)
    /* Try out OLED I2C bus */
    Wire.begin(TTGO_V2_OLED_PIN_SDA, TTGO_V2_OLED_PIN_SCL);
    if (!Baro_probe())
      return false;

    GPIO_21_22_are_busy = true;
#else
    return false;
#endif
  }

  return true;
}

static void ESP32_UATSerial_begin(unsigned long baud)
{
  /* open Standalone's I2C/UATSerial port */
  UATSerial.begin(baud, SERIAL_IN_BITS, SOC_GPIO_PIN_CE, SOC_GPIO_PIN_PWR);
}

static void ESP32_UATSerial_updateBaudRate(unsigned long baud)
{
  UATSerial.updateBaudRate(baud);
}

static void ESP32_UATModule_restart()
{
  digitalWrite(SOC_GPIO_PIN_TXE, LOW);
  pinMode(SOC_GPIO_PIN_TXE, OUTPUT);

  delay(100);

  digitalWrite(SOC_GPIO_PIN_TXE, HIGH);

  delay(100);

  pinMode(SOC_GPIO_PIN_TXE, INPUT);
}

static void ESP32_WDT_setup()
{
  enableLoopWDT();
}

static void ESP32_WDT_fini()
{
  disableLoopWDT();
}

static void ESP32_Button_setup()
{
  /* TODO */
}

static void ESP32_Button_loop()
{
  /* TODO */
}

static void ESP32_Button_fini()
{
  /* TODO */
}

const SoC_ops_t ESP32_ops = {
  SOC_ESP32,
  "ESP32",
  ESP32_setup,
  ESP32_loop,
  ESP32_fini,
  ESP32_reset,
  ESP32_getChipId,
  ESP32_getResetInfoPtr,
  ESP32_getResetInfo,
  ESP32_getResetReason,
  ESP32_getFreeHeap,
  ESP32_random,
  ESP32_Sound_test,
  ESP32_maxSketchSpace,
  ESP32_WiFi_setOutputPower,
  ESP32_WiFi_transmit_UDP,
  ESP32_WiFiUDP_stopAll,
  ESP32_WiFi_hostname,
  ESP32_WiFi_clients_count,
  ESP32_EEPROM_begin,
  ESP32_SPI_begin,
  ESP32_swSer_begin,
  ESP32_swSer_enableRx,
  &ESP32_Bluetooth_ops,
  ESP32_Display_setup,
  ESP32_Display_loop,
  ESP32_Display_fini,
  ESP32_Battery_setup,
  ESP32_Battery_voltage,
  ESP32_GNSS_PPS_Interrupt_handler,
  ESP32_get_PPS_TimeMarker,
  ESP32_Baro_setup,
  ESP32_UATSerial_begin,
  ESP32_UATModule_restart,
  ESP32_WDT_setup,
  ESP32_WDT_fini,
  ESP32_Button_setup,
  ESP32_Button_loop,
  ESP32_Button_fini
};

#endif /* ESP32 */
