/*
 * Platform_ESP32.cpp
 * Copyright (C) 2019-2022 Linar Yusupov
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

#include "sdkconfig.h"

#include <SPI.h>
#include <esp_err.h>
#include <esp_wifi.h>
#include <soc/rtc_cntl_reg.h>
#include <rom/spi_flash.h>
#include <soc/adc_channel.h>
#include <flashchips.h>
#include <axp20x.h>

#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "WiFiHelper.h"
#include "BluetoothHelper.h"
#include "BatteryHelper.h"
#include "TFTHelper.h"
#include "GNSSHelper.h"

#include <battery.h>
#include <sqlite3.h>
#include <SD.h>

//#include "driver/i2s.h"

#include <esp_wifi.h>
#if !defined(CONFIG_IDF_TARGET_ESP32S2)
#include <esp_bt.h>
#endif /* CONFIG_IDF_TARGET_ESP32S2 */

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  28        /* Time ESP32 will go to sleep (in seconds) */

WebServer server ( 80 );

AXP20X_Class  axp;
PCF8563_Class *rtc = nullptr;
BMA           *bma = nullptr;
I2CBus        *i2c = nullptr;

static union {
  uint8_t efuse_mac[6];
  uint64_t chipmacid;
};

static sqlite3 *fln_db;
static sqlite3 *ogn_db;
static sqlite3 *icao_db;

SPIClass uSD_SPI(
#if !defined(CONFIG_IDF_TARGET_ESP32S2)
                 HSPI
#else
                 FSPI
#endif /* CONFIG_IDF_TARGET_ESP32S2 */
                );

#if 0
/* variables hold file, state of process wav file and wav file properties */
wavProperties_t wavProps;

//i2s configuration
int i2s_num = 0; // i2s port number
i2s_config_t i2s_config = {
     .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
     .sample_rate          = 22050,
     .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
     .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
     .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
     .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
     .dma_buf_count        = 8,
     .dma_buf_len          = 128   //Interrupt level 1
    };

i2s_pin_config_t pin_config = {
    .bck_io_num   = SOC_GPIO_PIN_BCLK,
    .ws_io_num    = SOC_GPIO_PIN_LRCLK,
    .data_out_num = SOC_GPIO_PIN_DOUT,
    .data_in_num  = -1  // Not used
};
#endif

RTC_DATA_ATTR int bootCount   = 0;
static portMUX_TYPE PMU_mutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE BMA_mutex = portMUX_INITIALIZER_UNLOCKED;
volatile bool PMU_Irq         = false;
volatile bool BMA_Irq         = false;

static bool has_usb_client    = false;

static void IRAM_ATTR ESP32_PMU_Interrupt_handler() {
  portENTER_CRITICAL_ISR(&PMU_mutex);
  PMU_Irq = true;
  portEXIT_CRITICAL_ISR(&PMU_mutex);
}

static void IRAM_ATTR ESP32_BMA_Interrupt_handler() {
  portENTER_CRITICAL_ISR(&BMA_mutex);
  BMA_Irq = true;
  portEXIT_CRITICAL_ISR(&BMA_mutex);
}

static uint32_t ESP32_getFlashId()
{
  return g_rom_flashchip.device_id;
}

static int ESP32_I2C_readBytes(uint8_t devAddress, uint8_t regAddress, uint8_t *data, uint8_t len)
{
    if (!i2c) return 0xFF;
    return i2c->readBytes(devAddress, regAddress, data, len);
}

static int ESP32_I2C_writeBytes(uint8_t devAddress, uint8_t regAddress, uint8_t *data, uint8_t len)
{
    if (!i2c) return 0xFF;
    return i2c->writeBytes(devAddress, regAddress, data, len);
}

#define TAG "MAC"

static void ESP32_setup()
{
  esp_err_t ret = ESP_OK;
  uint8_t null_mac[6] = {0};

  ++bootCount;

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

  uint32_t flash_id = ESP32_getFlashId();

  /*
   *    Board          |   Module   |  Flash memory IC
   *  -----------------+------------+--------------------
   *  DoIt ESP32       | WROOM      | GIGADEVICE_GD25Q32
   *  TTGO LoRa32 V2.0 | PICO-D4 IC | GIGADEVICE_GD25Q32
   *  TTGO T-Beam V06  |            | WINBOND_NEX_W25Q32_V
   *  TTGO T8  V1.8    | WROVER     | GIGADEVICE_GD25LQ32
   *  TTGO T5S V1.9    |            | WINBOND_NEX_W25Q32_V
   *  TTGO T-Watch     |            | WINBOND_NEX_W25Q128_V
   *  TTGO T8 S2 V1.1  |            | WINBOND_NEX_W25Q32_V
   *  Ai-T NodeMCU-S3  | ESP-S3-12K | GIGADEVICE_GD25Q64C
   *  TTGO T-Dongle    |            | BOYA_BY25Q32AL
   */

  if (psramFound()) {
    switch(flash_id)
    {
    case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q128_V):
      hw_info.model = SOFTRF_MODEL_SKYWATCH;
      hw_info.revision = HW_REV_T_WATCH_19;
      break;
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25LQ32):
      hw_info.model = SOFTRF_MODEL_WEBTOP_SERIAL;
      hw_info.revision = HW_REV_T8;
      break;
#if defined(CONFIG_IDF_TARGET_ESP32S2)
    case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q32_V):
    case MakeFlashId(BOYA_ID, BOYA_BY25Q32AL):
      hw_info.model = SOFTRF_MODEL_WEBTOP_USB;
      hw_info.revision = HW_REV_TDONGLE;
      break;
#endif /* CONFIG_IDF_TARGET_ESP32S2 */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25Q64):
      hw_info.model = SOFTRF_MODEL_WEBTOP_SERIAL;
      hw_info.revision = HW_REV_DEVKIT;
      break;
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
    default:
      hw_info.model = SOFTRF_MODEL_WEBTOP_SERIAL;
      hw_info.revision = HW_REV_UNKNOWN;
      break;
    }
  } else {
    switch(flash_id)
    {
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25Q32):
      hw_info.model = SOFTRF_MODEL_WEBTOP_SERIAL;
      hw_info.revision = HW_REV_DEVKIT;
      break;
    default:
      hw_info.model = SOFTRF_MODEL_WEBTOP_SERIAL;
      hw_info.revision = HW_REV_UNKNOWN;
      break;
    }
  }

  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    bool axp_present = false;
    bool bma_present = false;
    bool rtc_present = false;

    Wire1.begin(SOC_GPIO_PIN_TWATCH_SEN_SDA , SOC_GPIO_PIN_TWATCH_SEN_SCL);
    Wire1.beginTransmission(AXP202_SLAVE_ADDRESS);
    axp_present = (Wire1.endTransmission() == 0);

    Wire1.beginTransmission(BMA4_I2C_ADDR_SECONDARY);
    bma_present = (Wire1.endTransmission() == 0);

    Wire1.beginTransmission(PCF8563_SLAVE_ADDRESS);
    rtc_present = (Wire1.endTransmission() == 0);

    i2c = new I2CBus(Wire1, SOC_GPIO_PIN_TWATCH_SEN_SDA, SOC_GPIO_PIN_TWATCH_SEN_SCL);

    if (axp_present && (i2c != nullptr)) {
      axp.begin(ESP32_I2C_readBytes, ESP32_I2C_writeBytes, AXP202_SLAVE_ADDRESS);

      axp.enableIRQ(AXP202_ALL_IRQ, AXP202_OFF);
      axp.adc1Enable(0xFF, AXP202_OFF);
      axp.adc2Enable(0xFF, AXP202_OFF);

      axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);

      axp.setPowerOutPut(AXP202_LDO2, AXP202_ON); // BL
      axp.setPowerOutPut(AXP202_LDO3, AXP202_ON); // S76G (MCU + LoRa)
      axp.setLDO4Voltage(AXP202_LDO4_1800MV);
      axp.setPowerOutPut(AXP202_LDO4, AXP202_ON); // S76G (Sony GNSS)

      pinMode(SOC_GPIO_PIN_TWATCH_PMU_IRQ, INPUT_PULLUP);

      attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_TWATCH_PMU_IRQ),
                      ESP32_PMU_Interrupt_handler, FALLING);

#if DEBUG_POWER
      axp.adc1Enable(AXP202_BATT_VOL_ADC1 | AXP202_BATT_CUR_ADC1 |
                     AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1, AXP202_ON);
#else
      axp.adc1Enable(AXP202_BATT_VOL_ADC1, AXP202_ON);
#endif

      axp.enableIRQ(AXP202_PEK_LONGPRESS_IRQ | AXP202_PEK_SHORTPRESS_IRQ, true);
      axp.clearIRQ();
    }

    if (bma_present && (i2c != nullptr)) {
      bma = new BMA(*i2c);

      pinMode(SOC_GPIO_PIN_TWATCH_BMA_IRQ, INPUT);
      attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_TWATCH_BMA_IRQ),
                      ESP32_BMA_Interrupt_handler, RISING);
      hw_info.imu = IMU_BMA423;
    }

    if (rtc_present && (i2c != nullptr)) {
      rtc = new PCF8563_Class(*i2c);
      hw_info.rtc = RTC_PCF8563;
    }
  }

  /* SD-SPI init */
  uSD_SPI.begin(
#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32S3)
                SOC_GPIO_PIN_TWATCH_SD_SCK,
                SOC_GPIO_PIN_TWATCH_SD_MISO,
                SOC_GPIO_PIN_TWATCH_SD_MOSI,
                SOC_GPIO_PIN_TWATCH_SD_SS
#else
                SOC_GPIO_PIN_TDONGLE_SCK,
                SOC_GPIO_PIN_TDONGLE_MISO,
                SOC_GPIO_PIN_TDONGLE_MOSI,
                SOC_GPIO_PIN_TDONGLE_SS
#endif /* CONFIG_IDF_TARGET_ESP32SX */
               );

#if ARDUINO_USB_CDC_ON_BOOT && (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3))
  Serial.begin(SERIAL_OUT_BR);

  for (int i=0; i < 20; i++) {if (Serial) break; else delay(100);}
#else
#if defined(USE_USB_HOST)
  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS,
               SOC_GPIO_PIN_TDONGLE_CONS_RX, SOC_GPIO_PIN_TDONGLE_CONS_TX);
#else
  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#endif /* USE_USB_HOST */
#endif /* ARDUINO_USB_CDC_ON_BOOT && (CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3) */
}

const char SoftRF_text2 [] = "Edition";
const char SoftRF_text3 [] = "Standalone";
const char SoftRF_text4 [] = "Prime 2";
const char SoftRF_text5 [] = "Dongle";
const char SoftRF_text6 [] = "Badge";
const char SoftRF_text7 [] = "Academy";
const char SoftRF_text8 [] = "ES";
const char SoftRF_text9 [] = "Lego";
const char SoftRF_text10[] = "Prime 3";
const char SoftRF_text11[] = "Balkan";

static void ESP32_post_init()
{
  const char *str1 = "NO";
  const char *str2 = "DEVICE";

  switch (hw_info.display)
  {
  case DISPLAY_TFT_TTGO_135:
    if (has_usb_client) {
      str2 = SoftRF_text2;

      switch (hw_info.slave)
      {
      case SOFTRF_MODEL_STANDALONE:
        str1 = SoftRF_text3;
        break;
      case SOFTRF_MODEL_PRIME_MK2:
        str1 = SoftRF_text4;
        break;
      case SOFTRF_MODEL_DONGLE:
        str1 = SoftRF_text5;
        break;
      case SOFTRF_MODEL_BADGE:
        str1 = SoftRF_text6;
        break;
      case SOFTRF_MODEL_ACADEMY:
        str1 = SoftRF_text7;
        break;
      case SOFTRF_MODEL_ES:
        str1 = SoftRF_text8;
        break;
      case SOFTRF_MODEL_LEGO:
        str1 = SoftRF_text9;
        break;
      case SOFTRF_MODEL_PRIME_MK3:
        str1 = SoftRF_text10;
        break;
      case SOFTRF_MODEL_BALKAN:
        str1 = SoftRF_text11;
        break;
      default:
        str1 = "Unknown";
        break;
      }
    }

    TFT_Message(str1, str2);
    delay(3000);

    break;
  case DISPLAY_NONE:
  default:
    break;
  }
}

static void ESP32_loop()
{
  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

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

#if defined(USE_USB_HOST)
  if (hw_info.model == SOFTRF_MODEL_WEBTOP_USB &&
      hw_info.gnss  != GNSS_MODULE_NONE        &&
      settings->m.connection == CON_USB) {

    int c = -1;

    while (true) {
      if (Serial_GNSS_In.available() > 0) {
        c = Serial_GNSS_In.read();
      } else {
        /* return back if no input data */
        break;
      }

      if (c == -1) {
        /* retry */
        continue;
      }

      if (isPrintable(c) || c == '\r' || c == '\n') {
        if (SoC->USB_ops) {
          uint8_t symbol = (uint8_t) c;
          SoC->USB_ops->write(&symbol, 1);
        }
      }
    }
  }
#endif /* USE_USB_HOST */
}

static void ESP32_fini()
{
  uSD_SPI.end();

  esp_wifi_stop();

#if !defined(CONFIG_IDF_TARGET_ESP32S2)
  esp_bt_controller_disable();
#endif /* CONFIG_IDF_TARGET_ESP32S2 */

  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    axp.setChgLEDMode(AXP20X_LED_OFF);

    axp.setPowerOutPut(AXP202_LDO2, AXP202_OFF); // BL
#if !defined(EB_S76G_1_3)
    axp.setPowerOutPut(AXP202_LDO4, AXP202_OFF); // S76G (Sony GNSS)
#endif
    axp.setPowerOutPut(AXP202_LDO3, AXP202_OFF); // S76G (MCU + LoRa)

    delay(20);

    esp_sleep_enable_ext0_wakeup((gpio_num_t) SOC_GPIO_PIN_TWATCH_PMU_IRQ, 0); // 1 = High, 0 = Low
  }

//  Serial.println("Going to sleep now");
//  Serial.flush();

  esp_deep_sleep_start();
}

static void ESP32_reset()
{
  ESP.restart();
}

static void ESP32_sleep_ms(int ms)
{
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}

static uint32_t ESP32_getChipId()
{
  return (uint32_t) efuse_mac[5]        | (efuse_mac[4] << 8) | \
                   (efuse_mac[3] << 16) | (efuse_mac[2] << 24);
}

static bool ESP32_EEPROM_begin(size_t size)
{
  return EEPROM.begin(size);
}

static const int8_t ESP32_dBm_to_power_level[21] = {
  8,  /* 2    dBm, #0 */
  8,  /* 2    dBm, #1 */
  8,  /* 2    dBm, #2 */
  8,  /* 2    dBm, #3 */
  8,  /* 2    dBm, #4 */
  20, /* 5    dBm, #5 */
  20, /* 5    dBm, #6 */
  28, /* 7    dBm, #7 */
  28, /* 7    dBm, #8 */
  34, /* 8.5  dBm, #9 */
  34, /* 8.5  dBm, #10 */
  44, /* 11   dBm, #11 */
  44, /* 11   dBm, #12 */
  52, /* 13   dBm, #13 */
  52, /* 13   dBm, #14 */
  60, /* 15   dBm, #15 */
  60, /* 15   dBm, #16 */
  68, /* 17   dBm, #17 */
  74, /* 18.5 dBm, #18 */
  76, /* 19   dBm, #19 */
  78  /* 19.5 dBm, #20 */
};

static void ESP32_WiFi_set_param(int ndx, int value)
{
  uint32_t lt = value * 60; /* in minutes */

  switch (ndx)
  {
  case WIFI_PARAM_TX_POWER:
    if (value > 20) {
      value = 20; /* dBm */
    }

    if (value < 0) {
      value = 0; /* dBm */
    }

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(ESP32_dBm_to_power_level[value]));
    break;
  case WIFI_PARAM_DHCP_LEASE_TIME:
    tcpip_adapter_dhcps_option(
      (tcpip_adapter_dhcp_option_mode_t) TCPIP_ADAPTER_OP_SET,
      (tcpip_adapter_dhcp_option_id_t)   TCPIP_ADAPTER_IP_ADDRESS_LEASE_TIME,
      (void*) &lt, sizeof(lt));
    break;
  default:
    break;
  }
}

static bool ESP32_WiFi_hostname(String aHostname)
{
  return WiFi.setHostname(aHostname.c_str());
}

static void ESP32_WiFiUDP_stopAll()
{
/* not implemented yet */
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

static size_t ESP32_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  return 0; // WiFi_Receive_UDP(buf, max_size);
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

static void ESP32_swSer_begin(unsigned long baud)
{
  if (settings->m.connection == CON_SERIAL_MAIN) {
    uint32_t config = hw_info.model == SOFTRF_MODEL_SKYWATCH  &&
                      baud          == SERIAL_IN_BR           ?
                      SERIAL_IN_BITS : SERIAL_8N1;
    SerialInput.begin(baud, config, SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
  } else {
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
#if ARDUINO_USB_ON_BOOT == 0
#if defined(USE_USB_HOST)
    if (hw_info.model == SOFTRF_MODEL_WEBTOP_USB) {
      Serial_GNSS_In.updateBaudRate(SERIAL_GNSS_BR);
    }
#else
    Serial.updateBaudRate(baud);
#endif /* USE_USB_HOST */
#endif /* ARDUINO_USB_ON_BOOT */
#else
    Serial.updateBaudRate(baud);
#endif /* defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) */
  }
}

static void ESP32_swSer_enableRx(boolean arg)
{

}

static uint32_t ESP32_maxSketchSpace()
{
  return 0x1E0000;
}

static void ESP32_Battery_setup()
{
  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    /* T-Beam v08 and T-Watch have PMU */

    /* TBD */

  } else if (hw_info.model == SOFTRF_MODEL_WEBTOP_USB) {
#if defined(CONFIG_IDF_TARGET_ESP32S2)
    calibrate_voltage(ADC1_GPIO9_CHANNEL);
#endif /* CONFIG_IDF_TARGET_ESP32S2 */
  } else if (hw_info.model == SOFTRF_MODEL_WEBTOP_SERIAL) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    calibrate_voltage(ADC1_GPIO2_CHANNEL);
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
  }
}

static float ESP32_Battery_voltage()
{
  float voltage = 0.0;

  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    /* T-Beam v08 and T-Watch have PMU */
    if (axp.isBatteryConnect()) {
      voltage = axp.getBattVoltage();
    }
  }

  return (voltage * 0.001);
}

static bool ESP32_DB_init()
{
  int ss_pin = (hw_info.model == SOFTRF_MODEL_WEBTOP_USB) ?
               SOC_GPIO_PIN_TDONGLE_SS : SOC_GPIO_PIN_TWATCH_SD_SS;

  if (!SD.begin(ss_pin, uSD_SPI)) {
    Serial.println(F("ERROR: Failed to mount microSD card."));
    return false;
  }

  sqlite3_initialize();

  sqlite3_open("/sd/Aircrafts/fln.db", &fln_db);

  if (fln_db == NULL)
  {
    Serial.println(F("Failed to open FlarmNet DB\n"));
    return false;
  }

  sqlite3_open("/sd/Aircrafts/ogn.db", &ogn_db);

  if (ogn_db == NULL)
  {
    Serial.println(F("Failed to open OGN DB\n"));
    sqlite3_close(fln_db);
    return false;
  }

  sqlite3_open("/sd/Aircrafts/icao.db", &icao_db);

  if (icao_db == NULL)
  {
    Serial.println(F("Failed to open ICAO DB\n"));
    sqlite3_close(fln_db);
    sqlite3_close(ogn_db);
    return false;
  }

  return true;
}

static bool ESP32_DB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  sqlite3_stmt *stmt;
  char *query = NULL;
  int error;
  bool rval = false;
  const char *reg_key, *db_key;
  sqlite3 *db;

  switch (type)
  {
  case DB_OGN:
    switch (settings->m.idpref)
    {
    case ID_TAIL:
      reg_key = "accn";
      break;
    case ID_MAM:
      reg_key = "acmodel";
      break;
    case ID_REG:
    default:
      reg_key = "acreg";
      break;
    }
    db_key  = "devices";
    db      = ogn_db;
    break;
  case DB_ICAO:
    switch (settings->m.idpref)
    {
    case ID_TAIL:
      reg_key = "owner";
      break;
    case ID_MAM:
      reg_key = "type";
      break;
    case ID_REG:
    default:
      reg_key = "registration";
      break;
    }
    db_key  = "aircrafts";
    db      = icao_db;
    break;
  case DB_FLN:
  default:
    switch (settings->m.idpref)
    {
    case ID_TAIL:
      reg_key = "tail";
      break;
    case ID_MAM:
      reg_key = "type";
      break;
    case ID_REG:
    default:
      reg_key = "registration";
      break;
    }
    db_key  = "aircrafts";
    db      = fln_db;
    break;
  }

  if (db == NULL) {
    return false;
  }

  error = asprintf(&query, "select %s from %s where id = %d",reg_key, db_key, id);

  if (error == -1) {
    return false;
  }

  sqlite3_prepare_v2(db, query, strlen(query), &stmt, NULL);

  while (sqlite3_step(stmt) != SQLITE_DONE) {
    if (sqlite3_column_type(stmt, 0) == SQLITE3_TEXT) {

      size_t len = strlen((char *) sqlite3_column_text(stmt, 0));

      if (len > 0) {
        len = len > size ? size : len;
        strncpy(buf, (char *) sqlite3_column_text(stmt, 0), len);
        if (len < size) {
          buf[len] = 0;
        } else if (len == size) {
          buf[len-1] = 0;
        }
        rval = true;
      }
    }
  }

  sqlite3_finalize(stmt);

  free(query);

  return rval;
}

static void ESP32_DB_fini()
{

    if (fln_db != NULL) {
      sqlite3_close(fln_db);
    }

    if (ogn_db != NULL) {
      sqlite3_close(ogn_db);
    }

    if (icao_db != NULL) {
      sqlite3_close(icao_db);
    }

    sqlite3_shutdown();

    SD.end();
}

#if 0
/* write sample data to I2S */
int i2s_write_sample_nb(uint32_t sample)
{
  return i2s_write_bytes((i2s_port_t)i2s_num, (const char *)&sample,
                          sizeof(uint32_t), 100);
}

/* read 4 bytes of data from wav file */
int read4bytes(File file, uint32_t *chunkId)
{
  int n = file.read((uint8_t *)chunkId, sizeof(uint32_t));
  return n;
}

/* these are functions to process wav file */
int readRiff(File file, wavRiff_t *wavRiff)
{
  int n = file.read((uint8_t *)wavRiff, sizeof(wavRiff_t));
  return n;
}
int readProps(File file, wavProperties_t *wavProps)
{
  int n = file.read((uint8_t *)wavProps, sizeof(wavProperties_t));
  return n;
}

static void play_file(char *filename)
{
  headerState_t state = HEADER_RIFF;

  File wavfile = SD.open(filename);

  if (wavfile) {
    int c = 0;
    int n;
    while (wavfile.available()) {
      switch(state){
        case HEADER_RIFF:
        wavRiff_t wavRiff;
        n = readRiff(wavfile, &wavRiff);
        if(n == sizeof(wavRiff_t)){
          if(wavRiff.chunkID == CCCC('R', 'I', 'F', 'F') && wavRiff.format == CCCC('W', 'A', 'V', 'E')){
            state = HEADER_FMT;
//            Serial.println("HEADER_RIFF");
          }
        }
        break;
        case HEADER_FMT:
        n = readProps(wavfile, &wavProps);
        if(n == sizeof(wavProperties_t)){
          state = HEADER_DATA;
        }
        break;
        case HEADER_DATA:
        uint32_t chunkId, chunkSize;
        n = read4bytes(wavfile, &chunkId);
        if(n == 4){
          if(chunkId == CCCC('d', 'a', 't', 'a')){
//            Serial.println("HEADER_DATA");
          }
        }
        n = read4bytes(wavfile, &chunkSize);
        if(n == 4){
//          Serial.println("prepare data");
          state = DATA;
        }
        //initialize i2s with configurations above
        i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
        i2s_set_pin((i2s_port_t)i2s_num, &pin_config);
        //set sample rates of i2s to sample rate of wav file
        i2s_set_sample_rates((i2s_port_t)i2s_num, wavProps.sampleRate);
        break;
        /* after processing wav file, it is time to process music data */
        case DATA:
        uint32_t data;
        n = read4bytes(wavfile, &data);
        i2s_write_sample_nb(data);
        break;
      }
    }
    wavfile.close();
  } else {
    Serial.println(F("error opening WAV file"));
  }
  if (state == DATA) {
    i2s_driver_uninstall((i2s_port_t)i2s_num); //stop & destroy i2s driver
  }
}

static void play_memory(const unsigned char *data, int size)
{
  headerState_t state = HEADER_RIFF;
  wavRiff_t *wavRiff;
  wavProperties_t *props;

  while (size > 0) {
    switch(state){
    case HEADER_RIFF:
      wavRiff = (wavRiff_t *) data;

      if(wavRiff->chunkID == CCCC('R', 'I', 'F', 'F') && wavRiff->format == CCCC('W', 'A', 'V', 'E')){
        state = HEADER_FMT;
      }
      data += sizeof(wavRiff_t);
      size -= sizeof(wavRiff_t);
      break;

    case HEADER_FMT:
      props = (wavProperties_t *) data;
      state = HEADER_DATA;
      data += sizeof(wavProperties_t);
      size -= sizeof(wavProperties_t);
      break;

    case HEADER_DATA:
      uint32_t chunkId, chunkSize;
      chunkId = *((uint32_t *) data);
      data += sizeof(uint32_t);
      size -= sizeof(uint32_t);
      chunkSize = *((uint32_t *) data);
      state = DATA;
      data += sizeof(uint32_t);
      size -= sizeof(uint32_t);

      //initialize i2s with configurations above
      i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
      i2s_set_pin((i2s_port_t)i2s_num, &pin_config);
      //set sample rates of i2s to sample rate of wav file
      i2s_set_sample_rates((i2s_port_t)i2s_num, props->sampleRate);
      break;

      /* after processing wav file, it is time to process music data */
    case DATA:
      i2s_write_sample_nb(*((uint32_t *) data));
      data += sizeof(uint32_t);
      size -= sizeof(uint32_t);
      break;
    }
  }

  if (state == DATA) {
    i2s_driver_uninstall((i2s_port_t)i2s_num); //stop & destroy i2s driver
  }
}

#include "Melody.h"
#endif

static void ESP32_TTS(char *message)
{
#if 0
  char filename[MAX_FILENAME_LEN];

  if (strcmp(message, "POST")) {
    if (settings->m.voice != VOICE_OFF && settings->m.adapter == ADAPTER_TTGO_T5S) {

      if (SD.cardType() == CARD_NONE)
        return;

      EPD_Message("VOICE", "ALERT");

      bool wdt_status = loopTaskWDTEnabled;

      if (wdt_status) {
        disableLoopWDT();
      }

      char *word = strtok (message, " ");

      while (word != NULL)
      {
          strcpy(filename, WAV_FILE_PREFIX);
          strcat(filename,  settings->m.voice == VOICE_1 ? VOICE1_SUBDIR :
                           (settings->m.voice == VOICE_2 ? VOICE2_SUBDIR :
                           (settings->m.voice == VOICE_3 ? VOICE3_SUBDIR :
                            "" )));
          strcat(filename, word);
          strcat(filename, WAV_FILE_SUFFIX);
          play_file(filename);
          word = strtok (NULL, " ");

          yield();
      }

      if (wdt_status) {
        enableLoopWDT();
      }
    }
  } else {
    if (settings->m.voice != VOICE_OFF && settings->m.adapter == ADAPTER_TTGO_T5S) {
      play_memory(melody_wav, (int) melody_wav_len);
    } else {

    }
  }
#endif
}

#include <AceButton.h>
using namespace ace_button;

AceButton button_mode(SOC_GPIO_PIN_TWATCH_BUTTON);

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {

#if 0
  // Print out a message for all events.
  if        (button == &button_mode) {
    Serial.print(F("MODE "));
  }

  Serial.print(F("handleEvent(): eventType: "));
  Serial.print(eventType);
  Serial.print(F("; buttonState: "));
  Serial.println(buttonState);
#endif

  if (hw_info.model == SOFTRF_MODEL_WEBTOP_USB) {
    switch (eventType) {
      case AceButton::kEventClicked:
      case AceButton::kEventReleased:
        if (button == &button_mode) {
          if (hw_info.display == DISPLAY_TFT_TTGO_240) {
            TFT_Mode_Cycle();
          }
        }
        break;
      case AceButton::kEventDoubleClicked:
        break;
      case AceButton::kEventLongPressed:
        if (button == &button_mode) {
          shutdown("  OFF  ");
          Serial.println(F("This will never be printed."));
        }
        break;
    }
  }
}

/* Callbacks for push button interrupt */
void onModeButtonEvent() {
  button_mode.check();
}

static void ESP32_Button_setup()
{
  int button_pin = (hw_info.model == SOFTRF_MODEL_WEBTOP_USB) ?
                   SOC_GPIO_PIN_TDONGLE_BUTTON : SOC_GPIO_PIN_TWATCH_BUTTON;

  // Button(s)) uses external pull up resistor.
  pinMode(button_pin, button_pin == 0 ? INPUT_PULLUP : INPUT);

  button_mode.init(button_pin);

  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  ButtonConfig* ModeButtonConfig = button_mode.getButtonConfig();
  ModeButtonConfig->setEventHandler(handleEvent);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureClick);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
//  ModeButtonConfig->setDebounceDelay(15);
  ModeButtonConfig->setClickDelay(600);
//  ModeButtonConfig->setDoubleClickDelay(1000);
  ModeButtonConfig->setLongPressDelay(2000);

  attachInterrupt(digitalPinToInterrupt(button_pin), onModeButtonEvent, CHANGE );
}

static void ESP32_Button_loop()
{
  button_mode.check();
}

static void ESP32_Button_fini()
{

}

static bool ESP32_Baro_setup() {

  bool rval = false;

  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    /* Pre-init 2nd ESP32 I2C bus to stick on these pins */
    Wire.begin(SOC_GPIO_PIN_TWATCH_EXT_SDA, SOC_GPIO_PIN_TWATCH_EXT_SCL);

    rval = true;
  }

  return rval;
}

static void ESP32_WDT_setup()
{
  enableLoopWDT();
}

static void ESP32_WDT_fini()
{
  disableLoopWDT();
}

static void ESP32_Service_Mode(boolean arg)
{
  if (arg) {
//    Serial.begin(SERIAL_IN_BR, SERIAL_IN_BITS);
#if defined(CONFIG_IDF_TARGET_ESP32)
     Serial.updateBaudRate(SERIAL_IN_BR);
#endif
  WiFi_fini();
    axp.setGPIOMode(AXP_GPIO_2, AXP_IO_OUTPUT_LOW_MODE);  // MCU_reset
    delay(10);
    axp.setGPIOMode(AXP_GPIO_1, AXP_IO_OUTPUT_HIGH_MODE); // BOOT0 high
    delay(100);
    axp.setGPIOMode(AXP_GPIO_2, AXP_IO_FLOATING_MODE);    // release MCU_reset (it has pull-up)
    delay(500);
    axp.setGPIOMode(AXP_GPIO_1, AXP_IO_FLOATING_MODE);    // release BOOT0 (it has pull-down)

    inServiceMode = true;
  } else {
//    Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#if defined(CONFIG_IDF_TARGET_ESP32)
    Serial.updateBaudRate(SERIAL_OUT_BR);
#endif
    axp.setGPIOMode(AXP_GPIO_2, AXP_IO_OUTPUT_LOW_MODE);  // MCU_reset
    delay(10);
    axp.setGPIOMode(AXP_GPIO_1, AXP_IO_OUTPUT_LOW_MODE);  // BOOT0 low
    delay(100);
    axp.setGPIOMode(AXP_GPIO_2, AXP_IO_FLOATING_MODE);    // release MCU_reset (it has pull-up)
    delay(500);
    axp.setGPIOMode(AXP_GPIO_1, AXP_IO_FLOATING_MODE);    // release BOOT0 (it has pull-down)

    inServiceMode = false;
  }
}

#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)

#if defined(USE_USB_HOST)

#include "usb/usb_host.h"

#include "usb_host.hpp"
#include "usb_acm.hpp"

#define USB_TX_FIFO_SIZE (1024)
#define USB_RX_FIFO_SIZE (1024)
#define USB_MAX_WRITE_CHUNK_SIZE 64

cbuf *USB_RX_FIFO, *USB_TX_FIFO;
USBhost host;           // host is required to detect any device, before USB class is initialized
USBacmDevice *device;   // when USB class is detected from descriptor

void acm_events(int event, void *data, size_t len)
{
    switch (event)
    {
    case CDC_CTRL_SET_CONTROL_LINE_STATE:
        log_i("CDC_CTRL_SET_CONTROL_LINE_STATE");
        device->setLineCoding(SERIAL_OUT_BR, 0, 0, 8);
        break;

    case CDC_DATA_IN:
    {
        device->INDATA();
        if (len > 0) {
          USB_RX_FIFO->write((char *) data,
                       USB_RX_FIFO->room() > len ?
                       len : USB_RX_FIFO->room());
        }
        break;
    }
    case CDC_DATA_OUT:

        break;

    case CDC_CTRL_SET_LINE_CODING:
        log_i("CDC_CTRL_SET_LINE_CODING");
        break;
    }
}

#define MAKE_USB_ID(v,p) ((uint32_t) v << 16 | (uint32_t) p)

void client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV)
    {
        host.open(event_msg);
        usb_device_info_t info = host.getDeviceInfo();
        log_i("device speed: %s, device address: %d, max ep_ctrl size: %d, config: %d", info.speed ? "USB_SPEED_FULL" : "USB_SPEED_LOW", info.dev_addr, info.bMaxPacketSize0, info.bConfigurationValue);
        const usb_device_desc_t *dev_desc = host.getDeviceDescriptor();

        uint8_t slave = SOFTRF_MODEL_UNKNOWN;

        switch (MAKE_USB_ID(dev_desc->idVendor, dev_desc->idProduct))
        {
        case MAKE_USB_ID(0x0483, 0x5740):
          slave = SOFTRF_MODEL_DONGLE; /* or Bracelet */
          break;
        case MAKE_USB_ID(0x239A, 0x8029):
          slave = SOFTRF_MODEL_BADGE;
          break;
        case MAKE_USB_ID(0x2341, 0x804d):
        case MAKE_USB_ID(0x2886, 0x802f):
          slave = SOFTRF_MODEL_ACADEMY;
          break;
        case MAKE_USB_ID(0x1d50, 0x6089):
          slave = SOFTRF_MODEL_ES;
          break;
        case MAKE_USB_ID(0x2e8a, 0x000a):
        case MAKE_USB_ID(0x2e8a, 0xf00a):
          slave = SOFTRF_MODEL_LEGO;
          break;
        case MAKE_USB_ID(0x1A86, 0x55D4): /* CH9102 */
          slave = SOFTRF_MODEL_PRIME_MK2;
          break;
        case MAKE_USB_ID(0x303a, 0x1001):
          slave = SOFTRF_MODEL_PRIME_MK3;
          break;
        case MAKE_USB_ID(0x15ba, 0x0044):
          slave = SOFTRF_MODEL_BALKAN;
          break;
        case MAKE_USB_ID(0x303a, 0x0100):
          slave = SOFTRF_MODEL_STANDALONE;
          break;
        }

        int offset = 0;
        for (size_t i = 0; i < dev_desc->bNumConfigurations; i++)
        {
            const usb_config_desc_t *config_desc = host.getConfigurationDescriptor();
            for (size_t n = 0; n < config_desc->bNumInterfaces; n++)
            {
                const usb_intf_desc_t *intf = usb_parse_interface_descriptor(config_desc, n, 0, &offset);
                if (intf->bInterfaceClass == 0x0a) // CDC ACM
                {
                    device = new USBacmDevice(config_desc, &host);
                    n = config_desc->bNumInterfaces;
                    if (device)
                    {
                        device->init();
                        device->onEvent(acm_events);
                        device->setControlLine(1, 1);
                        has_usb_client = true;
                        hw_info.slave  = slave;
                        device->INDATA();
                    }
                }

//                printf("config: %d[%d], interface: %d[%d], intf class: %d\n", i, dev_desc->bNumConfigurations, n, config_desc->bNumInterfaces, intf->bInterfaceClass);
            }
        }
    }
    else
    {
        log_w("DEVICE gone event");
        if (device)
        {
            device->deinit();
            delete(device);
            has_usb_client = false;
        }
        device = NULL;
    }
}

static void ESP32S2_USB_setup()
{
  USB_RX_FIFO = new cbuf(USB_RX_FIFO_SIZE);
  USB_TX_FIFO = new cbuf(USB_TX_FIFO_SIZE);

  host.registerClientCb(client_event_callback);
  host.init();

  delay(2000);
}

static void ESP32S2_USB_loop()
{
    if (device && device->isConnected())
    {
          uint8_t chunk[USB_MAX_WRITE_CHUNK_SIZE];
          size_t size = (USB_TX_FIFO->available() < USB_MAX_WRITE_CHUNK_SIZE ?
                         USB_TX_FIFO->available() : USB_MAX_WRITE_CHUNK_SIZE);

          USB_TX_FIFO->read((char *) chunk, size);
          device->OUTDATA(chunk, size);
    }
}

static void ESP32S2_USB_fini()
{
    delete(USB_RX_FIFO);
    delete(USB_TX_FIFO);
}

static int ESP32S2_USB_available()
{
  int rval = 0;

  rval = USB_RX_FIFO->available();

  return rval;
}

static int ESP32S2_USB_read()
{
  int rval = -1;

  rval = USB_RX_FIFO->read();

  return rval;
}

static size_t ESP32S2_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  rval = USB_TX_FIFO->write((char *) buffer,
                      (USB_TX_FIFO->room() > size ? size : USB_TX_FIFO->room()));

  return rval;
}

#elif ARDUINO_USB_CDC_ON_BOOT

#define USBSerial Serial

static void ESP32S2_USB_setup()
{

}

static void ESP32S2_USB_loop()
{

}

static void ESP32S2_USB_fini()
{

}

static int ESP32S2_USB_available()
{
  int rval = 0;

  if (USBSerial) {
    rval = USBSerial.available();
  }

  return rval;
}

static int ESP32S2_USB_read()
{
  int rval = -1;

  if (USBSerial) {
    rval = USBSerial.read();
  }

  return rval;
}

static size_t ESP32S2_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  if (USBSerial && (size < USBSerial.availableForWrite())) {
    rval = USBSerial.write(buffer, size);
  }

  return rval;
}
#endif /* USE_USB_HOST || ARDUINO_USB_CDC_ON_BOOT */

IODev_ops_t ESP32S2_USBSerial_ops = {
  "ESP32S2 USB",
  ESP32S2_USB_setup,
  ESP32S2_USB_loop,
  ESP32S2_USB_fini,
  ESP32S2_USB_available,
  ESP32S2_USB_read,
  ESP32S2_USB_write
};

#endif /* CONFIG_IDF_TARGET_ESP32S2 */

const SoC_ops_t ESP32_ops = {
  SOC_ESP32,
  "ESP32"
#if defined(CONFIG_IDF_TARGET_ESP32S2)
  "-S2"
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  "-S3"
#endif /* CONFIG_IDF_TARGET_ESP32SX */
  "" ,
  ESP32_setup,
  ESP32_post_init,
  ESP32_loop,
  ESP32_fini,
  ESP32_reset,
  ESP32_sleep_ms,
  ESP32_getChipId,
  ESP32_EEPROM_begin,
  ESP32_WiFi_set_param,
  ESP32_WiFi_hostname,
  ESP32_WiFiUDP_stopAll,
  ESP32_WiFi_transmit_UDP,
  ESP32_WiFi_Receive_UDP,
  ESP32_WiFi_clients_count,
  ESP32_swSer_begin,
  ESP32_swSer_enableRx,
  ESP32_maxSketchSpace,
  ESP32_Battery_setup,
  ESP32_Battery_voltage,
  ESP32_DB_init,
  ESP32_DB_query,
  ESP32_DB_fini,
  ESP32_TTS,
  ESP32_Button_setup,
  ESP32_Button_loop,
  ESP32_Button_fini,
  ESP32_Baro_setup,
  ESP32_WDT_setup,
  ESP32_WDT_fini,
  ESP32_Service_Mode,
#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32S3)
  &ESP32_Bluetooth_ops,
#else
  NULL,
#endif /* CONFIG_IDF_TARGET_ESP32S2 */
#if (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)) && \
   (ARDUINO_USB_CDC_ON_BOOT || defined(USE_USB_HOST))
  &ESP32S2_USBSerial_ops,
#else
  NULL,
#endif /* USE_USB_HOST */
};

#endif /* ESP32 */
