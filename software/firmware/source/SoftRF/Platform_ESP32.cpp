/*
 * Platform_ESP32.cpp
 * Copyright (C) 2018 Linar Yusupov
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
#include <soc/rtc_cntl_reg.h>
#include <Wire.h>

#include "Platform_ESP32.h"
#include "SoCHelper.h"
#include "SoundHelper.h"
#include "EEPROMHelper.h"
#include "RFHelper.h"
#include "WiFiHelper.h"
#include "BluetoothHelper.h"

#include <U8x8lib.h>

#define LEDC_CHANNEL_BUZZER     0
#define LEDC_RESOLUTION_BUZZER  8

// RFM95W pin mapping
const lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = SOC_GPIO_PIN_RST,
    .dio = {SOC_GPIO_PIN_DIO0, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

HardwareSerial Serial1(1);
TwoWire Wire1 = TwoWire(1);

WebServer server ( 80 );

U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C u8x8_ttgo(TTGO_V2_OLED_PIN_RST,
                                                TTGO_V2_OLED_PIN_SCL,
                                                TTGO_V2_OLED_PIN_SDA);

U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C u8x8_heltec(HELTEC_OLED_PIN_RST,
                                                  HELTEC_OLED_PIN_SCL,
                                                  HELTEC_OLED_PIN_SDA);

static U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C *u8x8 = NULL;


static void ESP32_setup()
{

#if ESP32_DISABLE_BROWNOUT_DETECTOR
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

  ledcSetup(LEDC_CHANNEL_BUZZER, 0, LEDC_RESOLUTION_BUZZER);
  ledcAttachPin(SOC_GPIO_PIN_BUZZER, LEDC_CHANNEL_BUZZER);

  analogReadResolution(10);
  analogSetPinAttenuation(SOC_GPIO_PIN_BATTERY, ADC_11db);

  /* Pre-init 1st ESP32 I2C bus to stick on these pins */
  Wire.begin(SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);

  /* SSD1306 I2C OLED probing */
  Wire1.begin(TTGO_V2_OLED_PIN_SDA , TTGO_V2_OLED_PIN_SCL);
  Wire1.beginTransmission(SSD1306_OLED_I2C_ADDR);
  if (Wire1.endTransmission() == 0) {
    u8x8 = &u8x8_ttgo;
  } else {
    /*
     * This does NOT work well with "stock" ESP32 Arduino Core's TwoWire implementation yet.
     * Use I2C code (lib and hal) from Chuck Todd's repo instead:
     * https://github.com/stickbreaker/arduino-esp32
     */
    Wire1.begin(HELTEC_OLED_PIN_SDA , HELTEC_OLED_PIN_SCL);
    Wire1.beginTransmission(SSD1306_OLED_I2C_ADDR);
    if (Wire1.endTransmission() == 0) {
      u8x8 = &u8x8_heltec;
    }
  }

  if (u8x8) {
    u8x8->begin();
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    u8x8->clear();
    u8x8->draw2x2String(2, 3, "SoftRF");
  }
}

static uint32_t ESP32_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  union {
    uint8_t efuse_mac[6];
    uint64_t chipmacid;
  };
  chipmacid = ESP.getEfuseMac();

  return (uint32_t) efuse_mac[5]        | (efuse_mac[4] << 8) | \
                   (efuse_mac[3] << 16) | (efuse_mac[2] << 24);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static uint32_t ESP32_getFlashChipId()
{
/* not implemented yet */
  return 0;
}

static uint32_t ESP32_getFlashChipRealSize()
{
/* not implemented yet */
  return 0;
}

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static void* ESP32_getResetInfoPtr()
{
/* not implemented yet */
  return (void *) &reset_info;
}

static String ESP32_getResetInfo()
{
/* not implemented yet */
  return "No reset information available.";
}

static String ESP32_getResetReason()
{
/* not implemented yet */
  return "No reset reason.";
}

static long ESP32_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

static void ESP32_Sound_test(int var)
{
  if (settings->volume != BUZZER_OFF) {

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

  if (WiFi.getMode() == WIFI_STA) {
    ClientIP = ESP32_WiFi_get_broadcast();

    Uni_Udp.beginPacket(ClientIP, port);
    Uni_Udp.write(buf, size);
    Uni_Udp.endPacket();

  } else {
    wifi_sta_list_t stations;
    ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));

    tcpip_adapter_sta_list_t infoList;
    ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));

    int i = 0;
    while(i < infoList.num) {
      ClientIP = infoList.sta[i++].ip.addr;

      Uni_Udp.beginPacket(ClientIP, port);
      Uni_Udp.write(buf, size);
      Uni_Udp.endPacket();
    }
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

static bool ESP32_EEPROM_begin(size_t size)
{
  return EEPROM.begin(size);
}

static void ESP32_SPI_begin()
{
  SPI.begin(SOC_GPIO_PIN_SCK, SOC_GPIO_PIN_MISO, SOC_GPIO_PIN_MOSI, SOC_GPIO_PIN_SS);
}

static void ESP32_swSer_begin(unsigned long baud)
{
  swSer.begin(baud, SERIAL_8N1, SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
}

static void ESP32_swSer_enableRx(boolean arg)
{

}

static bool OLED_display_frontpage = false;
static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
extern uint32_t tx_packets_counter, rx_packets_counter;

const char *OLED_Protocol_ID[] = {
  [RF_PROTOCOL_LEGACY]    = "L",
  [RF_PROTOCOL_OGNTP]     = "O",
  [RF_PROTOCOL_P3I]       = "P",
  [RF_PROTOCOL_ADSB_1090] = "A",
  [RF_PROTOCOL_ADSB_UAT]  = "U",
  [RF_PROTOCOL_FANET]     = "F"
};

static void ESP32_OLED_loop()
{
  char buf[16];

  if (u8x8) {
    if (!OLED_display_frontpage) {

      u8x8->clear();

      u8x8->drawString(1, 1, "ID");

      itoa(ThisAircraft.addr & 0xFFFFFF, buf, 16);
      u8x8->draw2x2String(0, 2, buf);

      u8x8->drawString(8, 1, "PROTOCOL");

      u8x8->draw2x2String(14, 2, OLED_Protocol_ID[ThisAircraft.protocol]);

      u8x8->drawString(1, 5, "RX");

      itoa(rx_packets_counter % 1000, buf, 10);
      u8x8->draw2x2String(0, 6, buf);

      u8x8->drawString(9, 5, "TX");

      itoa(tx_packets_counter % 1000, buf, 10);
      u8x8->draw2x2String(8, 6, buf);

      OLED_display_frontpage = true;
    } else {
      if (rx_packets_counter > prev_rx_packets_counter) {
        itoa(rx_packets_counter % 1000, buf, 10);
        u8x8->draw2x2String(0, 6, buf);
        prev_rx_packets_counter = rx_packets_counter;
      }
      if (tx_packets_counter > prev_tx_packets_counter) {
        itoa(tx_packets_counter % 1000, buf, 10);
        u8x8->draw2x2String(8, 6, buf);
        prev_tx_packets_counter = tx_packets_counter;
      }
    }
  }
}

SoC_ops_t ESP32_ops = {
  SOC_ESP32,
  "ESP32",
  ESP32_setup,
  ESP32_getChipId,
  ESP32_getFlashChipId,
  ESP32_getFlashChipRealSize,
  ESP32_getResetInfoPtr,
  ESP32_getResetInfo,
  ESP32_getResetReason,
  ESP32_random,
  ESP32_Sound_test,
  ESP32_maxSketchSpace,
  ESP32_WiFi_setOutputPower,
  ESP32_WiFi_get_broadcast,
  ESP32_WiFi_transmit_UDP,
  ESP32_WiFiUDP_stopAll,
  ESP32_WiFi_hostname,
  ESP32_EEPROM_begin,
  ESP32_SPI_begin,
  ESP32_swSer_begin,
  ESP32_swSer_enableRx,
  &ESP32_Bluetooth_ops,
  ESP32_OLED_loop
};

#endif /* ESP32 */
