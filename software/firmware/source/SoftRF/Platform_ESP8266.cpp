/*
 * Platform_ESP8266.cpp
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
#if defined(ESP8266)

#include <ESP8266TrueRandom.h>

#include "Platform_ESP8266.h"
#include "SoCHelper.h"
#include "SoundHelper.h"
#include "EEPROMHelper.h"
#include "RFHelper.h"
#include "WiFiHelper.h"

#include <ets_sys.h>
#include <osapi.h>
#include <gpio.h>
#include <os_type.h>

// RFM95W pin mapping
const lmic_pinmap lmic_pins = {
    .nss = D8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {D0, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

SoftwareSerial swSer(SOC_GPIO_PIN_SWSER_RX, SOC_GPIO_PIN_SWSER_TX , false, 256);

ESP8266WebServer server ( 80 );

void ICACHE_FLASH_ATTR user_init()
{   
  // init gpio subsytem
  gpio_init();
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA3_U, FUNC_GPIO10);
}

static bool ESP8266_probe()
{

  return true;
}

static void ESP8266_setup()
{

}

static uint32_t ESP8266_getChipId()
{
  return ESP.getChipId();
}

static uint32_t ESP8266_getFlashChipId()
{
  return ESP.getFlashChipId();
}

static uint32_t ESP8266_getFlashChipRealSize()
{
  return ESP.getFlashChipRealSize();
}

static void* ESP8266_getResetInfoPtr()
{
  return (void *) ESP.getResetInfoPtr();
}

static String ESP8266_getResetInfo()
{
  return ESP.getResetInfo();
}

static String ESP8266_getResetReason()
{
  return ESP.getResetReason();
}

static long ESP8266_random(long howsmall, long howBig)
{
  return ESP8266TrueRandom.random(howsmall, howBig);
}

static void ESP8266_Sound_test(int var)
{
  if (settings->volume != BUZZER_OFF) {
    swSer.enableRx(false);

    if (var == REASON_DEFAULT_RST ||
        var == REASON_EXT_SYS_RST ||
        var == REASON_SOFT_RESTART) {
      tone(10, 440, 500);delay(500);
      tone(10, 640, 500);delay(500);
      tone(10, 840, 500);delay(500);
      tone(10, 1040, 500);
    } else if (var == REASON_WDT_RST) {
      tone(10, 440, 500);delay(500);
      tone(10, 1040, 500);delay(500);
      tone(10, 440, 500);delay(500);
      tone(10, 1040, 500);
    } else {
      tone(10, 1040, 500);delay(500);
      tone(10, 840, 500);delay(500);
      tone(10, 640, 500);delay(500);
      tone(10, 440, 500);
    }
    delay(600);

    swSer.enableRx(true);
  }
}

static uint32_t ESP8266_maxSketchSpace()
{
  return (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
}

static void ESP8266_WiFi_setOutputPower(int dB)
{
  WiFi.setOutputPower(dB);
}

static IPAddress ESP8266_WiFi_get_broadcast()
{
  struct ip_info ipinfo;
  IPAddress broadcastIp;

  if (WiFi.getMode() == WIFI_STA) {
    wifi_get_ip_info(STATION_IF, &ipinfo);
  } else {
    wifi_get_ip_info(SOFTAP_IF, &ipinfo);
  }
  broadcastIp = ~ipinfo.netmask.addr | ipinfo.ip.addr;
//  Serial.println(broadcastIp.toString());
  return broadcastIp;
}

static void ESP8266_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  IPAddress ClientIP;

/*
 * Unicast UDP transfer gives much better reception quality
 * for Android clients rather than broadcast one.
 * But it is still disabled due to detected instability of
 * NodeMCU & unicast UDPs which I have not not resolved yet.
 */
  if (true) {
//  if (WiFi.getMode() == WIFI_STA) {
    ClientIP = ESP8266_WiFi_get_broadcast();

    Uni_Udp.beginPacket(ClientIP, port);
    Uni_Udp.write(buf, size);
    Uni_Udp.endPacket();

  } else {
    struct station_info *stat_info = wifi_softap_get_station_info();

    while (stat_info != NULL) {
      ClientIP = stat_info->ip.addr;

      Uni_Udp.beginPacket(ClientIP, port);
      Uni_Udp.write(buf, size);
      Uni_Udp.endPacket();

      stat_info = STAILQ_NEXT(stat_info, next);
    }
  }
}

static void ESP8266_WiFiUDP_stopAll()
{
  WiFiUDP::stopAll();
}

static bool ESP8266_WiFi_hostname(String aHostname)
{
  return WiFi.hostname(aHostname);
}

SoC_ops_t ESP8266_ops = {
  "ESP8266",
  ESP8266_probe,
  ESP8266_setup,
  ESP8266_getChipId,
  ESP8266_getFlashChipId,
  ESP8266_getFlashChipRealSize,
  ESP8266_getResetInfoPtr,
  ESP8266_getResetInfo,
  ESP8266_getResetReason,
  ESP8266_random,
  ESP8266_Sound_test,
  ESP8266_maxSketchSpace,
  ESP8266_WiFi_setOutputPower,
  ESP8266_WiFi_get_broadcast,
  ESP8266_WiFi_transmit_UDP,
  ESP8266_WiFiUDP_stopAll,
  ESP8266_WiFi_hostname
};

#endif /* ESP8266 */
