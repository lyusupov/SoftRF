/*
 * Platform_ESP8266.cpp
 * Copyright (C) 2018-2019 Linar Yusupov
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

#include <Wire.h>
#include <ESP8266TrueRandom.h>
#include <SPI.h>

#include "Platform_ESP8266.h"
#include "SoCHelper.h"
#include "SoundHelper.h"
#include "EEPROMHelper.h"
#include "RFHelper.h"
#include "WiFiHelper.h"
#include "LEDHelper.h"
#include "GNSSHelper.h"

#include <ets_sys.h>
#include <osapi.h>
#include <gpio.h>
#include <os_type.h>

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .rxtx = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN },
    .rst = SOC_GPIO_PIN_RST,
    .dio = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

Exp_SoftwareSerial swSer(SOC_GPIO_PIN_SWSER_RX, SOC_GPIO_PIN_SWSER_TX , false, 256);

ESP8266WebServer server ( 80 );

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIX_NUM, SOC_GPIO_PIN_LED,
                              NEO_GRB + NEO_KHZ800);

void ICACHE_FLASH_ATTR user_init()
{   
  // init gpio subsytem
  gpio_init();
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA3_U, FUNC_GPIO10);
}

static void ESP8266_setup()
{

}

static uint32_t ESP8266_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  return ESP.getChipId();
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
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
//    swSer.enableRx(false);

    if (var == REASON_DEFAULT_RST ||
        var == REASON_EXT_SYS_RST ||
        var == REASON_SOFT_RESTART) {
      tone(SOC_GPIO_PIN_BUZZER, 440, 500);delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 640, 500);delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 840, 500);delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 1040, 500);
    } else if (var == REASON_WDT_RST) {
      tone(SOC_GPIO_PIN_BUZZER, 440, 500);delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 1040, 500);delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 440, 500);delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 1040, 500);
    } else {
      tone(SOC_GPIO_PIN_BUZZER, 1040, 500);delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 840, 500);delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 640, 500);delay(500);
      tone(SOC_GPIO_PIN_BUZZER, 440, 500);
    }
    delay(600);

//    swSer.enableRx(true);
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

  return broadcastIp;
}

static void ESP8266_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  IPAddress ClientIP;

  if (WiFi.getMode() == WIFI_STA) {
    ClientIP = ESP8266_WiFi_get_broadcast();

    swSer.enableRx(false);

    Uni_Udp.beginPacket(ClientIP, port);
    Uni_Udp.write(buf, size);
    Uni_Udp.endPacket();

    swSer.enableRx(true);

  } else {
    struct station_info *stat_info = wifi_softap_get_station_info();

    while (stat_info != NULL) {
      ClientIP = stat_info->ip.addr;

      swSer.enableRx(false);

      Uni_Udp.beginPacket(ClientIP, port);
      Uni_Udp.write(buf, size);
      Uni_Udp.endPacket();

      swSer.enableRx(true);

      stat_info = STAILQ_NEXT(stat_info, next);
    }
    wifi_softap_free_station_info();
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

static bool ESP8266_EEPROM_begin(size_t size)
{
  EEPROM.begin(size);
  return true;
}

static void ESP8266_SPI_begin()
{
  SPI.begin();
}

static void ESP8266_swSer_begin(unsigned long baud)
{
  swSer.begin(baud);
}

static void ESP8266_swSer_enableRx(boolean arg)
{
  swSer.enableRx(arg);
}

static byte ESP8266_Display_setup()
{
  return DISPLAY_NONE;
}

static void ESP8266_Display_loop()
{

}

static void ESP8266_Battery_setup()
{

}

static float ESP8266_Battery_voltage()
{
  return analogRead (SOC_GPIO_PIN_BATTERY) / SOC_A0_VOLTAGE_DIVIDER ;
}

void ESP8266_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long ESP8266_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool ESP8266_Baro_setup() {
  if (hw_info.rf != RF_IC_SX1276 || RF_SX1276_RST_is_connected)
    return false;

#if DEBUG
    Serial.println(F("INFO: RESET pin of SX1276 radio is not connected to MCU."));
#endif

  Wire.pins(SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);

  return true;
}

static void ESP8266_UATSerial_begin(unsigned long baud)
{
  UATSerial.begin(baud);
}

static void ESP8266_CC13XX_restart()
{
  /* TBD */
}

static void ESP8266_WDT_setup()
{
  /* TBD */
}

const SoC_ops_t ESP8266_ops = {
  SOC_ESP8266,
  "ESP8266",
  ESP8266_setup,
  ESP8266_getChipId,
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
  ESP8266_WiFi_hostname,
  ESP8266_EEPROM_begin,
  ESP8266_SPI_begin,
  ESP8266_swSer_begin,
  ESP8266_swSer_enableRx,
  NULL, /* ESP8266 has no built-in Bluetooth */
  ESP8266_Display_setup,
  ESP8266_Display_loop,
  ESP8266_Battery_setup,
  ESP8266_Battery_voltage,
  ESP8266_GNSS_PPS_Interrupt_handler,
  ESP8266_get_PPS_TimeMarker,
  ESP8266_Baro_setup,
  ESP8266_UATSerial_begin,
  ESP8266_CC13XX_restart,
  ESP8266_WDT_setup
};

#endif /* ESP8266 */
