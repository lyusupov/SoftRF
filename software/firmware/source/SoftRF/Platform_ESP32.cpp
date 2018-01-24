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

#include "Platform_ESP32.h"
#include "SoCHelper.h"
#include "SoundHelper.h"
#include "EEPROMHelper.h"
#include "RFHelper.h"
#include "WiFiHelper.h"

// RFM95W pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {23, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

HardwareSerial Serial1(1);

WebServer server ( 80 );

static bool ESP32_probe()
{
/* not implemented yet */
  return true;
}

static void ESP32_setup()
{
/* not implemented yet */
}

static uint32_t ESP32_getChipId()
{
  return ESP.getEfuseMac() & 0xFFFFFFFF;
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
/* not implemented yet */
}

static uint32_t ESP32_maxSketchSpace()
{
  return 0x140000;
}

static void ESP32_WiFi_setOutputPower(int dB)
{
/* not implemented yet */
}

static IPAddress ESP32_WiFi_get_broadcast()
{
/* not implemented yet */
  IPAddress broadcastIp((192,168,1,255));

  return broadcastIp;
}

static void ESP32_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
/* not implemented yet */
}

static void ESP32_WiFiUDP_stopAll()
{
/* not implemented yet */
}

static bool ESP32_WiFi_hostname(String aHostname)
{
  return WiFi.setHostname(aHostname.c_str());
}

SoC_ops_t ESP32_ops = {
  "ESP32",
  ESP32_probe,
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
  ESP32_WiFi_hostname
};

#endif /* ESP32 */
