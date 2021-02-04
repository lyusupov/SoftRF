/*
 * Platform_ESP8266.cpp
 * Copyright (C) 2019-2021 Linar Yusupov
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

#include "SoCHelper.h"
#include "EPDHelper.h"
#include "EEPROMHelper.h"
#include "WiFiHelper.h"

#include "SkyView.h"

Exp_SoftwareSerial SerialInput(SOC_GPIO_PIN_SWSER_RX, SOC_GPIO_PIN_SWSER_TX , false, 256);

ESP8266WebServer server ( 80 );

/* Waveshare E-Paper ESP8266 Driver Board */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> epd_waveshare(GxEPD2_270(/*CS=D8*/ SS, /*DC=D2*/ 4, /*RST=D1*/ 5, /*BUSY=D0*/ 16));

/* NodeMCU, D8 does not work for me as CS pin for 2.7" e-Paper Pi HAT  */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> epd_nodemcu(GxEPD2_270(/*CS=D4*/ D4, /*DC=D2*/ 4, /*RST=D1*/ 5, /*BUSY=D0*/ 16));

static void ESP8266_setup()
{

}

static void ESP8266_fini()
{

}

static uint32_t ESP8266_getChipId()
{
  return ESP.getChipId();
}

static bool ESP8266_EEPROM_begin(size_t size)
{
  EEPROM.begin(size);
  return true;
}

static void ESP8266_WiFi_setOutputPower(int dB)
{
  WiFi.setOutputPower(dB);
}

static bool ESP8266_WiFi_hostname(String aHostname)
{
  return WiFi.hostname(aHostname);
}

static void ESP8266_swSer_begin(unsigned long baud)
{
  SerialInput.begin(baud);
}

static void ESP8266_swSer_enableRx(boolean arg)
{
  SerialInput.enableRx(arg);
}

static uint32_t ESP8266_maxSketchSpace()
{
  return (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
}

static void ESP8266_WiFiUDP_stopAll()
{
  WiFiUDP::stopAll();
}

static void ESP8266_Battery_setup()
{

}

static float ESP8266_Battery_voltage()
{
  return analogRead (SOC_GPIO_PIN_BATTERY) / SOC_A0_VOLTAGE_DIVIDER ;
}

static void ESP8266_EPD_setup()
{
  switch(settings->adapter)
  {
  case ADAPTER_WAVESHARE_ESP8266:
    display = &epd_waveshare;
    break;
  case ADAPTER_NODEMCU:
  default:
    display = &epd_nodemcu;
    break;
  }
}

static void ESP8266_EPD_fini()
{

}

static bool ESP8266_EPD_is_ready()
{
  return true;
}

static void ESP8266_EPD_update(int val)
{
  EPD_Update_Sync(val);
}

static size_t ESP8266_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  return WiFi_Receive_UDP(buf, max_size);
}

static int ESP8266_WiFi_clients_count()
{
  struct station_info *stat_info;
  int clients = 0;
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
  case WIFI_AP:
    stat_info = wifi_softap_get_station_info();

    while (stat_info != NULL) {
      clients++;

      stat_info = STAILQ_NEXT(stat_info, next);
    }
    wifi_softap_free_station_info();

    return clients;
  case WIFI_STA:
  default:
    return -1; /* error */
  }
}

static bool ESP8266_DB_init()
{
  return false;
}

static bool ESP8266_DB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  return false;
}

static void ESP8266_DB_fini()
{

}

static void ESP8266_TTS(char *message)
{
  if (!strcmp(message, "POST")) {
    if (hw_info.display == DISPLAY_EPD_2_7) {
      /* keep boot-time SkyView logo on the screen for 7 seconds */
      delay(7000);
    }
  }
}

static void ESP8266_Button_setup()
{

}

static void ESP8266_Button_loop()
{

}

static void ESP8266_Button_fini()
{

}

static void ESP8266_WDT_setup()
{
  /* TBD */
}

static void ESP8266_WDT_fini()
{
  /* TBD */
}

const SoC_ops_t ESP8266_ops = {
  SOC_ESP8266,
  "ESP8266",
  ESP8266_setup,
  ESP8266_fini,
  ESP8266_getChipId,
  ESP8266_EEPROM_begin,
  ESP8266_WiFi_setOutputPower,
  ESP8266_WiFi_hostname,
  ESP8266_swSer_begin,
  ESP8266_swSer_enableRx,
  ESP8266_maxSketchSpace,
  ESP8266_WiFiUDP_stopAll,
  ESP8266_Battery_setup,
  ESP8266_Battery_voltage,
  ESP8266_EPD_setup,
  ESP8266_EPD_fini,
  ESP8266_EPD_is_ready,
  ESP8266_EPD_update,
  ESP8266_WiFi_Receive_UDP,
  ESP8266_WiFi_clients_count,
  ESP8266_DB_init,
  ESP8266_DB_query,
  ESP8266_DB_fini,
  ESP8266_TTS,
  ESP8266_Button_setup,
  ESP8266_Button_loop,
  ESP8266_Button_fini,
  ESP8266_WDT_setup,
  ESP8266_WDT_fini,
  NULL
};

#endif /* ESP8266 */
