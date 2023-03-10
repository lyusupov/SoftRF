/*
 * Platform_RP2040.cpp
 * Copyright (C) 2023 Linar Yusupov
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
#if defined(ARDUINO_ARCH_RP2040)

#include "SoCHelper.h"
#include "EPDHelper.h"
#include "EEPROMHelper.h"
#if !defined(EXCLUDE_WIFI)
#include "WiFiHelper.h"
#endif /* EXCLUDE_WIFI */

#include "SkyView.h"

#include <hardware/watchdog.h>

#if !defined(ARDUINO_ARCH_MBED)
#include "pico/unique_id.h"

#define PICO_ON_DEVICE 1
extern "C" {
#include "pico/binary_info.h"
}
#else
extern "C"
{
#include "hardware/flash.h"
}
#endif /* ARDUINO_ARCH_MBED */

#include <pico_sleep.h>

#define SOFTRF_DESC "Multifunctional, compatible DIY general aviation proximity awareness system"
#define SOFTRF_URL  "https://github.com/lyusupov/SoftRF"

#if !defined(ARDUINO_ARCH_MBED)
bi_decl(bi_program_name(SKYVIEW_IDENT));
bi_decl(bi_program_description(SOFTRF_DESC));
bi_decl(bi_program_version_string(SKYVIEW_FIRMWARE_VERSION));
bi_decl(bi_program_build_date_string(__DATE__));
bi_decl(bi_program_url(SOFTRF_URL));
extern char __flash_binary_end;
bi_decl(bi_binary_end((intptr_t)&__flash_binary_end));
#endif /* ARDUINO_ARCH_MBED */

#if defined(EXCLUDE_WIFI)
char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence
#else
WebServer server ( 80 );
#endif /* EXCLUDE_WIFI */

/* Waveshare E-Paper Driver Board */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> epd_waveshare_W3(GxEPD2_270(/*CS=D8*/ SS, /*DC=D2*/ 4, /*RST=D1*/ 5, /*BUSY=D0*/ 16));
GxEPD2_BW<GxEPD2_270_T91, GxEPD2_270_T91::HEIGHT> epd_waveshare_T91(GxEPD2_270_T91(/*CS=D8*/ SS, /*DC=D2*/ 4, /*RST=D1*/ 5, /*BUSY=D0*/ 16));

/* NodeMCU, D8 does not work for me as CS pin for 2.7" e-Paper Pi HAT  */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> epd_nodemcu_W3(GxEPD2_270(/*CS=D4*/ D4, /*DC=D2*/ 4, /*RST=D1*/ 5, /*BUSY=D0*/ 16));
GxEPD2_BW<GxEPD2_270_T91, GxEPD2_270_T91::HEIGHT> epd_nodemcu_T91(GxEPD2_270_T91(/*CS=D4*/ D4, /*DC=D2*/ 4, /*RST=D1*/ 5, /*BUSY=D0*/ 16));

static uint32_t bootCount __attribute__ ((section (".noinit")));
static bool wdt_is_active = false;

static RP2040_board_id RP2040_board    = RP2040_RAK11300; /* default */

#define UniqueIDsize 2

static union {
#if !defined(ARDUINO_ARCH_MBED)
  pico_unique_board_id_t RP2040_unique_flash_id;
#endif /* ARDUINO_ARCH_MBED */
  uint32_t RP2040_chip_id[UniqueIDsize];
};

static void RP2040_setup()
{
#if !defined(ARDUINO_ARCH_MBED)
  pico_get_unique_board_id(&RP2040_unique_flash_id);
#else
  flash_get_unique_id((uint8_t *)&RP2040_chip_id);
#endif /* ARDUINO_ARCH_MBED */

#if defined(ARDUINO_RASPBERRY_PI_PICO)
  RP2040_board = (SoC->getChipId() == 0xcf516424) ?
                  RP2040_WEACT : RP2040_RPIPICO;
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
  RP2040_board = RP2040_RPIPICO_W;
#endif /* ARDUINO_RASPBERRY_PI_PICO */
}

static void RP2040_fini()
{

}

static void RP2040_reset()
{
  NVIC_SystemReset();
}

static uint32_t RP2040_getChipId()
{
  return __builtin_bswap32(RP2040_chip_id[UniqueIDsize - 1]);
}

extern "C" void * _sbrk   (int);

static uint32_t RP2040_getFreeHeap()
{
  char top;
  return &top - reinterpret_cast<char*>(_sbrk(0));
}

static bool RP2040_EEPROM_begin(size_t size)
{
  EEPROM.begin(size);
  return true;
}

static void RP2040_WiFi_setOutputPower(int dB)
{
#if !defined(EXCLUDE_WIFI)
  WiFi.defaultLowPowerMode();
#endif /* EXCLUDE_WIFI */
}

static bool RP2040_WiFi_hostname(String aHostname)
{
#if !defined(EXCLUDE_WIFI)
  WiFi.hostname(aHostname.c_str());
#endif /* EXCLUDE_WIFI */
  return true;
}

static void RP2040_swSer_begin(unsigned long baud)
{
  SerialInput.begin(baud);
}

static void RP2040_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static uint32_t RP2040_maxSketchSpace()
{
  return 1048576; /* TBD */
}

static void RP2040_WiFiUDP_stopAll()
{
#if !defined(EXCLUDE_WIFI)
  WiFiUDP::stopAll();
#endif /* EXCLUDE_WIFI */
}

static void RP2040_Battery_setup()
{

}

static float RP2040_Battery_voltage()
{
  return analogRead (SOC_GPIO_PIN_BATTERY) / SOC_A0_VOLTAGE_DIVIDER ;
}

static void RP2040_EPD_setup()
{
  switch(settings->adapter)
  {
  case ADAPTER_WAVESHARE_ESP8266:
    display = &epd_waveshare_W3;
//    display = &epd_waveshare_T91;
    break;
  case ADAPTER_NODEMCU:
  default:
    display = &epd_nodemcu_W3;
//    display = &epd_nodemcu_T91;
    break;
  }
}

static void RP2040_EPD_fini()
{

}

static bool RP2040_EPD_is_ready()
{
  return true;
}

static void RP2040_EPD_update(int val)
{
  EPD_Update_Sync(val);
}

static size_t RP2040_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
#if !defined(EXCLUDE_WIFI)
  return WiFi_Receive_UDP(buf, max_size);
#else
  return 0;
#endif /* EXCLUDE_WIFI */
}

static int RP2040_WiFi_clients_count()
{
#if !defined(EXCLUDE_WIFI)
  struct station_info *stat_info;
  int clients = 0;
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
#if 0 /* TBD */
  case WIFI_AP:
    stat_info = wifi_softap_get_station_info();

    while (stat_info != NULL) {
      clients++;

      stat_info = STAILQ_NEXT(stat_info, next);
    }
    wifi_softap_free_station_info();

    return clients;
#endif
  case WIFI_STA:
  default:
    return -1; /* error */
  }
#else
  return -1;
#endif /* EXCLUDE_WIFI */
}

static bool RP2040_DB_init()
{
  return false;
}

static bool RP2040_DB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  return false;
}

static void RP2040_DB_fini()
{

}

static void RP2040_TTS(char *message)
{
  if (!strcmp(message, "POST")) {
    if (hw_info.display == DISPLAY_EPD_2_7) {
      /* keep boot-time SkyView logo on the screen for 7 seconds */
      delay(7000);
    }
  }
}

static void RP2040_Button_setup()
{

}

static void RP2040_Button_loop()
{

}

static void RP2040_Button_fini()
{

}

static void RP2040_WDT_setup()
{
  /* TBD */
}

static void RP2040_WDT_fini()
{
  /* TBD */
}

const SoC_ops_t RP2040_ops = {
  SOC_RP2040,
  "RP2040",
  RP2040_setup,
  RP2040_fini,
  RP2040_reset,
  RP2040_getChipId,
  RP2040_getFreeHeap,
  RP2040_EEPROM_begin,
  RP2040_WiFi_setOutputPower,
  RP2040_WiFi_hostname,
  RP2040_swSer_begin,
  RP2040_swSer_enableRx,
  RP2040_maxSketchSpace,
  RP2040_WiFiUDP_stopAll,
  RP2040_Battery_setup,
  RP2040_Battery_voltage,
  RP2040_EPD_setup,
  RP2040_EPD_fini,
  RP2040_EPD_is_ready,
  RP2040_EPD_update,
  RP2040_WiFi_Receive_UDP,
  RP2040_WiFi_clients_count,
  RP2040_DB_init,
  RP2040_DB_query,
  RP2040_DB_fini,
  RP2040_TTS,
  RP2040_Button_setup,
  RP2040_Button_loop,
  RP2040_Button_fini,
  RP2040_WDT_setup,
  RP2040_WDT_fini,
  NULL
};

#endif /* ARDUINO_ARCH_RP2040 */
