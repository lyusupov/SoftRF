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
#include "EEPROMHelper.h"
#if !defined(EXCLUDE_WIFI)
#include "WiFiHelper.h"
#endif /* EXCLUDE_WIFI */

#include "SkyWatch.h"

#include <hardware/watchdog.h>
#include <Adafruit_SleepyDog.h>

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

#if defined(USE_TINYUSB)
#if defined(USE_USB_HOST)
#include "pio_usb.h"
#endif /* USE_USB_HOST */
#include "Adafruit_TinyUSB.h"
#endif /* USE_TINYUSB */

#define SOFTRF_DESC "Multifunctional, compatible DIY general aviation proximity awareness system"
#define SOFTRF_URL  "https://github.com/lyusupov/SoftRF"

#if !defined(ARDUINO_ARCH_MBED)
bi_decl(bi_program_name(WEBTOP_IDENT));
bi_decl(bi_program_description(SOFTRF_DESC));
bi_decl(bi_program_version_string(SKYWATCH_FIRMWARE_VERSION));
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

static uint32_t bootCount __attribute__ ((section (".noinit")));
static bool wdt_is_active              = false;

static RP2040_board_id RP2040_board    = RP2040_RPIPICO_W; /* default */
const char *RP2040_Device_Manufacturer = SOFTRF_IDENT;
const char *RP2040_Device_Model        = WEBTOP_IDENT " Pico";
const uint16_t RP2040_Device_Version   = SKYWATCH_USB_FW_VERSION;

#define UniqueIDsize                   2

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

#if defined(USE_TINYUSB)
  USBDevice.setManufacturerDescriptor(RP2040_Device_Manufacturer);
  USBDevice.setProductDescriptor(RP2040_Device_Model);
  USBDevice.setDeviceVersion(RP2040_Device_Version);
#endif /* USE_TINYUSB */

#if !defined(ARDUINO_ARCH_MBED)
  SerialInput.setRX(SOC_GPIO_PIN_CONS_RX);
  SerialInput.setTX(SOC_GPIO_PIN_CONS_TX);
  SerialInput.setFIFOSize(255);
  SerialInput.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

  Serial_GNSS_In.setRX(SOC_GPIO_PIN_GNSS_RX);
  Serial_GNSS_In.setTX(SOC_GPIO_PIN_GNSS_TX);
  Serial_GNSS_In.setFIFOSize(255);
#endif /* ARDUINO_ARCH_MBED */

  hw_info.model = SOFTRF_MODEL_WEBTOP_USB;
  hw_info.revision = HW_REV_PICO_W;

  RP2040_board = (SoC->getChipId() == 0xcf516424) ?
                  RP2040_WEACT : RP2040_board;

  USBSerial.begin(SERIAL_OUT_BR);

  for (int i=0; i < 20; i++) {if (USBSerial) break; else delay(100);}
}

static void RP2040_post_init()
{

}

static void RP2040_loop()
{
  if (wdt_is_active) {
#if !defined(ARDUINO_ARCH_MBED)
    Watchdog.reset();
#endif /* ARDUINO_ARCH_MBED */
  }
}

static void RP2040_fini()
{
#if 0 /* TBD */
  // back from dormant state
  rosc_enable();
  clocks_init();

  rp2040.restart();
#endif
}

static void RP2040_reset()
{
  rp2040.restart();
}

static void RP2040_sleep_ms(int ms)
{
  /* TODO */
}

static uint32_t RP2040_getChipId()
{
  return __builtin_bswap32(RP2040_chip_id[UniqueIDsize - 1]);
}

static uint32_t RP2040_getFreeHeap()
{
  return rp2040.getFreeHeap();
}

static bool RP2040_EEPROM_begin(size_t size)
{
  EEPROM.begin(size);
  return true;
}

static void RP2040_WiFi_set_param(int ndx, int value)
{
#if !defined(EXCLUDE_WIFI)
  switch (ndx)
  {
  case WIFI_PARAM_TX_POWER:
    switch (value)
    {
    case WIFI_TX_POWER_MAX:
      WiFi.aggressiveLowPowerMode();
      break;
    case WIFI_TX_POWER_MIN:
      WiFi.defaultLowPowerMode();
      break;
    case WIFI_TX_POWER_MED:
    default:
      WiFi.noLowPowerMode();
      break;
    }
    break;
  case WIFI_PARAM_DHCP_LEASE_TIME:
    if (WiFi.getMode() == WIFI_AP) {
      /* TBD */
    }
    break;
  default:
    break;
  }
#endif /* EXCLUDE_WIFI */
}

static bool RP2040_WiFi_hostname(String aHostname)
{
  bool rval = false;
#if !defined(EXCLUDE_WIFI)
  if (RP2040_board != RP2040_WEACT && rp2040.isPicoW()) {
    WiFi.hostname(aHostname.c_str());
    rval = true;
  }
#endif /* EXCLUDE_WIFI */
  return rval;
}

static void RP2040_WiFiUDP_stopAll()
{
#if !defined(EXCLUDE_WIFI)
  WiFiUDP::stopAll();
#endif /* EXCLUDE_WIFI */
}

#if 0 // !defined(EXCLUDE_WIFI)
static IPAddress RP2040_WiFi_get_broadcast()
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
#endif /* EXCLUDE_WIFI */

static void RP2040_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
#if !defined(EXCLUDE_WIFI)
  IPAddress ClientIP;
  struct station_info *stat_info;
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
#if 0 /* TBD */
  case WIFI_STA:
    ClientIP = RP2040_WiFi_get_broadcast();

    Uni_Udp.beginPacket(ClientIP, port);
    Uni_Udp.write(buf, size);
    Uni_Udp.endPacket();

    break;
  case WIFI_AP:
    stat_info = wifi_softap_get_station_info();

    while (stat_info != NULL) {
      ClientIP = stat_info->ip.addr;

      Uni_Udp.beginPacket(ClientIP, port);
      Uni_Udp.write(buf, size);
      Uni_Udp.endPacket();

      stat_info = STAILQ_NEXT(stat_info, next);
    }
    wifi_softap_free_station_info();
    break;
#endif
  case WIFI_OFF:
  default:
    break;
  }
#endif /* EXCLUDE_WIFI */
}

static size_t RP2040_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  return 0; // WiFi_Receive_UDP(buf, max_size);
}

static int RP2040_WiFi_clients_count()
{
#if !defined(EXCLUDE_WIFI)
  struct station_info *stat_info;
  int clients = 0;
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
    return WiFi.softAPgetStationNum();
  case WIFI_STA:
  default:
    return -1; /* error */
  }
#else
  return -1;
#endif /* EXCLUDE_WIFI */
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

static void RP2040_Battery_setup()
{

}

static float RP2040_Battery_voltage()
{
  return analogRead (SOC_GPIO_PIN_BATTERY) / SOC_A0_VOLTAGE_DIVIDER ;
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

static bool RP2040_Baro_setup()
{
  return false;
}

static void RP2040_WDT_setup()
{
#if !defined(ARDUINO_ARCH_MBED)
  Watchdog.enable(4000);
#endif /* ARDUINO_ARCH_MBED */
  wdt_is_active = true;
}

static void RP2040_WDT_fini()
{
  if (wdt_is_active) {
#if !defined(ARDUINO_ARCH_MBED)
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
#endif /* ARDUINO_ARCH_MBED */
    wdt_is_active = false;
  }
}

static void RP2040_Service_Mode(boolean arg)
{

}

const SoC_ops_t RP2040_ops = {
  SOC_RP2040,
  "RP2040",
  RP2040_setup,
  RP2040_post_init,
  RP2040_loop,
  RP2040_fini,
  RP2040_reset,
  RP2040_sleep_ms,
  RP2040_getChipId,
  RP2040_getFreeHeap,
  RP2040_EEPROM_begin,
  RP2040_WiFi_set_param,
  RP2040_WiFi_hostname,
  RP2040_WiFiUDP_stopAll,
  RP2040_WiFi_transmit_UDP,
  RP2040_WiFi_Receive_UDP,
  RP2040_WiFi_clients_count,
  RP2040_swSer_begin,
  RP2040_swSer_enableRx,
  RP2040_maxSketchSpace,
  RP2040_Battery_setup,
  RP2040_Battery_voltage,
  RP2040_DB_init,
  RP2040_DB_query,
  RP2040_DB_fini,
  RP2040_TTS,
  RP2040_Button_setup,
  RP2040_Button_loop,
  RP2040_Button_fini,
  RP2040_Baro_setup,
  RP2040_WDT_setup,
  RP2040_WDT_fini,
  RP2040_Service_Mode,
  NULL,
  NULL
};

#endif /* ARDUINO_ARCH_RP2040 */
