/*
 * Platform_LPC43.cpp
 * Copyright (C) 2021-2022 Linar Yusupov
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

#if defined(HACKRF_ONE)

#include "src/system/Time.h"
#include "src/driver/LED.h"
#include "src/driver/GNSS.h"
#include "src/driver/RF.h"
#include "src/driver/Sound.h"
#include "src/driver/EEPROM.h"
#include "src/driver/Battery.h"
#include "src/protocol/data/GDL90.h"
#include "src/protocol/data/NMEA.h"
#include "src/protocol/data/D1090.h"
#include "src/system/SoC.h"
#include "src/driver/Baro.h"
#include <src/TrafficHelper.h>
#include "src/protocol/data/JSON.h"
#include "src/driver/Bluetooth.h"
#include "src/driver/WiFi.h"

#include "Adafruit_USBD_Device.h"

eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;

ufo_t ThisAircraft;

char UDPpacketBuffer[UDP_PACKET_BUFSIZE]; // buffer to hold incoming and outgoing packets

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

hardware_info_t hw_info = {
  .model    = DEFAULT_SOFTRF_MODEL,
  .revision = 0,
  .soc      = SOC_NONE,
  .rf       = RF_IC_NONE,
  .gnss     = GNSS_MODULE_NONE,
  .baro     = BARO_MODULE_NONE,
  .display  = DISPLAY_NONE,
  .storage  = STORAGE_NONE,
  .rtc      = RTC_NONE,
  .imu      = IMU_NONE
};

const uint16_t LPC43_Vendor_Id = 0x1d50; /* OpenMoko, Inc. */
const uint16_t LPC43_Device_Id = 0x6089; /* HackRF One */
const char *LPC43_Device_Manufacturer = SOFTRF_IDENT;
const char *LPC43_Device_Model = "ES Edition";
const uint16_t LPC43_Device_Version = SOFTRF_USB_FW_VERSION;

void LPC43_setup(void)
{
  eeprom_block.field.magic                  = SOFTRF_EEPROM_MAGIC;
  eeprom_block.field.version                = SOFTRF_EEPROM_VERSION;
  eeprom_block.field.settings.mode          = SOFTRF_MODE_NORMAL;
  eeprom_block.field.settings.rf_protocol   = RF_PROTOCOL_ADSB_1090;
  eeprom_block.field.settings.band          = RF_BAND_EU;
  eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_GLIDER;
  eeprom_block.field.settings.txpower       = RF_TX_POWER_OFF;
  eeprom_block.field.settings.volume        = BUZZER_OFF;
  eeprom_block.field.settings.pointer       = DIRECTION_NORTH_UP;
  eeprom_block.field.settings.bluetooth     = BLUETOOTH_OFF;
  eeprom_block.field.settings.alarm         = TRAFFIC_ALARM_DISTANCE;

  eeprom_block.field.settings.nmea_g        = true;
  eeprom_block.field.settings.nmea_p        = false;
  eeprom_block.field.settings.nmea_l        = true;
  eeprom_block.field.settings.nmea_s        = true;
  eeprom_block.field.settings.nmea_out      = NMEA_UART;
  eeprom_block.field.settings.gdl90         = GDL90_OFF;
  eeprom_block.field.settings.d1090         = D1090_OFF;
  eeprom_block.field.settings.json          = JSON_OFF;
  eeprom_block.field.settings.stealth       = false;
  eeprom_block.field.settings.no_track      = false;
  eeprom_block.field.settings.power_save    = POWER_SAVE_NONE;
  eeprom_block.field.settings.freq_corr     = 0;
  eeprom_block.field.settings.igc_key[0]    = 0;
  eeprom_block.field.settings.igc_key[1]    = 0;
  eeprom_block.field.settings.igc_key[2]    = 0;
  eeprom_block.field.settings.igc_key[3]    = 0;

}

static void LPC43_post_init()
{

}

static void LPC43_loop()
{

}

static void LPC43_fini(int reason)
{

}

static void LPC43_reset()
{

}

extern uint32_t LPC43_Device_ID;

static uint32_t LPC43_getChipId()
{
  return DevID_Mapper(LPC43_Device_ID);
}

static void* LPC43_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static long LPC43_random(long howsmall, long howBig)
{
  return howsmall + random() % (howBig - howsmall);
}

static void LPC43_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{

}

static void LPC43_SPI_begin()
{

}

static void LPC43_swSer_begin(unsigned long baud)
{

}

static byte LPC43_Display_setup()
{
  byte rval = DISPLAY_NONE;

  return rval;
}

static void LPC43_Display_loop()
{

}

static void LPC43_Display_fini(int reason)
{

}

static void LPC43_Battery_setup()
{
  /* TBD */
}

static float LPC43_Battery_param(uint8_t param)
{
  float rval;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = BATTERY_THRESHOLD_USB;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = BATTERY_CUTOFF_USB;
    break;

  case BATTERY_PARAM_CHARGE:
    /* TBD */

    rval = 100;
    break;

  case BATTERY_PARAM_VOLTAGE:
  default:
    rval = BATTERY_THRESHOLD_USB + 0.05;
    break;
  }

  return rval;
}

void LPC43_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long LPC43_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static void LPC43_UATSerial_begin(unsigned long baud)
{

}

static void LPC43_UATModule_restart()
{

}

static void LPC43_WDT_setup()
{
  /* TBD */
}

static void LPC43_WDT_fini()
{
  /* TBD */
}

static void LPC43_Button_setup()
{
  /* TODO */
}

static void LPC43_Button_loop()
{
  /* TODO */
}

static void LPC43_Button_fini()
{
  /* TODO */
}

static void LPC43_USB_setup()
{
  TinyUSB_Device_Init(0);

  USBDevice.setID(LPC43_Vendor_Id, LPC43_Device_Id);
  USBDevice.setManufacturerDescriptor(LPC43_Device_Manufacturer);
  USBDevice.setProductDescriptor(LPC43_Device_Model);
  USBDevice.setDeviceVersion(LPC43_Device_Version);

  if (USBSerial && USBSerial != Serial) {
    USBSerial.begin(SERIAL_OUT_BR);
  }
}

static void LPC43_USB_loop()
{
  USBDevice.task();
}

static void LPC43_USB_fini()
{
  if (USBSerial && USBSerial != Serial) {
    USBSerial.end();
  }
}

static int LPC43_USB_available()
{
  int rval = 0;

  if (USBSerial) {
    rval = USBSerial.available();
  }

  return rval;
}

static int LPC43_USB_read()
{
  int rval = -1;

  if (USBSerial) {
    rval = USBSerial.read();
  }

  return rval;
}

static size_t LPC43_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  if (USBSerial && (size < USBSerial.availableForWrite())) {
    rval = USBSerial.write(buffer, size);
  }

  return rval;
}

IODev_ops_t LPC43_USBSerial_ops = {
  "LPC43 USBSerial",
  LPC43_USB_setup,
  LPC43_USB_loop,
  LPC43_USB_fini,
  LPC43_USB_available,
  LPC43_USB_read,
  LPC43_USB_write
};

const SoC_ops_t LPC43_ops = {
  SOC_LPC43,
  "LPC43",
  LPC43_setup,
  LPC43_post_init,
  LPC43_loop,
  LPC43_fini,
  LPC43_reset,
  LPC43_getChipId,
  LPC43_getResetInfoPtr,
  NULL,
  NULL,
  NULL,
  LPC43_random,
  NULL,
  NULL,
  NULL,
  NULL,
  LPC43_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  LPC43_SPI_begin,
  LPC43_swSer_begin,
  NULL,
  NULL,
  &LPC43_USBSerial_ops,
  NULL,
  LPC43_Display_setup,
  LPC43_Display_loop,
  LPC43_Display_fini,
  LPC43_Battery_setup,
  LPC43_Battery_param,
  NULL,
  LPC43_get_PPS_TimeMarker,
  NULL,
  LPC43_UATSerial_begin,
  LPC43_UATModule_restart,
  LPC43_WDT_setup,
  LPC43_WDT_fini,
  LPC43_Button_setup,
  LPC43_Button_loop,
  LPC43_Button_fini,
  NULL
};

void setup_CPP(void)
{
  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  if (SoC->USB_ops) {
     SoC->USB_ops->setup();
  }

  Serial.begin(SERIAL_OUT_BR);

  Serial.println();
  Serial.print(F(SOFTRF_IDENT "-"));
  Serial.print(SoC->name);
  Serial.print(F(" FW.REV: " SOFTRF_FIRMWARE_VERSION " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));
  Serial.println(F("Copyright (C) 2015-2022 Linar Yusupov. All rights reserved."));
  Serial.flush();

  ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;
  ThisAircraft.aircraft_type = settings->aircraft_type;
  ThisAircraft.protocol = settings->rf_protocol;
  ThisAircraft.stealth  = settings->stealth;
  ThisAircraft.no_track = settings->no_track;

  hw_info.gnss = GNSS_setup();
  Traffic_setup();
}

void main_loop_CPP(void)
{

  GNSS_loop();

  ThisAircraft.timestamp = now();
  if (isValidFix()) {
    ThisAircraft.latitude = gnss.location.lat();
    ThisAircraft.longitude = gnss.location.lng();
    ThisAircraft.altitude = gnss.altitude.meters();
    ThisAircraft.course = gnss.course.deg();
    ThisAircraft.speed = gnss.speed.knots();
    ThisAircraft.hdop = (uint16_t) gnss.hdop.value();
    ThisAircraft.geoid_separation = gnss.separation.meters();
  }

//  success = RF_Receive();
//  if (success && isValidFix()) ParseData();

  if (isValidFix()) {
    Traffic_loop();
  }

  Sound_loop();

  NMEA_loop();

  SoC->loop();

  if (SoC->Bluetooth_ops) {
    SoC->Bluetooth_ops->loop();
  }

  if (SoC->USB_ops) {
    SoC->USB_ops->loop();
  }

  if (SoC->UART_ops) {
     SoC->UART_ops->loop();
  }

  Battery_loop();

  SoC->Button_loop();

#if defined(TAKE_CARE_OF_MILLIS_ROLLOVER)
  /* restart the device when uptime is more than 47 days */
  if (millis() > (47 * 24 * 3600 * 1000UL)) {
    SoC->reset();
  }
#endif /* TAKE_CARE_OF_MILLIS_ROLLOVER */

  yield();
}

extern mode_s_t state;

void once_per_second_task_CPP(void)
{
  struct mode_s_aircraft *a = state.aircrafts;

  while (a) {
    if (a->even_cprtime && a->odd_cprtime &&
        abs((long) (a->even_cprtime - a->odd_cprtime)) <= MODE_S_INTERACTIVE_TTL * 1000 ) {
      if (es1090_decode(a, &ThisAircraft, &fo)) {
         memset(fo.raw, 0, sizeof(fo.raw));

     int i;

      Traffic_Update(&fo);

      for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (Container[i].addr == fo.addr) {
          uint8_t alert_bak = Container[i].alert;
          Container[i] = fo;
          Container[i].alert = alert_bak;
          return;
        }
      }

      int max_dist_ndx = 0;
      int min_level_ndx = 0;

      for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (now() - Container[i].timestamp > ENTRY_EXPIRATION_TIME) {
          Container[i] = fo;
          return;
        }
#if !defined(EXCLUDE_TRAFFIC_FILTER_EXTENSION)
        if  (Container[i].distance > Container[max_dist_ndx].distance)  {
          max_dist_ndx = i;
        }
        if  (Container[i].alarm_level < Container[min_level_ndx].alarm_level)  {
          min_level_ndx = i;
        }
#endif /* EXCLUDE_TRAFFIC_FILTER_EXTENSION */
      }
#if !defined(EXCLUDE_TRAFFIC_FILTER_EXTENSION)
      if (fo.alarm_level > Container[min_level_ndx].alarm_level) {
        Container[min_level_ndx] = fo;
        return;
      }

      if (fo.distance    <  Container[max_dist_ndx].distance &&
          fo.alarm_level >= Container[max_dist_ndx].alarm_level) {
        Container[max_dist_ndx] = fo;
        return;
      }
#endif /* EXCLUDE_TRAFFIC_FILTER_EXTENSION */


      }
    }
    a = a->next;
  }

////    NMEA_Export();
//    GDL90_Export();

  if (isValidFix()) {
//    D1090_Export();
  }

  if (isValidFix()) {
    Traffic_loop();
  }
}

void shutdown(int reason)
{

}

#endif /* HACKRF_ONE */
