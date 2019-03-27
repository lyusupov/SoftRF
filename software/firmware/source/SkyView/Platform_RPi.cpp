/*
 * Platform_RPi.cpp
 * Copyright (C) 2019 Linar Yusupov
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

/*
 * Usage example:
 *
 *  pi@raspberrypi $ make -f Makefile.RPi
 *
 *     < ... skipped ... >
 *
 *
 */

#if defined(RASPBERRY_PI)

#include <stdio.h>

#include "SoCHelper.h"
#include "NMEAHelper.h"
#include "EPDHelper.h"
#include "TrafficHelper.h"
#include "EEPROMHelper.h"
#include "WiFiHelper.h"
#include "GDL90Helper.h"
#include "BatteryHelper.h"

#include "SkyView.h"

TTYSerial SerialInput("/dev/ttyUSB0");

static const uint8_t SS    = 8; // pin 24

/* Waveshare Pi HAT 2.7" */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> epd_waveshare(GxEPD2_270(/*CS=5*/ SS,
                                       /*DC=*/ 25, /*RST=*/ 17, /*BUSY=*/ 24));

lmic_pinmap lmic_pins = {
    .nss = LMIC_UNUSED_PIN,
    .rxtx = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN },
    .rst = LMIC_UNUSED_PIN,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void radio_irq_handler (u1_t dio) {
}
u1_t radio_has_irq (void) {
    return 0;
}

eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;

char UDPpacketBuffer[UDP_PACKET_BUFSIZE]; // buffer to hold incoming packets

hardware_info_t hw_info = {
  .model    = SOFTRF_MODEL_RASPBERRY,
  .revision = 0,
  .soc      = SOC_NONE,
  .display  = DISPLAY_NONE
};

static void RPi_setup()
{

  eeprom_block.field.settings.adapter         = ADAPTER_WAVESHARE_PI_HAT_2_7;

  eeprom_block.field.settings.connection      = CON_SERIAL;
  eeprom_block.field.settings.baudrate        = B38400;
  eeprom_block.field.settings.protocol        = PROTOCOL_NMEA;
  eeprom_block.field.settings.map_orientation = DIRECTION_NORTH_UP;

  strcpy(eeprom_block.field.settings.ssid,      DEFAULT_AP_SSID);
  strcpy(eeprom_block.field.settings.psk,       DEFAULT_AP_PSK);

  eeprom_block.field.settings.bluetooth       = BLUETOOTH_OFF;

  strcpy(eeprom_block.field.settings.bt_name,   DEFAULT_BT_NAME);
  strcpy(eeprom_block.field.settings.bt_key,    DEFAULT_BT_KEY);
}

static uint32_t RPi_getChipId()
{
  return gethostid();
}

static void RPi_swSer_begin(unsigned long baud)
{
  SerialInput.begin(baud);
}

static void RPi_Battery_setup()
{
  /* TBD */
}

static float RPi_Battery_voltage()
{
  return 0.0;  /* TBD */
}

static void RPi_EPD_setup()
{
  display = &epd_waveshare;
}

static size_t RPi_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  return 0; /* TBD */
}

const SoC_ops_t RPi_ops = {
  SOC_RPi,
  "RPi",
  RPi_setup,
  RPi_getChipId,
  NULL,
  NULL,
  NULL,
  RPi_swSer_begin,
  NULL,
  NULL,
  NULL,
  RPi_Battery_setup,
  RPi_Battery_voltage,
  RPi_EPD_setup,
  RPi_WiFi_Receive_UDP
};

int main()
{
  // Init GPIO bcm
  if (!bcm2835_init()) {
      fprintf( stderr, "bcm2835_init() Failed\n\n" );
      exit(EXIT_FAILURE);
  }

  Serial.begin(38400);

  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  Battery_setup();

  Serial.print(F("Intializing E-ink display module (may take up to 10 seconds)... "));
  Serial.flush();
  hw_info.display = EPD_setup();
  if (hw_info.display != DISPLAY_NONE) {
    Serial.println(F(" done."));
  } else {
    Serial.println(F(" failed!"));
  }

  switch (settings->protocol)
  {
  case PROTOCOL_GDL90:
    GDL90_setup();
    break;
  case PROTOCOL_NMEA:
  default:
    NMEA_setup();
    break;
  }

  while (true) {

    switch (settings->protocol)
    {
    case PROTOCOL_GDL90:
      GDL90_loop();
      break;
    case PROTOCOL_NMEA:
    default:
      NMEA_loop();
      break;
    }

    EPD_loop();
    Traffic_ClearExpired();
  }

  return 0;
}

#endif /* RASPBERRY_PI */
