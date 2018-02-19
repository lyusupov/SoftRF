/*
 * SoftRF(.ino) firmware
 * Copyright (C) 2016-2018 Linar Yusupov
 *
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 *
 * Web: http://github.com/lyusupov/SoftRF
 *
 * Arduino core for ESP8266 is developed/supported by ESP8266 Community (support-esp8266@esp8266.com)
 * AVR/Arduino nRF905 Library/Driver is developed by Zak Kemble, contact@zakkemble.co.uk
 * flarm_decode is developed by Stanislaw Pusep, http://github.com/creaktive
 * Arduino Time Library is developed by Paul Stoffregen, http://github.com/PaulStoffregen
 * "Aircraft" and MAVLink Libraries are developed by Andy Little
 * TinyGPS++ and PString Libraries are developed by Mikal Hart
 * Adafruit NeoPixel Library is developed by Phil Burgess, Michael Miller and others
 * TrueRandom Library is developed by Peter Knight
 * IBM LMIC framework is maintained by Matthijs Kooijman
 * ESP8266FtpServer is developed by David Paiva
 * Lib_crc is developed by Lammert Bies
 * OGN library is developed by Pawel Jalocha
 * NMEA library is developed by Timur Sinitsyn, Tobias Simon, Ferry Huberts
 * ADS-B encoder C++ library is developed by yangbinbin (yangbinbin_ytu@163.com)
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

   NodeMCU 1.0 GPIO pins:
   2 -> CE
   4 -> PWR
   16 -> TXE
   0 -> CD
   5 -> DR
   15 -> CSN
   12 -> SO
   13 -> SI
   14 -> SCK
*/

#include "OTAHelper.h"
#include "TimeHelper.h"
#include "LEDHelper.h"
#include "GNSSHelper.h"
#include "RFHelper.h"
#include "SoundHelper.h"
#include "EEPROMHelper.h"
#include "BatteryHelper.h"
#include "MAVLinkHelper.h"
#include "GDL90Helper.h"
#include "NMEAHelper.h"
#include "D1090Helper.h"
#include "SoCHelper.h"
#include "WiFiHelper.h"
#include "WebHelper.h"
#include "BaroHelper.h"

#include "SoftRF.h"

#if LOGGER_IS_ENABLED
#include "LogHelper.h"
#endif /* LOGGER_IS_ENABLED */

#define DEBUG 0
#define DEBUG_TIMING 0

#define isTimeToDisplay() (millis() - LEDTimeMarker > 1000)
#define isTimeToExport() (millis() - ExportTimeMarker > 1000)
#define isValidFix() (gnss.location.isValid() && (gnss.location.age() <= 3000))

ufo_t ThisAircraft;

unsigned long LEDTimeMarker = 0;
unsigned long ExportTimeMarker = 0;

void setup()
{
  rst_info *resetInfo;

  SoC_setup(); // Has to be very first procedure in the execution order

  resetInfo = (rst_info *) SoC->getResetInfoPtr();

  Serial.begin(38400);  
  //Misc_info();

#if LOGGER_IS_ENABLED
  Logger_setup();
#endif /* LOGGER_IS_ENABLED */

  Serial.println(""); Serial.print(F("Reset reason: ")); Serial.println(resetInfo->reason);
  Serial.println(SoC->getResetReason());
  Serial.print(F("Free heap size: ")); Serial.println(ESP.getFreeHeap());
  Serial.println(SoC->getResetInfo()); Serial.println("");

  EEPROM_setup();
  Battery_setup();

  ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;

  RF_setup();
  delay(100);

  if (rf_chip && (rf_chip->type == RF_IC_SX1276)) {
    Baro_setup();
  }

  if (settings->mode == SOFTRF_MODE_UAV_BEACON) {
    Serial.begin(57600);
    MAVLink_setup();
    ThisAircraft.aircraft_type = AIRCRAFT_TYPE_UAV;  
  }  else {
    GNSS_setup();
    ThisAircraft.aircraft_type = settings->aircraft_type;
  }
  ThisAircraft.protocol = settings->rf_protocol;

  SoC->swSer_enableRx(false);

  LED_setup();

  WiFi_setup();
  OTA_setup();
  Web_setup();
  NMEA_setup();

  delay(1000);

  /* expedite restart on WDT reset */
  if (resetInfo->reason != REASON_WDT_RST) {
    LED_test();
  }

  SoC->Sound_test(resetInfo->reason);

  switch (settings->mode)
  {
  case SOFTRF_MODE_TXRX_TEST:
    Time_setup();
    break;
  case SOFTRF_MODE_BRIDGE:
    break;
  case SOFTRF_MODE_NORMAL:
  case SOFTRF_MODE_UAV_BEACON:
  default:
    SoC->swSer_enableRx(true);
    break;
  }
}

void loop()
{
  // Do common RF stuff first
  RF_loop();

  switch (settings->mode)
  {
  case SOFTRF_MODE_TXRX_TEST:
    txrx_test_loop();
    break;
  case SOFTRF_MODE_UAV_BEACON:
    uav_loop();
    break;
  case SOFTRF_MODE_BRIDGE:
    bridge_loop();
    break;
  case SOFTRF_MODE_NORMAL:
  default:
    normal_loop();
    break;
  }

  // Handle Air Connect
  NMEA_loop();

  // Handle DNS
  WiFi_loop();

  // Handle Web
  Web_loop();

  // Handle OTA update.
  OTA_loop();

#if LOGGER_IS_ENABLED
  Logger_loop();
#endif /* LOGGER_IS_ENABLED */

  delay(0);
}

void normal_loop()
{
  bool success;

  PickGNSSFix();

  GNSSTimeSync();

  ThisAircraft.timestamp = now();
  if (isValidFix()) {
    ThisAircraft.latitude = gnss.location.lat();
    ThisAircraft.longitude = gnss.location.lng();
    ThisAircraft.altitude = gnss.altitude.meters();
    ThisAircraft.course = gnss.course.deg();
    ThisAircraft.speed = gnss.speed.knots() ;

    RF_Transmit(RF_Encode());
  }

  success = RF_Receive();

#if DEBUG
  success = true;
#endif

  if (success && isValidFix()) ParseData();

  if (isTimeToDisplay()) {
    if (isValidFix()) {
      LED_DisplayTraffic();
    } else {
      LED_Clear();
    }
    LEDTimeMarker = millis();
  }

  if (isTimeToExport() && isValidFix()) {
    NMEA_Export();
    GDL90_Export();
    D1090_Export();
    ExportTimeMarker = millis();
  }

  ClearExpired();

}

#define MAVisValidFix() (the_aircraft.gps.fix_type == 3 /* 3D fix */ )

void uav_loop()
{
  bool success = false;

  PickMAVLinkFix();

  MAVLinkTimeSync();
  MAVLinkSetWiFiPower();

  ThisAircraft.timestamp = now();

  if (MAVisValidFix()) {
    ThisAircraft.latitude = the_aircraft.location.gps_lat / 1e7;
    ThisAircraft.longitude = the_aircraft.location.gps_lon / 1e7;
    ThisAircraft.altitude = the_aircraft.location.gps_alt / 1000.0;
    ThisAircraft.course = the_aircraft.location.gps_cog;
    ThisAircraft.speed = (the_aircraft.location.gps_vog / 100.0) / _GPS_MPS_PER_KNOT ;

    RF_Transmit(RF_Encode());
  }

  success = RF_Receive();

  if (success && MAVisValidFix()) ParseData();

  if (isTimeToExport() && MAVisValidFix()) {
    MAVLinkShareTraffic();
    ExportTimeMarker = millis();
  }

  ClearExpired();
}

unsigned int pos_ndx = 0;
unsigned long TxPosUpdMarker = 0;

void txrx_test_loop()
{
  bool success = false;
#if DEBUG_TIMING
  unsigned long tx_start_ms, tx_end_ms, rx_start_ms, rx_end_ms;
  unsigned long parse_start_ms, parse_end_ms, led_start_ms, led_end_ms;
  unsigned long export_start_ms, export_end_ms;
#endif
  ThisAircraft.timestamp = now();

  if (TxPosUpdMarker == 0 || (millis() - TxPosUpdMarker) > 4000 ) {
    ThisAircraft.latitude =  pgm_read_float( &txrx_test_positions[pos_ndx][0]);
    ThisAircraft.longitude =  pgm_read_float( &txrx_test_positions[pos_ndx][1]);
    pos_ndx = (pos_ndx + 1) % TXRX_TEST_NUM_POSITIONS;
    TxPosUpdMarker = millis();
  }
  ThisAircraft.altitude = TXRX_TEST_ALTITUDE;
  ThisAircraft.course = TXRX_TEST_COURSE;
  ThisAircraft.speed = TXRX_TEST_SPEED;
#if DEBUG_TIMING
  tx_start_ms = millis();
#endif
  RF_Transmit(RF_Encode());
#if DEBUG_TIMING
  tx_end_ms = millis();
  rx_start_ms = millis();
#endif
  success = RF_Receive();
#if DEBUG_TIMING
  rx_end_ms = millis();
#endif

#if DEBUG_TIMING
  parse_start_ms = millis();
#endif
  if (success) ParseData();
#if DEBUG_TIMING
  parse_end_ms = millis();
#endif

#if DEBUG_TIMING
  led_start_ms = millis();
#endif
  if (isTimeToDisplay()) {
    LED_DisplayTraffic();
    LEDTimeMarker = millis();
  }
#if DEBUG_TIMING
  led_end_ms = millis();
#endif

#if DEBUG_TIMING
  export_start_ms = millis();
#endif
  if (isTimeToExport()) {
    NMEA_Position();
    NMEA_Export();
    GDL90_Export();
    D1090_Export();
    ExportTimeMarker = millis();
  }
#if DEBUG_TIMING
  export_end_ms = millis();
#endif

#if DEBUG_TIMING
  if (tx_end_ms - tx_start_ms) {
    Serial.print(F("TX start: "));
    Serial.print(tx_start_ms);
    Serial.print(F(" TX stop: "));
    Serial.println(tx_end_ms);
  }
  if (rx_end_ms - rx_start_ms) {
    Serial.print(F("RX start: "));
    Serial.print(rx_start_ms);
    Serial.print(F(" RX stop: "));
    Serial.println(rx_end_ms);
  }
  if (parse_end_ms - parse_start_ms) {
    Serial.print(F("Parse start: "));
    Serial.print(parse_start_ms);
    Serial.print(F(" Parse stop: "));
    Serial.println(parse_end_ms);
  }
  if (led_end_ms - led_start_ms) {
    Serial.print(F("LED start: "));
    Serial.print(led_start_ms);
    Serial.print(F(" LED stop: "));
    Serial.println(led_end_ms);
  }
  if (export_end_ms - export_start_ms) {
    Serial.print(F("Export start: "));
    Serial.print(export_start_ms);
    Serial.print(F(" Export stop: "));
    Serial.println(export_end_ms);
  }
#endif

  ClearExpired();
}

void bridge_loop()
{
  bool success;

  size_t tx_size = Raw_Receive_UDP(&TxPkt[0]);

  if (tx_size > 0) {
    RF_Transmit(tx_size);
  }

  success = RF_Receive();

  if(success)
  {

    fo.raw = Bin2Hex(RxBuffer);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,")); StdOut.print(now()); StdOut.print(F(",")); StdOut.println(fo.raw);
    }

    Raw_Transmit_UDP();
  }

  if (isTimeToDisplay()) {
    LED_Clear();  
    LEDTimeMarker = millis();
  }
}
