/*
 * Multi(.ino) firmware
 * Copyright (C) 2020 Linar Yusupov
 *
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 *
 * Web: http://github.com/lyusupov/SoftRF
 *
 * Credits:
 *   flarm_decode is developed by Stanislaw Pusep, http://github.com/creaktive
 *   Arduino Time Library is developed by Paul Stoffregen, http://github.com/PaulStoffregen
 *   "Aircraft" and MAVLink Libraries are developed by Andy Little
 *   TinyGPS++ and PString Libraries are developed by Mikal Hart
 *   TrueRandom Library is developed by Peter Knight
 *   Lib_crc is developed by Lammert Bies
 *   OGN library is developed by Pawel Jalocha
 *   NMEA library is developed by Timur Sinitsyn, Tobias Simon, Ferry Huberts
 *   ADS-B encoder C++ library is developed by yangbinbin (yangbinbin_ytu@163.com)
 *   Adafruit BMP085 library is developed by Limor Fried and Ladyada
 *   Adafruit BMP280 library is developed by Kevin Townsend
 *   Adafruit MPL3115A2 library is developed by Limor Fried and Kevin Townsend
 *   EGM96 data is developed by XCSoar team
 *   Dump978 library is developed by Oliver Jowett
 *   FEC library is developed by Phil Karn
 *   Basic MAC library is developed by Michael Kuyper
 *   port of Basic MAC library for Arduino is developed by Matthijs Kooijman
 *   Arduino core for ASR650x is developed by Aaron Lee (HelTec Automation)
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

#include "TimeHelper.h"
#include "LEDHelper.h"
#include "GNSSHelper.h"
#include "RFHelper.h"
#include "SoundHelper.h"
#include "EEPROMHelper.h"
#include "GDL90Helper.h"
#include "NMEAHelper.h"
#include "D1090Helper.h"
#include "BaroHelper.h"
#include "SoCHelper.h"
#include "TrafficHelper.h"
#include "MAVLinkHelper.h"
#include "BatteryHelper.h"
#include "JSONHelper.h"

#include "SoftRF.h"

#define isValidFix()      isValidGNSSFix()
#define isTimeToExport()  (millis() - ExportTimeMarker > 1000)

ufo_t ThisAircraft;

hardware_info_t hw_info = {
  .model    = SOFTRF_MODEL_RETRO,
  .revision = 0,
  .soc      = SOC_NONE,
  .rf       = RF_IC_NONE,
  .gnss     = GNSS_MODULE_NONE,
  .baro     = BARO_MODULE_NONE,
  .display  = DISPLAY_NONE
};

unsigned long LEDTimeMarker    = 0;
unsigned long ExportTimeMarker = 0;

void setup() {

  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  Serial.begin(SERIAL_OUT_BR);

  Serial.println();
  Serial.print(F(SOFTRF_IDENT));
  Serial.print(SoC->name);
  Serial.print(F(" FW.REV: " SOFTRF_FIRMWARE_VERSION
#if defined(SOFTRF_ADDRESS)
    "I"
#endif
    " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));
  Serial.println(F("Copyright (C) 2015-2020 Linar Yusupov. All rights reserved."));
  Serial.flush();

  Serial.println();
  Serial.print(F("Reset reason: "));
  Serial.println(SoC->getResetReason());

  EEPROM_setup();

  ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;

  hw_info.rf = RF_setup();

  delay(100);

#if !(defined(EXCLUDE_BMP180) && defined(EXCLUDE_BMP280) && defined(EXCLUDE_MPL3115A2))
  hw_info.baro = Baro_setup();
#endif

#if !defined(EXCLUDE_MAVLINK)
  if (settings->mode == SOFTRF_MODE_UAV) {
    Serial.begin(57600);
    MAVLink_setup();
    ThisAircraft.aircraft_type = AIRCRAFT_TYPE_UAV;  
  } else 
#else
  settings->mode = SOFTRF_MODE_NORMAL;
#endif /* EXCLUDE_MAVLINK */
  {
    hw_info.gnss = GNSS_setup();
    ThisAircraft.aircraft_type = settings->aircraft_type;
  }
  ThisAircraft.protocol = settings->rf_protocol;
  ThisAircraft.stealth  = settings->stealth;
  ThisAircraft.no_track = settings->no_track;

  Battery_setup();
  Traffic_setup();

  SoC->swSer_enableRx(false);

  NMEA_setup();

  SoC->swSer_enableRx(true);

  SoC->WDT_setup();
}

void loop() {

  // Do common RF stuff first
  RF_loop();

  switch (settings->mode)
  {
#if 0
  case SOFTRF_MODE_TXRX_TEST:
    txrx_test_loop();
    break;
#endif
#if !defined(EXCLUDE_MAVLINK)
  case SOFTRF_MODE_UAV:
    uav_loop();
    break;
#endif /* EXCLUDE_MAVLINK */
  case SOFTRF_MODE_NORMAL:
  default:
    normal_loop();
    break;
  }

  // Show status info on tiny OLED display
  SoC->Display_loop();

  SoC->loop();

  Battery_loop();

  yield();
}

void shutdown(const char *msg)
{
  SoC->WDT_fini();

  SoC->swSer_enableRx(false);

  NMEA_fini();

  if (settings->mode != SOFTRF_MODE_UAV) {
    GNSS_fini();
  }

  SoC->Display_fini(msg);

  RF_Shutdown();

  SoC_fini();
}

void normal_loop()
{
  bool success;

#if !(defined(EXCLUDE_BMP180) && defined(EXCLUDE_BMP280) && defined(EXCLUDE_MPL3115A2))
  Baro_loop();
#endif

  PickGNSSFix();

  GNSSTimeSync();

  ThisAircraft.timestamp = now();
  if (isValidFix()) {
    ThisAircraft.latitude = gnss.location.lat();
    ThisAircraft.longitude = gnss.location.lng();
    ThisAircraft.altitude = gnss.altitude.meters();
    ThisAircraft.course = gnss.course.deg();
    ThisAircraft.speed = gnss.speed.knots();
    ThisAircraft.hdop = (uint16_t) gnss.hdop.value();
    ThisAircraft.geoid_separation = gnss.separation.meters();

#if !defined(EXCLUDE_EGM96)
    /*
     * When geoidal separation is zero or not available - use approx. EGM96 value
     */
    if (ThisAircraft.geoid_separation == 0.0) {
      ThisAircraft.geoid_separation = (float) LookupSeparation(
                                                ThisAircraft.latitude,
                                                ThisAircraft.longitude
                                              );
      /* we can assume the GPS unit is giving ellipsoid height */
      ThisAircraft.altitude -= ThisAircraft.geoid_separation;
    }
#endif /* EXCLUDE_EGM96 */

    RF_Transmit(RF_Encode(&ThisAircraft), true);
  }

  success = RF_Receive();

#if DEBUG
  success = true;
#endif

  if (success && isValidFix()) ParseData();

  if (isValidFix()) {
    Traffic_loop();
  }

  if (isTimeToExport()) {
    NMEA_Export();

    if (isValidFix()) {
      GDL90_Export();
      D1090_Export();
    }
    ExportTimeMarker = millis();
  }

  NMEA_loop();

  ClearExpired();
}

#if 0
unsigned int pos_ndx = 0;
unsigned long TxPosUpdMarker = 0;

void txrx_test_loop()
{
  bool success = false;
#if DEBUG_TIMING
  unsigned long baro_start_ms, baro_end_ms;
  unsigned long tx_start_ms, tx_end_ms, rx_start_ms, rx_end_ms;
  unsigned long parse_start_ms, parse_end_ms;
  unsigned long export_start_ms, export_end_ms;
  unsigned long oled_start_ms, oled_end_ms;
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
  ThisAircraft.vs = TXRX_TEST_VS;

#if DEBUG_TIMING
  baro_start_ms = millis();
#endif
  Baro_loop();
#if DEBUG_TIMING
  baro_end_ms = millis();
#endif

#if defined(ENABLE_AHRS)
  AHRS_loop();
#endif /* ENABLE_AHRS */

#if DEBUG_TIMING
  tx_start_ms = millis();
#endif
  RF_Transmit(RF_Encode(&ThisAircraft), true);
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

#if defined(ENABLE_TTN)
  TTN_loop();
#endif

  Traffic_loop();

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
  oled_start_ms = millis();
#endif
//  SoC->Display_loop();
#if DEBUG_TIMING
  oled_end_ms = millis();
#endif

#if DEBUG_TIMING
  if (baro_start_ms - baro_end_ms) {
    Serial.print(F("Baro start: "));
    Serial.print(baro_start_ms);
    Serial.print(F(" Baro stop: "));
    Serial.println(baro_end_ms);
  }
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
  if (export_end_ms - export_start_ms) {
    Serial.print(F("Export start: "));
    Serial.print(export_start_ms);
    Serial.print(F(" Export stop: "));
    Serial.println(export_end_ms);
  }
  if (oled_end_ms - oled_start_ms) {
    Serial.print(F("OLED start: "));
    Serial.print(oled_start_ms);
    Serial.print(F(" OLED stop: "));
    Serial.println(oled_end_ms);
  }
#endif

  // Handle Air Connect
  NMEA_loop();

  ClearExpired();
}
#endif

#if !defined(EXCLUDE_MAVLINK)
void uav_loop()
{
  bool success = false;

  PickMAVLinkFix();

  MAVLinkTimeSync();
  MAVLinkSetWiFiPower();

  ThisAircraft.timestamp = now();

  if (isValidMAVFix()) {
    ThisAircraft.latitude = the_aircraft.location.gps_lat / 1e7;
    ThisAircraft.longitude = the_aircraft.location.gps_lon / 1e7;
    ThisAircraft.altitude = the_aircraft.location.gps_alt / 1000.0;
    ThisAircraft.course = the_aircraft.location.gps_cog;
    ThisAircraft.speed = (the_aircraft.location.gps_vog / 100.0) / _GPS_MPS_PER_KNOT;
    ThisAircraft.pressure_altitude = the_aircraft.location.baro_alt;
    ThisAircraft.hdop = the_aircraft.location.gps_hdop;

    RF_Transmit(RF_Encode(&ThisAircraft), true);
  }

  success = RF_Receive();

  if (success && isValidMAVFix()) ParseData();

  if (isTimeToExport() && isValidMAVFix()) {
    MAVLinkShareTraffic();
    ExportTimeMarker = millis();
  }

  ClearExpired();
}
#endif /* EXCLUDE_MAVLINK */