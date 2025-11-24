/*
 * WebHelper.cpp
 * Copyright (C) 2016-2025 Linar Yusupov
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

#include "../system/SoC.h"

//#include <Arduino.h>

#include "../driver/Battery.h"
#include "../driver/RF.h"

#include "Web.h"
#include "../driver/Baro.h"

#include "../driver/LED.h"
#include "../driver/Sound.h"
#include "../driver/Bluetooth.h"
#include "../TrafficHelper.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
#if !defined(EXCLUDE_MAVLINK)
#include "../protocol/data/MAVLink.h"
#endif /* EXCLUDE_MAVLINK */
#include "../system/Time.h"

#if defined(ENABLE_AHRS)
#include "../driver/AHRS.h"
#endif /* ENABLE_AHRS */

#if defined(ENABLE_RECORDER)
#include "../system/Recorder.h"
#endif /* ENABLE_RECORDER */

const char about_html[] PROGMEM = "<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>About</title>\
  </head>\
<body>\
<h1 align=center>About</h1>\
<p>This firmware is a part of SoftRF project</p>\
<p>URL: http://github.com/lyusupov/SoftRF</p>\
<p>Author: <b>Linar Yusupov</b></p>\
<p>E-mail: linar.r.yusupov@gmail.com</p>\
<h2 align=center>Credits</h2>\
<p align=center>(in historical order)</p>\
<table width=100%%>\
<tr><th align=left>Ivan Grokhotkov</th><td align=left>Arduino Core for ESP8266</td></tr>\
<tr><th align=left>Zak Kemble</th><td align=left>nRF905 library</td></tr>\
<tr><th align=left>Stanislaw Pusep</th><td align=left>flarm_decode</td></tr>\
<tr><th align=left>Paul Stoffregen</th><td align=left>Arduino Time Library</td></tr>\
<tr><th align=left>Mikal Hart</th><td align=left>TinyGPS++ and PString Libraries</td></tr>\
<tr><th align=left>Phil Burgess</th><td align=left>Adafruit NeoPixel Library</td></tr>\
<tr><th align=left>Andy Little</th><td align=left>Aircraft and MAVLink Libraries</td></tr>\
<tr><th align=left>Peter Knight</th><td align=left>TrueRandom Library</td></tr>\
<tr><th align=left>Matthijs Kooijman</th><td align=left>IBM LMIC and Semtech Basic MAC frameworks for Arduino</td></tr>\
<tr><th align=left>David Paiva</th><td align=left>ESP8266FtpServer</td></tr>\
<tr><th align=left>Lammert Bies</th><td align=left>Lib_crc</td></tr>\
<tr><th align=left>Pawel Jalocha</th><td align=left>OGN/ADS-L library</td></tr>\
<tr><th align=left>Timur Sinitsyn, Tobias Simon, Ferry Huberts</th><td align=left>NMEA library</td></tr>\
<tr><th align=left>yangbinbin (yangbinbin_ytu@163.com)</th><td align=left>ADS-B encoder C++ library</td></tr>\
<tr><th align=left>Hristo Gochkov</th><td align=left>Arduino Core for ESP32</td></tr>\
<tr><th align=left>Evandro Copercini</th><td align=left>ESP32 BT SPP library</td></tr>\
<tr><th align=left>Limor Fried and Ladyada</th><td align=left>Adafruit BMP085 library</td></tr>\
<tr><th align=left>Kevin Townsend</th><td align=left>Adafruit BMP280 library</td></tr>\
<tr><th align=left>Limor Fried and Kevin Townsend</th><td align=left>Adafruit MPL3115A2 library</td></tr>\
<tr><th align=left>Oliver Kraus</th><td align=left>U8g2 LCD, OLED and eInk library</td></tr>\
<tr><th align=left>Michael Miller</th><td align=left>NeoPixelBus library</td></tr>\
<tr><th align=left>Shenzhen Xin Yuan (LilyGO) ET company</th><td align=left>TTGO T-Beam, T-Watch, T-TWR and T3-C6</td></tr>\
<tr><th align=left>JS Foundation</th><td align=left>jQuery library</td></tr>\
<tr><th align=left>XCSoar team</th><td align=left>EGM96 data</td></tr>\
<tr><th align=left>Mike McCauley</th><td align=left>BCM2835 C library</td></tr>\
<tr><th align=left>Dario Longobardi</th><td align=left>SimpleNetwork library</td></tr>\
<tr><th align=left>Benoit Blanchon</th><td align=left>ArduinoJson library</td></tr>\
<tr><th align=left>flashrom.org project</th><td align=left>Flashrom library</td></tr>\
<tr><th align=left>Robert Wessels and Tony Cave</th><td align=left>EasyLink library</td></tr>\
<tr><th align=left>Oliver Jowett</th><td align=left>Dump978 library</td></tr>\
<tr><th align=left>Phil Karn</th><td align=left>FEC library</td></tr>\
<tr><th align=left>Lewis He</th><td align=left>AXP20X, XPowersLib and SensorsLib libraries</td></tr>\
<tr><th align=left>Bodmer</th><td align=left>TFT library</td></tr>\
<tr><th align=left>Michael Kuyper</th><td align=left>Basic MAC library</td></tr>\
<tr><th align=left>Earle Philhower</th><td align=left>Arduino Core for RP2XXX and ESP8266Audio library</td></tr>\
<tr><th align=left>Steve Marple</th><td align=left>IniFile library</td></tr>\
<tr><th align=left>Vasilis Georgitzikis</th><td align=left>MD5 library</td></tr>\
<tr><th align=left>Evan Krall and Somkiat Nakhonthai</th><td align=left>LibAPRS-ESP32 library</td></tr>\
<tr><th align=left>Gibbon Zen</th><td align=left>SA818 library</td></tr>\
<tr><th align=left>Steve Jack</th><td align=left>OpenDroneID library</td></tr>\
<tr><th align=left>Martino Facchin</th><td align=left>Arduino Core for Renesas fsp</td></tr>\
<tr><th align=left>Khoi Hoang</th><td align=left>WiFiWebServer and Functional-Vlpp libraries</td></tr>\
</table>\
<hr>\
Copyright (C) 2015-2025 &nbsp;&nbsp;&nbsp; Linar Yusupov\
</body>\
</html>";

char *Root_content() {

  float vdd = Battery_voltage() ;
  bool low_voltage = (Battery_voltage() <= Battery_threshold());

  unsigned int timestamp = (unsigned int) ThisAircraft.timestamp;
  unsigned int sats;

#if !defined(EXCLUDE_MAVLINK)
  if (settings->mode == SOFTRF_MODE_UAV)
    sats = the_aircraft.gps.num_sats;
  else
#endif /* EXCLUDE_MAVLINK */
    sats = gnss.satellites.value(); // Number of satellites in use (u32)

  char str_lat[16];
  char str_lon[16];
  char str_alt[16];
  char str_Vcc[8];

  size_t size = 2420;
  char *offset;
  size_t len = 0;

  char *Root_temp = (char *) malloc(size);
  if (Root_temp == NULL) {
    return Root_temp;
  }
  offset = Root_temp;

  dtostrf(ThisAircraft.latitude,  8, 4, str_lat);
  dtostrf(ThisAircraft.longitude, 8, 4, str_lon);
  dtostrf(ThisAircraft.altitude,  7, 1, str_alt);
  dtostrf(vdd, 4, 2, str_Vcc);

  snprintf_P ( offset, size,
    PSTR("<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>SoftRF status</title>\
  </head>\
<body>\
 <table width=100%%>\
  <tr><!-- <td align=left><h1>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</h1></td> -->\
  <td align=center><h1>SoftRF status</h1></td>\
  <!-- <td align=right><img src='/logo.png'></td> --></tr>\
 </table>\
 <table width=100%%>\
  <tr><th align=left>Device Id</th><td align=right>%06X</td></tr>\
  <tr><th align=left>Software Version</th><td align=right>%s&nbsp;&nbsp;%s</td></tr>"
#if !defined(ENABLE_AHRS)
 "</table><table width=100%%>\
  <tr><td align=left><table><tr><th align=left>GNSS&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td>\
  <td align=center><table><tr><th align=left>Radio&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td>\
  <td align=right><table><tr><th align=left>Baro&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td></tr>\
  </table><table width=100%%>"
#else
 "<tr><td align=left><table><tr><th align=left>GNSS&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td>\
  <td align=right><table><tr><th align=left>Radio&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td></tr>\
  <tr><td align=left><table><tr><th align=left>Baro&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td>\
  <td align=right><table><tr><th align=left>AHRS&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td></tr>"
#endif /* ENABLE_AHRS */
 "<tr><th align=left>Uptime</th><td align=right>%02d:%02d:%02d</td></tr>"
#if !defined(RASPBERRY_PI) && !defined(LUCKFOX_LYRA)
 "<tr><th align=left>Free memory</th><td align=right>%u</td></tr>"
#endif /* RASPBERRY_PI */
 "<tr><th align=left>Battery voltage</th><td align=right><font color=%s>%s</font></td></tr>"
#if defined(USE_USB_HOST) && defined(CONFIG_IDF_TARGET_ESP32S3)
  "<tr><th align=left>USB client</th><td align=right>%s %s</td></tr>"
#endif /* USE_USB_HOST */
 "</table>\
 <table width=100%%>\
   <tr><th align=left>Packets</th>\
    <td align=right><table><tr>\
     <th align=left>Tx&nbsp;&nbsp;</th><td align=right>%u</td>\
     <th align=left>&nbsp;&nbsp;&nbsp;&nbsp;Rx&nbsp;&nbsp;</th><td align=right>%u</td>\
   </tr></table></td></tr>\
 </table>\
 <h2 align=center>Most recent GNSS fix</h2>\
 <table width=100%%>\
  <tr><th align=left>Time</th><td align=right>%u</td></tr>\
  <tr><th align=left>Satellites</th><td align=right>%d</td></tr>\
  <tr><th align=left>Latitude</th><td align=right>%s</td></tr>\
  <tr><th align=left>Longitude</th><td align=right>%s</td></tr>\
  <tr><td align=left><b>Altitude</b>&nbsp;&nbsp;(above MSL)</td><td align=right>%s</td></tr>\
 </table>\
 <hr>\
 <table width=100%%>\
  <tr>\
    <td align=left><input type=button onClick=\"location.href='/settings'\" value='Settings'></td>\
    <td align=center><input type=button onClick=\"location.href='/about'\" value='About'></td>"),
    ThisAircraft.addr, SOFTRF_FIRMWARE_VERSION
#if defined(USE_USB_HOST) && defined(CONFIG_IDF_TARGET_ESP32S3)
    "H"
#endif /* USE_USB_HOST */
#if defined(SOFTRF_ADDRESS)
    "I"
#endif /* SOFTRF_ADDRESS */
    ,
    (SoC == NULL ? "NONE" : SoC->name),
    GNSS_name[hw_info.gnss],
    (hw_info.rf == RF_IC_R820T || hw_info.rf == RF_IC_MAX2837 ||
     hw_info.rf == RF_IC_MSI001 ? "SDR" :
     rf_chip   == NULL ? "NONE" : rf_chip->name),
    (baro_chip == NULL ? "NONE" : baro_chip->name),
#if defined(ENABLE_AHRS)
    (ahrs_chip == NULL ? "NONE" : ahrs_chip->name),
#endif /* ENABLE_AHRS */
    UpTime.hours, UpTime.minutes, UpTime.seconds,
#if !defined(RASPBERRY_PI) && !defined(LUCKFOX_LYRA)
    SoC->getFreeHeap(),
#endif /* RASPBERRY_PI */
    low_voltage ? "red" : "green", str_Vcc,
#if defined(USE_USB_HOST) && defined(CONFIG_IDF_TARGET_ESP32S3)
    ESP32_USB_Serial.connected ? supported_USB_devices[ESP32_USB_Serial.index].first_name : "",
    ESP32_USB_Serial.connected ? supported_USB_devices[ESP32_USB_Serial.index].last_name  : "N/A",
#endif /* USE_USB_HOST */
    tx_packets_counter, rx_packets_counter,
    timestamp, sats, str_lat, str_lon, str_alt
  );

  len = strlen(offset);
  offset += len;
  size -= len;

#if defined(ENABLE_RECORDER)
  if (FR_is_active) {
    snprintf_P ( offset, size,
    PSTR("<td align=center>&nbsp;&nbsp;&nbsp;<input type=button onClick=\"location.href='/flights'\" value='Flights'></td>"));
    len = strlen(offset);
    offset += len;
    size -= len;
  }
#endif /* ENABLE_RECORDER */

  /* SoC specific part 1 */
  if (SoC->id != SOC_RP2040      && SoC->id != SOC_RP2350_ARM &&
      SoC->id != SOC_RP2350_RISC && SoC->id != SOC_RA4M1      &&
      SoC->id != SOC_RPi         && SoC->id != SOC_RK3506) {
    snprintf_P ( offset, size, PSTR("\
    <td align=right><input type=button onClick=\"location.href='/firmware'\" value='Firmware update'></td>"));
    len = strlen(offset);
    offset += len;
    size -= len;
  }

  snprintf_P ( offset, size, PSTR("\
  </tr>\
 </table>\
</body>\
</html>")
  );

  return Root_temp;
}

char *Settings_content() {

  size_t size = 5520;
  char *offset;
  size_t len = 0;
  char *Settings_temp = (char *) malloc(size);

  if (Settings_temp == NULL) {
    return Settings_temp;
  }

  offset = Settings_temp;

  /* Common part 1 */
  snprintf_P ( offset, size,
    PSTR("<html>\
<head>\
<meta name='viewport' content='width=device-width, initial-scale=1'>\
<title>Settings</title>\
</head>\
<body>\
<h1 align=center>Settings</h1>\
<form action='/input' method='GET'>\
<table width=100%%>\
<tr>\
<th align=left>Mode</th>\
<td align=right>\
<select name='mode'>\
<option %s value='%d'>Normal</option>\
<!--<option %s value='%d'>Tx/Rx Test</option>-->\
<option %s value='%d'>Bridge</option>"
#if defined(EXCLUDE_MAVLINK)
"<!--<option %s value='%d'>UAV</option>-->"
#else
"<option %s value='%d'>UAV</option>"
#endif /* EXCLUDE_MAVLINK */
"</select>\
</td>\
</tr>"),
  (settings->mode == SOFTRF_MODE_NORMAL    ? "selected" : ""), SOFTRF_MODE_NORMAL,
  (settings->mode == SOFTRF_MODE_TXRX_TEST ? "selected" : ""), SOFTRF_MODE_TXRX_TEST,
  (settings->mode == SOFTRF_MODE_BRIDGE    ? "selected" : ""), SOFTRF_MODE_BRIDGE,
  (settings->mode == SOFTRF_MODE_UAV       ? "selected" : ""), SOFTRF_MODE_UAV
/*  (settings->mode == SOFTRF_MODE_WATCHOUT ? "selected" : ""), SOFTRF_MODE_WATCHOUT, */
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  /* Radio specific part 1 */
  if (hw_info.rf == RF_IC_SX1276 ||
      hw_info.rf == RF_IC_SX1262 ||
      hw_info.rf == RF_IC_LR1110 ||
      hw_info.rf == RF_IC_LR1121 ||
      hw_info.rf == RF_IC_SX1280) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Protocol</th>\
<td align=right>\
<select name='protocol'>\
<option %s value='%d'>%s</option>\
<option %s value='%d'>%s</option>\
<option %s value='%d'>%s</option>\
<option %s value='%d'>%s</option>"
#if defined(ENABLE_ADSL)
"<option %s value='%d'>%s</option>"
#else
"<!--<option %s value='%d'>%s</option>-->"
#endif /* ENABLE_ADSL */
#if defined(ENABLE_PROL)
"<option %s value='%d'>%s</option>"
#else
"<!--<option %s value='%d'>%s</option>-->"
#endif /* ENABLE_PROL */
"</select>\
</td>\
</tr>"),
    (settings->rf_protocol == RF_PROTOCOL_LEGACY   ? "selected" : ""),
     RF_PROTOCOL_LEGACY, legacy_proto_desc.name,
    (settings->rf_protocol == RF_PROTOCOL_OGNTP    ? "selected" : ""),
     RF_PROTOCOL_OGNTP, ogntp_proto_desc.name,
    (settings->rf_protocol == RF_PROTOCOL_P3I      ? "selected" : ""),
     RF_PROTOCOL_P3I, p3i_proto_desc.name,
    (settings->rf_protocol == RF_PROTOCOL_FANET    ? "selected" : ""),
     RF_PROTOCOL_FANET, fanet_proto_desc.name,
    (settings->rf_protocol == RF_PROTOCOL_ADSL_860 ? "selected" : ""),
     RF_PROTOCOL_ADSL_860, adsl_proto_desc.name,
    (settings->rf_protocol == RF_PROTOCOL_APRS     ? "selected" : ""),
     RF_PROTOCOL_APRS, prol_proto_desc.name
    );
  } else if (hw_info.rf == RF_IC_SX1231 ||
             hw_info.rf == RF_IC_SI4432 ||
             hw_info.rf == RF_IC_SI4463 ||
             hw_info.rf == RF_IC_CC1101) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Protocol</th>\
<td align=right>\
<select name='protocol'>\
<option %s value='%d'>%s</option>\
<option %s value='%d'>%s</option>\
<option %s value='%d'>%s</option>"
#if defined(ENABLE_ADSL)
"<option %s value='%d'>%s</option>"
#else
"<!--<option %s value='%d'>%s</option>-->"
#endif /* ENABLE_ADSL */
"</select>\
</td>\
</tr>"),
    (settings->rf_protocol == RF_PROTOCOL_LEGACY   ? "selected" : ""),
     RF_PROTOCOL_LEGACY, legacy_proto_desc.name,
    (settings->rf_protocol == RF_PROTOCOL_OGNTP    ? "selected" : ""),
     RF_PROTOCOL_OGNTP, ogntp_proto_desc.name,
    (settings->rf_protocol == RF_PROTOCOL_P3I      ? "selected" : ""),
     RF_PROTOCOL_P3I, p3i_proto_desc.name,
    (settings->rf_protocol == RF_PROTOCOL_ADSL_860 ? "selected" : ""),
     RF_PROTOCOL_ADSL_860, adsl_proto_desc.name
    );
  } else if (hw_info.rf == RF_IC_R820T   ||
             hw_info.rf == RF_IC_MAX2837 ||
             hw_info.rf == RF_IC_MSI001) {
    snprintf_P ( offset, size,
      PSTR("<tr><th align=left>Protocol</th><td align=right>%s</td></tr>"),
      es1090_proto_desc.name);
  } else {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Protocol</th>\
<td align=right>%s\
</td>\
</tr>"),
    (settings->rf_protocol == RF_PROTOCOL_LEGACY   ? legacy_proto_desc.name :
    (settings->rf_protocol == RF_PROTOCOL_ADSB_UAT ? uat978_proto_desc.name :
    (settings->rf_protocol == RF_PROTOCOL_FANET    ? fanet_proto_desc.name  :
    (settings->rf_protocol == RF_PROTOCOL_ADSL_860 ? adsl_proto_desc.name   :
    (settings->rf_protocol == RF_PROTOCOL_APRS     ? aprs_proto_desc.name   :
     "UNK")))))
    );
  }
  len = strlen(offset);
  offset += len;
  size -= len;

  /* Common part 2 */
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
<th align=left>Region</th>\
<td align=right>\
<select name='band'>\
<!--<option %s value='%d'>AUTO</option>-->\
<option %s value='%d'>EU (868.2 MHz)</option>\
<option %s value='%d'>RU (868.8 MHz)</option>\
<option %s value='%d'>CN (470 MHz)</option>\
<option %s value='%d'>US/CA (915 MHz)</option>\
<option %s value='%d'>NZ (869.25 MHz)</option>\
<!--<option %s value='%d'>UK (869.52 MHz)</option>-->\
<option %s value='%d'>AU (921 MHz)</option>\
<option %s value='%d'>IN (866 MHz)</option>\
<option %s value='%d'>KR (920.9 MHz)</option>\
<option %s value='%d'>IL (916.2 MHz)</option>\
<!--<option %s value='%d'>Reserved</option>-->\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Aircraft type</th>\
<td align=right>\
<select name='acft_type'>\
<option %s value='%d'>Glider</option>\
<option %s value='%d'>Towplane</option>\
<option %s value='%d'>Powered</option>\
<option %s value='%d'>Helicopter</option>\
<option %s value='%d'>UAV</option>\
<option %s value='%d'>Hangglider</option>\
<option %s value='%d'>Paraglider</option>\
<option %s value='%d'>Balloon</option>\
<option %s value='%d'>Static</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Alarm trigger</th>\
<td align=right>\
<select name='alarm'>\
<option %s value='%d'>None</option>\
<option %s value='%d'>Distance</option>\
<option %s value='%d'>Vector</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Tx Power</th>\
<td align=right>\
<select name='txpower'>\
<option %s value='%d'>Full</option>\
<option %s value='%d'>Low</option>\
<option %s value='%d'>Off</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Volume</th>\
<td align=right>\
<select name='volume'>\
<option %s value='%d'>Loud</option>\
<option %s value='%d'>Low</option>\
<option %s value='%d'>Off</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>LED ring direction</th>\
<td align=right>\
<select name='pointer'>\
<option %s value='%d'>CoG Up</option>\
<option %s value='%d'>North Up</option>\
<option %s value='%d'>Off</option>\
</select>\
</td>\
</tr>"),
  (settings->band == RF_BAND_AUTO ? "selected" : ""), RF_BAND_AUTO,
  (settings->band == RF_BAND_EU   ? "selected" : ""), RF_BAND_EU,
  (settings->band == RF_BAND_RU   ? "selected" : ""), RF_BAND_RU,
  (settings->band == RF_BAND_CN   ? "selected" : ""), RF_BAND_CN,
  (settings->band == RF_BAND_US   ? "selected" : ""), RF_BAND_US,
  (settings->band == RF_BAND_NZ   ? "selected" : ""), RF_BAND_NZ,
  (settings->band == RF_BAND_UK   ? "selected" : ""), RF_BAND_UK,
  (settings->band == RF_BAND_AU   ? "selected" : ""), RF_BAND_AU,
  (settings->band == RF_BAND_IN   ? "selected" : ""), RF_BAND_IN,
  (settings->band == RF_BAND_KR   ? "selected" : ""), RF_BAND_KR,
  (settings->band == RF_BAND_IL   ? "selected" : ""), RF_BAND_IL,
  (settings->band == RF_BAND_RSVD ? "selected" : ""), RF_BAND_RSVD,
  (settings->aircraft_type == AIRCRAFT_TYPE_GLIDER ? "selected" : ""),  AIRCRAFT_TYPE_GLIDER,
  (settings->aircraft_type == AIRCRAFT_TYPE_TOWPLANE ? "selected" : ""),  AIRCRAFT_TYPE_TOWPLANE,
  (settings->aircraft_type == AIRCRAFT_TYPE_POWERED ? "selected" : ""),  AIRCRAFT_TYPE_POWERED,
  (settings->aircraft_type == AIRCRAFT_TYPE_HELICOPTER ? "selected" : ""),  AIRCRAFT_TYPE_HELICOPTER,
  (settings->aircraft_type == AIRCRAFT_TYPE_UAV ? "selected" : ""),  AIRCRAFT_TYPE_UAV,
  (settings->aircraft_type == AIRCRAFT_TYPE_HANGGLIDER ? "selected" : ""),  AIRCRAFT_TYPE_HANGGLIDER,
  (settings->aircraft_type == AIRCRAFT_TYPE_PARAGLIDER ? "selected" : ""),  AIRCRAFT_TYPE_PARAGLIDER,
  (settings->aircraft_type == AIRCRAFT_TYPE_BALLOON ? "selected" : ""),  AIRCRAFT_TYPE_BALLOON,
  (settings->aircraft_type == AIRCRAFT_TYPE_STATIC ? "selected" : ""),  AIRCRAFT_TYPE_STATIC,
  (settings->alarm == TRAFFIC_ALARM_NONE ? "selected" : ""),  TRAFFIC_ALARM_NONE,
  (settings->alarm == TRAFFIC_ALARM_DISTANCE ? "selected" : ""),  TRAFFIC_ALARM_DISTANCE,
  (settings->alarm == TRAFFIC_ALARM_VECTOR ? "selected" : ""),  TRAFFIC_ALARM_VECTOR,
  (settings->txpower == RF_TX_POWER_FULL ? "selected" : ""),  RF_TX_POWER_FULL,
  (settings->txpower == RF_TX_POWER_LOW ? "selected" : ""),  RF_TX_POWER_LOW,
  (settings->txpower == RF_TX_POWER_OFF ? "selected" : ""),  RF_TX_POWER_OFF,
  (settings->volume == BUZZER_VOLUME_FULL ? "selected" : ""), BUZZER_VOLUME_FULL,
  (settings->volume == BUZZER_VOLUME_LOW ? "selected" : ""), BUZZER_VOLUME_LOW,
  (settings->volume == BUZZER_OFF ? "selected" : ""), BUZZER_OFF,
  (settings->pointer == DIRECTION_TRACK_UP ? "selected" : ""), DIRECTION_TRACK_UP,
  (settings->pointer == DIRECTION_NORTH_UP ? "selected" : ""), DIRECTION_NORTH_UP,
  (settings->pointer == LED_OFF ? "selected" : ""), LED_OFF
  );

  len = strlen(offset);
  offset += len;
  size -= len;

#if !defined(EXCLUDE_BLUETOOTH)
  /* SoC specific part 1 */
  if (SoC->id == SOC_ESP32      || SoC->id == SOC_RP2040 ||
      SoC->id == SOC_RP2350_ARM || SoC->id == SOC_RP2350_RISC) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Built-in Bluetooth</th>\
<td align=right>\
<select name='bluetooth'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>SPP</option>\
<option %s value='%d'>LE</option>\
<!--<option %s value='%d'>A2DP</option>-->\
</select>\
</td>\
</tr>"),
    (settings->bluetooth == BLUETOOTH_NONE ? "selected" : ""), BLUETOOTH_NONE,
    (settings->bluetooth == BLUETOOTH_SPP  ? "selected" : ""), BLUETOOTH_SPP,
    (settings->bluetooth == BLUETOOTH_LE_HM10_SERIAL ? "selected" : ""), BLUETOOTH_LE_HM10_SERIAL,
    (settings->bluetooth == BLUETOOTH_A2DP_SOURCE    ? "selected" : ""), BLUETOOTH_A2DP_SOURCE
    );

    len = strlen(offset);
    offset += len;
    size -= len;

  } else if (SoC->id == SOC_ESP32S3 || SoC->id == SOC_ESP32C2 ||
             SoC->id == SOC_ESP32C3 || SoC->id == SOC_ESP32C5 ||
             SoC->id == SOC_ESP32C6 || SoC->id == SOC_ESP32P4 ||
             SoC->id == SOC_RA4M1) {

    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Built-in Bluetooth</th>\
<td align=right>\
<select name='bluetooth'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>LE</option>\
</select>\
</td>\
</tr>"),
    (settings->bluetooth == BLUETOOTH_NONE           ? "selected" : ""), BLUETOOTH_NONE,
    (settings->bluetooth == BLUETOOTH_LE_HM10_SERIAL ? "selected" : ""), BLUETOOTH_LE_HM10_SERIAL
    );

    len = strlen(offset);
    offset += len;
    size -= len;
  }
#endif /* EXCLUDE_BLUETOOTH */

  /* Common part 3 */
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
<th align=left>NMEA sentences:</th>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;GNSS</th>\
<td align=right>\
<input type='radio' name='nmea_g' value='0' %s>Off\
<input type='radio' name='nmea_g' value='1' %s>On\
</td>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Private</th>\
<td align=right>\
<input type='radio' name='nmea_p' value='0' %s>Off\
<input type='radio' name='nmea_p' value='1' %s>On\
</td>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Legacy</th>\
<td align=right>\
<input type='radio' name='nmea_l' value='0' %s>Off\
<input type='radio' name='nmea_l' value='1' %s>On\
</td>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Sensors</th>\
<td align=right>\
<input type='radio' name='nmea_s' value='0' %s>Off\
<input type='radio' name='nmea_s' value='1' %s>On\
</td>\
</tr>\
<tr>\
<th align=left>NMEA output</th>\
<td align=right>\
<select name='nmea_out'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>Serial</option>\
<option %s value='%d'>UDP</option>"),
  (!settings->nmea_g ? "checked" : "") , (settings->nmea_g ? "checked" : ""),
  (!settings->nmea_p ? "checked" : "") , (settings->nmea_p ? "checked" : ""),
  (!settings->nmea_l ? "checked" : "") , (settings->nmea_l ? "checked" : ""),
  (!settings->nmea_s ? "checked" : "") , (settings->nmea_s ? "checked" : ""),
  (settings->nmea_out == NMEA_OFF  ? "selected" : ""), NMEA_OFF,
  (settings->nmea_out == NMEA_UART ? "selected" : ""), NMEA_UART,
  (settings->nmea_out == NMEA_UDP  ? "selected" : ""), NMEA_UDP);

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 2 */
  if (SoC->id == SOC_ESP32      || SoC->id == SOC_ESP32S3     ||
      SoC->id == SOC_ESP32C2    || SoC->id == SOC_ESP32C3     ||
      SoC->id == SOC_ESP32C5    || SoC->id == SOC_ESP32C6     ||
      SoC->id == SOC_ESP32P4    || SoC->id == SOC_RP2040      ||
      SoC->id == SOC_RP2350_ARM || SoC->id == SOC_RP2350_RISC ||
      SoC->id == SOC_RA4M1) {
    snprintf_P ( offset, size,
      PSTR(
#if defined(NMEA_TCP_SERVICE)
"<option %s value='%d'>TCP</option>"
#else
"<!--<option %s value='%d'>TCP</option>-->"
#endif /* NMEA_TCP_SERVICE */
"<option %s value='%d'>Bluetooth</option>"),
      (settings->nmea_out == NMEA_TCP       ? "selected" : ""), NMEA_TCP,
      (settings->nmea_out == NMEA_BLUETOOTH ? "selected" : ""), NMEA_BLUETOOTH);

    len = strlen(offset);
    offset += len;
    size -= len;
  }
  if (SoC->id == SOC_ESP32S2 || SoC->id == SOC_ESP32S3    ||
      SoC->id == SOC_ESP32C3 || SoC->id == SOC_ESP32C5    ||
      SoC->id == SOC_ESP32C6 || SoC->id == SOC_ESP32P4    ||
      SoC->id == SOC_RP2040  || SoC->id == SOC_RP2350_ARM ||
      SoC->id == SOC_RP2350_RISC) {
    snprintf_P ( offset, size,
      PSTR("<option %s value='%d'>USB</option>"),
      (settings->nmea_out == NMEA_USB       ? "selected" : ""), NMEA_USB);

    len = strlen(offset);
    offset += len;
    size -= len;
  }

  /* Common part 4 */
  snprintf_P ( offset, size,
    PSTR("\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>GDL90</th>\
<td align=right>\
<select name='gdl90'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>Serial</option>\
<option %s value='%d'>UDP</option>"),
  (settings->gdl90 == GDL90_OFF  ? "selected" : ""), GDL90_OFF,
  (settings->gdl90 == GDL90_UART ? "selected" : ""), GDL90_UART,
  (settings->gdl90 == GDL90_UDP  ? "selected" : ""), GDL90_UDP);

  len = strlen(offset);
  offset += len;
  size -= len;

#if !defined(EXCLUDE_BLUETOOTH)
  /* SoC specific part 3 */
  if (SoC->id == SOC_ESP32      || SoC->id == SOC_ESP32S3     ||
      SoC->id == SOC_ESP32C2    || SoC->id == SOC_ESP32C3     ||
      SoC->id == SOC_ESP32C5    || SoC->id == SOC_ESP32C6     ||
      SoC->id == SOC_ESP32P4    || SoC->id == SOC_RP2040      ||
      SoC->id == SOC_RP2350_ARM || SoC->id == SOC_RP2350_RISC ||
      SoC->id == SOC_RA4M1) {
    snprintf_P ( offset, size,
      PSTR("<option %s value='%d'>Bluetooth</option>"),
      (settings->gdl90 == GDL90_BLUETOOTH ? "selected" : ""), GDL90_BLUETOOTH);

    len = strlen(offset);
    offset += len;
    size -= len;
  }
#endif /* EXCLUDE_BLUETOOTH */

  if (SoC->id == SOC_ESP32S2 || SoC->id == SOC_ESP32S3    ||
      SoC->id == SOC_ESP32C3 || SoC->id == SOC_ESP32C5    ||
      SoC->id == SOC_ESP32C6 || SoC->id == SOC_ESP32P4    ||
      SoC->id == SOC_RP2040  || SoC->id == SOC_RP2350_ARM ||
      SoC->id == SOC_RP2350_RISC) {
    snprintf_P ( offset, size,
      PSTR("<option %s value='%d'>USB</option>"),
      (settings->gdl90 == GDL90_USB       ? "selected" : ""), GDL90_USB);

    len = strlen(offset);
    offset += len;
    size -= len;
  }

  /* Common part 5 */
  snprintf_P ( offset, size,
    PSTR("\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Dump1090</th>\
<td align=right>\
<select name='d1090'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>Serial</option>"),
  (settings->d1090 == D1090_OFF  ? "selected" : ""), D1090_OFF,
  (settings->d1090 == D1090_UART ? "selected" : ""), D1090_UART);

  len = strlen(offset);
  offset += len;
  size -= len;

#if !defined(EXCLUDE_BLUETOOTH)
  /* SoC specific part 4 */
  if (SoC->id == SOC_ESP32      || SoC->id == SOC_ESP32S3     ||
      SoC->id == SOC_ESP32C2    || SoC->id == SOC_ESP32C3     ||
      SoC->id == SOC_ESP32C5    || SoC->id == SOC_ESP32C6     ||
      SoC->id == SOC_ESP32P4    || SoC->id == SOC_RP2040      ||
      SoC->id == SOC_RP2350_ARM || SoC->id == SOC_RP2350_RISC ||
      SoC->id == SOC_RA4M1) {
    snprintf_P ( offset, size,
      PSTR("<option %s value='%d'>Bluetooth</option>"),
      (settings->d1090 == D1090_BLUETOOTH ? "selected" : ""), D1090_BLUETOOTH);

    len = strlen(offset);
    offset += len;
    size -= len;
  }
#endif /* EXCLUDE_BLUETOOTH */

  if (SoC->id == SOC_ESP32S2 || SoC->id == SOC_ESP32S3    ||
      SoC->id == SOC_ESP32C3 || SoC->id == SOC_ESP32C5    ||
      SoC->id == SOC_ESP32C6 || SoC->id == SOC_ESP32P4    ||
      SoC->id == SOC_RP2040  || SoC->id == SOC_RP2350_ARM ||
      SoC->id == SOC_RP2350_RISC) {
    snprintf_P ( offset, size,
      PSTR("<option %s value='%d'>USB</option>"),
      (settings->d1090 == D1090_USB       ? "selected" : ""), D1090_USB);

    len = strlen(offset);
    offset += len;
    size -= len;
  }

  /* Common part 6 */
  snprintf_P ( offset, size,
    PSTR("\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Power save</th>\
<td align=right>\
<select name='power_save'>\
<option %s value='%d'>Disabled</option>\
<option %s value='%d'>WiFi OFF (10 min.)</option>\
<option %s value='%d'>GNSS</option>"),
  (settings->power_save == POWER_SAVE_NONE ? "selected" : ""), POWER_SAVE_NONE,
  (settings->power_save == POWER_SAVE_WIFI ? "selected" : ""), POWER_SAVE_WIFI,
  (settings->power_save == POWER_SAVE_GNSS ? "selected" : ""), POWER_SAVE_GNSS
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  /* Radio specific part 2 */
  if (rf_chip && rf_chip->type == RF_IC_SA8X8) {
    snprintf_P(offset, size, PSTR("<option %s value='%d'>No receive</option>"),
    (settings->power_save == POWER_SAVE_NORECEIVE ? "selected" : ""), POWER_SAVE_NORECEIVE);

    len = strlen(offset);
    offset += len;
    size -= len;
  }

  /* Common part 7 */
  snprintf_P ( offset, size,
    PSTR("\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Stealth</th>\
<td align=right>\
<input type='radio' name='stealth' value='0' %s>Off\
<input type='radio' name='stealth' value='1' %s>On\
</td>\
</tr>\
<tr>\
<th align=left>No track</th>\
<td align=right>\
<input type='radio' name='no_track' value='0' %s>Off\
<input type='radio' name='no_track' value='1' %s>On\
</td>\
</tr>"),
  (!settings->stealth  ? "checked" : "") , (settings->stealth  ? "checked" : ""),
  (!settings->no_track ? "checked" : "") , (settings->no_track ? "checked" : "")
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  /* Radio specific part 3 */
  if (rf_chip && (rf_chip->type == RF_IC_SX1276 ||
                  rf_chip->type == RF_IC_CC1101 ||
                  rf_chip->type == RF_IC_SX1231 ||
                  rf_chip->type == RF_IC_SI4432 ||
                  rf_chip->type == RF_IC_SI4463 ||
     (rf_chip->type == RF_IC_LR1121 && hw_info.model == SOFTRF_MODEL_NANO) ||
                  rf_chip->type == RF_IC_SX1280)) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Radio CF correction (&#177;, kHz)</th>\
<td align=right>\
<INPUT type='number' name='rfc' min='%d' max='%d' value='%d'>\
</td>\
</tr>"),
    rf_chip->type == RF_IC_SX1280 ? -90 : -30,
    rf_chip->type == RF_IC_SX1280 ?  90 :  30,
    settings->freq_corr);

    len = strlen(offset);
    offset += len;
    size -= len;
  }

#if defined(USE_OGN_ENCRYPTION)
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
<th align=left>IGC key</th>\
<td align=right>\
<INPUT type='text' name='igc_key' maxlength='32' size='32' value='%08X%08X%08X%08X'>\
</td>\
</tr>"),
  settings->igc_key[0], settings->igc_key[1], settings->igc_key[2], settings->igc_key[3]);

  len = strlen(offset);
  offset += len;
  size -= len;
#endif

  /* Common part 8 */
  snprintf_P ( offset, size,
    PSTR("\
</table>\
<p align=center><INPUT type='submit' value='Save and restart'></p>\
</form>\
</body>\
</html>")
  );

  return Settings_temp;
}

char *Input_content() {
  size_t size = 1700;

  char *Input_temp = (char *) malloc(size);
  if (Input_temp == NULL) {
    return Input_temp;
  }

  snprintf_P ( Input_temp, size,
PSTR("<html>\
<head>\
<meta http-equiv='refresh' content='15; url=/'>\
<meta name='viewport' content='width=device-width, initial-scale=1'>\
<title>SoftRF Settings</title>\
</head>\
<body>\
<h1 align=center>New settings:</h1>\
<table width=100%%>\
<tr><th align=left>Mode</th><td align=right>%d</td></tr>\
<tr><th align=left>Protocol</th><td align=right>%d</td></tr>\
<tr><th align=left>Band</th><td align=right>%d</td></tr>\
<tr><th align=left>Aircraft type</th><td align=right>%d</td></tr>\
<tr><th align=left>Alarm trigger</th><td align=right>%d</td></tr>\
<tr><th align=left>Tx Power</th><td align=right>%d</td></tr>\
<tr><th align=left>Volume</th><td align=right>%d</td></tr>\
<tr><th align=left>LED pointer</th><td align=right>%d</td></tr>\
<tr><th align=left>Bluetooth</th><td align=right>%d</td></tr>\
<tr><th align=left>NMEA GNSS</th><td align=right>%s</td></tr>\
<tr><th align=left>NMEA Private</th><td align=right>%s</td></tr>\
<tr><th align=left>NMEA Legacy</th><td align=right>%s</td></tr>\
<tr><th align=left>NMEA Sensors</th><td align=right>%s</td></tr>\
<tr><th align=left>NMEA Out</th><td align=right>%d</td></tr>\
<tr><th align=left>GDL90</th><td align=right>%d</td></tr>\
<tr><th align=left>DUMP1090</th><td align=right>%d</td></tr>\
<tr><th align=left>Stealth</th><td align=right>%s</td></tr>\
<tr><th align=left>No track</th><td align=right>%s</td></tr>\
<tr><th align=left>Power save</th><td align=right>%d</td></tr>\
<tr><th align=left>Freq. correction</th><td align=right>%d</td></tr>\
<tr><th align=left>IGC key</th><td align=right>%08X%08X%08X%08X</td></tr>\
</table>\
<hr>\
  <p align=center><h1 align=center>Restart is in progress... Please, wait!</h1></p>\
</body>\
</html>"),
  settings->mode, settings->rf_protocol, settings->band,
  settings->aircraft_type, settings->alarm, settings->txpower,
  settings->volume, settings->pointer, settings->bluetooth,
  BOOL_STR(settings->nmea_g), BOOL_STR(settings->nmea_p),
  BOOL_STR(settings->nmea_l), BOOL_STR(settings->nmea_s),
  settings->nmea_out, settings->gdl90, settings->d1090,
  BOOL_STR(settings->stealth), BOOL_STR(settings->no_track),
  settings->power_save, settings->freq_corr,
  settings->igc_key[0], settings->igc_key[1], settings->igc_key[2], settings->igc_key[3]
  );

  return Input_temp;
}

#if defined(EXCLUDE_WEBUI) || \
   (defined(EXCLUDE_WIFI)  && defined(EXCLUDE_ETHERNET))
void Web_setup()    {}
void Web_loop()     {}
void Web_fini()     {}
#else

static uint32_t prev_rx_pkt_cnt = 0;

#if !defined(ARDUINO_ARCH_RENESAS)
#include "../Logo.h"
#endif /* ARDUINO_ARCH_RENESAS */

#if !defined(EXCLUDE_OTA)
#include "jquery_min_js.h"
#endif /* EXCLUDE_OTA */

#if defined(ESPHOSTSPI)
#include <WebServer.h>
WebServer server ( 80 );
#else
#include <WiFiWebServer.h>
WiFiWebServer server ( 80 );
#endif

byte getVal(char c)
{
   if(c >= '0' && c <= '9')
     return (byte)(c - '0');
   else
     return (byte)(toupper(c)-'A'+10);
}

#if DEBUG
void Hex2Bin(String str, byte *buffer)
{
  char hexdata[2 * PKT_SIZE + 1];

  str.toCharArray(hexdata, sizeof(hexdata));
  for(int j = 0; j < PKT_SIZE * 2 ; j+=2)
  {
    buffer[j>>1] = getVal(hexdata[j+1]) + (getVal(hexdata[j]) << 4);
  }
}
#endif

void handleSettings() {
  char *settings = Settings_content();

  if (settings) {
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
    server.sendHeader(String(F("Pragma")), String(F("no-cache")));
    server.sendHeader(String(F("Expires")), String(F("-1")));
#if !defined(USE_ARDUINO_WIFI) || defined(USING_WIFI101_GENERIC)
    server.send ( 200, "text/html", settings );
#else

#define HTML_MAX_CHUNK_SIZE (JS_MAX_CHUNK_SIZE / 2)

    char *content = settings;
    size_t bytes_left = strlen(settings);
    size_t chunk_size;

    server.setContentLength(bytes_left);
    server.send(200, String(F("text/html")), "");

    do {
      chunk_size = bytes_left > HTML_MAX_CHUNK_SIZE ?
                   HTML_MAX_CHUNK_SIZE : bytes_left;
      server.sendContent(content, chunk_size);
      content += chunk_size;
      bytes_left -= chunk_size;
    } while (bytes_left > 0) ;
#endif /* USE_ARDUINO_WIFI */
    SoC->swSer_enableRx(true);
    free(settings);
  }
}

void handleRoot() {
  char *root = Root_content();

  if (root) {
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
    server.sendHeader(String(F("Pragma")), String(F("no-cache")));
    server.sendHeader(String(F("Expires")), String(F("-1")));
    server.send ( 200, "text/html", root );
    SoC->swSer_enableRx(true);
    free(root);
  }
}

void handleInput() {

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    if (server.argName(i).equals("mode")) {
      settings->mode = server.arg(i).toInt();
    } else if (server.argName(i).equals("protocol")) {
      settings->rf_protocol = server.arg(i).toInt();
    } else if (server.argName(i).equals("band")) {
      settings->band = server.arg(i).toInt();
    } else if (server.argName(i).equals("acft_type")) {
      settings->aircraft_type = server.arg(i).toInt();
    } else if (server.argName(i).equals("alarm")) {
      settings->alarm = server.arg(i).toInt();
    } else if (server.argName(i).equals("txpower")) {
      settings->txpower = server.arg(i).toInt();
    } else if (server.argName(i).equals("volume")) {
      settings->volume = server.arg(i).toInt();
    } else if (server.argName(i).equals("pointer")) {
      settings->pointer = server.arg(i).toInt();
    } else if (server.argName(i).equals("bluetooth")) {
      settings->bluetooth = server.arg(i).toInt();
    } else if (server.argName(i).equals("nmea_g")) {
      settings->nmea_g = server.arg(i).toInt();
    } else if (server.argName(i).equals("nmea_p")) {
      settings->nmea_p = server.arg(i).toInt();
    } else if (server.argName(i).equals("nmea_l")) {
      settings->nmea_l = server.arg(i).toInt();
    } else if (server.argName(i).equals("nmea_s")) {
      settings->nmea_s = server.arg(i).toInt();
    } else if (server.argName(i).equals("nmea_out")) {
      settings->nmea_out = server.arg(i).toInt();
    } else if (server.argName(i).equals("gdl90")) {
      settings->gdl90 = server.arg(i).toInt();
    } else if (server.argName(i).equals("d1090")) {
      settings->d1090 = server.arg(i).toInt();
    } else if (server.argName(i).equals("stealth")) {
      settings->stealth = server.arg(i).toInt();
    } else if (server.argName(i).equals("no_track")) {
      settings->no_track = server.arg(i).toInt();
    } else if (server.argName(i).equals("power_save")) {
      settings->power_save = server.arg(i).toInt();
    } else if (server.argName(i).equals("rfc")) {
      settings->freq_corr = server.arg(i).toInt();
#if defined(USE_OGN_ENCRYPTION)
    } else if (server.argName(i).equals("igc_key")) {
      char buf[32 + 1];
      server.arg(i).toCharArray(buf, sizeof(buf));
      settings->igc_key[3] = strtoul(buf + 24, NULL, 16);
      buf[24] = 0;
      settings->igc_key[2] = strtoul(buf + 16, NULL, 16);
      buf[16] = 0;
      settings->igc_key[1] = strtoul(buf +  8, NULL, 16);
      buf[ 8] = 0;
      settings->igc_key[0] = strtoul(buf +  0, NULL, 16);
#endif
    }
  }

  char *input = Input_content();

  if (input) {
    SoC->swSer_enableRx(false);
    server.send ( 200, "text/html", input );
//  SoC->swSer_enableRx(true);
    delay(1000);
    free(input);
    EEPROM_store();
    Sound_fini();
    RF_Shutdown();
    delay(1000);
    SoC->reset();
  }
}

#if defined(ENABLE_RECORDER)

#include <SdFat_Adafruit_Fork.h>
extern SdFat uSD;

#define FILESYSTEM       uSD
#define FLIGHTS_DIR      "/Flights"
#define MAX_IGC_FILE_NUM 200

typedef struct
{
  String filename;
  String fsize;
  uint16_t date;
  uint16_t time;
  uint16_t s_ndx; /* reserved */
} fileinfo;

String   webpage;
fileinfo Filenames[MAX_IGC_FILE_NUM];
static int numfiles;

String ConvBinUnits(int bytes, int resolution) {
  if      (bytes < 1024)                 {
    return String(bytes) + " B";
  }
  else if (bytes < 1024 * 1024)          {
    return String((bytes / 1024.0), resolution) + " KB";
  }
  else if (bytes < (1024 * 1024 * 1024)) {
    return String((bytes / 1024.0 / 1024.0), resolution) + " MB";
  }
  else return "";
}

void Flights() {
  char buf[48];
  String filename;
  numfiles = 0;

  File32 root = FILESYSTEM.open(FLIGHTS_DIR);
  if (root) {
    root.rewindDirectory();
    File32 file = root.openNextFile();
    while (file) {
      if (!file.isDirectory()) {
        file.getName(buf, sizeof(buf));
        filename = String(buf);
        if (filename.endsWith(".igc") || filename.endsWith(".IGC")) {
          Filenames[numfiles].filename = (filename.startsWith("/") ? filename.substring(1) : filename);
          Filenames[numfiles].fsize    = ConvBinUnits(file.size(), 1);
          file.getModifyDateTime(&Filenames[numfiles].date, &Filenames[numfiles].time);
          numfiles++;

          if (numfiles >= MAX_IGC_FILE_NUM) break;
        }
      }
      file = root.openNextFile();
    }
    root.close();
  }
}

void Handle_Flight_Download() {
  String filename;
  int index = 0;

  Flights();

  webpage  = "<html>";
  webpage += "<head>";
  webpage += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  webpage += "<title>Flights</title>";
  webpage += "</head>";
  webpage += "<body>";

  if (numfiles > 0) {
    webpage += "<table width=100%%><tr>";
    webpage += "<td align=center><h2>Select a flight to download</h2></td>";
    webpage += "</tr></table>";
    webpage += "<table width=100%%>";
    webpage += "<tr><th align=left>File Name</th><th align=right>Size</th></tr>";
    webpage += "<tr><td><hr></td><td><hr></td></tr>";

    index = numfiles - 1;
    while (index >= 0) {
      filename = Filenames[index].filename;
      webpage += "<tr><td align=left><a href='" FLIGHTS_DIR "/" + filename +
                 "'>" + filename + "</a><td align=right>" +
                 Filenames[index].fsize + "</td></tr>";
      index--;
    }
    webpage += "<tr><td><hr></td><td><hr></td></tr>";
    if (numfiles >= MAX_IGC_FILE_NUM) {
      webpage += "<tr><td align=left>view is limited to " +
                 String(MAX_IGC_FILE_NUM) + " files only</td></tr>";
    } else {
      webpage += "<tr><td align=left>" + String(numfiles) + " file(s) total</td></tr>";
    }
    webpage += "</table>";
  } else {
    webpage += "<h2>No flights found</h2>";
  }

  webpage += "</body>";
  webpage += "</html>";

  server.send(200, "text/html", webpage);
}

String getContentType(String filename) {
  if (server.hasArg("download")) {
    return "application/octet-stream";
  } else if (filename.endsWith(".htm")) {
    return "text/html";
  } else if (filename.endsWith(".html")) {
    return "text/html";
  } else if (filename.endsWith(".css")) {
    return "text/css";
  } else if (filename.endsWith(".js")) {
    return "application/javascript";
  } else if (filename.endsWith(".png")) {
    return "image/png";
  } else if (filename.endsWith(".gif")) {
    return "image/gif";
  } else if (filename.endsWith(".jpg")) {
    return "image/jpeg";
  } else if (filename.endsWith(".ico")) {
    return "image/x-icon";
  } else if (filename.endsWith(".xml")) {
    return "text/xml";
  } else if (filename.endsWith(".pdf")) {
    return "application/x-pdf";
  } else if (filename.endsWith(".zip")) {
    return "application/x-zip";
  } else if (filename.endsWith(".gz")) {
    return "application/x-gzip";
  } else if (filename.endsWith(".igc") || filename.endsWith(".IGC")) {
    return "application/octet-stream";
  }
  return "text/plain";
}

bool handleFileRead(String path) {
//  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) {
    path += "index.htm";
  }
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if (FILESYSTEM.exists(pathWithGz) || FILESYSTEM.exists(path)) {
    char buf[1024];

    if (FILESYSTEM.exists(pathWithGz)) {
      path += ".gz";
    }
    File32 file = FILESYSTEM.open(path, O_RDONLY);
    size_t size = file.size();

    SoC->WDT_fini();

    server.setContentLength(size);
    server.send(200, contentType, "");
    while (size > 0) {
      size_t nread = file.readBytes(buf, sizeof(buf));
      server.client().write(buf, nread);
      size -= nread;
    }

    file.close();

    SoC->WDT_setup();

    return true;
  }
  return false;
}
#endif /* ENABLE_RECORDER */

void handleNotFound() {
#if defined(ENABLE_RECORDER)
  if (!handleFileRead(server.uri()))
#endif /* ENABLE_RECORDER */
  {
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";

    for ( uint8_t i = 0; i < server.args(); i++ ) {
      message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
    }

    server.send ( 404, "text/plain", message );
  }
}

void Web_setup()
{
  server.on ( "/", handleRoot );
  server.on ( "/settings", handleSettings );
  server.on ( "/about", []() {
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
    server.sendHeader(String(F("Pragma")), String(F("no-cache")));
    server.sendHeader(String(F("Expires")), String(F("-1")));
    server.send_P ( 200, PSTR("text/html"), about_html);
    SoC->swSer_enableRx(true);
  } );

  server.on ( "/input", handleInput );
  server.on ( "/inline", []() {
    server.send ( 200, "text/plain", "this works as well" );
  } );

  server.onNotFound ( handleNotFound );

#if !defined(EXCLUDE_OTA)
  server.on("/firmware", HTTP_GET, [](){
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Connection")), String(F("close")));
    server.sendHeader(String(F("Access-Control-Allow-Origin")), String(F("*")));
    server.send_P(200,
      PSTR("text/html"),
      PSTR("\
<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>Firmware update</title>\
  </head>\
<body>\
<body>\
 <h1 align=center>Firmware update</h1>\
 <hr>\
 <table width=100%%>\
  <tr>\
    <td align=left>\
<script src='/jquery.min.js'></script>\
<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>\
    <input type='file' name='update'>\
    <input type='submit' value='Update'>\
</form>\
<div id='prg'>progress: 0%</div>\
<script>\
$('form').submit(function(e){\
    e.preventDefault();\
      var form = $('#upload_form')[0];\
      var data = new FormData(form);\
       $.ajax({\
            url: '/update',\
            type: 'POST',\
            data: data,\
            contentType: false,\
            processData:false,\
            xhr: function() {\
                var xhr = new window.XMLHttpRequest();\
                xhr.upload.addEventListener('progress', function(evt) {\
                    if (evt.lengthComputable) {\
                        var per = evt.loaded / evt.total;\
                        $('#prg').html('progress: ' + Math.round(per*100) + '%');\
                    }\
               }, false);\
               return xhr;\
            },\
            success:function(d, s) {\
                console.log('success!')\
           },\
            error: function (a, b, c) {\
            }\
          });\
});\
</script>\
    </td>\
  </tr>\
 </table>\
</body>\
</html>")
    );
  SoC->swSer_enableRx(true);
  });

  server.on("/update", HTTP_POST, [](){
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Connection")), String(F("close")));
    server.sendHeader(String(F("Access-Control-Allow-Origin")), "*");
    server.send(200, String(F("text/plain")), (Update.hasError())?"FAIL":"OK");
//    SoC->swSer_enableRx(true);
    Sound_fini();
    RF_Shutdown();
    delay(1000);
    SoC->reset();
  },[](){
    HTTPUpload& upload = server.upload();
    if(upload.status == UPLOAD_FILE_START){
      Serial_setDebugOutput(true);
      SoC->WiFiUDP_stopAll();
      SoC->WDT_fini();
      Serial.printf("Update: %s\r\n", upload.filename.c_str());
      uint32_t maxSketchSpace = SoC->maxSketchSpace();
      if(!Update.begin(maxSketchSpace)){//start with max available size
        Update.printError(Serial);
      }
    } else if(upload.status == UPLOAD_FILE_WRITE){
      if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
        Update.printError(Serial);
      }
    } else if(upload.status == UPLOAD_FILE_END){
      if(Update.end(true)){ //true to set the size to the current progress
        Serial.printf("Update Success: %u\r\nRebooting...\r\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
      Serial_setDebugOutput(false);
    }
    yield();
  });
#endif /* EXCLUDE_OTA */

/* FLASH memory usage optimization */
#if !defined(ARDUINO_ARCH_RP2040)       && \
    !defined(ARDUINO_ARCH_RP2350)       && \
    !defined(CONFIG_IDF_TARGET_ESP32C2) && \
    !defined(CONFIG_IDF_TARGET_ESP32C6) && \
    !defined(CONFIG_IDF_TARGET_ESP32S3) && \
    !defined(ARDUINO_ARCH_RENESAS)

  server.on ( "/logo.png", []() {
    server.send_P ( 200, "image/png", Logo, sizeof(Logo) );
  } );
#endif /* ARDUINO_ARCH_RP2XXX CONFIG_IDF_TARGET_ESP32C6 ARDUINO_ARCH_RENESAS */

#if !defined(EXCLUDE_OTA)
  server.on ( "/jquery.min.js", []() {

    PGM_P content = jquery_min_js_gz;
    size_t bytes_left = jquery_min_js_gz_len;
    size_t chunk_size;

    server.setContentLength(bytes_left);
    server.sendHeader(String(F("Content-Encoding")),String(F("gzip")));
    server.send(200, String(F("application/javascript")), "");

    do {
      chunk_size = bytes_left > JS_MAX_CHUNK_SIZE ? JS_MAX_CHUNK_SIZE : bytes_left;
      server.sendContent_P(content, chunk_size);
      content += chunk_size;
      bytes_left -= chunk_size;
    } while (bytes_left > 0) ;

  } );
#endif /* EXCLUDE_OTA */

#if defined(ENABLE_RECORDER)
  server.on("/flights", HTTP_GET, Handle_Flight_Download);
#endif /* ENABLE_RECORDER */

  server.begin();
  Serial.println (F("HTTP server has started at port: 80"));

  delay(1000);
}

void Web_loop()
{
  server.handleClient();
}

void Web_fini()
{
  server.stop();
}

#endif /* EXCLUDE_WIFI */
