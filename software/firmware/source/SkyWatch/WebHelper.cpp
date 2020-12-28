/*
 * WebHelper.cpp
 * Copyright (C) 2016-2021 Linar Yusupov
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

#include <Arduino.h>
#include <TimeLib.h>

#include "SoCHelper.h"
#include "WebHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "BatteryHelper.h"
#include "GDL90Helper.h"
#include "BaroHelper.h"

#include <protocol.h>
#include <freqplan.h>

extern String host_name;

#define NOLOGO

static uint32_t prev_rx_pkt_cnt = 0;

#if !defined(NOLOGO)
static const char Logo[] PROGMEM = {
#include "Logo.h"
    } ;
#endif

#include "jquery_min_js.h"

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

static const char about_html[] PROGMEM = "<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>About</title>\
  </head>\
<body>\
<h1 align=center>About</h1>\
<p>This firmware is a part of open SoftRF project</p>\
<p>URL: http://github.com/lyusupov/SoftRF</p>\
<p>Author: <b>Linar Yusupov</b></p>\
<p>E-mail: linar.r.yusupov@gmail.com</p>\
<h2 align=center>Credits</h2>\
<p align=center>(in historical order)</p>\
<table width=100%%>\
<tr><th align=left>Paul Stoffregen</th><td align=left>Arduino Time library</td></tr>\
<tr><th align=left>Mikal Hart</th><td align=left>TinyGPS++ and PString libraries</td></tr>\
<tr><th align=left>Hristo Gochkov</th><td align=left>Arduino core for ESP32</td></tr>\
<tr><th align=left>JS Foundation</th><td align=left>jQuery library</td></tr>\
<tr><th align=left>Adafruit Industries</th><td align=left>SSD1306 and GFX libraries</td></tr>\
<tr><th align=left>Ryan David</th><td align=left>GDL90 decoder</td></tr>\
<tr><th align=left>Arundale Ramanathan</th><td align=left>Sqlite3 Arduino library</td></tr>\
<tr><th align=left>FlarmNet<br>GliderNet</th><td align=left>aircrafts data</td></tr>\
<tr><th align=left>Shenzhen Xin Yuan<br>(LilyGO) ET company</th><td align=left>TTGO T-Watch</td></tr>\
<tr><th align=left>Brian Park</th><td align=left>AceButton library</td></tr>\
<tr><th align=left>flashrom.org project</th><td align=left>Flashrom library</td></tr>\
<tr><th align=left>Evandro Copercini</th><td align=left>ESP32 BT SPP library</td></tr>\
<tr><th align=left>Lewis He</th><td align=left>AXP20X, BMA423, FT5206 and PCF8563 libraries</td></tr>\
<tr><th align=left>Bodmer</th><td align=left>TFT library</td></tr>\
</table>\
<hr>\
Copyright (C) 2019-2021 &nbsp;&nbsp;&nbsp; Linar Yusupov\
</body>\
</html>";

#if defined(EXPERIMENTAL)
static const char service_html[] PROGMEM = "<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>Service Mode</title>\
  </head>\
<body>\
<h1 align=center>Service Mode</h1>\
<br><br><p align=center>You've entered service mode.</p>\
<p align=center><input type=button onClick=\"location.href='/leave'\" value='Leave'></p>\
</body>\
</html>";

static const char leave_html[] PROGMEM = "<html>\
  <head>\
    <meta http-equiv='refresh' content='12; url=/'>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>Service Mode</title>\
  </head>\
<body>\
<h1 align=center>Service Mode</h1>\
<br><br><p align=center>You are leaving service mode. Please, wait...</p>\
</body>\
</html>";

static const char root_html[] PROGMEM = "<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>SkyWatch</title>\
  </head>\
<body>\
<h1 align=center>SkyWatch</h1>\
<br><br><p align=center>Welcome to SkyWatch !</p>\
<p align=center><input type=button onClick=\"location.href='http://192.168.1.1/status'\" value='Continue'></p>\
</body>\
</html>";

/** Is this an IP? */
boolean isIp(String str) {
  for (size_t i = 0; i < str.length(); i++) {
    int c = str.charAt(i);
    if (c != '.' && (c < '0' || c > '9')) {
      return false;
    }
  }
  return true;
}

/** IP to String? */
String toStringIp(IPAddress ip) {
  String res = "";
  for (int i = 0; i < 3; i++) {
    res += String((ip >> (8 * i)) & 0xFF) + ".";
  }
  res += String(((ip >> 8 * 3)) & 0xFF);
  return res;
}

/*
 * Redirect to captive portal if we got a request for another domain.
 * Return true in that case so the page handler do not try to handle the request again.
 */
bool captivePortal() {
  if (!isIp(server.hostHeader()) && server.hostHeader() != (String(host_name) + ".local")) {
//    Serial.println("Request redirected to captive portal");
    server.sendHeader(String(F("Location")), String("http://") + toStringIp(server.client().localIP()), true);
    server.send(302, "text/plain", "");   // Empty content inhibits Content-length header so we have to close the socket ourselves.
    server.client().stop(); // Stop is needed because we sent no content length
    return true;
  }
  return false;
}

void handleRoot() {

  if (captivePortal()) { // If caprive portal redirect instead of displaying the page.
    return;
  }

    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
    server.sendHeader(String(F("Pragma")), String(F("no-cache")));
    server.sendHeader(String(F("Expires")), String(F("-1")));
    server.send_P ( 200, PSTR("text/html"), root_html);
    SoC->swSer_enableRx(true);
}
#endif /* EXPERIMENTAL */

void handleSettings() {

  size_t size = 7160;
  char *offset;
  size_t len = 0;
  char *Settings_temp = (char *) malloc(size);

  if (Settings_temp == NULL) {
    return;
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
<th align=left>Connection port</th>\
<td align=right>\
<select name='connection'>\
<option %s value='%d'>MAIN</option>\
<option %s value='%d'>AUX</option>\
<!--<option %s value='%d'>WiFi UDP</option>\
<option %s value='%d'>Bluetooth SPP</option> -->\
</select>\
</td>\
</tr>\
<!-- <tr>\
<th align=left>Protocol</th>\
<td align=right>\
<select name='protocol'>\
<option %s value='%d'>NMEA</option>\
<option %s value='%d'>GDL90</option>\
</select>\
</td>\
</tr> -->\
<tr>\
<th align=left>Baud rate</th>\
<td align=right>\
<select name='baudrate'>\
<option %s value='%d'>4800</option>\
<option %s value='%d'>9600</option>\
<option %s value='%d'>19200</option>\
<option %s value='%d'>38400</option>\
<option %s value='%d'>57600</option>"),
  (settings->m.connection == CON_SERIAL_MAIN  ? "selected" : ""), CON_SERIAL_MAIN,
  (settings->m.connection == CON_SERIAL_AUX   ? "selected" : ""), CON_SERIAL_AUX,
  (settings->m.connection == CON_WIFI_UDP     ? "selected" : ""), CON_WIFI_UDP,
  (settings->m.connection == CON_BLUETOOTH    ? "selected" : ""), CON_BLUETOOTH,
  (settings->m.protocol   == PROTOCOL_NMEA    ? "selected" : ""), PROTOCOL_NMEA,
  (settings->m.protocol   == PROTOCOL_GDL90   ? "selected" : ""), PROTOCOL_GDL90,
  (settings->m.baudrate   == B4800            ? "selected" : ""), B4800,
  (settings->m.baudrate   == B9600            ? "selected" : ""), B9600,
  (settings->m.baudrate   == B19200           ? "selected" : ""), B19200,
  (settings->m.baudrate   == B38400           ? "selected" : ""), B38400,
  (settings->m.baudrate   == B57600           ? "selected" : ""), B57600
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 1 */
  if (SoC->id == SOC_ESP32) {
    snprintf_P ( offset, size,
      PSTR("\
<option %s value='%d'>115200</option>\
<option %s value='%d'>2000000</option>"),
    (settings->m.baudrate   == B115200        ? "selected" : ""), B115200,
    (settings->m.baudrate   == B2000000       ? "selected" : ""), B2000000
    );
    len = strlen(offset);
    offset += len;
    size -= len;
  }

  snprintf_P ( offset, size,
    PSTR("</select></td></tr><tr><th>&nbsp;</th><td>&nbsp;</td></tr>"));

  len = strlen(offset);
  offset += len;
  size -= len;

#if 0
  /* Common part 2 */
  snprintf_P ( offset, size,
    PSTR("\
<!-- <tr>\
<th align=left>Mode</th>\
<td align=right>\
<select name='mode'>\
<option %s value='%d'>Normal</option>\
<option %s value='%d'>Tx/Rx Test</option>\
<option %s value='%d'>Bridge</option>\
<option %s value='%d'>UAV</option>\
</select>\
</td>\
</tr> -->"),
  (settings->s.mode == SOFTRF_MODE_NORMAL ? "selected" : "") , SOFTRF_MODE_NORMAL,
  (settings->s.mode == SOFTRF_MODE_TXRX_TEST ? "selected" : ""), SOFTRF_MODE_TXRX_TEST,
  (settings->s.mode == SOFTRF_MODE_BRIDGE ? "selected" : ""), SOFTRF_MODE_BRIDGE,
  (settings->s.mode == SOFTRF_MODE_UAV ? "selected" : ""), SOFTRF_MODE_UAV
/*  (settings->s.mode == SOFTRF_MODE_WATCHOUT ? "selected" : ""), SOFTRF_MODE_WATCHOUT, */
  );

  len = strlen(offset);
  offset += len;
  size -= len;
#endif

  /* Radio specific part */
  if (hw_info.rf == RF_IC_SX1276) {  /* + RF_IC_CC13XX */
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Protocol</th>\
<td align=right>\
<select name='protocol'>\
<option %s value='%d'>%s</option>\
<option %s value='%d'>%s</option>\
<option %s value='%d'>%s</option>\
<option %s value='%d'>%s</option>\
<option %s value='%d'>%s</option>\
</select>\
</td>\
</tr>"),
    (settings->s.rf_protocol == RF_PROTOCOL_LEGACY ? "selected" : ""),
     RF_PROTOCOL_LEGACY, "Legacy",
    (settings->s.rf_protocol == RF_PROTOCOL_OGNTP ? "selected" : ""),
     RF_PROTOCOL_OGNTP, "OGNTP",
    (settings->s.rf_protocol == RF_PROTOCOL_P3I ? "selected" : ""),
     RF_PROTOCOL_P3I, "P3I",
    (settings->s.rf_protocol == RF_PROTOCOL_ADSB_UAT ? "selected" : ""),
     RF_PROTOCOL_ADSB_UAT, "UAT",
    (settings->s.rf_protocol == RF_PROTOCOL_FANET ? "selected" : ""),
     RF_PROTOCOL_FANET, "FANET"
    );
  } else {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Protocol</th>\
<td align=right>%s\
</td>\
</tr>"),
    (settings->s.rf_protocol == RF_PROTOCOL_LEGACY   ? "Legacy" :
    (settings->s.rf_protocol == RF_PROTOCOL_ADSB_UAT ? "UAT" :
     "UNK"))
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
<option %s value='%d'>AUTO</option>\
<option %s value='%d'>EU (868.2 MHz)</option>\
<option %s value='%d'>RU (868.8 MHz)</option>\
<option %s value='%d'>CN (470 MHz)</option>\
<option %s value='%d'>US/CA (915 MHz)</option>\
<option %s value='%d'>NZ (869.25 MHz)</option>\
<option %s value='%d'>UK (869.52 MHz)</option>\
<option %s value='%d'>AU (921 MHz)</option>\
<option %s value='%d'>IN (866 MHz)</option>\
<option %s value='%d'>KR (920.9 MHz)</option>\
<option %s value='%d'>IL (916.2 MHz)</option>\
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
<!-- <tr>\
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
</tr> -->"),
  (settings->s.band == RF_BAND_AUTO ? "selected" : ""), RF_BAND_AUTO,
  (settings->s.band == RF_BAND_EU ? "selected" : ""), RF_BAND_EU,
  (settings->s.band == RF_BAND_RU ? "selected" : ""), RF_BAND_RU,
  (settings->s.band == RF_BAND_CN ? "selected" : ""), RF_BAND_CN,
  (settings->s.band == RF_BAND_US ? "selected" : ""), RF_BAND_US,
  (settings->s.band == RF_BAND_NZ ? "selected" : ""), RF_BAND_NZ,
  (settings->s.band == RF_BAND_UK ? "selected" : ""), RF_BAND_UK,
  (settings->s.band == RF_BAND_AU ? "selected" : ""), RF_BAND_AU,
  (settings->s.band == RF_BAND_IN ? "selected" : ""), RF_BAND_IN,
  (settings->s.band == RF_BAND_KR ? "selected" : ""), RF_BAND_KR,
  (settings->s.band == RF_BAND_IL ? "selected" : ""), RF_BAND_IL,
  (settings->s.aircraft_type == AIRCRAFT_TYPE_GLIDER ? "selected" : ""),  AIRCRAFT_TYPE_GLIDER,
  (settings->s.aircraft_type == AIRCRAFT_TYPE_TOWPLANE ? "selected" : ""),  AIRCRAFT_TYPE_TOWPLANE,
  (settings->s.aircraft_type == AIRCRAFT_TYPE_POWERED ? "selected" : ""),  AIRCRAFT_TYPE_POWERED,
  (settings->s.aircraft_type == AIRCRAFT_TYPE_HELICOPTER ? "selected" : ""),  AIRCRAFT_TYPE_HELICOPTER,
  (settings->s.aircraft_type == AIRCRAFT_TYPE_UAV ? "selected" : ""),  AIRCRAFT_TYPE_UAV,
  (settings->s.aircraft_type == AIRCRAFT_TYPE_HANGGLIDER ? "selected" : ""),  AIRCRAFT_TYPE_HANGGLIDER,
  (settings->s.aircraft_type == AIRCRAFT_TYPE_PARAGLIDER ? "selected" : ""),  AIRCRAFT_TYPE_PARAGLIDER,
  (settings->s.aircraft_type == AIRCRAFT_TYPE_BALLOON ? "selected" : ""),  AIRCRAFT_TYPE_BALLOON,
  (settings->s.aircraft_type == AIRCRAFT_TYPE_STATIC ? "selected" : ""),  AIRCRAFT_TYPE_STATIC,
  (settings->s.alarm == TRAFFIC_ALARM_NONE ? "selected" : ""),  TRAFFIC_ALARM_NONE,
  (settings->s.alarm == TRAFFIC_ALARM_DISTANCE ? "selected" : ""),  TRAFFIC_ALARM_DISTANCE,
  (settings->s.alarm == TRAFFIC_ALARM_VECTOR ? "selected" : ""),  TRAFFIC_ALARM_VECTOR,
  (settings->s.txpower == RF_TX_POWER_FULL ? "selected" : ""),  RF_TX_POWER_FULL,
  (settings->s.txpower == RF_TX_POWER_LOW ? "selected" : ""),  RF_TX_POWER_LOW,
  (settings->s.txpower == RF_TX_POWER_OFF ? "selected" : ""),  RF_TX_POWER_OFF,
  (settings->s.volume == BUZZER_VOLUME_FULL ? "selected" : ""), BUZZER_VOLUME_FULL,
  (settings->s.volume == BUZZER_VOLUME_LOW ? "selected" : ""), BUZZER_VOLUME_LOW,
  (settings->s.volume == BUZZER_OFF ? "selected" : ""), BUZZER_OFF,
  (settings->s.pointer == DIRECTION_TRACK_UP ? "selected" : ""), DIRECTION_TRACK_UP,
  (settings->s.pointer == DIRECTION_NORTH_UP ? "selected" : ""), DIRECTION_NORTH_UP,
  (settings->s.pointer == LED_OFF ? "selected" : ""), LED_OFF
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 1 */
  if (SoC->id == SOC_ESP32) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Built-in Bluetooth</th>\
<td align=right>\
<select name='bluetooth'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>SPP</option>\
<option %s value='%d'>LE</option>\
</select>\
</td>\
</tr>"),
    (settings->s.bluetooth == BLUETOOTH_OFF ? "selected" : ""), BLUETOOTH_OFF,
    (settings->s.bluetooth == BLUETOOTH_SPP ? "selected" : ""), BLUETOOTH_SPP,
    (settings->s.bluetooth == BLUETOOTH_LE_HM10_SERIAL ? "selected" : ""), BLUETOOTH_LE_HM10_SERIAL
    );

    len = strlen(offset);
    offset += len;
    size -= len;
  }

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
  (!settings->s.nmea_g ? "checked" : "") , (settings->s.nmea_g ? "checked" : ""),
  (!settings->s.nmea_p ? "checked" : "") , (settings->s.nmea_p ? "checked" : ""),
  (!settings->s.nmea_l ? "checked" : "") , (settings->s.nmea_l ? "checked" : ""),
  (!settings->s.nmea_s ? "checked" : "") , (settings->s.nmea_s ? "checked" : ""),
  (settings->s.nmea_out == NMEA_OFF ? "selected" : ""), NMEA_OFF,
  (settings->s.nmea_out == NMEA_UART ? "selected" : ""), NMEA_UART,
  (settings->s.nmea_out == NMEA_UDP ? "selected" : ""), NMEA_UDP);

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 2 */
  if (SoC->id == SOC_ESP32) {
    snprintf_P ( offset, size,
      PSTR("\
<option %s value='%d'>TCP</option>\
<option %s value='%d'>Bluetooth</option>"),
    (settings->s.nmea_out == NMEA_TCP ? "selected" : ""), NMEA_TCP,
    (settings->s.nmea_out == NMEA_BLUETOOTH ? "selected" : ""), NMEA_BLUETOOTH);

    len = strlen(offset);
    offset += len;
    size -= len;
  }

#if 0
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
  (settings->s.gdl90 == GDL90_OFF ? "selected" : ""), GDL90_OFF,
  (settings->s.gdl90 == GDL90_UART ? "selected" : ""), GDL90_UART,
  (settings->s.gdl90 == GDL90_UDP ? "selected" : ""), GDL90_UDP);

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 3 */
  if (SoC->id == SOC_ESP32) {
    snprintf_P ( offset, size,
      PSTR("\
<option %s value='%d'>Bluetooth</option>"),
    (settings->s.gdl90 == GDL90_BLUETOOTH ? "selected" : ""), GDL90_BLUETOOTH);

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
  (settings->s.d1090 == D1090_OFF ? "selected" : ""), D1090_OFF,
  (settings->s.d1090 == D1090_UART ? "selected" : ""), D1090_UART);

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 4 */
  if (SoC->id == SOC_ESP32) {
    snprintf_P ( offset, size,
      PSTR("\
<option %s value='%d'>Bluetooth</option>"),
    (settings->s.d1090 == D1090_BLUETOOTH ? "selected" : ""), D1090_BLUETOOTH);

    len = strlen(offset);
    offset += len;
    size -= len;
  }

  /* SoC specific part 5 */
  if (SoC->id == SOC_ESP32) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Display adapter</th>\
<td align=right>\
<select name='adapter'>\
<option %s value='%d'>e-Paper TTGO T5S</option>\
<option %s value='%d'>e-Paper Waveshare ESP32</option>\
<option %s value='%d'>OLED</option>\
</select>\
</td>\
</tr>"),
    (settings->m.adapter == ADAPTER_TTGO_T5S        ? "selected" : ""), ADAPTER_TTGO_T5S,
    (settings->m.adapter == ADAPTER_WAVESHARE_ESP32 ? "selected" : ""), ADAPTER_WAVESHARE_ESP32,
    (settings->m.adapter == ADAPTER_OLED            ? "selected" : ""), ADAPTER_OLED
    );
  } else if (SoC->id == SOC_ESP8266) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Display adapter</th>\
<td align=right>\
<select name='adapter'>\
<option %s value='%d'>NodeMCU</option>\
<option %s value='%d'>e-Paper Waveshare ESP8266</option>\
</select>\
</td>\
</tr>"),
    (settings->m.adapter == ADAPTER_NODEMCU           ? "selected" : ""), ADAPTER_NODEMCU,
    (settings->m.adapter == ADAPTER_WAVESHARE_ESP8266 ? "selected" : ""), ADAPTER_WAVESHARE_ESP8266
    );
  }

  len = strlen(offset);
  offset += len;
  size -= len;
#endif

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
<option %s value='%d'>WiFi OFF (5 min.)</option>\
<option %s value='%d'>GNSS</option>\
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
  (settings->s.power_save == POWER_SAVE_NONE ? "selected" : ""), POWER_SAVE_NONE,
  (settings->s.power_save == POWER_SAVE_WIFI ? "selected" : ""), POWER_SAVE_WIFI,
  (settings->s.power_save == POWER_SAVE_GNSS ? "selected" : ""), POWER_SAVE_GNSS,
  (!settings->s.stealth ? "checked" : "") , (settings->s.stealth ? "checked" : ""),
  (!settings->s.no_track ? "checked" : "") , (settings->s.no_track ? "checked" : "")
  );

  len = strlen(offset);
  offset += len;
  size -= len;

#if 0
    /* Common part 7 */
  snprintf_P ( offset, size,
    PSTR("\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Source WiFi SSID</th>\
<td align=right>\
<INPUT type='text' name='ssid' maxlength='15' size='15' value='%s'>\
</td>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;PSK</th>\
<td align=right>\
<INPUT type='text' name='psk' maxlength='15' size='15' value='%s'>\
</td>\
</tr>"),
   settings->m.ssid, settings->m.psk);

  len = strlen(offset);
  offset += len;
  size -= len;
#endif

/*

<tr>\
<th align=left>Built-in Bluetooth</th>\
<td align=right>\
<select name='bluetooth'>\
<option %s value='%d'>off</option>\
<option %s value='%d'>SPP</option>\
</select>\
</td>\
</tr>\

    (settings->m.bluetooth == BLUETOOTH_OFF ? "selected" : ""), BLUETOOTH_OFF,
    (settings->m.bluetooth == BLUETOOTH_SPP ? "selected" : ""), BLUETOOTH_SPP,

 */

#if 0
  /* SoC specific part 8 */
  if (SoC->id == SOC_ESP32) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Source BT Name</th>\
<td align=right>\
<INPUT type='text' name='bt_name' maxlength='15' size='15' value='%s'>\
</td>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Key</th>\
<td align=right>\
<INPUT type='text' name='bt_key' maxlength='15' size='15' value='%s'>\
</td>\
</tr>"),
     settings->m.bt_name, settings->m.bt_key);

    len = strlen(offset);
    offset += len;
    size -= len;
  }
#endif

  if (hw_info.display != DISPLAY_NONE) {
    /* SoC specific part 9 */
    snprintf_P ( offset, size,
      PSTR("\
<tr><th>&nbsp;</th><td>&nbsp;</td></tr>\
<tr>\
<th align=left>Units</th>\
<td align=right>\
<select name='units'>\
<option %s value='%d'>metric</option>\
<option %s value='%d'>imperial</option>\
<option %s value='%d'>mixed</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>View mode</th>\
<td align=right>\
<select name='vmode'>\
<option %s value='%d'>status</option>\
<option %s value='%d'>radar</option>\
<option %s value='%d'>text</option>\
<option %s value='%d'>time</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Radar orientation</th>\
<td align=right>\
<select name='orientation'>\
<option %s value='%d'>CoG Up</option>\
<option %s value='%d'>North Up</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Zoom level</th>\
<td align=right>\
<select name='zoom'>\
<option %s value='%d'>lowest</option>\
<option %s value='%d'>low</option>\
<option %s value='%d'>medium</option>\
<option %s value='%d'>high</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Aircrafts data</th>\
<td align=right>\
<select name='adb'>\
<option %s value='%d'>auto</option>\
<option %s value='%d'>FlarmNet</option>\
<option %s value='%d'>GliderNet</option>\
<option %s value='%d'>ICAO</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>ID preference</th>\
<td align=right>\
<select name='idpref'>\
<option %s value='%d'>registration</option>\
<option %s value='%d'>tail/CN</option>\
<option %s value='%d'>make & model</option>\
</select>\
</td>\
</tr>"),
    (settings->m.units == UNITS_METRIC     ? "selected" : ""), UNITS_METRIC,
    (settings->m.units == UNITS_IMPERIAL   ? "selected" : ""), UNITS_IMPERIAL,
    (settings->m.units == UNITS_MIXED      ? "selected" : ""), UNITS_MIXED,
    (settings->m.vmode == VIEW_MODE_STATUS ? "selected" : ""), VIEW_MODE_STATUS,
    (settings->m.vmode == VIEW_MODE_RADAR  ? "selected" : ""), VIEW_MODE_RADAR,
    (settings->m.vmode == VIEW_MODE_TEXT   ? "selected" : ""), VIEW_MODE_TEXT,
    (settings->m.vmode == VIEW_MODE_TIME   ? "selected" : ""), VIEW_MODE_TIME,
    (settings->m.orientation == DIRECTION_TRACK_UP ? "selected" : ""), DIRECTION_TRACK_UP,
    (settings->m.orientation == DIRECTION_NORTH_UP ? "selected" : ""), DIRECTION_NORTH_UP,
    (settings->m.zoom == ZOOM_LOWEST ? "selected" : ""), ZOOM_LOWEST,
    (settings->m.zoom == ZOOM_LOW    ? "selected" : ""), ZOOM_LOW,
    (settings->m.zoom == ZOOM_MEDIUM ? "selected" : ""), ZOOM_MEDIUM,
    (settings->m.zoom == ZOOM_HIGH   ? "selected" : ""), ZOOM_HIGH,
    (settings->m.adb == DB_AUTO      ? "selected" : ""), DB_AUTO,
    (settings->m.adb == DB_FLN       ? "selected" : ""), DB_FLN,
    (settings->m.adb == DB_OGN       ? "selected" : ""), DB_OGN,
    (settings->m.adb == DB_ICAO      ? "selected" : ""), DB_ICAO,
    (settings->m.idpref == ID_REG    ? "selected" : ""), ID_REG,
    (settings->m.idpref == ID_TAIL   ? "selected" : ""), ID_TAIL,
    (settings->m.idpref == ID_MAM    ? "selected" : ""), ID_MAM
    );

    len = strlen(offset);
    offset += len;
    size -= len;

#if 0
    /* SoC specific part 10 */
    if (SoC->id == SOC_ESP32) {
      snprintf_P ( offset, size,
        PSTR("\
<tr>\
<th align=left>Voice</th>\
<td align=right>\
<select name='voice'>\
<option %s value='%d'>off</option>\
<option %s value='%d'>voice 1</option>\
<option %s value='%d'>voice 2</option>\
<option %s value='%d'>voice 3</option>\
</select>\
</td>\
</tr>"),
      (settings->m.voice == VOICE_OFF  ? "selected" : ""), VOICE_OFF,
      (settings->m.voice == VOICE_1    ? "selected" : ""), VOICE_1,
      (settings->m.voice == VOICE_2    ? "selected" : ""), VOICE_2,
      (settings->m.voice == VOICE_3    ? "selected" : ""), VOICE_3
      );

      len = strlen(offset);
      offset += len;
      size -= len;
    }
#endif

  } /* hw_info.display != DISPLAY_NONE */

  /* Common part 8 */
  snprintf_P ( offset, size,
    PSTR("\
<!-- <tr>\
<th align=left>e-Paper 'ghosts' removal</th>\
<td align=right>\
<select name='aghost'>\
<option %s value='%d'>off</option>\
<option %s value='%d'>auto</option>\
<option %s value='%d'> 2 minutes</option>\
<option %s value='%d'> 5 minutes</option>\
<option %s value='%d'>10 minutes</option>\
</select>\
</td>\
</tr> -->\
</table>\
<p align=center><INPUT type='submit' value='Save and restart'></p>\
</form>\
</body>\
</html>"),
    (settings->m.aghost == ANTI_GHOSTING_OFF   ? "selected" : ""), ANTI_GHOSTING_OFF,
    (settings->m.aghost == ANTI_GHOSTING_AUTO  ? "selected" : ""), ANTI_GHOSTING_AUTO,
    (settings->m.aghost == ANTI_GHOSTING_2MIN  ? "selected" : ""), ANTI_GHOSTING_2MIN,
    (settings->m.aghost == ANTI_GHOSTING_5MIN  ? "selected" : ""), ANTI_GHOSTING_5MIN,
    (settings->m.aghost == ANTI_GHOSTING_10MIN ? "selected" : ""), ANTI_GHOSTING_10MIN
  );

  SoC->swSer_enableRx(false);
  server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
  server.sendHeader(String(F("Pragma")), String(F("no-cache")));
  server.sendHeader(String(F("Expires")), String(F("-1")));
  server.send ( 200, "text/html", Settings_temp );
  SoC->swSer_enableRx(true);
  free(Settings_temp);
}

void handleStatus() {
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  float vdd = Battery_voltage() ;
  bool low_voltage = (Battery_voltage() <= Battery_threshold());

  time_t timestamp = now();
  char str_Vcc[8];

  size_t size = 2700;
  char *offset;
  size_t len = 0;

  char *Root_temp = (char *) malloc(size);
  if (Root_temp == NULL) {
    return;
  }
  offset = Root_temp;

  dtostrf(vdd, 4, 2, str_Vcc);

  snprintf_P ( offset, size,
    PSTR("<html>\
<head>\
  <meta name='viewport' content='width=device-width, initial-scale=1'>\
  <title>%s status</title>\
</head>\
<body>\
<table width=100%%>\
  <tr><!-- <td align=left><h1>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</h1></td> -->\
  <td align=center><h1>%s status</h1></td>\
  <!-- <td align=right><img src='/logo.png'></td> --></tr>\
</table>\
<table width=100%%>\
<tr><th align=left>Device Id</th><td align=right>%06X</td></tr>\
<tr><th align=left>Software Version</th><td align=right>%s&nbsp;&nbsp;%s</td></tr>\
<tr><th align=left>Uptime</th><td align=right>%02d:%02d:%02d</td></tr>\
<tr><th align=left>Free memory</th><td align=right>%u</td></tr>\
<tr><th align=left>Battery voltage</th><td align=right><font color=%s>%s</font></td></tr>\
<tr><th align=left>&nbsp;</th><td align=right>&nbsp;</td></tr>"),
    hw_info.model == SOFTRF_MODEL_SKYWATCH ? SKYWATCH_IDENT : SOFTRF_IDENT " WT",
    hw_info.model == SOFTRF_MODEL_SKYWATCH ? SKYWATCH_IDENT : SOFTRF_IDENT " WT",
    SoC->getChipId() & 0xFFFFFF, SKYWATCH_FIRMWARE_VERSION,
    (SoC == NULL ? "NONE" : SoC->name),
    hr, min % 60, sec % 60, ESP.getFreeHeap(),
    low_voltage ? "red" : "green", str_Vcc
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    snprintf_P ( offset, size,
      PSTR("\
<tr><th align=left>Display</th><td align=right>%s</td></tr>\
<tr><th align=left>Storage</th><td align=right>%s</td></tr>\
<tr><th align=left>Baro</th><td align=right>%s</td></tr>"),
      hw_info.display      == DISPLAY_EPD_2_7  ? "e-Paper" :
      hw_info.display      == DISPLAY_OLED_2_4 ? "OLED"    :
      hw_info.display      == DISPLAY_TFT_TTGO ? "LCD"     : "NONE",
      hw_info.storage      == STORAGE_uSD      ? "uSD"     : "NONE",
      (baro_chip == NULL ? "NONE" : baro_chip->name)
    );

    len = strlen(offset);
    offset += len;
    size -= len;
  }

  snprintf_P ( offset, size,
    PSTR("<tr><th align=left>Connection type</th><td align=right>%s</td></tr>"),
    settings->m.connection == CON_SERIAL_MAIN  ? "Main Serial" :
    settings->m.connection == CON_SERIAL_AUX   ? "AUX Serial" :
    settings->m.connection == CON_BLUETOOTH    ? "Bluetooth" :
    settings->m.connection == CON_WIFI_UDP     ? "WiFi" : "NONE"
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  switch (settings->m.connection)
  {
  case CON_WIFI_UDP:
    snprintf_P ( offset, size,
      PSTR("\
<tr><th align=left>Link partner</th><td align=right>%s</td></tr>\
<tr><th align=left>Link status</th><td align=right>%s established</td></tr>\
<tr><th align=left>Assigned IP address</th><td align=right>%s</td></tr>"),
      settings->m.ssid && strlen(settings->m.ssid) > 0 ? settings->m.ssid : "NOT SET",
      WiFi.status() == WL_CONNECTED ? "" : "not",
      WiFi.localIP().toString().c_str()
    );
    len = strlen(offset);
    offset += len;
    size -= len;
  case CON_SERIAL_MAIN:
  case CON_SERIAL_AUX:
  case CON_BLUETOOTH:
    switch (settings->m.protocol)
    {
    case PROTOCOL_GDL90:
      snprintf_P ( offset, size,
        PSTR("\
<tr><th align=left>Connection status</th><td align=right>%s connected</td></tr>\
<tr><th align=left>Data type</th><td align=right>%s %s</td></tr>"),
        GDL90_isConnected()  ? "" : "not",
        GDL90_isConnected()  && !GDL90_hasHeartBeat() ? "UNK" : "",
        GDL90_hasHeartBeat() ? "GDL90"  : ""
      );
      break;
    case PROTOCOL_NMEA:
    default:
      snprintf_P ( offset, size,
        PSTR("\
<tr><th align=left>Connection status</th><td align=right>%s connected</td></tr>\
<tr><th align=left>Data type</th><td align=right>%s %s %s</td></tr>\
<tr><th align=left>GNSS fix</th><td align=right>%s</td></tr>\
<tr><th align=left>Slave Id</th><td align=right>%06X</td></tr>\
<tr><th align=left>RF protocol</th><td align=right>%s</td></tr>\
</table>\
<table width=100%%>\
<tr><th align=left>Packets</th>\
  <td align=right><table><tr>\
   <th align=left>Tx&nbsp;&nbsp;</th><td align=right>%u</td>\
   <th align=left>&nbsp;&nbsp;&nbsp;&nbsp;Rx&nbsp;&nbsp;</th><td align=right>%u</td>\
  </tr></table></td></tr>"),
        NMEA_isConnected() ? "" : "not",
        NMEA_isConnected() && !(NMEA_hasGNSS() || NMEA_hasFLARM()) ? "UNK" : "",
        NMEA_hasGNSS()     ? "GNSS"  : "",
        NMEA_hasFLARM()    ? "FLARM" : "",
        NMEA_has3DFix()    ? "3D" : "NONE",
        ThisDevice.addr,
        ThisDevice.protocol == RF_PROTOCOL_LEGACY   ? "Legacy" :
        ThisDevice.protocol == RF_PROTOCOL_OGNTP    ? "OGNTP"  :
        ThisDevice.protocol == RF_PROTOCOL_P3I      ? "P3I"    :
        ThisDevice.protocol == RF_PROTOCOL_ADSB_UAT ? "UAT"    :
        ThisDevice.protocol == RF_PROTOCOL_FANET    ? "FANET"  : "UNK",
        tx_packets_counter, rx_packets_counter
      );
      break;
    }

    len = strlen(offset);
    offset += len;
    size -= len;
    break;
  case CON_NONE:
  default:
    break;
  }

  snprintf_P ( offset, size,
    PSTR(" </table>\
<hr>\
<table width=100%%>\
  <tr>\
    <td align=left><input type=button onClick=\"location.href='/settings'\" value='Settings'></td>\
    <td align=center><input type=button onClick=\"location.href='/about'\" value='About'></td>\
    <td align=right><input type=button onClick=\"location.href='/firmware'\" value='Firmware update'></td>\
  </tr>\
</table>\
</body>\
</html>")
  );

  SoC->swSer_enableRx(false);
  server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
  server.sendHeader(String(F("Pragma")), String(F("no-cache")));
  server.sendHeader(String(F("Expires")), String(F("-1")));
  server.send ( 200, "text/html", Root_temp );
  SoC->swSer_enableRx(true);
  free(Root_temp);
}

void handleInput() {

  char *Input_temp = (char *) malloc(3200);
  if (Input_temp == NULL) {
    return;
  }

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    if (server.argName(i).equals("mode")) {
      settings->s.mode = server.arg(i).toInt();
    } else if (server.argName(i).equals("protocol")) {
      settings->s.rf_protocol = server.arg(i).toInt();
    } else if (server.argName(i).equals("band")) {
      settings->s.band = server.arg(i).toInt();
    } else if (server.argName(i).equals("acft_type")) {
      settings->s.aircraft_type = server.arg(i).toInt();
    } else if (server.argName(i).equals("alarm")) {
      settings->s.alarm = server.arg(i).toInt();
    } else if (server.argName(i).equals("txpower")) {
      settings->s.txpower = server.arg(i).toInt();
    } else if (server.argName(i).equals("volume")) {
      settings->s.volume = server.arg(i).toInt();
    } else if (server.argName(i).equals("pointer")) {
      settings->s.pointer = server.arg(i).toInt();
    } else if (server.argName(i).equals("bluetooth")) {
      settings->s.bluetooth = server.arg(i).toInt();
    } else if (server.argName(i).equals("nmea_g")) {
      settings->s.nmea_g = server.arg(i).toInt();
    } else if (server.argName(i).equals("nmea_p")) {
      settings->s.nmea_p = server.arg(i).toInt();
    } else if (server.argName(i).equals("nmea_l")) {
      settings->s.nmea_l = server.arg(i).toInt();
    } else if (server.argName(i).equals("nmea_s")) {
      settings->s.nmea_s = server.arg(i).toInt();
    } else if (server.argName(i).equals("nmea_out")) {
      settings->s.nmea_out = server.arg(i).toInt();
    } else if (server.argName(i).equals("gdl90")) {
      settings->s.gdl90 = server.arg(i).toInt();
    } else if (server.argName(i).equals("d1090")) {
      settings->s.d1090 = server.arg(i).toInt();
    } else if (server.argName(i).equals("stealth")) {
      settings->s.stealth = server.arg(i).toInt();
    } else if (server.argName(i).equals("no_track")) {
      settings->s.no_track = server.arg(i).toInt();
    } else if (server.argName(i).equals("power_save")) {
      settings->s.power_save = server.arg(i).toInt();
    } else if (server.argName(i).equals("adapter")) {
      settings->m.adapter = server.arg(i).toInt();
    } else if (server.argName(i).equals("connection")) {
      settings->m.connection = server.arg(i).toInt();
    } else if (server.argName(i).equals("protocol")) {
      settings->m.protocol = server.arg(i).toInt();
    } else if (server.argName(i).equals("baudrate")) {
      settings->m.baudrate = server.arg(i).toInt();
    } else if (server.argName(i).equals("ssid")) {
      server.arg(i).toCharArray(settings->m.ssid, sizeof(settings->m.ssid));
    } else if (server.argName(i).equals("psk")) {
      server.arg(i).toCharArray(settings->m.psk, sizeof(settings->m.psk));
    } else if (server.argName(i).equals("units")) {
      settings->m.units = server.arg(i).toInt();
    } else if (server.argName(i).equals("vmode")) {
      settings->m.vmode = server.arg(i).toInt();
    } else if (server.argName(i).equals("orientation")) {
      settings->m.orientation = server.arg(i).toInt();
    } else if (server.argName(i).equals("zoom")) {
      settings->m.zoom = server.arg(i).toInt();
    } else if (server.argName(i).equals("adb")) {
      settings->m.adb = server.arg(i).toInt();
    } else if (server.argName(i).equals("idpref")) {
      settings->m.idpref = server.arg(i).toInt();
    } else if (server.argName(i).equals("voice")) {
      settings->m.voice = server.arg(i).toInt();
    } else if (server.argName(i).equals("aghost")) {
      settings->m.aghost = server.arg(i).toInt();
//    } else if (server.argName(i).equals("bluetooth")) {
//      settings->m.bluetooth = server.arg(i).toInt();
    } else if (server.argName(i).equals("bt_name")) {
      server.arg(i).toCharArray(settings->m.bt_name, sizeof(settings->m.bt_name));
    } else if (server.argName(i).equals("bt_key")) {
      server.arg(i).toCharArray(settings->m.bt_key, sizeof(settings->m.bt_key));
    }
  }
  snprintf_P ( Input_temp, 3200,
PSTR("<html>\
<head>\
<meta http-equiv='refresh' content='15; url=/'>\
<meta name='viewport' content='width=device-width, initial-scale=1'>\
<title>%s Settings</title>\
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
<tr><th align=left>Adapter</th><td align=right>%d</td></tr>\
<tr><th align=left>Connection</th><td align=right>%d</td></tr>\
<tr><th align=left>Protocol</th><td align=right>%d</td></tr>\
<tr><th align=left>Baud rate</th><td align=right>%d</td></tr>\
<tr><th align=left>WiFi SSID</th><td align=right>%s</td></tr>\
<tr><th align=left>WiFi PSK</th><td align=right>%s</td></tr>\
<tr><th align=left>Units</th><td align=right>%d</td></tr>\
<tr><th align=left>View mode</th><td align=right>%d</td></tr>\
<tr><th align=left>Radar orientation</th><td align=right>%d</td></tr>\
<tr><th align=left>Zoom level</th><td align=right>%d</td></tr>\
<tr><th align=left>Aircrafts data</th><td align=right>%d</td></tr>\
<tr><th align=left>ID preference</th><td align=right>%d</td></tr>\
<tr><th align=left>Voice</th><td align=right>%d</td></tr>\
<tr><th align=left>'Ghosts' removal</th><td align=right>%d</td></tr>\
<tr><th align=left>Bluetooth</th><td align=right>%d</td></tr>\
<tr><th align=left>BT Name</th><td align=right>%s</td></tr>\
<tr><th align=left>BT Key</th><td align=right>%s</td></tr>\
</table>\
<hr>\
  <p align=center><h1 align=center>Restart is in progress... Please, wait!</h1></p>\
</body>\
</html>"),
  hw_info.model == SOFTRF_MODEL_SKYWATCH ? SKYWATCH_IDENT : SOFTRF_IDENT " WT",
  settings->s.mode, settings->s.rf_protocol, settings->s.band,
  settings->s.aircraft_type, settings->s.alarm, settings->s.txpower,
  settings->s.volume, settings->s.pointer, settings->s.bluetooth,
  BOOL_STR(settings->s.nmea_g), BOOL_STR(settings->s.nmea_p),
  BOOL_STR(settings->s.nmea_l), BOOL_STR(settings->s.nmea_s),
  settings->s.nmea_out, settings->s.gdl90, settings->s.d1090,
  BOOL_STR(settings->s.stealth), BOOL_STR(settings->s.no_track),
  settings->s.power_save,
  settings->m.adapter, settings->m.connection, settings->m.protocol,
  settings->m.baudrate, settings->m.ssid, settings->m.psk,
  settings->m.units, settings->m.vmode, settings->m.orientation, settings->m.zoom,
  settings->m.adb, settings->m.idpref, settings->m.voice, settings->m.aghost,
  settings->m.bluetooth, settings->m.bt_name, settings->m.bt_key
  );

  SoC->swSer_enableRx(false);
  server.send ( 200, "text/html", Input_temp );
//  SoC->swSer_enableRx(true);
  delay(1000);
  free(Input_temp);
  NMEA_Save_Settings();
  EEPROM_store();
  delay(1000);
  ESP.restart();
}

void handleNotFound() {
#if defined(EXPERIMENTAL)
  if (captivePortal()) { // If caprive portal redirect instead of displaying the page.
    return;
  }
#endif /* EXPERIMENTAL */
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

void Web_setup()
{
#if defined(EXPERIMENTAL)
  server.on ( "/", handleRoot );
//  server.on ( "/generate_204", handleRoot); // Android captive portal.
  server.on ( "/fwlink", handleRoot);       // Microsoft captive portal.
  server.on ( "/status", handleStatus );
#else
  server.on ( "/", handleStatus );
#endif /* EXPERIMENTAL */
  server.on ( "/settings", handleSettings );
  server.on ( "/about", []() {
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
    server.sendHeader(String(F("Pragma")), String(F("no-cache")));
    server.sendHeader(String(F("Expires")), String(F("-1")));
    server.send_P ( 200, PSTR("text/html"), about_html);
    SoC->swSer_enableRx(true);
  } );
#if defined(EXPERIMENTAL)
  server.on ( "/service", []() {
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
    server.sendHeader(String(F("Pragma")), String(F("no-cache")));
    server.sendHeader(String(F("Expires")), String(F("-1")));
    server.send_P ( 200, PSTR("text/html"), service_html);
    SoC->swSer_enableRx(true);
    SoC->Service_Mode(true);
  } );
  server.on ( "/leave", []() {
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
    server.sendHeader(String(F("Pragma")), String(F("no-cache")));
    server.sendHeader(String(F("Expires")), String(F("-1")));
    server.send_P ( 200, PSTR("text/html"), leave_html);
    SoC->swSer_enableRx(true);
    SoC->Service_Mode(false);
  } );
#endif /* EXPERIMENTAL */
  server.on ( "/input", handleInput );
  server.on ( "/inline", []() {
    server.send ( 200, "text/plain", "this works as well" );
  } );
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
 <!--<p align=center>(main board)</p>-->\
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
<!--<hr>\
 <h1 align=center>Radio/GNSS board</h1>\
 <p align=center><input type=button onClick=\"location.href='/service'\" value='Service Mode'></p>-->\
</body>\
</html>")
    );
  SoC->swSer_enableRx(true);
  });
  server.onNotFound ( handleNotFound );

  server.on("/update", HTTP_POST, [](){
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Connection")), String(F("close")));
    server.sendHeader(String(F("Access-Control-Allow-Origin")), "*");
    server.send(200, String(F("text/plain")), (Update.hasError())?"FAIL":"OK");
//    SoC->swSer_enableRx(true);
    delay(1000);
    ESP.restart();
  },[](){
    HTTPUpload& upload = server.upload();
    if(upload.status == UPLOAD_FILE_START){
      Serial.setDebugOutput(true);
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
      Serial.setDebugOutput(false);
    }
    yield();
  });

#if !defined(NOLOGO)
  server.on ( "/logo.png", []() {
    server.send_P ( 200, "image/png", Logo, sizeof(Logo) );
  } );
#endif

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
