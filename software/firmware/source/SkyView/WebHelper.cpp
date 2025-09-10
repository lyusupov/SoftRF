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

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#include <TimeLib.h>

#include "SoCHelper.h"

#include "WebHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "BatteryHelper.h"
#include "GDL90Helper.h"

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
<tr><th align=left>Ivan Grokhotkov</th><td align=left>Arduino core for ESP8266</td></tr>\
<tr><th align=left>Paul Stoffregen</th><td align=left>Arduino Time library</td></tr>\
<tr><th align=left>Mikal Hart</th><td align=left>TinyGPS++ and PString libraries</td></tr>\
<tr><th align=left>Hristo Gochkov</th><td align=left>Arduino core for ESP32</td></tr>\
<tr><th align=left>JS Foundation</th><td align=left>jQuery library</td></tr>\
<tr><th align=left>Mike McCauley</th><td align=left>BCM2835 C library</td></tr>\
<tr><th align=left>Jean-Marc Zingg</th><td align=left>GxEPD2 library</td></tr>\
<tr><th align=left>Adafruit Industries</th><td align=left>SPIFlash, SSD1306, INA219 and GFX libraries</td></tr>\
<tr><th align=left>Ryan David</th><td align=left>GDL90 decoder</td></tr>\
<tr><th align=left>Arundale Ramanathan</th><td align=left>Sqlite3 Arduino library</td></tr>\
<tr><th align=left>FlarmNet<br>GliderNet</th><td align=left>aircrafts data</td></tr>\
<tr><th align=left>Shenzhen Xin Yuan<br>(LilyGO) ET company</th><td align=left>TTGO T5S boards</td></tr>\
<tr><th align=left>Tuan Nha</th><td align=left>ESP32 I2S WAV player</td></tr>\
<tr><th align=left>Brian Park</th><td align=left>AceButton library</td></tr>\
<tr><th align=left>flashrom.org project</th><td align=left>Flashrom library</td></tr>\
<tr><th align=left>Earle Philhower</th><td align=left>Arduino Core for Raspberry Pi RP2040, ESP8266Audio library</td></tr>\
<tr><th align=left>Bill Greiman</th><td align=left>SdFat library</td></tr>\
<tr><th align=left>Ioulianos Kakoulidis</th><td align=left>Arduino uCDB library</td></tr>\
<tr><th align=left>sekigon-gonnoc</th><td align=left>Pico PIO USB library</td></tr>\
</table>\
<hr>\
Copyright (C) 2019-2025 &nbsp;&nbsp;&nbsp; Linar Yusupov\
</body>\
</html>";

char *Root_content() {

  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  float vdd = Battery_voltage() ;
  bool low_voltage = (Battery_voltage() <= Battery_threshold());

  time_t timestamp = now();
  char str_Vcc[8];

  size_t size = 2300;
  char *offset;
  size_t len = 0;

  char *Root_temp = (char *) malloc(size);
  if (Root_temp == NULL) {
    return Root_temp;
  }
  offset = Root_temp;

  dtostrf(vdd, 4, 2, str_Vcc);

  snprintf_P ( offset, size,
    PSTR("<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>SkyView status</title>\
  </head>\
<body>\
 <table width=100%%>\
  <tr><!-- <td align=left><h1>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</h1></td> -->\
  <td align=center><h1>SkyView status</h1></td>\
  <!-- <td align=right><img src='/logo.png'></td> --></tr>\
 </table>\
 <table width=100%%>\
  <tr><th align=left>Device Id</th><td align=right>%06X</td></tr>\
  <tr><th align=left>Software Version</th><td align=right>%s&nbsp;&nbsp;%s</td></tr>\
  <tr><th align=left>Uptime</th><td align=right>%02d:%02d:%02d</td></tr>"
#if !defined(RASPBERRY_PI) && !defined(LUCKFOX_LYRA)
 "<tr><th align=left>Free memory</th><td align=right>%u</td></tr>"
#endif /* RASPBERRY_PI */
 "<tr><th align=left>Battery voltage</th><td align=right><font color=%s>%s</font></td></tr>\
  <tr><th align=left>&nbsp;</th><td align=right>&nbsp;</td></tr>\
  <tr><th align=left>Display</th><td align=right>%s</td></tr>\
  <tr><th align=left>Connection type</th><td align=right>%s</td></tr>"),
    SoC->getChipId() & 0xFFFFFF, SKYVIEW_FIRMWARE_VERSION,
    (SoC == NULL ? "NONE" : SoC->name),
    hr, min % 60, sec % 60,
#if !defined(RASPBERRY_PI) && !defined(LUCKFOX_LYRA)
    SoC->getFreeHeap(),
#endif /* RASPBERRY_PI */
    low_voltage ? "red" : "green", str_Vcc,
    hw_info.display      == DISPLAY_EPD_2_7   ||
    hw_info.display      == DISPLAY_EPD_4_7   ? "e-Paper" :
    hw_info.display      == DISPLAY_OLED_2_4  ? "OLED" : "NONE",
    settings->connection == CON_SERIAL_MAIN   ? "Serial" :
    settings->connection == CON_BLUETOOTH_SPP ? "Bluetooth SPP" :
    settings->connection == CON_BLUETOOTH_LE  ? "Bluetooth LE" :
    settings->connection == CON_USB           ? "USB" :
    settings->connection == CON_WIFI_UDP      ? "WiFi" : "NONE"
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  switch (settings->connection)
  {
#if !defined(EXCLUDE_WIFI)
  case CON_WIFI_UDP:
    snprintf_P ( offset, size,
      PSTR("\
  <tr><th align=left>Link partner</th><td align=right>%s</td></tr>\
  <tr><th align=left>Link status</th><td align=right>%s established</td></tr>\
  <tr><th align=left>Assigned IP address</th><td align=right>%s</td></tr>"),
      settings->server && strlen(settings->server) > 0 ? settings->server : "NOT SET",
      WiFi.status() == WL_CONNECTED ? "" : "not",
      WiFi.localIP().toString().c_str()
    );
    len = strlen(offset);
    offset += len;
    size -= len;
#endif /* EXCLUDE_WIFI */
  case CON_SERIAL_MAIN:
  case CON_SERIAL_AUX:
  case CON_BLUETOOTH_SPP:
  case CON_BLUETOOTH_LE:
  case CON_USB:
    switch (settings->protocol)
    {
    case PROTOCOL_GDL90:
      snprintf_P ( offset, size,
        PSTR("\
  <tr><th align=left>Connection status</th><td align=right>%s connected</td></tr>\
  <tr><th align=left>Data type</th><td align=right>%s %s</td></tr>\
  "),
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
  "),
        NMEA_isConnected() ? "" : "not",
        NMEA_isConnected() && !(NMEA_hasGNSS() || NMEA_hasFLARM()) ? "UNK" : "",
        NMEA_hasGNSS()     ? "GNSS"  : "",
        NMEA_hasFLARM()    ? "FLARM" : ""
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
    <td align=center><input type=button onClick=\"location.href='/about'\" value='About'></td>"));
  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 5 */
  if (SoC->id != SOC_RP2040     && SoC->id != SOC_RP2350_RISC &&
      SoC->id != SOC_RP2350_ARM && SoC->id != SOC_RPi         &&
      SoC->id != SOC_RK3506) {
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

  size_t size = 4980;
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
<table width=100%%>"));

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 1 */
  if (SoC->id == SOC_ESP32   || SoC->id == SOC_ESP32S2 ||
      SoC->id == SOC_ESP32C3 || SoC->id == SOC_ESP32C5 ||
      SoC->id == SOC_ESP32C6) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Display adapter</th>\
<td align=right>\
<select name='adapter'>\
<option %s value='%d'>e-Paper TTGO T5S</option>\
<!-- <option %s value='%d'>e-Paper TTGO T5 4.7</option> -->\
<option %s value='%d'>e-Paper Waveshare ESP32</option>\
<!-- <option %s value='%d'>OLED</option> -->\
</select>\
</td>\
</tr>"),
    (settings->adapter == ADAPTER_TTGO_T5S        ? "selected" : ""), ADAPTER_TTGO_T5S,
    (settings->adapter == ADAPTER_TTGO_T5_4_7     ? "selected" : ""), ADAPTER_TTGO_T5_4_7,
    (settings->adapter == ADAPTER_WAVESHARE_ESP32 ? "selected" : ""), ADAPTER_WAVESHARE_ESP32,
    (settings->adapter == ADAPTER_OLED            ? "selected" : ""), ADAPTER_OLED
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
    (settings->adapter == ADAPTER_NODEMCU           ? "selected" : ""), ADAPTER_NODEMCU,
    (settings->adapter == ADAPTER_WAVESHARE_ESP8266 ? "selected" : ""), ADAPTER_WAVESHARE_ESP8266
    );
  } else if (SoC->id == SOC_RP2040      || SoC->id == SOC_RP2350_ARM ||
             SoC->id == SOC_RP2350_RISC || SoC->id == SOC_ESP32S3    ||
             SoC->id == SOC_RK3506) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Display adapter</th>\
<td align=right>\
<select name='adapter'>\
<option %s value='%d'>e-Paper Waveshare Pico</option>\
<option %s value='%d'>e-Paper Waveshare Pico V2</option>\
</select>\
</td>\
</tr>"),
    (settings->adapter == ADAPTER_WAVESHARE_PICO_2_7    ? "selected" : ""), ADAPTER_WAVESHARE_PICO_2_7,
    (settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2 ? "selected" : ""), ADAPTER_WAVESHARE_PICO_2_7_V2
    );
  } else if (SoC->id == SOC_RPi || SoC->id == SOC_ESP32P4) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Display adapter</th>\
<td align=right>\
<select name='adapter'>\
<option %s value='%d'>e-Paper Waveshare HAT 2.7</option>\
<option %s value='%d'>e-Paper Waveshare HAT 2.7 V2</option>\
</select>\
</td>\
</tr>"),
    (settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7    ? "selected" : ""), ADAPTER_WAVESHARE_PI_HAT_2_7,
    (settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7_V2 ? "selected" : ""), ADAPTER_WAVESHARE_PI_HAT_2_7_V2
    );
  }

  len = strlen(offset);
  offset += len;
  size -= len;

  /* Common part 2 */
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
<th align=left>Connection type</th>\
<td align=right>\
<select name='connection'>\
<option %s value='%d'>Serial</option>"),
  (settings->connection == CON_SERIAL_MAIN   ? "selected" : ""), CON_SERIAL_MAIN
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 2 */
  if (SoC->id != SOC_RP2040      &&
      SoC->id != SOC_RP2350_RISC &&
      SoC->id != SOC_RP2350_ARM /* || WiFi.getMode() != WIFI_AP */) {
    snprintf_P ( offset, size, PSTR("<option %s value='%d'>WiFi UDP</option>"),
      (settings->connection == CON_WIFI_UDP ? "selected" : ""), CON_WIFI_UDP);
    len = strlen(offset);
    offset += len;
    size -= len;
  }

  /* SoC specific part 3 */
  if (SoC->id == SOC_ESP32       || SoC->id == SOC_RP2040 ||
      SoC->id == SOC_RP2350_RISC || SoC->id == SOC_RP2350_ARM) {
    snprintf_P ( offset, size, PSTR("<option %s value='%d'>Bluetooth SPP</option>"),
    (settings->connection == CON_BLUETOOTH_SPP ? "selected" : ""), CON_BLUETOOTH_SPP);
    len = strlen(offset);
    offset += len;
    size -= len;
  }

  /* SoC specific part 4 */
  if (SoC->id == SOC_ESP32       || SoC->id == SOC_ESP32S3     ||
      SoC->id == SOC_ESP32C3     || SoC->id == SOC_ESP32C5     ||
      SoC->id == SOC_ESP32C6     || SoC->id == SOC_ESP32P4     ||
      SoC->id == SOC_RP2040      || SoC->id == SOC_RP2350_RISC ||
      SoC->id == SOC_RP2350_ARM) {
    snprintf_P ( offset, size, PSTR("<option %s value='%d'>Bluetooth LE</option>"),
    (settings->connection == CON_BLUETOOTH_LE  ? "selected" : ""), CON_BLUETOOTH_LE);
    len = strlen(offset);
    offset += len;
    size -= len;
  }

  /* SoC specific part 5 */
  if (SoC->id == SOC_RP2040      ||
      SoC->id == SOC_RP2350_RISC ||
      SoC->id == SOC_RP2350_ARM) {
    snprintf_P ( offset, size, PSTR("<option %s value='%d'>USB</option>"),
      (settings->connection == CON_USB ? "selected" : ""), CON_USB);
    len = strlen(offset);
    offset += len;
    size -= len;
  }

  /* Common part 3 */
  snprintf_P ( offset, size,
    PSTR("\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Protocol</th>\
<td align=right>\
<select name='protocol'>\
<option %s value='%d'>NMEA</option>\
<option %s value='%d'>GDL90</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Baud rate</th>\
<td align=right>\
<select name='baudrate'>\
<option %s value='%d'>4800</option>\
<option %s value='%d'>9600</option>\
<option %s value='%d'>19200</option>\
<option %s value='%d'>38400</option>\
<option %s value='%d'>57600</option>"),
  (settings->protocol   == PROTOCOL_NMEA     ? "selected" : ""), PROTOCOL_NMEA,
  (settings->protocol   == PROTOCOL_GDL90    ? "selected" : ""), PROTOCOL_GDL90,
  (settings->baudrate   == B4800             ? "selected" : ""), B4800,
  (settings->baudrate   == B9600             ? "selected" : ""), B9600,
  (settings->baudrate   == B19200            ? "selected" : ""), B19200,
  (settings->baudrate   == B38400            ? "selected" : ""), B38400,
  (settings->baudrate   == B57600            ? "selected" : ""), B57600
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 6 */
  if (SoC->id == SOC_ESP32       || SoC->id == SOC_ESP32S2 ||
      SoC->id == SOC_ESP32S3     || SoC->id == SOC_ESP32C3 ||
      SoC->id == SOC_ESP32C5     || SoC->id == SOC_ESP32C6 ||
      SoC->id == SOC_RPi         || SoC->id == SOC_RK3506  ||
      SoC->id == SOC_ESP32P4     || SoC->id == SOC_RP2040  ||
      SoC->id == SOC_RP2350_RISC || SoC->id == SOC_RP2350_ARM) {
    snprintf_P ( offset, size,
      PSTR("\
<option %s value='%d'>115200</option>\
<option %s value='%d'>2000000</option>"),
    (settings->baudrate   == B115200        ? "selected" : ""), B115200,
    (settings->baudrate   == B2000000       ? "selected" : ""), B2000000
    );
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
<th align=left>Source Id</th>\
<td align=right>\
<INPUT type='text' name='server' maxlength='17' size='17' value='%s'>\
</td>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
&nbsp;&nbsp;Key</th>\
<td align=right>\
<INPUT type='text' name='key' maxlength='17' size='17' value='%s'>\
</td>\
</tr>\
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
<th align=left>Screen rotation</th>\
<td align=right>\
<select name='rotation'>\
<option %s value='%d'>0</option>\
<option %s value='%d'>180</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>View mode</th>\
<td align=right>\
<select name='vmode'>\
<option %s value='%d'>radar</option>\
<option %s value='%d'>text</option>\
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
<option %s value='%d'>off</option>\
<!-- <option %s value='%d'>auto</option> -->\
<option %s value='%d'>FlarmNet</option>\
<option %s value='%d'>GliderNet</option>"),
  settings->server, settings->key,
  (settings->units == UNITS_METRIC    ? "selected" : ""), UNITS_METRIC,
  (settings->units == UNITS_IMPERIAL  ? "selected" : ""), UNITS_IMPERIAL,
  (settings->units == UNITS_MIXED     ? "selected" : ""), UNITS_MIXED,
  (settings->rotate == ROTATE_0       ? "selected" : ""), ROTATE_0,
  (settings->rotate == ROTATE_180     ? "selected" : ""), ROTATE_180,
  (settings->vmode == VIEW_MODE_RADAR ? "selected" : ""), VIEW_MODE_RADAR,
  (settings->vmode == VIEW_MODE_TEXT  ? "selected" : ""), VIEW_MODE_TEXT,
  (settings->orientation == DIRECTION_TRACK_UP ? "selected" : ""), DIRECTION_TRACK_UP,
  (settings->orientation == DIRECTION_NORTH_UP ? "selected" : ""), DIRECTION_NORTH_UP,
  (settings->zoom == ZOOM_LOWEST ? "selected" : ""), ZOOM_LOWEST,
  (settings->zoom == ZOOM_LOW    ? "selected" : ""), ZOOM_LOW,
  (settings->zoom == ZOOM_MEDIUM ? "selected" : ""), ZOOM_MEDIUM,
  (settings->zoom == ZOOM_HIGH   ? "selected" : ""), ZOOM_HIGH,
  (settings->adb == DB_NONE      ? "selected" : ""), DB_NONE,
  (settings->adb == DB_AUTO      ? "selected" : ""), DB_AUTO,
  (settings->adb == DB_FLN       ? "selected" : ""), DB_FLN,
  (settings->adb == DB_OGN       ? "selected" : ""), DB_OGN
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 7 */
  if (SoC->id == SOC_ESP32) {
    snprintf_P ( offset, size,
      PSTR("<option %s value='%d'>ICAO</option>"),
      (settings->adb == DB_ICAO  ? "selected" : ""), DB_ICAO);

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
<th align=left>ID preference</th>\
<td align=right>\
<select name='idpref'>\
<option %s value='%d'>registration</option>\
<option %s value='%d'>tail/CN</option>\
<option %s value='%d'>make & model</option>\
<option %s value='%d'>class</option>\
</select>\
</td>\
</tr>"),


  (settings->idpref == ID_REG    ? "selected" : ""), ID_REG,
  (settings->idpref == ID_TAIL   ? "selected" : ""), ID_TAIL,
  (settings->idpref == ID_MAM    ? "selected" : ""), ID_MAM,
  (settings->idpref == ID_TYPE   ? "selected" : ""), ID_TYPE
  );

  len = strlen(offset);
  offset += len;
  size -= len;

#if !defined(EXCLUDE_AUDIO)
  /* SoC specific part 7 */
  if (SoC->id == SOC_ESP32   || SoC->id == SOC_ESP32S2 ||
      SoC->id == SOC_ESP32C3 || SoC->id == SOC_ESP32C5 ||
      SoC->id == SOC_ESP32C6 || SoC->id == SOC_ESP32P4) {
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
    (settings->voice == VOICE_OFF  ? "selected" : ""), VOICE_OFF,
    (settings->voice == VOICE_1    ? "selected" : ""), VOICE_1,
    (settings->voice == VOICE_2    ? "selected" : ""), VOICE_2,
    (settings->voice == VOICE_3    ? "selected" : ""), VOICE_3
    );

    len = strlen(offset);
    offset += len;
    size -= len;
  } else if (SoC->id == SOC_ESP32S3     || SoC->id == SOC_RP2040 ||
             SoC->id == SOC_RP2350_RISC || SoC->id == SOC_RP2350_ARM) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Voice</th>\
<td align=right>\
<select name='voice'>\
<option %s value='%d'>off</option>\
<option %s value='%d'>voice 1</option>\
</select>\
</td>\
</tr>"),
    (settings->voice == VOICE_OFF  ? "selected" : ""), VOICE_OFF,
    (settings->voice == VOICE_1    ? "selected" : ""), VOICE_1
    );

    len = strlen(offset);
    offset += len;
    size -= len;
  }
#endif /* EXCLUDE_AUDIO */

  /* Common part 6 */
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
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
</tr>\
<tr>\
<th align=left>Traffic filter</th>\
<td align=right>\
<select name='filter'>\
<option %s value='%d'>off</option>\
<option %s value='%d'>by Altitude (&#177; 500 m)</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Power save</th>\
<td align=right>\
<select name='power_save'>\
<option %s value='%d'>Disabled</option>\
<option %s value='%d'>WiFi OFF (10 min.)</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Team Member Id</th>\
<td align=right>\
<INPUT type='text' name='team' maxlength='6' size='6' value='%06X'>\
</td>\
</tr>\
</table>\
<p align=center><INPUT type='submit' value='Save and restart'></p>\
</form>\
</body>\
</html>"),
    (settings->aghost     == ANTI_GHOSTING_OFF   ? "selected" : ""), ANTI_GHOSTING_OFF,
    (settings->aghost     == ANTI_GHOSTING_AUTO  ? "selected" : ""), ANTI_GHOSTING_AUTO,
    (settings->aghost     == ANTI_GHOSTING_2MIN  ? "selected" : ""), ANTI_GHOSTING_2MIN,
    (settings->aghost     == ANTI_GHOSTING_5MIN  ? "selected" : ""), ANTI_GHOSTING_5MIN,
    (settings->aghost     == ANTI_GHOSTING_10MIN ? "selected" : ""), ANTI_GHOSTING_10MIN,
    (settings->filter     == TRAFFIC_FILTER_OFF  ? "selected" : ""), TRAFFIC_FILTER_OFF,
    (settings->filter     == TRAFFIC_FILTER_500M ? "selected" : ""), TRAFFIC_FILTER_500M,
    (settings->power_save == POWER_SAVE_NONE     ? "selected" : ""), POWER_SAVE_NONE,
    (settings->power_save == POWER_SAVE_WIFI     ? "selected" : ""), POWER_SAVE_WIFI,
     settings->team
  );

  return Settings_temp;
}

char *Input_content() {

  char *Input_temp = (char *) malloc(1970);
  if (Input_temp == NULL) {
    return Input_temp;
  }

  snprintf_P ( Input_temp, 2000,
PSTR("<html>\
<head>\
<meta http-equiv='refresh' content='15; url=/'>\
<meta name='viewport' content='width=device-width, initial-scale=1'>\
<title>SkyView Settings</title>\
</head>\
<body>\
<h1 align=center>New settings:</h1>\
<table width=100%%>\
<tr><th align=left>Adapter</th><td align=right>%d</td></tr>\
<tr><th align=left>Connection</th><td align=right>%d</td></tr>\
<tr><th align=left>Protocol</th><td align=right>%d</td></tr>\
<tr><th align=left>Baud rate</th><td align=right>%d</td></tr>\
<tr><th align=left>Server</th><td align=right>%s</td></tr>\
<tr><th align=left>Key</th><td align=right>%s</td></tr>\
<tr><th align=left>Units</th><td align=right>%d</td></tr>\
<tr><th align=left>Screen rotation</th><td align=right>%d</td></tr>\
<tr><th align=left>View mode</th><td align=right>%d</td></tr>\
<tr><th align=left>Radar orientation</th><td align=right>%d</td></tr>\
<tr><th align=left>Zoom level</th><td align=right>%d</td></tr>\
<tr><th align=left>Aircrafts data</th><td align=right>%d</td></tr>\
<tr><th align=left>ID preference</th><td align=right>%d</td></tr>\
<tr><th align=left>Voice</th><td align=right>%d</td></tr>\
<tr><th align=left>'Ghosts' removal</th><td align=right>%d</td></tr>\
<tr><th align=left>Filter</th><td align=right>%d</td></tr>\
<tr><th align=left>Power Save</th><td align=right>%d</td></tr>\
<tr><th align=left>Team</th><td align=right>%06X</td></tr>\
</table>\
<hr>\
  <p align=center><h1 align=center>Restart is in progress... Please, wait!</h1></p>\
</body>\
</html>"),
  settings->adapter, settings->connection, settings->protocol,
  settings->baudrate, settings->server, settings->key,
  settings->units, settings->rotate, settings->vmode, settings->orientation,
  settings->zoom, settings->adb, settings->idpref, settings->voice,
  settings->aghost, settings->filter, settings->power_save, settings->team
  );

  return Input_temp;
}

#if defined(EXCLUDE_WIFI)
void Web_setup()    {}
void Web_loop()     {}
void Web_fini()     {}
#else

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

void handleSettings() {
  char *settings = Settings_content();

  if (settings) {
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
    server.sendHeader(String(F("Pragma")), String(F("no-cache")));
    server.sendHeader(String(F("Expires")), String(F("-1")));
    server.send ( 200, "text/html", settings );
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
    if (server.argName(i).equals("adapter")) {
      settings->adapter = server.arg(i).toInt();
    } else if (server.argName(i).equals("connection")) {
      settings->connection = server.arg(i).toInt();
    } else if (server.argName(i).equals("protocol")) {
      settings->protocol = server.arg(i).toInt();
    } else if (server.argName(i).equals("baudrate")) {
      settings->baudrate = server.arg(i).toInt();
    } else if (server.argName(i).equals("server")) {
      server.arg(i).toCharArray(settings->server, sizeof(settings->server));
    } else if (server.argName(i).equals("key")) {
      server.arg(i).toCharArray(settings->key, sizeof(settings->key));
    } else if (server.argName(i).equals("units")) {
      settings->units = server.arg(i).toInt();
    } else if (server.argName(i).equals("rotation")) {
      settings->rotate = server.arg(i).toInt();
    } else if (server.argName(i).equals("vmode")) {
      settings->vmode = server.arg(i).toInt();
    } else if (server.argName(i).equals("orientation")) {
      settings->orientation = server.arg(i).toInt();
    } else if (server.argName(i).equals("zoom")) {
      settings->zoom = server.arg(i).toInt();
    } else if (server.argName(i).equals("adb")) {
      settings->adb = server.arg(i).toInt();
    } else if (server.argName(i).equals("idpref")) {
      settings->idpref = server.arg(i).toInt();
    } else if (server.argName(i).equals("voice")) {
      settings->voice = server.arg(i).toInt();
    } else if (server.argName(i).equals("aghost")) {
      settings->aghost = server.arg(i).toInt();
    } else if (server.argName(i).equals("filter")) {
      settings->filter = server.arg(i).toInt();
    } else if (server.argName(i).equals("power_save")) {
      settings->power_save = server.arg(i).toInt();
    } else if (server.argName(i).equals("team")) {
      char buf[7];
      server.arg(i).toCharArray(buf, sizeof(buf));
      settings->team = strtoul(buf, NULL, 16);
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
    delay(1000);
    SoC->reset();
  }
}

void handleNotFound() {

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
  server.onNotFound ( handleNotFound );

  server.on("/update", HTTP_POST, [](){
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Connection")), String(F("close")));
    server.sendHeader(String(F("Access-Control-Allow-Origin")), "*");
    server.send(200, String(F("text/plain")), (Update.hasError())?"FAIL":"OK");
//    SoC->swSer_enableRx(true);
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

#if !defined(NOLOGO)
  server.on ( "/logo.png", []() {
    server.send_P ( 200, "image/png", Logo, sizeof(Logo) );
  } );
#endif

/* FLASH memory usage optimization */
#if !defined(ARDUINO_ARCH_RP2040)
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
#endif /* ARDUINO_ARCH_RP2040 */

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
