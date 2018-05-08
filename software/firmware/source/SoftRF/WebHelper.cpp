/*
 * WebHelper.cpp
 * Copyright (C) 2016-2018 Linar Yusupov
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

#include "BatteryHelper.h"
#include "RFHelper.h"
#include "SoCHelper.h"
#include "WebHelper.h"
#include "BaroHelper.h"
#include "LEDHelper.h"
#include "SoundHelper.h"
#include "BluetoothHelper.h"

static uint32_t prev_rx_pkt_cnt = 0;

byte getVal(char c)
{
   if(c >= '0' && c <= '9')
     return (byte)(c - '0');
   else
     return (byte)(toupper(c)-'A'+10);
}

void Hex2Bin(String str, byte *buffer)
{
  char hexdata[2 * PKT_SIZE + 1];
  
  str.toCharArray(hexdata, sizeof(hexdata));
  for(int j = 0; j < PKT_SIZE * 2 ; j+=2)
  {
    buffer[j>>1] = getVal(hexdata[j+1]) + (getVal(hexdata[j]) << 4);
  }
}

String Bin2Hex(byte *buffer)
{
  String str = "";
  for (int i=0; i < PKT_SIZE; i++) {
    byte c = buffer[i];
    str += (c < 0x10 ? "0" : "") + String(c, HEX);
  }
  return str;
}

void handleSettings() {

  size_t size = 3600;
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
<th align=left>Mode</th>\
<td align=right>\
<select name='mode'>\
<option %s value='%d'>Normal</option>\
<option %s value='%d'>Tx/Rx Test</option>\
<option %s value='%d'>Bridge</option>\
<option %s value='%d'>UAV</option>\
</select>\
</td>\
</tr>"),
  (settings->mode == SOFTRF_MODE_NORMAL ? "selected" : "") , SOFTRF_MODE_NORMAL,
  (settings->mode == SOFTRF_MODE_TXRX_TEST ? "selected" : ""), SOFTRF_MODE_TXRX_TEST,
  (settings->mode == SOFTRF_MODE_BRIDGE ? "selected" : ""), SOFTRF_MODE_BRIDGE,
  (settings->mode == SOFTRF_MODE_UAV ? "selected" : ""), SOFTRF_MODE_UAV
/*  (settings->mode == SOFTRF_MODE_WATCHOUT ? "selected" : ""), SOFTRF_MODE_WATCHOUT, */
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  /* Radio specific part */
  if (rf_chip && (rf_chip->type == RF_IC_SX1276)) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Protocol</th>\
<td align=right>\
<select name='protocol'>\
<option %s value='%d'>Legacy</option>\
<option %s value='%d'>OGNTP</option>\
<option %s value='%d'>P3I</option>\
<option %s value='%d'>FANET</option>\
</select>\
</td>\
</tr>"),
    (settings->rf_protocol == RF_PROTOCOL_LEGACY ? "selected" : "") , RF_PROTOCOL_LEGACY,
    (settings->rf_protocol == RF_PROTOCOL_OGNTP ? "selected" : ""), RF_PROTOCOL_OGNTP,
    (settings->rf_protocol == RF_PROTOCOL_P3I ? "selected" : ""), RF_PROTOCOL_P3I,
    (settings->rf_protocol == RF_PROTOCOL_FANET ? "selected" : ""), RF_PROTOCOL_FANET
    );

    len = strlen(offset);
    offset += len;
    size -= len;
  }

  /* Common part 2 */
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
<th align=left>Region</th>\
<td align=right>\
<select name='band'>\
<option %s value='%d'>AUTO</option>\
<option %s value='%d'>EU (868.4 MHz)</option>\
<option %s value='%d'>RU (868.8 MHz)</option>\
<option %s value='%d'>CN (433 MHz)</option>\
<option %s value='%d'>US (915 MHz)</option>\
<option %s value='%d'>NZ (869.25 MHz)</option>\
<option %s value='%d'>UK (869.52 MHz)</option>\
<option %s value='%d'>AU (921 MHz)</option>\
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
<tr>\
<th align=left>LED ring:</th>\
</tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Direction</th>\
<td align=right>\
<select name='pointer'>\
<option %s value='%d'>CoG Up</option>\
<option %s value='%d'>North Up</option>\
<option %s value='%d'>Off</option>\
</select>\
</td>\
</tr>"),
  (settings->band == RF_BAND_AUTO ? "selected" : ""), RF_BAND_AUTO,
  (settings->band == RF_BAND_EU ? "selected" : ""), RF_BAND_EU,
  (settings->band == RF_BAND_RU ? "selected" : ""), RF_BAND_RU,
  (settings->band == RF_BAND_CN ? "selected" : ""), RF_BAND_CN,
  (settings->band == RF_BAND_US ? "selected" : ""),  RF_BAND_US,
  (settings->band == RF_BAND_NZ ? "selected" : ""), RF_BAND_NZ,
  (settings->band == RF_BAND_UK ? "selected" : ""), RF_BAND_UK,
  (settings->band == RF_BAND_AU ? "selected" : ""),  RF_BAND_AU,
  (settings->aircraft_type == AIRCRAFT_TYPE_GLIDER ? "selected" : ""),  AIRCRAFT_TYPE_GLIDER,
  (settings->aircraft_type == AIRCRAFT_TYPE_TOWPLANE ? "selected" : ""),  AIRCRAFT_TYPE_TOWPLANE,
  (settings->aircraft_type == AIRCRAFT_TYPE_POWERED ? "selected" : ""),  AIRCRAFT_TYPE_POWERED,
  (settings->aircraft_type == AIRCRAFT_TYPE_HELICOPTER ? "selected" : ""),  AIRCRAFT_TYPE_HELICOPTER,
  (settings->aircraft_type == AIRCRAFT_TYPE_UAV ? "selected" : ""),  AIRCRAFT_TYPE_UAV,
  (settings->aircraft_type == AIRCRAFT_TYPE_HANGGLIDER ? "selected" : ""),  AIRCRAFT_TYPE_HANGGLIDER,
  (settings->aircraft_type == AIRCRAFT_TYPE_PARAGLIDER ? "selected" : ""),  AIRCRAFT_TYPE_PARAGLIDER,
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

  /* SoC specific part */
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
    (settings->bluetooth == BLUETOOTH_OFF ? "selected" : ""), BLUETOOTH_OFF,
    (settings->bluetooth == BLUETOOTH_SPP ? "selected" : ""), BLUETOOTH_SPP,
    (settings->bluetooth == BLUETOOTH_LE_HM10_SERIAL ? "selected" : ""), BLUETOOTH_LE_HM10_SERIAL
    );

    len = strlen(offset);
    offset += len;
    size -= len;
  }

  /* Common part 3 */
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
<th align=left>NMEA:</th>\
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
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;UDP</th>\
<td align=right>\
<input type='radio' name='nmea_u' value='0' %s>Off\
<input type='radio' name='nmea_u' value='1' %s>On\
</td>\
</tr>\
<tr>\
<th align=left>GDL90</th>\
<td align=right>\
<input type='radio' name='gdl90' value='0' %s>Off\
<input type='radio' name='gdl90' value='1' %s>On\
</td>\
</tr>\
<tr>\
<th align=left>Dump1090</th>\
<td align=right>\
<input type='radio' name='d1090' value='0' %s>Off\
<input type='radio' name='d1090' value='1' %s>On\
</td>\
</tr>\
</table>\
<p align=center><INPUT type='submit' value='Save and restart'><p>\
</form>\
</body>\
</html>"),
  (!settings->nmea_g ? "checked" : "") , (settings->nmea_g ? "checked" : ""),
  (!settings->nmea_p ? "checked" : "") , (settings->nmea_p ? "checked" : ""),
  (!settings->nmea_l ? "checked" : "") , (settings->nmea_l ? "checked" : ""),
  (!settings->nmea_u ? "checked" : "") , (settings->nmea_u ? "checked" : ""),
  (!settings->gdl90 ? "checked" : "") , (settings->gdl90 ? "checked" : ""),
  (!settings->d1090 ? "checked" : "") , (settings->d1090 ? "checked" : "")
  );

  SoC->swSer_enableRx(false);
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.send ( 200, "text/html", Settings_temp );
  SoC->swSer_enableRx(true);
  free(Settings_temp);
}

void handleRoot() {
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  float vdd = Battery_voltage() ;

  time_t timestamp = ThisAircraft.timestamp;
  unsigned int sats = gnss.satellites.value(); // Number of satellites in use (u32)
  char str_lat[16];
  char str_lon[16];
  char str_alt[16];
  char str_Vcc[8];

  char *Root_temp = (char *) malloc(2048);
  if (Root_temp == NULL) {
    return;
  }

  dtostrf(ThisAircraft.latitude, 8, 4, str_lat);
  dtostrf(ThisAircraft.longitude, 8, 4, str_lon);
  dtostrf(ThisAircraft.altitude, 7, 1, str_alt);
  dtostrf(vdd, 4, 2, str_Vcc);

  snprintf_P ( Root_temp, 2048,
    PSTR("<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>SoftRF status</title>\
  </head>\
<body>\
 <h1 align=center>SoftRF status</h1>\
 <table width=100%%>\
  <tr><th align=left>Device Id</th><td align=right>%X</td></tr>\
  <tr><th align=left>Software Version</th><td align=right>%s&nbsp;&nbsp;%s</td></tr>\
  <tr><th align=left>Radio</th><td align=right>%s</td></tr>\
  <tr><th align=left>Baro</th><td align=right>%s</td></tr>\
  <tr><th align=left>Uptime</th><td align=right>%02d:%02d:%02d</td></tr>\
  <tr><th align=left>Battery voltage</th><td align=right>%s</td></tr>\
  <tr><th align=left>Packets:</th></tr>\
  <tr><th align=left>&nbsp;&nbsp;&nbsp;&nbsp;- transmitted</th><td align=right>%u</td></tr>\
  <tr><th align=left>&nbsp;&nbsp;&nbsp;&nbsp;- received</th><td align=right>%u</td></tr>\
 </table>\
 <h2 align=center>Most recent GNSS fix</h2>\
 <table width=100%%>\
  <tr><th align=left>Time</th><td align=right>%u</td></tr>\
  <tr><th align=left>Sattelites</th><td align=right>%d</td></tr>\
  <tr><th align=left>Latitude</th><td align=right>%s</td></tr>\
  <tr><th align=left>Longitude</th><td align=right>%s</td></tr>\
  <tr><th align=left>Altitude</th><td align=right>%s</td></tr>\
 </table>\
 <hr>\
 <table width=100%%>\
  <tr>\
    <td align=left><input type=button onClick=\"location.href='/settings'\" value='Settings'></td>\
    <td align=right><input type=button onClick=\"location.href='/firmware'\" value='Firmware update'></td>\
  </tr>\
 </table>\
</body>\
</html>"),
    ThisAircraft.addr, SOFTRF_FIRMWARE_VERSION,
    (SoC == NULL ? "NONE" : SoC->name),
    (rf_chip == NULL ? "NONE" : rf_chip->name),
    (baro_chip == NULL ? "NONE" : baro_chip->name),
    hr, min % 60, sec % 60, str_Vcc, tx_packets_counter, rx_packets_counter,
    timestamp, sats, str_lat, str_lon, str_alt
  );
  SoC->swSer_enableRx(false);
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.send ( 200, "text/html", Root_temp );
  SoC->swSer_enableRx(true);
  free(Root_temp);
}

#define BOOL_STR(x) (x ? "true":"false")

void handleInput() {

  char *Input_temp = (char *) malloc(1200);
  if (Input_temp == NULL) {
    return;
  }

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    if (server.argName(i).equals("mode")) {
      settings->mode = server.arg(i).toInt();
    } else if (server.argName(i).equals("protocol")) {
      settings->rf_protocol = server.arg(i).toInt();
    } else if (server.argName(i).equals("band")) {
      settings->band = server.arg(i).toInt();
    } else if (server.argName(i).equals("acft_type")) {
      settings->aircraft_type = server.arg(i).toInt();
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
    } else if (server.argName(i).equals("nmea_u")) {
      settings->nmea_u = server.arg(i).toInt();
    } else if (server.argName(i).equals("gdl90")) {
      settings->gdl90 = server.arg(i).toInt();
    } else if (server.argName(i).equals("d1090")) {
      settings->d1090 = server.arg(i).toInt();
    }
  }
  snprintf_P ( Input_temp, 1200,
PSTR("<html>\
<head>\
<meta http-equiv='refresh' content='15; url=/'/>\
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
<tr><th align=left>Tx Power</th><td align=right>%d</td></tr>\
<tr><th align=left>Volume</th><td align=right>%d</td></tr>\
<tr><th align=left>LED pointer</th><td align=right>%d</td></tr>\
<tr><th align=left>Bluetooth</th><td align=right>%d</td></tr>\
<tr><th align=left>NMEA GNSS</th><td align=right>%s</td></tr>\
<tr><th align=left>NMEA Private</th><td align=right>%s</td></tr>\
<tr><th align=left>NMEA Legacy</th><td align=right>%s</td></tr>\
<tr><th align=left>NMEA UDP</th><td align=right>%s</td></tr>\
<tr><th align=left>GDL90</th><td align=right>%s</td></tr>\
<tr><th align=left>DUMP1090</th><td align=right>%s</td></tr>\
</table>\
<hr>\
  <p align=center><h1 align=center>Restart is in progress... Please, wait!</h1>\<p>\
</body>\
</html>"),
  settings->mode, settings->rf_protocol, settings->band, settings->aircraft_type,
  settings->txpower, settings->volume, settings->pointer, settings->bluetooth,
  BOOL_STR(settings->nmea_g), BOOL_STR(settings->nmea_p),
  BOOL_STR(settings->nmea_l), BOOL_STR(settings->nmea_u),
  BOOL_STR(settings->gdl90), BOOL_STR(settings->d1090)
  );
  SoC->swSer_enableRx(false);
  server.send ( 200, "text/html", Input_temp );
  SoC->swSer_enableRx(true);
  free(Input_temp);
  EEPROM_store();
  delay(3000);
  ESP.restart();
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
  
  server.on ( "/input", handleInput );
  server.on ( "/inline", []() {
    server.send ( 200, "text/plain", "this works as well" );
  } );
  server.on("/firmware", HTTP_GET, [](){
    SoC->swSer_enableRx(false);
    server.sendHeader("Connection", "close");
    server.sendHeader("Access-Control-Allow-Origin", "*");
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
    <form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>\
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
    server.sendHeader("Connection", "close");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", (Update.hasError())?"FAIL":"OK");
    SoC->swSer_enableRx(true);
    ESP.restart();
  },[](){
    HTTPUpload& upload = server.upload();
    if(upload.status == UPLOAD_FILE_START){
      Serial.setDebugOutput(true);
      SoC->WiFiUDP_stopAll();
      Serial.printf("Update: %s\n", upload.filename.c_str());
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
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
      Serial.setDebugOutput(false);
    }
    yield();
  });

  server.begin();
  Serial.println (F("HTTP server has started at port: 80"));

  delay(1000);
}

void Web_loop()
{
  server.handleClient();
}

