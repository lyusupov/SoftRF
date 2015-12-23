
#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include "WebHelper.h"


ESP8266WebServer server ( 80 );

char hexdata[2 * PKT_SIZE + 1] ;
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
  
void handleAlertRoot() {

  char temp[800];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  snprintf ( temp, 800,

"<html>\
  <head>\
    <meta http-equiv='refresh' content='5'>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>SoftRF Demo</title>\
  <style>\
  .center {\
    position: absolute;\
    top: 50\%;\
    left: 50\%;\
    -moz-transform: translateX(-50\%) translateY(-50\%);\
    -webkit-transform: translateX(-50\%) translateY(-50\%);\
    transform: translateX(-50\%) translateY(-50\%);\
  }?\
  </style>\
  </head>\
  <body bgcolor=%s>\
  <h1 align=center>SoftRF</h1>\
  <table width=100\%>\
   <tr><th align=left>Uptime</th><td align=right>%02d:%02d:%02d</td></tr>\
  </table>\
  <div class=\"center\"><h1><font color=white>%s</font></h1></div>\
  </body>\
</html>",

    rx_packets_counter > prev_rx_pkt_cnt ? "red" : "green",
    hr, min % 60, sec % 60,
    rx_packets_counter > prev_rx_pkt_cnt ? "Traffic alert!" : "No traffic detected"
  );
  server.send ( 200, "text/html", temp );
  prev_rx_pkt_cnt = rx_packets_counter;
}

void handleRxRoot() {
  char Root_temp[2048];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  char str_lat[16];
  char str_lon[16];
  char str_dist[16];

  dtostrf(fo.latitude, 8, 4, str_lat);
  dtostrf(fo.longtitude, 8, 4, str_lon);
  dtostrf(fo.distance, 8, 2, str_dist);

  snprintf ( Root_temp, 2048,

"<html>\
  <head>\
    <meta http-equiv='refresh' content='5'>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>SoftRF Demo</title>\
  </head>\
<body>\
 <h1 align=center>SoftRF</h1>\
 <table width=100\%>\
  <tr><th align=left>Uptime</th><td align=right>%02d:%02d:%02d</td></tr>\
  <tr><th align=left>Packets received</th><td align=right>%u</td></tr>\
 </table>\
 <h2 align=center>Most recent packet</h2>\
 <table width=100\%>\
  <tr><th align=left>Time stamp</th><td align=right>%u</td></tr>\
  <tr><th align=left>Raw</th><td align=right><font size=1>%s</font></td></tr>\
 </table>\
 <h2 align=center>Decoded</h2>\
 <table width=100\%>\
  <tr><th align=left>Address</th><td align=right>%X</td></tr>\
  <tr><th align=left>Latitude</th><td align=right>%s</td></tr>\
  <tr><th align=left>Longtitude</th><td align=right>%s</td></tr>\
  <tr><th align=left>Altitude</th><td align=right>%d</td></tr>\
  <tr><th align=left>Distance</th><td align=right>%s</td></tr>\
  <tr><th align=left>Vs</th><td align=right>%d</td></tr>\
  <tr><th align=left>Type</th><td align=right>%u</td></tr>\
  <tr><th align=left>Stealth</th><td align=right>%d</td></tr>\
  <tr><th align=left>No_track</th><td align=right>%d</td></tr>\
 </table>\
</body>\
</html>",
    hr, min % 60, sec % 60, rx_packets_counter,
    fo.timestamp, fo.raw.c_str(),
    fo.addr, str_lat, str_lon, fo.altitude, str_dist,
    fo.vs, fo.type, fo.stealth, fo.no_track
  );
  server.send ( 200, "text/html", Root_temp );

}

void handleTxRoot() {

  char temp[800];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  snprintf ( temp, 800,

"<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>SoftRF Demo</title>\
  </head>\
  <body>\
  <h1 align=center>SoftRF</h1>\
  <table width=100\%>\
   <tr><th align=left>Uptime</th><td align=right>%02d:%02d:%02d</td></tr>\
  </table>\
  <h2 align=center>Enter 24 bytes of hexadecimal data</h2>\
  <p align=center><sub>example: 0282dd204901f981798a85b69764bdf99ed77fd3c2300000</sub></p>\
   <FORM action=\"input\" method=\"GET\">\
    <p align=center><INPUT type=\"text\" name=\"data\" maxlength=\"48\" size=\"48\" value=\"%s\"><p>\
    <p align=center><INPUT type=\"submit\" value=\"TRANSMIT\"><p>\
   </FORM>\
  </body>\
</html>",

    hr, min % 60, sec % 60, hexdata
  );
  server.send ( 200, "text/html", temp );
}

void handleCloudRoot() {
  char Root_temp[2048];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  snprintf ( Root_temp, 2048,

"<html>\
  <head>\
    <meta http-equiv='refresh' content='5'>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>SoftRF Demo</title>\
  </head>\
<body>\
 <h1 align=center>SoftRF</h1>\
 <table width=100\%>\
  <tr><th align=left>Uptime</th><td align=right>%02d:%02d:%02d</td></tr>\
  <tr><th align=left>Packets received</th><td align=right>%u</td></tr>\
  <tr><th align=left>Cloud server</th><td align=right>%s</td></tr>\
  <tr><th align=left>Link status</th><td align=right>%s</td></tr>\
 </table>\
 <h2 align=center>Most recent packet</h2>\
 <table width=100\%>\
  <tr><th align=left>Time stamp</th><td align=right>%u</td></tr>\
  <tr><th align=left>Raw</th><td align=right><font size=1>%s</font></td></tr>\
 </table>\
 <h2 align=center>Message sent to cloud</h2>\
 <p align=center><font size=2>%s</font></p>\
</body>\
</html>",
    hr, min % 60, sec % 60, rx_packets_counter, CLOUD_HOSTNAME,
    client.connected() ? "connected" : "disconnected",
    fo.timestamp, fo.raw.c_str(),
    (strlen(UDPpacketBuffer) > 0 ? UDPpacketBuffer : "NONE")
  );
  server.send ( 200, "text/html", Root_temp );

}

void handleInput() {

  char temp[500];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    if (server.argName(i).equals("data")) {
      switch(txready) {
      case TX_CLEAR:
      case TX_DATA_READY:
          server.arg(i).toCharArray(hexdata, sizeof(hexdata));
          for(int j = 0; j < PKT_SIZE * 2 ; j+=2)
          {
            TxBuffer[j>>1] = getVal(hexdata[j+1]) + (getVal(hexdata[j]) << 4);
          }
          txready = TX_DATA_READY;
          break;
       case TX_SENT:

          txready = TX_CLEAR;
          break;
      default:
          continue;
      }
      snprintf ( temp, 500,
"<html>\
  <head>\
    <meta http-equiv='refresh' content='%s'/>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>SoftRF Demo</title>\
  </head>\
  <body>\
  <h1 align=center>SoftRF</h1>\
  <table width=100\%>\
   <tr><th align=left>Uptime</th><td align=right>%02d:%02d:%02d</td></tr>\
  </table>\
  <h2 align=center>%s</h2>\
  </body>\
</html>",
      ( txready ? "1" : "1; url=/" ), hr, min % 60, sec % 60 , ( txready ? "Transfer started..." : "Transfer completed." )
      );
      server.send ( 200, "text/html", temp );
    }
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
  //server.on ( "/", handleAlertRoot );
  //server.on ( "/", handleRxRoot );
  //server.on ( "/", handleTxRoot );
  server.on ( "/", handleCloudRoot );

  server.on ( "/input", handleInput );
  server.on ( "/inline", []() {
    server.send ( 200, "text/plain", "this works as well" );
  } );
  server.onNotFound ( handleNotFound );

  server.onFileUpload([](){
    if(server.uri() != "/update") return;
    HTTPUpload& upload = server.upload();
    if(upload.status == UPLOAD_FILE_START){
      Serial.setDebugOutput(true);
      WiFiUDP::stopAll();
      Serial.printf("Update: %s\n", upload.filename.c_str());
      uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
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
  server.on("/update", HTTP_POST, [](){
    server.sendHeader("Connection", "close");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", (Update.hasError())?"FAIL":"OK");
    ESP.restart();
  });

  server.begin();
  Serial.println ( "HTTP server started" );

  TxDataTemplate.toCharArray(hexdata, sizeof(hexdata));

  delay(1000);
}

void Web_loop()
{
  server.handleClient();
}

