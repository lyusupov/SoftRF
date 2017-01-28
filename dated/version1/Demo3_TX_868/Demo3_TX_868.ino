/*
 * Demo of SoftRF
 *  
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 * License: GNU GPL v3 
 * Web: http://github.com/lyusupov/SoftRF
 *
 * AVR/Arduino nRF905 Library/Driver is developed by Zak Kemble, contact@zakkemble.co.uk
 * flarm_decode is developed by Stanislaw Pusep, http://github.com/creaktive
 * Arduino JSON library is developed by Benoit Blanchon, http://github.com/bblanchon
 * Arduino Time Library is developed by Paul Stoffregen, http://github.com/PaulStoffregen
 */

/*
 *
 * NodeMCU 1.0 GPIO pins:
 * 2 -> CE
 * 4 -> PWR
 * 16 -> TXE
 * 0 -> CD
 * 5 -> DR
 * 15 -> CSN
 * 12 -> SO
 * 13 -> SI
 * 14 -> SCK
 */
 
#include <nRF905.h>
#include <SPI.h>
#include <TimeLib.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <FS.h>
#include <ArduinoOTA.h>

#include <ArduinoJson.h>

#include "WiFiHelper.h"
#include "OTAHelper.h"
#include "WebHelper.h"
#include "TimeHelper.h"
#include "flarm_codec.h"

#include "Demo3_TX_868.h"

#define DEBUG 0

ufo_t fo;
tx_state txready = TX_CLEAR;
uint32_t rx_packets_counter = 0;


String TxDataTemplate = "0282dd204901f981798a85b69764bdf99ed77fd3c2300000";

byte TxBuffer[PKT_SIZE], RxBuffer[PKT_SIZE];

void setup()
{
	// Start up
	nRF905_init();

  nRF905_setFrequency(NRF905_BAND_868 , RF_FREQ);
  nRF905_setTransmitPower(NRF905_PWR_10);
  nRF905_setCRC(NRF905_CRC_16);
  //nRF905_setCRC(NRF905_CRC_DISABLE);
 
	// Set address of this device
	byte addr[] = RXADDR;
	nRF905_setRXAddress(addr);

	// Put into receive mode
	nRF905_receive();

	Serial.begin(115200);

  delay(100);
 
  Serial.println("\r\n");
  Serial.print("Chip ID: 0x");
  Serial.println(ESP.getChipId(), HEX);

  WiFi_setup();
  OTA_setup();
  Web_setup();
  Time_setup();
}

void loop()
{
  unsigned long startTime = millis();
  bool success;


  StaticJsonBuffer<512> jsonBuffer;

  if (txready == TX_DATA_READY) {

  	// Make data
    char *data = (char *) TxBuffer;

  	// Set address of device to send to
  	byte addr[] = TXADDR;
  	nRF905_setTXAddress(addr);

  	// Set payload data
  	nRF905_setData(data, NRF905_PAYLOAD_SIZE );

  	// Send payload (send fails if other transmissions are going on, keep trying until success)
  	while(!nRF905_send()) { yield(); } ;

    txready = TX_SENT;
  }

	// Put into receive mode
	nRF905_receive();

	// Wait for reply with timeout
	unsigned long sendStartTime = millis();
	while(1)
	{
		success = nRF905_getData(RxBuffer, sizeof(RxBuffer));
		if(success)// Got data
			break;

		// Timeout
		if(millis() - sendStartTime > TIMEOUT)
			break;
    yield();
	}

#if DEBUG
  success = true;
#endif

	if(success)
	{
    char *q;

    rx_packets_counter++;

#if DEBUG
    Hex2Bin(TxDataTemplate, RxBuffer);
#endif

    fo.raw = Bin2Hex(RxBuffer);

    q = flarm_decode(
        (flarm_packet *) RxBuffer,
        LATITUDE, LONGTITUDE, ALTITUDE,
        now(),
        0,
        0
    );

    if (q) {
      JsonObject& root = jsonBuffer.parseObject(q);
    
      if (!root.success()) {
        Serial.println("Json parseObject() failed");
      }

      fo.addr = root["addr"];
      fo.timestamp = root["time"];
      fo.latitude = root["lat"];
      fo.longtitude = root["lon"];
      fo.altitude = root["alt"];
      fo.distance = root["dist"];
      fo.vs = root["vs"];
      fo.type = root["type"];
      fo.stealth = root["stealth"];
      fo.no_track = root["no_track"];
      fo.ns[0] = root["ns"][0]; fo.ns[1] = root["ns"][1];
      fo.ns[2] = root["ns"][2]; fo.ns[3] = root["ns"][3];
      fo.ew[0] = root["ew"][0]; fo.ew[1] = root["ew"][1];
      fo.ew[2] = root["ew"][2]; fo.ew[3] = root["ew"][3];
#if 0
      Serial.println(fo.addr);
      Serial.println(fo.latitude);
      Serial.println(fo.longtitude);
      Serial.println(fo.altitude);
      Serial.println(fo.distance);
#endif
    }
    // Serial.println(fo.raw);
	}

  if(success)
  {
    //WiFi_forward_to_argus();
    //WiFi_forward_to_xcsoar();
    //WiFi_forward_to_cloud();
  }
  // Handle OTA update.
  OTA_loop();
  
  // Handle Web
  Web_loop();

  yield();
}
