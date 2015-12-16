/*
 * 
 * 
 */

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <FS.h>
#include <ArduinoOTA.h>

#include "OTAHelper.h"

void OTA_setup()
{
  // Start OTA server.
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.begin();
}

void OTA_loop()
{
  // Handle OTA server.
  ArduinoOTA.handle();
}

