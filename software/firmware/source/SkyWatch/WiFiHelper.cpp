/*
 * WiFiHelper.cpp
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

#include <FS.h>
#include <TimeLib.h>

//#include "OTAHelper.h"
//#include "GNSSHelper.h"
#include "EEPROMHelper.h"
#include "SoCHelper.h"
#include "WiFiHelper.h"
#include "TrafficHelper.h"
//#include "RFHelper.h"
#include "WebHelper.h"
#include "NMEAHelper.h"
#include "BatteryHelper.h"

String station_ssid = MY_ACCESSPOINT_SSID ;
String station_psk  = MY_ACCESSPOINT_PSK ;

String host_name;

IPAddress local_IP(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

/**
 * Default WiFi connection information.
 *
 */
const char* ap_default_psk = "12345678"; ///< Default PSK.

#if defined(USE_DNS_SERVER)
#include <DNSServer.h>

const byte DNS_PORT = 53;
DNSServer dnsServer;
bool dns_active = false;
#endif

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Uni_Udp;

unsigned int RFlocalPort = RELAY_SRC_PORT;      // local port to listen for UDP packets

char UDPpacketBuffer[256]; // buffer to hold incoming and outgoing packets

#if defined(POWER_SAVING_WIFI_TIMEOUT)
static unsigned long WiFi_No_Clients_Time_ms = 0;
#endif

/**
 * @brief Read WiFi connection information from file system.
 * @param ssid String pointer for storing SSID.
 * @param pass String pointer for storing PSK.
 * @return True or False.
 * 
 * The config file have to containt the WiFi SSID in the first line
 * and the WiFi PSK in the second line.
 * Line seperator can be \r\n (CR LF) \r or \n.
 */
bool loadConfig(String *ssid, String *pass)
{
  // open file for reading.
  File configFile = SPIFFS.open("/cl_conf.txt", "r");
  if (!configFile)
  {
    Serial.println(F("Failed to open cl_conf.txt."));

    return false;
  }

  // Read content from config file.
  String content = configFile.readString();
  configFile.close();

  content.trim();

  // Check if ther is a second line available.
  int8_t pos = content.indexOf("\r\n");
  uint8_t le = 2;
  // check for linux and mac line ending.
  if (pos == -1)
  {
    le = 1;
    pos = content.indexOf("\n");
    if (pos == -1)
    {
      pos = content.indexOf("\r");
    }
  }

  // If there is no second line: Some information is missing.
  if (pos == -1)
  {
    Serial.println(F("Invalid content."));
    Serial.println(content);

    return false;
  }

  // Store SSID and PSK into string vars.
  *ssid = content.substring(0, pos);
  *pass = content.substring(pos + le);

  ssid->trim();
  pass->trim();

#ifdef SERIAL_VERBOSE
  Serial.println("----- file content -----");
  Serial.println(content);
  Serial.println("----- file content -----");
  Serial.println("ssid: " + *ssid);
  Serial.println("psk:  " + *pass);
#endif

  return true;
} // loadConfig


/**
 * @brief Save WiFi SSID and PSK to configuration file.
 * @param ssid SSID as string pointer.
 * @param pass PSK as string pointer,
 * @return True or False.
 */
bool saveConfig(String *ssid, String *pass)
{
  // Open config file for writing.
  File configFile = SPIFFS.open("/cl_conf.txt", "w");
  if (!configFile)
  {
    Serial.println(F("Failed to open cl_conf.txt for writing"));

    return false;
  }

  // Save SSID and PSK.
  configFile.println(*ssid);
  configFile.println(*pass);

  configFile.close();

  return true;
} // saveConfig

/**
 * @brief Arduino setup function.
 */
void WiFi_setup()
{
#if 0
  // Initialize file system.
  if (!SPIFFS.begin())
  {
    Serial.println(F("Failed to mount file system"));
    return;
  }

  // Load wifi connection information.
  if (! loadConfig(&station_ssid, &station_psk))
  {
    station_ssid = MY_ACCESSPOINT_SSID ;
    station_psk = MY_ACCESSPOINT_PSK ;

    Serial.println(F("No WiFi connection information available."));
  }
#endif

  // Check WiFi connection
  // ... check mode
  if (WiFi.getMode() != WIFI_STA)
  {
    WiFi.mode(WIFI_STA);
    delay(10);
  }

  // ... Compare file config with sdk config.
  if (WiFi.SSID() != station_ssid || WiFi.psk() != station_psk)
  {
    Serial.println(F("WiFi config changed."));

    // ... Try to connect to WiFi station.
    WiFi.begin(station_ssid.c_str(), station_psk.c_str());

    // ... Pritn new SSID
    Serial.print(F("new SSID: "));
    Serial.println(WiFi.SSID());

    // ... Uncomment this for debugging output.
    //WiFi.printDiag(Serial);
  }
  else
  {
    // ... Begin with sdk config.
    WiFi.begin();
  }

  // Set Hostname.
  host_name += hw_info.model == SOFTRF_MODEL_SKYWATCH ?
                                SKYWATCH_IDENT : SOFTRF_IDENT;
  host_name += "-";
  host_name += String((SoC->getChipId() & 0xFFFFFF), HEX);
  SoC->WiFi_hostname(host_name);

  // Print hostname.
  Serial.println("Hostname: " + host_name);

  Serial.println(F("Wait for WiFi connection."));

  // ... Give ESP 10 seconds to connect to station.
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000)
  {
    Serial.write('.');
    //Serial.print(WiFi.status());
    delay(500);
  }
  Serial.println();

  // Check connection
  if(WiFi.status() == WL_CONNECTED)
  {
    // ... print IP Address
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println(F("Can not connect to WiFi station. Go into AP mode."));
    
    // Go into software AP mode.
    WiFi.mode(WIFI_AP);
    SoC->WiFi_set_param(WIFI_PARAM_TX_POWER, WIFI_TX_POWER_MED); // 10 dBm
    SoC->WiFi_set_param(WIFI_PARAM_DHCP_LEASE_TIME, WIFI_DHCP_LEASE_HRS);
    delay(10);

    Serial.print(F("Setting soft-AP configuration ... "));
    Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ?
      F("Ready") : F("Failed!"));

    Serial.print(F("Setting soft-AP ... "));
    Serial.println(WiFi.softAP(host_name.c_str(), ap_default_psk) ?
      F("Ready") : F("Failed!"));
#if defined(USE_DNS_SERVER)
    // if DNSServer is started with "*" for domain name, it will reply with
    // provided IP to all DNS request
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    dns_active = true;
#endif
    Serial.print(F("IP address: "));
    Serial.println(WiFi.softAPIP());
  }

  Uni_Udp.begin(RFlocalPort);
  Serial.print(F("UDP server has started at port: "));
  Serial.println(RFlocalPort);

#if defined(POWER_SAVING_WIFI_TIMEOUT)
  WiFi_No_Clients_Time_ms = millis();
#endif
}

void WiFi_loop()
{
#if defined(USE_DNS_SERVER)
  if (dns_active) {
    dnsServer.processNextRequest();
  }
#endif

#if defined(POWER_SAVING_WIFI_TIMEOUT)
  if ((settings->s.power_save & POWER_SAVE_WIFI) && WiFi.getMode() == WIFI_AP) {
    if (SoC->WiFi_clients_count() == 0) {
      if ((millis() - WiFi_No_Clients_Time_ms) > POWER_SAVING_WIFI_TIMEOUT) {
        NMEA_fini();
        Web_fini();
        WiFi_fini();

        if (settings->s.nmea_p) {
          Serial.println(F("$PSRFS,WIFI_OFF"));
        }
      }
    } else {
      WiFi_No_Clients_Time_ms = millis();
    }
  }
#endif
}

void WiFi_fini()
{
  Uni_Udp.stop();

  WiFi.mode(WIFI_OFF);
}
