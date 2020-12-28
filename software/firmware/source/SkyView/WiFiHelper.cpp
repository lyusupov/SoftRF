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


#include "EEPROMHelper.h"
#include "SoCHelper.h"
#include "WiFiHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"
#include "WebHelper.h"

#include "SkyView.h"

String host_name = HOSTNAME;

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

unsigned int UDP_Data_Port = 0;           // local port to listen for UDP packets

char UDPpacketBuffer[UDP_PACKET_BUFSIZE]; // buffer to hold incoming packets

static unsigned long WiFi_STA_TimeMarker = 0;
static bool WiFi_STA_connected = false;

#if defined(POWER_SAVING_WIFI_TIMEOUT)
static unsigned long WiFi_No_Clients_Time_ms = 0;
#endif

size_t WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  int noBytes = Uni_Udp.parsePacket();
  if ( noBytes ) {

    if (noBytes > max_size) {
      noBytes = max_size;
    }

    // We've received a packet, read the data from it
    Uni_Udp.read(buf,noBytes); // read the packet into the buffer

    return (size_t) noBytes;
  } else {
    return 0;
  }
}

/**
 * @brief Arduino setup function.
 */
void WiFi_setup()
{

  // Set Hostname.
  host_name += String((SoC->getChipId() & 0xFFFFFF), HEX);
  SoC->WiFi_hostname(host_name);

  // Print hostname.
  Serial.println("Hostname: " + host_name);

  WiFiMode_t mode = (settings->connection == CON_WIFI_UDP ||
                     settings->connection == CON_WIFI_TCP ) ?
                     WIFI_AP_STA : WIFI_AP;
  WiFi.mode(mode);

  SoC->WiFi_setOutputPower(WIFI_TX_POWER_MED); // 10 dB
  // WiFi.setOutputPower(0); // 0 dB
  //system_phy_set_max_tpw(4 * 0); // 0 dB
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

  if (settings->connection == CON_WIFI_UDP) {
    switch (settings->protocol)
    {
    case PROTOCOL_NMEA:
      UDP_Data_Port = NMEA_UDP_PORT;
      break;
    case PROTOCOL_GDL90:
      UDP_Data_Port = GDL90_DST_PORT;
      break;
    default:
      UDP_Data_Port = 0;
      break;
    }

    if (UDP_Data_Port) {
      Uni_Udp.begin(UDP_Data_Port);
      Serial.print(F("UDP server has started at port: "));
      Serial.println(UDP_Data_Port);
    }
  }

  if (settings->connection == CON_WIFI_UDP ||
      settings->connection == CON_WIFI_TCP ) {
    if (strnlen(settings->server, sizeof(settings->server)) > 0 &&
        strnlen(settings->key,  sizeof(settings->key))  > 0) {
      WiFi.begin(settings->server, settings->key);

      Serial.print(F("Wait for WiFi connection to "));
      Serial.print(settings->server);
      Serial.println(F(" AP..."));
    }

    WiFi_STA_TimeMarker = millis();
  }

#if defined(POWER_SAVING_WIFI_TIMEOUT)
  WiFi_No_Clients_Time_ms = millis();
#endif
}

void WiFi_loop()
{
  if (settings->connection == CON_WIFI_UDP ||
      settings->connection == CON_WIFI_TCP ) {
    if (WiFi.status() != WL_CONNECTED) {
#if 0
      if (millis() - WiFi_STA_TimeMarker > 2000) {
        Serial.print(".");
        WiFi_STA_TimeMarker = millis();
      }
#endif
      if (WiFi_STA_connected == true) {
        WiFi_STA_connected = false;
        Serial.print(F("Disconnected from WiFi AP "));
        Serial.println(settings->server);
      }
    } else {
      if (WiFi_STA_connected == false) {
        Serial.println("");
        Serial.print(F("Connected to WiFi AP "));
        Serial.println(settings->server);
        Serial.print(F("IP address: "));
        Serial.println(WiFi.localIP());
        WiFi_STA_connected = true;
      }
    }
  }
#if defined(USE_DNS_SERVER)
  if (dns_active) {
    dnsServer.processNextRequest();
  }
#endif

#if defined(POWER_SAVING_WIFI_TIMEOUT)
  if ((settings->power_save & POWER_SAVE_WIFI) && WiFi.getMode() == WIFI_AP) {
    if (SoC->WiFi_clients_count() == 0) {
      if ((millis() - WiFi_No_Clients_Time_ms) > POWER_SAVING_WIFI_TIMEOUT) {
        Web_fini();
        WiFi_fini();

        if (settings->protocol == PROTOCOL_NMEA) {
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
  if (settings->connection == CON_WIFI_UDP && UDP_Data_Port) {
    Uni_Udp.stop();
  }

  WiFi.mode(WIFI_OFF);
}
