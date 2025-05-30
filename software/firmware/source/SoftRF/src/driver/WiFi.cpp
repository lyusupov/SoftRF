/*
 * WiFiHelper.cpp
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

#if defined(EXCLUDE_WIFI)
void WiFi_setup()   {}
void WiFi_loop()    {}
void WiFi_fini()    {}
#else

#include <TimeLib.h>

#include "../system/OTA.h"
#include "GNSS.h"
#include "EEPROM.h"
#include "WiFi.h"
#include "../TrafficHelper.h"
#include "RF.h"
#include "../ui/Web.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
#include "Battery.h"

String station_ssid = MY_ACCESSPOINT_SSID ;
String station_psk  = MY_ACCESSPOINT_PSK ;

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
WiFiUDP *Uni_Udp = NULL;

unsigned int RFlocalPort = RELAY_SRC_PORT;      // local port to listen for UDP packets

char UDPpacketBuffer[256]; // buffer to hold incoming and outgoing packets

#if defined(POWER_SAVING_WIFI_TIMEOUT)
static unsigned long WiFi_No_Clients_Time_ms = 0;
#endif

#if defined(ENABLE_REMOTE_ID)
#include "../protocol/radio/RemoteID.h"
static unsigned long RID_Time_Marker = 0;
#endif /* ENABLE_REMOTE_ID */

size_t Raw_Receive_UDP(uint8_t *buf)
{
  int noBytes;
  if (Uni_Udp && (noBytes = Uni_Udp->parsePacket())) {

    if (noBytes > MAX_PKT_SIZE) {
      noBytes = MAX_PKT_SIZE;
    }

    // We've received a packet, read the data from it
    Uni_Udp->read(buf,noBytes); // read the packet into the buffer

    return (size_t) noBytes;
  } else {
    return 0;
  }
}

void Raw_Transmit_UDP()
{
    size_t rx_size = RF_Payload_Size(settings->rf_protocol);
    rx_size = rx_size > sizeof(fo.raw) ? sizeof(fo.raw) : rx_size;
    String str = Bin2Hex(fo.raw, rx_size);
    size_t len = str.length();
    // ASSERT(sizeof(UDPpacketBuffer) > 2 * PKT_SIZE + 1)
    str.toCharArray(UDPpacketBuffer, sizeof(UDPpacketBuffer));
    UDPpacketBuffer[len] = '\n';
    SoC->WiFi_transmit_UDP(RELAY_DST_PORT, (byte *)UDPpacketBuffer, len + 1);
}

#if defined(USE_ARDUINO_WIFI)
void WiFi_setup()
{
  char id_06x[8];
  snprintf(id_06x, sizeof(id_06x),"%06x", SoC->getChipId() & 0x00FFFFFFU);
  // Set Hostname.
  host_name += "-";
  host_name += String(id_06x);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    // while (true);
    return;
  }

#if !defined(_WIFI_ESP_AT_H_) && !defined(ARDUINO_ARCH_MBED)
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
#endif /* _WIFI_ESP_AT_H_ */

  WiFi.setHostname(host_name.c_str());

  // Print hostname.
  Serial.println("Hostname: " + host_name);

  if (station_ssid.length() > 0) {
    // ... Try to connect to WiFi station.
    WiFi.begin(station_ssid.c_str(), station_psk.c_str());

    // ... Print new SSID
    Serial.print(F("new SSID: "));
    Serial.println(WiFi.SSID());

    Serial.println(F("Wait for WiFi connection."));

    // ... Give Wi-Fi 10-20 seconds to connect to an AP.
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - startTime < WIFI_STA_TIMEOUT)
    {
      Serial.write('.'); Serial.flush();
      //Serial.print(WiFi.status());
      delay(500);
    }
    Serial.println();

    // Check connection
    if (WiFi.status() == WL_CONNECTED) {
      // print the SSID of the network you're attached to:
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID());

#if !defined(ARDUINO_RASPBERRY_PI_PICO)
      // print your board's IP address:
      IPAddress ip = WiFi.localIP();
      Serial.print("IP Address: ");
      Serial.println(ip);
#endif /* ARDUINO_RASPBERRY_PI_PICO */

      // print the received signal strength:
      long rssi = WiFi.RSSI();
      Serial.print("signal strength (RSSI):");
      Serial.print(rssi);
      Serial.println(" dBm");
    } else {
      Serial.println(F("Can not connect to WiFi station. Go into AP mode."));
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    // Go into software AP mode.
    Serial.print(F("Setting soft-AP configuration ... "));
    WiFi.config(local_IP, gateway, gateway, subnet);
    Serial.println(F("Ready"));

    SoC->WiFi_set_param(WIFI_PARAM_TX_POWER, WIFI_TX_POWER_MED); // 10 dBm
    SoC->WiFi_set_param(WIFI_PARAM_DHCP_LEASE_TIME, WIFI_DHCP_LEASE_HRS);

    Serial.print(F("Setting soft-AP ... "));
    Serial.println(WiFi.beginAP(host_name.c_str(), ap_default_psk) == WL_AP_LISTENING ?
      F("Ready") : F("Failed!"));

    delay(10);

//  Serial.print(F("IP address: "));
//  Serial.println(WiFi.softAPIP());
  }

  if (settings->nmea_out == NMEA_UDP  ||
      settings->gdl90    == GDL90_UDP ||
      settings->d1090    == D1090_UDP) {
    Uni_Udp = new WiFiUDP();

    Uni_Udp->begin(RFlocalPort);
    Serial.print(F("UDP server has started at port: "));
    Serial.println(RFlocalPort);
  }

#if defined(POWER_SAVING_WIFI_TIMEOUT)
  WiFi_No_Clients_Time_ms = millis();
#endif
}

#else /* USE_ARDUINO_WIFI */

/**
 * @brief Arduino setup function.
 */
void WiFi_setup()
{
  char id_06x[8];
  snprintf(id_06x, sizeof(id_06x),"%06x", SoC->getChipId() & 0x00FFFFFFU);
  // Set Hostname.
  host_name += "-";
  host_name += String(id_06x);

  if (SoC->id == SOC_ESP8266    || SoC->id == SOC_RP2040 ||
      SoC->id == SOC_RP2350_ARM || SoC->id == SOC_RP2350_RISC) {
    WiFi.mode(WIFI_STA);
    if (SoC->WiFi_hostname(host_name) == false) {
      return;
    }
  } else { /* ESP32, -S2, -S3, -C3 */
    SoC->WiFi_hostname(host_name);
    WiFi.mode(WIFI_STA);
    delay(10);
  }

  // Print hostname.
  Serial.println("Hostname: " + host_name);

  if (station_ssid.length() > 0) {
    // ... Try to connect to WiFi station.
    WiFi.begin(station_ssid.c_str(), station_psk.c_str());

    // ... Print new SSID
    Serial.print(F("new SSID: "));
    Serial.println(WiFi.SSID());

    Serial.println(F("Wait for WiFi connection."));

    // ... Give Wi-Fi 10-20 seconds to connect to an AP.
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - startTime < WIFI_STA_TIMEOUT)
    {
      Serial.write('.'); Serial.flush();
      //Serial.print(WiFi.status());
      delay(500);
    }
    Serial.println();

    // Check connection
    if (WiFi.status() == WL_CONNECTED) {
      // ... print IP Address
      Serial.print(F("IP address: "));
      Serial.println(WiFi.localIP());
    } else {
      Serial.println(F("Can not connect to WiFi station. Go into AP mode."));
      WiFi.mode(WIFI_OFF);
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
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

  if (settings->nmea_out == NMEA_UDP  ||
      settings->gdl90    == GDL90_UDP ||
      settings->d1090    == D1090_UDP) {
    Uni_Udp = new WiFiUDP();

    Uni_Udp->begin(RFlocalPort);
    Serial.print(F("UDP server has started at port: "));
    Serial.println(RFlocalPort);
  }

#if defined(POWER_SAVING_WIFI_TIMEOUT)
  WiFi_No_Clients_Time_ms = millis();
#endif

#if defined(ENABLE_REMOTE_ID)
  rid_init();
  RID_Time_Marker = millis();
#endif /* ENABLE_REMOTE_ID */
}
#endif /* USE_ARDUINO_WIFI */

void WiFi_loop()
{
#if defined(USE_DNS_SERVER)
  if (dns_active) {
    dnsServer.processNextRequest();
  }
#endif

#if defined(POWER_SAVING_WIFI_TIMEOUT)
  if ((settings->power_save & POWER_SAVE_WIFI) && WiFi.getMode() == WIFI_AP) {
    if (SoC->WiFi_clients_count() == 0) {
      if ((millis() - WiFi_No_Clients_Time_ms) > POWER_SAVING_WIFI_TIMEOUT) {
        NMEA_fini();
        Web_fini();
        WiFi_fini();

        if (settings->nmea_p) {
          StdOut.println(F("$PSRFS,WIFI_OFF"));
        }
      }
    } else {
      WiFi_No_Clients_Time_ms = millis();
    }
  }
#endif

#if defined(ENABLE_REMOTE_ID) && !defined(USE_ARDUINO_WIFI)
  if (rid_enabled() && WiFi.getMode() == WIFI_AP && isValidFix()) {
    if ((millis() - RID_Time_Marker) > (RID_TX_INTERVAL_MIN + RID_TX_INTERVAL_MAX)/2) {
      rid_encode((void *) &utm_data, &ThisAircraft);
      squitter.transmit(&utm_data);

      RID_Time_Marker = millis();
    }
  }
#endif /* ENABLE_REMOTE_ID */
}

void WiFi_fini()
{
  if (Uni_Udp) Uni_Udp->stop();

#if defined(USE_ARDUINO_WIFI)
#if !defined(_WIFI_ESP_AT_H_)
  WiFi.end();
#else
  WiFi.endAP();
#endif /* _WIFI_ESP_AT_H_ */
#else
  WiFi.mode(WIFI_OFF);
#endif /* USE_ARDUINO_WIFI */
}

#endif /* EXCLUDE_WIFI */
