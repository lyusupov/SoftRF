/* -*- tab-width: 2; mode: c; -*-
 *
 * C++ class for Arduino to function as a wrapper around opendroneid.
 * This file has the wifi beacon setup code.
 *
 * Copyright (c) 2023, Steve Jack.
 *
 * May  '23:  WiFi country code now defined in id_open.h.
 *
 * Nov. '22:  Split out from id_open.cpp.
 *
 * MIT licence.
 *
 * NOTES
 *
 * 
 */

#pragma GCC diagnostic warning "-Wunused-variable"

#include <Arduino.h>

#include "id_open.h"

#if ID_OD_WIFI_BEACON && (!USE_BEACON_FUNC)

/*
 * The variables setup by the following function are defined in id_open.h.
 * Some of the tags are copied from beacon frames sent by the Raspberry Pi
 * which is known to work with Android ID apps.
 *
 */

void ID_OpenDrone::init_beacon() {

  int           i;
  const uint8_t wifi_channels  = WIFI_COUNTRY_NCHAN;
  const char    wifi_country[] = WIFI_COUNTRY_CC;

  struct __attribute__((__packed__)) beacon_header {

    uint8_t control[2];          //  0-1:  frame control  
    uint8_t duration[2];         //  2-3:  duration
    uint8_t dest_addr[6];        //  4-9:  destination
    uint8_t src_addr[6];         // 10-15: source  
    uint8_t bssid[6];            // 16-21: base station
    uint8_t seq[2];              // 22-23: sequence
    uint8_t timestamp[8];        // 24-31: 
    uint8_t interval[2];         //
    uint8_t capability[2];
  } *header;

  header                  = (struct beacon_header *) beacon_frame;
  beacon_timestamp        = header->timestamp;
  beacon_seq              = header->seq;
  
  header->control[0]      = 0x80;
  header->interval[0]     = (uint8_t)  beacon_interval;
  header->interval[1]     = (uint8_t) (beacon_interval >> 8);

  memcpy(header->capability,capability(),2);

  for (i = 0; i < 6; ++i) {

    header->dest_addr[i] = 0xff;
    header->src_addr[i]  = 
    header->bssid[i]     = WiFi_mac_addr[i];
  }

#if defined(ESP32_WIFI_OPTION) && (ESP32_WIFI_OPTION == 2)
  header->src_addr[5]++;
  header->bssid[5]++;
#endif

  beacon_offset = sizeof(struct beacon_header);

  beacon_frame[beacon_offset++] = 0;
  beacon_frame[beacon_offset++] = ssid_length;

  for (i = 0; (i < 32)&&(ssid[i]); ++i) {

    beacon_frame[beacon_offset++] = ssid[i];
  }

  // Supported rates
#if 0
  beacon_frame[beacon_offset++] = 0x01; // This is what ODID 1.0 does.
  beacon_frame[beacon_offset++] = 0x01;
  beacon_frame[beacon_offset++] = 0x8c; // 11b, 6(B) Mbit/sec
#else
  beacon_offset = tag_rates(beacon_frame,beacon_offset);
#endif

  // DS
  beacon_frame[beacon_offset++] = 0x03;
  beacon_frame[beacon_offset++] = 0x01;
  beacon_frame[beacon_offset++] = wifi_channel;

#if 1
  // Traffic Indication Map
  beacon_frame[beacon_offset++] = 0x05;
  beacon_frame[beacon_offset++] = 0x04;
  beacon_frame[beacon_offset++] = 0x00;
  beacon_frame[beacon_offset++] = 0x02;
  beacon_frame[beacon_offset++] = 0x00;
  beacon_frame[beacon_offset++] = 0x00;
#endif

#if 1
  // Country Information
  beacon_frame[beacon_offset++] = 0x07;
  beacon_frame[beacon_offset++] = 0x06;
  beacon_frame[beacon_offset++] = wifi_country[0];
  beacon_frame[beacon_offset++] = wifi_country[1];
  beacon_frame[beacon_offset++] = 0x20;
  beacon_frame[beacon_offset++] = 0x01;
  beacon_frame[beacon_offset++] = wifi_channels;
  beacon_frame[beacon_offset++] = 0x14;
#endif

#if 0
  // Power Constraint
  beacon_frame[beacon_offset++] = 0x20;
  beacon_frame[beacon_offset++] = 0x01;
  beacon_frame[beacon_offset++] = 0x00;
#endif

#if 0
  // TPC Report Transmit Power
  beacon_frame[beacon_offset++] = 0x23;
  beacon_frame[beacon_offset++] = 0x02;
  beacon_frame[beacon_offset++] = 0x11;
  beacon_frame[beacon_offset++] = 0x00;
#endif

#if 0
  // ERP Information
  beacon_frame[beacon_offset++] = 0x2a;
  beacon_frame[beacon_offset++] = 0x01;
  beacon_frame[beacon_offset++] = 0x00;
#endif

#if 1
  // Extended Supported Rates
  beacon_offset = tag_ext_rates(beacon_frame,beacon_offset);
#endif

#if 1
  // Other WiFi chip dependent tags, e.g. HT Capabilities, HT Information
  beacon_offset = misc_tags(beacon_frame,beacon_offset);
#endif

#if 0
  // WPA Information
  beacon_frame[beacon_offset++] = 0xdd;
  beacon_frame[beacon_offset++] = 0x1a;

  beacon_frame[beacon_offset++] = 0x00; // Microsoft
  beacon_frame[beacon_offset++] = 0x50;
  beacon_frame[beacon_offset++] = 0xf2;
  beacon_frame[beacon_offset++] = 0x01;
  beacon_frame[beacon_offset++] = 0x01;
  beacon_frame[beacon_offset++] = 0x00;

  beacon_frame[beacon_offset++] = 0x00;
  beacon_frame[beacon_offset++] = 0x50;
  beacon_frame[beacon_offset++] = 0xf2;
  beacon_frame[beacon_offset++] = 0x02;
  beacon_frame[beacon_offset++] = 0x02;
  beacon_frame[beacon_offset++] = 0x00;

  beacon_frame[beacon_offset++] = 0x00;
  beacon_frame[beacon_offset++] = 0x50;
  beacon_frame[beacon_offset++] = 0xf2;
  beacon_frame[beacon_offset++] = 0x04;

  beacon_frame[beacon_offset++] = 0x00;
  beacon_frame[beacon_offset++] = 0x50;
  beacon_frame[beacon_offset++] = 0xf2;
  beacon_frame[beacon_offset++] = 0x02;
  beacon_frame[beacon_offset++] = 0x01;
  beacon_frame[beacon_offset++] = 0x00;

  beacon_frame[beacon_offset++] = 0x00;
  beacon_frame[beacon_offset++] = 0x50;
  beacon_frame[beacon_offset++] = 0xf2;
  beacon_frame[beacon_offset++] = 0x02;
#endif

  return;
}

/*
 *
 */

#endif
