/*
 *
 * Protocol_APRS.cpp
 * Encoder for Automatic Packet Reporting System radio protocol
 * Copyright (C) 2023 Linar Yusupov
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

#include <stdint.h>

#include <protocol.h>
#include <LibAPRSesp.h>
#include <pbuf.h>
#include <parse_aprs.h>

#include "../../../SoftRF.h"
#include "../../driver/RF.h"

const rf_proto_desc_t aprs_proto_desc = {
  "APRS",
  .type            = RF_PROTOCOL_APRS,
  .modulation_type = RF_MODULATION_TYPE_2FSK, /* Bell 202 AFSK */
  .preamble_type   = APRS_PREAMBLE_TYPE,
  .preamble_size   = APRS_PREAMBLE_SIZE,
  .syncword        = APRS_SYNCWORD,
  .syncword_size   = APRS_SYNCWORD_SIZE,
  .net_id          = 0x0000, /* not in use */
  .payload_type    = RF_PAYLOAD_DIRECT,
  .payload_size    = APRS_PAYLOAD_SIZE,
  .payload_offset  = 0,
  .crc_type        = APRS_CRC_TYPE,
  .crc_size        = APRS_CRC_SIZE,

  .bitrate         = RF_BITRATE_38400, /* 1200 */
  .deviation       = RF_FREQUENCY_DEVIATION_9_6KHZ, /* TBD */
  .whitening       = RF_WHITENING_NONE, /* TBD */
  .bandwidth       = RF_RX_BANDWIDTH_SS_50KHZ, /* TBD */

  .air_time        = APRS_AIR_TIME,

  .tm_type         = RF_TIMING_INTERVAL,
  .tx_interval_min = APRS_TX_INTERVAL_MIN,
  .tx_interval_max = APRS_TX_INTERVAL_MAX,
  .slot0           = {0, 0},
  .slot1           = {0, 0}
};

extern AX25Msg Incoming_APRS_Packet;
extern char Outgoing_APRS_Comment[80];

struct pbuf_t aprs;
ParseAPRS aprsParse;

int packet2Raw(String &tnc2, AX25Msg &Packet) {
  if (Packet.len < 5) return 0;

  tnc2 = String(Packet.src.call);

  if (Packet.src.ssid > 0) {
      tnc2 += String(F("-"));
      tnc2 += String(Packet.src.ssid);
  }

  tnc2 += String(F(">"));
  tnc2 += String(Packet.dst.call);

  if (Packet.dst.ssid > 0) {
      tnc2 += String(F("-"));
      tnc2 += String(Packet.dst.ssid);
  }

  for (int i = 0; i < Packet.rpt_count; i++) {
      tnc2 += String(",");
      tnc2 += String(Packet.rpt_list[i].call);
      if (Packet.rpt_list[i].ssid > 0) {
          tnc2 += String("-");
          tnc2 += String(Packet.rpt_list[i].ssid);
      }
      if (Packet.rpt_flags & (1 << i)) tnc2 += "*";
  }

  tnc2 += String(F(":"));
  tnc2 += String((const char *)Packet.info);
  tnc2 += String("\n");

  // #ifdef DEBUG_TNC
  //     Serial.printf("[%d] ", ++pkgTNC_count);
  //     Serial.print(tnc2);
  // #endif

  return tnc2.length();
}

bool aprs_decode(void *pkt, ufo_t *this_aircraft, ufo_t *fop) {

  String tnc2;
  packet2Raw(tnc2, Incoming_APRS_Packet);

  Serial.println("APRS RX: " + tnc2);

  int start_val = tnc2.indexOf(">", 0);
  if (start_val > 3)
  {
    String src_call = tnc2.substring(0, start_val);
    memset(&aprs, 0, sizeof(pbuf_t));
    aprs.buf_len = 300;
    aprs.packet_len = tnc2.length();
    tnc2.toCharArray(&aprs.data[0], aprs.packet_len);
    int start_info = tnc2.indexOf(":", 0);
    int end_ssid = tnc2.indexOf(",", 0);
    int start_dst = tnc2.indexOf(">", 2);
    int start_dstssid = tnc2.indexOf("-", start_dst);
    if ((start_dstssid > start_dst) && (start_dstssid < start_dst + 10))
    {
        aprs.dstcall_end_or_ssid = &aprs.data[start_dstssid];
    }
    else
    {
        aprs.dstcall_end_or_ssid = &aprs.data[end_ssid];
    }
    aprs.info_start = &aprs.data[start_info + 1];
    aprs.dstname = &aprs.data[start_dst + 1];
    aprs.dstname_len = end_ssid - start_dst;
    aprs.dstcall_end = &aprs.data[end_ssid];
    aprs.srccall_end = &aprs.data[start_dst];

    if (aprsParse.parse_aprs(&aprs))
    {
#ifndef RASPBERRY_PI
      Serial.print("lat: "); Serial.println(aprs.lat);
      Serial.print("lon: "); Serial.println(aprs.lng);
      Serial.print("alt: "); Serial.println(aprs.altitude);
      Serial.print("crs: "); Serial.println(aprs.course);
      Serial.print("spd: "); Serial.println(aprs.speed);
#endif

      fop->protocol  = RF_PROTOCOL_APRS;

      fop->latitude  = aprs.lat;
      fop->longitude = aprs.lng;
      fop->altitude  = (float) aprs.altitude / _GPS_FEET_PER_METER;
      fop->course    = (float) aprs.course;
      fop->speed     = (float) aprs.speed;

      fop->addr_type = ADDR_TYPE_ANONYMOUS;
      fop->timestamp = (uint32_t) this_aircraft->timestamp;

      return false /* true */;
    }
  }

  return false;
}

static void nmea_lat(float lat, char *buf)
{
  int   lat_int = (int) lat;
  float lat_dec = lat - lat_int;

  sprintf(buf, "%02d%05.2f%c", abs(lat_int), fabsf(lat_dec * 60), lat < 0 ? 'S' : 'N');
}

static void nmea_lon(float lon, char *buf)
{
  int   lon_int = (int) lon;
  float lon_dec = lon - lon_int;

  sprintf(buf, "%03d%05.2f%c", abs(lon_int), fabsf(lon_dec * 60), lon < 0 ? 'W' : 'E');
}

size_t aprs_encode(void *pkt, ufo_t *this_aircraft) {

  char buf[12];

  //snprintf(buf, sizeof(buf), "FLR%06X", this_aircraft->addr);
  //APRS_setCallsign(buf, 0);

  APRS_setDestination("OGNFLR", 0);

  // Let's first set our latitude and longtitude.
  // These should be in NMEA format!
  nmea_lat(this_aircraft->latitude, buf);
  APRS_setLat(buf);
  nmea_lon(this_aircraft->longitude, buf);
  APRS_setLon(buf);
  
  // We can optionally set power/height/gain/directivity
  // information. These functions accept ranges
  // from 0 to 10, directivity 0 to 9.
  // See this site for a calculator:
  // http://www.aprsfl.net/phgr.php
  // LibAPRS will only add PHG info if all four variables
  // are defined!
  //APRS_setPower(2);
  //APRS_setHeight(4);
  //APRS_setGain(7);
  //APRS_setDirectivity(0);

  snprintf(Outgoing_APRS_Comment, sizeof(Outgoing_APRS_Comment),
           "%03d/%03d/A=%06d !W10! id%08X",
           (int) this_aircraft->course,
           (int) this_aircraft->speed, /* knots */
           (int) (this_aircraft->altitude * _GPS_FEET_PER_METER), /* negative - TBD */
           this_aircraft->addr);

  strcpy((char *) pkt, "NOT IN USE");

  return aprs_proto_desc.payload_size;
}
