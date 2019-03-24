/*
 * GDL90Helper.cpp
 * Copyright (C) 2019 Linar Yusupov
 *
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

#include "SoCHelper.h"
#include "GDL90Helper.h"
#include "EEPROMHelper.h"

#include "SkyView.h"

extern "C" {
#include <gdl90.h>
}

#define GDL90_RINGBUF_SIZE  sizeof(gdl_message_escaped_t)

static unsigned char gdl90_ringbuf[GDL90_RINGBUF_SIZE];
static unsigned int gdl90buf_head = 0;
static unsigned char prev_c = 0;

gdl_message_t message;

gdl90_msg_heartbeat heartbeat;
gdl90_msg_traffic_report_t traffic;
gdl90_msg_traffic_report_t ownship;
gdl90_msg_ownship_geo_altitude geo_altitude;

void GDL90_setup()
{
  if (settings->protocol == PROTOCOL_GDL90) {

    gdl90_crcInit();

    if (settings->connection == CON_SERIAL) {
      uint32_t SerialBaud;

      switch (settings->baudrate)
      {
      case B4800:
        SerialBaud = 4800;
        break;
      case B9600:
        SerialBaud = 9600;
        break;
      case B19200:
        SerialBaud = 19200;
        break;
      case B57600:
        SerialBaud = 57600;
        break;
      case B115200:
        SerialBaud = 115200;
        break;
      case B2000000:
        SerialBaud = 2000000;
        break;
      case B38400:
      default:
        SerialBaud = 38400;
        break;
      }

      SoC->swSer_begin(SerialBaud);
    }
  }
}

void GDL90_loop()
{
  unsigned int gdl90buf_tail;
  size_t msg_size;

  while (SerialInput.available()) {
    unsigned char c = SerialInput.read();

    if (c == GDL90_CONTROL_ESCAPE) {
      prev_c = c;
      continue;
    } else if (prev_c == GDL90_CONTROL_ESCAPE) {
      prev_c = c;
      c ^= GDL90_ESCAPE_BYTE;
    } else {
      prev_c = c;
    }

    gdl90_ringbuf[gdl90buf_head % GDL90_RINGBUF_SIZE] = c;
    gdl90buf_head++;

    msg_size = 1 /* flag */ + 1 /* id */ + GDL90_MSG_LEN_HEARTBEAT + 2 /* FC */ + 1 /* flag */;
    gdl90buf_tail = gdl90buf_head - msg_size;
    if (c == GDL90_FLAG_BYTE && 
        gdl90_ringbuf[ gdl90buf_tail    % GDL90_RINGBUF_SIZE] == GDL90_FLAG_BYTE &&
        gdl90_ringbuf[(gdl90buf_tail+1) % GDL90_RINGBUF_SIZE] == MSG_ID_HEARTBEAT) {

      unsigned char *buf = (unsigned char *) &message;
      for (uint8_t i=0; i < msg_size; i++) {
          buf[i] = gdl90_ringbuf[(gdl90buf_tail + i) % GDL90_RINGBUF_SIZE];
      }

      decode_gdl90_heartbeat(&message, &heartbeat);
//    print_gdl90_heartbeat(&heartbeat);
    }

    msg_size = 1 /* flag */ + 1 /* id */ + GDL90_MSG_LEN_OWNSHIP_GEOMETRIC + 2 /* FC */ + 1 /* flag */;
    gdl90buf_tail = gdl90buf_head - msg_size;
    if (c == GDL90_FLAG_BYTE && 
        gdl90_ringbuf[ gdl90buf_tail    % GDL90_RINGBUF_SIZE] == GDL90_FLAG_BYTE &&
        gdl90_ringbuf[(gdl90buf_tail+1) % GDL90_RINGBUF_SIZE] == MSG_ID_OWNSHIP_GEOMETRIC) {

      unsigned char *buf = (unsigned char *) &message;
      for (uint8_t i=0; i < msg_size; i++) {
          buf[i] = gdl90_ringbuf[(gdl90buf_tail + i) % GDL90_RINGBUF_SIZE];
      }

      decode_gdl90_ownship_geo_altitude(&message, &geo_altitude);
//    print_gdl90_ownship_geo_altitude(&geo_altitude);
    }

    msg_size = 1 /* flag */ + 1 /* id */ + GDL90_MSG_LEN_TRAFFIC_REPORT + 2 /* FC */ + 1 /* flag */;
    gdl90buf_tail = gdl90buf_head - msg_size;
    if (c == GDL90_FLAG_BYTE && 
        gdl90_ringbuf[ gdl90buf_tail    % GDL90_RINGBUF_SIZE] == GDL90_FLAG_BYTE &&
        gdl90_ringbuf[(gdl90buf_tail+1) % GDL90_RINGBUF_SIZE] == MSG_ID_TRAFFIC_REPORT) {

      unsigned char *buf = (unsigned char *) &message;
      for (uint8_t i=0; i < msg_size; i++) {
          buf[i] = gdl90_ringbuf[(gdl90buf_tail + i) % GDL90_RINGBUF_SIZE];
      }

      decode_gdl90_traffic_report(&message, &traffic);
//    print_gdl90_traffic_report(&traffic);
    }

    msg_size = 1 /* flag */ + 1 /* id */ + GDL90_MSG_LEN_OWNSHIP_REPORT + 2 /* FC */ + 1 /* flag */;
    gdl90buf_tail = gdl90buf_head - msg_size;
    if (c == GDL90_FLAG_BYTE && 
        gdl90_ringbuf[ gdl90buf_tail    % GDL90_RINGBUF_SIZE] == GDL90_FLAG_BYTE &&
        gdl90_ringbuf[(gdl90buf_tail+1) % GDL90_RINGBUF_SIZE] == MSG_ID_OWNSHIP_REPORT) {

      unsigned char *buf = (unsigned char *) &message;
      for (uint8_t i=0; i < msg_size; i++) {
          buf[i] = gdl90_ringbuf[(gdl90buf_tail + i) % GDL90_RINGBUF_SIZE];
      }

      decode_gdl90_traffic_report(&message, &ownship);
//    print_gdl90_traffic_report(&ownship);
    }
  }
}
