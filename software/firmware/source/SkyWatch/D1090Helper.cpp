/*
 * D1090Helper.cpp
 * Copyright (C) 2016-2026 Linar Yusupov
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

#include <adsb_encoder.h>
#include <TimeLib.h>
#include <protocol.h>

#include "SoCHelper.h"
#include "D1090Helper.h"
#include "GNSSHelper.h"
#include "EEPROMHelper.h"
#include "TrafficHelper.h"
#include "WiFiHelper.h"

static unsigned long D1090_Data_TimeMarker = 0;
static char D1090Buffer[D1090_BUFFER_SIZE]; // buffer for D1090 data
static int D1090_cnt = 0;

unsigned long D1090_Frames_Count = 0;
unsigned long D1090_Acfts_Count  = 0;

#include <mode-s.h>
mode_s_t modes_state;

static void D1090_Parse_Character(char c)
{
  if (c == -1) {
    /* retry */
    return;
  }

  if (isPrintable(c) || c == '\r' || c == '\n') {
    D1090Buffer[D1090_cnt] = c;
  } else {
    /* ignore */
    return;
  }

  int ndx = D1090_cnt - 31;

  if (ndx >= 0) {
    if (D1090Buffer[ndx   ] == '*'  &&
        D1090Buffer[ndx+29] == ';'  &&
        D1090Buffer[ndx+30] == '\r' &&
        D1090Buffer[ndx+31] == '\n') {

      D1090_Frames_Count++;

      size_t write_size = D1090_cnt - ndx + 1;
      D1090_Out((byte *) &D1090Buffer[ndx], write_size);

      uint8_t buf[14];
      struct mode_s_msg mm;

      uint8_t *msg = (uint8_t *) &D1090Buffer[ndx+1];

      for (int i=0; i<28; i+=2) {
        if (msg[i] == ';') break;

        uint8_t out = 0;
        uint8_t h = msg[i];

        if (isdigit(h)) {
          out |= ((h - '0'     ) << 4);
        } else if (islower(h)) {
          out |= ((h - 'a' + 10) << 4);
        } else {
          out |= ((h - 'A' + 10) << 4);
        }

        uint8_t l = msg[i + 1];

        if (isdigit(l)) {
          out |= (l - '0'     );
        } else if (islower(l)) {
          out |= (l - 'a' + 10);
        } else {
          out |= (l - 'A' + 10);
        }

        buf[i>>1] = out;
      }

      mode_s_decode(&modes_state, &mm, buf);

      if (modes_state.check_crc == 0 || mm.crcok) {

//  printf("%02d %03d %02x%02x%02x\r\n", mm.msgtype, mm.msgbits, mm.aa1, mm.aa2, mm.aa3);

          D1090_Acfts_Count = 0;
          struct mode_s_aircraft *a = modes_state.aircrafts;

          while (a) {
            D1090_Acfts_Count++;
            a = a->next;
          }

          if (D1090_Acfts_Count < MAX_TRACKING_OBJECTS) {
            interactiveReceiveData(&modes_state, &mm);
          }
      }
    }
  }

  if (D1090Buffer[D1090_cnt] == '\n' || D1090_cnt == sizeof(D1090Buffer)-1) {
    D1090_cnt = 0;
  } else {
    D1090_cnt++;
  }
}

void D1090_setup()
{
  if (settings->m.protocol == PROTOCOL_D1090) {

    switch (settings->m.connection)
    {
    case CON_SERIAL_MAIN:
    case CON_SERIAL_AUX:
      uint32_t SerialBaud;

      switch (settings->m.baudrate)
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
      break;
    case CON_BLUETOOTH:
#if 0
      if (SoC->Bluetooth_ops) {
        SoC->Bluetooth_ops->setup();
      }
#endif
      break;
    case CON_USB:
    case CON_NONE:
    case CON_WIFI_UDP:
    default:
      break;
    }

    mode_s_init(&modes_state);

    D1090_Data_TimeMarker = millis();
  }
}

void D1090_loop()
{
  size_t size;

  switch (settings->m.connection)
  {
  case CON_SERIAL_MAIN:
    while (SerialInput.available() > 0) {
      char c = SerialInput.read();
//      Serial.print(c);
      D1090_Parse_Character(c);
      D1090_Data_TimeMarker = millis();
    }
    break;
  case CON_SERIAL_AUX:
    /* read data from Type-C USB port */
    while (Serial.available() > 0) {
      char c = Serial.read();
//        Serial.print(c);
      D1090_Parse_Character(c);
      D1090_Data_TimeMarker = millis();
    }
    break;
  case CON_USB:
    /* read data from Type-C USB port in Host mode */
    if (SoC->USB_ops) {
      while (SoC->USB_ops->available() > 0) {
        char c = SoC->USB_ops->read();
//        Serial.print(c);
        D1090_Parse_Character(c);
        D1090_Data_TimeMarker = millis();
      }
    }
    break;
  case CON_WIFI_UDP:
    size = SoC->WiFi_Receive_UDP((uint8_t *) UDPpacketBuffer, sizeof(UDPpacketBuffer));
    if (size > 0) {
      for (size_t i=0; i < size; i++) {
//        Serial.print(UDPpacketBuffer[i]);
        D1090_Parse_Character(UDPpacketBuffer[i]);
      }
      D1090_Data_TimeMarker = millis();
    }
    break;
  case CON_BLUETOOTH:
    if (SoC->Bluetooth_ops) {
      while (SoC->Bluetooth_ops->available() > 0) {
        char c = SoC->Bluetooth_ops->read();
//        Serial.print(c);
        D1090_Parse_Character(c);
        D1090_Data_TimeMarker = millis();
      }
    }
    break;
  case CON_NONE:
  default:
    break;
  }

  interactiveRemoveStaleAircrafts(&modes_state);

  D1090_Acfts_Count = 0;
  struct mode_s_aircraft *a = modes_state.aircrafts;

  while (a) {
    D1090_Acfts_Count++;
    a = a->next;
  }
}

void D1090_Out(byte *buf, size_t size)
{
  if (size > 0) {
    switch(settings->m.data_dest)
    {
    case D1090_UART:
    case D1090_USB:
      Serial.write(buf, size);
      break;
    case D1090_BLUETOOTH:
      {
        if (SoC->Bluetooth_ops) {
          SoC->Bluetooth_ops->write(buf, size);
        }
      }
      break;
    case D1090_UDP:
      {
        SoC->WiFi_transmit_UDP(D1090_DST_PORT, buf, size);
      }
      break;
    case D1090_TCP:
    case D1090_OFF:
    default:
      break;
    }
  }
}
