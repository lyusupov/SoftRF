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

static void D1090_Parse_Character(char c)
{
  /* TODO */
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
