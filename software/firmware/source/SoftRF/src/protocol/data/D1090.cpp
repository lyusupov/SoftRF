/*
 * D1090Helper.cpp
 * Copyright (C) 2016-2022 Linar Yusupov
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

#include "../../system/SoC.h"
#include "D1090.h"
#include "../../driver/GNSS.h"
#include "GDL90.h"
#include "../../driver/EEPROM.h"
#include "../../TrafficHelper.h"

#define ADDR_TO_HEX_STR(s, c) (s += ((c) < 0x10 ? "0" : "") + String((c), HEX))

#define DF17_FRAME_TO_HEX_STR(s)                        \
      ({                                                \
        for (int i=0; i < sizeof(frame_data_t); i++) {  \
          byte c = df17.msg[i];                         \
          s += (c < 0x10 ? "0" : "") + String(c, HEX);  \
        }                                               \
      })

#if defined(ENABLE_D1090_INPUT)
#include "../radio/ES1090.h"

extern mode_s_t state;
#endif /* ENABLE_D1090_INPUT */

static void D1090_Out(byte *buf, size_t size)
{
  switch(settings->d1090)
  {
  case D1090_UART:
    if (SoC->UART_ops) {
      SoC->UART_ops->write(buf, size);
    } else {
      SerialOutput.write(buf, size);
    }
    break;
  case D1090_USB:
    {
      if (SoC->USB_ops) {
        SoC->USB_ops->write(buf, size);
      }
    }
    break;
  case D1090_BLUETOOTH:
    {
      if (SoC->Bluetooth_ops) {
        SoC->Bluetooth_ops->write(buf, size);
      }
    }
    break;
  case D1090_UDP:
  case D1090_TCP:
  case D1090_OFF:
  default:
    break;
  }
}

void D1090_Export()
{
  frame_data_t df17;
  float distance;
  String str;
  time_t this_moment = now();

#if defined(ENABLE_D1090_INPUT) || \
    defined(ENABLE_RTLSDR) || defined(ENABLE_HACKRF) || defined(ENABLE_MIRISDR)
  struct mode_s_aircraft *a;

  for (a = state.aircrafts; a; a = a->next) {
    if (a->even_cprtime && a->odd_cprtime &&
        abs((long) (a->even_cprtime - a->odd_cprtime)) <= MODE_S_INTERACTIVE_TTL * 1000 ) {
      if (es1090_decode(a, &ThisAircraft, &fo)) {
        memset(fo.raw, 0, sizeof(fo.raw));
        Traffic_Update(&fo);
        Traffic_Add(&fo);
      }
    }
  }

  interactiveRemoveStaleAircrafts(&state);
#endif /* ENABLE_D1090_INPUT || ENABLE_RTLSDR || ENABLE_HACKRF || ENABLE_MIRISDR */

  if (settings->d1090 != D1090_OFF && isValidFix()) {
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].addr && (this_moment - Container[i].timestamp) <= EXPORT_EXPIRATION_TIME) {

        distance = Container[i].distance;

        if (distance < ALARM_ZONE_NONE) {

          float altitude;
          /* If the aircraft's data has standard pressure altitude - make use it */
          if (Container[i].pressure_altitude != 0.0) {
            altitude = Container[i].pressure_altitude;
          } else if (ThisAircraft.pressure_altitude != 0.0) {
            /* If this SoftRF unit is equiped with baro sensor - try to make an adjustment */
            float altDiff = ThisAircraft.pressure_altitude - ThisAircraft.altitude;
            altitude = Container[i].altitude + altDiff;
          } else {
            /* If no other choice - report GNSS altitude as pressure altitude */
            altitude = Container[i].altitude;
          }
          altitude *= _GPS_FEET_PER_METER;

          df17 = make_air_position_frame(11, Container[i].addr,
            Container[i].latitude, Container[i].longitude,
            altitude, CPR_EVEN, DF17);

          str = "*";
          DF17_FRAME_TO_HEX_STR(str);
          str += ";\r\n*";

          df17 = make_air_position_frame(11, Container[i].addr,
            Container[i].latitude, Container[i].longitude,
            altitude, CPR_ODD, DF17);

          DF17_FRAME_TO_HEX_STR(str);
          str += ";\r\n*";

          String callsign = String(GDL90_CallSign_Prefix[Container[i].protocol]);
        
          ADDR_TO_HEX_STR(callsign, (Container[i].addr >> 16) & 0xFF);
          ADDR_TO_HEX_STR(callsign, (Container[i].addr >>  8) & 0xFF);
          ADDR_TO_HEX_STR(callsign, (Container[i].addr      ) & 0xFF);

          callsign.toUpperCase();

          df17 = make_aircraft_identification_frame(Container[i].addr,
            (unsigned char*) callsign.c_str(),
            Category_Set_D,
            AT_TO_GDL90(Container[i].aircraft_type),
            DF17);

          DF17_FRAME_TO_HEX_STR(str);
          str += ";\r\n*";

          df17 = make_velocity_frame(Container[i].addr,
            Container[i].speed * cos(Container[i].course * PI / 180),
            Container[i].speed * sin(Container[i].course * PI / 180),
            Container[i].vs,
            DF17);

          DF17_FRAME_TO_HEX_STR(str);
          str.toUpperCase();
          str += ";\r\n";

          D1090_Out((byte *) str.c_str(), str.length());
        }
      }
    }
  }
}

#if defined(ENABLE_D1090_INPUT)

void D1090_Import(uint8_t *msg)
{
  uint8_t buf[14];
  struct mode_s_msg mm;

  for (int i=0; i<28; i+=2) {
    if (msg[1 + i] == ';') break;

    uint8_t out = 0;
    uint8_t h = msg[1 + i];

    if (isdigit(h)) {
      out |= ((h - '0'     ) << 4);
    } else if (islower(h)) {
      out |= ((h - 'a' + 10) << 4);
    } else {
      out |= ((h - 'A' + 10) << 4);
    }

    uint8_t l = msg[1 + i + 1];

    if (isdigit(l)) {
      out |= (l - '0'     );
    } else if (islower(l)) {
      out |= (l - 'a' + 10);
    } else {
      out |= (l - 'A' + 10);
    }

    buf[i>>1] = out;
  }

  mode_s_decode(&state, &mm, buf);

  if (state.check_crc == 0 || mm.crcok) {

//  printf("%02d %03d %02x%02x%02x\r\n", mm.msgtype, mm.msgbits, mm.aa1, mm.aa2, mm.aa3);

      int acfts_in_sight = 0;
      struct mode_s_aircraft *a = state.aircrafts;

      while (a) {
        acfts_in_sight++;
        a = a->next;
      }

      if (acfts_in_sight < MAX_TRACKING_OBJECTS) {
        interactiveReceiveData(&state, &mm);
      }
  }
}

#endif /* ENABLE_D1090_INPUT */
