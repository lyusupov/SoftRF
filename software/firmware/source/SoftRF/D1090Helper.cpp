/*
 * D1090Helper.cpp
 * Copyright (C) 2016-2020 Linar Yusupov
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

#include "SoCHelper.h"
#include "D1090Helper.h"
#include "GNSSHelper.h"
#include "GDL90Helper.h"
#include "EEPROMHelper.h"
#include "TrafficHelper.h"

#define ADDR_TO_HEX_STR(s, c) (s += ((c) < 0x10 ? "0" : "") + String((c), HEX))

#define DF17_FRAME_TO_HEX_STR(s)                        \
      ({                                                \
        for (int i=0; i < sizeof(frame_data_t); i++) {  \
          byte c = df17.msg[i];                         \
          s += (c < 0x10 ? "0" : "") + String(c, HEX);  \
        }                                               \
      })

static void D1090_Out(byte *buf, size_t size)
{
  switch(settings->d1090)
  {
  case D1090_UART:
    {
      SerialOutput.write(buf, size);
    }
    break;
  case D1090_BLUETOOTH:
    {
      if (SoC->Bluetooth) {
        SoC->Bluetooth->write(buf, size);
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

  if (settings->d1090 != D1090_OFF) {
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].addr && (this_moment - Container[i].timestamp) <= EXPORT_EXPIRATION_TIME) {

        distance = Container[i].distance;

        if (distance < ALARM_ZONE_NONE) {

          double altitude;
          /* If the aircraft's data has standard pressure altitude - make use it */
          if (Container[i].pressure_altitude != 0.0) {
            altitude = (double) Container[i].pressure_altitude;
          } else if (ThisAircraft.pressure_altitude != 0.0) {
            /* If this SoftRF unit is equiped with baro sensor - try to make an adjustment */
            float altDiff = ThisAircraft.pressure_altitude - ThisAircraft.altitude;
            altitude = (double)(Container[i].altitude + altDiff);
          } else {
            /* If no other choice - report GNSS altitude as pressure altitude */
            altitude = (double) Container[i].altitude;
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
