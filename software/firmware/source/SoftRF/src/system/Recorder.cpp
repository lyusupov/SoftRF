/*
 * RecorderHelper.cpp
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

#include "SoC.h"

#if !defined(ENABLE_RECORDER)
void Recorder_setup()   {}
void Recorder_loop()    {}
void Recorder_fini()    {}
#else

#include <SdFat.h>
#include <FlightRecorder.h>

#include "Recorder.h"
#include "../driver/GNSS.h"
#include "../driver/Baro.h"

extern SdFat uSD;

FlightRecorder FR;

bool FR_is_active = false;

static const char *m10s_specs = "u-blox,MAX-M10S,49,50000,GPS,GLO,BDS,GAL";
static const char *l76k_specs = "Quectel,L76K,32,50000,GPS,GLO,BDS";

void Recorder_setup()
{
  const char *gnss_specs = hw_info.gnss == GNSS_MODULE_U10  ? m10s_specs :
                           hw_info.gnss == GNSS_MODULE_AT65 ? l76k_specs :
                           "Generic NMEA";

  if (hw_info.gnss     != GNSS_MODULE_NONE &&
      hw_info.baro     != BARO_MODULE_NONE &&
      (hw_info.storage == STORAGE_CARD     ||
       hw_info.storage == STORAGE_FLASH_AND_CARD)) {
    if (!FR_is_active && uSD.volumeBegin()) {
      FR_is_active = FR.begin(&uSD, SoC->getChipId(), gnss_specs);
    }
  }
}

void Recorder_loop()
{
  if (FR_is_active) {
    FR.loop(&gnss, ThisAircraft.pressure_altitude);
  }
}

void Recorder_fini()
{
  if (FR_is_active) {
    FR.end();
    FR_is_active = false;
  }
}

#endif /* ENABLE_RECORDER */
