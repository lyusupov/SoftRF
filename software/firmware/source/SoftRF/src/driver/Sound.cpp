/*
 * SoundHelper.cpp
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

#include "../system/SoC.h"

#if defined(EXCLUDE_SOUND)
void  Sound_setup()       {}
bool  Sound_Notify()      {return true;}
void  Sound_loop()        {}
void  Sound_fini()        {}
#else

#include "Sound.h"
#include "EEPROM.h"

static unsigned long SoundTimeMarker = 0;

void Sound_setup(void)
{
  SoC->Sound_tone(0, settings->volume);
  SoundTimeMarker = 0;
}

bool Sound_Notify(void)
{
  bool rval = false;

  if (SoundTimeMarker == 0) {
    SoC->Sound_tone(ALARM_TONE_HZ, settings->volume);
    SoundTimeMarker = millis();
    rval = true;
  }

  return rval;
}

void Sound_loop(void)
{
  if (SoundTimeMarker != 0 && millis() - SoundTimeMarker > ALARM_TONE_MS) {
    SoC->Sound_tone(0, settings->volume);
    SoundTimeMarker = 0;
  }
}

void Sound_fini(void)
{
  Sound_setup();
}

#endif /* EXCLUDE_SOUND */
