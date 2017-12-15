/*
 * This file is part of nmealib.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <nmealib/validate.h>

#include <nmealib/context.h>

/** Invalid NMEA character: non-ASCII */
static const NmeaInvalidCharacter nmealibInvalidNonAsciiCharsName = {
    .character = '*', //
    .description = "non-ASCII character" //
    };

/** Invalid NMEA character/description pairs */
static const NmeaInvalidCharacter nmealibInvalidCharacters[] = {
    {
        .character = '$', //
        .description = "sentence delimiter" //
    },
    {
        .character = '*', //
        .description = "checksum field delimiter" //
    },
    {
        .character = '!', //
        .description = "exclamation mark" //
    },
    {
        .character = '\\', //
        .description = "backslash" //
    },
    {
        .character = '^', //
        .description = "power" //
    },
    {
        .character = '~', //
        .description = "tilde" //
    },
    {
        .character = '\0', //
        .description = NULL //
    }//
};

const NmeaInvalidCharacter *nmeaValidateIsInvalidCharacter(const char c) {
  size_t i = 0;

  if ((c < 32) //
      || (c > 126)) {
    return &nmealibInvalidNonAsciiCharsName;
  }

  while (nmealibInvalidCharacters[i].description) {
    if (c == nmealibInvalidCharacters[i].character) {
      return &nmealibInvalidCharacters[i];
    }

    i++;
  }

  return NULL;
}

const NmeaInvalidCharacter *nmeaValidateSentenceHasInvalidCharacters(const char *s, const size_t sz) {
  size_t i = 0;

  if (!s //
      || !sz) {
    return NULL;
  }

  for (i = 0; i < sz; i++) {
    const NmeaInvalidCharacter *invalid = nmeaValidateIsInvalidCharacter(s[i]);
    if (invalid) {
      return invalid;
    }
  }

  return NULL;
}

bool nmeaValidateTime(const NmeaTime *t, const char *prefix, const char *s) {
  if (!t) {
    return false;
  }

  if ((t->hour > 23) //
      || (t->min > 59) //
      || (t->sec > 60) //
      || (t->hsec > 99)) {
    nmeaContextError("%s parse error: invalid time '%02u:%02u:%02u.%03u' (hh:mm:ss.mmm) in '%s'", prefix, t->hour,
        t->min, t->sec, t->hsec * 10, s);
    return false;
  }

  return true;
}

bool nmeaValidateDate(const NmeaTime *t, const char *prefix, const char *s) {
  if (!t) {
    return false;
  }

  if ((t->year < 1900) //
      || (t->year > 2089) //
      || (t->mon < 1) //
      || (t->mon > 12) //
      || (t->day < 1) //
      || (t->day > 31)) {
    nmeaContextError("%s parse error: invalid date '%02u-%02u-%04u' (dd-mm-yyyy) in '%s'", prefix, t->day, t->mon,
        t->year, s);
    return false;
  }

  return true;
}

bool nmeaValidateNSEW(char c, const bool ns, const char *prefix, const char *s) {
  char cu[] = {
      0,
      0,
      0 };

  if (c) {
    cu[0] = c;
  } else {
    cu[0] = '\\';
    cu[1] = '0';
  }

  if (ns) {
    if ((c != 'N') //
        && (c != 'S')) {
      nmeaContextError("%s parse error: invalid North/South '%s' in '%s'", prefix, cu, s);
      return false;
    }
  } else {
    if ((c != 'E') //
        && (c != 'W')) {
      nmeaContextError("%s parse error: invalid East/West '%s' in '%s'", prefix, cu, s);
      return false;
    }
  }

  return true;
}

bool nmeaValidateFix(NmeaFix fix, const char *prefix, const char *s) {
  if ((fix < NMEALIB_FIX_FIRST) //
      || (fix > NMEALIB_FIX_LAST)) {
    nmeaContextError("%s parse error: invalid fix %d, expected [%d, %d] in '%s'", prefix, fix, NMEALIB_FIX_FIRST,
        NMEALIB_FIX_LAST, s);
    return false;
  }

  return true;
}

bool nmeaValidateSignal(NmeaSignal sig, const char *prefix, const char *s) {
  if ((sig < NMEALIB_SIG_FIRST) //
      || (sig > NMEALIB_SIG_LAST)) {
    nmeaContextError("%s parse error: invalid signal %d, expected [%d, %d] in '%s'", prefix, sig, NMEALIB_SIG_FIRST,
        NMEALIB_SIG_LAST, s);
    return false;
  }

  return true;
}

bool nmeaValidateMode(char c, const char *prefix, const char *s) {
  if (!c) {
    return false;
  }

  if ((c != 'N') //
      && (c != 'A') //
      && (c != 'D') //
      && (c != 'P') //
      && (c != 'R') //
      && (c != 'F') //
      && (c != 'E') //
      && (c != 'M') //
      && (c != 'S')) {
    nmeaContextError("%s parse error: invalid mode '%c' in '%s'", prefix, c, s);
    return false;
  }

  return true;
}

bool nmeaValidateSatellite(NmeaSatellite *sat, const char *prefix, const char *s) {
  if (!sat) {
    return false;
  }

  if ((sat->elevation < -180) //
      || (sat->elevation > 180)) {
    nmeaContextError("%s parse error: invalid satellite elevation %d in '%s'", prefix, sat->elevation, s);
    return false;
  }

  if (sat->azimuth > 359) {
    nmeaContextError("%s parse error: invalid satellite azimuth %u in '%s'", prefix, sat->azimuth, s);
    return false;
  }

  if (sat->snr > 99) {
    nmeaContextError("%s parse error: invalid satellite signal %u in '%s'", prefix, sat->snr, s);
    return false;
  }

  return true;
}
