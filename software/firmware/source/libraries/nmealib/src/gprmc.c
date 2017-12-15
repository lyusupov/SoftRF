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

#include <nmealib/gprmc.h>

#include <nmealib/context.h>
#include <nmealib/nmath.h>
#include <nmealib/sentence.h>
#include <nmealib/util.h>
#include <nmealib/validate.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

bool nmeaGPRMCParse(const char *s, const size_t sz, NmeaGPRMC *pack) {
  size_t tokenCount;
  char timeBuf[16];
  char dateBuf[16];
  bool v23Saved;

  if (!s //
      || !sz //
      || !pack) {
    return false;
  }

  nmeaContextTraceBuffer(s, sz);

  /* Clear before parsing, to be able to detect absent fields */
  memset(timeBuf, 0, sizeof(timeBuf));
  memset(dateBuf, 0, sizeof(dateBuf));
  memset(pack, 0, sizeof(*pack));
  pack->latitude = NaN;
  pack->longitude = NaN;
  pack->speed = NaN;
  pack->track = NaN;
  pack->magvar = NaN;

  /* parse */
  tokenCount = nmeaScanf(s, sz, //
      "$GPRMC,%16s,%C,%F,%C,%F,%C,%f,%f,%16s,%F,%C,%C*", //
      timeBuf, //
      &pack->sigSelection, //
      &pack->latitude, //
      &pack->latitudeNS, //
      &pack->longitude, //
      &pack->longitudeEW, //
      &pack->speed, //
      &pack->track, //
      dateBuf, //
      &pack->magvar, //
      &pack->magvarEW, //
      &pack->sig);

  /* see that there are enough tokens */
  if ((tokenCount != 11) //
      && (tokenCount != 12)) {
    nmeaContextError(NMEALIB_GPRMC_PREFIX " parse error: need 11 or 12 tokens, got %lu in '%s'",
        (long unsigned) tokenCount, s);
    goto err;
  }

  pack->v23 = (tokenCount == 12);

  /* determine which fields are present and validate them */

  if (*timeBuf) {
    if (!nmeaTimeParseTime(timeBuf, &pack->utc) //
        || !nmeaValidateTime(&pack->utc, NMEALIB_GPRMC_PREFIX, s)) {
      goto err;
    }

    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_UTCTIME);
  } else {
    pack->utc.hour = 0;
    pack->utc.min = 0;
    pack->utc.sec = 0;
    pack->utc.hsec = 0;
  }

  if (pack->sigSelection //
      && (pack->sigSelection != 'A') //
      && (pack->sigSelection != 'V')) {
    nmeaContextError(NMEALIB_GPRMC_PREFIX " parse error: invalid status '%c' in '%s'", pack->sigSelection, s);
    goto err;
  }

  if (!pack->v23) {
    /* no mode */
    if (pack->sigSelection) {
      pack->sig = '\0';
      nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_SIG);
    } else {
      pack->sigSelection = '\0';
      pack->sig = '\0';
    }
  } else {
    /* with mode */
    if (pack->sigSelection //
        && pack->sig) {
      if (!nmeaValidateMode(pack->sig, NMEALIB_GPRMC_PREFIX, s)) {
        goto err;
      }

      nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_SIG);
    } else {
      pack->sigSelection = '\0';
      pack->sig = '\0';
    }
  }

  if (!isNaN(pack->latitude)) {
    if (!nmeaValidateNSEW(pack->latitudeNS, true, NMEALIB_GPRMC_PREFIX, s)) {
      goto err;
    }

    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_LAT);
  } else {
    pack->latitude = 0.0;
    pack->latitudeNS = '\0';
  }

  if (!isNaN(pack->longitude)) {
    if (!nmeaValidateNSEW(pack->longitudeEW, false, NMEALIB_GPRMC_PREFIX, s)) {
      goto err;
    }

    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_LON);
  } else {
    pack->longitude = 0.0;
    pack->longitudeEW = '\0';
  }

  if (!isNaN(pack->speed)) {
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_SPEED);
  } else {
    pack->speed = 0.0;
  }

  if (!isNaN(pack->track)) {
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_TRACK);
  } else {
    pack->track = 0.0;
  }

  if (*dateBuf) {
    if (!nmeaTimeParseDate(dateBuf, &pack->utc) //
        || !nmeaValidateDate(&pack->utc, NMEALIB_GPRMC_PREFIX, s)) {
      goto err;
    }

    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_UTCDATE);
  } else {
    pack->utc.year = 0;
    pack->utc.mon = 0;
    pack->utc.day = 0;
  }

  if (!isNaN(pack->magvar)) {
    if (!nmeaValidateNSEW(pack->magvarEW, false, NMEALIB_GPRMC_PREFIX, s)) {
      goto err;
    }

    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_MAGVAR);
  } else {
    pack->magvar = 0.0;
    pack->magvarEW = '\0';
  }

  return true;

err:
  v23Saved = pack->v23;
  memset(pack, 0, sizeof(*pack));
  pack->v23 = v23Saved;
  return false;
}

void nmeaGPRMCToInfo(const NmeaGPRMC *pack, NmeaInfo *info) {
  if (!pack //
      || !info) {
    return;
  }

  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SMASK);

  info->smask |= NMEALIB_SENTENCE_GPRMC;

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_UTCTIME)) {
    info->utc.hour = pack->utc.hour;
    info->utc.min = pack->utc.min;
    info->utc.sec = pack->utc.sec;
    info->utc.hsec = pack->utc.hsec;
    nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_UTCTIME);
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_SIG)) {
    if (!pack->v23) {
      /* no mode */
      if ((pack->sigSelection == 'A') //
          && (info->sig == NMEALIB_SIG_INVALID)) {
        info->sig = NMEALIB_SIG_FIX;
        nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SIG);
      }
    } else {
      /* with mode */

      if (pack->sigSelection != 'A') {
        info->sig = NMEALIB_SIG_INVALID;
      } else {
        info->sig = nmeaInfoModeToSignal(pack->sig);
      }
      nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SIG);
    }
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_LAT)) {
    info->latitude = ((pack->latitudeNS == 'N') ?
        pack->latitude :
        -pack->latitude);
    nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_LAT);
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_LON)) {
    info->longitude = ((pack->longitudeEW == 'E') ?
        pack->longitude :
        -pack->longitude);
    nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_LON);
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_SPEED)) {
    info->speed = pack->speed * NMEALIB_KNOT_TO_KPH;
    nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SPEED);
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_TRACK)) {
    info->track = pack->track;
    nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_TRACK);
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_UTCDATE)) {
    info->utc.year = pack->utc.year;
    info->utc.mon = pack->utc.mon;
    info->utc.day = pack->utc.day;
    nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_UTCDATE);
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_MAGVAR)) {
    info->magvar = ((pack->magvarEW == 'E') ?
        pack->magvar :
        -pack->magvar);
    nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_MAGVAR);
  }
}

void nmeaGPRMCFromInfo(const NmeaInfo *info, NmeaGPRMC *pack) {
  if (!pack //
      || !info) {
    return;
  }

  memset(pack, 0, sizeof(*pack));

  pack->v23 = true;

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_UTCTIME)) {
    pack->utc.hour = info->utc.hour;
    pack->utc.min = info->utc.min;
    pack->utc.sec = info->utc.sec;
    pack->utc.hsec = info->utc.hsec;
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_UTCTIME);
  }

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_SIG)) {
    pack->sigSelection = ((info->sig != NMEALIB_SIG_INVALID) ?
        'A' :
        'V');
    pack->sig = nmeaInfoSignalToMode(info->sig);
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_SIG);
  }

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_LAT)) {
    pack->latitude = fabs(info->latitude);
    pack->latitudeNS = ((info->latitude >= 0.0) ?
        'N' :
        'S');
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_LAT);
  }

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_LON)) {
    pack->longitude = fabs(info->longitude);
    pack->longitudeEW = ((info->longitude >= 0.0) ?
        'E' :
        'W');
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_LON);
  }

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_SPEED)) {
    pack->speed = info->speed * NMEALIB_KPH_TO_KNOT;
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_SPEED);
  }

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_TRACK)) {
    pack->track = info->track;
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_TRACK);
  }

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_UTCDATE)) {
    pack->utc.year = info->utc.year;
    pack->utc.mon = info->utc.mon;
    pack->utc.day = info->utc.day;
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_UTCDATE);
  }

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_MAGVAR)) {
    pack->magvar = fabs(info->magvar);
    pack->magvarEW = ((info->magvar >= 0.0) ?
        'E' :
        'W');
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_MAGVAR);
  }
}

size_t nmeaGPRMCGenerate(char *s, const size_t sz, const NmeaGPRMC *pack) {

#define dst       (&s[chars])
#define available ((sz <= (size_t) chars) ? 0 : (sz - (size_t) chars))

  int chars = 0;

  if (!s //
      || !pack) {
    return 0;
  }

  chars += snprintf(dst, available, "$" NMEALIB_GPRMC_PREFIX);

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_UTCTIME)) {
    chars += snprintf(dst, available, //
        ",%02u%02u%02u.%02u", //
        pack->utc.hour, //
        pack->utc.min, //
        pack->utc.sec, //
        pack->utc.hsec);
  } else {
    chars += snprintf(dst, available, ",");
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_SIG) //
      && pack->sigSelection) {
    chars += snprintf(dst, available, ",%c", pack->sigSelection);
  } else {
    chars += snprintf(dst, available, ",");
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_LAT)) {
    chars += snprintf(dst, available, ",%09.4f", pack->latitude);
    if (pack->latitudeNS) {
      chars += snprintf(dst, available, ",%c", pack->latitudeNS);
    } else {
      chars += snprintf(dst, available, ",");
    }
  } else {
    chars += snprintf(dst, available, ",,");
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_LON)) {
    chars += snprintf(dst, available, ",%010.4f", pack->longitude);
    if (pack->longitudeEW) {
      chars += snprintf(dst, available, ",%c", pack->longitudeEW);
    } else {
      chars += snprintf(dst, available, ",");
    }
  } else {
    chars += snprintf(dst, available, ",,");
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_SPEED)) {
    chars += snprintf(dst, available, ",%03.1f", pack->speed);
  } else {
    chars += snprintf(dst, available, ",");
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_TRACK)) {
    chars += snprintf(dst, available, ",%03.1f", pack->track);
  } else {
    chars += snprintf(dst, available, ",");
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_UTCDATE)) {
    chars += snprintf(dst, available, //
        ",%02u%02u%02u", //
        pack->utc.day, //
        pack->utc.mon, //
        pack->utc.year % 100);
  } else {
    chars += snprintf(dst, available, ",");
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_MAGVAR)) {
    chars += snprintf(dst, available, ",%03.1f", pack->magvar);
    if (pack->magvarEW) {
      chars += snprintf(dst, available, ",%c", pack->magvarEW);
    } else {
      chars += snprintf(dst, available, ",");
    }
  } else {
    chars += snprintf(dst, available, ",,");
  }

  if (pack->v23) {
    if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_SIG) //
        && pack->sig) {
      chars += snprintf(dst, available, ",%c", pack->sig);
    } else {
      chars += snprintf(dst, available, ",");
    }
  }

  /* checksum */
  chars += nmeaAppendChecksum(s, sz, (size_t) chars);

  return (size_t) chars;

#undef available
#undef dst

}
