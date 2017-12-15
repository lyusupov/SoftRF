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

#include <nmealib/gpvtg.h>

#include <nmealib/context.h>
#include <nmealib/nmath.h>
#include <nmealib/sentence.h>
#include <nmealib/util.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

bool nmeaGPVTGParse(const char *s, const size_t sz, NmeaGPVTG *pack) {
  size_t tokenCount;
  bool speedK = false;
  bool speedN = false;

  if (!s //
      || !sz //
      || !pack) {
    return false;
  }

  nmeaContextTraceBuffer(s, sz);

  /* Clear before parsing, to be able to detect absent fields */
  memset(pack, 0, sizeof(*pack));
  pack->track = NaN;
  pack->mtrack = NaN;
  pack->spn = NaN;
  pack->spk = NaN;

  /* parse */
  tokenCount = nmeaScanf(s, sz, //
      "$" NMEALIB_GPVTG_PREFIX ",%f,%C,%f,%C,%f,%C,%f,%C*", //
      &pack->track, //
      &pack->trackT, //
      &pack->mtrack, //
      &pack->mtrackM, //
      &pack->spn, //
      &pack->spnN, //
      &pack->spk, //
      &pack->spkK);

  /* see that there are enough tokens */
  if (tokenCount != 8) {
    nmeaContextError(NMEALIB_GPVTG_PREFIX " parse error: need 8 tokens, got %lu in '%s'", (long unsigned) tokenCount,
        s);
    goto err;
  }

  /* determine which fields are present and validate them */

  if (!isNaN(pack->track)) {
    if (pack->trackT != 'T') {
      nmeaContextError(NMEALIB_GPVTG_PREFIX " parse error: invalid track unit, got '%c', expected 'T'", pack->trackT);
      goto err;
    }

    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_TRACK);
  } else {
    pack->track = 0.0;
    pack->trackT = '\0';
  }

  if (!isNaN(pack->mtrack)) {
    if (pack->mtrackM != 'M') {
      nmeaContextError(NMEALIB_GPVTG_PREFIX " parse error: invalid mtrack unit, got '%c', expected 'M'",
          pack->mtrackM);
      goto err;
    }

    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_MTRACK);
  } else {
    pack->mtrack = 0.0;
    pack->mtrackM = '\0';
  }

  if (!isNaN(pack->spn)) {
    if (pack->spnN != 'N') {
      nmeaContextError(NMEALIB_GPVTG_PREFIX " parse error: invalid knots speed unit, got '%c', expected 'N'",
          pack->spnN);
      goto err;
    }

    speedN = true;
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_SPEED);
  } else {
    pack->spn = 0.0;
    pack->spnN = '\0';
  }

  if (!isNaN(pack->spk)) {
    if (pack->spkK != 'K') {
      nmeaContextError(NMEALIB_GPVTG_PREFIX " parse error: invalid kph speed unit, got '%c', expected 'K'",
          pack->spkK);
      goto err;
    }

    speedK = true;
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_SPEED);
  } else {
    pack->spk = 0.0;
    pack->spkK = '\0';
  }

  if (!speedK && speedN) {
    pack->spk = pack->spn * NMEALIB_KNOT_TO_KPH;
    pack->spkK = 'K';
  } else if (speedK && !speedN) {
    pack->spn = pack->spk * NMEALIB_KPH_TO_KNOT;
    pack->spnN = 'N';
  }

  return true;

err:
  memset(pack, 0, sizeof(*pack));
  return false;
}

void nmeaGPVTGToInfo(const NmeaGPVTG *pack, NmeaInfo *info) {
  if (!pack //
      || !info) {
    return;
  }

  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SMASK);

  info->smask |= NMEALIB_SENTENCE_GPVTG;

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_TRACK)) {
    info->track = pack->track;
    nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_TRACK);
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_MTRACK)) {
    info->mtrack = pack->mtrack;
    nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_MTRACK);
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_SPEED)) {
    if (pack->spkK) {
      info->speed = pack->spk;
    } else {
      info->speed = pack->spn * NMEALIB_KNOT_TO_KPH;
    }
    nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SPEED);
  }
}

void nmeaGPVTGFromInfo(const NmeaInfo *info, NmeaGPVTG *pack) {
  if (!pack //
      || !info) {
    return;
  }

  memset(pack, 0, sizeof(*pack));

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_TRACK)) {
    pack->track = info->track;
    pack->trackT = 'T';
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_TRACK);
  }

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_MTRACK)) {
    pack->mtrack = info->mtrack;
    pack->mtrackM = 'M';
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_MTRACK);
  }

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_SPEED)) {
    pack->spn = info->speed * NMEALIB_KPH_TO_KNOT;
    pack->spnN = 'N';
    pack->spk = info->speed;
    pack->spkK = 'K';
    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_SPEED);
  }
}

size_t nmeaGPVTGGenerate(char *s, const size_t sz, const NmeaGPVTG *pack) {

#define dst       (&s[chars])
#define available ((sz <= (size_t) chars) ? 0 : (sz - (size_t) chars))

  int chars = 0;

  if (!s //
      || !pack) {
    return 0;
  }

  chars += snprintf(dst, available, "$" NMEALIB_GPVTG_PREFIX);

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_TRACK)) {
    chars += snprintf(dst, available, ",%03.1f", pack->track);
    if (pack->trackT) {
      chars += snprintf(dst, available, ",%c", pack->trackT);
    } else {
      chars += snprintf(dst, available, ",");
    }
  } else {
    chars += snprintf(dst, available, ",,");
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_MTRACK)) {
    chars += snprintf(dst, available, ",%03.1f", pack->mtrack);
    if (pack->mtrackM) {
      chars += snprintf(dst, available, ",%c", pack->mtrackM);
    } else {
      chars += snprintf(dst, available, ",");
    }
  } else {
    chars += snprintf(dst, available, ",,");
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_SPEED)) {
    chars += snprintf(dst, available, ",%03.1f", pack->spn);
    if (pack->spnN) {
      chars += snprintf(dst, available, ",%c", pack->spnN);
    } else {
      chars += snprintf(dst, available, ",");
    }

    chars += snprintf(dst, available, ",%03.1f", pack->spk);
    if (pack->spkK) {
      chars += snprintf(dst, available, ",%c", pack->spkK);
    } else {
      chars += snprintf(dst, available, ",");
    }
  } else {
    chars += snprintf(dst, available, ",,,,");
  }

  /* checksum */
  chars += nmeaAppendChecksum(s, sz, (size_t) chars);

  return (size_t) chars;

#undef available
#undef dst

}
