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

#include <nmealib/gpgsv.h>

#include <nmealib/context.h>
#include <nmealib/sentence.h>
#include <nmealib/validate.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

size_t nmeaGPGSVsatellitesToSentencesCount(const size_t satellites) {
  size_t sentenceCount;

  if (!satellites) {
    return 1;
  }

  sentenceCount = satellites >> NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE_SHIFT;

  if (satellites & NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE_MOD_MASK) {
    sentenceCount++;
  }

  return sentenceCount;
}

bool nmeaGPGSVParse(const char *s, const size_t sz, NmeaGPGSV *pack) {

#define sat0 pack->inView[0]
#define sat1 pack->inView[1]
#define sat2 pack->inView[2]
#define sat3 pack->inView[3]

  size_t tokenCount;
  size_t tokenCountExpected;
  size_t satellitesInSentence;
  size_t i;

  if (!pack) {
    return false;
  }

  memset(pack, 0, sizeof(*pack));

  if (!s //
      || !sz) {
    return false;
  }

  nmeaContextTraceBuffer(s, sz);

  /* Clear before parsing, to be able to detect absent fields */
  pack->sentenceCount = UINT_MAX;
  pack->sentence = UINT_MAX;
  pack->inViewCount = UINT_MAX;

  /* parse */
  tokenCount = nmeaScanf(s, sz, //
      "$" NMEALIB_GPGSV_PREFIX ",%u,%u,%u" //
      ",%u,%d,%u,%u"//
      ",%u,%d,%u,%u"//
      ",%u,%d,%u,%u"//
      ",%u,%d,%u,%u*",//
      &pack->sentenceCount, &pack->sentence, &pack->inViewCount, //
      &sat0.prn, &sat0.elevation, &sat0.azimuth, &sat0.snr, //
      &sat1.prn, &sat1.elevation, &sat1.azimuth, &sat1.snr, //
      &sat2.prn, &sat2.elevation, &sat2.azimuth, &sat2.snr, //
      &sat3.prn, &sat3.elevation, &sat3.azimuth, &sat3.snr);

  if ((pack->sentenceCount == UINT_MAX) //
      || (pack->sentence == UINT_MAX) //
      || (pack->inViewCount == UINT_MAX)) {
    goto err;
  }

  /* check data */

  if (pack->inViewCount > NMEALIB_MAX_SATELLITES) {
    nmeaContextError(NMEALIB_GPGSV_PREFIX " parse error: can't handle %u satellites (maximum is %u)", pack->inViewCount,
    NMEALIB_MAX_SATELLITES);
    goto err;
  }

  if (!pack->sentenceCount) {
    nmeaContextError(NMEALIB_GPGSV_PREFIX " parse error: sentences count %u is invalid in '%s'", pack->sentenceCount,
        s);
    goto err;
  }

  if (pack->sentenceCount > NMEALIB_GPGSV_MAX_SENTENCES) {
    nmeaContextError(NMEALIB_GPGSV_PREFIX " parse error: can't handle %u sentences (maximum is %u)",
        pack->sentenceCount,
        NMEALIB_GPGSV_MAX_SENTENCES);
    goto err;
  }

  if (pack->sentenceCount != nmeaGPGSVsatellitesToSentencesCount(pack->inViewCount)) {
    nmeaContextError(
        NMEALIB_GPGSV_PREFIX " parse error: sentence count %u does not correspond to satellite count %u in '%s'",
        pack->sentenceCount, pack->inViewCount, s);
    goto err;
  }

  if (!pack->sentence) {
    nmeaContextError(NMEALIB_GPGSV_PREFIX " parse error: sentence index %u is invalid in '%s'", pack->sentence, s);
    goto err;
  }

  if (pack->sentence > pack->sentenceCount) {
    nmeaContextError(
        NMEALIB_GPGSV_PREFIX " parse error: sentence index %u is beyond the sentence count (%u) in '%s'",
        pack->sentence, pack->sentenceCount, s);
    goto err;
  }

  /* see that there are enough tokens */
  if (pack->sentence != pack->sentenceCount) {
    satellitesInSentence = NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE;
  } else {
    satellitesInSentence = pack->inViewCount - ((pack->sentenceCount - 1) << NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE_SHIFT);
  }

  tokenCountExpected = 3 + (4 * satellitesInSentence); /* 4 fields per satellite */

  if ((tokenCount != tokenCountExpected) //
      && (tokenCount != 19)) {
    nmeaContextError(NMEALIB_GPGSV_PREFIX " parse error: need %lu (or 19) tokens, got %lu in '%s'",
        (long unsigned) tokenCountExpected, (long unsigned) tokenCount, s);
    goto err;
  }

  /* validate all satellites */
  for (i = 0; i < NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE; i++) {
    NmeaSatellite *sat = &pack->inView[i];
    if (!nmeaValidateSatellite(sat, NMEALIB_GPGSV_PREFIX, s)) {
      goto err;
    }
  }

  nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_SATINVIEWCOUNT | NMEALIB_PRESENT_SATINVIEW);

  return true;

err:
  memset(pack, 0, sizeof(*pack));
  return false;

#undef sat3
#undef sat2
#undef sat1
#undef sat0

}

void nmeaGPGSVToInfo(const NmeaGPGSV *pack, NmeaInfo *info) {
  if (!pack //
      || !info) {
    return;
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_SATINVIEWCOUNT)) {
    if (pack->inViewCount > NMEALIB_MAX_SATELLITES) {
      nmeaContextError("%s error: can't handle %u satellites (maximum is %u)", __FUNCTION__, pack->inViewCount,
      NMEALIB_MAX_SATELLITES);
      return;
    }

    info->satellites.inViewCount = pack->inViewCount;
    nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINVIEWCOUNT);
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_SATINVIEW)) {
    size_t i;
    size_t p;

    if (!pack->sentenceCount) {
      nmeaContextError("%s error: sentences count %u is invalid", __FUNCTION__, pack->sentenceCount);
      return;
    }

    if (pack->sentenceCount > NMEALIB_GPGSV_MAX_SENTENCES) {
      nmeaContextError("%s error: can't handle %u sentences (maximum is %u)", __FUNCTION__, pack->sentenceCount,
      NMEALIB_GPGSV_MAX_SENTENCES);
      return;
    }

    if (pack->sentenceCount != nmeaGPGSVsatellitesToSentencesCount(pack->inViewCount)) {
      nmeaContextError("%s error: sentences count %u does not correspond to satellite count %u", __FUNCTION__,
          pack->sentenceCount, pack->inViewCount);
      return;
    }

    if (!pack->sentence) {
      nmeaContextError("%s error: sentence index %u is invalid", __FUNCTION__, pack->sentence);
      return;
    }

    if (pack->sentence > pack->sentenceCount) {
      nmeaContextError("%s error: sentence %u is beyond the sentence count (%u)", __FUNCTION__, pack->sentence,
          pack->sentenceCount);
      return;
    }

    if (pack->sentence <= pack->sentenceCount) {
      /* clear non-present satellites */
      size_t start = pack->sentence << NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE_SHIFT;
      size_t clearCount = NMEALIB_MAX_SATELLITES - start;
      if (clearCount) {
        memset(&info->satellites.inView[start], 0, clearCount * sizeof(info->satellites.inView[0]));
      }
    }

    i = (pack->sentence - 1) << NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE_SHIFT;

    for (p = 0; (p < NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE) && (i < NMEALIB_MAX_SATELLITES); p++, i++) {
      const NmeaSatellite *src = &pack->inView[p];
      if (!src->prn) {
        memset(&info->satellites.inView[i], 0, sizeof(info->satellites.inView[i]));
      } else {
        info->satellites.inView[i] = *src;
      }
    }

    nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINVIEW);

    info->progress.gpgsvInProgress = (pack->sentence != pack->sentenceCount);
  }

  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SMASK);

  info->smask |= NMEALIB_SENTENCE_GPGSV;
}

void nmeaGPGSVFromInfo(const NmeaInfo *info, NmeaGPGSV *pack, size_t sentence) {
  size_t inViewCount;
  size_t sentences;

  if (!pack) {
    return;
  }

  memset(pack, 0, sizeof(*pack));

  if (!info //
      || !nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_SATINVIEWCOUNT) //
      || !info->satellites.inViewCount) {
    return;
  }

  inViewCount = info->satellites.inViewCount;
  sentences = nmeaGPGSVsatellitesToSentencesCount(inViewCount);

  if (sentence >= sentences) {
    return;
  }

  pack->inViewCount = (unsigned int) inViewCount;
  pack->sentenceCount = (unsigned int) sentences;
  nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_SATINVIEWCOUNT);

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_SATINVIEW)) {
    size_t i;
    size_t p;

    pack->sentence = (unsigned int) sentence + 1;

    i = sentence << NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE_SHIFT;

    for (p = 0; (p < NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE) && (i < NMEALIB_MAX_SATELLITES); p++, i++) {
      pack->inView[p] = info->satellites.inView[i];
    }

    nmeaInfoSetPresent(&pack->present, NMEALIB_PRESENT_SATINVIEW);
  }
}

size_t nmeaGPGSVGenerate(char *s, const size_t sz, const NmeaGPGSV *pack) {

#define dst       (&s[chars])
#define available ((sz <= (size_t) chars) ? 0 : (sz - (size_t) chars))

  int chars = 0;
  size_t inViewCount = 0;
  size_t sentenceCount = 1;
  size_t sentence = 1;
  size_t satellitesInSentence = 0;
  size_t i = 0;

  if (!s //
      || !pack) {
    return 0;
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_SATINVIEWCOUNT)) {
    inViewCount = pack->inViewCount;
    sentenceCount = pack->sentenceCount;
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_SATINVIEW)) {
    sentence = pack->sentence;
  }

  chars += snprintf(dst, available, //
      "$" NMEALIB_GPGSV_PREFIX ",%lu,%lu,%lu", //
      (long unsigned) sentenceCount, //
      (long unsigned) sentence, //
      (long unsigned) inViewCount);

  if (pack->sentence != pack->sentenceCount) {
    satellitesInSentence = NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE;
  } else {
    satellitesInSentence = inViewCount - ((pack->sentenceCount - 1) << NMEALIB_GPGSV_MAX_SATS_PER_SENTENCE_SHIFT);
  }

  if (nmeaInfoIsPresentAll(pack->present, NMEALIB_PRESENT_SATINVIEW)) {
    for (i = 0; i < satellitesInSentence; i++) {
      const NmeaSatellite *sat = &pack->inView[i];
      if (sat->prn) {
        chars += snprintf(dst, available, ",%u,%d,%u,%u", sat->prn, sat->elevation, sat->azimuth, sat->snr);
      } else {
        chars += snprintf(dst, available, ",,,,");
      }
    }
  }

  /* checksum */
  chars += nmeaAppendChecksum(s, sz, (size_t) chars);

  return (size_t) chars;

#undef available
#undef dst

}
