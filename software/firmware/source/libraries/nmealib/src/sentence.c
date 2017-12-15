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

#include <nmealib/sentence.h>

#include <stdlib.h>
#include <string.h>

NmeaSentence nmeaSentenceFromPrefix(const char *s, const size_t sz) {
  const char *str = s;
  size_t size = sz;
  size_t i = 0;

  if (!str //
      || !size) {
    return NMEALIB_SENTENCE_GPNON;
  }

  if (*str == '$') {
    str++;
    size--;
  }

  if (size < NMEALIB_PREFIX_LENGTH) {
    return NMEALIB_SENTENCE_GPNON;
  }

  while (nmealibSentencePrefixToType[i].prefix) {
    if (!strncmp(str, nmealibSentencePrefixToType[i].prefix, NMEALIB_PREFIX_LENGTH)) {
      return nmealibSentencePrefixToType[i].sentence;
    }

    i++;
  }

  return NMEALIB_SENTENCE_GPNON;
}

bool nmeaSentenceToInfo(const char *s, const size_t sz, NmeaInfo *info) {
  switch (nmeaSentenceFromPrefix(s, sz)) {
    case NMEALIB_SENTENCE_GPGGA: {
      NmeaGPGGA gpgga;
      if (nmeaGPGGAParse(s, sz, &gpgga)) {
        nmeaGPGGAToInfo(&gpgga, info);
        return true;
      }

      return false;
    }

    case NMEALIB_SENTENCE_GPGSA: {
      NmeaGPGSA gpgsa;
      if (nmeaGPGSAParse(s, sz, &gpgsa)) {
        nmeaGPGSAToInfo(&gpgsa, info);
        return true;
      }

      return false;
    }

    case NMEALIB_SENTENCE_GPGSV: {
      NmeaGPGSV gpgsv;
      if (nmeaGPGSVParse(s, sz, &gpgsv)) {
        nmeaGPGSVToInfo(&gpgsv, info);
        return true;
      }

      return false;
    }

    case NMEALIB_SENTENCE_GPRMC: {
      NmeaGPRMC gprmc;
      if (nmeaGPRMCParse(s, sz, &gprmc)) {
        nmeaGPRMCToInfo(&gprmc, info);
        return true;
      }

      return false;
    }

    case NMEALIB_SENTENCE_GPVTG: {
      NmeaGPVTG gpvtg;
      if (nmeaGPVTGParse(s, sz, &gpvtg)) {
        nmeaGPVTGToInfo(&gpvtg, info);
        return true;
      }

      return false;
    }

    case NMEALIB_SENTENCE_GPNON:
    default:
      return false;
  }
}

size_t nmeaSentenceFromInfo(NmeaMallocedBuffer *buf, const NmeaInfo *info, const NmeaSentence mask) {

#define dst       (&s[chars])
#define available ((sz <= (size_t) chars) ? 0 : (sz - (size_t) chars))

#define generateSentence(expression) { \
  size_t addedChars = expression; \
  while (addedChars >= available) { \
    sz += NMEALIB_BUFFER_CHUNK_SIZE; \
    s = realloc(s, sz); \
    if (!s) { \
      return 0; \
    } \
    addedChars = expression; \
  } \
  chars += addedChars; \
}

  char *s;
  size_t sz;
  size_t chars;
  NmeaSentence msk;

  if (!buf //
      || (!buf->buffer && buf->bufferSize) //
      || (buf->buffer && !buf->bufferSize) //
      || !info //
      || !mask) {
    return 0;
  }

  sz = buf->bufferSize;
  s = buf->buffer;

  if (!s) {
    sz = NMEALIB_BUFFER_CHUNK_SIZE;
    s = malloc(sz);
    if (!s) {
      /* can't be covered in a test */
      return 0;
    }
  }

  *s = '\0';

  chars = 0;
  msk = mask;

  while (msk) {
    if (msk & NMEALIB_SENTENCE_GPGGA) {
      NmeaGPGGA pack;
      nmeaGPGGAFromInfo(info, &pack);
      generateSentence(nmeaGPGGAGenerate(dst, available, &pack));
      msk &= (NmeaSentence) ~NMEALIB_SENTENCE_GPGGA;
    } else if (msk & NMEALIB_SENTENCE_GPGSA) {
      NmeaGPGSA pack;
      nmeaGPGSAFromInfo(info, &pack);
      generateSentence(nmeaGPGSAGenerate(dst, available, &pack));
      msk &= (NmeaSentence) ~NMEALIB_SENTENCE_GPGSA;
    } else if (msk & NMEALIB_SENTENCE_GPGSV) {
      size_t satCount = nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_SATINVIEWCOUNT) ?
          info->satellites.inViewCount :
          0;
      NmeaGPGSV pack;
      size_t sentence;
      size_t sentences = nmeaGPGSVsatellitesToSentencesCount(satCount);

      for (sentence = 0; sentence < sentences; sentence++) {
        nmeaGPGSVFromInfo(info, &pack, sentence);
        generateSentence(nmeaGPGSVGenerate(dst, available, &pack));
      }
      msk &= (NmeaSentence) ~NMEALIB_SENTENCE_GPGSV;
    } else if (msk & NMEALIB_SENTENCE_GPRMC) {
      NmeaGPRMC pack;
      nmeaGPRMCFromInfo(info, &pack);
      generateSentence(nmeaGPRMCGenerate(dst, available, &pack));
      msk &= (NmeaSentence) ~NMEALIB_SENTENCE_GPRMC;
    } else if (msk & NMEALIB_SENTENCE_GPVTG) {
      NmeaGPVTG pack;
      nmeaGPVTGFromInfo(info, &pack);
      generateSentence(nmeaGPVTGGenerate(dst, available, &pack));
      msk &= (NmeaSentence) ~NMEALIB_SENTENCE_GPVTG;
    } else {
      /* no more known sentences to process */
      break;
    }
  }

  s[chars] = '\0';

  buf->buffer = s;
  buf->bufferSize = sz;

  return chars;

#undef generateSentence
#undef available
#undef dst

}
