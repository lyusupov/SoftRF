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

/**
 * @file
 * NMEA Sentences
 *
 * The table below describes which NmeaInfo fields are present in the
 * sentences that are supported by the library.
 *
 * | NmeaInfo field       | GPGGA | GPGSA | GPGSV | GPRMC | GPVTG |
 * | :------------------- | :---: | :---: | :---: | :---: | :---: |
 * | present              | x     | x     | x     | x     | x     |
 * | smask                | x     | x     | x     | x     | x     |
 * | utc (date)           |       |       |       | x     |       |
 * | utc (time)           | x     |       |       | x     |       |
 * | sig                  | x     | x (1) |       | x (3) |       |
 * | fix                  |       | x     |       |       |       |
 * | PDOP                 |       | x     |       |       |       |
 * | HDOP                 | x     | x     |       |       |       |
 * | VDOP                 |       | x     |       |       |       |
 * | lat                  | x     |       |       | x     |       |
 * | lon                  | x     |       |       | x     |       |
 * | elv                  | x     |       |       |       |       |
 * | height               | x     |       |       |       |       |
 * | speed                |       |       |       | x     | x     |
 * | track                |       |       |       | x     | x     |
 * | mtrack               |       |       |       |       | x     |
 * | magvar               |       |       |       | x     |       |
 * | dgps_age             | x     |       |       |       |       |
 * | dgps_sid             | x     |       |       |       |       |
 * | satinfo inuse count  |       | x (2) |       |       |       |
 * | satinfo inuse        |       | x     |       |       |       |
 * | satinfo inview count | x     |       | x     |       |       |
 * | satinfo inview       |       |       | x     |       |       |
 *
 * (1) Only sets the NmeaInfo sig when it is not set yet.
 * (2) Not present in the sentence but the library sets it up.
 * (3) If the sentence is a v2.3+ sentence then the NmeaInfo sig is set
 *     normally, otherwise the NmeaInfo sig is only set when it is not
 *     set yet.
 */

#ifndef __NMEALIB_SENTENCE_H__
#define __NMEALIB_SENTENCE_H__

#include <nmealib/gpgga.h>
#include <nmealib/gpgsa.h>
#include <nmealib/gpgsv.h>
#include <nmealib/gprmc.h>
#include <nmealib/gpvtg.h>

#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Supported NMEA sentences
 */
typedef enum _NmeaSentence {
  NMEALIB_SENTENCE_GPNON = 0u,
  NMEALIB_SENTENCE_FIRST = NMEALIB_SENTENCE_GPNON,
  NMEALIB_SENTENCE_GPGGA = (1u << 0),
  NMEALIB_SENTENCE_GPGSA = (1u << 1),
  NMEALIB_SENTENCE_GPGSV = (1u << 2),
  NMEALIB_SENTENCE_GPRMC = (1u << 3),
  NMEALIB_SENTENCE_GPVTG = (1u << 4),
  NMEALIB_SENTENCE_LAST = NMEALIB_SENTENCE_GPVTG
} NmeaSentence;

/** The bit-mask with all NmeaSentence entries */
#define NMEALIB_SENTENCE_MASK (NMEALIB_SENTENCE_LAST | (NMEALIB_SENTENCE_LAST - 1))

/** The fixed length of a NMEA prefix */
#define NMEALIB_PREFIX_LENGTH 5

/**
 * The type definition for an entry mapping a NMEA sentence prefix to a sentence type
 */
typedef struct _NmeaSentencePrefixToType {
  const char *prefix;
  const NmeaSentence sentence;
} NmeaSentencePrefixToType;

/**
 * The map from NMEA sentence prefix to sentence type
 */
static const NmeaSentencePrefixToType nmealibSentencePrefixToType[] = {
    {
        .prefix = NMEALIB_GPGGA_PREFIX, //
        .sentence = NMEALIB_SENTENCE_GPGGA //
    },
    {
        .prefix = NMEALIB_GPGSA_PREFIX, //
        .sentence = NMEALIB_SENTENCE_GPGSA //
    },
    {
        .prefix = NMEALIB_GPGSV_PREFIX, //
        .sentence = NMEALIB_SENTENCE_GPGSV //
    },
    {
        .prefix = NMEALIB_GPRMC_PREFIX, //
        .sentence = NMEALIB_SENTENCE_GPRMC //
    },
    {
        .prefix = NMEALIB_GPVTG_PREFIX, //
        .sentence = NMEALIB_SENTENCE_GPVTG //
    },
    {
        .prefix = NULL, //
        .sentence = NMEALIB_SENTENCE_GPNON //
    }//
};

/**
 * A malloced buffer and its size.
 *
 * This type is used in functions that need allocated memory of variable
 * size: the function can allocate memory and reallocate it if needed.
 *
 * Using this type allows for the users of the functions to decide for
 * themselves whether or not to re-use (previously) allocated memory or
 * to treat the allocated memory as once-off.
 *
 * When handing this type to a function, one of the following 2 cases
 * MUST be used (the function will check for these):
 * 1- buffer is NULL and bufferSize is zero: this indicates that
 *    the function should allocate memory and can reallocate memory
 *    to increase the allocated size if needed.
 * 2- buffer is not NULL and bufferSize is not zero: this indicates that
 *    the user has already allocated memory so the function should not
 *    (initially) allocate memory. However, the function can reallocate
 *    memory to increase the allocated size if needed.
 */
typedef struct _NmeaMallocedBuffer {
  char *buffer;
  size_t bufferSize;
} NmeaMallocedBuffer;

/**
 * Determine the NMEA prefix from the sentence type.
 *
 * @param sentence The sentence type
 * @return The NMEA prefix, or NULL when the sentence type is unknown
 */
static INLINE const char *nmeaSentenceToPrefix(NmeaSentence sentence) {
  size_t i = 0;

  while (nmealibSentencePrefixToType[i].prefix) {
    if (nmealibSentencePrefixToType[i].sentence == sentence) {
      return nmealibSentencePrefixToType[i].prefix;
    }

    i++;
  }

  return NULL;
}

/**
 * Determine the sentence type from the start of the specified NMEA
 * sentence
 *
 * If the first character of the string is equal to the NMEA start-of-line
 * character ('$') then that character is skipped.
 *
 * @param s The NMEA sentence
 * @param sz The length of the NMEA sentence
 * @return The packet type, or GPNON when it could not be determined
 */
NmeaSentence nmeaSentenceFromPrefix(const char *s, const size_t sz);

/**
 * Parse a NMEA sentence into an unsanitised NmeaInfo structure
 *
 * @param s The NMEA sentence
 * @param sz The length of the NMEA sentence
 * @param info The unsanitised NmeaInfo structure in which to stored the information
 * @return True when successful
 */
bool nmeaSentenceToInfo(const char *s, const size_t sz, NmeaInfo *info);

/**
 * Generate NMEA sentences from a sanitised NmeaInfo structure.
 *
 * Allocates memory as needed.
 *
 * @param buf The allocated buffer (do read the comments of NmeaMallocedBuffer)
 * @param info The sanitised NmeaInfo structure
 * @param mask The bit-mask of sentences to generate
 * @return The total length of the generated sentences
 */
size_t nmeaSentenceFromInfo(NmeaMallocedBuffer *buf, const NmeaInfo *info, const NmeaSentence mask);

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* __NMEALIB_SENTENCE_H__ */
