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

#ifndef __NMEALIB_PARSER_H__
#define __NMEALIB_PARSER_H__

#include <nmealib/info.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef NMEALIB_MAX_SENTENCE_LENGTH
  #define NMEALIB_PARSER_SENTENCE_SIZE (NMEALIB_MAX_SENTENCE_LENGTH)
#else
  #define NMEALIB_PARSER_SENTENCE_SIZE (NMEALIB_BUFFER_CHUNK_SIZE)
#endif

typedef enum _NmeaParserSentenceState {
  NMEALIB_SENTENCE_STATE_SKIP_UNTIL_START,
  NMEALIB_SENTENCE_STATE_READ_SENTENCE,
  NMEALIB_SENTENCE_STATE_READ_CHECKSUM,
  NMEALIB_SENTENCE_STATE_READ_EOL
} NmeaParserSentenceState;

/**
 * NMEA frame parser structure
 */
typedef struct _NmeaParserSentence {
    NmeaParserSentenceState state;

    bool checksumPresent;

    char checksumCharacters[2];
    char checksumCharactersCount;

    int checksumRead;
    int checksumCalculated;

    unsigned char eolCharactersCount;
} NmeaParserSentence;

/**
 * parsed NMEA data and frame parser state
 */
typedef struct _NmeaParser {
    NmeaParserSentence sentence;
    size_t bufferLength;
    char *buffer;
    size_t bufferSize;
} NmeaParser;

/**
 * Initialise the parser
 *
 * Allocates memory for the parse buffer.
 *
 * @param parser The parser
 * @param sz The size for the allocated parse buffer, If zero then
 * NMEALIB_PARSER_SENTENCE_SIZE is used
 * @return True on success
 */
bool nmeaParserInit(NmeaParser *parser, size_t sz);

/**
 * Destroy the parser
 *
 * Frees memory of the parse buffer.
 *
 * @param parser The parser
 * @return True on success
 */
bool nmeaParserDestroy(NmeaParser *parser);

/**
 * Parse NMEA sentences from a (string) buffer and store the results in the
 * info structure
 *
 * @param parser The parser
 * @param s The (string) buffer
 * @param sz The length of the string in the buffer
 * @param info The info structure in which to store the information
 * @return The number of sentences that were parsed
 */
size_t nmeaParserParse(NmeaParser *parser, const char *s, size_t sz, NmeaInfo *info);

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* __NMEALIB_PARSER_H__ */
