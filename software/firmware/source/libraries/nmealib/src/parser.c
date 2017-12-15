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

#include <nmealib/parser.h>

#include <nmealib/sentence.h>
#include <nmealib/validate.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#define NMEALIB_PARSER_EOL_CHAR_1 ('\r')
#define NMEALIB_PARSER_EOL_CHAR_2 ('\n')

void nmeaParserReset(NmeaParser *parser, NmeaParserSentenceState new_state);
bool nmeaParserIsHexCharacter(char c);
bool nmeaParserProcessCharacter(NmeaParser *parser, const char *c);

bool nmeaParserIsHexCharacter(char c) {
  switch (tolower(c)) {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    case 'a':
    case 'b':
    case 'c':
    case 'd':
    case 'e':
    case 'f':
      return true;

    default:
      return false;
  }
}

void nmeaParserReset(NmeaParser *parser, NmeaParserSentenceState new_state) {
  if (!parser) {
    return;
  }

  memset(&parser->sentence, 0, sizeof(parser->sentence));
  parser->sentence.state = new_state;

  if (parser->buffer) {
    parser->buffer[0] = '\0';
    parser->buffer[parser->bufferSize - 1] = '\0';
  }
  parser->bufferLength = 0;
}

bool nmeaParserInit(NmeaParser *parser, size_t sz) {
  if (!parser) {
    return false;
  }

  parser->bufferSize = !sz ? NMEALIB_PARSER_SENTENCE_SIZE : sz;
  parser->buffer = malloc(parser->bufferSize);
  if (!parser->buffer) {
    /* can't be covered in a test */
    return false;
  }

  nmeaParserReset(parser, NMEALIB_SENTENCE_STATE_SKIP_UNTIL_START);
  return true;
}

bool nmeaParserDestroy(NmeaParser *parser) {
  if (!parser) {
    return false;
  }

  nmeaParserReset(parser, NMEALIB_SENTENCE_STATE_SKIP_UNTIL_START);

  free(parser->buffer);
  parser->buffer = NULL;
  parser->bufferSize = 0;

  return true;
}

bool nmeaParserProcessCharacter(NmeaParser *parser, const char *c) {
  if (!parser //
      || !c //
      || !parser->buffer) {
    return false;
  }

  /* always reset when we encounter a start-of-sentence character */
  if (*c == '$') {
    nmeaParserReset(parser, NMEALIB_SENTENCE_STATE_READ_SENTENCE);
    parser->buffer[parser->bufferLength++] = *c;
    return false;
  }

  /* just return when we haven't encountered a start-of-sentence character yet */
  if (parser->sentence.state == NMEALIB_SENTENCE_STATE_SKIP_UNTIL_START) {
    return false;
  }

  /* this character belongs to the sentence */

  /* check whether the sentence still fits in the buffer */
  if (parser->bufferLength >= (parser->bufferSize - 1)) {
    nmeaParserReset(parser, NMEALIB_SENTENCE_STATE_SKIP_UNTIL_START);
    return false;
  }

  parser->buffer[parser->bufferLength++] = *c;

  switch (parser->sentence.state) {
    case NMEALIB_SENTENCE_STATE_SKIP_UNTIL_START: /* can't occur but keep compiler happy */
    default: /* can't occur but keep compiler happy */
    case NMEALIB_SENTENCE_STATE_READ_SENTENCE:
      if (*c == '*') {
        parser->sentence.state = NMEALIB_SENTENCE_STATE_READ_CHECKSUM;
        parser->sentence.checksumCharactersCount = 0;
      } else if (*c == NMEALIB_PARSER_EOL_CHAR_1) {
        parser->sentence.state = NMEALIB_SENTENCE_STATE_READ_EOL;
        parser->sentence.eolCharactersCount = 1;
      } else if (nmeaValidateIsInvalidCharacter(*c)) {
        nmeaParserReset(parser, NMEALIB_SENTENCE_STATE_SKIP_UNTIL_START);
        return false;
      } else {
        parser->sentence.checksumCalculated ^= (int) *c;
      }

      break;

    case NMEALIB_SENTENCE_STATE_READ_CHECKSUM:
      if (!nmeaParserIsHexCharacter(*c)) {
        nmeaParserReset(parser, NMEALIB_SENTENCE_STATE_SKIP_UNTIL_START);
        return false;
      }

      switch (parser->sentence.checksumCharactersCount) {
        case 0:
          parser->sentence.checksumCharacters[0] = *c;
          parser->sentence.checksumCharacters[1] = 0;
          parser->sentence.checksumCharactersCount = 1;
          break;

        case 1:
        default: /* can't occur but keep compiler happy */
          parser->sentence.checksumCharacters[1] = *c;
          parser->sentence.checksumCharactersCount = 2;
          parser->sentence.checksumRead = nmeaStringToInteger(parser->sentence.checksumCharacters, 2, 16);
          parser->sentence.checksumPresent = true;
          parser->sentence.state = NMEALIB_SENTENCE_STATE_READ_EOL;
          break;
      }
      break;

    case NMEALIB_SENTENCE_STATE_READ_EOL:
      switch (parser->sentence.eolCharactersCount) {
        case 0:
          if (*c != NMEALIB_PARSER_EOL_CHAR_1) {
            nmeaParserReset(parser, NMEALIB_SENTENCE_STATE_SKIP_UNTIL_START);
            return false;
          }

          parser->sentence.eolCharactersCount = 1;
          break;

        case 1:
        default: /* can't occur but keep compiler happy */
          if (*c != NMEALIB_PARSER_EOL_CHAR_2) {
            nmeaParserReset(parser, NMEALIB_SENTENCE_STATE_SKIP_UNTIL_START);
            return false;
          }

          parser->sentence.eolCharactersCount = 2;

          /* strip off the end-of-line characters */
          parser->bufferLength -= parser->sentence.eolCharactersCount;
          parser->buffer[parser->bufferLength] = '\0';

          parser->sentence.state = NMEALIB_SENTENCE_STATE_SKIP_UNTIL_START;
          return (!parser->sentence.checksumCharactersCount
                  || (parser->sentence.checksumCharactersCount
                      && (parser->sentence.checksumRead == parser->sentence.checksumCalculated)));
      }
      break;
  }

  return false;
}

size_t nmeaParserParse(NmeaParser *parser, const char *s, size_t sz, NmeaInfo *info) {
  size_t sentences_count = 0;
  size_t charIndex = 0;

  if (!parser //
      || !s //
      || !sz //
      || !info //
      || !parser->buffer) {
    return 0;
  }

  for (charIndex = 0; charIndex < sz; charIndex++) {
    bool sentence_read_successfully = nmeaParserProcessCharacter(parser, &s[charIndex]);
    if (sentence_read_successfully) {
      if (nmeaSentenceToInfo(parser->buffer, parser->bufferLength, info)) {
        sentences_count++;
      }
    }
  }

  return sentences_count;
}
