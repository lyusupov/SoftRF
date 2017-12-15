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

#include <nmealib/util.h>

#include <nmealib/context.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/** The maximum size of a string-to-number conversion buffer*/
#define NMEALIB_CONVSTR_BUF    64

void nmeaRandomInit(void) {
  srandom((unsigned int) time(NULL));
}

double nmeaRandom(const double min, const double max) {
  long value;
  int randomFile;
  double range = fabs(max - min);

  randomFile = open("/dev/urandom", O_RDONLY);
  if (randomFile == -1) {
    /* can't be covered in a test */
    randomFile = open("/dev/random", O_RDONLY);
  }

  if ((randomFile == -1) //
      || (read(randomFile, &value, sizeof(value)) != sizeof(value))) {
    /* can't be covered in a test */
    value = random();
  }

  if (randomFile != -1) {
    close(randomFile);
  }

  return min + ((fabs((double) value) * range) / (double) LONG_MAX);
}

size_t nmeaStringTrim(const char **s) {
  const char *str;
  size_t sz;

  if (!s //
      || !*s) {
    return 0;
  }

  str = *s;

  while (isspace(*str)) {
    str++;
  }

  sz = strlen(str);

  while (sz && isspace(str[sz - 1])) {
    sz--;
  }

  *s = str;
  return sz;
}

bool nmeaStringContainsWhitespace(const char *s, size_t sz) {
  size_t i = 0;

  if (!s) {
    return false;
  }

  while ((i < sz) && s[i]) {
    if (isspace(s[i])) {
      return true;
    }
    i++;
  }

  return false;
}

unsigned int nmeaCalculateCRC(const char *s, const size_t sz) {
  size_t i = 0;
  int crc = 0;

  if (!s //
      || !sz) {
    return 0xff;
  }

  if (s[i] == '$') {
    i++;
  }

  for (; i < sz; i++) {
    crc ^= (int) s[i];
  }

  return ((unsigned int) crc & 0xff);
}

int nmeaStringToInteger(const char *s, size_t sz, int radix) {
  long r = nmeaStringToLong(s, sz, radix);

  if (r < INT_MIN) {
    r = INT_MIN;
  }

  if (r > INT_MAX) {
    r = INT_MAX;
  }

  return (int) r;
}

unsigned int nmeaStringToUnsignedInteger(const char *s, size_t sz, int radix) {
  unsigned long r = nmeaStringToUnsignedLong(s, sz, radix);

  if (r > UINT_MAX) {
    r = UINT_MAX;
  }

  return (unsigned int) r;
}

long nmeaStringToLong(const char *s, size_t sz, int radix) {
  char buf[NMEALIB_CONVSTR_BUF];
  char *endPtr = NULL;
  long value;

  if (!s //
      || !sz //
      || (sz >= NMEALIB_CONVSTR_BUF) //
      || (radix < 1)) {
    return 0;
  }

  memcpy(buf, s, sz);
  buf[sz] = '\0';

  errno = 0;
  value = strtol(buf, &endPtr, radix);

  if ((errno != ERANGE) //
      && ((endPtr == buf) //
          || (*buf == '\0'))) {
    /* invalid conversion */
    nmeaContextError("Could not convert '%s' to a long integer", buf);
    return LONG_MAX;
  }

  return value;
}

unsigned long nmeaStringToUnsignedLong(const char *s, size_t sz, int radix) {
  char buf[NMEALIB_CONVSTR_BUF];
  char *endPtr = NULL;
  unsigned long value;

  if (!s //
      || !sz //
      || (sz >= NMEALIB_CONVSTR_BUF) //
      || (radix < 1)) {
    return 0;
  }

  memcpy(buf, s, sz);
  buf[sz] = '\0';

  errno = 0;
  value = strtoul(buf, &endPtr, radix);

  if ((errno != ERANGE) //
      && ((endPtr == buf) //
          || (*buf == '\0'))) {
    /* invalid conversion */
    nmeaContextError("Could not convert '%s' to an unsigned long integer", buf);
    return ULONG_MAX;
  }

  return value;
}

double nmeaStringToDouble(const char *s, const size_t sz) {
  char buf[NMEALIB_CONVSTR_BUF];
  char *endPtr = NULL;
  double value;

  if (!s //
      || !sz //
      || (sz >= NMEALIB_CONVSTR_BUF)) {
    return 0.0;
  }

  memcpy(buf, s, sz);
  buf[sz] = '\0';

  errno = 0;
  value = strtod(buf, &endPtr);

  if ((errno != ERANGE) //
      && ((endPtr == buf) //
          || (*buf == '\0'))) {
    /* invalid conversion */
    nmeaContextError("Could not convert '%s' to a double", buf);
    return NaN;
  }

  return value;
}

int nmeaAppendChecksum(char *s, size_t sz, size_t len) {

#define dst       (&s[len])
#define available ((sz <= len) ? 0 : (sz - len))

  if (!s) {
    return 0;
  }

  return snprintf(dst, available, "*%02X\r\n", nmeaCalculateCRC(s, len));

#undef available
#undef dst

}

int nmeaPrintf(char *s, size_t sz, const char *format, ...) {

#define dst       (s)
#define available ((sz <= (size_t) chars) ? 0 : (sz - (size_t) chars))

  int chars = 0;
  va_list args;

  if (!s //
      || !format) {
    return 0;
  }

  va_start(args, format);

  chars = vsnprintf(dst, available, format, args);

  va_end(args);

  if (chars >= 0) {
    chars += nmeaAppendChecksum(dst, sz, (size_t) chars);
  }

  return chars;

#undef available
#undef dst

}

size_t nmeaScanf(const char *s, size_t sz, const char *format, ...) {

#define sCharsLeft (sEnd - sCharacter)

#define NMEALIB_SCANF_COMPARE   1u
#define NMEALIB_SCANF_TOKEN     2u

  size_t tokens = 0;
  unsigned char state = NMEALIB_SCANF_COMPARE;

  const char *sCharacter = s;
  const char *sTokenStart = sCharacter;
  const char *sEnd = &s[sz];

  const char *formatCharacter = format;
  const char *formatStart = format;

  size_t width = 0;
  size_t widthMax = 0; /* for strings; includes null-terminator */
  size_t widthCount = 0;

  void *arg = NULL;
  va_list args;

  if (!s //
      || !format) {
    return 0;
  }

  va_start(args, format);

  for (formatCharacter = format; *formatCharacter && (sCharacter <= sEnd); formatCharacter++) {
    switch (state) {
      case NMEALIB_SCANF_TOKEN:
        if (isdigit(*formatCharacter)) {
          widthCount++;
          break;
        }

        /* start of a token */

        sTokenStart = sCharacter;
        tokens++;

        if (!widthCount) {
          /* no width specified */
          width = 0;
          widthMax = (size_t) (sCharsLeft + 1);
        } else {
          /* width specified */
          width = nmeaStringToUnsignedInteger(formatStart, widthCount, 10);
          widthMax = width;
        }

        if (!width) {
          /* no width specified */

          if (('C' == toupper(*formatCharacter)) //
              && (*sCharacter != formatCharacter[1])) {
            width = 1;
          }

          if (!formatCharacter[1] //
              || (0 == (sCharacter = (char *) memchr(sCharacter, formatCharacter[1], (size_t) sCharsLeft)))) {
            sCharacter = sEnd;
          }
        } else if (('S' == toupper(*formatCharacter)) //
                   || ('C' == toupper(*formatCharacter))) {
          /* string/char maximum width specified */

          if (!formatCharacter[1] //
              || (0 == (sCharacter = (char *) memchr(sCharacter, formatCharacter[1], (size_t) sCharsLeft)))) {
            sCharacter = sEnd;
          }
        } else {
          /* exact width specified */
          sCharacter += width;
        }

        if (sCharsLeft < 0) {
          sCharacter = sEnd;
        }

        if ((sTokenStart >= sEnd) //
            || (*sTokenStart == '*') //
            || (*sTokenStart == '\0')) {
          /* empty field at the end of the string */
          width = 0;
        } else {
          width = (size_t) (sCharacter - sTokenStart);
        }

        width = MIN(width, widthMax);

        /* go back to the compare state after storing arg */
        state = NMEALIB_SCANF_COMPARE;

        arg = NULL;
        switch (*formatCharacter) {
          case 'c':
          case 'C':
            arg = (void *) va_arg(args, char *);
            if (width && arg) {
              if (*formatCharacter == 'c') {
                *((char *) arg) = *sTokenStart;
              } else {
                *((char *) arg) = (char) toupper(*sTokenStart);
              }
            }
            break;

          case 's':
            arg = (void *) va_arg(args, char *);
            if (width && arg) {
              memcpy(arg, sTokenStart, width);
              if (width < widthMax) {
                ((char *) arg)[width] = '\0';
              } else {
                ((char *) arg)[widthMax - 1] = '\0';
              }
            }
            break;

          case 'f':
          case 'F':
            arg = (void *) va_arg(args, double *);
            if (width && arg) {
              double v = nmeaStringToDouble(sTokenStart, width);
              if (isNaN(v)) {
                tokens = 0;
                goto out;
              }
              if (*formatCharacter == 'f') {
                *((double *) arg) = v;
              } else {
                *((double *) arg) = fabs(v);
              }
            }
            break;

          case 'd':
            arg = (void *) va_arg(args, int *);
            if (width && arg) {
              int v = nmeaStringToInteger(sTokenStart, width, 10);
              if (v == INT_MAX) {
                tokens = 0;
                goto out;
              }
              *((int *) arg) = v;
            }
            break;

          case 'u':
            arg = (void *) va_arg(args, unsigned int *);
            if (width && arg) {
              unsigned int v = nmeaStringToUnsignedInteger(sTokenStart, width, 10);
              if (v == UINT_MAX) {
                tokens = 0;
                goto out;
              }
              *((unsigned int *) arg) = nmeaStringToUnsignedInteger(sTokenStart, width, 10);
            }
            break;

          case 'l':
            arg = (void *) va_arg(args, long *);
            if (width && arg) {
              long v = nmeaStringToLong(sTokenStart, width, 10);
              if (v == LONG_MAX) {
                tokens = 0;
                goto out;
              }
              *((long *) arg) = v;
            }
            break;

          default:
            tokens--;
            nmeaContextError("Unknown format character '%c' in '%s' (%s)", *formatCharacter, format, __FUNCTION__);
            goto out;
        } /* switch (*formatCharacter) */
        break;

      case NMEALIB_SCANF_COMPARE:
      default:
        if (*formatCharacter == '%') {
          /* start of format */
          formatStart = &formatCharacter[1];
          widthCount = 0;
          state = NMEALIB_SCANF_TOKEN;
        } else if (*sCharacter++ != *formatCharacter) {
          /* compare regular character between s and format */
          goto out;
        }
        break;
    } /* switch (state) */
  } /* for */

out:
  va_end(args);
  return tokens;

#undef NMEALIB_SCANF_TOKEN
#undef NMEALIB_SCANF_COMPARE

}
