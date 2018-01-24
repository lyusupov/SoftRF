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

#ifndef __NMEALIB_TOK_H__
#define __NMEALIB_TOK_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifndef INLINE
#define INLINE inline __attribute__((always_inline))
#endif

#ifndef MAX
#define MAX(x,y) (((x) >= (y)) ? (x) : (y))
#endif

#ifndef MIN
#define MIN(x,y) (((x) <= (y)) ? (x) : (y))
#endif

/** The power-of-2 chunk size of a buffer allocation */
#if !defined(ESP8266)  && !defined(ESP32)
#define NMEALIB_BUFFER_CHUNK_SIZE (4096UL)
#else
#define NMEALIB_BUFFER_CHUNK_SIZE (256UL)
#endif

/** NaN that is a double (and not a float) */
#define NaN strtod("NAN()", NULL)

/** isnan for doubles and floats alike */
#define isNaN(x) (x != x)

/**
 * Initialise the random number generation
 */
void nmeaRandomInit(void);

/**
 * Generate a random number
 *
 * @param min The minimum value of the generated random number
 * @param max The maximum value of the generated random number
 * @return A random number in the range [min, max]
 */
double nmeaRandom(const double min, const double max);

/**
 * Trim a string of whitespace
 *
 * @param s The location of the string variable
 * @return The length of the trimmed string
 */
size_t nmeaStringTrim(const char **s);

/**
 * Determine whether a string contains whitespace
 *
 * @param s The string to check for whitespace
 * @param sz The length of the string
 * @return True when the string contains whitespace
 */
bool nmeaStringContainsWhitespace(const char *s, size_t sz);

/**
 * Calculate the NMEA (CRC-8) checksum of a NMEA sentence.
 *
 * If the string starts with the NMEA start-of-line character '$' then that
 * character is skipped as per the NMEA spec.
 *
 * @param s The NMEA sentence
 * @param sz The length of the NMEA sentence
 * @return The NMEA checksum
 */
unsigned int nmeaCalculateCRC(const char *s, const size_t sz);

/**
 * Convert a string to an integer
 *
 * @param s The string
 * @param sz The length of the string
 * @param radix The radix of the numbers in the string
 * @return The converted number, or 0 on failure
 */
int nmeaStringToInteger(const char *s, const size_t sz, const int radix);

/**
 * Convert a string to an unsigned integer
 *
 * @param s The string
 * @param sz The length of the string
 * @param radix The radix of the numbers in the string
 * @return The converted number, or 0 on failure
 */
unsigned int nmeaStringToUnsignedInteger(const char *s, size_t sz, int radix);

/**
 * Convert string to a long integer
 *
 * @param s The string
 * @param sz The length of the string
 * @param radix The radix of the numbers in the string
 * @return The converted number, or 0 on failure
 */
long nmeaStringToLong(const char *s, size_t sz, int radix);

/**
 * Convert string to an unsigned long integer
 *
 * @param s The string
 * @param sz The length of the string
 * @param radix The radix of the numbers in the string
 * @return The converted number, or 0 on failure
 */
unsigned long nmeaStringToUnsignedLong(const char *s, size_t sz, int radix);

/**
 * Convert string to a floating point number
 *
 * @param s The string
 * @param sz The length of the string
 * @return The converted number, or 0.0 on failure
 */
double nmeaStringToDouble(const char *s, const size_t sz);

/**
 * Append a NMEA checksum to the string in the buffer
 *
 * @param s The buffer containing the string
 * @param sz The size of the buffer
 * @param len The length of the string in the buffer
 * @return The number of printed characters, -1 on error
 */
int nmeaAppendChecksum(char *s, size_t sz, size_t len);

/**
 * Format a string (with vsnprintf) and add the NMEA checksum
 *
 * @param s The buffer
 * @param sz The size of the buffer
 * @param format The string format to use
 * @return The number of printed characters, -1 on error
 */
int nmeaPrintf(char *s, size_t sz, const char *format, ...) __attribute__ ((format(printf, 3, 4)));

/**
 * Analyse a string (specific for NMEA sentences)
 *
 * @param s the string
 * @param sz the length of the string
 * @param format the string format to use
 * @return the number of scanned tokens
 */
size_t nmeaScanf(const char *s, size_t sz, const char *format, ...);

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* __NMEALIB_TOK_H__ */
