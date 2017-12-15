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
 * Extended descriptions of sentences are taken from
 *   http://www.gpsinformation.org/dale/nmea.htm
 */

#ifndef __NMEALIB_GPVTG_H__
#define __NMEALIB_GPVTG_H__

#include <nmealib/info.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */

/** The NMEA prefix */
#define NMEALIB_GPVTG_PREFIX "GPVTG"

/**
 * GPVTG packet information structure (Track made good and ground speed)
 *
 * <pre>
 * $GPVTG,track,T,mtrack,M,speedN,N,speedK,K*checksum
 * </pre>
 *
 * | Field       | Description                           | present   |
 * | :---------: | ------------------------------------- | :-------: |
 * | $GPVTG      | NMEA prefix                           | -         |
 * | track       | Track, in degress true north          | TRACK (1) |
 * | T           | Track indicator (True north)          | TRACK (1) |
 * | mtrack      | Magnetic track made good              | TRACK (2) |
 * | M           | Magnetic track indicator (Made good)  | TRACK (2) |
 * | speedN      | Ground speed, in knots                | SPEED (3) |
 * | N           | Ground speed unit (kNots)             | SPEED (3) |
 * | speedk      | Ground speed, in kph                  | SPEED (4) |
 * | K           | Ground speed unit (Kph)               | SPEED (4) |
 * | checksum    | NMEA checksum                         | -         |
 *
 * (1) These fields are both required for a valid track<br/>
 * (2) These fields are both required for a valid magnetic track<br/>
 * (3) These fields are both required for a valid speed in knots<br/>
 * (4) These fields are both required for a valid speed in kph<br/>
 *
 * Example:
 *
 * <pre>
 * $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
 * </pre>
 */
typedef struct _NmeaGPVTG {
  uint32_t present;
  double   track;
  char     trackT;
  double   mtrack;
  char     mtrackM;
  double   spn;
  char     spnN;
  double   spk;
  char     spkK;
} NmeaGPVTG;

/**
 * Parse a GPVTG sentence
 *
 * @param s The sentence
 * @param sz The length of the sentence
 * @param pack Where the result should be stored
 * @return True on success
 */
bool nmeaGPVTGParse(const char *s, const size_t sz, NmeaGPVTG *pack);

/**
 * Update an unsanitised NmeaInfo structure from a GPVTG packet structure
 *
 * @param pack The GPVTG packet structure
 * @param info The unsanitised NmeaInfo structure
 */
void nmeaGPVTGToInfo(const NmeaGPVTG *pack, NmeaInfo *info);

/**
 * Convert a sanitised NmeaInfo structure into a NmeaGPVTG structure
 *
 * @param info The sanitised NmeaInfo structure
 * @param pack The NmeaGPVTG structure
 */
void nmeaGPVTGFromInfo(const NmeaInfo *info, NmeaGPVTG *pack);

/**
 * Generate a GPVTG sentence
 *
 * @param s The buffer to generate the sentence in
 * @param sz The size of the buffer
 * @param pack The NmeaGPVTG structure
 * @return The length of the generated sentence; less than zero on failure,
 * larger than sz when the size of the buffer is too small to generate the
 * sentence in
 */
size_t nmeaGPVTGGenerate(char *s, const size_t sz, const NmeaGPVTG *pack);

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* __NMEALIB_GPVTG_H__ */
