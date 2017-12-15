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

#ifndef __NMEALIB_GPRMC_H__
#define __NMEALIB_GPRMC_H__

#include <nmealib/info.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */

/** The NMEA prefix */
#define NMEALIB_GPRMC_PREFIX "GPRMC"

/**
 * GPRMC -packet information structure (Recommended Minimum sentence C)
 *
 * <pre>
 * $GPRMC,time,sig,lat,ns,lon,ew,speed,track,date,magvar,magvar ew,mode*checksum
 * </pre>
 *
 * | Field       | Description                                    | present    |
 * | :---------: | ---------------------------------------------- | :--------: |
 * | $GPRMC      | NMEA prefix                                    | -          |
 * | time        | Fix time, in the format HHMMSS.hh (UTC)        | UTCTIME    |
 * | sig         | Selection of 2D or 3D fix (A = auto, V = void) | SIG        |
 * | lat         | Latitude, in NDEG (DDMM.SSS)                   | LAT (1)    |
 * | ns          | North or south ('N' or 'S')                    | LAT (1)    |
 * | lon         | Longitude, in NDEG (DDDMM.SSS)                 | LON (2)    |
 * | ew          | East or west ('E' or 'W')                      | LON (2)    |
 * | speed       | Speed over the ground, in knots                | SPEED      |
 * | track       | Track angle, in degrees true north             | TRACK      |
 * | date        | Fix date, in the format DDMMYY (UTC)           | UTCDATE    |
 * | magvar      | Magnetic variation                             | MAGVAR (3) |
 * | magvar ew   | Magnetic variation east or west ('E' or 'W')   | MAGVAR (3) |
 * | mode        | Mode, N=not valid, or [ADPRFEMS]               | SIG (4)    |
 * | checksum    | NMEA checksum                                  | -          |
 *
 * (1) These fields are both required for a valid latitude<br/>
 * (2) These fields are both required for a valid longitude<br/>
 * (3) These fields are both required for a valid magnetic variation<br/>
 * (4) This field is only present in NMEA sentences with version v2.3 (and above).
 *     If present, then the selection field and this field are both required for a valid signal<br/>
 *
 * Example:
 *
 * <pre>
 * $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
 * $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W,A*6A (v2.3)
 * </pre>
 */
typedef struct _NmeaGPRMC {
  bool     v23;
  uint32_t present;
  NmeaTime utc;
  char     sigSelection;
  double   latitude;
  char     latitudeNS;
  double   longitude;
  char     longitudeEW;
  double   speed;
  double   track;
  double   magvar;
  char     magvarEW;
  char     sig;
} NmeaGPRMC;

/**
 * Parse a GPRMC sentence
 *
 * @param s The sentence
 * @param sz The length of the sentence
 * @param pack Where the result should be stored
 * @return True on success
 */
bool nmeaGPRMCParse(const char *s, const size_t sz, NmeaGPRMC *pack);

/**
 * Update an unsanitised NmeaInfo structure from a GPRMC packet structure
 *
 * @param pack The GPRMC packet structure
 * @param info The unsanitised NmeaInfo structure
 */
void nmeaGPRMCToInfo(const NmeaGPRMC *pack, NmeaInfo *info);

/**
 * Convert a sanitised NmeaInfo structure into a NmeaGPRMC structure
 *
 * @param info The sanitised NmeaInfo structure
 * @param pack The NmeaGPRMC structure
 */
void nmeaGPRMCFromInfo(const NmeaInfo *info, NmeaGPRMC *pack);

/**
 * Generate a GPRMC sentence
 *
 * @param s The buffer to generate the sentence in
 * @param sz The size of the buffer
 * @param pack The NmeaGPRMC structure
 * @return The length of the generated sentence; less than zero on failure,
 * larger than sz when the size of the buffer is too small to generate the
 * sentence in
 */
size_t nmeaGPRMCGenerate(char *s, const size_t sz, const NmeaGPRMC *pack);

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* __NMEALIB_GPRMC_H__ */
