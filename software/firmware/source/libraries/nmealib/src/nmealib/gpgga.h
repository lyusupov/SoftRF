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

#ifndef __NMEALIB_GPGGA_H__
#define __NMEALIB_GPGGA_H__

#include <nmealib/info.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */

/** The NMEA prefix */
#define NMEALIB_GPGGA_PREFIX "GPGGA"

/**
 * GPGGA packet information structure (Global Positioning System Fix Data)
 *
 * Essential fix data which provide 3D location and accuracy data.
 *
 * <pre>
 * $GPGGA,time,latitude,ns,longitude,ew,signal,satellites,hdop,elv,elv unit,height,height unit,dgps age,dgps id*checksum
 * </pre>
 *
 * | Field       | Description                                            | present        |
 * | :---------: | ------------------------------------------------------ | :------------: |
 * | $GPGGA      | NMEA prefix                                            | -              |
 * | time        | Fix time (UTC) (5)                                     | UTCTIME        |
 * | latitude    | Latitude, in NDEG (DDMM.SSS)                           | LAT (1)        |
 * | ns          | North or South ('N' or 'S')                            | LAT (1)        |
 * | longitude   | Longitude, in NDEG (DDDMM.SSS)                         | LON (2)        |
 * | ew          | East or West ('E' or 'W')                              | LON (2)        |
 * | signal      | Signal quality (see the NMEALIB_SIG_* defines)         | SIG            |
 * | satellites  | Number of satellites being tracked                     | SATINVIEWCOUNT |
 * | hdop        | Horizontal dilution of position                        | HDOP           |
 * | elv         | Elevation above mean sea level, in meters              | ELV (3)        |
 * | elv unit    | Unit of elevation ('M')                                | ELV (3)        |
 * | height      | Height of geoid (mean sea level) above WGS84 ellipsoid | - (4)          |
 * | height unit | Unit of height ('M')                                   | - (4)          |
 * | dgps age    | Time since last DGPS update, in seconds                | - (4)          |
 * | dgps id     | DGPS station ID number                                 | - (4)          |
 * | checksum    | NMEA checksum                                          | -              |
 *
 * (1) These fields are both required for a valid latitude<br/>
 * (2) These fields are both required for a valid longitude<br/>
 * (3) These fields are both required for a valid elevation<br/>
 * (4) Not supported yet<br/>
 * (5) Supported formats: HHMMSS, HHMMSS.t, HHMMSS.hh, HHMMSS.mmm<br/>
 *
 * Example:
 *
 * <pre>
 * $GPGGA,123519.43,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
 * </pre>
 *
 * Note that if the height of geoid is missing then the elevation should be
 * suspect. Some non-standard implementations report elevation with respect to
 * the ellipsoid rather than geoid elevation. Some units do not report negative
 * elevations at all. This is the only sentence that reports elevation.
 */
typedef struct _NmeaGPGGA {
  uint32_t     present;
  NmeaTime     utc;
  double       latitude;
  char         latitudeNS;
  double       longitude;
  char         longitudeEW;
  NmeaSignal   sig;
  unsigned int inViewCount;
  double       hdop;
  double       elevation;
  char         elevationM;
  double       height;
  char         heightM;
  double       dgpsAge;
  unsigned int dgpsSid;
} NmeaGPGGA;

/**
 * Parse a GPGGA sentence
 *
 * @param s The sentence
 * @param sz The length of the sentence
 * @param pack Where the result should be stored
 * @return True on success
 */
bool nmeaGPGGAParse(const char *s, const size_t sz, NmeaGPGGA *pack);

/**
 * Update an unsanitised NmeaInfo structure from a GPGGA packet structure
 *
 * @param pack The GPGGA packet structure
 * @param info The unsanitised NmeaInfo structure
 */
void nmeaGPGGAToInfo(const NmeaGPGGA *pack, NmeaInfo *info);

/**
 * Convert a sanitised NmeaInfo structure into a NmeaGPGGA structure
 *
 * @param info The sanitised NmeaInfo structure
 * @param pack The NmeaGPGGA structure
 */
void nmeaGPGGAFromInfo(const NmeaInfo *info, NmeaGPGGA *pack);

/**
 * Generate a GPGGA sentence
 *
 * @param s The buffer to generate the sentence in
 * @param sz The size of the buffer
 * @param pack The NmeaGPGGA structure
 * @return The length of the generated sentence; less than zero on failure,
 * larger than sz when the size of the buffer is too small to generate the
 * sentence in
 */
size_t nmeaGPGGAGenerate(char *s, const size_t sz, const NmeaGPGGA *pack);

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* __NMEALIB_GPGGA_H__ */
