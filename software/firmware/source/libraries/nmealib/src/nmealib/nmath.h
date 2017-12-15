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

#ifndef __NMEALIB_MATH_H__
#define __NMEALIB_MATH_H__

#include <nmealib/info.h>
#include <stdbool.h>

#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */

#define NMEALIB_KNOT_TO_KPH           (1.852)
#define NMEALIB_KPH_TO_KNOT           (0.5399568034557234996739794041786808520)
#define NMEALIB_DEGREE_TO_RADIAN      (0.0174532925199432957692369076848861271)
#define NMEALIB_RADIAN_TO_DEGREE      (57.2957795130823208767981548141051703)
#define NMEALIB_DOP_TO_METER          (5.0)
#define NMEALIB_METER_TO_DOP          (0.2)
#define NMEALIB_MPS_TO_KPH            (3.6)
#define NMEALIB_PI                    (3.1415926535897932384626433832795029)
#define NMEALIB_EARTHRADIUS_KM        (6378.137)
#define NMEALIB_EARTHRADIUS_M         (6378137)
#define NMEALIB_EARTH_SEMIMAJORAXIS_M (6356752.3142)
#define NMEALIB_EARTH_FLATTENING      (1.0 / 298.257223563)

/*
 * Degrees and Radians
 */

/**
 * Convert decimal degrees to radians
 *
 * @param v Degrees
 * @return Radians
 */
double nmeaMathDegreeToRadian(const double v);

/**
 * Convert radians to decimal degrees
 *
 * @param v Radians
 * @return Degrees
 */
double nmeaMathRadianToDegree(const double v);

/*
 * NDEG (NMEA degree)
 */

/**
 * Convert NDEG (NMEA degrees) to decimal degrees
 *
 * @param v NDEG (NMEA degrees)
 * @return Decimal degrees
 */
double nmeaMathNdegToDegree(const double v);

/**
 * Convert decimal degrees to NDEG (NMEA degrees)
 *
 * @param v Decimal degrees
 * @return NDEG (NMEA degrees)
 */
double nmeaMathDegreeToNdeg(const double v);

/**
 * Convert NDEG (NMEA degrees) to radians
 *
 * @param v NDEG (NMEA degrees)
 * @return Radians
 */
double nmeaMathNdegToRadian(const double v);

/**
 * Convert radians to NDEG (NMEA degrees)
 *
 * @param v Radians
 * @return NDEG (NMEA degrees)
 */
double nmeaMathRadianToNdeg(const double v);

/*
 * DOP
 */

/**
 * Calculate PDOP (Position Dilution Of Precision) from HDOP and VDOP
 *
 * @param hdop HDOP
 * @param vdop VDOP
 * @return PDOP
 */
double nmeaMathPdopCalculate(const double hdop, const double vdop);

/**
 * Convert DOP to meters, using the NMEALIB_DOP_FACTOR factor
 *
 * @param dop The DOP
 * @return The DOP in meters
 */
double nmeaMathDopToMeters(const double dop);

/**
 * Convert DOP in meters to plain DOP, using the NMEALIB_DOP_FACTOR factor
 *
 * @param meters The DOP in meters
 * @return The plain DOP
 */
double nmeaMathMetersToDop(const double meters);

/*
 * Positions
 */

/**
 * Convert a NmeaInfo position to a radians position
 *
 * @param info The NmeaInfo position
 * @param pos The radians position
 */
void nmeaMathInfoToPosition(const NmeaInfo *info, NmeaPosition *pos);

/**
 * Convert a radians position to a NmeaInfo position
 *
 * @param pos The radians position
 * @param info The NmeaInfo position
 */
void nmeaMathPositionToInfo(const NmeaPosition *pos, NmeaInfo *info);

/**
 * Calculate the distance between two points
 *
 * @param from The 'from' position (in radians)
 * @param to The 'to' position (in radians)
 * @return Distance in meters
 */
double nmeaMathDistance(const NmeaPosition *from, const NmeaPosition *to);

/**
 * Calculate the distance between two points
 *
 * This function uses an algorithm for an oblate spheroid earth model.
 * The algorithm is described here:
 *   http://www.ngs.noaa.gov/PUBS_LIB/inverse.pdf
 *
 * @param from The 'from' position (in radians)
 * @param to The 'to' position (in radians)
 * @param fromAzimuth The azimuth at 'from' position (in radians)
 * @param toAzimuth The azimuth at 'to' position (in radians)
 * @return Distance in meters
 */
double nmeaMathDistanceEllipsoid(const NmeaPosition *from, const NmeaPosition *to, double *fromAzimuth,
    double *toAzimuth);

/**
 * Perform a flat (horizontal) move.
 *
 * @param from The 'from' position (in radians)
 * @param to The 'to' position (in radians)
 * @param azimuth Azimuth (in degrees, [0, 359])
 * @param distance The distance (in km)
 * @return True on success
 */
bool nmeaMathMoveFlat(const NmeaPosition *from, NmeaPosition *to, double azimuth, double distance);

/**
 * Perform a flat (horizontal) move against the ellipsoid.
 *
 * This function uses an algorithm for an oblate spheroid earth model.
 * The algorithm is described here:
 *   http://www.ngs.noaa.gov/PUBS_LIB/inverse.pdf
 *
 * @param from The 'from' position (in radians)
 * @param to The 'to' position (in radians)
 * @param azimuth Azimuth (in radians)
 * @param distance The distance (in km)
 * @param toAzimuth Azimuth at end position (in radians)
 * @return True on success
 */
bool nmeaMathMoveFlatEllipsoid(const NmeaPosition *from, NmeaPosition *to, double azimuth, double distance,
    double *toAzimuth);

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* __NMEALIB_MATH_H__ */
