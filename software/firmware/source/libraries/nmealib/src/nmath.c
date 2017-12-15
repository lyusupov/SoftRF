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

#include <nmealib/nmath.h>

#include <nmealib/util.h>
#include <math.h>

double nmeaMathDegreeToRadian(const double v) {
  return (v * NMEALIB_DEGREE_TO_RADIAN);
}

double nmeaMathRadianToDegree(const double v) {
  return (v * NMEALIB_RADIAN_TO_DEGREE);
}

double nmeaMathNdegToDegree(const double v) {
  double hours;
  double minSec = modf(v / 100.0, &hours);

  return (hours + ((minSec * 10.0) / 6.0));
}

double nmeaMathDegreeToNdeg(const double v) {
  double degrees;
  double faction = modf(v, &degrees);

  return ((degrees * 100.0) + (faction * 60.0));
}

double nmeaMathNdegToRadian(const double v) {
  return nmeaMathDegreeToRadian(nmeaMathNdegToDegree(v));
}

double nmeaMathRadianToNdeg(const double v) {
  return nmeaMathDegreeToNdeg(nmeaMathRadianToDegree(v));
}

double nmeaMathPdopCalculate(const double hdop, const double vdop) {
  return sqrt(pow(hdop, 2) + pow(vdop, 2));
}

double nmeaMathDopToMeters(const double dop) {
  return (dop * NMEALIB_DOP_TO_METER);
}

double nmeaMathMetersToDop(const double meters) {
  return (meters * NMEALIB_METER_TO_DOP);
}

void nmeaMathInfoToPosition(const NmeaInfo *info, NmeaPosition *pos) {
  if (!pos) {
    return;
  }

  pos->lat = nmeaMathNdegToRadian(NMEALIB_LATITUDE_DEFAULT_NDEG);
  pos->lon = nmeaMathNdegToRadian(NMEALIB_LONGITUDE_DEFAULT_NDEG);

  if (!info) {
    return;
  }

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_LAT)) {
    pos->lat = nmeaMathNdegToRadian(info->latitude);
  }

  if (nmeaInfoIsPresentAll(info->present, NMEALIB_PRESENT_LON)) {
    pos->lon = nmeaMathNdegToRadian(info->longitude);
  }
}

void nmeaMathPositionToInfo(const NmeaPosition *pos, NmeaInfo *info) {
  if (!info) {
    return;
  }

  info->latitude = NMEALIB_LATITUDE_DEFAULT_NDEG;
  info->longitude = NMEALIB_LONGITUDE_DEFAULT_NDEG;

  if (!pos) {
    return;
  }

  info->latitude = nmeaMathRadianToNdeg(pos->lat);
  info->longitude = nmeaMathRadianToNdeg(pos->lon);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_LAT | NMEALIB_PRESENT_LON);
}

double nmeaMathDistance(const NmeaPosition *from, const NmeaPosition *to) {
  if (!from //
      || !to) {
    return NaN;
  }

  if ((from->lat == to->lat) //
      && (from->lon == to->lon)) {
    return 0.0;
  }

  return ((double) NMEALIB_EARTHRADIUS_M) * acos(sin(to->lat) * sin(from->lat) //
      + cos(to->lat) * cos(from->lat) * cos(to->lon - from->lon));
}

double nmeaMathDistanceEllipsoid(const NmeaPosition *from, const NmeaPosition *to, double *fromAzimuth,
    double *toAzimuth) {
  /* All variables */
  double f, a, b, sqr_a, sqr_b;
  double L, phi1, phi2, U1, U2, sin_U1, sin_U2, cos_U1, cos_U2;
  double sigma, sin_sigma, cos_sigma, cos_2_sigmam, sqr_cos_2_sigmam, sqr_cos_alpha, lambda, sin_lambda, cos_lambda,
      delta_lambda;
  int remaining_steps;
  double sqr_u, A, B, delta_sigma, lambda_prev;

  if (!from //
      || !to) {
    return NaN;
  }

  if ((from->lat == to->lat) //
      && (from->lon == to->lon)) {
    if (fromAzimuth) {
      *fromAzimuth = 0.0;
    }
    if (toAzimuth) {
      *toAzimuth = 0.0;
    }

    return 0.0;
  }

  /* Earth geometry */
  f = NMEALIB_EARTH_FLATTENING;
  a = NMEALIB_EARTH_SEMIMAJORAXIS_M;
  b = (1 - f) * a;
  sqr_a = a * a;
  sqr_b = b * b;

  /* Calculation */
  L = to->lon - from->lon;
  phi1 = from->lat;
  phi2 = to->lat;
  U1 = atan((1 - f) * tan(phi1));
  U2 = atan((1 - f) * tan(phi2));
  sin_U1 = sin(U1);
  sin_U2 = sin(U2);
  cos_U1 = cos(U1);
  cos_U2 = cos(U2);

  /* Initialize iteration */
  sigma = 0;
  sin_sigma = sin(sigma);
  cos_sigma = cos(sigma);
  cos_2_sigmam = 0;
  sqr_cos_2_sigmam = cos_2_sigmam * cos_2_sigmam;
  sqr_cos_alpha = 0;
  lambda = L;
  sin_lambda = sin(lambda);
  cos_lambda = cos(lambda);
  lambda_prev = (double) 2.0 * (double) NMEALIB_PI;
  delta_lambda = lambda_prev - lambda;
  if (delta_lambda < 0) {
    delta_lambda = -delta_lambda;
  }
  remaining_steps = 20;

  while ((delta_lambda > 1e-12) && (remaining_steps > 0)) {
    double tmp1, tmp2, sin_alpha, cos_alpha, C;

    /* Calculation */
    tmp1 = cos_U2 * sin_lambda;
    tmp2 = cos_U1 * sin_U2 - sin_U1 * cos_U2 * cos_lambda;
    sin_sigma = sqrt(tmp1 * tmp1 + tmp2 * tmp2);
    cos_sigma = sin_U1 * sin_U2 + cos_U1 * cos_U2 * cos_lambda;
    sin_alpha = cos_U1 * cos_U2 * sin_lambda / sin_sigma;
    cos_alpha = cos(asin(sin_alpha));
    sqr_cos_alpha = cos_alpha * cos_alpha;
    cos_2_sigmam = cos_sigma - 2 * sin_U1 * sin_U2 / sqr_cos_alpha;
    sqr_cos_2_sigmam = cos_2_sigmam * cos_2_sigmam;
    C = f / 16 * sqr_cos_alpha * (4 + f * (4 - 3 * sqr_cos_alpha));
    lambda_prev = lambda;
    sigma = asin(sin_sigma);
    lambda = L
        + (1 - C) * f * sin_alpha
            * (sigma + C * sin_sigma * (cos_2_sigmam + C * cos_sigma * (-1 + 2 * sqr_cos_2_sigmam)));
    delta_lambda = lambda_prev - lambda;
    if (delta_lambda < 0)
      delta_lambda = -delta_lambda;
    sin_lambda = sin(lambda);
    cos_lambda = cos(lambda);
    remaining_steps--;
  }

  /* More calculation  */
  sqr_u = sqr_cos_alpha * (sqr_a - sqr_b) / sqr_b;
  A = 1 + sqr_u / 16384 * (4096 + sqr_u * (-768 + sqr_u * (320 - 175 * sqr_u)));
  B = sqr_u / 1024 * (256 + sqr_u * (-128 + sqr_u * (74 - 47 * sqr_u)));
  delta_sigma = B * sin_sigma
      * (cos_2_sigmam
          + B / 4
              * (cos_sigma * (-1 + 2 * sqr_cos_2_sigmam)
                  - B / 6 * cos_2_sigmam * (-3 + 4 * sin_sigma * sin_sigma) * (-3 + 4 * sqr_cos_2_sigmam)));

  /* Calculate result */
  if (fromAzimuth) {
    double tan_alpha_1 = cos_U2 * sin_lambda / (cos_U1 * sin_U2 - sin_U1 * cos_U2 * cos_lambda);
    *fromAzimuth = atan(tan_alpha_1);
  }
  if (toAzimuth) {
    double tan_alpha_2 = cos_U1 * sin_lambda / (-sin_U1 * cos_U2 + cos_U1 * sin_U2 * cos_lambda);
    *toAzimuth = atan(tan_alpha_2);
  }

  return b * A * (sigma - delta_sigma);
}

bool nmeaMathMoveFlat(const NmeaPosition *from, NmeaPosition *to, double azimuth, double distance) {
  NmeaPosition pos;

  if (!from //
      || !to) {
    return false;
  }

  if (isNaN(from->lat) //
      || isNaN(from->lon)) {
    to->lat = NaN;
    to->lon = NaN;
    return false;
  }

  pos = *from;
  distance /= NMEALIB_EARTHRADIUS_KM; /* Angular distance covered on earth's surface */
  azimuth = nmeaMathDegreeToRadian(azimuth);

  to->lat = asin(sin(pos.lat) * cos(distance) + cos(pos.lat) * sin(distance) * cos(azimuth));
  to->lon = pos.lon + atan2(sin(azimuth) * sin(distance) * cos(pos.lat), cos(distance) - sin(pos.lat) * sin(to->lat));

  return true;
}

bool nmeaMathMoveFlatEllipsoid(const NmeaPosition *from, NmeaPosition *to, double azimuth, double distance,
    double *toAzimuth) {
  /* Variables */
  double f, a, b, sqr_a, sqr_b;
  double phi1, tan_U1, sin_U1, cos_U1, s, alpha1, sin_alpha1, cos_alpha1;
  double sigma1, sin_alpha, sqr_cos_alpha, sqr_u, A, B;
  double sigma_initial, sigma, sigma_prev, sin_sigma, cos_sigma, cos_2_sigmam, sqr_cos_2_sigmam, delta_sigma;
  int remaining_steps;
  double tmp1, phi2, lambda, C, L;

  if (!from //
      || !to) {
    return false;
  }

  if (isNaN(from->lat) //
      || isNaN(from->lon)) {
    to->lat = NaN;
    to->lon = NaN;
    if (toAzimuth) {
      *toAzimuth = NaN;
    }

    return false;
  }

  if (fabs(distance) < 1e-12) {
    *to = *from;
    if (toAzimuth) {
      *toAzimuth = azimuth;
    }

    return true;
  }

  /* Earth geometry */
  f = NMEALIB_EARTH_FLATTENING;
  a = NMEALIB_EARTH_SEMIMAJORAXIS_M;
  b = (1 - f) * a;
  sqr_a = a * a;
  sqr_b = b * b;

  /* Calculation */
  phi1 = from->lat;
  tan_U1 = (1 - f) * tan(phi1);
  cos_U1 = 1 / sqrt(1 + tan_U1 * tan_U1);
  sin_U1 = tan_U1 * cos_U1;
  s = distance;
  alpha1 = azimuth;
  sin_alpha1 = sin(alpha1);
  cos_alpha1 = cos(alpha1);
  sigma1 = atan2(tan_U1, cos_alpha1);
  sin_alpha = cos_U1 * sin_alpha1;
  sqr_cos_alpha = 1 - sin_alpha * sin_alpha;
  sqr_u = sqr_cos_alpha * (sqr_a - sqr_b) / sqr_b;
  A = 1 + sqr_u / 16384 * (4096 + sqr_u * (-768 + sqr_u * (320 - 175 * sqr_u)));
  B = sqr_u / 1024 * (256 + sqr_u * (-128 + sqr_u * (74 - 47 * sqr_u)));

  /* Initialize iteration */
  sigma_initial = s / (b * A);
  sigma = sigma_initial;
  sin_sigma = sin(sigma);
  cos_sigma = cos(sigma);
  cos_2_sigmam = cos(2 * sigma1 + sigma);
  sqr_cos_2_sigmam = cos_2_sigmam * cos_2_sigmam;
  delta_sigma = 0;
  sigma_prev = 2 * NMEALIB_PI;
  remaining_steps = 20;

  while ((fabs(sigma - sigma_prev) > 1e-12) && (remaining_steps > 0)) {
    cos_2_sigmam = cos(2 * sigma1 + sigma);
    sqr_cos_2_sigmam = cos_2_sigmam * cos_2_sigmam;
    sin_sigma = sin(sigma);
    cos_sigma = cos(sigma);
    delta_sigma = B * sin_sigma
        * (cos_2_sigmam
            + B / 4
                * (cos_sigma * (-1 + 2 * sqr_cos_2_sigmam)
                    - B / 6 * cos_2_sigmam * (-3 + 4 * sin_sigma * sin_sigma) * (-3 + 4 * sqr_cos_2_sigmam)));
    sigma_prev = sigma;
    sigma = sigma_initial + delta_sigma;
    remaining_steps--;
  }

  /* Calculate result */
  tmp1 = (sin_U1 * sin_sigma - cos_U1 * cos_sigma * cos_alpha1);
  phi2 = atan2(sin_U1 * cos_sigma + cos_U1 * sin_sigma * cos_alpha1,
      (1 - f) * sqrt(sin_alpha * sin_alpha + tmp1 * tmp1));
  lambda = atan2(sin_sigma * sin_alpha1, cos_U1 * cos_sigma - sin_U1 * sin_sigma * cos_alpha1);
  C = f / 16 * sqr_cos_alpha * (4 + f * (4 - 3 * sqr_cos_alpha));
  L = lambda
      - (1 - C) * f * sin_alpha
          * (sigma + C * sin_sigma * (cos_2_sigmam + C * cos_sigma * (-1 + 2 * sqr_cos_2_sigmam)));

  /* Result */
  to->lon = from->lon + L;
  to->lat = phi2;
  if (toAzimuth) {
    *toAzimuth = atan2(sin_alpha, -sin_U1 * sin_sigma + cos_U1 * cos_sigma * cos_alpha1);
  }

  return true;
}
