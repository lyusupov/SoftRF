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

#include <nmealib/info.h>
#include <nmealib/nmath.h>
#include <nmealib/parser.h>
#include <stdio.h>
#include <string.h>

#define NUM_POINTS 4

int main(int argc __attribute__((unused)), char *argv[] __attribute__((unused))) {
  const char *buf[] = {
      "$GPRMC,213916.199,A,4221.0377,N,07102.9778,W,0.00,,010207,,,A*6A\r\n",
      "$GPRMC,213917.199,A,4221.0510,N,07102.9549,W,0.23,175.43,010207,,,A*77\r\n",
      "$GPRMC,213925.000,A,4221.1129,N,07102.9146,W,0.00,,010207,,,A*68\r\n",
      "$GPRMC,111609.14,A,5001.27,N,3613.06,E,11.2,0.0,261206,0.0,E*50\r\n" };

  NmeaPosition pos[NUM_POINTS], pos_moved[NUM_POINTS][2];
  double dist[NUM_POINTS][2];
  double azimuth[NUM_POINTS][2], azimuth_moved[NUM_POINTS];
  int result[2];
  int it = 0;

  NmeaParser parser;
  nmeaParserInit(&parser, 0);

  for (it = 0; it < NUM_POINTS; it++) {
    NmeaInfo info;
    nmeaInfoClear(&info);
    (void) nmeaParserParse(&parser, buf[it], strlen(buf[it]), &info);
    nmeaMathInfoToPosition(&info, &pos[it]);
  }

  for (it = 0; it < NUM_POINTS; it++) {
    dist[it][0] = nmeaMathDistance(&pos[0], &pos[it]);
    dist[it][1] = nmeaMathDistanceEllipsoid(&pos[0], &pos[it], &azimuth[it][0], &azimuth[it][1]);
  }

  for (it = 0; it < NUM_POINTS; it++) {
    result[0] = nmeaMathMoveFlat(&pos[0], &pos_moved[it][0], azimuth[it][0], dist[it][0]);
    result[1] = nmeaMathMoveFlatEllipsoid(&pos[0], &pos_moved[it][1], azimuth[it][0], dist[it][0], &azimuth_moved[it]);

  }

  /* Output of results */
  printf("Coordinate points:\n");
  for (it = 0; it < NUM_POINTS; it++) {
    printf("P%d in radians: lat:%9.6lf lon:%9.6lf  \tin degree: lat:%+010.6lf° lon:%+011.6lf°\n", it, pos[it].lat,
        pos[it].lon, nmeaMathRadianToDegree(pos[it].lat), nmeaMathRadianToDegree(pos[it].lon));
  }

  printf("\nCalculation results:\n");
  for (it = 0; it < NUM_POINTS; it++) {
    printf("\n");
    printf("Distance P0 to P%d\ton spheroid:  %14.3lf m\n", it, dist[it][0]);
    printf("Distance P0 to P%d\ton ellipsoid: %14.3lf m\n", it, dist[it][1]);
    printf("Azimuth  P0 to P%d\tat start: %8.3lf°\tat end: %8.3lf°\n", it, nmeaMathRadianToDegree(azimuth[it][0]),
        nmeaMathRadianToDegree(azimuth[it][1]));
    printf("Move     P0 to P%d\t         \tAzimuth at end: %8.3lf°\n", it, nmeaMathRadianToDegree(azimuth_moved[it]));
    printf("Move     P0 to P%d\ton spheroid:  %3s lat:%+010.6lf° lon:%+011.6lf°\n", it, result[0] == 1 ?
        "OK" :
        "nOK", nmeaMathRadianToDegree(pos_moved[it][0].lat), nmeaMathRadianToDegree(pos_moved[it][0].lon));
    printf("Move     P0 to P%d\ton ellipsoid: %3s lat:%+010.6lf° lon:%+011.6lf°\n", it, result[0] == 1 ?
        "OK" :
        "nOK", nmeaMathRadianToDegree(pos_moved[it][1].lat), nmeaMathRadianToDegree(pos_moved[it][1].lon));
    printf("Move     P0 to P%d\toriginal:         lat:%+010.6lf° lon:%+011.6lf°\n", it,
        nmeaMathRadianToDegree(pos[it].lat), nmeaMathRadianToDegree(pos[it].lon));
  }

  nmeaParserDestroy(&parser);

  return 0;
}
