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

int main(int argc __attribute__((unused)), char *argv[] __attribute__((unused))) {
  const char *buf[] = {
      "$GPRMC,173843,A,3349.896,N,11808.521,W,000.0,360.0,230108,013.4,E*69\r\n",
      "$GPGGA,111609.14,5001.27,N,3613.06,E,3,08,0.0,10.2,M,0.0,M,0.0,0000*70\r\n",
      "$GPGSV,2,1,08,01,05,005,80,02,05,050,80,03,05,095,80,04,05,140,80*7f\r\n",
      "$GPGSV,2,2,08,05,05,185,80,06,05,230,80,07,05,275,80,08,05,320,80*71\r\n",
      "$GPGSA,A,3,01,02,03,04,05,06,07,08,00,00,00,00,0.0,0.0,0.0*3a\r\n",
      "$GPRMC,111609.14,A,5001.27,N,3613.06,E,11.2,0.0,261206,0.0,E*50\r\n",
      "$GPVTG,217.5,T,208.8,M,000.00,N,000.01,K*4C\r\n" };

  int it;
  NmeaInfo info;
  NmeaParser parser;
  NmeaPosition dpos;

  nmeaInfoClear(&info);
  nmeaParserInit(&parser, 0);

  for (it = 0; it < 7; it++) {
    nmeaParserParse(&parser, buf[it], strlen(buf[it]), &info);

    nmeaMathInfoToPosition(&info, &dpos);
    printf("%03d, Lat: %f, Lon: %f, Sig: %d, Fix: %d\n", it, dpos.lat, dpos.lon, info.sig, info.fix);
  }

  nmeaParserDestroy(&parser);

  return 0;
}
