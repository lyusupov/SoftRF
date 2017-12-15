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
#include <nmealib/sentence.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int main(int argc __attribute__((unused)), char *argv[] __attribute__((unused))) {
  NmeaMallocedBuffer buf;
  NmeaInfo info;
  size_t i;

  memset(&buf, 0, sizeof(buf));
  nmeaInfoClear(&info);
  nmeaTimeSet(&info.utc, &info.present, NULL);

  info.sig = NMEALIB_SIG_SENSITIVE;
  info.fix = NMEALIB_FIX_3D;
  info.latitude = 5000.0;
  info.longitude = 3600.0;
  info.speed = 2.14 * NMEALIB_MPS_TO_KPH;
  info.elevation = 10.86;
  info.track = 45;
  info.mtrack = 55;
  info.magvar = 55;
  info.hdop = 2.3;
  info.vdop = 1.2;
  info.pdop = 2.594224354;

  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SIG);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_FIX);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LAT);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LON);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SPEED);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_ELV);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_TRACK);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_MTRACK);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_MAGVAR);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HDOP);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_VDOP);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_PDOP);

  info.satellites.inUseCount = NMEALIB_MAX_SATELLITES;
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINUSECOUNT);
  for (i = 0; i < NMEALIB_MAX_SATELLITES; i++) {
    info.satellites.inUse[i] = (unsigned int) (i + 1);
  }
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINUSE);

  info.satellites.inViewCount = NMEALIB_MAX_SATELLITES;
  for (i = 0; i < NMEALIB_MAX_SATELLITES; i++) {
    info.satellites.inView[i].prn = (unsigned int) i + 1;
    info.satellites.inView[i].elevation = (int) ((i * 10) % 90);
    info.satellites.inView[i].azimuth = (unsigned int) (i + 1);
    info.satellites.inView[i].snr = 99 - (unsigned int) i;
  }
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINVIEWCOUNT | NMEALIB_PRESENT_SATINVIEW);

  for (i = 0; i < 10; i++) {
    size_t gen_sz = nmeaSentenceFromInfo(&buf, &info, //
        NMEALIB_SENTENCE_GPGGA //
        | NMEALIB_SENTENCE_GPGSA //
        | NMEALIB_SENTENCE_GPGSV //
        | NMEALIB_SENTENCE_GPRMC //
        | NMEALIB_SENTENCE_GPVTG);
    if (gen_sz) {
      printf("%s\n", buf.buffer);
      usleep(500000);
      info.speed += .1;
    }
  }

  free(buf.buffer);
  buf.buffer = NULL;
  buf.bufferSize = 0;

  return 0;
}
