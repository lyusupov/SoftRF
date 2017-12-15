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

#include <nmealib/generator.h>

#include <nmealib/context.h>
#include <nmealib/nmath.h>
#include <nmealib/sentence.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/*
 * Forward declarations
 */

bool nmeaGeneratorInvokeNoise(NmeaGenerator *gen, NmeaInfo *info);

bool nmeaGeneratorInitStatic(NmeaGenerator *gen, NmeaInfo *info);
bool nmeaGeneratorInvokeStatic(NmeaGenerator *gen, NmeaInfo *info);
bool nmeaGeneratorResetStatic(NmeaGenerator *gen, NmeaInfo *info);

bool nmeaGeneratorInitRotate(NmeaGenerator *gen, NmeaInfo *info);
bool nmeaGeneratorInvokeRotate(NmeaGenerator *gen, NmeaInfo *info);
bool nmeaGeneratorResetRotate(NmeaGenerator *gen, NmeaInfo *info);

bool nmeaGeneratorInitRandomMove(NmeaGenerator *gen, NmeaInfo *info);
bool nmeaGeneratorInvokeRandomMove(NmeaGenerator *gen, NmeaInfo *info);

/*
 * NOISE generator
 */

/**
 * NOISE Generator invoke function
 *
 * Does not touch smask nor utc in info.
 *
 * @param gen The generator
 * @param info The info structure to use during generation
 * @return True on success
 */
bool nmeaGeneratorInvokeNoise(NmeaGenerator *gen __attribute__ ((unused)), NmeaInfo *info) {
  size_t i;
  size_t inUseCount;

  if (!info) {
    return false;
  }

  info->sig = (int) lrint(nmeaRandom(NMEALIB_SIG_FIX, NMEALIB_SIG_SENSITIVE));
  info->fix = (int) lrint(nmeaRandom(NMEALIB_FIX_2D, NMEALIB_FIX_3D));
  info->pdop = nmeaRandom(0.0, 9.0);
  info->hdop = nmeaRandom(0.0, 9.0);
  info->vdop = nmeaRandom(0.0, 9.0);
  info->latitude = nmeaRandom(0.0, 100.0);
  info->longitude = nmeaRandom(0.0, 100.0);
  info->elevation = nmeaRandom(-100.0, 100.0);
  info->height = nmeaRandom(-100.0, 100.0);
  info->speed = nmeaRandom(0.0, 100.0);
  info->track = nmeaRandom(0.0, 360.0);
  info->mtrack = nmeaRandom(0.0, 360.0);
  info->magvar = nmeaRandom(0.0, 360.0);
  info->dgpsAge = nmeaRandom(0.0, 100.0);
  info->dgpsSid = (unsigned int) lrint(nmeaRandom(0.0, 100.0));

  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SIG);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_FIX);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_PDOP);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_HDOP);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_VDOP);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_LAT);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_LON);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_ELV);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_HEIGHT);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SPEED);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_TRACK);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_MTRACK);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_MAGVAR);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_DGPSAGE);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_DGPSSID);

  info->satellites.inUseCount = 0;
  info->satellites.inViewCount = 0;

  for (i = 0; i < NMEALIB_MAX_SATELLITES; i++) {
    inUseCount = (size_t) labs(lrint(nmeaRandom(0.0, 3.0)));

    info->satellites.inUse[i] = inUseCount ?
        (unsigned int) i :
        0;
    if (inUseCount) {
      info->satellites.inUseCount++;
    }

    info->satellites.inView[i].prn = (unsigned int) i;
    info->satellites.inView[i].elevation = (int) lrint(nmeaRandom(0.0, 90.0));
    info->satellites.inView[i].azimuth = (unsigned int) lrint(nmeaRandom(0.0, 359.0));
    info->satellites.inView[i].snr = inUseCount ?
        (unsigned int) lrint(nmeaRandom(40.0, 99.0)) :
        (unsigned int) lrint(nmeaRandom(0.0, 40.0));
    if (info->satellites.inView[i].snr) {
      info->satellites.inViewCount++;
    }
  }

  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINUSECOUNT);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINUSE);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINVIEWCOUNT);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINVIEW);

  return true;
}

/*
 * STATIC generator
 */

/**
 * STATIC Generator initialiser function
 *
 * Only touches sig, fix and satinfo in info.
 *
 * @param gen The generator
 * @param info The info structure to use during generation
 * @return True on success
 */
bool nmeaGeneratorInitStatic(NmeaGenerator *gen, NmeaInfo *info) {
  if (!gen //
      || !info) {
    return false;
  }

  info->sig = NMEALIB_SIG_SENSITIVE;
  info->fix = NMEALIB_FIX_3D;

  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SIG);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_FIX);

  nmeaGeneratorResetStatic(gen, info);

  return true;
}

/**
 * STATIC Generator invoke function
 *
 * Only touches utc in info.
 *
 * @param gen The generator
 * @param info The info structure to use during generation
 * @return True on success
 */
bool nmeaGeneratorInvokeStatic(NmeaGenerator *gen __attribute__ ((unused)), NmeaInfo *info) {
  if (!info) {
    return false;
  }

  nmeaTimeSet(&info->utc, &info->present, NULL);

  return true;
}

/**
 * STATIC Generator reset function
 *
 * Only resets the satinfo to 4 sats in inUse and in inView.
 *
 * @param gen The generator
 * @param info The info structure to use during generation
 * @return True on success
 */
bool nmeaGeneratorResetStatic(NmeaGenerator *gen __attribute__ ((unused)), NmeaInfo *info) {
  if (!info) {
    return false;
  }

  info->satellites.inUseCount = 4;
  info->satellites.inViewCount = 4;

  info->satellites.inUse[0] = 1;
  info->satellites.inView[0].prn = 1;
  info->satellites.inView[0].elevation = 50;
  info->satellites.inView[0].azimuth = 0;
  info->satellites.inView[0].snr = 99;

  info->satellites.inUse[1] = 2;
  info->satellites.inView[1].prn = 2;
  info->satellites.inView[1].elevation = 50;
  info->satellites.inView[1].azimuth = 90;
  info->satellites.inView[1].snr = 99;

  info->satellites.inUse[2] = 3;
  info->satellites.inView[2].prn = 3;
  info->satellites.inView[2].elevation = 50;
  info->satellites.inView[2].azimuth = 180;
  info->satellites.inView[2].snr = 99;

  info->satellites.inUse[3] = 4;
  info->satellites.inView[3].prn = 4;
  info->satellites.inView[3].elevation = 50;
  info->satellites.inView[3].azimuth = 270;
  info->satellites.inView[3].snr = 99;

  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINUSECOUNT);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINUSE);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINVIEWCOUNT);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINVIEW);

  return true;
}

/*
 * SAT_ROTATE generator
 */

/**
 * SAT_ROTATE Generator initialiser function
 *
 * Only touches sig, fix and satinfo in info.
 *
 * @param gen The generator
 * @param info The info structure to use during generation
 * @return True on success
 */
bool nmeaGeneratorInitRotate(NmeaGenerator *gen, NmeaInfo *info) {
  if (!gen //
      || !info) {
    return false;
  }

  info->sig = NMEALIB_SIG_SENSITIVE;
  info->fix = NMEALIB_FIX_3D;

  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SIG);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_FIX);

  nmeaGeneratorResetRotate(gen, info);

  return true;
}

/**
 * SAT_ROTATE Generator invoke function.
 *
 * @param gen The generator
 * @param info The info structure to use during generation
 * @return True on success
 */
bool nmeaGeneratorInvokeRotate(NmeaGenerator *gen __attribute__ ((unused)), NmeaInfo *info) {
  size_t i;
  size_t inViewCount;
  double degreesPerSatellite;
  double azimuth;

  if (!info) {
    return false;
  }

  inViewCount = info->satellites.inViewCount;
  degreesPerSatellite = 360.0 / (inViewCount ?
      (double) inViewCount :
      1.0);
  azimuth = (inViewCount ?
      (info->satellites.inView[0].azimuth) :
      0) + 5;

  nmeaTimeSet(&info->utc, &info->present, NULL);

  for (i = 0; i < inViewCount; i++) {
    while (azimuth >= 360.0) {
      azimuth -= 360.0;
    }
    info->satellites.inView[i].azimuth = (unsigned int) azimuth;
    azimuth += degreesPerSatellite;
  }

  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINVIEWCOUNT);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINVIEW);

  return true;
}

/**
 * SAT_ROTATE Generator reset function.
 *
 * @param gen The generator
 * @param info The info structure to use during generation
 * @return True on success
 */
bool nmeaGeneratorResetRotate(NmeaGenerator *gen __attribute__ ((unused)), NmeaInfo *info) {
  size_t i;
  double degrees = 360 / 8;
  double azimuth = 0;

  if (!info) {
    return false;
  }

  info->satellites.inUseCount = 8;
  info->satellites.inViewCount = 8;

  for (i = 0; i < info->satellites.inViewCount; i++) {
    info->satellites.inUse[i] = (unsigned int) (i + 1);
    info->satellites.inView[i].prn = (unsigned int) (i + 1);
    info->satellites.inView[i].elevation = 5;
    info->satellites.inView[i].azimuth = (unsigned int) azimuth;
    info->satellites.inView[i].snr = 80;
    azimuth += degrees;
  }

  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINUSECOUNT);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINUSE);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINVIEWCOUNT);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SATINVIEW);

  return true;
}

/*
 * POS_RANDMOVE generator
 */

/**
 * POS_RANDMOVE Generator initialiser function
 *
 * Only touches sig, fix, track, mtrack, magvar and speed in info.
 *
 * @param gen The generator
 * @param info The info structure to use during generation
 * @return True on success
 */
bool nmeaGeneratorInitRandomMove(NmeaGenerator *gen __attribute__ ((unused)), NmeaInfo *info) {
  if (!info) {
    return false;
  }

  info->sig = NMEALIB_SIG_SENSITIVE;
  info->fix = NMEALIB_FIX_3D;
  info->speed = 20.0;
  info->track = 0.0;
  info->mtrack = 0.0;
  info->magvar = 0.0;

  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SIG);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_FIX);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SPEED);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_TRACK);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_MTRACK);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_MAGVAR);

  return true;
}

/**
 * POS_RANDMOVE Generator invoke function
 *
 * @param gen The generator
 * @param info The info structure to use during generation
 * @return True on success
 */
bool nmeaGeneratorInvokeRandomMove(NmeaGenerator *gen __attribute__ ((unused)), NmeaInfo *info) {
  NmeaPosition pos;

  if (!info) {
    return false;
  }

  info->track += nmeaRandom(-10.0, 10.0);
  info->mtrack += nmeaRandom(-10.0, 10.0);
  info->speed += nmeaRandom(-2.0, 3.0);

  if (info->track < 0.0) {
    info->track = 360.0 + info->track;
  }
  if (info->track >= 360.0) {
    info->track -= 360.0;
  }

  if (info->mtrack < 0.0) {
    info->mtrack = 360.0 + info->mtrack;
  }
  if (info->mtrack >= 360.0) {
    info->mtrack -= 360.0;
  }

  if (info->speed < 1.0) {
    info->speed = 1.0;
  }
  if (info->speed > 40.0) {
    info->speed = 40.0;
  }


  nmeaMathInfoToPosition(info, &pos);
  nmeaMathMoveFlat(&pos, &pos, info->track, info->speed / 3600.0);
  nmeaMathPositionToInfo(&pos, info);

  info->magvar = info->track;

  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_LAT);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_LON);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SPEED);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_TRACK);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_MTRACK);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_MAGVAR);

  return true;
}

/*
 * Generator
 */

bool nmeaGeneratorInit(NmeaGenerator *gen, NmeaInfo *info) {
  bool r;
  uint32_t present;
  uint32_t smask;
  NmeaGenerator *g;

  if (!gen //
      || !info) {
    return false;
  }

  r = true;
  present = info->present;
  smask = info->smask;
  g = gen;

  nmeaInfoClear(info);
  nmeaTimeSet(&info->utc, &info->present, NULL);

  info->present = present;
  info->smask = smask;
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_SMASK);

  info->latitude = NMEALIB_LATITUDE_DEFAULT_NDEG;
  info->longitude = NMEALIB_LONGITUDE_DEFAULT_NDEG;
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_LAT);
  nmeaInfoSetPresent(&info->present, NMEALIB_PRESENT_LON);

  while (r && g) {
    if (g->init) {
      r = (*g->init)(g, info);
    }
    g = g->next;
  }

  return r;
}

NmeaGenerator *nmeaGeneratorCreate(NmeaGeneratorType type, NmeaInfo *info) {
  NmeaGenerator *gen = 0;

  if (!info) {
    return NULL;
  }

  gen = calloc(1, sizeof(NmeaGenerator));
  if (!gen) {
    /* can't be covered in a test */
    return NULL;
  }

  switch (type) {
    case NMEALIB_GENERATOR_NOISE:
      gen->invoke = nmeaGeneratorInvokeNoise;
      break;

    case NMEALIB_GENERATOR_STATIC:
    case NMEALIB_GENERATOR_SAT_STATIC:
      gen->init = nmeaGeneratorInitStatic;
      gen->invoke = nmeaGeneratorInvokeStatic;
      gen->reset = nmeaGeneratorResetStatic;
      break;

    case NMEALIB_GENERATOR_ROTATE:
    case NMEALIB_GENERATOR_SAT_ROTATE:
      gen->init = nmeaGeneratorInitRotate;
      gen->invoke = nmeaGeneratorInvokeRotate;
      gen->reset = nmeaGeneratorResetRotate;

      if (type == NMEALIB_GENERATOR_ROTATE) {
        nmeaGeneratorAppend(gen, nmeaGeneratorCreate(NMEALIB_GENERATOR_POS_RANDMOVE, info));
      }
      break;

    case NMEALIB_GENERATOR_POS_RANDMOVE:
      gen->init = nmeaGeneratorInitRandomMove;
      gen->invoke = nmeaGeneratorInvokeRandomMove;
      break;

    default:
      free(gen);
      return NULL;
  };

  nmeaGeneratorInit(gen, info);

  return gen;
}

bool nmeaGeneratorReset(NmeaGenerator *gen, NmeaInfo *info) {
  bool r = true;

  if (!gen //
      || !info) {
    return false;
  }

  if (gen->reset) {
    r = (*gen->reset)(gen, info);
  }

  return r;
}

void nmeaGeneratorDestroy(NmeaGenerator *gen) {
  if (!gen) {
    return;
  }

  if (gen->next) {
    nmeaGeneratorDestroy(gen->next);
    gen->next = NULL;
  }

  free(gen);
}

bool nmeaGeneratorInvoke(NmeaGenerator *gen, NmeaInfo *info) {
  bool r = true;

  if (!gen //
      || !info) {
    return false;
  }

  if (gen->invoke) {
    r = (*gen->invoke)(gen, info);
  }

  if (r //
      && gen->next) {
    r = nmeaGeneratorInvoke(gen->next, info);
  }

  return r;
}

void nmeaGeneratorAppend(NmeaGenerator *to, NmeaGenerator *gen) {
  NmeaGenerator *next;

  if (!to //
      || !gen //
      || (to == gen)) {
    return;
  }

  next = to;
  while (next->next) {
    next = next->next;
    if ((next == to) //
        || (next == gen)) {
      return;
    }
  }

  next->next = gen;
}

size_t nmeaGeneratorGenerateFrom(NmeaMallocedBuffer *buf, NmeaInfo *info, NmeaGenerator *gen, NmeaSentence mask) {
  size_t r;

  if (!buf //
      || (!buf->buffer && buf->bufferSize) //
      || (buf->buffer && !buf->bufferSize) //
      || !info //
      || !gen //
      || !mask) {
    return 0;
  }

  r = nmeaGeneratorInvoke(gen, info);
  if (!r) {
    return 0;
  }

  return nmeaSentenceFromInfo(buf, info, mask);
}
