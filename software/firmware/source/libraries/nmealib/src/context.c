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

#include <nmealib/context.h>

#include <nmealib/util.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

/**
 * The structure with the nmealib context.
 */
typedef struct _NmeaContext {
  volatile NmeaContextPrintFunction traceFunction;
  volatile NmeaContextPrintFunction errorFunction;
} NmeaContext;

/** The nmealib context */
static NmeaContext nmealibContext = {
    .traceFunction = NULL,
    .errorFunction = NULL };

NmeaContextPrintFunction nmeaContextSetTraceFunction(NmeaContextPrintFunction traceFunction) {
  NmeaContextPrintFunction r = nmealibContext.traceFunction;
  nmealibContext.traceFunction = traceFunction;
  return r;
}

NmeaContextPrintFunction nmeaContextSetErrorFunction(NmeaContextPrintFunction errorFunction) {
  NmeaContextPrintFunction r = nmealibContext.errorFunction;
  nmealibContext.errorFunction = errorFunction;
  return r;
}

void nmeaContextTraceBuffer(const char *s, size_t sz) {
  NmeaContextPrintFunction f = nmealibContext.traceFunction;
  if (f && s && sz) {
    (*f)(s, sz);
  }
}

#define nmeaContextBufferEnlarge(buf, sz) { \
  if (!(buf = realloc(buf, sz))) { \
    /* can't be covered in a test */ \
    goto out; \
  } \
}

void nmeaContextTrace(const char *s, ...) {
  NmeaContextPrintFunction f = nmealibContext.traceFunction;
  if (s && f) {
    va_list args;
    va_list argsCopy;
    char *buf = NULL;
    size_t bufSz = NMEALIB_BUFFER_CHUNK_SIZE;
    int printedChars;

    va_start(args, s);
    va_copy(argsCopy, args);

    nmeaContextBufferEnlarge(buf, bufSz);
    buf[0] = '\0';

    printedChars = vsnprintf(buf, bufSz, s, args);
    if (printedChars <= 0) {
      goto out;
    }
    if ((size_t) printedChars >= bufSz) {
      bufSz = (size_t) printedChars + 1;
      nmeaContextBufferEnlarge(buf, bufSz);
      printedChars = vsnprintf(buf, bufSz, s, argsCopy);
    }

    buf[bufSz - 1] = '\0';

    (*f)(buf, (size_t) printedChars);

out:
    va_end(argsCopy);
    va_end(args);
    free(buf);
  }
}

void nmeaContextError(const char *s, ...) {
  NmeaContextPrintFunction f = nmealibContext.errorFunction;
  if (s && f) {
    va_list args;
    va_list argsCopy;
    char *buf = NULL;
    size_t bufSz = NMEALIB_BUFFER_CHUNK_SIZE;
    int printedChars;

    va_start(args, s);
    va_copy(argsCopy, args);

    nmeaContextBufferEnlarge(buf, bufSz);
    buf[0] = '\0';

    printedChars = vsnprintf(buf, bufSz, s, args);
    if (printedChars <= 0) {
      goto out;
    }
    if ((size_t) printedChars >= bufSz) {
      bufSz = (size_t) printedChars + 1;
      nmeaContextBufferEnlarge(buf, bufSz);
      printedChars = vsnprintf(buf, bufSz, s, argsCopy);
    }

    buf[bufSz - 1] = '\0';

    (*f)(buf, (size_t) printedChars);

out:
    va_end(argsCopy);
    va_end(args);
    free(buf);
  }
}
