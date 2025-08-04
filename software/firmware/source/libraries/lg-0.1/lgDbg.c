/*
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org/>
*/

#include <inttypes.h>
#include <time.h>
#include <sys/time.h>

#include "lgpio.h"

#include "lgDbg.h"

#define LG_DBG_MAX_BUFS 8

uint64_t lgDbgLevel = LG_DEBUG_ALWAYS;

char *lgDbgStr2Hex(int count, const char *buf)
{
   static char str[LG_DBG_MAX_BUFS][128];
   static int which = 0;

   int i, c;

   if (++which >= LG_DBG_MAX_BUFS) which = 0;

   if (count && buf)
   {
      if (count > 40) c = 40; else c = count;

      for (i=0; i<c; i++) sprintf(str[which]+(3*i), "%02X ", buf[i]);
      str[which][(3*c)-1] = 0;
   }
   else str[which][0] = 0;

   return str[which];
}

/* ----------------------------------------------------------------------- */

char *lgDbgInt2Str(int count, const int *buf)
{
   static char str[LG_DBG_MAX_BUFS][128];
   static int which = 0;

   int i, pos;

   if (++which >= LG_DBG_MAX_BUFS) which = 0;

   if (count && buf)
   {
      pos = 0;

      for (i=0; i<count; i++)
      {
         pos += sprintf(str[which]+pos, "%d ", buf[i]);
         str[which][pos] = 0;
         if (pos > 100) break;
      }
   }
   else str[which][0] = 0;

   return str[which];
}

char *lgDbgTimeStamp(void)
{
   static struct timeval last;
   static char buf[32];
   struct timeval now;

   struct tm tmp;

   gettimeofday(&now, NULL);

   if (now.tv_sec != last.tv_sec)
   {
      localtime_r(&now.tv_sec, &tmp);
      strftime(buf, sizeof(buf), "%F %T", &tmp);
      last.tv_sec = now.tv_sec;
   }

   return buf;
}

char *lgDbgBuf2Str(int count, const char *buf)
{
   static char str[128];
   int i, c;

   if (count && buf)
   {
      if (count > 40) c = 40; else c = count;

      for (i=0; i<c; i++) sprintf(str+(3*i), "%02X ", buf[i]);
      str[(3*c)-1] = 0;
   }
   else str[0] = 0;

   return str;
}

