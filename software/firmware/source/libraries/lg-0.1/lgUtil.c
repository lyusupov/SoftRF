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

#include <stdlib.h>
#include <inttypes.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "lgpio.h"

#include "lgDbg.h"

static char xConfigDir[LG_MAX_PATH];
static char xWorkDir[LG_MAX_PATH];

uint64_t lguTimestamp(void)
{
   struct timespec xts;

   clock_gettime(CLOCK_REALTIME, &xts); // get current time

   return ((uint64_t)1E9 * xts.tv_sec) + xts.tv_nsec;
}

double lguTime(void)
{
   return (double)lguTimestamp() / (double)1E9;
}

void lguSleep(double sleepSecs)
{
   struct timespec ts, rem;

   if (sleepSecs > 0.0)
   {
      ts.tv_sec = sleepSecs;
      ts.tv_nsec = (sleepSecs-(double)ts.tv_sec) * 1E9;

      while (clock_nanosleep(CLOCK_REALTIME, 0, &ts, &rem)) ts = rem;
   }
}

int lguSbcName(char *buf, int count)
{
   LG_DBG(LG_DEBUG_TRACE, "");

   if ((buf == NULL)  || (count < 1)) return 0;

   gethostname(buf, count);
   buf[count-1] = 0;

   return strlen(buf) + 1; /* include null terminator in count */
}

int lguVersion(void)
{
   LG_DBG(LG_DEBUG_TRACE, "");

   return LGPIO_VERSION;
}

void xSetConfigDir(const char *dirStr)
{
   if (dirStr[0] == '/') // absolute path
   {
      if (strlen(dirStr) < LG_MAX_PATH)
         strcpy(xConfigDir, dirStr);
      else
         strcpy(xConfigDir, "/tmp");
   }
   else  // relative to working directory
   {
      if (getcwd(xConfigDir, LG_MAX_PATH) != NULL)
      {
         // only append relative path if there is room
         if ((strlen(xConfigDir) + strlen(dirStr) + 2) < LG_MAX_PATH)
         {
            strcat(xConfigDir, "/");
            strcat(xConfigDir, dirStr);
         }
      }
      else strcpy(xConfigDir, "/tmp");
   }
}

void lguSetConfigDir(const char *dirPath)
{
   LG_DBG(LG_DEBUG_TRACE, "dirPath=%s", dirPath);

   if (!xConfigDir[0])
   {
      lguGetWorkDir(); // make sure working directory is known

      xSetConfigDir(dirPath);
   }
}

const char *lguGetConfigDir(void)
{
   const char *dirStr;

   if (!xConfigDir[0])
   {
      lguGetWorkDir(); // make sure working directory is known

      dirStr = getenv(LG_CD);

      if (dirStr) xSetConfigDir(dirStr);
      else
      {
         if (getcwd(xConfigDir, LG_MAX_PATH) == NULL)
            strcpy(xConfigDir, "/tmp");
      }
   }

   return xConfigDir;
}

void xSetWorkDir(const char *dirStr)
{
   if (dirStr[0] == '/') // absolute path
   {
      if (strlen(dirStr) < LG_MAX_PATH)
         strcpy(xWorkDir, dirStr);
      else
         strcpy(xWorkDir, "/tmp");
   }
   else  // relative to working directory
   {
      if (getcwd(xWorkDir, LG_MAX_PATH) != NULL)
      {
         if ((strlen(xWorkDir) + strlen(dirStr) + 2) < LG_MAX_PATH)
         {
            strcat(xWorkDir, "/");
            strcat(xWorkDir, dirStr);
         }
      } 
      else strcpy(xWorkDir, "/tmp");
   }
}

void lguSetWorkDir(const char *dirPath)
{
   LG_DBG(LG_DEBUG_TRACE, "dirPath=%s", dirPath);

   if (!xWorkDir[0])
   {
      xSetWorkDir(dirPath);

      if (chdir(xWorkDir) < 0) LG_DBG(LG_DEBUG_ALWAYS,
         "can't set working directory (%s)", strerror(errno));
   }
}

const char *lguGetWorkDir(void)
{
   const char *dirStr;

   if (!xWorkDir[0])
   {
      dirStr = getenv(LG_WD);

      if (dirStr) xSetWorkDir(dirStr);
      else
      {
         if (getcwd(xWorkDir, LG_MAX_PATH) == NULL)
            strcpy(xWorkDir, "/tmp");
      }

      if (chdir(xWorkDir) < 0) LG_DBG(LG_DEBUG_ALWAYS,
         "can't set working directory (%s)", strerror(errno));
   }

   return xWorkDir;
}

int lguSetInternal(int cfgId, uint64_t cfgVal)
{
   LG_DBG(LG_DEBUG_TRACE, "Id=%d val=%"PRIu64"", cfgId, cfgVal);

   switch(cfgId)
   {
      case LG_CFG_ID_DEBUG_LEVEL:
         lgDbgLevel = cfgVal | 1;
         break;

      case LG_CFG_ID_MIN_DELAY:
         if (cfgVal <= 1000) lgMinTxDelay = cfgVal;
         else return LG_BAD_CONFIG_VALUE;
         break;

      default:
         return LG_BAD_CONFIG_ID;
   }
   return LG_OKAY;
}

int lguGetInternal(int cfgId, uint64_t *cfgVal)
{
   LG_DBG(LG_DEBUG_TRACE, "Id=%d", cfgId);

   switch(cfgId)
   {
      case LG_CFG_ID_DEBUG_LEVEL:
         *cfgVal = lgDbgLevel;
         break;

      case LG_CFG_ID_MIN_DELAY:
         *cfgVal = lgMinTxDelay;
         break;

      default:
         *cfgVal = 0;
         return LG_BAD_CONFIG_ID;
   }
   return LG_OKAY;
}

