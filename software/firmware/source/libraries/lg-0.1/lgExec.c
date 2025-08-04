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
#include <string.h>
#include <stdlib.h>
#include <fnmatch.h>
#include <ctype.h>

#include "lgpio.h"
#include "rgpiod.h"

#include "lgCfg.h"
#include "lgCmd.h"
#include "lgCtx.h"
#include "lgDbg.h"
#include "lgHdl.h"
#include "lgMD5.h"

typedef enum
{
   DEV,
   DEVNEXT,
   DEV2,
   SUBDEV,
   SUBDEVNEXT,
   SUBDEV2,
   DOT,
   COMMA,
} checkDevSubdev_t;

static lgCfg_p Cfg;

static pthread_once_t xInited = PTHREAD_ONCE_INIT;

static uint64_t xMakeSalt(void)
{
   struct timespec xts;

   clock_gettime(CLOCK_REALTIME, &xts);

   return ((xts.tv_sec + xts.tv_nsec) * random()) + (random() + xts.tv_nsec);
}

static int xLoadConfig(void)
{
   char cfgFile[LG_MAX_PATH];

   snprintf(cfgFile, LG_MAX_PATH, "%s/permits", lguGetConfigDir());

   if (Cfg) lgCfgFree(Cfg);

   Cfg = lgCfgRead(cfgFile);

   if (Cfg) lgCfgPrint(Cfg, stderr);

   if (Cfg) return LG_OKAY; else return LG_BAD_CONFIG_FILE;
}

static void xInit(void)
{
   xLoadConfig();
}

int xCheckDevSubdev(char *str, int dev, int subdev)
{
   int num;
   int r1=0, r2=0;
   checkDevSubdev_t expect=DEV;

   while (1)
   {
      printf("%c", *str);
      switch (*str)
      {
         case '0':
         case '1':
         case '2':
         case '3':
         case '4':
         case '5':
         case '6':
         case '7':
         case '8':
         case '9':

            if ((expect == DEV) || (expect == SUBDEV))
            {
               num = 0;
               while (isdigit(*str))
                  {num = (num * 10) + (*str) - '0'; ++str;}
               --str;
               r1 = num;

               if (expect == DEV)
               {
                  if ((dev == r1) && (subdev < 0)) return 1;
                  expect = DEVNEXT;
               }
               else
               {
                  if (subdev == r1) return 1;
                  expect = SUBDEVNEXT;
               }
            }

            else if ((expect == DEV2) || (expect == SUBDEV2))
            {
               num = 0;
               while (isdigit(*str))
                  {num = (num * 10) + (*str) - '0'; ++str;}
               --str;
               r2 = num;

               if (expect == DEV2)
               {
                  if ((dev < r1) || (dev > r2)) return 0;  /* fail */
                  if (subdev < 0) return 1; /*pass */
                  expect = DOT;
               }
               else
               {
                  if ((subdev >= r1) && (subdev <= r2)) return 1;
                  expect = COMMA;
               }
            }

            else return 0;

            break;

         case '\0':
            return 0;
            break;

         case ' ':
         case '\t':
            break;

         case '*':
            if (expect == DEV)
            {
               if (subdev < 0) return 1; /* pass */
               expect = DOT;
            }

            else if (expect == SUBDEV) return 1;

            else return 0;

            break;
    
         case '-':

            if      (expect == DEVNEXT)    expect = DEV2;
            else if (expect == SUBDEVNEXT) expect = SUBDEV2;
            else return 0;

            break;
            
         case '.':

            if (expect == DOT) expect = SUBDEV;

            else if (expect == DEVNEXT)
            {
               if (dev != r1) return 0; /* can't pass */
               expect = SUBDEV;
            }

            else return 0;

            break;

         case ',':

            if ((expect == COMMA) || (expect == SUBDEVNEXT))
            {
               expect = SUBDEV;
            }
            else return 0;

            break;

         default:
            return 0;
      }
      ++str;
   }

   return 0;
}

static int xCheckDebugPermissions(lgCtx_p Ctx)
{
   if (Ctx->permits.debug)
   {
      if ((strcmp(Ctx->permits.debug, "Y") == 0) ||
          (strcmp(Ctx->permits.debug, "y") == 0))
      return 1;
   }
   return 0;
}

static int xCheckShellPermissions(lgCtx_p Ctx)
{
   if (Ctx->permits.shell)
   {
      if ((strcmp(Ctx->permits.shell, "Y") == 0) ||
          (strcmp(Ctx->permits.shell, "y") == 0))
      return 1;
   }
   return 0;
}

static int xCheckNotifyPermissions(lgCtx_p Ctx)
{
   if (Ctx->permits.notify)
   {
      if ((strcmp(Ctx->permits.notify, "Y") == 0) ||
          (strcmp(Ctx->permits.notify, "y") == 0))
      return 1;
   }
   return 0;
}


static int xCheckScriptPermissions(lgCtx_p Ctx)
{
   if (Ctx->permits.scripts)
   {
      if ((strcmp(Ctx->permits.scripts, "Y") == 0) ||
          (strcmp(Ctx->permits.scripts, "y") == 0))
      return 1;
   }
   return 0;
}


static int xCheckSerialPermissions(lgCtx_p Ctx, char *serDev)
{
   char buf[LG_MAX_PATH];
   char *str, *pos, *token;
   const char *delim = ":";

   if (!Ctx->permits.serial) return 0;

   strncpy(buf, Ctx->permits.serial, sizeof(buf)-1);
   buf[sizeof(buf)-1] = 0;

   str = buf;
   while ((token=lgCfgNextToken(&str, delim, &pos)))
   {
      LG_DBG(LG_DEBUG_ALWAYS, "%s and %s", serDev, token);

      if (fnmatch(token, serDev, 0) == 0) return 1;
   }
   return 0;
}

static int xCheckI2cPermissions(lgCtx_p Ctx, int i2cDev, int i2cAddr)
{
   char buf[LG_MAX_PATH];
   char *str, *pos, *token;
   const char *delim = ":";

   if (!Ctx->permits.i2c) return 0;

   strncpy(buf, Ctx->permits.i2c, sizeof(buf)-1);
   buf[sizeof(buf)-1] = 0;

   str = buf;
   while ((token=lgCfgNextToken(&str, delim, &pos)))
   {
      LG_DBG(LG_DEBUG_ALWAYS, "%d.%d and %s", i2cDev, i2cAddr, token);

      if (xCheckDevSubdev(token, i2cDev, i2cAddr)) return 1;
   }
   return 0;
}

static int xCheckSpiPermissions(lgCtx_p Ctx, int spiDev, int spiChan)
{
   char buf[LG_MAX_PATH];
   char *str, *pos, *token;
   const char *delim = ":";

   if (!Ctx->permits.spi) return 0;

   strncpy(buf, Ctx->permits.spi, sizeof(buf)-1);
   buf[sizeof(buf)-1] = 0;

   str = buf;
   while ((token=lgCfgNextToken(&str, delim, &pos)))
   {
      LG_DBG(LG_DEBUG_ALWAYS, "%d.%d and %s", spiDev, spiChan, token);

      if (xCheckDevSubdev(token, spiDev, spiChan)) return 1;
   }
   return 0;
}

static int xCheckGpioPermissions(lgCtx_p Ctx, int gpioDev, int gpio)
{
   char buf[LG_MAX_PATH];
   char *str, *pos, *token;
   const char *delim = ":";

   if (!Ctx->permits.gpio) return 0;

   strncpy(buf, Ctx->permits.gpio, sizeof(buf)-1);
   buf[sizeof(buf)-1] = 0;

   str = buf;
   while ((token=lgCfgNextToken(&str, delim, &pos)))
   {
      LG_DBG(LG_DEBUG_ALWAYS, "%d.%d and %s", gpioDev, gpio, token);

      if (xCheckDevSubdev(token, gpioDev, gpio)) return 1;
   }
   return 0;
}

static int xSetGpioPermissions(lgCtx_p Ctx, int gpioDev, int handle)
{
   lgChipInfo_t chipInfo;
   int i, banned;

   if (lgGpioGetChipInfo(handle, &chipInfo) == 0)
   {
      for (i=0; i<chipInfo.lines; i++)
      {
         if (xCheckGpioPermissions(Ctx, gpioDev, i)) banned = 0; else banned = 1;
         lgGpioSetBannedState(handle, i, banned);
      }
      return 0;
   }

   return 1;
}

static int xPathBad(char *name)
{
   if (strstr(name, "..")) return 1;
   if (strstr(name, "\\.")) return 1;
   if (strstr(name, "./")) return 1;
   return 0;
}

static int xApprove(lgCtx_p Ctx, char *filename, int mode)
{
   char match[LG_MAX_PATH];
   char buffer[LG_MAX_PATH];
   char paths[LG_MAX_PATH];
   char mperm=0;
   char perm;
   char term;
   char *token, *pos, *str;
   const char *delim =":";
   int approve = 0;

   if (!Ctx->permits.files) return 0;

   if (xPathBad(filename))  return 0;

   mode &= LG_FILE_RW;

   if (!mode) return 0;

   buffer[0] = 0;
   match[0] = 0;

   strncpy(paths, Ctx->permits.files, sizeof(paths)-1);
   paths[sizeof(paths)-1] = 0;

   str = paths;
   while ((token=lgCfgNextToken(&str, delim, &pos)))
   {
      LG_DBG(LG_DEBUG_ALWAYS, "token: '%s'", token);
      buffer[0] = 0;
      perm = 0;
      term = 0;
      sscanf(token, " %1000s %c%c", buffer, &perm, &term);

      if (term != 0)
      {
         LG_DBG(LG_DEBUG_ALWAYS, "ignored, bad termination: %s", token);
         continue;
      }

      if (xPathBad(buffer))
      {
         LG_DBG(LG_DEBUG_ALWAYS, "ignored, risky: %s", token);
         continue;
      }

      if (fnmatch(buffer, filename, 0) == 0)
      {
         LG_DBG(LG_DEBUG_ALWAYS, "[%s] matches %s", buffer, filename);
         if (match[0])  // prior match?
         {
            if (fnmatch(match, buffer, 0) == 0)  // more precise match?
            {
               LG_DBG(LG_DEBUG_ALWAYS, "%s matches [%s]", match, buffer);
               strcpy(match, buffer); // most precise match so far
               mperm = perm;
            }
         }
         else  // no prior match
         {
            strcpy(match, buffer);
            mperm = perm;
         }
      }
   }

   if (match[0])
   {
      switch (toupper(mperm))
      {
         case 'R':
            if (mode == LG_FILE_READ) approve = 1;
            break;
         case 'W':
            if (mode == LG_FILE_WRITE) approve = 1;
            break;
         case 'U':
            approve = 1;
            break;
      }
   }

   return approve;
}

static void xClearUserPermits(lgCtx_p Ctx)
{
   memset(&Ctx->permits, 0, sizeof(Ctx->permits));
}

static void xSetUserPermits(lgCtx_p Ctx)
{
   if (Cfg)
   {
      Ctx->permits.files =   lgCfgGetValue(Cfg, "files",   Ctx->user);
      Ctx->permits.scripts = lgCfgGetValue(Cfg, "scripts", Ctx->user);
      Ctx->permits.i2c =     lgCfgGetValue(Cfg, "i2c",     Ctx->user);
      Ctx->permits.spi =     lgCfgGetValue(Cfg, "spi",     Ctx->user);
      Ctx->permits.serial =  lgCfgGetValue(Cfg, "serial",  Ctx->user);
      Ctx->permits.gpio =    lgCfgGetValue(Cfg, "gpio",    Ctx->user);
      Ctx->permits.notify =  lgCfgGetValue(Cfg, "notify",  Ctx->user);
      Ctx->permits.debug =   lgCfgGetValue(Cfg, "debug",   Ctx->user);
      Ctx->permits.shell =   lgCfgGetValue(Cfg, "shell",   Ctx->user);
   }
   else xClearUserPermits(Ctx);
}

static int xSetUser(lgCtx_p Ctx, char *user, char *buf)
{
   int result = LG_BAD_USERNAME;

   LG_DBG(LG_DEBUG_TRACE, "user=%s", user);

   if (strlen(user))
   {
      if (gPermits)
      {
         Ctx->approved = 0;
         xClearUserPermits(Ctx);
      }
      snprintf(Ctx->salt1, LG_SALT_LEN, "%s", user);
      snprintf(Ctx->user,  LG_USER_LEN, "%s", user+LG_SALT_LEN);
      snprintf(Ctx->salt2, LG_SALT_LEN, "%015"PRIx64, xMakeSalt());
      strcpy(buf, Ctx->salt2);
      result = LG_SALT_LEN;
   }

   return result;
}

static int xShareSetUse(lgCtx_p Ctx, int share)
{
   LG_DBG(LG_DEBUG_TRACE, "share=%d", share);

   Ctx->autoSetShare = share;
   Ctx->autoUseShare = share;

   return LG_OKAY;
}

static int xShareUse(lgCtx_p Ctx, int share)
{
   LG_DBG(LG_DEBUG_TRACE, "share=%d", share);

   Ctx->autoUseShare = share;

   return LG_OKAY;
}

static int xPassword(lgCtx_p Ctx, char *hash)
{
   int result;
   char myHash[34];
   char secretFile[LG_MAX_PATH];

   result = LG_BAD_SECRET;

   LG_DBG(LG_DEBUG_TRACE, "hash=%s", hash);

   if (!gPermits) return LG_OKAY;

   if (strcmp(Ctx->user, LG_DEFAULT_USER) == 0)
   {
      /* default user is a special case */
      Ctx->approved = 1;
      xSetUserPermits(Ctx);
      return LG_OKAY;
   }

   if ((strlen(lguGetConfigDir()) + strlen("/.lg_secret")) < LG_MAX_PATH)
   {
      sprintf(secretFile, "%s/.lg_secret", lguGetConfigDir());
   }
   else return result;

   lgMd5UserHash(Ctx->user, Ctx->salt1, Ctx->salt2, secretFile, myHash);

   LG_DBG(LG_DEBUG_TRACE, "myhash=%s", myHash);

   if (strcmp(hash, myHash) == 0)
   {
      result = LG_OKAY;
      Ctx->approved = 1;
      xSetUserPermits(Ctx);
   }
   return result;
}

int lgExecCmd(lgCmd_p cmdP, int cmdBufSize)
{
   static int xPid = 0;
   int res;
   uint32_t tmp1;
   int i;
   int size;
   lgCtx_p Ctx;
   lgLineInfo_t lInfo;
   lgChipInfo_t cInfo;
   res = LG_OKAY;
   char *cmdExt=(char*)&cmdP[1];
   uint32_t *argI=(uint32_t*)&cmdP[1];
   uint64_t *argQ=(uint64_t*)&cmdP[1];

   pthread_once(&xInited, xInit);

   Ctx = lgCtxGet();

   if (Ctx == NULL) return LG_NO_MEMORY;

   if (Ctx->owner == 0)
   {
      Ctx->owner = ++xPid;

      /* set default user if no preset user */
      if (!strlen(Ctx->user))
         strncpy(Ctx->user, LG_DEFAULT_USER, LG_USER_LEN);

      /* set permissions for user */
      Ctx->approved = 1;
      xSetUserPermits(Ctx);
   }

   size = cmdP->size;

   cmdP->size = 0;

   if (size) cmdExt[size] = 0; /* terminate trailing string */

   cmdBufSize -= sizeof(lgCmd_t);

/*
   fprintf(stderr, "owner=%d cmd=%d m0=%d m1=%d m2=%d m3=%d\n",
      Ctx->owner, cmdP->cmd, argI[0], argI[1], argI[2], argI[3]);    
*/

   switch (cmdP->cmd)
   {
      case LG_CMD_TICK:
         argQ[0] = lguTimestamp();
         cmdP->size = 8;
         res = 8;
         break;

      case LG_CMD_CGI:
         if (!gPermits || xCheckDebugPermissions(Ctx))
         {
            res = lguGetInternal(argI[0], &argQ[0]);
            if (res == LG_OKAY)
            {
               cmdP->size = 8;
               res = 8;
            }
         }
         else
            res = LG_NO_PERMISSIONS;
         break;

      case LG_CMD_CSI:
         if (!gPermits || xCheckDebugPermissions(Ctx))
            res = lguSetInternal(argI[2], argQ[0]);
         else
            res = LG_NO_PERMISSIONS;
         break;

      case LG_CMD_FC: res = lgFileClose(argI[0]); break;

      case LG_CMD_FL:
         res = LG_NO_FILE_ACCESS;

         if (xPathBad(cmdExt+4)) break;

         if (!gPermits || xApprove(Ctx, cmdExt+4, LG_FILE_READ))
         {
            if (argI[0] > (cmdBufSize-4)) argI[0] = cmdBufSize-4;
            res = lgFileList(cmdExt+4, cmdExt, argI[0]);
            if (res > 0) cmdP->size = res;
         }
         break;

      case LG_CMD_FO:
         res = LG_NO_FILE_ACCESS;

         if (xPathBad(cmdExt+4)) break;

         if (!gPermits || xApprove(Ctx, cmdExt+4, argI[0]))
         {
            res = lgFileOpen(cmdExt+4, argI[0]);
         }
         break;

      case LG_CMD_FR:
         if (argI[1] > cmdBufSize) argI[1] = cmdBufSize;
         res = lgFileRead(argI[0], cmdExt, argI[1]);
         if (res > 0) cmdP->size = res;
         break;

      case LG_CMD_FREE:
         lgHdlPurgeByOwner(Ctx->owner);
         res = 0;
         break;

      case LG_CMD_FS:
         res = lgFileSeek(argI[0], argI[1], argI[2]);
         break;

      case LG_CMD_FW: res = lgFileWrite(argI[0], cmdExt+4, size-4); break;

      case LG_CMD_I2CC: res = lgI2cClose(argI[0]); break;

      case LG_CMD_I2CO:
         if (!gPermits || xCheckI2cPermissions(Ctx, argI[0], argI[1]))
            res = lgI2cOpen(argI[0], argI[1], argI[2]);
         else
            res = LG_NO_PERMISSIONS;
         break;

      case LG_CMD_I2CPC:
         res = lgI2cProcessCall(argI[0], argI[1], argI[2]);
         break;

      case LG_CMD_I2CPK:
         res = lgI2cBlockProcessCall(argI[0], argI[1], cmdExt+8, size-8);
         if (res > 0)
         {
            memcpy(cmdExt, cmdExt+8, res);
            cmdP->size = res;
         }
         break;

      case LG_CMD_I2CRB: res = lgI2cReadByteData(argI[0], argI[1]); break;

      case LG_CMD_I2CRD:
         if (argI[1] > cmdBufSize) argI[1] = cmdBufSize;
         res = lgI2cReadDevice(argI[0], cmdExt, argI[1]);
         if (res > 0) cmdP->size = res;
         break;

      case LG_CMD_I2CRI:
         res = lgI2cReadI2CBlockData(argI[0], argI[1], cmdExt, argI[2]);
         if (res > 0) cmdP->size = res;
         break;

      case LG_CMD_I2CRK:
         res = lgI2cReadBlockData(argI[0], argI[1], cmdExt);
         if (res > 0) cmdP->size = res;
         break;

      case LG_CMD_I2CRS: res = lgI2cReadByte(argI[0]); break;

      case LG_CMD_I2CRW: res = lgI2cReadWordData(argI[0], argI[1]); break;

      case LG_CMD_I2CWB:
         res = lgI2cWriteByteData(argI[0], argI[1], argI[2]);
         break;

      case LG_CMD_I2CWD:
         res = lgI2cWriteDevice(argI[0], cmdExt+4, size-4);
         break;

      case LG_CMD_I2CWI:
         res = lgI2cWriteI2CBlockData(argI[0], argI[1], cmdExt+8, size-8);
         break;

      case LG_CMD_I2CWK:
         res = lgI2cWriteBlockData(argI[0], argI[1], cmdExt+8, size-8);
         break;

      case LG_CMD_I2CWQ: res = lgI2cWriteQuick(argI[0], argI[1]); break;

      case LG_CMD_I2CWS: res = lgI2cWriteByte(argI[0], argI[1]); break;

      case LG_CMD_I2CWW:
         res = lgI2cWriteWordData(argI[0], argI[1], argI[2]);
         break;

      case LG_CMD_I2CZ:
         if (size < cmdBufSize)
         {
            tmp1 = cmdBufSize - size;
            res = lgI2cZip(argI[0], cmdExt+4, size-4, cmdExt+size, tmp1);
            if (res > 0)
            {
               memcpy(cmdExt, cmdExt+size, res);
               cmdP->size = res;
            }
         }
         else res = LG_BAD_I2C_WLEN;
         break;

      case LG_CMD_LCFG:
         if (!gPermits || xCheckDebugPermissions(Ctx))
            res = xLoadConfig();
         else
            res = LG_NO_PERMISSIONS;
         break;

      case LG_CMD_MICS:
         if (argI[0] <= LG_MAX_MICS_DELAY) lguSleep((double)argI[0]/1E6);
         else res = LG_BAD_MICS_DELAY;
         break;

      case LG_CMD_MILS:
         if (argI[0] <= LG_MAX_MILS_DELAY) lguSleep((double)argI[0]/1E3);
         else res = LG_BAD_MILS_DELAY;
         break;

      case LG_CMD_NR: res = lgNotifyResume(argI[0]); break;

      case LG_CMD_NC: res = lgNotifyClose(argI[0]); break;

      case LG_CMD_NO:
         if (!gPermits || xCheckNotifyPermissions(Ctx))
            res = lgNotifyOpen();
         else
            res = LG_NO_PERMISSIONS;
         break;

      case LG_CMD_NOIB:
         res = lgNotifyOpenInBand(argI[0]);
         break;

      case LG_CMD_NP: res = lgNotifyPause(argI[0]); break;

      case LG_CMD_LGV: res = lguVersion(); break;

      case LG_CMD_PCD:
         strcpy(cmdExt, lguGetConfigDir());
         res = strlen(cmdExt);
         if (res > 0) cmdP->size = res;
         break;

      case LG_CMD_PROC:
         if (!gPermits || xCheckScriptPermissions(Ctx))
            res = lgScriptStore(cmdExt);
         else
            res = LG_NO_PERMISSIONS;
         break;

      case LG_CMD_PROCD: res = lgScriptDelete(argI[0]); break;

      case LG_CMD_PROCP:
         res = lgScriptStatus(argI[0], (uint32_t *)(cmdExt+4));
         if (res >= 0)
         {
            argI[0] = res;
            res = 4 + (4*LG_MAX_SCRIPT_PARAMS);
            cmdP->size = res;
         }
         break;

      case LG_CMD_PROCR:
         res = lgScriptRun(argI[0], (size/4)-1, &argI[1]);
         break;

      case LG_CMD_PROCS: res = lgScriptStop(argI[0]); break;

      case LG_CMD_PROCU:
         res = lgScriptUpdate(argI[0], (size/4)-1, &argI[1]);
         break;

      case LG_CMD_PWD:
         strcpy(cmdExt, lguGetWorkDir());
         res = strlen(cmdExt);
         if (res > 0) cmdP->size = res;
         break;

      case LG_CMD_SBC:
         res = lguSbcName(cmdExt, cmdBufSize);
         if (res > 0) cmdP->size = res;
         break;

      case LG_CMD_SERC: res = lgSerialClose(argI[0]); break;

      case LG_CMD_SERDA: res = lgSerialDataAvailable(argI[0]); break;

      case LG_CMD_SERO:
         if (!gPermits || xCheckSerialPermissions(Ctx, cmdExt+8))
            res = lgSerialOpen(cmdExt+8, argI[0], argI[1]);
         else
            res = LG_NO_PERMISSIONS;
         break;

      case LG_CMD_SERR:
         if (argI[1] > cmdBufSize) argI[1] = cmdBufSize;
         res = lgSerialRead(argI[0], cmdExt, argI[1]);
         if (res > 0) cmdP->size = res;
         break;

      case LG_CMD_SERRB: res = lgSerialReadByte(argI[0]); break;

      case LG_CMD_SERW: res = lgSerialWrite(argI[0], cmdExt+4, size-4); break;

      case LG_CMD_SERWB: res = lgSerialWriteByte(argI[0], argI[1]); break;

      case LG_CMD_SHELL:
         if (!gPermits || xCheckShellPermissions(Ctx))
            res = lgShell(cmdExt+4, cmdExt+4+argI[0]);
         else
            res = LG_NO_PERMISSIONS;
         break;

      case LG_CMD_SPIC:
         res = lgSpiClose(argI[0]);
         break;

      case LG_CMD_SPIO:
         if (!gPermits || xCheckSpiPermissions(Ctx, argI[0], argI[1]))
            res = lgSpiOpen(argI[0], argI[1], argI[2], argI[3]);
         else
            res = LG_NO_PERMISSIONS;
         break;

      case LG_CMD_SPIR:
         if (argI[1] > cmdBufSize) argI[1] = cmdBufSize;
         res = lgSpiRead(argI[0], cmdExt, argI[1]);
         if (res > 0) cmdP->size = res;
         break;

      case LG_CMD_SPIW:
         if (size < cmdBufSize)
            res = lgSpiWrite(argI[0], cmdExt+4, size-4);
         else
            res = LG_BAD_SPI_COUNT;
         break;

      case LG_CMD_SPIX:
         if (size < cmdBufSize)
         {
            res = lgSpiXfer(argI[0], cmdExt+4, cmdExt, size-4);
            if (res > 0) cmdP->size = res;
         }
         else
            res = LG_BAD_SPI_COUNT;
         break;

      case LG_CMD_USER:
         res = xSetUser(Ctx, cmdExt, cmdExt);
         if (res > 0) cmdP->size = res;
         break;

      case LG_CMD_PASSW:
         res = xPassword(Ctx, cmdExt);
         break;

      case LG_CMD_SHARE: // share
         res = xShareSetUse(Ctx, argI[0]);
         break;

      case LG_CMD_SHRS: // h share
         res = lgHdlSetShare(argI[0], argI[1]);
         break;

      case LG_CMD_SHRU: // share
         res = xShareUse(Ctx, argI[0]);
         break;

      case LG_CMD_GC:
         // handle
         res = lgGpiochipClose(argI[0]); break;

      case LG_CMD_GO:
         // gpioDev
         if (!gPermits || xCheckGpioPermissions(Ctx, argI[0], -1))
         {
            res = lgGpiochipOpen(argI[0]);
            if (res >= 0)
            {
               /* have handle, initialise permissions */
               if (gPermits) xSetGpioPermissions(Ctx, argI[0], res);
            }
         }
         else
            res = LG_NO_PERMISSIONS;
         break;

      case LG_CMD_GR:
         // handle gpio
         res = lgGpioRead(argI[0], argI[1]);
         break;

      case LG_CMD_GW:
         // handle gpio value
         res = lgGpioWrite(argI[0], argI[1], argI[2]);
         break;

      case LG_CMD_GDEB:
         // handle gpio value
         res = lgGpioSetDebounce(argI[0], argI[1], argI[2]);
         break;

      case LG_CMD_GWDOG:
         // handle gpio value
         res = lgGpioSetWatchdog(argI[0], argI[1], argI[2]);
         break;

      case LG_CMD_GSI:
         // handle gpio
         res = lgGpioClaimInput(argI[0],       0, argI[1]);
         break;

      case LG_CMD_GSIX:
         // handle lFlags gpio
         res = lgGpioClaimInput(argI[0], argI[1], argI[2]);
         break;

      case LG_CMD_GSGI:
         // handle size *gpios
         tmp1 = (size/4)-1;
         res = lgGroupClaimInput(
            argI[0], 0, tmp1, (const int *)argI+1);
         break;

      case LG_CMD_GSGIX:
         // handle lFlags size *gpios
         tmp1 = (size/4)-2;
         res = lgGroupClaimInput(
            argI[0], argI[1], tmp1, (const int *)argI+2);
         break;

      case LG_CMD_GSO:
         // handle gpio
         res = lgGpioClaimOutput(argI[0], 0, argI[1], 0);
         break;

      case LG_CMD_GSOX:
         // handle lFlags gpio value
         res = lgGpioClaimOutput(argI[0], argI[1], argI[2], argI[3]);
         break;

      case LG_CMD_GSGO:
         // handle size *gpios
         tmp1 = (size/4)-1;
         for (i=0; i<tmp1; i++) argI[1+tmp1+i] = 0; /* default GPIO low */
         res = lgGroupClaimOutput(
            argI[0], 0, tmp1,
            (const int *)argI+1, (const int *)argI+1+tmp1);
         break;

      case LG_CMD_GSGOX:
         // handle lFlags size *gpios *values
         tmp1 = ((size/4)-2)/2;
         res = lgGroupClaimOutput(
            argI[0], argI[1], tmp1,
            (const int *)argI+2, (const int *)argI+2+tmp1);
         break;

      case LG_CMD_GGR:
         // in: handle group
         // out: bits64 res
         res = lgGroupRead(argI[0], argI[1], &argQ[0]);
         if (res >= 0)
         {
            argI[2] = res;         // group size
            res = 12;
            cmdP->size = res;
         }
         break;

      case LG_CMD_GGW:
         // bitsQ handle group
         res = lgGroupWrite(argI[2], argI[3], argQ[0], -1);
         break;
         
      case LG_CMD_GGWX:
         // bitsQ maskQ handle group
         res = lgGroupWrite(argI[4], argI[5], argQ[0], argQ[1]);
         break;
         
      case LG_CMD_GSA:
         // handle gpio nfyh
         res = lgGpioClaimAlert(argI[0], 0, LG_BOTH_EDGES, argI[1], argI[2]);
         break;
         
      case LG_CMD_GSAX:
         // handle lFlags eFlags gpio nfyh
         res = lgGpioClaimAlert(argI[0], argI[1], argI[2], argI[3], argI[4]);
         break;
         
      case LG_CMD_GP:
         // handle gpio micros_on micros_off
         res = lgTxPulse(
            argI[0], argI[1], argI[2], argI[3], 0, 0);
         break;
       
      case LG_CMD_GPX:
         // handle gpio micros_on micros_off offset cycles
         res = lgTxPulse(
            argI[0], argI[1], argI[2], argI[3], argI[4], argI[5]);
         break;
       
      case LG_CMD_P:
         // handle gpio freq dutyc
         // freq and dutyc are received as 1000 times value
         res = lgTxPwm(
            argI[0], argI[1], argI[2]/1000.0, argI[3]/1000.0, 0, 0);
         break;
       
      case LG_CMD_PX:
         // handle gpio freq dutyc offset cycles
         // freq and dutyc are received as 1000 times value
         res = lgTxPwm(
            argI[0], argI[1], argI[2]/1000.0, argI[3]/1000.0,
            argI[4], argI[5]);
         break;
       
      case LG_CMD_S:
         // handle gpio width
         res = lgTxServo(
            argI[0], argI[1], argI[2], 50, 0, 0);
         break;
       
      case LG_CMD_SX:
         // handle gpio width freq offset cycles
         res = lgTxServo(
            argI[0], argI[1], argI[2], argI[3], argI[4], argI[5]);
         break;
       
      case LG_CMD_GWAVE:
         // pulseQ* handle gpio
         tmp1 = (size-8)/24;
         res = lgTxWave(
            argI[tmp1*6], argI[(tmp1*6)+1], tmp1, (lgPulse_p)&argQ[0]);
         break;

      case LG_CMD_GMODE:
         // handle gpio
         res = lgGpioGetMode(argI[0], argI[1]);
         break;

      case LG_CMD_GBUSY:
         // handle gpio type
         res = lgTxBusy(argI[0], argI[1], argI[2]);
         break;
         
      case LG_CMD_GROOM:
         // handle gpio type
         res = lgTxRoom(argI[0], argI[1], argI[2]);
         break;
         
      case LG_CMD_GSF:
         // handle gpio
         res = lgGpioFree(argI[0], argI[1]);
         break;
         
      case LG_CMD_GSGF:
         // handle gpio
         res = lgGroupFree(argI[0], argI[1]);
         break;

      case LG_CMD_GIC:
         // handle
         res = lgGpioGetChipInfo(argI[0], &cInfo);
         if (res == LG_OKAY)
         {
            argI[0] = cInfo.lines;

            memcpy(cmdExt+4, &cInfo.name, sizeof(cInfo.name));

            memcpy(cmdExt+4+sizeof(cInfo.name),
               &cInfo.label, sizeof(cInfo.label));

            res = 4 + sizeof(cInfo.name) + sizeof(cInfo.label);
            cmdP->size = res;
         }
         
         break;

      case LG_CMD_GIL:
         // handle gpio
         res = lgGpioGetLineInfo(argI[0], argI[1], &lInfo);
         if (res == LG_OKAY)
         {
            argI[0] = lInfo.offset;
            argI[1] = lInfo.lFlags;

            memcpy(cmdExt+8, &lInfo.name, sizeof(lInfo.name));

            memcpy(cmdExt+8+sizeof(lInfo.name),
               &lInfo.user, sizeof(lInfo.user));

            res = 8 + sizeof(lInfo.name) + sizeof(lInfo.user);
            cmdP->size = res;
         }
         break;
         
      default:
         res = LG_UNKNOWN_COMMAND;
         break;
   }

   cmdP->status = res;

   return res;
}

