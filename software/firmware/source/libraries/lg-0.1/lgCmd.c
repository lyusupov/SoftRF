/* This is free and unencumbered software released into the public domain.

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

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
#include <inttypes.h>

#include "lgpio.h"
#include "rgpiod.h"

#include "lgCmd.h"

cmdInfo_t cmdInfo[]=
{
   /* num          str    vfyt retv script*/

   /* FILES */

   {LG_CMD_FO,    "FO",    102, 2, 0}, // lgFileOpen
   {LG_CMD_FC,    "FC",    101, 0, 1}, // lgFileClose
   {LG_CMD_FR,    "FR",    101, 6, 0}, // lgFileRead
   {LG_CMD_FW,    "FW",    103, 0, 0}, // lgFileWrite
   {LG_CMD_FS,    "FS",    101, 2, 1}, // lgFileSeek
   {LG_CMD_FL,    "FL",    102, 6, 0}, // lgFileList

   /* GPIO */

   {LG_CMD_GO,    "GO",    101, 2, 1}, // lgGpiochipOpen
   {LG_CMD_GC,    "GC",    101, 0, 1}, // lgGpiochipClose

   {LG_CMD_GIC,   "GIC",   101, 10, 0}, // lgGpioGetChipInfo
   {LG_CMD_GIL,   "GIL",   101, 11, 0}, // lgGpioGetLineInfo
   {LG_CMD_GMODE, "GMODE", 101,  2, 1}, // lgGpioGetMode

   {LG_CMD_GSI,   "GSI",   101, 0, 1}, // lgGpioClaimInput (simple)
   {LG_CMD_GSIX,  "GSIX",  101, 0, 1}, // lgGpioClaimInput
   {LG_CMD_GSO,   "GSO",   101, 0, 1}, // lgGpioClaimOutput (simple)
   {LG_CMD_GSOX,  "GSOX",  101, 0, 1}, // lgGpioClaimOutput
   {LG_CMD_GSA,   "GSA",   101, 0, 1}, // lgGpioClaimAlert (simple)
   {LG_CMD_GSAX,  "GSAX",  101, 0, 1}, // lgGpioClaimAlert
   {LG_CMD_GSF,   "GSF",   101, 0, 1}, // lgGpioFree

   {LG_CMD_GSGI,  "GSGI",  101, 0, 0}, // lgGroupClaimInput (simple)
   {LG_CMD_GSGIX, "GSGIX", 101, 0, 0}, // lgGroupClaimInput
   {LG_CMD_GSGO,  "GSGO",  101, 0, 0}, // lgGroupClaimOutput (simple)
   {LG_CMD_GSGOX, "GSGOX", 101, 0, 0}, // lgGroupClaimOutput
   {LG_CMD_GSGF,  "GSGF",  101, 0, 1}, // lgGroupFree

   {LG_CMD_GR,    "GR",    101, 2, 1}, // lgGpioRead
   {LG_CMD_GW,    "GW",    101, 0, 1}, // lgGpioWrite

   {LG_CMD_GGR,   "GGR",   101, 9, 1}, // lgGroupRead
   {LG_CMD_GGW,   "GGW",   101, 0, 1}, // lgGroupWrite (simple)
   {LG_CMD_GGWX,  "GGWX",  101, 0, 1}, // lgGroupWrite

   {LG_CMD_GP,    "GP",    101, 2, 1}, // lgTxPulse (simple)
   {LG_CMD_GPX,   "GPX",   101, 2, 1}, // lgTxPulse
   {LG_CMD_GWAVE, "GWAVE", 101, 2, 1}, // lgTxWave
   {LG_CMD_GBUSY, "GBUSY", 101, 2, 1}, // lgTxBusy
   {LG_CMD_GROOM, "GROOM", 101, 2, 1}, // lgTxRoom
   {LG_CMD_P,     "P",     101, 2, 1}, // lgTxPwm (simple)
   {LG_CMD_PX,    "PX",    101, 2, 1}, // lgTxPwm
   {LG_CMD_S,     "S",     101, 2, 1}, // lgTxServo (simple)
   {LG_CMD_SX,    "SX",    101, 2, 1}, // lgTxServo

   {LG_CMD_GDEB,  "GDEB",  101, 0, 1}, // lgGpioSetDebounce
   {LG_CMD_GWDOG, "GWDOG", 101, 0, 1}, // lgGpioSetWatchdog

   /* I2C */

   {LG_CMD_I2CO,  "I2CO",  101, 2, 1}, // lgI2cOpen
   {LG_CMD_I2CC,  "I2CC",  101, 0, 1}, // lgI2cClose

   {LG_CMD_I2CWQ, "I2CWQ", 101, 0, 1}, // lgI2cWriteQuick

   {LG_CMD_I2CRS, "I2CRS", 101, 2, 1}, // lgI2cReadByte
   {LG_CMD_I2CWS, "I2CWS", 101, 0, 1}, // lgI2cWriteByte

   {LG_CMD_I2CRB, "I2CRB", 101, 2, 1}, // lgI2cReadByteData
   {LG_CMD_I2CWB, "I2CWB", 101, 0, 1}, // lgI2cWriteByteData

   {LG_CMD_I2CRW, "I2CRW", 101, 2, 1}, // lgI2cReadWordData
   {LG_CMD_I2CWW, "I2CWW", 101, 0, 1}, // lgI2cWriteWordData

   {LG_CMD_I2CRK, "I2CRK", 101, 6, 0}, // lgI2cReadBlockData
   {LG_CMD_I2CWK, "I2CWK", 104, 0, 0}, // lgI2cWriteBlockData

   {LG_CMD_I2CRI, "I2CRI", 101, 6, 0}, // lgI2cReadI2CBlockData
   {LG_CMD_I2CWI, "I2CWI", 104, 0, 0}, // lgI2cWriteI2CBlockData

   {LG_CMD_I2CRD, "I2CRD", 101, 6, 0}, // lgI2cReadDevice
   {LG_CMD_I2CWD, "I2CWD", 103, 0, 0}, // lgI2cWriteDevice

   {LG_CMD_I2CPC, "I2CPC", 101, 2, 1}, // lgI2cProcessCall
   {LG_CMD_I2CPK, "I2CPK", 104, 6, 0}, // lgI2cBlockProcessCall

   {LG_CMD_I2CZ,  "I2CZ",  103, 6, 0}, // lgI2cZip

   /* NOTIFICATIONS */

   {LG_CMD_NO,    "NO",    100, 2, 1}, // lgNotifyOpen
   {LG_CMD_NC,    "NC",    101, 0, 1}, // lgNotifyClose
   {LG_CMD_NP,    "NP",    101, 0, 1}, // lgNotifyPause
   {LG_CMD_NR,    "NR",    101, 0, 1}, // lgNotifyResume

   /* SCRIPTS */

   {LG_CMD_PROC,  "PROC",  106, 2, 0}, // lgScriptStore
   {LG_CMD_PROCR, "PROCR", 107, 0, 0}, // lgScriptRun
   {LG_CMD_PROCU, "PROCU", 107, 0, 0}, // lgScriptUpdate
   {LG_CMD_PROCP, "PROCP", 101, 7, 0}, // lgScriptStatus
   {LG_CMD_PROCS, "PROCS", 101, 0, 0}, // lgScriptStop
   {LG_CMD_PROCD, "PROCD", 101, 0, 0}, // lgScriptDelete

   {LG_CMD_PARSE, "PARSE", 106, 0, 0}, // cmdParseScript

   /* SERIAL */

   {LG_CMD_SERO,  "SERO",  108, 2, 0}, // lgSerialOpen
   {LG_CMD_SERC,  "SERC",  101, 0, 1}, // lgSerialClose

   {LG_CMD_SERRB, "SERRB", 101, 2, 1}, // lgSerialReadByte
   {LG_CMD_SERWB, "SERWB", 101, 0, 1}, // lgSerialWriteByte

   {LG_CMD_SERR,  "SERR",  101, 6, 0}, // lgSerialRead
   {LG_CMD_SERW,  "SERW",  103, 0, 0}, // lgSerialWrite

   {LG_CMD_SERDA, "SERDA", 101, 2, 1}, // lgSerialDataAvailable

   /* SHELL */

   {LG_CMD_SHELL, "SHELL", 109, 2, 0}, // lgShell

   /* SPI */

   {LG_CMD_SPIO,  "SPIO",  101, 2, 0}, // lgSpiOpen
   {LG_CMD_SPIC,  "SPIC",  101, 0, 1}, // lgSpiClose

   {LG_CMD_SPIR,  "SPIR",  101, 6, 0}, // lgSpiRead
   {LG_CMD_SPIW,  "SPIW",  103, 0, 0}, // lgSpiWrite

   {LG_CMD_SPIX,  "SPIX",  103, 6, 0}, // lgSpiXfer

   /* UTILITIES */

   {LG_CMD_LGV,   "LGV",   100, 5, 1}, // lguVersion
   {LG_CMD_SBC,   "SBC",   100, 6, 1}, // lguSbcName

   {LG_CMD_CGI,   "CGI",   101, 3, 1}, // lguGetInternals
   {LG_CMD_CSI,   "CSI",   101, 1, 1}, // lguSetInternals

   {LG_CMD_TICK,  "T",     100, 3, 1}, // lguTimestamp
   {LG_CMD_TICK,  "TICK",  100, 3, 1}, // lguTimestamp

   {LG_CMD_MICS,  "MICS",  101, 0, 1}, // lguSleep
   {LG_CMD_MILS,  "MILS",  101, 0, 1}, // lguSleep

   {LG_CMD_USER,  "U",     105, 6, 0}, // xSetUser
   {LG_CMD_USER,  "USER",  105, 6, 0}, // xSetUser

   {LG_CMD_SHARE, "C",     101, 0, 1}, // xShareSetUse
   {LG_CMD_SHARE, "SHARE", 101, 0, 1}, // xShareSetUse

   {LG_CMD_LCFG,  "LCFG",  100, 0, 0}, // xLoadConfig

   {LG_CMD_PCD,   "PCD",   100, 6, 0}, // lguGetConfigDir
   {LG_CMD_PWD,   "PWD",   100, 6, 0}, // lguGetWorkDir

   {LG_CMD_SHRS,  "SHRS",  101, 0, 1}, // lgHdlSetShare
   {LG_CMD_SHRU,  "SHRU",  101, 0, 1}, // xShareUse


   {LG_CMD_PASSW, "PASSW", 105, 0, 0}, // xPassword


   // script commands

   {LG_CMD_ADD  , "ADD"  , 201, 0, 1},
   {LG_CMD_AND  , "AND"  , 201, 0, 1},
   {LG_CMD_CALL , "CALL" , 202, 0, 1},
   {LG_CMD_CMDR  ,"CMDR" , 201, 0, 1},
   {LG_CMD_CMDW , "CMDW" , 201, 0, 1},
   {LG_CMD_CMP  , "CMP"  , 201, 0, 1},
   {LG_CMD_DCR  , "DCR"  , 203, 0, 1},
   {LG_CMD_DCRA , "DCRA" , 200, 0, 1},
   {LG_CMD_DIV  , "DIV"  , 201, 0, 1},
   {LG_CMD_HALT , "HALT" , 200, 0, 1},
   {LG_CMD_INR  , "INR"  , 203, 0, 1},
   {LG_CMD_INRA , "INRA" , 200, 0, 1},
   {LG_CMD_JGE  , "JGE"  , 202, 0, 1},
   {LG_CMD_JGT  , "JGT"  , 202, 0, 1},
   {LG_CMD_JLE  , "JLE"  , 202, 0, 1},
   {LG_CMD_JLT  , "JLT"  , 202, 0, 1},
   {LG_CMD_JMP  , "JMP"  , 202, 0, 1},
   {LG_CMD_JNZ  , "JNZ"  , 202, 0, 1},
   {LG_CMD_JZ   , "JZ"   , 202, 0, 1},
   {LG_CMD_LD   , "LD"   , 204, 0, 1},
   {LG_CMD_LDA  , "LDA"  , 201, 0, 1},
   {LG_CMD_LDAB , "LDAB" , 201, 0, 1},
   {LG_CMD_MLT  , "MLT"  , 201, 0, 1},
   {LG_CMD_MOD  , "MOD"  , 201, 0, 1},
   {LG_CMD_NOP  , "NOP"  , 200, 0, 1},
   {LG_CMD_OR   , "OR"   , 201, 0, 1},
   {LG_CMD_POP  , "POP"  , 203, 0, 1},
   {LG_CMD_POPA , "POPA" , 200, 0, 1},
   {LG_CMD_PUSH , "PUSH" , 203, 0, 1},
   {LG_CMD_PUSHA, "PUSHA", 200, 0, 1},
   {LG_CMD_RET  , "RET"  , 200, 0, 1},
   {LG_CMD_RL   , "RL"   , 204, 0, 1},
   {LG_CMD_RLA  , "RLA"  , 201, 0, 1},
   {LG_CMD_RR   , "RR"   , 204, 0, 1},
   {LG_CMD_RRA  , "RRA"  , 201, 0, 1},
   {LG_CMD_SHL  , "SHL"  , 204, 0, 1},
   {LG_CMD_SHLA , "SHLA" , 201, 0, 1},
   {LG_CMD_SHR  , "SHR"  , 204, 0, 1},
   {LG_CMD_SHRA , "SHRA" , 201, 0, 1},
   {LG_CMD_STA  , "STA"  , 203, 0, 1},
   {LG_CMD_STAB , "STAB" , 201, 0, 1},
   {LG_CMD_SUB  , "SUB"  , 201, 0, 1},
   {LG_CMD_SYS  , "SYS"  , 206, 0, 1},
   {LG_CMD_TAG  , "TAG"  , 202, 0, 1},
   {LG_CMD_X    , "X"    , 205, 0, 1},
   {LG_CMD_XA   , "XA"   , 203, 0, 1},
   {LG_CMD_XOR  , "XOR"  , 201, 0, 1},

};

static int cmdMatch(char *str)
{
   int i;

   for (i=0; i<(sizeof(cmdInfo)/sizeof(cmdInfo_t)); i++)
   {
      if (strcasecmp(str, cmdInfo[i].name) == 0) return i;
   }
   return CMD_UNKNOWN_CMD;
}

static int getNum(
   char *str, uintmax_t *val, int8_t *opt, uintmax_t max, int real)
{
   int m, n;
   uintmax_t v;
   float f;

   *opt = 0;

   if (real)
   {
      m = sscanf(str, " %f %n", &f, &n);

      if (m == 1)
      {
         *val = (f * 1000.0) + 0.5;
         *opt = CMD_NUMERIC;
         if (*val > max) *opt = -CMD_NUMERIC;
         return n;
      }
   }

   m = sscanf(str, " %ji %n", &v, &n);

   if (m == 1)
   {
      if (real) v = v * 1000;
      *val = v;
      *opt = CMD_NUMERIC;
      if (v > max) *opt = -CMD_NUMERIC;
      return n;
   }

   m = sscanf(str, " v%ji %n", &v, &n);

   if (m == 1)
   {
      *val = v;
      if (v < LG_MAX_SCRIPT_VARS) *opt = CMD_VAR;
      else *opt = -CMD_VAR;
      return n;
   }

   m = sscanf(str, " p%ji %n", &v, &n);

   if (m == 1)
   {
      *val = v;
      if (v < LG_MAX_SCRIPT_PARAMS) *opt = CMD_PAR;
      else *opt = -CMD_PAR;
      return n;
   }

   return 0;
}

static char intCmdStr[32];
static int intCmdIdx;

char *cmdStr(void)
{
   return intCmdStr;
}

int cmdScanf(char *text, cmdCtl_p ctlP, lgCmd_p cmdP, char *fmt, int *matched)
{
   uint32_t *argI=(uint32_t*)&cmdP[1];
   uint64_t *argQ=(uint64_t*)&cmdP[1];
   uint8_t  *argB=(uint8_t*) &cmdP[1];

   int par = 0;
   char *at = fmt;
   uintmax_t var;
   int8_t opt;
   int i;
   int eaten;
   int valid = 1;
   int upto = 1;
   int offset = 0;

   *matched = 0;

   while (valid && *at)
   {
      switch (*at++)
      {
         case 'q':
            upto = 200;
         case 'Q':  // 64-bit arg
            for (i=0; i<upto; i++)
            {
               ctlP->eaten +=
                  getNum(text+ctlP->eaten, &var, &opt, 0xffffffffffffffff, 0);
               if (opt > 0)
               {
                  argQ[offset/8] = var;
                  if (par<CMD_MAX_OPT) ctlP->opt[par] = opt;
                  offset+=8;
                  ++par;
                  (*matched)++;
               }
               else
               {
                  valid = 0;
                  break;
               }
            }
            upto = 1;
            break;

         case 'f':
            upto = 200;
         case 'F':  // 32-bit arg (float)
            for (i=0; i<upto; i++)
            {
               ctlP->eaten +=
                  getNum(text+ctlP->eaten, &var, &opt, 0xffffffff, 1);
               if (opt > 0)
               {
                  argI[offset/4] = var;
                  if (par<CMD_MAX_OPT) ctlP->opt[par] = opt;
                  offset+=4;
                  ++par;
                  (*matched)++;
               }
               else
               {
                  valid = 0;
                  break;
               }
            }
            upto = 1;
            break;

         case 'i':
            upto = 200;
         case 'I':  // 32-bit arg
            for (i=0; i<upto; i++)
            {
               ctlP->eaten +=
                  getNum(text+ctlP->eaten, &var, &opt, 0xffffffff, 0);
               if (opt > 0)
               {
                  argI[offset/4] = var;
                  if (par<CMD_MAX_OPT) ctlP->opt[par] = opt;
                  offset+=4;
                  ++par;
                  (*matched)++;
               }
               else
               {
                  valid = 0;
                  break;
               }
            }
            upto = 1;
            break;

         case 'b':
            upto = 200;
         case 'B':  // 8-bit ext
            for (i=0; i<upto; i++)
            {
               eaten = getNum(text+ctlP->eaten, &var, &opt, 0xff, 0);
               if (opt == CMD_NUMERIC)
               {
                  if (((int)var>=0) && ((int)var<=255))
                  {
                     argB[offset] = var;
                     ++offset;
                     ++par;
                     (*matched)++;
                     ctlP->eaten += eaten;
                  }
                  else
                  {
                     valid = 0;
                     break;
                  }
               }
               else
               {
                  valid = 0;
                  break;
               }
            }
            upto = 1;
            break;

         case '*':
            upto = 200;
            break;

         case '<':
            offset -= 4; /* retreat 4 bytes */
            break;

         case '>':
            offset += 4; /* advance 4 bytes */
            break;

         default:
            valid = 0;
            break;
      }
   }

   return valid;
}

int cmdParse(char *text, cmdCtl_p ctlP, lgCmd_p cmdP, int cmdBufSize)
{
   int m, valid, idx, pp, pars, n, n2;
   int matches;
   uint32_t *arg=(uint32_t*)&cmdP[1];
   char *ext=(char*)&cmdP[1];
   uintmax_t var;
   uint32_t tI[10];
   int8_t tB[10];

   bzero(&ctlP->opt, sizeof(ctlP->opt));

   sscanf(text+ctlP->eaten, " %31s %n", intCmdStr, &pp);

   ctlP->eaten += pp;

   cmdP->cmd = -1;

   idx = cmdMatch(intCmdStr);

   intCmdIdx = idx;

   if (idx < 0) return idx;

   valid = 0;

   cmdP->cmd = cmdInfo[idx].cmd;
   cmdP->size = 0;

   switch (cmdInfo[idx].vt)
   {
      case 100: /*
                   No parameters, always valid.
                */
         pars = 0;
         valid = 1;

         break;

      case 101: /*
                */

         switch (cmdP->cmd)
         {
            case LG_CMD_CGI:   // id
            case LG_CMD_FC:    // h
            case LG_CMD_I2CC:  // h
            case LG_CMD_I2CRS: // h
            case LG_CMD_MICS:  // v
            case LG_CMD_MILS:  // v
            case LG_CMD_NR:    // h
            case LG_CMD_NC:    // h
            case LG_CMD_NP:    // h
            case LG_CMD_PROCD: // h
            case LG_CMD_PROCP: // h
            case LG_CMD_PROCS: // h
            case LG_CMD_SERC:  // h
            case LG_CMD_SERDA: // h
            case LG_CMD_SERRB: // h
            case LG_CMD_SHARE: // v
            case LG_CMD_SHRU:  // v
            case LG_CMD_SPIC:  // h
            case LG_CMD_GC:    // h
            case LG_CMD_GIC:   // h
            case LG_CMD_GO:    // gc
               pars = 1;
               valid = cmdScanf(text, ctlP, cmdP, "I", &matches);
               break;

            case LG_CMD_CSI: // id valQ --> valQ id
               pars = 3; /* Q counts as 2 */
               valid = cmdScanf(text, ctlP, cmdP, ">>I<<<Q", &matches);
               break;

            case LG_CMD_GWAVE: // h g (vQ mQ dQ)* --> (vQ mQ dQ)* h g

               ctlP->eaten +=
                  getNum(text+ctlP->eaten, &var, &tB[0], 0xffffffff, 0);
               tI[0] = var;

               ctlP->eaten +=
                  getNum(text+ctlP->eaten, &var, &tB[1], 0xffffffff, 0);
               tI[1] = var;

               cmdScanf(text, ctlP, cmdP, "*Q", &matches);

               if ((tB[0] > 0) && (tB[1] > 0) &&
                   matches && ((matches % 3) == 0))
               {
                  pars = 2 + (matches * 2);
                  arg[matches*2] = tI[0];
                  arg[(matches*2)+1] = tI[1];
                  valid = 1;
               }

               break;

            case LG_CMD_FR:    // h v
            case LG_CMD_GGR:   // h g
            case LG_CMD_GIL:   // h g
            case LG_CMD_GMODE: // h g
            case LG_CMD_GR:    // h g
            case LG_CMD_GSI:   // h g
            case LG_CMD_GSO:   // h g
            case LG_CMD_GSF:   // h g
            case LG_CMD_GSGF:  // h g
            case LG_CMD_I2CRB: // h b
            case LG_CMD_I2CRD: // 
            case LG_CMD_I2CRK:
            case LG_CMD_I2CRW:
            case LG_CMD_I2CWQ:
            case LG_CMD_I2CWS:
            case LG_CMD_SERR:
            case LG_CMD_SERWB:
            case LG_CMD_SHRS:  // h v
            case LG_CMD_SPIR:
               pars = 2;
               valid = cmdScanf(text, ctlP, cmdP, "II", &matches);
               break;
                              
            case LG_CMD_GDEB:  // h g v
            case LG_CMD_GROOM: // h g t
            case LG_CMD_GBUSY: // h g t
            case LG_CMD_GSA:   // h g nfyh
            case LG_CMD_GSIX:  // h lf g
            case LG_CMD_GW:    // h g v
            case LG_CMD_GWDOG: // h g v
            case LG_CMD_FS:
            case LG_CMD_I2CO:
            case LG_CMD_I2CPC:
            case LG_CMD_I2CRI:
            case LG_CMD_I2CWB:
            case LG_CMD_I2CWW:
            case LG_CMD_S:    // h g width
               pars = 3;
               valid = cmdScanf(text, ctlP, cmdP, "III", &matches);
               break;
                              
            case LG_CMD_P:    // h g freq dutyc
               pars = 4;
               valid = cmdScanf(text, ctlP, cmdP, "IIFF", &matches);
               break;
                              
            case LG_CMD_GP:   // h g m_on m_off
            case LG_CMD_GSOX: // h lf g v
            case LG_CMD_SPIO:
               pars = 4;
               valid = cmdScanf(text, ctlP, cmdP, "IIII", &matches);
               break;
                              
            case LG_CMD_GGW: // h g bitsQ -> bitsQ h g
               pars = 4; /* Q counts as 2 */
               valid = cmdScanf(text, ctlP, cmdP, ">>II<<<<Q", &matches);
               break;
                              
            case LG_CMD_GSAX: // h lf ef g nfyh
               pars = 5;
               valid = cmdScanf(text, ctlP, cmdP, "IIIII", &matches);
               break;
                              
            case LG_CMD_GGWX: // h g bitsQ maskQ -> bitsQ maskQ h g
               pars = 6; /* Q counts as 2 */
               valid = cmdScanf(text, ctlP, cmdP, ">>>>II<<<<<<QQ", &matches);
               break;
                              
            case LG_CMD_PX:  // h g freq dutyc off cyc
               pars = 6;
               valid = cmdScanf(text, ctlP, cmdP, "IIFFII", &matches);
               break;
                              
            case LG_CMD_GPX: // h g m_on m_off off cyc
            case LG_CMD_SX:  // h g freq width off cyc
               pars = 6;
               valid = cmdScanf(text, ctlP, cmdP, "IIIIII", &matches);
               break;
                              
            case LG_CMD_GSGO: // h g*
            case LG_CMD_GSGI: // h g*
               valid = cmdScanf(text, ctlP, cmdP, "i", &matches);
               pars = matches;
               if (pars > 1) valid = 1;
               break;

            case LG_CMD_GSGIX: // h lf g*
               valid = cmdScanf(text, ctlP, cmdP, "i", &matches);
               pars = matches;
               if (pars > 2) valid = 1;
               break;

            case LG_CMD_GSGOX: // h lf g* v*
               valid = cmdScanf(text, ctlP, cmdP, "i", &matches);
               pars = matches;
               if ((pars > 3) && ((pars % 2) == 0)) valid = 1;
               break;
         }

         if (valid) cmdP->size = pars * 4;

         break;

      case 102: /* 
                   FL FO
                   Two parameters, first a string, other positive.
                */
         m = sscanf(text+ctlP->eaten, " %*s%n %n", &n, &n2);
         if ((m >= 0) && n)
         {
            cmdP->size = n+4;
            ctlP->opt[1] = CMD_NUMERIC;
            memcpy(ext+4, text+ctlP->eaten, n);
            ctlP->eaten += n2;

            ctlP->eaten +=
               getNum(text+ctlP->eaten, &var, &ctlP->opt[0], 0xffffffff, 0);
            arg[0] = var;

            if ((ctlP->opt[0] > 0) && ((int)arg[0] >= 0))
            {
               valid = 1;
            }
         }

         break;

      case 103: /* BI2CZ  BSPIX  FW  I2CWD  I2CZ  SERW SPIW  SPIX

                   Two or more parameters, first >=0, rest 0-255.
                */

         valid = cmdScanf(text, ctlP, cmdP, "I>b", &matches);
         pars = matches;

         if (pars > 1)
         {
            valid = 1;
            cmdP->size = 4 + (pars-1);
         }

         break;

      case 104: /* I2CPK  I2CWI  I2CWK

                   Three to 34 parameters, all 0-255.
                */

         valid = cmdScanf(text, ctlP, cmdP, "II>>b", &matches);
         pars = matches;

         if ((pars > 2) && (pars < 35))
         {
            valid = 1;
            cmdP->size = 8 + (pars-2);
         }

         break;

      case 105: /* 
                   PASSW USER
                   One parameter, a string.
                */
         m = sscanf(text+ctlP->eaten, " %*s%n %n", &n, &n2);
         if ((m >= 0) && n)
         {
            cmdP->size = n;
            memcpy(ext, text+ctlP->eaten, n);
            ctlP->eaten += n2;
            valid = 1;
         }

         break;

      case 106: /* 
                   PARSE PROC
                   One parameter, string (rest of input).
                */
         cmdP->size = strlen(text+ctlP->eaten);
         memcpy(ext, text+ctlP->eaten, cmdP->size);
         ctlP->eaten += cmdP->size;
         valid = 1;

         break;

      case 107: /* 
                   One to 11 parameters, first positive,
                   optional remainder, any value.
                */

         valid = cmdScanf(text, ctlP, cmdP, "i", &matches);
         pars = matches;

         if ((pars > 0) && (pars <= (LG_MAX_SCRIPT_PARAMS+1)))
         {
            valid = 1;
            cmdP->size = pars * 4;
         }

         break;

      case 108: /* 
                   SERO
                   Three parameters, first a string, rest >=0
                */
         m = sscanf(text+ctlP->eaten, " %*s%n %n", &n, &n2);
         if ((m >= 0) && n)
         {
            cmdP->size = n+8;
            ctlP->opt[1] = CMD_NUMERIC;
            memcpy(ext+8, text+ctlP->eaten, n);
            ctlP->eaten += n2;

            ctlP->eaten +=
               getNum(text+ctlP->eaten, &var, &ctlP->opt[0], 0xffffffff, 0);
            arg[0] = var;

            ctlP->eaten +=
               getNum(text+ctlP->eaten, &var, &ctlP->opt[1], 0xffffffff, 0);
            arg[1] = var;

            if ((ctlP->opt[0] > 0) && ((int)arg[0] >= 0) &&
                (ctlP->opt[1] > 0) && ((int)arg[1] >= 0))
               valid = 1;
         }

         break;

      case 109: /* 
                   SHELL
                   Two string parameters, the first space teminated.
                   The second arbitrary.
                */
         m = sscanf(text+ctlP->eaten, " %*s%n %n", &n, &n2);

         if ((m >= 0) && n)
         {
            valid = 1;

            arg[0] = n+1; /* length includes terminating 0 */

            memcpy(ext+4, text+ctlP->eaten, n);
            ext[4+n] = 0; /* terminate first string */

            ctlP->eaten += n;

            n2 = strlen(text+ctlP->eaten+1);

            memcpy(ext+n+5, text+ctlP->eaten+1, n2);
            ext[4+n+n2+1] = 0; /* terminate second string */

            ctlP->eaten += n2;
            ctlP->eaten ++;
            cmdP->size = 4+n+n2+2;
         }

         break;


      case 200: /*
                   Script. No parameters, always valid.
                */
         valid = 1;

         break;

      case 201: /* 
                   One parameter, any value.
                */
         ctlP->eaten +=
            getNum(text+ctlP->eaten, &var, &ctlP->opt[0], 0xffffffff, 0);
         arg[0] = var;

         if (ctlP->opt[0] > 0) valid = 1;

         break;

      case 202: /* 
                   One numeric parameter, any value.
                */
         ctlP->eaten +=
            getNum(text+ctlP->eaten, &var, &ctlP->opt[0], 0xffffffff, 0);
         arg[0] = var;
         if (ctlP->opt[0] == CMD_NUMERIC) valid = 1;

         break;

      case 203: /* 
                   One register parameter.
                */
         ctlP->eaten +=
            getNum(text+ctlP->eaten, &var, &ctlP->opt[0], 0xffffffff, 0);
         arg[0] = var;

         if ((ctlP->opt[0] > 0) && (arg[0] < LG_MAX_SCRIPT_VARS)) valid = 1;

         break;

      case 204: /* 
                   Two parameters, first register, second any value.
                */
         ctlP->eaten +=
            getNum(text+ctlP->eaten, &var, &ctlP->opt[0], 0xffffffff, 0);
         arg[0] = var;

         ctlP->eaten +=
            getNum(text+ctlP->eaten, &var, &ctlP->opt[1], 0xffffffff, 0);
         arg[1] = var;

         if ((ctlP->opt[0] > 0) &&
             (arg[0] < LG_MAX_SCRIPT_VARS) &&
             (ctlP->opt[1] > 0)) valid = 1;

         break;

      case 205: /* 
                   Two register parameters.
                */
         ctlP->eaten +=
            getNum(text+ctlP->eaten, &var, &ctlP->opt[0], 0xffffffff, 0);
         arg[0] = var;

         ctlP->eaten +=
            getNum(text+ctlP->eaten, &var, &ctlP->opt[1], 0xffffffff, 0);
         arg[1] = var;

         if ((ctlP->opt[0] > 0) && (arg[0] < LG_MAX_SCRIPT_VARS) &&
             (ctlP->opt[1] > 0) && (arg[1] < LG_MAX_SCRIPT_VARS)) valid = 1;

         break;

      case 206: /* 
                   One parameter, a string.
                */
         m = sscanf(text+ctlP->eaten, " %*s%n %n", &n, &n2);
         if ((m >= 0) && n)
         {
            cmdP->size = n;
            ctlP->opt[3] = CMD_NUMERIC;
            memcpy(ext, text+ctlP->eaten, n);
            ctlP->eaten += n2;
            valid = 1;
         }

         break;
   }

   if (valid) return idx; else return CMD_BAD_PARAMETER;
}

int cmdParseScript(char *script, cmdScript_t *s, int diags)
{
   int idx, len, b, i, j, tags, resolved;
   int status;
   cmdInstr_t instr;
   cmdCtl_t ctlP;
   lgCmd_t cmdBuf[CMD_MAX_EXTENSION/sizeof(lgCmd_t)];
   lgCmd_p cmdP=cmdBuf;
   uint32_t *arg=(uint32_t*)&cmdP[1];

   ctlP.inScript = 1;
   ctlP.eaten = 0;

   status = 0;

   cmdTagStep_t tag_step[LG_MAX_SCRIPT_TAGS];

   len = strlen(script);

   /* calloc space for PARAMS, VARS, CMDS, and STRINGS */

   b = (sizeof(int) * (LG_MAX_SCRIPT_PARAMS + LG_MAX_SCRIPT_VARS)) +
       (sizeof(cmdInstr_t) * (len + 2) / 2) + len;

   s->par = calloc(1, b);

   if (s->par == NULL) return -1;

   s->var = s->par + LG_MAX_SCRIPT_PARAMS;

   s->instr = (cmdInstr_t *)(s->var + LG_MAX_SCRIPT_VARS);

   s->str_area = (char *)(s->instr + ((len + 2) / 2));

   s->str_area_len = len;
   s->str_area_pos = 0;

   s->instrs = 0;

   tags = 0;

   idx = 0;

   while (ctlP.eaten<len)
   {
      idx = cmdParse(script, &ctlP, cmdBuf, sizeof(cmdBuf));

      /* abort if command is illegal in a script */

      if ((idx >= 0) || (idx != CMD_UNKNOWN_CMD))
      {
         if (!cmdInfo[intCmdIdx].cvis) idx = CMD_NOT_IN_SCRIPT;
      }

      if (idx >= 0)
      {
         fprintf(stderr, "cmd=%d size=%d a0=%d a1=%d a2=%d a3=%d\n",
            cmdP->cmd, cmdP->size, arg[0], arg[1], arg[2], arg[3]);
         if (cmdP->size)
         {
            //memcpy(s->str_area + s->str_area_pos, v, cmdP->size);
            //s->str_area[s->str_area_pos + cmdP->size] = 0;
            //p[4] = (intptr_t) s->str_area + s->str_area_pos;
            //s->str_area_pos += (cmdP->size + 1);
         }


         instr.cmd = cmdP->cmd;
         memcpy(&instr.arg[0], &arg[0], sizeof(instr.arg));

         if (cmdP->cmd == LG_CMD_TAG)
         {
            if (tags < LG_MAX_SCRIPT_TAGS)
            {
               /* check tag not already used */
               for (j=0; j<tags; j++)
               {
                  if (tag_step[j].tag == instr.arg[0])
                  {
                     if (diags)
                     {
                        fprintf(stderr,
                           "Duplicate tag: %"PRId32"\n", instr.arg[0]);
                     }

                     if (!status) status = LG_DUP_TAG;
                     idx = -1;
                  }
               }

               tag_step[tags].tag = instr.arg[0];
               tag_step[tags].step = s->instrs;
               tags++;
            }
            else
            {
               if (diags)
               {
                  fprintf(stderr, "Too many tags: %"PRId32"\n", instr.arg[0]);
               }
               if (!status) status = LG_TOO_MANY_TAGS;
               idx = -1;
            }
         }
      }
      else
      {
         if (diags)
         {
            if (idx == CMD_UNKNOWN_CMD)
               fprintf(stderr, "Unknown command: %s\n", cmdStr());
            else if (idx == CMD_NOT_IN_SCRIPT)
               fprintf(stderr, "Command illegal in script: %s\n", cmdStr());
            else
               fprintf(stderr, "Bad parameter to %s\n", cmdStr());
         }
         if (!status) status = LG_BAD_SCRIPT_CMD;
      }

      if (idx >= 0)
      {
         if (cmdP->cmd != LG_CMD_TAG)
         {
            memcpy(instr.opt, &ctlP.opt, sizeof(instr.opt));
            s->instr[s->instrs++] = instr;
         }
      }
   }

   for (i=0; i<s->instrs; i++)
   {
      instr = s->instr[i];

      /* resolve jumps */

      if ((instr.cmd == LG_CMD_JMP) || (instr.cmd == LG_CMD_CALL) ||
          (instr.cmd == LG_CMD_JZ)  || (instr.cmd == LG_CMD_JNZ)  ||
          (instr.cmd == LG_CMD_JGE) || (instr.cmd == LG_CMD_JGT)  ||
          (instr.cmd == LG_CMD_JLE) || (instr.cmd == LG_CMD_JLT))
      {
         resolved = 0;

         for (j=0; j<tags; j++)
         {
            if (instr.arg[0] == tag_step[j].tag)
            {
               s->instr[i].arg[0] = tag_step[j].step;
               resolved = 1;
               break;
            }
         }

         if (!resolved)
         {
            if (diags)
            {
               fprintf(stderr, "Can't resolve tag %"PRId32"\n", instr.arg[0]);
            }
            if (!status) status = LG_BAD_TAG;
         }
      }
   }
   return status;
}

