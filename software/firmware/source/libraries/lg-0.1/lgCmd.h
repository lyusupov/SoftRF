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

#ifndef LG_CMD_H
#define LG_CMD_H

#include <stdio.h>
#include <string.h>

#include "lgpio.h"

#define CMD_MAX_PARAM 512
#define CMD_MAX_EXTENSION (1<<16)

#define CMD_UNKNOWN_CMD   -1
#define CMD_BAD_PARAMETER -2
#define CMD_EXT_TOO_SMALL -3
#define CMD_NOT_IN_SCRIPT -4

#define CMD_MAX_ARG 8
#define CMD_MAX_OPT 8

#define CMD_NUMERIC 1
#define CMD_VAR     2
#define CMD_PAR     3

#define LG_MAX_SCRIPT_TAGS   50
#define LG_MAX_SCRIPT_VARS  150
#define LG_MAX_SCRIPT_PARAMS 10

#define LG_MAGIC 0x6c67646d /* ASCII lgdm */

typedef struct
{
   union
   {
      uint32_t magic;
      int32_t status;
   };
   uint32_t size;
   uint16_t cmd;
   uint16_t doubles;
   uint16_t longs;
   uint16_t shorts;
} lgCmd_t, *lgCmd_p;

typedef struct
{
   int    eaten;
   int    inScript;
   int8_t opt[CMD_MAX_OPT];
} cmdCtl_t, *cmdCtl_p;

typedef struct
{
   int   cmd;  /* command number            */
   char *name; /* command name              */
   int   vt;   /* command verification type */
   int   rv;   /* command return value type */
   int   cvis; /* command valid in a script */
} cmdInfo_t;

typedef struct
{
   uint32_t tag;
   int      step;
} cmdTagStep_t;

typedef struct
{
   uint32_t arg[CMD_MAX_ARG];
   uint16_t cmd;
   int8_t opt[CMD_MAX_OPT];
} cmdInstr_t;

typedef struct
{
   /*
     +-----------+---------+---------+----------------+
     | PARAMS... | VARS... | CMDS... | STRING AREA... |
     +-----------+---------+---------+----------------+
   */
   int *par;
   int *var;
   cmdInstr_t *instr;
   int instrs;
   char *str_area;
   int str_area_len;
   int str_area_pos;
} cmdScript_t;

extern cmdInfo_t cmdInfo[];

extern char *cmdUsage;

int cmdParse(char *text, cmdCtl_p ctlP, lgCmd_p cmdP, int cmdBufSize);

int cmdParseScript(char *script, cmdScript_t *s, int diags);

char *cmdStr(void);

#endif

