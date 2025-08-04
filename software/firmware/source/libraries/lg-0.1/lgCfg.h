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

#ifndef LG_CFG_H
#define LG_CFG_H

#include <stdio.h>

#define CFG_OKAY 0
#define CFG_BAD_FILE -1
#define CFG_MISSING_VALUE -2

typedef struct lgCfg_s lgCfg_t, *lgCfg_p;

lgCfg_p lgCfgRead(char *file);

char    *lgCfgGetValue(lgCfg_p cfg, char *section, char *key);

void    lgCfgFree(lgCfg_p cfg);

void    lgCfgPrint (lgCfg_p cfg, FILE *stream);

/* this function alters str, make a copy if you need it again */
char *lgCfgStrip(char *str);

/* this function alters str, make a copy if you need it again */
char *lgCfgNextToken(char **str, const char *delim, char **pos);

#endif

