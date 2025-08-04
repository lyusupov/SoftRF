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

#ifndef LG_DBG_H
#define LG_DBG_H

#include <stdio.h>
#include <stdint.h>

extern uint64_t lgDbgLevel;
extern int lgMinTxDelay;

/* Debug constants
*/

#define LG_DEBUG_MIN_LEVEL   (1<<0)
#define LG_DEBUG_ALWAYS      (1<<0)
#define LG_DEBUG_TRACE       (1<<1)
#define LG_DEBUG_USER        (1<<2)
#define LG_DEBUG_SCRIPT      (1<<3)
#define LG_DEBUG_STARTUP     (1<<4)
#define LG_DEBUG_GPIO        (1<<5)
#define LG_DEBUG_ALLOC       (1<<6)
#define LG_DEBUG_FILE        (1<<7)
#define LG_DEBUG_FAST_TICK   (1<<8)
#define LG_DEBUG_FREQUENT    (1<<9)
#define LG_DEBUG_INTERNAL    (1<<10)
#define LG_DEBUG_INIT        (1<<11)
#define LG_DEBUG_MAX_LEVEL   (1<<12) 

#define LG_DBG(mask, format, arg...)                               \
   do                                                              \
   {                                                               \
      if (lgDbgLevel & mask)                                       \
         fprintf(stderr, "%s %s: " format "\n" ,                   \
            lgDbgTimeStamp(), __FUNCTION__ , ## arg);              \
   }                                                               \
   while (0)

#define PARAM_ERROR(x, format, arg...)                             \
   do                                                              \
   {                                                               \
      LG_DBG(LG_DEBUG_USER, format, ## arg);                       \
      return x;                                                    \
   }                                                               \
   while (0)

#define ALLOC_ERROR(x, format, arg...)                             \
   do                                                              \
   {                                                               \
      LG_DBG(LG_DEBUG_ALWAYS, format, ## arg);                     \
      return x;                                                    \
   }                                                               \
   while (0)

char *lgDbgBuf2Str(int count, const char *buf);
char *lgDbgInt2Str(int count, const int *buf);
char *lgDbgStr2Hex(int count, const char *buf);
char *lgDbgTimeStamp(void);

#endif

