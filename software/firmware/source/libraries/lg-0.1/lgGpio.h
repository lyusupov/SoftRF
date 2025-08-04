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

#ifndef _LG_GPIO_H
#define _LG_GPIO_H

#include "lgpio.h"

typedef struct lgLineInf_s
{
   int      banned;
   int      mode;
   int      lFlags;
   int      eFlags;
   int      group_size;
   int      fd;
   int      debounce_us;
   int      watchdog_us;
   callbk_t alertFunc;
   void     *userdata;
   uint32_t offset;
   uint32_t *offsets;
   uint8_t  *values;
} lgLineInf_t, *lgLineInf_p;

typedef struct lgChipObj_s
{
   int gpiochip;
   int handle; /* needed for auto resource free */
   uint32_t lines;
   int fd;
   lgLineInf_p LineInf;
   char name[LG_GPIO_NAME_LEN];
   char label[LG_GPIO_LABEL_LEN];
   char userLabel[LG_GPIO_USER_LEN];
} lgChipObj_t, *lgChipObj_p;

void xWrite(lgChipObj_p chip, int gpio, int value);
void xGroupWrite(
   lgChipObj_p chip, int gpio, uint64_t groupBits, uint64_t groupMask);

extern callbk_t lgGpioSamplesFunc;
extern void *lgGpioSamplesUserdata;

#endif

