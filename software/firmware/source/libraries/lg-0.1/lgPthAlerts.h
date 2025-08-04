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

#ifndef LG_PTH_ALERTS_H
#define LG_PTH_ALERTS_H

#include <pthread.h>

#include "lgpio.h"
#include "lgGpio.h"

typedef struct lgAlertRec_s
{
   uint64_t last_rpt_ts;
   uint64_t last_evt_ts;
   uint64_t debounce_nanos;
   uint64_t watchdog_nanos;
   int last_evt_lv;
   int last_rpt_lv;
   int debounced;
   int watchdogd;
   int eFlags;
   int gpio;
   int nfyHandle;
   lgLineInf_p state;
   int active;
   lgChipObj_p chip;
   struct lgAlertRec_s *prev;
   struct lgAlertRec_s *next;
} lgAlertRec_t, *lgAlertRec_p;

lgAlertRec_p lgGpioGetAlertRec(lgChipObj_p chip, int gpio);

lgAlertRec_p lgGpioCreateAlertRec(
   lgChipObj_p chip, int gpio, lgLineInf_p state, int nfyHandle);

void *lgPthAlert(void);
void lgPthAlertStart(void);
void lgPthAlertStop(lgChipObj_p chip);

#endif

