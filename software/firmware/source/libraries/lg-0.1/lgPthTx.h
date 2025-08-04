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

#ifndef LG_PTH_PWM_H
#define LG_PTH_PWM_H

#include <pthread.h>

#include "lgpio.h"
#include "lgGpio.h"

#define LG_TX_BUF 10

typedef struct lgTxRec_s
{
   int active;
   struct lgTxRec_s *prev;
   struct lgTxRec_s *next;
   int next_micros;
   lgChipObj_p chip;
   int gpio;
   int entries; /* number of entries in LG_TX_BUF arrays */
   int type;    /* PWM or WAVE */
   union
   {
      struct
      {
         int micros_on[LG_TX_BUF];
         int micros_off[LG_TX_BUF];
         int cycles[LG_TX_BUF];
         int micros_offset; // start offset micros into cycle
         int next_level;
      };
      struct
      {
         lgPulse_p pulses[LG_TX_BUF];
         int num_pulses[LG_TX_BUF];
         int pulse_pos;
      };
   };
} lgTxRec_t, *lgTxRec_p;

lgTxRec_p lgGpioGetTxRec(lgChipObj_p chip, int gpio, int type);

lgTxRec_p lgGpioCreateTxRec(
   lgChipObj_p chip,
   int gpio,
   int micros_on,
   int micros_off,
   int micros_offset,
   int cycles);

lgTxRec_p lgGroupCreateWaveRec(
   lgChipObj_p chip, int gpio, int count, lgPulse_p pulses);

void lgPthTxStart(void);
void lgPthTxStop(lgChipObj_p chip);
void lgPthTxLock(void);
void lgPthTxUnlock(void);

#endif

