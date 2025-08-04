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

#include "lgDbg.h"
#include "lgHdl.h"
#include "lgPthTx.h"

int lgMinTxDelay = 10;

static pthread_t pthTx;
static pthread_mutex_t lgTxMutex = PTHREAD_MUTEX_INITIALIZER;
static volatile lgTxRec_p txRec = NULL;
static int pthTxRunning = LG_THREAD_NONE;
static struct timespec pthTxReq;
static int pthTxDelayMicros = 0;

void *lgPthTx(void)
{
   lgTxRec_p p, t;
   int i;

   clock_gettime(CLOCK_MONOTONIC, &pthTxReq); // get current time

   while (1)
   {
      lgPthTxLock();

      // update times

      for (p=txRec; p!=NULL; p=p->next)
         p->next_micros -= pthTxDelayMicros;

      // output changed edges and calculate next delay

      pthTxDelayMicros = 20000; // run at least fifty times a second

      p = txRec;
      
      while (p != NULL)
      {
         if (p->active)
         {
            if (p->next_micros <= 0)
            {
               if (p->type == LG_TX_PWM)
               {
                  if (p->next_level || (p->micros_on[0] == 0))
                  {
                      /* start of cycle */

                     if ((p->cycles[0] <= 0) && (p->entries > 1))
                     {
                        for (i=0; i<p->entries; i++)
                        {
                           p->micros_on[i] = p->micros_on[i+1];
                           p->micros_off[i] = p->micros_off[i+1];
                           p->cycles[i] = p->cycles[i+1];
                        }
                        --p->entries;
                     }
                  
                     if (p->cycles[0] == 0) /* 0 is a result of countdown */
                     {
                        xWrite(p->chip, p->gpio, 0);
                        p->active = 0;
                     }
                     else if (p->micros_on[0])
                     {
                        xWrite(p->chip, p->gpio, 1);
                        p->next_micros += p->micros_on[0];
                        if (p->micros_off[0]) p->next_level = 0;
                     }
                     else
                     {
                        xWrite(p->chip, p->gpio, 0);
                        p->next_micros += p->micros_off[0];
                        p->next_level = 1;
                     }

                     if (--p->cycles[0] < 0) p->cycles[0] = -1;
                  }
                  else /* middle of cycle */
                  {
                     xWrite(p->chip, p->gpio, 0);
                     p->next_micros += p->micros_off[0];
                     p->next_level = 1;
                  }
               }
               else if (p->type == LG_TX_WAVE)
               {
                  if (p->pulse_pos >= p->num_pulses[0])
                  {
                     if (p->entries > 1)
                     {
                        for (i=0; i<p->entries; i++)
                        {
                           p->pulses[i] = p->pulses[i+1];
                           p->num_pulses[i] = p->num_pulses[i+1];
                        }
                        --p->entries;
                        p->pulse_pos = 0;
                     }
                  }

                  if (p->pulse_pos < p->num_pulses[0])
                  {
                     xGroupWrite(p->chip, p->gpio,
                        p->pulses[0][p->pulse_pos].bits,
                        p->pulses[0][p->pulse_pos].mask);
                     p->next_micros += p->pulses[0][p->pulse_pos].delay;
                     (p->pulse_pos)++;
                  }
                  else p->active = 0;
               }
            }

            if (p->next_micros < pthTxDelayMicros)
               pthTxDelayMicros = p->next_micros;
         }
         else
         {
            /* delete inactive record */
            
            if (p->prev) p->prev->next = p->next;
            else txRec = p->next;

            if (p->next) p->next->prev = p->prev;

            if (p->type == LG_TX_WAVE)
            {
               /* free the malloc'd pulses */
               for (i=0; i<p->entries; i++)
               {
                  free(p->pulses[i]);
                  p->pulses[i] = NULL;
               }
            }

            t = p; p = p->prev; free(t);
         }

         if (p) p = p->next;
      }

      pthTxReq.tv_nsec += (pthTxDelayMicros * 1000);
      if (pthTxReq.tv_nsec >= 1000000000)
      {
         pthTxReq.tv_sec += 1;
         pthTxReq.tv_nsec -= 1000000000;
      }

      lgPthTxUnlock();

      // sleep until next edge

      while (clock_nanosleep(
         CLOCK_MONOTONIC, TIMER_ABSTIME, &pthTxReq, NULL));
   }

   pthTxRunning = LG_THREAD_NONE;
   pthread_exit(NULL);
}

void lgPthTxStart(void)
{
   if (!pthTxRunning)
   {
      if (pthread_create(&pthTx, NULL, (void*)lgPthTx, NULL) == 0)
      {
         pthread_detach(pthTx);
         pthTxRunning = LG_THREAD_STARTED;
      }
   }
}

void lgPthTxLock(void)
{
   pthread_mutex_lock(&lgTxMutex);
}

void lgPthTxUnlock(void)
{
   pthread_mutex_unlock(&lgTxMutex);
}

void lgPthTxStop(lgChipObj_p chip)
{
   lgTxRec_p pwm;

   /* stop any PWM on chip */
   
   for (pwm=txRec; pwm!=NULL; pwm=pwm->next)
   {
      if (chip->handle == pwm->chip->handle) pwm->active =0;
   }
}

lgTxRec_p lgGpioGetTxRec(lgChipObj_p chip, int gpio, int type)
{
   lgTxRec_p p;

   p = txRec;

   while ((p != NULL) &&
          ((p->chip != chip) || (p->gpio != gpio) || (p->type !=type)))
      p = p->next;

   return p;
}

lgTxRec_p lgGpioCreateTxRec(
   lgChipObj_p chip,
   int gpio,
   int micros_on,
   int micros_off,
   int micros_offset,
   int cycles)
{
   lgTxRec_p p;
   int usec, ct, cyc, frac, left;

   p = malloc(sizeof(lgTxRec_t));

   if (p)
   {
      p->type = LG_TX_PWM;
      p->chip = chip;
      p->gpio = gpio;
      p->entries = 1;
      p->micros_on[0] = micros_on;
      p->micros_off[0] = micros_off;
      p->micros_offset = micros_offset;
      if (cycles) p->cycles[0] = cycles; else p->cycles[0] = -1;
      if (micros_on) p->next_level = 1; else p->next_level = 0;
      p->active = 1;

      lgPthTxLock();

      usec = pthTxReq.tv_nsec / 1000;
      ct = micros_on + micros_off;
      cyc = usec / ct;
      frac = usec - (ct *cyc);
      left = ct - frac + pthTxDelayMicros + micros_offset;
      p->next_micros = left;
      
      p->prev = NULL;
      p->next = txRec;
      if (txRec) txRec->prev = p;
      txRec = p;

      lgPthTxUnlock();
   }

   return p;
}

lgTxRec_p lgGroupCreateWaveRec(
   lgChipObj_p chip,
   int gpio,
   int count,
   lgPulse_p pulses)
{
   lgTxRec_p p;

   p = malloc(sizeof(lgTxRec_t));

   if (p)
   {
      p->type = LG_TX_WAVE;
      p->chip = chip;
      p->gpio = gpio;
      p->entries = 1;
      p->active = 1;

      p->pulses[0] = pulses;
      p->num_pulses[0] = count;
      p->pulse_pos = 0;

      lgPthTxLock();

      p->next_micros = pthTxDelayMicros;
      
      p->prev = NULL;
      p->next = txRec;
      if (txRec) txRec->prev = p;
      txRec = p;

      lgPthTxUnlock();
   }

   return p;
}

