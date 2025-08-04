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

#define _GNU_SOURCE /* needed for ppoll */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>

#include "lgDbg.h"
#include "lgHdl.h"
#include "lgGpio.h"
#include "lgPthAlerts.h"

#define LG_MAX_ALERTS 2000
#define LG_GPIO_MAX_ALERTS_PER_READ 128

pthread_t pthAlert;
pthread_mutex_t lgAlertMutex = PTHREAD_MUTEX_INITIALIZER;
volatile lgAlertRec_p alertRec = NULL;
pthread_mutex_t lgAlertCondMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t lgAlertCond = PTHREAD_COND_INITIALIZER;
int pthAlertRunning = LG_THREAD_NONE;

lgGpioAlert_t aBuf[LG_MAX_ALERTS];

static uint64_t xNowTimestamp;

static void xWaitForSignal(pthread_cond_t *cond, pthread_mutex_t *mutex)
{
   pthread_mutex_lock(mutex);
   pthread_cond_wait(cond, mutex);
   pthread_mutex_unlock(mutex);
}

void xSendUnwaitSignal(pthread_cond_t *cond, pthread_mutex_t *mutex)
{
   pthread_mutex_lock(mutex);
   pthread_cond_signal(cond);
   pthread_mutex_unlock(mutex);
}

int tscomp(const void *p1, const void *p2)
{
   const lgGpioAlert_t *e1 = p1;
   const lgGpioAlert_t *e2 = p2;

   return e1->report.timestamp - e2->report.timestamp;
}

void emitNotifications(int count)
{
   static int maxHandles = 20;
   static int *handles = NULL;
   lgGpioReport_t report[LG_MAX_ALERTS];
   int numHandles;
   int i;
   int status;
   lgNotify_t *h;
   int emit;
   int d;
   int sent;
   int max_emits;
   int err;

   if (handles == NULL) handles = malloc(sizeof(int) * maxHandles);

   numHandles = lgHdlGetHandlesForType(
      LG_HDL_TYPE_NOTIFY, handles, maxHandles);
   
   if (numHandles > maxHandles)
   {
      LG_DBG(LG_DEBUG_ALWAYS, "too many notifications");
      numHandles = maxHandles;
   }

   for (i=0; i<numHandles; i++)
   {
      status = lgHdlGetLockedObjTrusted(
         handles[i], LG_HDL_TYPE_NOTIFY, (void **)&h);
      
      if (status < 0) continue;

      if (h->state == LG_NOTIFY_CLOSING)
      {
         lgHdlFree(handles[i], LG_HDL_TYPE_NOTIFY);
      }
      else if (h->state >= LG_NOTIFY_RUNNING)
      {
         emit = 0;

         if (h->state == LG_NOTIFY_RUNNING)
         {
            for (d=0; d<count; d++)
            {
               /* is this for this notify handle? */
               if (handles[i] == aBuf[d].nfyHandle)
               {
                  report[emit] = aBuf[d].report;
                  emit++;
               }
            }
         }

         if (emit)
         {
            max_emits = h->max_emits;

            sent = 0;

            while (emit > 0)
            {
               if (emit > max_emits)
               {
                  err = write(h->fd,
                           report+sent,
                           max_emits*sizeof(lgGpioReport_t));

                  if (err != (max_emits*sizeof(lgGpioReport_t)))
                  {
                     if (err < 0)
                     {
                        if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
                        {
                           /* serious error, no point continuing */

                           LG_DBG(LG_DEBUG_ALWAYS, "fd=%d err=%d errno=%d",
                              h->fd, err, errno);

                           LG_DBG(LG_DEBUG_ALWAYS, "%s", strerror(errno));

                           h->state = LG_NOTIFY_CLOSING;
                           break;
                        }
                        //else gpioStats.wouldBlockPipeWrite++;
                     }
                     else
                     {
                        //gpioStats.shortPipeWrite++;
                        LG_DBG(LG_DEBUG_ALWAYS, "sent %zd, asked for %d",
                           err/sizeof(lgGpioReport_t), max_emits);
                     }
                  }
                  else
                  {
                     //gpioStats.goodPipeWrite++;
                  }

                  sent += max_emits;
                  emit    -= max_emits;
               }
               else
               {
                  err = write(h->fd,
                           report+sent,
                           emit*sizeof(lgGpioReport_t));

                  if (err != (emit*sizeof(lgGpioReport_t)))
                  {
                     if (err < 0)
                     {
                        if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
                        {
                           LG_DBG(LG_DEBUG_ALWAYS, "fd=%d err=%d errno=%d",
                              h->fd, err, errno);

                           LG_DBG(LG_DEBUG_ALWAYS, "%s", strerror(errno));

                           /* serious error, no point continuing */
                           h->state = LG_NOTIFY_CLOSING;
                           break;
                        }
                        //else gpioStats.wouldBlockPipeWrite++;
                     }
                     else
                     {
                        //gpioStats.shortPipeWrite++;
                        LG_DBG(LG_DEBUG_ALWAYS, "sent %zd, asked for %d",
                           err/sizeof(lgGpioReport_t), emit);
                     }
                  }
                  else
                  {
                     //gpioStats.goodPipeWrite++;
                  }

                  sent += emit;
                  emit = 0;
               }
            }
         }
      }

      lgHdlUnlock(handles[i]);
   }
}

int emit(int count, uint64_t tmax)
{
   int i;

   for (i=0; i<count; i++)
   {
      if (aBuf[i].report.timestamp > tmax) break;
   }
 
   if (lgGpioSamplesFunc)
      (lgGpioSamplesFunc)(i, aBuf, lgGpioSamplesUserdata);
   
   emitNotifications(i);

   return i;
}

void printbuf(int count, char *str)
{
   int i;

   fprintf(stderr, "%s\n", str);
   for (i=0; i<count; i++)
   {
      fprintf(stderr, "%"PRIu64" %d %d %d (%d of %d)\n",
         aBuf[i].report.timestamp, aBuf[i].report.level,
         aBuf[i].report.chip, aBuf[i].report.gpio, i+1, count);
   }
}

void lgcheck(int count, char *str)
{
   int i;
   int64_t diff;

   for (i=1; i<count; i++)
   {
      diff = aBuf[i-1].report.timestamp - aBuf[i].report.timestamp;
      if (diff > 0)
      {
            fprintf(stderr, "%s\n%"PRIu64" %d %d %d\n",
            str, aBuf[i-1].report.timestamp, aBuf[i-1].report.level,
            aBuf[i-1].report.chip, aBuf[i-1].report.gpio);
            fprintf(stderr, "%"PRIu64" %d %d %d\n\n",
            aBuf[i].report.timestamp, aBuf[i].report.level,
            aBuf[i].report.chip, aBuf[i].report.gpio);
      }
   }
}

void xDebWatEvt(
   lgAlertRec_p p, uint64_t ts, int *cp, struct gpioevent_data *ep)
{
   int64_t nano_diff;

   if (p->debounce_nanos && !p->debounced)
   {
      /*
      Only report stable edges.  A stable edge is defined as one
      which has not changed for debounce nanoseconds.

      The following rules are applied.

      1.  If only rising edges are being monitored a rising edge
      is reported if and only no other edge is detected for at
      least debounce nanoseconds after it occurred.

      2.  If only falling edges are being monitored a falling edge
      is reported if and only no other edge is detected for at
      least debounce nanoseconds after it occurred.

      3.  If both edges are being monitored an edge is reported
      if and only no other edge is detected for at least debounce
      nanoseconds after it occurred and the edge is different to
      the previously reported edge.
      */

      nano_diff = ts - p->last_evt_ts;

      if (nano_diff > p->debounce_nanos)
      {
         /* GPIO stable for debounce period */

         if ((p->eFlags != LG_BOTH_EDGES) ||
             (p->last_rpt_lv != p->last_evt_lv))
         {
            aBuf[*cp].report.timestamp = p->last_evt_ts + p->debounce_nanos;
            aBuf[*cp].report.level = p->last_evt_lv;
            aBuf[*cp].report.chip = p->chip->gpiochip;
            aBuf[*cp].report.gpio = p->gpio;
            aBuf[*cp].report.flags = 0;
            aBuf[*cp].nfyHandle = p->nfyHandle;

            if (++(*cp) < LG_MAX_ALERTS)
            {
               p->last_rpt_ts = p->last_evt_ts + p->debounce_nanos;
               p->last_rpt_lv = p->last_evt_lv;
               p->debounced = 1;
               p->watchdogd = 0;
            }
            else
            {
               --(*cp);
               LG_DBG(LG_DEBUG_ALWAYS, "more than %d alerts", LG_MAX_ALERTS);
            }
         }
      }
   }

   if (p->watchdog_nanos && !p->watchdogd)
   {
      /*
      Ensure that a watchdog report is sent for a GPIO in the
      absence of edge reports.

      The following rule is applied.

      If no edge report has been issued for the GPIO in
      the last watchdog nanoseconds then one watchdog report
      is generated.

      Note that only one watchdog report is sent until the
      watchdog is reset by the sending of an edge report.
      */

      nano_diff = ts - p->last_rpt_ts;

      if (nano_diff > p->watchdog_nanos)
      {
         aBuf[*cp].report.timestamp = ts;
         aBuf[*cp].report.level = LG_TIMEOUT;
         aBuf[*cp].report.chip = p->chip->gpiochip;
         aBuf[*cp].report.gpio = p->gpio;
         aBuf[*cp].report.flags = 0;
         aBuf[*cp].nfyHandle = p->nfyHandle;

         if (++(*cp) < LG_MAX_ALERTS)
         {
            p->watchdogd = 1;
            p->last_rpt_ts = ts;
            p->last_rpt_lv = LG_TIMEOUT;
         }
         else
         {
            --(*cp);
            LG_DBG(LG_DEBUG_ALWAYS, "more than %d alerts", LG_MAX_ALERTS);
         }
      }
   }

   if (ep != NULL)
   {
      p->last_evt_ts = ts;
      p->last_evt_lv = 2 - ep->id; /* (falling) 2 (rising) 1 -> 0 1 */
      p->debounced = 0;

      if (!p->debounce_nanos) // report straightaway if no debounce
      {

         aBuf[*cp].report.timestamp = p->last_evt_ts;
         aBuf[*cp].report.level = p->last_evt_lv; 
         aBuf[*cp].report.chip = p->chip->gpiochip;
         aBuf[*cp].report.gpio = p->gpio;
         aBuf[*cp].report.flags = 0;
         aBuf[*cp].nfyHandle = p->nfyHandle;

         if (++(*cp) < LG_MAX_ALERTS)
         {
            p->watchdogd = 0;
            p->last_rpt_ts = p->last_evt_ts;
            p->last_rpt_lv = p->last_evt_lv;
         }
         else
         {
            --(*cp);
            LG_DBG(LG_DEBUG_ALWAYS, "more than %d alerts", LG_MAX_ALERTS);
         }
      }
   }
}

void *lgPthAlert(void)
{
   lgAlertRec_p p, t;
   int i, e;
   int num_gpio;
   int gpiobasecount;
   int retval;
   int count=0;
   int sent;
   int bytes;
   uint64_t tmax;
   struct pollfd pfd[64];
   lgAlertRec_p pAlertRec[64];
   struct gpioevent_data eIn[LG_GPIO_MAX_ALERTS_PER_READ];
   struct timespec tspec = {0, 5e5}; /* 0.5 ms timeout */

   while (1)
   {
      pthread_mutex_lock(&lgAlertMutex);

      if (alertRec != NULL)
      {
         p = alertRec;
         i = 0;

         /* poll active alerts */

         while (p != NULL)
         {
            if (p->active)
            {
               pfd[i].fd= p->state->fd;
               pfd[i].events = POLLIN|POLLPRI;
               pAlertRec[i] = p;
               i++;
            }
            else
            {
               /* delete inactive record */

               if (p->prev) p->prev->next = p->next;
               else alertRec = p->next;

               if (p->next) p->next->prev = p->prev;

               t = p; p = p->prev; free(t);
            }

            if (p) p = p->next;
         }

         num_gpio = i;

         pthread_mutex_unlock(&lgAlertMutex);

         if (num_gpio > 0)
         {
            tmax = -1; /* largest value as int */

            retval = ppoll(pfd, num_gpio, &tspec, NULL);

            xNowTimestamp = lguTimestamp();

            for (i=0; i<num_gpio; i++)
            {
               gpiobasecount = count;

               p = pAlertRec[i];

               if ((retval > 0) && (pfd[i].revents))
               {
                  /* GPIO changed during ppoll */

                  bytes = read(pfd[i].fd, &eIn, sizeof(eIn));

                  if (bytes > 0)
                  {
                     e = 0;

                     while (bytes >= sizeof(eIn[0]))
                     {
                        /* debounce and watchdog */
                        xDebWatEvt(p, eIn[e].timestamp, &count, &eIn[e]);

                        bytes -= sizeof(eIn[0]);

                        e++;
                     }

                     if (e)
                     {
                        p->last_rpt_ts = eIn[e-1].timestamp;

                        if (eIn[e-1].timestamp < tmax)
                        {
                           tmax = eIn[e-1].timestamp;
                        }
                     }

                     if (bytes)
                     {
                        if (p->active)
                           LG_DBG(LG_DEBUG_ALWAYS, "bytes left=%d (%s)",
                              bytes, strerror(errno));
                     }
                  }
                  else
                  {
                     if (p->active)
                        LG_DBG(LG_DEBUG_ALWAYS, "read error %d (%s)",
                           errno, strerror(errno));
                  }
               }
               else 
               {
                  /* GPIO not changed during ppoll */

                  xDebWatEvt(p, xNowTimestamp, &count, NULL);
               }

               if (gpiobasecount < count)
               {
                  if (p->state->alertFunc)
                  {
                     (p->state->alertFunc)(count-gpiobasecount,
                        &aBuf[gpiobasecount], p->state->userdata);
                  }
               }

            }


            if (count > 1)
            {
               // printbuf(count, "pre qsort");
               qsort(aBuf, count, sizeof(aBuf[0]), tscomp);
               lgcheck(count, "check post qsort");
               // printbuf(count, "post qsort");
            }

            /* emit any due alerts */

            // printbuf(count, "pre emit");
            sent = emit(count, tmax);
            if (sent)
            {
               if (sent != count)
               {
                  /* shuffle entries down */
                  memmove(aBuf, aBuf+sent, sizeof(aBuf[0])*(count-sent));
               }
               count -= sent;
            }
            //printbuf(count, "post emit");
         }
         else /* no active alerts */
         {
            emit(count, -1); /* empty the buffer */
            count = 0;

            xWaitForSignal(&lgAlertCond, &lgAlertCondMutex);
         }
      }
      else /* no alerts */
      {
         pthread_mutex_unlock(&lgAlertMutex);

         emit(count, -1); /* empty the buffer */
         count = 0;

         xWaitForSignal(&lgAlertCond, &lgAlertCondMutex);
      }
   }

   pthAlertRunning = LG_THREAD_NONE;

   pthread_exit(NULL);
}

void lgPthAlertStart(void)
{
   if (!pthAlertRunning)
   {
      if (pthread_create(&pthAlert, NULL, (void*)lgPthAlert, NULL) == 0)
      {
         pthread_detach(pthAlert);
         pthAlertRunning = LG_THREAD_STARTED;
      }
   }
}

void lgPthAlertStop(lgChipObj_p chip)
{
   lgAlertRec_p evt;

   /* stop any alert reads on chip */
   
   for (evt=alertRec; evt!=NULL; evt=evt->next)
   {
      if (chip->handle == evt->chip->handle) evt->active =0;
   }

   xSendUnwaitSignal(&lgAlertCond, &lgAlertCondMutex);
}

lgAlertRec_p lgGpioGetAlertRec(lgChipObj_p chip, int gpio)
{
   lgAlertRec_p p = alertRec;

   while ((p != NULL) && ((p->chip != chip) || (p->gpio != gpio)))
      p = p->next;
   return p;
}

lgAlertRec_p lgGpioCreateAlertRec(
   lgChipObj_p chip, int gpio, lgLineInf_p state, int nfyHandle)
{
   lgAlertRec_p p;

   p = malloc(sizeof(lgAlertRec_t));

   if (p)
   {
      p->chip = chip;
      p->gpio = gpio;
      p->state = state;
      p->nfyHandle = nfyHandle;
      p->active = 1;
      p->debounced = 1;
      p->watchdogd = 1;
      p->last_rpt_lv = -1; /* impossible level */
      p->debounce_nanos = state->debounce_us * 1e3;
      p->watchdog_nanos = state->watchdog_us * 1e3;
      p->eFlags = state->eFlags;
      pthread_mutex_lock(&lgAlertMutex);

      p->prev = NULL;
      p->next = alertRec;
      if (alertRec) alertRec->prev = p;
      alertRec = p;

      pthread_mutex_unlock(&lgAlertMutex);

      xSendUnwaitSignal(&lgAlertCond, &lgAlertCondMutex);
   }
   return p;
}


