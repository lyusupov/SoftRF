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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <netdb.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <sys/select.h>

#include <arpa/inet.h>

#include "rgpiod.h"

#include "rgpio.h"
#include "lgCfg.h"
#include "lgMD5.h"

#define LG_MAX_REPORTS_PER_READ 4096

#define STACK_SIZE (256*1024)

#define MAX_SBC 32

typedef void (*CBF_t) ();

struct callback_s
{

   int id;
   int sbc;
   int chip;
   int gpio;
   int edge;
   CBF_t f;
   void * user;
   callback_t *prev;
   callback_t *next;
};

typedef struct
{
   size_t count; // number of elements
   size_t bytes; // bytes per element
   size_t size;  // total number of bytes
   const void *ptr;
} lgExtent_t;

/* GLOBALS ---------------------------------------------------------------- */

static int             gAbort = 0;

static int             gPiInUse     [MAX_SBC];

static int             gPigCommand  [MAX_SBC];
static int             gPigHandle   [MAX_SBC];
static int             gPigNotify   [MAX_SBC];

static uint32_t        gLastLevel   [MAX_SBC];

static pthread_t       *gPthNotify  [MAX_SBC];

static pthread_mutex_t gCmdMutex    [MAX_SBC];
static int             gCancelState [MAX_SBC];

static uint8_t         *gMsgBuf     [MAX_SBC];

static callback_t     *gCallBackFirst = 0;
static callback_t     *gCallBackLast  = 0;

/* PRIVATE ---------------------------------------------------------------- */

static uint64_t xMakeSalt(void)
{
   struct timespec xts;

   clock_gettime(CLOCK_REALTIME, &xts);

   return ((xts.tv_sec + xts.tv_nsec) * random()) + (random() + xts.tv_nsec);
}

static void xStopAll(void)
{
   static int xInited = 0;

   if (!xInited) gAbort = 1;

   fflush(NULL);

   xInited = 1;

   signal(SIGINT, SIG_DFL);
   raise(SIGINT);
}

static void xSignalHandler(int signum)
{
   xStopAll();
}

static void _pml(int sbc)
{
   int cancelState;

   pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &cancelState);
   pthread_mutex_lock(&gCmdMutex[sbc]);
   gCancelState[sbc] = cancelState;
}

static void _pmu(int sbc)
{
   int cancelState;

   cancelState = gCancelState[sbc];
   pthread_mutex_unlock(&gCmdMutex[sbc]);
   pthread_setcancelstate(cancelState, NULL);
}

static int lg_command
   (int sbc, int command, int extents, lgExtent_t *ext, int rl)
{
   int i;
   lgCmd_p h;
   uint8_t *p;
   size_t len;
 
   if ((sbc < 0) || (sbc >= MAX_SBC) || !gPiInUse[sbc])
   {
      return lgif_unconnected_sbc;
   }

   if (gAbort)
   {
      rgpiod_stop(sbc);
      return lgif_unconnected_sbc;
   }

   _pml(sbc);

   p = gMsgBuf[sbc];
   
   h = (lgCmd_p) p;

   h->magic = LG_MAGIC;
   h->size = 0;
   h->cmd = command;
   h->doubles = 0;
   h->longs = 0;
   h->shorts = 0;

   p += sizeof(lgCmd_t);

   for (i=0; i<extents; i++)
   {
      h->size += ext[i].size;
      memcpy(p, ext[i].ptr, ext[i].size);
      p += ext[i].size;

      switch(ext[i].bytes)
      {
         case 8:
            h->doubles += ext[i].count;
            break;

         case 4:
            h->longs += ext[i].count;
            break;

         case 2:
            h->shorts += ext[i].count;
            break;
      }
   }

   len = sizeof(lgCmd_t) + h->size;

   if (send(gPigCommand[sbc], gMsgBuf[sbc], len, 0) != len)
   {
      _pmu(sbc);
      return lgif_bad_send;
   }

   if (recv(gPigCommand[sbc], h, sizeof(lgCmd_t), MSG_WAITALL) !=
      sizeof(lgCmd_t))
   {
      _pmu(sbc);
      return lgif_bad_recv;
   }

   if (rl) _pmu(sbc);

   return h->status;
}


static int lg_notify(int sbc)
{
   lgCmd_t h;
 
   if ((sbc < 0) || (sbc >= MAX_SBC) || !gPiInUse[sbc])
      return lgif_unconnected_sbc;

   h.magic = LG_MAGIC;
   h.size = 0;
   h.cmd = LG_CMD_NOIB;
   h.doubles = 0;
   h.longs = 0;
   h.shorts = 0;

   _pml(sbc);

   if (send(gPigNotify[sbc], &h, sizeof(h), 0) != sizeof(h))
   {
      _pmu(sbc);
      return lgif_bad_send;
   }

   if (recv(gPigNotify[sbc], &h, sizeof(h), MSG_WAITALL) != sizeof(h))
   {
      _pmu(sbc);
      return lgif_bad_recv;
   }

   _pmu(sbc);

   return h.status;
}

static int lg_command_0(int sbc, int command, int rl)
   {return lg_command(sbc, command, 0, NULL, rl);}

static int lg_command_1(int sbc, int command, int p1, int rl)
{
   lgExtent_t ext[1];
   uint32_t pars[]={p1};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   return lg_command(sbc, command, 1, ext, rl);
}

static int lg_command_2(int sbc, int command, int p1, int p2, int rl)
{
   lgExtent_t ext[1];
   uint32_t pars[]={p1, p2};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   return lg_command(sbc, command, 1, ext, rl);
}

static int lg_command_3(int sbc, int command, int p1, int p2, int p3, int rl)
{
   lgExtent_t ext[1];
   uint32_t pars[]={p1, p2, p3};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   return lg_command(sbc, command, 1, ext, rl);
}


static int lgOpenSocket(const char *addrStr, const char *portStr)
{
   int sock, err, opt;
   struct addrinfo hints, *res, *rp;

   memset (&hints, 0, sizeof (hints));

   hints.ai_family   = PF_UNSPEC;
   hints.ai_socktype = SOCK_STREAM;
   hints.ai_flags   |= AI_CANONNAME;

   err = getaddrinfo (addrStr, portStr, &hints, &res);

   if (err) return lgif_bad_getaddrinfo;

   for (rp=res; rp!=NULL; rp=rp->ai_next)
   {
      sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);

      if (sock == -1) continue;

      /* Disable the Nagle algorithm. */
      opt = 1;
      setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char*)&opt, sizeof(int));

      if (connect(sock, rp->ai_addr, rp->ai_addrlen) != -1) break;
   }

   freeaddrinfo(res);

   if (rp == NULL) return lgif_bad_connect;

   return sock;
}

static void dispatch_notification(int sbc, lgGpioReport_t *r)
{
   callback_t *p;

/*   
   fprintf(stderr, "ts=%"PRIu64" c=%d g=%d l=%d f=%d\n",
      r->timestamp, r->chip, r->gpio, r->level, r->flags);
*/   

   if (r->flags == 0)
   {
      p = gCallBackFirst;

      while (p)
      {
         if ((p->sbc == sbc) && (p->chip == r->chip) && (p->gpio == r->gpio))
         {
            (p->f)(sbc, p->chip, p->gpio, r->level, r->timestamp, p->user);
         }
         p = p->next;
      }
   }
   else /* no flags currently defined, ignore */
   {
   }
}

static void *pthNotifyThread(void *x)
{
   static int got = 0;
   int sbc;
   int bytes, r;
   lgGpioReport_t report[LG_MAX_REPORTS_PER_READ];

   sbc = *((int*)x);
   free(x); /* memory allocated in rgpiod_start */

   while (1)
   {
      bytes = read(gPigNotify[sbc], (char*)&report+got, sizeof(report)-got);

      if (bytes > 0) got += bytes;
      else break;

      r = 0;

      while (got >= sizeof(lgGpioReport_t))
      {
         dispatch_notification(sbc, &report[r]);

         r++;

         got -= sizeof(lgGpioReport_t);
      }

      /* copy any partial report to start of array */
      
      if (got && r) report[0] = report[r];
   }

   fprintf(stderr, "notify thread for sbc %d broke with read error %d\n",
      sbc, bytes);

   while (1) sleep(1);

   return NULL;
}

static int intCallback(
   int sbc, int chip, int gpio, int edge, void *f, void *user)
{
   static int id = 0;
   callback_t *p;

   /*
   printf("sbc=%d chip=%d gpio=%d edge=%d f=%p u=%p\n",
      sbc, chip, gpio, edge, f, user);
   */

   if ((chip < 64) && (gpio < 64) && (edge <= 3) && f)
   {
      /* prevent duplicates */

      p = gCallBackFirst;

      while (p)
      {
         if ((p->sbc  == sbc)  &&
             (p->chip == chip) &&
             (p->gpio == gpio) &&
             (p->edge == edge) &&
             (p->f    == f))
         {
            return lgif_duplicate_callback;
         }
         p = p->next;
      }

      p = malloc(sizeof(callback_t));

      if (p)
      {
         if (!gCallBackFirst) gCallBackFirst = p;

         p->id = id++;
         p->sbc = sbc;
         p->chip = chip;
         p->gpio = gpio;
         p->edge = edge;
         p->f = f;
         p->user = user;
         p->next = 0;
         p->prev = gCallBackLast;

         if (p->prev) (p->prev)->next = p;
         gCallBackLast = p;

         return p->id;
      }

      return lgif_bad_malloc;
   }

   return lgif_bad_callback;
}

static int recvMax(int sbc, void *buf, int bufsize, int sent)
{
   /*
   Copy at most bufSize bytes from the receieved message to
   buf.  Discard the rest of the message.
   */
   uint8_t scratch[4096];
   int remaining, fetch, count;

   if (sent < bufsize) count = sent; else count = bufsize;

   if (count) recv(gPigCommand[sbc], buf, count, MSG_WAITALL);

   remaining = sent - count;

   while (remaining)
   {
      fetch = remaining;
      if (fetch > sizeof(scratch)) fetch = sizeof(scratch);
      recv(gPigCommand[sbc], scratch, fetch, MSG_WAITALL);
      remaining -= fetch;
   }

   return count;
}

/* PUBLIC ----------------------------------------------------------------- */

/* START/STOP */

int rgpiod_start(const char *addrStr, const char *portStr)
{
   static int xInited = 0;
   int sbc;
   int *userdata;
   struct sigaction new_action, old_action;

   if (!xInited)
   {
      for (sbc=0; sbc<MAX_SBC; sbc++)
         pthread_mutex_init(&gCmdMutex[sbc], NULL);

      /* Set up the structure to specify the new action. */
      new_action.sa_handler = xSignalHandler;
      sigemptyset (&new_action.sa_mask);
      new_action.sa_flags = 0;

      sigaction (SIGINT, NULL, &old_action);

      if (old_action.sa_handler != SIG_IGN)
         sigaction (SIGINT, &new_action, NULL);

      atexit(xStopAll);

      xInited = 1;
   }

   for (sbc=0; sbc<MAX_SBC; sbc++)
   {
      _pml(sbc);
      if (!gPiInUse[sbc])
      {
         gPiInUse[sbc] = 1;
         _pmu(sbc);
         break;
      }
      _pmu(sbc);
   }

   if (sbc >= MAX_SBC) return lgif_too_many_pis;

   if ((!addrStr)  || (!strlen(addrStr)))
   {
      addrStr = getenv(LG_ENVADDR);

      if ((!addrStr) || (!strlen(addrStr)))
      {
         addrStr = LG_DEFAULT_SOCKET_ADDR_STR;
      }
   }

   if ((!portStr) || (!strlen(portStr)))
   {
      portStr = getenv(LG_ENVPORT);

      if ((!portStr) || (!strlen(portStr)))
      {
         portStr = LG_DEFAULT_SOCKET_PORT_STR;
      }
   }

   gPigCommand[sbc] = lgOpenSocket(addrStr, portStr);

   if (gPigCommand[sbc] >= 0)
   {
      gPigNotify[sbc] = lgOpenSocket(addrStr, portStr);

      if (gPigNotify[sbc] >= 0)
      {
         gPigHandle[sbc] = lg_notify(sbc);

         if (gPigHandle[sbc] < 0) return lgif_bad_noib;
         else
         {
            gLastLevel[sbc] = 0;

            /* must be freed by pthNotifyThread */
            userdata = malloc(sizeof(*userdata));
            *userdata = sbc;

            gPthNotify[sbc] = thread_start(pthNotifyThread, userdata);

            if (gPthNotify[sbc])
            {
               gMsgBuf[sbc] = malloc(CMD_MAX_EXTENSION);
               if (gMsgBuf[sbc] != NULL) return sbc;
               else return lgif_bad_malloc;
            }
            else return lgif_notify_failed;

         }
      }
      else return gPigNotify[sbc];
   }
   else return gPigCommand[sbc];
}

void rgpiod_stop(int sbc)
{
   if ((sbc < 0) || (sbc >= MAX_SBC) || !gPiInUse[sbc]) return;

   if (gPthNotify[sbc])
   {
      thread_stop(gPthNotify[sbc]);
      gPthNotify[sbc] = 0;
   }

   if (gPigCommand[sbc] >= 0)
   {
      //lg_command_0(sbc, LG_CMD_FREE, 1);

      if (gPigHandle[sbc] >= 0)
      {
         //lg_command_1(sbc, LG_CMD_NC, gPigHandle[sbc], 1);
         gPigHandle[sbc] = -1;
      }

      close(gPigCommand[sbc]);
      gPigCommand[sbc] = -1;
   }

   if (gPigNotify[sbc] >= 0)
   {
      close(gPigNotify[sbc]);
      gPigNotify[sbc] = -1;
   }

   _pml(sbc);
   gPiInUse[sbc] = 0;
   free(gMsgBuf[sbc]);
   gMsgBuf[sbc] = NULL;
   _pmu(sbc);
}


/* FILES */

int file_open(int sbc, const char *file, int mode)
{
   lgExtent_t ext[2];
   uint32_t pars[] = {mode};
   int len;

   len = strlen(file);

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = len;
   ext[1].count = len;
   ext[1].bytes = 1;
   ext[1].ptr = file;

   return lg_command(sbc, LG_CMD_FO, 2, ext, 1);
}

int file_close(int sbc, int handle)
   {return lg_command_1(sbc, LG_CMD_FC, handle, 1);}

int file_write(int sbc, int handle, const char *buf, int count)
{
   lgExtent_t ext[2];
   uint32_t pars[]={handle};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = count;
   ext[1].ptr = buf;

   return lg_command(sbc, LG_CMD_FW, 2, ext, 1);
}

int file_read(int sbc, int handle, char *buf, int count)
{
   int bytes;

   bytes = lg_command_2(sbc, LG_CMD_FR, handle, count, 0);

   if (bytes > 0)
   {
      bytes = recvMax(sbc, buf, count, bytes);
   }

   _pmu(sbc);

   return bytes;
}

int file_seek(int sbc, int handle, int32_t seekOffset, int seekFrom)
   {return lg_command_3(sbc, LG_CMD_FS, handle, seekOffset, seekFrom, 1);}

int file_list(int sbc, const char *fpat,  char *buf, int count)
{
   int len;
   int bytes;
   uint32_t pars[] = {60000};
   lgExtent_t ext[2];

   len = strlen(fpat);

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = len;
   ext[1].count = len;
   ext[1].bytes = 1;
   ext[1].ptr = fpat;

   bytes = lg_command(sbc, LG_CMD_FL, 2, ext, 0);

   if (bytes > 0)
   {
      bytes = recvMax(sbc, buf, count, bytes);
   }

   _pmu(sbc);

   return bytes;
}

/* GPIO */

int gpiochip_open(int sbc, int device)
{
   int status;

   status = lg_command_1(sbc, LG_CMD_GO, device, 1);
   if (status >= 0) status |= (device<<16);

   return status;
}


int gpiochip_close(int sbc, int handle)
   {return lg_command_1(sbc, LG_CMD_GC, handle&0xffff, 1);}

int gpio_get_chip_info(int sbc, int handle, lgChipInfo_p chipInfP)
{
   int status;
   int bytes;
   lgExtent_t ext[1];
   uint32_t pars[] = {handle&0xffff};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   bytes = lg_command(sbc, LG_CMD_GIC, 1, ext, 0);

   if (bytes > 0)
   {
      recvMax(sbc, chipInfP, sizeof(lgChipInfo_t), bytes);
      status = LG_OKAY;
   }
   else status = bytes;

   _pmu(sbc);

   return status;
}

int gpio_get_line_info(int sbc, int handle, int gpio, lgLineInfo_p lineInfP)
{
   int status;
   int bytes;
   lgExtent_t ext[1];
   uint32_t pars[] = {handle&0xffff, gpio};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   bytes = lg_command(sbc, LG_CMD_GIL, 1, ext, 0);

   if (bytes > 0)
   {
      recvMax(sbc, lineInfP, sizeof(lgLineInfo_t), bytes);
      status = LG_OKAY;
   }
   else status = bytes;

   _pmu(sbc);

   return status;
}

int gpio_get_mode(int sbc, int handle, int gpio)
   {return lg_command_2(sbc, LG_CMD_GMODE, handle&0xffff, gpio, 1);}

int group_read(int sbc, int handle, int group, uint64_t *value)
{
   int status;
   int bytes;
   lgExtent_t ext[1];
   uint32_t pars[] = {handle&0xffff, group};
   uint8_t retval[12];

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   bytes = lg_command(sbc, LG_CMD_GGR, 1, ext, 0);

   if (bytes > 0)
   {
      recvMax(sbc, &retval, 12, bytes);
      *value = *(uint64_t*)retval;
      status = *(uint32_t*)(retval+8);
   }
   else status = bytes;

   _pmu(sbc);

   return status;
}


int group_write(
   int sbc, int handle, int group, uint64_t groupBits, uint64_t groupMask)
{
   lgExtent_t ext[2];
   uint64_t parq[] = {groupBits, groupMask};
   uint32_t pars[] = {handle&0xffff, group};

   ext[0].size = sizeof(parq);
   ext[0].count = sizeof(parq)/sizeof(parq[0]);
   ext[0].bytes = sizeof(parq[0]);
   ext[0].ptr = &parq;

   ext[1].size = sizeof(pars);
   ext[1].count = sizeof(pars)/sizeof(pars[0]);
   ext[1].bytes = sizeof(pars[0]);
   ext[1].ptr = &pars;

   return lg_command(sbc, LG_CMD_GGWX, 2, ext, 1);
}

int gpio_claim_input(int sbc, int handle, int lFlags, int gpio)
   {return lg_command_3(sbc, LG_CMD_GSIX, handle&0xffff, lFlags, gpio, 1);}

int group_claim_input(
   int sbc, int handle, int lFlags, int size, const int *gpio)
{
   lgExtent_t ext[2];
   uint32_t pars[] = {handle&0xffff, lFlags};
   int i;
   int status=LG_NO_MEMORY;
   uint32_t *g32 = malloc(size * 4);
   
   if (g32)
   {
      ext[0].size = sizeof(pars);
      ext[0].count = sizeof(pars)/sizeof(pars[0]);
      ext[0].bytes = sizeof(pars[0]);
      ext[0].ptr = &pars;

      ext[1].size = size*4;
      ext[1].count = size;
      ext[1].bytes = 4;
      ext[1].ptr = g32;

      for (i=0; i<size; i++) g32[i] = gpio[i];
      status = lg_command(sbc, LG_CMD_GSGIX, 2, ext, 1);
      free (g32);
   }
   return status;
}

int gpio_claim_output(
   int sbc, int handle, int lFlags, int gpio, int value)
{
   lgExtent_t ext[1];
   uint32_t pars[] = {handle&0xffff, lFlags, gpio, value};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   return lg_command(sbc, LG_CMD_GSOX, 1, ext, 1);
}


int group_claim_output(
   int sbc, int handle, int lFlags,
   int size, const int *gpio, const int *values)
{
   lgExtent_t ext[2];
   uint32_t pars[] = {handle&0xffff, lFlags};
   int i;
   int status=LG_NO_MEMORY;
   uint32_t *g32 = malloc(size * 8);
   
   if (g32)
   {
      ext[0].size = sizeof(pars);
      ext[0].count = sizeof(pars)/sizeof(pars[0]);
      ext[0].bytes = sizeof(pars[0]);
      ext[0].ptr = &pars;

      ext[1].size = size*8;
      ext[1].count = size*2;
      ext[1].bytes =  4;
      ext[1].ptr = g32;

      for (i=0; i<size; i++)
      { 
         g32[i] = gpio[i];
         g32[i+size] = values[i];
      }
      status = lg_command(sbc, LG_CMD_GSGOX, 2, ext, 1);
      free (g32);
   }
   return status;
}

int gpio_free(int sbc, int handle, int gpio)
   {return lg_command_2(sbc, LG_CMD_GSF, handle&0xffff, gpio, 1);}

int group_free(int sbc, int handle, int gpio)
   {return lg_command_2(sbc, LG_CMD_GSGF, handle&0xffff, gpio, 1);}

int gpio_read(int sbc, int handle, int gpio)
   {return lg_command_2(sbc, LG_CMD_GR, handle&0xffff, gpio, 1);}

int gpio_write(int sbc, int handle, int gpio, int value)
   {return lg_command_3(sbc, LG_CMD_GW, handle&0xffff, gpio, value, 1);}

int gpio_set_debounce_time(int sbc, int handle, int gpio, int debounce_us)
{
   return lg_command_3(
      sbc, LG_CMD_GDEB, handle&0xffff, gpio, debounce_us, 1);
}

int gpio_set_watchdog_time(int sbc, int handle, int gpio, int watchdog_us)
{
   return lg_command_3(
      sbc, LG_CMD_GWDOG, handle&0xffff, gpio, watchdog_us, 1);
}

int tx_pulse(
   int sbc, int handle, int gpio,
   int micros_on, int micros_off,
   int offset, int cycles)
{
   lgExtent_t ext[1];
   uint32_t pars[] =
      {handle&0xffff, gpio, micros_on, micros_off, offset, cycles};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   return lg_command(sbc, LG_CMD_GPX, 1, ext, 1);
}

int tx_pwm(
   int sbc, int handle, int gpio,
   float pwmFrequency, float pwmDutyCycle,
   int offset, int cycles)
{
   lgExtent_t ext[1];
   uint32_t pars[] =
      {handle&0xffff, gpio, pwmFrequency*1000, pwmDutyCycle*1000,
         offset, cycles};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   return lg_command(sbc, LG_CMD_PX, 1, ext, 1);
}

int tx_servo(
   int sbc, int handle, int gpio, int width,
   int freq, int offset, int cycles)
{
   lgExtent_t ext[1];
   uint32_t pars[] =
      {handle&0xffff, gpio, width, freq, offset, cycles};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   return lg_command(sbc, LG_CMD_SX, 1, ext, 1);
}

int tx_wave(
   int sbc, int handle, int gpio, int count, lgPulse_p pulses)
{
   lgExtent_t ext[2];
   uint32_t pars[] = {handle&0xffff, gpio};

   ext[0].size = count * sizeof(lgPulse_t);
   ext[0].count = count * 3;
   ext[0].bytes = 8;
   ext[0].ptr = pulses;

   ext[1].size = sizeof(pars);
   ext[1].count = sizeof(pars)/sizeof(pars[0]);
   ext[1].bytes = sizeof(pars[0]);
   ext[1].ptr = &pars;

   return lg_command(sbc, LG_CMD_GWAVE, 2, ext, 1);
}

int tx_busy(int sbc, int handle, int gpio, int kind)
   {return lg_command_3(sbc, LG_CMD_GBUSY, handle&0xffff, gpio, kind, 1);}

int tx_room(int sbc, int handle, int gpio, int kind)
   {return lg_command_3(sbc, LG_CMD_GROOM, handle&0xffff, gpio, kind, 1);}

int gpio_claim_alert(
   int sbc, int handle, int lFlags, int eFlags, int gpio, int nfyHandle)
{
   lgExtent_t ext[1];
   uint32_t pars[] = {handle&0xffff, lFlags, eFlags, gpio, nfyHandle};

   if (nfyHandle < 0) pars[4] = gPigHandle[sbc];

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   return lg_command(sbc, LG_CMD_GSAX, 1, ext, 1);
}

int callback(
   int sbc, int handle, int gpio, int edge, CBFunc_t f, void *user)
{
   return intCallback(sbc, handle>>16, gpio, edge, f, user);
}

int callback_cancel(int id)
{
   callback_t *p;

   p = gCallBackFirst;

   while (p)
   {
      if (p->id == id)
      {
         if (p->prev) {p->prev->next = p->next;}
         else         {gCallBackFirst = p->next;}

         if (p->next) {p->next->prev = p->prev;}
         else         {gCallBackLast = p->prev;}

         free(p);

         return 0;
      }
      p = p->next;
   }
   return lgif_callback_not_found;
}

/* I2C */

int i2c_open(int sbc, int i2c_bus, int i2c_addr, int i2c_flags)
   {return lg_command_3(sbc, LG_CMD_I2CO, i2c_bus, i2c_addr, i2c_flags, 1);}

int i2c_close(int sbc, int handle)
   {return lg_command_1(sbc, LG_CMD_I2CC, handle, 1);}

int i2c_write_quick(int sbc, int handle, int bit)
   {return lg_command_2(sbc, LG_CMD_I2CWQ, handle, bit, 1);}

int i2c_write_byte(int sbc, int handle, int val)
   {return lg_command_2(sbc, LG_CMD_I2CWS, handle, val, 1);}

int i2c_read_byte(int sbc, int handle)
   {return lg_command_1(sbc, LG_CMD_I2CRS, handle, 1);}

int i2c_write_byte_data(int sbc, int handle, int reg, int val)
   {return lg_command_3(sbc, LG_CMD_I2CWB, handle, reg, val, 1);}

int i2c_write_word_data(int sbc, int handle, int reg, int val)
   {return lg_command_3(sbc, LG_CMD_I2CWW, handle, reg, val, 1);}

int i2c_read_byte_data(int sbc, int handle, int reg)
   {return lg_command_2(sbc, LG_CMD_I2CRB, handle, reg, 1);}

int i2c_read_word_data(int sbc, int handle, int reg)
   {return lg_command_2(sbc, LG_CMD_I2CRW, handle, reg, 1);}

int i2c_process_call(int sbc, int handle, int reg, int val)
   {return lg_command_3(sbc, LG_CMD_I2CPC, handle, reg, val, 1);}

int i2c_write_block_data(
   int sbc, int handle, int reg, const char *buf, int count)
{
   lgExtent_t ext[2];
   uint32_t pars[] = {handle, reg};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = count;
   ext[1].count = count;
   ext[1].bytes = 1;
   ext[1].ptr = buf;

   return lg_command(sbc, LG_CMD_I2CWK, 2, ext, 1);
}

int i2c_read_block_data(int sbc, int handle, int reg, char *buf)
{
   int bytes;

   bytes = lg_command_2(sbc, LG_CMD_I2CRK, handle, reg, 0);

   if (bytes > 0)
   {
      bytes = recvMax(sbc, buf, 32, bytes);
   }

   _pmu(sbc);

   return bytes;
}

int i2c_block_process_call(
   int sbc, int handle, int reg, char *buf, int count)
{
   int bytes;
   lgExtent_t ext[2];
   uint32_t pars[]= {handle, reg};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = count;
   ext[1].count = count;
   ext[1].bytes = 1;
   ext[1].ptr = buf;

   bytes = lg_command(sbc, LG_CMD_I2CPK, 2, ext, 0);

   if (bytes > 0)
   {
      bytes = recvMax(sbc, buf, 32, bytes);
   }

   _pmu(sbc);

   return bytes;
}

int i2c_read_i2c_block_data(
   int sbc, int handle, int reg, char *buf, int count)
{
   int bytes;
   lgExtent_t ext[1];
   uint32_t pars[] = {handle, reg, count};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   bytes = lg_command(sbc, LG_CMD_I2CRI, 1, ext, 0);

   if (bytes > 0)
   {
      bytes = recvMax(sbc, buf, count, bytes);
   }

   _pmu(sbc);

   return bytes;
}


int i2c_write_i2c_block_data(
   int sbc, int handle, int reg, const char *buf, int count)
{
   lgExtent_t ext[2];
   uint32_t pars[] = {handle, reg};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = count;
   ext[1].count = count;
   ext[1].bytes = 1;
   ext[1].ptr = buf;

   return lg_command(sbc, LG_CMD_I2CWI, 2, ext, 1);
}

int i2c_read_device(int sbc, int handle, char *buf, int count)
{
   int bytes;

   bytes = lg_command_2(sbc, LG_CMD_I2CRD, handle, count, 0);

   if (bytes > 0)
   {
      bytes = recvMax(sbc, buf, count, bytes);
   }

   _pmu(sbc);

   return bytes;
}

int i2c_write_device(int sbc, int handle, const char *buf, int count)
{
   lgExtent_t ext[2];
   uint32_t pars[] = {handle};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = count;
   ext[1].count = count;
   ext[1].bytes = 1;
   ext[1].ptr = buf;

   return lg_command(sbc, LG_CMD_I2CWD, 2, ext, 1);
}

int i2c_zip(
   int sbc, int handle, const char *inBuf, int inLen,char *outBuf, int outLen)
{
   int bytes;
   lgExtent_t ext[2];
   uint32_t pars[] = {handle};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = inLen;
   ext[1].count = inLen;
   ext[1].bytes = 1;
   ext[1].ptr = inBuf;

   bytes = lg_command(sbc, LG_CMD_I2CZ, 2, ext, 0);

   if (bytes > 0)
   {
      bytes = recvMax(sbc, outBuf, outLen, bytes);
   }

   _pmu(sbc);

   return bytes;
}


/* NOTIFICATIONS */

int notify_open(int sbc)
   {return lg_command_0(sbc, LG_CMD_NO, 1);}

int notify_resume(int sbc, int handle)
   {return lg_command_1(sbc, LG_CMD_NR, handle, 1);}

int notify_pause(int sbc, int handle)
   {return lg_command_1(sbc, LG_CMD_NP, handle, 1);}

int notify_close(int sbc, int handle)
   {return lg_command_1(sbc, LG_CMD_NC, handle, 1);}


/* SCRIPTS */

int script_store(int sbc, const char *script)
{
   int len;
   lgExtent_t ext[1];

   len = strlen(script);

   if (!len) return 0;

   ext[0].size = len;
   ext[0].count = len;
   ext[0].bytes = 1;
   ext[0].ptr = script;

   return lg_command(sbc, LG_CMD_PROC, 1, ext, 1);
}

int script_run(int sbc, int handle, int count, const uint32_t *param)
{
   lgExtent_t ext[2];
   uint32_t pars[] = {handle};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = 4 * count;
   ext[1].count = count;
   ext[1].bytes = 4;
   ext[1].ptr = param;

   return lg_command(sbc, LG_CMD_PROCR, 2, ext, 1);
}

int script_update(int sbc, int handle, int count, const uint32_t *param)
{
   lgExtent_t ext[2];
   uint32_t pars[] = {handle};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = 4 * count;
   ext[1].count = count;
   ext[1].bytes = 4;
   ext[1].ptr = param;

   return lg_command(sbc, LG_CMD_PROCU, 2, ext, 1);
}

int script_status(int sbc, int handle, uint32_t *param)
{
   int status;
   uint32_t p[LG_MAX_SCRIPT_PARAMS+1]; /* space for script status */

   status = lg_command_1(sbc, LG_CMD_PROCP, handle, 0);

   if (status > 0)
   {
      recvMax(sbc, p, sizeof(p), status);
      status = p[0];
      if (param) memcpy(param, p+1, sizeof(p)-4);
   }

   _pmu(sbc);

   return status;
}

int script_stop(int sbc, int handle)
   {return lg_command_1(sbc, LG_CMD_PROCS, handle, 1);}

int script_delete(int sbc, int handle)
   {return lg_command_1(sbc, LG_CMD_PROCD, handle, 1);}

/* SERIAL */

int serial_open(int sbc, const char *dev, int baud, int flags)
{
   int len;
   lgExtent_t ext[2];
   uint32_t pars[] = {baud, flags};

   len = strlen(dev);

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = len;
   ext[1].count = len;
   ext[1].bytes = 1;
   ext[1].ptr = dev;

   return lg_command(sbc, LG_CMD_SERO, 2, ext, 1);
}

int serial_close(int sbc, int handle)
   {return lg_command_1(sbc, LG_CMD_SERC, handle, 1);}

int serial_write_byte(int sbc, int handle, int val)
   {return lg_command_2(sbc, LG_CMD_SERWB, handle, val, 1);}

int serial_read_byte(int sbc, int handle)
   {return lg_command_1(sbc, LG_CMD_SERRB, handle, 1);}

int serial_write(int sbc, int handle, const char *buf, int count)
{
   lgExtent_t ext[2];
   uint32_t pars[] = {handle};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = count;
   ext[1].count = count;
   ext[1].bytes = 1;
   ext[1].ptr = buf;

   return lg_command(sbc, LG_CMD_SERW, 2, ext, 1);
}

int serial_read(int sbc, int handle, char *buf, int count)
{
   int bytes;

   bytes = lg_command_2(sbc, LG_CMD_SERR, handle, count, 0);

   if (bytes > 0)
   {
      bytes = recvMax(sbc, buf, count, bytes);
   }

   _pmu(sbc);

   return bytes;
}

int serial_data_available(int sbc, int handle)
   {return lg_command_1(sbc, LG_CMD_SERDA, handle, 1);}

/* SHELL */

int shell(int sbc, const char *scriptName, const char *scriptString)
{
   lgExtent_t ext[3];
   uint32_t ln, ls;

   ln = strlen(scriptName)+1;
   ls = strlen(scriptString)+1;

   ext[0].size = 4;
   ext[0].count = 1;
   ext[0].bytes = 4;
   ext[0].ptr = &ln;

   ext[1].size = ln; /* include null byte */
   ext[1].count = ln;
   ext[1].bytes = 1;
   ext[1].ptr = scriptName;

   ext[2].size = ls; /* include null byte */
   ext[2].count = ls;
   ext[2].bytes = 1;
   ext[2].ptr = scriptString;

   return lg_command(sbc, LG_CMD_SHELL, 3, ext, 1);
}


/* SPI */

int spi_open(int sbc,
   int device, int channel, int speed, int flags)
{
   lgExtent_t ext[1];
   uint32_t pars[] = {device, channel, speed, flags};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   return lg_command(sbc, LG_CMD_SPIO, 1, ext, 1);
}

int spi_close(int sbc, int handle)
   {return lg_command_1(sbc, LG_CMD_SPIC, handle, 1);}

int spi_read(int sbc, int handle, char *buf, int count)
{
   int bytes;

   bytes = lg_command_2(sbc, LG_CMD_SPIR, handle, count, 0);

   if (bytes > 0)
   {
      bytes = recvMax(sbc, buf, count, bytes);
   }

   _pmu(sbc);

   return bytes;
}

int spi_write(int sbc, int handle, const char *buf, int count)
{
   lgExtent_t ext[2];
   uint32_t pars[] = {handle};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = count;
   ext[1].count = count;
   ext[1].bytes = 1;
   ext[1].ptr = buf;

   return lg_command(sbc, LG_CMD_SPIW, 2, ext, 1);
}

int spi_xfer(int sbc, int handle, const char *txBuf, char *rxBuf, int count)
{
   int bytes;
   lgExtent_t ext[2];
   uint32_t pars[] = {handle};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   ext[1].size = count;
   ext[1].count = count;
   ext[1].bytes = 1;
   ext[1].ptr = txBuf;

   bytes = lg_command(sbc, LG_CMD_SPIX, 2, ext, 0);

   if (bytes > 0)
   {
      bytes = recvMax(sbc, rxBuf, count, bytes);
   }

   _pmu(sbc);

   return bytes;
}

/* THREADS */

pthread_t *thread_start(lgThreadFunc_t thread_func, void *userdata)
{
   pthread_t *pth;
   pthread_attr_t pthAttr;

   pth = malloc(sizeof(pthread_t));

   if (pth)
   {
      if (pthread_attr_init(&pthAttr))
      {
         perror("pthread_attr_init failed");
         free(pth);
         return NULL;
      }

      if (pthread_attr_setstacksize(&pthAttr, STACK_SIZE))
      {
         perror("pthread_attr_setstacksize failed");
         free(pth);
         return NULL;
      }

      if (pthread_create(pth, &pthAttr, thread_func, userdata))
      {
         perror("pthread_create failed");
         free(pth);
         return NULL;
      }
   }

   return pth;
}

void thread_stop(pthread_t *pth)
{
   if (pth)
   {
      pthread_cancel(*pth);
      pthread_join(*pth, NULL);
      free(pth);
   }
}

/* UTILITIES */

int lgu_get_internal(int sbc, int config_id, uint64_t *config_value)
{
   int status;
   int bytes;
   lgExtent_t ext[1];
   uint32_t pars[] = {config_id};

   ext[0].size = sizeof(pars);
   ext[0].count = sizeof(pars)/sizeof(pars[0]);
   ext[0].bytes = sizeof(pars[0]);
   ext[0].ptr = &pars;

   bytes = lg_command(sbc, LG_CMD_CGI, 1, ext, 0);

   if (bytes > 0)
   {
      recvMax(sbc, config_value, 8, bytes);
      status = LG_OKAY;
   }
   else status = bytes;

   _pmu(sbc);

   return status;
}

int lgu_set_internal(int sbc, int config_id, uint64_t config_value)
{
   lgExtent_t ext[2];
   uint64_t parsQ[] = {config_value};
   uint32_t parsI[] = {config_id};

   ext[0].size = sizeof(parsQ);
   ext[0].count = sizeof(parsQ)/sizeof(parsQ[0]);
   ext[0].bytes = sizeof(parsQ[0]);
   ext[0].ptr = &parsQ;

   ext[1].size = sizeof(parsI);
   ext[1].count = sizeof(parsI)/sizeof(parsI[0]);
   ext[1].bytes = sizeof(parsI[0]);
   ext[1].ptr = &parsI;

   return lg_command(sbc, LG_CMD_CSI, 2, ext, 1);
}

int lgu_get_sbc_name(int sbc, char *name, int count)
{
   int bytes;

   bytes = lg_command_0(sbc, LG_CMD_SBC, 0);

   if (bytes > 0)
   {
      bytes = recvMax(sbc, name, count, bytes);
      name[count-1] = 0; /* make sure null terminated */
   }

   _pmu(sbc);

   return bytes;
}

int lgu_set_user(int sbc, char *user, char *secretFile)
{
   lgExtent_t ext[1];
   int len;
   int bytes;
   char hash[34];
   char salt1[LG_SALT_LEN];
   char salt2[LG_SALT_LEN];
   char buf[64];

   hash[0] = 0;

   if (strlen(user) == 0) user = LG_DEFAULT_USER;

   snprintf(salt1, LG_SALT_LEN, "%015"PRIx64, xMakeSalt());
   sprintf(buf, "%s.%s", salt1, user);

   len = strlen(buf);

   ext[0].size = len;
   ext[0].count = len;
   ext[0].bytes = 1;
   ext[0].ptr = buf;

   bytes = lg_command(sbc, LG_CMD_USER, 1, ext, 0);

   if (bytes < 0) return bytes;

   recvMax(sbc, salt2, LG_SALT_LEN, bytes);

   _pmu(sbc);

   lgMd5UserHash(user, salt1, salt2, secretFile, hash);

   len = strlen(hash);

   ext[0].size = len;
   ext[0].count = len;
   ext[0].bytes = 1;
   ext[0].ptr = hash;

   return lg_command(sbc, LG_CMD_PASSW, 1, ext, 1);
}

int lgu_set_share_id(int sbc, int handle, int share_id)
   {return lg_command_2(sbc, LG_CMD_SHRS, handle, share_id, 1);}

int lgu_use_share_id(int sbc, int share_id)
   {return lg_command_1(sbc, LG_CMD_SHRU, share_id, 1);}

uint32_t lgu_rgpio_version(void)
   {return RGPIO_VERSION;}

const char *lgu_error_text(int errnum)
{
   if (errnum > -1000) return lguErrorText(errnum);
   else
   {
      switch(errnum)
      {
         case lgif_bad_send:
            return "failed to send to rgpiod";
         case lgif_bad_recv:
            return "failed to receive from rgpiod";
         case lgif_bad_getaddrinfo:
            return "failed to find address of rgpiod";
         case lgif_bad_connect:
            return "failed to connect to rgpiod";
         case lgif_bad_socket:
            return "failed to create socket";
         case lgif_bad_noib:
            return "failed to open notification in band";
         case lgif_duplicate_callback:
            return "identical callback exists";
         case lgif_bad_malloc:
            return "failed to malloc";
         case lgif_bad_callback:
            return "bad callback parameter";
         case lgif_notify_failed:
            return "failed to create notification thread";
         case lgif_callback_not_found:
            return "callback not found";
         case lgif_unconnected_sbc:
            return "not connected to sbc";
         case lgif_too_many_pis:
            return "too many connected sbcs";

         default:
            return "unknown error";
      }
   }
}

void lgu_sleep(double sleepSecs)
{
   struct timespec ts, rem;

   if (sleepSecs > 0.0)
   {
      ts.tv_sec = sleepSecs;
      ts.tv_nsec = (sleepSecs-(double)ts.tv_sec) * 1E9;

      while (clock_nanosleep(CLOCK_REALTIME, 0, &ts, &rem)) ts = rem;
   }
}

double lgu_time(void)
   {return (double)lgu_timestamp() / (double)1E9;}

uint64_t lgu_timestamp(void)
{
   struct timespec xts;

   clock_gettime(CLOCK_REALTIME, &xts); // get current time

   return ((uint64_t)1E9 * xts.tv_sec) + xts.tv_nsec;
}

