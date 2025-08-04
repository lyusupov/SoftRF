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

#define _GNU_SOURCE /* needed for pipes */

#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <limits.h>

#include "lgpio.h"

#include "lgDbg.h"
#include "lgHdl.h"

static void xCreatePipe(const char *name, int perm)
{
   unlink(name);

   mkfifo(name, perm);

   if (chmod(name, perm) < 0)
   {
      LG_DBG(LG_DEBUG_ALWAYS,
         "Can't set permissions (%d) for %s, %m", perm, name);
      return;
   }
}


static void _notifyClose(lgNotify_t *h)
{
   char fifo[128];

   LG_DBG(LG_DEBUG_INTERNAL, "fd=%d pipe_no=%d objp=*%p",
      h->fd, h->pipe_number, h);

   if (h->fd >= 0) close(h->fd);
   
   if (h->pipe_number)
   {
      LG_DBG(LG_DEBUG_INTERNAL, "close notify pipe %d", h->fd);
      snprintf(fifo, sizeof(fifo), "/dev/lg%d", h->pipe_number-1);
      unlink(fifo);
   }
}

/* ----------------------------------------------------------------------- */

void lgNotifyCloseOrphans(int slot, int fd)
{
   static int *handles;
   static int maxHandles = 20;
   int i, numHandles;
   lgNotify_t *h;
   int status;

   /* Check for and close any orphaned notifications. */

   if (handles == NULL) handles = malloc(sizeof(int) * maxHandles);

   numHandles = lgHdlGetHandlesForType(LG_HDL_TYPE_NOTIFY, handles, maxHandles);
   
   if (numHandles > maxHandles) numHandles = maxHandles;

   for (i=0; i<numHandles; i++)
   {
      status = lgHdlGetLockedObjTrusted(
         handles[i], LG_HDL_TYPE_NOTIFY, (void **)&h);
   
      if (status < 0) continue;
      
      if ((handles[i] != slot) &&
          (status >= 0) &&
          (h->state >= LG_NOTIFY_RUNNING) &&
          (h->fd == fd))
      {
         LG_DBG(LG_DEBUG_USER, "closed orphaned fd=%d (handle=%d)",
            fd, handles[i]);
         lgHdlFree(handles[i], LG_HDL_TYPE_NOTIFY);
      }

      lgHdlUnlock(handles[i]);
   }
}

/* ----------------------------------------------------------------------- */

int lgNotifyOpenWithSize(int bufSize)
{
   int i, fd;
   char name[LG_MAX_PATH];
   lgNotify_t *h;
   int handle;

   LG_DBG(LG_DEBUG_INTERNAL, "bufSize=%d", bufSize);

   handle = lgHdlAlloc(
      LG_HDL_TYPE_NOTIFY, sizeof(lgNotify_t), (void**)&h, _notifyClose);

   if (handle < 0) {return LG_NO_MEMORY;}

   snprintf(name, sizeof(name), "%s/.lgd-nfy%d", lguGetWorkDir(), handle);

   xCreatePipe(name, 0664);

   fd = open(name, O_RDWR|O_NONBLOCK);

   h->fd = fd;
   h->pipe_number = handle+1; // 0 is reserved for no pipe

   if (fd < 0)
   {
      lgHdlFree(handle, LG_HDL_TYPE_NOTIFY);
      PARAM_ERROR(LG_BAD_PATHNAME, "open %s failed (%m)", name);
   }

   if (bufSize != 0)
   {
      i = fcntl(fd, F_SETPIPE_SZ, bufSize);
      if (i != bufSize)
      {
         lgHdlFree(handle, LG_HDL_TYPE_NOTIFY);
         PARAM_ERROR(LG_BAD_PATHNAME,
            "fcntl %s size %d failed (%m)", name, bufSize);
      }
   }

   h->max_emits  = MAX_EMITS;
   h->state = LG_NOTIFY_RUNNING;

   lgNotifyCloseOrphans(handle, fd);

   return handle;
}

int lgNotifyOpen(void)
{
   LG_DBG(LG_DEBUG_TRACE, "");

   return lgNotifyOpenWithSize(0);
}

/* ----------------------------------------------------------------------- */

int lgNotifyOpenInBand(int fd)
{
   int handle;
   lgNotify_t *h;

   LG_DBG(LG_DEBUG_TRACE, "fd=%d", fd);

   handle = lgHdlAlloc(
      LG_HDL_TYPE_NOTIFY, sizeof(lgNotify_t), (void**)&h, _notifyClose);

   if (handle < 0) {return LG_NO_MEMORY;}

   h->fd = fd;
   h->pipe_number = 0;
   h->max_emits = MAX_EMITS;
   h->state = LG_NOTIFY_RUNNING;

   //lgNotifyCloseOrphans(handle, fd);

   return handle;
}


/* ----------------------------------------------------------------------- */

int lgNotifyResume(int handle)
{
   int status;
   lgNotify_t *h;
   
   LG_DBG(LG_DEBUG_TRACE, "handle=%d", handle);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_NOTIFY, (void **)&h);

   if (status == LG_OKAY)
   {
      if (h->state > LG_NOTIFY_CLOSING) h->state = LG_NOTIFY_RUNNING;
      else
      {
         LG_DBG(LG_DEBUG_USER, "bad handle (%d)", handle);
         status = LG_BAD_HANDLE;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


/* ----------------------------------------------------------------------- */

int lgNotifyPause (int handle)
{
   int status;
   lgNotify_t *h;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d", handle);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_NOTIFY, (void **)&h);

   if (status == LG_OKAY)
   {
      if (h->state > LG_NOTIFY_CLOSING) h->state = LG_NOTIFY_PAUSED;
      else
      {
         LG_DBG(LG_DEBUG_USER, "bad handle (%d)", handle);
         status = LG_BAD_HANDLE;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


/* ----------------------------------------------------------------------- */

int lgNotifyClose(int handle)
{
   int status;
   lgNotify_t *h;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d", handle);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_NOTIFY, (void **)&h);

   if (status == LG_OKAY)
   {
      if (h->state > LG_NOTIFY_CLOSING) h->state = LG_NOTIFY_CLOSING;
      else
      {
         LG_DBG(LG_DEBUG_USER, "bad handle (%d)", handle);
         status = LG_BAD_HANDLE;
      }

      /* actual close done in alert thread */
      status = lgHdlFree(handle, LG_HDL_TYPE_NOTIFY);

      lgHdlUnlock(handle);
   }

   return status;
}


