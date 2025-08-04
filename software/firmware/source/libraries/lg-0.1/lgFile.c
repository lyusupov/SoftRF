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
#include <glob.h>
#include <string.h>
#include <fnmatch.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>


#include "lgpio.h"
#include "rgpiod.h"

#include "lgDbg.h"
#include "lgHdl.h"

typedef struct
{
   int16_t  fd;
   uint32_t mode;
} lgFileObj_t, *lgFileObj_p;

static void _fileClose(lgFileObj_p file)
{
   if (file) close(file->fd);
}

int lgFileOpen(char *file, int mode)
{
   int fd=-1;
   int handle, oflag, omode;
   struct stat statbuf;
   lgFileObj_p h;

   LG_DBG(LG_DEBUG_TRACE, "file=%s mode=%d", file, mode);

   if ( (mode < LG_FILE_MIN) ||
        (mode > LG_FILE_MAX) ||
        ((mode & LG_FILE_RW) == 0) )
      PARAM_ERROR(LG_BAD_FILE_MODE, "bad mode (%d)", mode);

   if ((mode > 3) && ((mode & LG_FILE_WRITE) == 0))
      PARAM_ERROR(LG_NO_FILE_ACCESS, "no permission to write file (%s)", file);

   omode = 0;
   oflag = 0;

   if (mode & LG_FILE_APPEND)
   {
      oflag |= O_APPEND;
   }

   if (mode & LG_FILE_CREATE)
   {
      oflag |= O_CREAT;
      omode |= (S_IRUSR|S_IWUSR);
   }

   if (mode & LG_FILE_TRUNC)
   {
      oflag |= O_TRUNC;
   }

   switch(mode&LG_FILE_RW)
   {
      case LG_FILE_READ:
         fd = open(file, O_RDONLY|oflag, omode);
         break;

      case LG_FILE_WRITE:
         fd = open(file, O_WRONLY|oflag, omode);
         break;

      case LG_FILE_RW:
         fd = open(file, O_RDWR|oflag, omode);
         break;
   }

   if (fd == -1)
   {
      return LG_FILE_OPEN_FAILED;
   }
   else
   {
      if (stat(file, &statbuf) == 0)
      {
         if (S_ISDIR(statbuf.st_mode))
         {
            close(fd);
            PARAM_ERROR(LG_FILE_IS_A_DIR, "file is a directory (%s)", file);
         }
      }
   }

   handle = lgHdlAlloc(
      LG_HDL_TYPE_FILE, sizeof(lgFileObj_t), (void**)&h, _fileClose);

   if (handle < 0)
   {
      close(fd);
      return LG_NO_MEMORY;
   }
   
   h->fd = fd;
   h->mode = mode;

   return handle;
}

int lgFileClose(int handle)
{
   int status;
   lgFileObj_p h;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d", handle);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_FILE, (void **)&h);

   if (status == LG_OKAY)
   {
      status = lgHdlFree(handle, LG_HDL_TYPE_FILE);

      lgHdlUnlock(handle);
   }

   return status;
}

int lgFileWrite(int handle, char *buf, int count)
{
   int w;
   int status;
   lgFileObj_p h;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d count=%d [%s]",
      handle, count, lgDbgBuf2Str(count, buf));

   if (!count)
      PARAM_ERROR(LG_BAD_FILE_PARAM, "bad count (%d)", count);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_FILE, (void **)&h);

   if (status == LG_OKAY)
   {
      if (h->mode & LG_FILE_WRITE)
      {
         w = write(h->fd, buf, count);

         if (w != count)
         {
            if (w == -1)
            {
               LG_DBG(LG_DEBUG_FILE, "write failed with errno %d", errno);
            }
            status = LG_BAD_FILE_WRITE;
         }
      }
      else
      {
         LG_DBG(LG_DEBUG_FILE, "file not opened for write");
         status = LG_FILE_NOT_WOPEN;
      }

      lgHdlUnlock(handle);
   }

   return status;
}

int lgFileRead(int handle, char *buf, int count)
{
   int status;
   lgFileObj_p h;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d count=%d buf=%p", handle, count, buf);

   if (!count)
      PARAM_ERROR(LG_BAD_FILE_PARAM, "bad count (%d)", count);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_FILE, (void **)&h);

   if (status == LG_OKAY)
   {
      if (h->mode & LG_FILE_READ)
      {
         status = read(h->fd, buf, count);

         if (status == -1)
         {
            LG_DBG(LG_DEBUG_FILE, "read failed with errno %d", errno);
            status = LG_BAD_FILE_READ;
         }
         else
         {
            buf[status] = 0;
         }
      }
      else
      {
         LG_DBG(LG_DEBUG_FILE, "file not opened for read");
         status = LG_FILE_NOT_ROPEN;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


int lgFileSeek(int handle, int32_t seekOffset, int seekFrom)
{
   int whence;
   int status;
   lgFileObj_p h;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d offset=%d from=%d",
      handle, seekOffset, seekFrom);

   switch (seekFrom)
   {
      case LG_FROM_START:
         whence = SEEK_SET;
         break;

      case LG_FROM_CURRENT:
         whence = SEEK_CUR;
         break;

      case LG_FROM_END:
         whence = SEEK_END;
         break;

      default:
         PARAM_ERROR(LG_BAD_FILE_SEEK, "bad seek from (%d)", seekFrom);
   }

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_FILE, (void **)&h);

   if (status == LG_OKAY)
   {
      status = lseek(h->fd, seekOffset, whence);

      if (status == -1)
      {
         LG_DBG(LG_DEBUG_FILE, "seek failed with errno %d", errno);
         status = LG_BAD_FILE_SEEK;
      }

      lgHdlUnlock(handle);
   }

   return status;
}

int lgFileList(char *fpat,  char *buf, int count)
{
   int len, bufpos;
   glob_t pglob;
   int i;

   LG_DBG(LG_DEBUG_TRACE, "fpat=%s count=%d buf=%p", fpat, count, buf);

   bufpos = 0;

   if (glob(fpat, GLOB_MARK, NULL, &pglob) == 0)
   {
      for (i=0; i<pglob.gl_pathc; i++)
      {
         len = strlen(pglob.gl_pathv[i]);
         if ((bufpos + len + 1) < count)
         {
            strcpy(buf+bufpos, pglob.gl_pathv[i]);
            bufpos += len;
            buf[bufpos++] = '\n';
         }
      }
   }
   else
   {
      bufpos = LG_NO_FILE_MATCH;
   }

   globfree(&pglob);

   return bufpos;
}

