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
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

#include "lgpio.h"

#include "lgDbg.h"
#include "lgHdl.h"

typedef struct
{
   int16_t  fd;
   uint32_t flags;
} lgSerialObj_t, *lgSerialObj_p;

static void _lgSerialClose(lgSerialObj_p ser)
{
   if (ser) close(ser->fd);
}

int lgSerialOpen(const char *serDev, int serBaud, int serFlags)
{
   struct termios new;
   int speed;
   int fd;
   int handle;
   char serName[LG_MAX_PATH];
   lgSerialObj_p ser;

   LG_DBG(LG_DEBUG_TRACE, "serDev=%s serBaud=%d serFlags=0x%X",
      serDev, serBaud, serFlags);

   switch (serBaud)
   {
      case     50: speed =     B50; break;
      case     75: speed =     B75; break;
      case    110: speed =    B110; break;
      case    134: speed =    B134; break;
      case    150: speed =    B150; break;
      case    200: speed =    B200; break;
      case    300: speed =    B300; break;
      case    600: speed =    B600; break;
      case   1200: speed =   B1200; break;
      case   1800: speed =   B1800; break;
      case   2400: speed =   B2400; break;
      case   4800: speed =   B4800; break;
      case   9600: speed =   B9600; break;
      case  19200: speed =  B19200; break;
      case  38400: speed =  B38400; break;
      case  57600: speed =  B57600; break;
      case 115200: speed = B115200; break;
      case 230400: speed = B230400; break;

      default:
         PARAM_ERROR(LG_BAD_SERIAL_SPEED, "bad speed (%d)", serBaud);
   }

   if (serFlags)
      PARAM_ERROR(LG_BAD_SERIAL_FLAGS, "bad serial flags (0x%X)", serFlags);

   snprintf(serName, LG_MAX_PATH, "/dev/%s", serDev);

   if ((fd = open(serName, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
   {
      return LG_SERIAL_OPEN_FAILED;
   }

   handle = lgHdlAlloc(LG_HDL_TYPE_SERIAL, sizeof(lgSerialObj_t),
               (void**)&ser, _lgSerialClose);

   if (handle < 0)
   {
      close(fd);
      return LG_NO_MEMORY;
   }

   tcgetattr(fd, &new);

   cfmakeraw(&new);

   cfsetispeed(&new, speed);
   cfsetospeed(&new, speed);

   new.c_cc [VMIN]  = 0;
   new.c_cc [VTIME] = 0;

   tcflush(fd, TCIFLUSH);
   tcsetattr(fd, TCSANOW, &new);

   //fcntl(fd, F_SETFL, O_RDWR);

   ser->fd = fd;
   ser->flags = serFlags;

   return handle;
}

int lgSerialClose(int handle)
{
   int status;
   lgSerialObj_p ser;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d", handle);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SERIAL, (void **)&ser);

   if (status == LG_OKAY)
   {
      status = lgHdlFree(handle, LG_HDL_TYPE_SERIAL);

      lgHdlUnlock(handle);
   }

   return status;
}


int lgSerialWriteByte(int handle, int bVal)
{
   char c;
   lgSerialObj_p ser;
   int status;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d bVal=%d", handle, bVal);

   if ((unsigned)bVal > 0xFF)
      PARAM_ERROR(LG_BAD_SERIAL_PARAM, "bad parameter (%d)", bVal);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SERIAL, (void **)&ser);

   if (status == LG_OKAY)
   {
      c = bVal;

      if (write(ser->fd, &c, 1) != 1) status = LG_SERIAL_WRITE_FAILED;

      lgHdlUnlock(handle);
   }

   return status;
}

int lgSerialReadByte(int handle)
{
   char x;
   int r;
   lgSerialObj_p ser;
   int status;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d", handle);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SERIAL, (void **)&ser);

   if (status == LG_OKAY)
   {
      r = read(ser->fd, &x, 1);

      if (r == 1)
         status = ((int)x) & 0xFF;
      else if (r == 0)
         status = LG_SERIAL_READ_NO_DATA;
      else if ((r == -1) && (errno == EAGAIN))
         status = LG_SERIAL_READ_NO_DATA;
      else
         status = LG_SERIAL_READ_FAILED;

      lgHdlUnlock(handle);
   }

   return status;
}

int lgSerialWrite(int handle, const char *txBuf, int count)
{
   int written=0, wrote=0;
   lgSerialObj_p ser;
   int status;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d count=%d [%s]",
      handle, count, lgDbgBuf2Str(count, txBuf));

   if (!count)
      PARAM_ERROR(LG_BAD_SERIAL_PARAM, "bad count (%d)", count);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SERIAL, (void **)&ser);

   if (status == LG_OKAY)
   {
      while ((written != count) && (wrote >= 0))
      {
         wrote = write(ser->fd, txBuf+written, count-written);

         if (wrote >= 0)
         {
            written += wrote; 

            if (written != count) usleep(50000);
         }
      }

      if (written != count) status = LG_SERIAL_WRITE_FAILED;

      lgHdlUnlock(handle);
   }

   return status;
}

int lgSerialRead(int handle, char *rxBuf, int count)
{
   int r;
   lgSerialObj_p ser;
   int status;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d count=%d rxBuf=%p", handle, count, rxBuf);

   if (!count)
      PARAM_ERROR(LG_BAD_SERIAL_PARAM, "bad count (%d)", count);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SERIAL, (void **)&ser);

   if (status == LG_OKAY)
   {
      r = read(ser->fd, rxBuf, count);

      if (r == -1)
      {
         if (errno == EAGAIN)
            status = LG_SERIAL_READ_NO_DATA;
         else
            status = LG_SERIAL_READ_FAILED;
      }
      else
      {
         if (r < count) rxBuf[r] = 0;
         status = r;
      }

      lgHdlUnlock(handle);
   }

   return status;
}

int lgSerialDataAvailable(int handle)
{
   int status;
   lgSerialObj_p ser;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d", handle);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SERIAL, (void **)&ser);

   if (status == LG_OKAY)
   {
      if (ioctl(ser->fd, FIONREAD, &status) == -1) status = 0;

      lgHdlUnlock(handle);
   }

   return status;
}

