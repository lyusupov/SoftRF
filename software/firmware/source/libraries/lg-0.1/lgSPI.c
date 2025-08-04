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
#include <linux/spi/spidev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "lgpio.h"

#include "lgDbg.h"
#include "lgHdl.h"

typedef struct
{
   int speed;
   int fd;
   uint32_t flags;
} lgSpiObj_t, *lgSpiObj_p;

static int xSpiXfer(
   int fd, int speed, const char *txBuf, char *rxBuf, int count)
{
   int err;
   struct spi_ioc_transfer spi;

   memset(&spi, 0, sizeof(spi));

   spi.tx_buf        = (uintptr_t)txBuf;
   spi.rx_buf        = (uintptr_t)rxBuf;
   spi.len           = count;
   spi.speed_hz      = speed;
   spi.delay_usecs   = 0;
   spi.bits_per_word = 8;
   spi.cs_change     = 0;

   err = ioctl(fd, SPI_IOC_MESSAGE(1), &spi);

   return err;
}

static void _lgSpiClose(lgSpiObj_p spi)
{
   if (spi) close(spi->fd);
}


int lgSpiOpen(
   int spiDev, int spiChan, int baud, int spiFlags)
{
   int handle;
   lgSpiObj_p spi;
   int fd;
   char  spiMode;
   char  spiBits  = 8;
   char dev[128];
   
   LG_DBG(LG_DEBUG_TRACE, "spiDev=%d spiChan=%d baud=%d spiFlags=0x%X",
      spiDev, spiChan, baud, spiFlags);

   spiMode  = spiFlags & 3;
   spiBits  = 8;

   sprintf(dev, "/dev/spidev%d.%d", spiDev, spiChan);

   if ((fd = open(dev, O_RDWR)) < 0)
   {
      return LG_SPI_OPEN_FAILED;
   }

   if (ioctl(fd, SPI_IOC_WR_MODE, &spiMode) < 0)
   {
      close(fd);
      return LG_SPI_IOCTL_FAILED;
   }

   if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spiBits) < 0)
   {
      close(fd);
      return LG_SPI_IOCTL_FAILED;
   }

   if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &baud) < 0)
   {
      close(fd);
      return LG_SPI_IOCTL_FAILED;
   }

   handle = lgHdlAlloc(
      LG_HDL_TYPE_SPI, sizeof(lgSpiObj_t), (void **)&spi, _lgSpiClose);

   if (handle < 0)
   {
      PARAM_ERROR(LG_NO_HANDLE, "no free handles");
   }

   spi->fd = fd;
   spi->speed = baud;
   spi->flags = spiFlags;

   return handle;
}

int lgSpiClose(int handle)
{
   int status;
   lgSpiObj_p spi;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d", handle);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SPI, (void **)&spi);

   if (status == LG_OKAY)
   {
      status = lgHdlFree(handle, LG_HDL_TYPE_SPI);
      lgHdlUnlock(handle);
   }

   return status;
}

int lgSpiRead(int handle, char *rxBuf, int count)
{
   int status;
   lgSpiObj_p spi;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d count=%d [%s]",
      handle, count, lgDbgBuf2Str(count, rxBuf));

   if (((unsigned)count > LG_MAX_SPI_DEVICE_COUNT) || !count)
      PARAM_ERROR(LG_BAD_SPI_COUNT, "bad count (%d)", count);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SPI, (void **)&spi);

   if (status == LG_OKAY)
   {
      status = xSpiXfer(spi->fd, spi->speed, NULL, rxBuf, count);

      lgHdlUnlock(handle);
   }

   return status;
}

int lgSpiWrite(int handle, const char *txBuf, int count)
{
   int status;
   lgSpiObj_p spi;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d count=%d [%s]",
      handle, count, lgDbgBuf2Str(count, txBuf));

   if (((unsigned)count > LG_MAX_SPI_DEVICE_COUNT) || !count)
      PARAM_ERROR(LG_BAD_SPI_COUNT, "bad count (%d)", count);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SPI, (void **)&spi);

   if (status == LG_OKAY)
   {
      status = xSpiXfer(spi->fd, spi->speed, txBuf, NULL, count);

      lgHdlUnlock(handle);
   }

   return status;
}

int lgSpiXfer(int handle, const char *txBuf, char *rxBuf, int count)
{
   int status;
   lgSpiObj_p spi;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d count=%d [%s]",
      handle, count, lgDbgBuf2Str(count, txBuf));

   if (((unsigned)count > LG_MAX_SPI_DEVICE_COUNT) || !count)
      PARAM_ERROR(LG_BAD_SPI_COUNT, "bad count (%d)", count);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SPI, (void **)&spi);

   if (status == LG_OKAY)
   {
      status = xSpiXfer(spi->fd, spi->speed, txBuf, rxBuf, count);

      lgHdlUnlock(handle);
   }

   return status;
}

