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
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include "lgpio.h"

#include "lgDbg.h"
#include "lgHdl.h"

#define LG_I2C_SLAVE   0x0703
#define LG_I2C_FUNCS   0x0705
#define LG_I2C_RDWR    0x0707
#define LG_I2C_SMBUS   0x0720

#define LG_I2C_SMBUS_READ  1
#define LG_I2C_SMBUS_WRITE 0

#define LG_I2C_SMBUS_QUICK            0
#define LG_I2C_SMBUS_BYTE             1
#define LG_I2C_SMBUS_BYTE_DATA        2
#define LG_I2C_SMBUS_WORD_DATA        3
#define LG_I2C_SMBUS_PROC_CALL        4
#define LG_I2C_SMBUS_BLOCK_DATA       5
#define LG_I2C_SMBUS_I2C_BLOCK_BROKEN 6
#define LG_I2C_SMBUS_BLOCK_PROC_CALL  7
#define LG_I2C_SMBUS_I2C_BLOCK_DATA   8

#define LG_I2C_SMBUS_BLOCK_MAX     32
#define LG_I2C_SMBUS_I2C_BLOCK_MAX 32

#define LG_I2C_FUNC_SMBUS_QUICK            0x00010000
#define LG_I2C_FUNC_SMBUS_READ_BYTE        0x00020000
#define LG_I2C_FUNC_SMBUS_WRITE_BYTE       0x00040000
#define LG_I2C_FUNC_SMBUS_READ_BYTE_DATA   0x00080000
#define LG_I2C_FUNC_SMBUS_WRITE_BYTE_DATA  0x00100000
#define LG_I2C_FUNC_SMBUS_READ_WORD_DATA   0x00200000
#define LG_I2C_FUNC_SMBUS_WRITE_WORD_DATA  0x00400000
#define LG_I2C_FUNC_SMBUS_PROC_CALL        0x00800000
#define LG_I2C_FUNC_SMBUS_READ_BLOCK_DATA  0x01000000
#define LG_I2C_FUNC_SMBUS_WRITE_BLOCK_DATA 0x02000000
#define LG_I2C_FUNC_SMBUS_READ_I2C_BLOCK   0x04000000
#define LG_I2C_FUNC_SMBUS_WRITE_I2C_BLOCK  0x08000000

typedef struct
{
   uint16_t state;
   int16_t  fd;
   uint32_t addr;
   uint32_t flags;
   uint32_t funcs;
} lgI2cObj_t, *lgI2cObj_p;

union lgI2cSmbusData
{
   uint8_t  byte;
   uint16_t word;
   uint8_t  block[LG_I2C_SMBUS_BLOCK_MAX + 2];
};

struct lgI2cSmbusIoctlData
{
   uint8_t read_write;
   uint8_t command;
   uint32_t size;
   union lgI2cSmbusData *data;
};

typedef struct
{
   lgI2cMsg_t *msgs; /* pointers to pi_i2c_msgs */
   uint32_t     nmsgs; /* number of pi_i2c_msgs */
} lgI2cRdwrIoctlData_t;

static int xI2cGetPar(const char *inBuf, int *inPos, int inCount, int *esc)
{
   int bytes;

   if (*esc) bytes = 2; else bytes = 1;

   *esc = 0;

   if (*inPos <= (inCount - bytes))
   {
      if (bytes == 1)
      {
         return inBuf[(*inPos)++];
      }
      else
      {
         (*inPos) += 2;
         return inBuf[*inPos-2] + (inBuf[*inPos-1]<<8);
      }
   }
   return -1;
}

static void _lgI2cClose(lgI2cObj_p i2c)
{
   if (i2c) close(i2c->fd);
}

static int xI2cSmbusAccess(
   int fd, char rw, uint8_t cmd, int size, union lgI2cSmbusData *data)
{
   struct lgI2cSmbusIoctlData args;

   LG_DBG(LG_DEBUG_INTERNAL, "rw=%d reg=%d cmd=%d data=%s",
      rw, cmd, size, lgDbgBuf2Str(data->byte+1, (char*)data));

   args.read_write = rw;
   args.command    = cmd;
   args.size       = size;
   args.data       = data;

   return ioctl(fd, LG_I2C_SMBUS, &args);
}

int lgI2cWriteQuick(int handle, int bit)
{
   int status;
   lgI2cObj_p i2c;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d bit=%d", handle, bit);

   if ((unsigned)bit > 1)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad bit (%d)", bit);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      if (i2c->funcs & LG_I2C_FUNC_SMBUS_QUICK)
      {
         status = xI2cSmbusAccess(
            i2c->fd, bit, 0, LG_I2C_SMBUS_QUICK, NULL);

         if (status < 0)
         {
            LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
            status = LG_I2C_WRITE_FAILED;
         }
      }
      else
      {
         LG_DBG(LG_DEBUG_USER, "write quick not supported by driver");
         status = LG_BAD_SMBUS_CMD;
      }

      lgHdlUnlock(handle);
   }

   return status;
}

int lgI2cReadByte(int handle)
{
   union lgI2cSmbusData data;
   lgI2cObj_p i2c;
   int status;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d", handle);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      if (i2c->funcs & LG_I2C_FUNC_SMBUS_READ_BYTE)
      {
         status = xI2cSmbusAccess(
            i2c->fd, LG_I2C_SMBUS_READ, 0, LG_I2C_SMBUS_BYTE, &data);

         if (status < 0)
         {
            LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
            status = LG_I2C_READ_FAILED;
         }
         else status = 0xFF & data.byte;
      }
      else
      {
         LG_DBG(LG_DEBUG_USER, "read byte not supported by driver");
         status = LG_BAD_SMBUS_CMD;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


int lgI2cWriteByte(int handle, int bVal)
{
   int status;
   lgI2cObj_p i2c;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d bVal=%d", handle, bVal);

   if ((unsigned) bVal > 0xFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad bVal (%d)", bVal);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      if (i2c->funcs & LG_I2C_FUNC_SMBUS_WRITE_BYTE)
      {
         status = xI2cSmbusAccess(
            i2c->fd,
            LG_I2C_SMBUS_WRITE,
            bVal,
            LG_I2C_SMBUS_BYTE,
            NULL);

         if (status < 0)
         {
            LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
            status = LG_I2C_WRITE_FAILED;
         }
      }
      else
      {
         LG_DBG(LG_DEBUG_USER, "write byte not supported by driver");
         status = LG_BAD_SMBUS_CMD;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


int lgI2cReadByteData(int handle, int reg)
{
   union lgI2cSmbusData data;
   int status;
   lgI2cObj_p i2c;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d reg=%d", handle, reg);

   if ((unsigned)reg > 0xFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad reg (%d)", reg);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      if (i2c->funcs & LG_I2C_FUNC_SMBUS_READ_BYTE_DATA)
      {
         status = xI2cSmbusAccess(i2c->fd,
            LG_I2C_SMBUS_READ, reg, LG_I2C_SMBUS_BYTE_DATA, &data);

         if (status < 0)
         {
            LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
            status = LG_I2C_READ_FAILED;
         }
         else status = 0xFF & data.byte;
      }
      else
      {
         LG_DBG(LG_DEBUG_USER, "read byte data not supported by driver");
         status = LG_BAD_SMBUS_CMD;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


int lgI2cWriteByteData(int handle, int reg, int bVal)
{
   union lgI2cSmbusData data;
   lgI2cObj_p i2c;
   int status;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d reg=%d bVal=%d", handle, reg, bVal);

   if ((unsigned)reg > 0xFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad reg (%d)", reg);

   if ((unsigned)bVal > 0xFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad bVal (%d)", bVal);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      if (i2c->funcs & LG_I2C_FUNC_SMBUS_WRITE_BYTE_DATA)
      {
         data.byte = bVal;

         status = xI2cSmbusAccess(
            i2c->fd,
            LG_I2C_SMBUS_WRITE,
            reg,
            LG_I2C_SMBUS_BYTE_DATA,
            &data);

         if (status < 0)
         {
            LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
            status = LG_I2C_WRITE_FAILED;
         }
      }
      else
      {
         LG_DBG(LG_DEBUG_USER, "write byte data not supported by driver");
         status = LG_BAD_SMBUS_CMD;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


int lgI2cReadWordData(int handle, int reg)
{
   union lgI2cSmbusData data;
   int status;
   lgI2cObj_p i2c;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d reg=%d", handle, reg);

   if ((unsigned)reg > 0xFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad reg (%d)", reg);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      if (i2c->funcs & LG_I2C_FUNC_SMBUS_READ_WORD_DATA)
      {
         status = (xI2cSmbusAccess(
            i2c->fd,
            LG_I2C_SMBUS_READ,
            reg,
            LG_I2C_SMBUS_WORD_DATA,
            &data));

         if (status < 0)
         {
            LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
            status = LG_I2C_READ_FAILED;
         }
         else status = 0xFFFF & data.word;
      }
      else
      {
         LG_DBG(LG_DEBUG_USER, "read word data not supported by driver");
         status = LG_BAD_SMBUS_CMD;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


int lgI2cWriteWordData(int handle, int reg, int wVal)
{
   union lgI2cSmbusData data;

   int status;
   lgI2cObj_p i2c;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d reg=%d wVal=%d", handle, reg, wVal);

   if ((unsigned)reg > 0xFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad reg (%d)", reg);

   if ((unsigned)wVal > 0xFFFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad wVal (%d)", wVal);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      if (i2c->funcs & LG_I2C_FUNC_SMBUS_WRITE_WORD_DATA)
      {
         data.word = wVal;

         status = xI2cSmbusAccess(
            i2c->fd,
            LG_I2C_SMBUS_WRITE,
            reg,
            LG_I2C_SMBUS_WORD_DATA,
            &data);

         if (status < 0)
         {
            LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
            status = LG_I2C_WRITE_FAILED;
         }
      }
      else
      {
         LG_DBG(LG_DEBUG_USER, "SMBUS command not supported by driver");
         status = LG_BAD_SMBUS_CMD;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


int lgI2cProcessCall(int handle, int reg, int wVal)
{
   union lgI2cSmbusData data;
   int status;
   lgI2cObj_p i2c;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d reg=%d wVal=%d", handle, reg, wVal);

   if ((unsigned)reg > 0xFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad reg (%d)", reg);

   if ((unsigned)wVal > 0xFFFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad wVal (%d)", wVal);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      if (i2c->funcs & LG_I2C_FUNC_SMBUS_PROC_CALL)
      {
         data.word = wVal;

         status = (xI2cSmbusAccess(
            i2c->fd,
            LG_I2C_SMBUS_WRITE,
            reg, LG_I2C_SMBUS_PROC_CALL,
            &data));

         if (status < 0)
         {
            LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
            status = LG_I2C_READ_FAILED;
         }
         else status = 0xFFFF & data.word;
      }
      else
      {
         LG_DBG(LG_DEBUG_USER, "process call not supported by driver");
         status = LG_BAD_SMBUS_CMD;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


int lgI2cReadBlockData(int handle, int reg, char *rxBuf)
{
   union lgI2cSmbusData data;

   int status;
   int i;
   lgI2cObj_p i2c;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d reg=%d rxBuf=%p", handle, reg, rxBuf);

   if ((unsigned)reg > 0xFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad reg (%d)", reg);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      if (i2c->funcs & LG_I2C_FUNC_SMBUS_READ_BLOCK_DATA)
      {
         status = (xI2cSmbusAccess(
            i2c->fd,
            LG_I2C_SMBUS_READ,
            reg,
            LG_I2C_SMBUS_BLOCK_DATA,
            &data));

         if (status < 0)
         {
            LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
            status = LG_I2C_READ_FAILED;
         }
         else
         {
            if (data.block[0] <= LG_I2C_SMBUS_BLOCK_MAX)
            {
               for (i=0; i<data.block[0]; i++) rxBuf[i] = data.block[i+1];

               status = data.block[0];
            }
            else status = LG_I2C_READ_FAILED;
         }
      }
      else
      {
         LG_DBG(LG_DEBUG_USER, "read block data not supported by driver");
         status = LG_BAD_SMBUS_CMD;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


int lgI2cWriteBlockData(
   int handle, int reg, const char *txBuf, int count)
{
   union lgI2cSmbusData data;

   int status;
   int i;
   lgI2cObj_p i2c;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d reg=%d count=%d [%s]",
      handle, reg, count, lgDbgBuf2Str(count, txBuf));

   if ((unsigned)reg > 0xFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad reg (%d)", reg);

   if ((count < 1) || (count > 32))
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad count (%d)", count);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      if (i2c->funcs & LG_I2C_FUNC_SMBUS_WRITE_BLOCK_DATA)
      {
         for (i=1; i<=count; i++) data.block[i] = txBuf[i-1];

         data.block[0] = count;

         status = xI2cSmbusAccess(
            i2c->fd,
            LG_I2C_SMBUS_WRITE,
            reg,
            LG_I2C_SMBUS_BLOCK_DATA,
            &data);

         if (status < 0)
         {
            LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
            status = LG_I2C_WRITE_FAILED;
         }
      }
      else
      {
         LG_DBG(LG_DEBUG_USER, "write block data not supported by driver");
         status = LG_BAD_SMBUS_CMD;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


int lgI2cBlockProcessCall(
   int handle, int reg, char *buf, int count)
{
   union lgI2cSmbusData data;

   int i;
   int status;
   lgI2cObj_p i2c;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d reg=%d count=%d [%s]",
      handle, reg, count, lgDbgBuf2Str(count, buf));

   if ((unsigned)reg > 0xFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad reg (%d)", reg);

   if ((count < 1) || (count > 32))
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad count (%d)", count);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      if (i2c->funcs & LG_I2C_FUNC_SMBUS_PROC_CALL)
      {
         for (i=1; i<=count; i++) data.block[i] = buf[i-1];

         data.block[0] = count;

         status = xI2cSmbusAccess(
            i2c->fd, LG_I2C_SMBUS_WRITE, reg,
            LG_I2C_SMBUS_BLOCK_PROC_CALL, &data);

         if (status < 0)
         {
            LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
            status = LG_I2C_READ_FAILED;
         }
         else
         {
            if (data.block[0] <= LG_I2C_SMBUS_BLOCK_MAX)
            {
               for (i=0; i<data.block[0]; i++) buf[i] = data.block[i+1];

               status = data.block[0];
            }
            else status = LG_I2C_READ_FAILED;
         }
      }
      else
      {
         LG_DBG(LG_DEBUG_USER, "block process call not supported by driver");
         status = LG_BAD_SMBUS_CMD;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


int lgI2cReadI2CBlockData(
   int handle, int reg, char *rxBuf, int count)
{
   union lgI2cSmbusData data;
   lgI2cObj_p i2c;
   int status;
   int i;
   uint32_t size;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d reg=%d count=%d rxBuf=%p",
      handle, reg, count, rxBuf);

   if ((unsigned)reg > 0xFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad reg (%d)", reg);

   if ((count < 1) || (count > 32))
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad count (%d)", count);

   if (count == 32)
      size = LG_I2C_SMBUS_I2C_BLOCK_BROKEN;
   else
      size = LG_I2C_SMBUS_I2C_BLOCK_DATA;

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      if (i2c->funcs & LG_I2C_FUNC_SMBUS_READ_I2C_BLOCK)
      {
         data.block[0] = count;

         status = xI2cSmbusAccess(
            i2c->fd, LG_I2C_SMBUS_READ, reg, size, &data);

         if (status < 0)
         {
            LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
            status = LG_I2C_READ_FAILED;
         }
         else
         {
            if (data.block[0] <= LG_I2C_SMBUS_I2C_BLOCK_MAX)
            {
               for (i=0; i<data.block[0]; i++) rxBuf[i] = data.block[i+1];

               status = data.block[0];
            }
            else status = LG_I2C_READ_FAILED;
         }
      }
      else
      {
         LG_DBG(LG_DEBUG_USER, "read I2C block not supported by driver");
         status = LG_BAD_SMBUS_CMD;
      }

      lgHdlUnlock(handle);
   }

   return status;
}


int lgI2cWriteI2CBlockData(
   int handle, int reg, const char *txBuf, int count)
{
   union lgI2cSmbusData data;

   int i;
   int status;
   lgI2cObj_p i2c;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d reg=%d count=%d [%s]",
      handle, reg, count, lgDbgBuf2Str(count, txBuf));

   if (reg > 0xFF)
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad reg (%d)", reg);

   if ((count < 1) || (count > 32))
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad count (%d)", count);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      if (i2c->funcs & LG_I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)
      {
         for (i=1; i<=count; i++) data.block[i] = txBuf[i-1];

         data.block[0] = count;

         status = xI2cSmbusAccess(
            i2c->fd,
            LG_I2C_SMBUS_WRITE,
            reg,
            LG_I2C_SMBUS_I2C_BLOCK_BROKEN,
            &data);

         if (status < 0)
         {
            LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
            status = LG_I2C_WRITE_FAILED;
         }
      }
      else
      {
         LG_DBG(LG_DEBUG_USER, "write I2C block not supported by driver");
         status = LG_BAD_SMBUS_CMD;
      }

      lgHdlUnlock(handle);
   }

   return status;
}

int lgI2cWriteDevice(int handle, const char *txBuf, int count)
{
   int bytes;
   lgI2cObj_p i2c;
   int status;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d count=%d [%s]",
      handle, count, lgDbgBuf2Str(count, txBuf));

   if ((count < 1) || (count > LG_MAX_I2C_DEVICE_COUNT))
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad count (%d)", count);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      bytes = write(i2c->fd, txBuf, count);

      if (bytes != count)
      {
         LG_DBG(LG_DEBUG_USER, "error=%d (%m)", bytes);
         status = LG_I2C_WRITE_FAILED;
      }

      lgHdlUnlock(handle);
   }

   return status;
}

int lgI2cReadDevice(int handle, char *rxBuf, int count)
{
   lgI2cObj_p i2c;
   int status;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d count=%d rxBuf=%p",
      handle, count, rxBuf);

   if ((count < 1) || (count > LG_MAX_I2C_DEVICE_COUNT))
      PARAM_ERROR(LG_BAD_I2C_PARAM, "bad count (%d)", count);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      status = read(i2c->fd, rxBuf, count);

      if (status != count)
      {
         LG_DBG(LG_DEBUG_USER, "error=%d (%m)", status);
         status = LG_I2C_READ_FAILED;
      }

      lgHdlUnlock(handle);
   }

   return status;
}

int lgI2cOpen(int i2cDev, int i2cAddr, int i2cFlags)
{
   int handle;
   char dev[32];
   int fd;
   uint32_t funcs;
   lgI2cObj_p i2c;

   LG_DBG(LG_DEBUG_TRACE, "i2cDev=%d i2cAddr=%d flags=0x%X",
      i2cDev, i2cAddr, i2cFlags);

   if ((unsigned)i2cAddr > LG_MAX_I2C_ADDR)
      PARAM_ERROR(LG_BAD_I2C_ADDR, "bad I2C address (%d)", i2cAddr);

   if (i2cFlags)
      PARAM_ERROR(LG_BAD_I2C_FLAGS, "bad I2C flags (0x%X)", i2cFlags);

   sprintf(dev, "/dev/i2c-%d", i2cDev);

   if ((fd = open(dev, O_RDWR)) < 0)
   {
      return LG_BAD_I2C_BUS;
   }

   if (ioctl(fd, LG_I2C_SLAVE, i2cAddr) < 0)
   {
      close(fd);
      return LG_I2C_OPEN_FAILED;
   }

   if (ioctl(fd, LG_I2C_FUNCS, &funcs) < 0)
   {
      funcs = -1; /* assume all smbus commands allowed */
   }

   LG_DBG(LG_DEBUG_ALLOC, "alloc i2c: *%p", (void*)i2c);

   handle = lgHdlAlloc(
      LG_HDL_TYPE_I2C, sizeof(lgI2cObj_t), (void **)&i2c, _lgI2cClose);

   if (handle < 0)
   {
      close(fd);
      return LG_NO_MEMORY;
   }

   i2c->fd = fd;
   i2c->addr = i2cAddr;
   i2c->flags = i2cFlags;
   i2c->funcs = funcs;

   return handle;
}

int lgI2cClose(int handle)
{
   int status;
   lgI2cObj_p i2c;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d", handle);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      status = lgHdlFree(handle, LG_HDL_TYPE_I2C);

      lgHdlUnlock(handle);
   }

   return status;
}

int lgI2cSegments(
   int handle, lgI2cMsg_t *segs, int numSegs)
{
   lgI2cRdwrIoctlData_t rdwr;
   lgI2cObj_p i2c;
   int status;

   LG_DBG(LG_DEBUG_USER, "handle=%d", handle);

   if (segs == NULL)
      PARAM_ERROR(LG_BAD_POINTER, "null segments");

   if (numSegs > LG_I2C_RDRW_IOCTL_MAX_MSGS)
      PARAM_ERROR(LG_TOO_MANY_SEGS, "too many segments (%d)", numSegs);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      rdwr.msgs = segs;
      rdwr.nmsgs = numSegs;

      status = ioctl(i2c->fd, LG_I2C_RDWR, &rdwr);

      if (status < 0) status = LG_BAD_I2C_SEG;

      lgHdlUnlock(handle);
   }

   return status;
}

static int xSegments(int fd, lgI2cMsg_t *segs, int numSegs)
{
   lgI2cRdwrIoctlData_t rdwr;
   int status;

   rdwr.msgs = segs;
   rdwr.nmsgs = numSegs;

   status = ioctl(fd, LG_I2C_RDWR, &rdwr);

   if (status < 0) status = LG_BAD_I2C_SEG;

   return status;
}


int lgI2cZip(
   int handle, const char *inBuf, int inCount, char *outBuf, int outCount)
{
   int numSegs, inPos, outPos, bytes, flags, addr;
   int esc, setesc;
   lgI2cMsg_t segs[LG_I2C_RDRW_IOCTL_MAX_MSGS];
   lgI2cObj_p i2c;
   int status;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d inBuf=%s outBuf=%p len=%d",
      handle, lgDbgBuf2Str(inCount, inBuf), outBuf, outCount);

   if (!inBuf || (inCount<1))
      PARAM_ERROR(LG_BAD_POINTER, "input buffer can't be NULL");

   if (!outBuf && outCount)
      PARAM_ERROR(LG_BAD_POINTER, "output buffer can't be NULL");

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_I2C, (void **)&i2c);

   if (status == LG_OKAY)
   {
      numSegs = 0;

      inPos = 0;
      outPos = 0;
      status = 0;

      addr = i2c->addr;
      flags = 0;
      esc = 0;
      setesc = 0;

      while (!status && (inPos < inCount))
      {
         LG_DBG(LG_DEBUG_INTERNAL,
            "status=%d inpos=%d inlen=%d cmd=%d addr=%d flags=%x",
            status, inPos, inCount, inBuf[inPos], addr, flags);

         switch (inBuf[inPos++])
         {
            case LG_I2C_END:
               status = 1;
               break;

            case LG_I2C_ADDR:
               addr = xI2cGetPar(inBuf, &inPos, inCount, &esc);
               if (addr < 0) status = LG_BAD_I2C_CMD;
               break;

            case LG_I2C_FLAGS:
               /* cheat to force two byte flags */
               esc = 1;
               flags = xI2cGetPar(inBuf, &inPos, inCount, &esc);
               if (flags < 0) status = LG_BAD_I2C_CMD;
               break;

            case LG_I2C_ESC:
               setesc = 1;
               break;

            case LG_I2C_READ:
               bytes = xI2cGetPar(inBuf, &inPos, inCount, &esc);

               if (bytes >= 0)
               {
                  if ((bytes + outPos) < outCount)
                  {
                     segs[numSegs].addr = addr;
                     segs[numSegs].flags = (flags|1);
                     segs[numSegs].len = bytes;
                     segs[numSegs].buf = (uint8_t *)(outBuf + outPos);
                     outPos += bytes;
                     numSegs++;
                     if (numSegs >= LG_I2C_RDRW_IOCTL_MAX_MSGS)
                     {
                        status = xSegments(i2c->fd, segs, numSegs);
                        if (status >= 0) status = 0; /* continue */
                        numSegs = 0;
                     }
                  }
                  else status = LG_BAD_I2C_RLEN;
               }
               else status = LG_BAD_I2C_RLEN;
               break;

            case LG_I2C_WRITE:

               bytes = xI2cGetPar(inBuf, &inPos, inCount, &esc);

               if (bytes >= 0)
               {
                  if ((bytes + inPos) < inCount)
                  {
                     segs[numSegs].addr = addr;
                     segs[numSegs].flags = (flags&0xfffe);
                     segs[numSegs].len = bytes;
                     segs[numSegs].buf = (uint8_t *)(inBuf + inPos);
                     inPos += bytes;
                     numSegs++;
                     if (numSegs >= LG_I2C_RDRW_IOCTL_MAX_MSGS)
                     {
                        status = xSegments(i2c->fd, segs, numSegs);
                        if (status >= 0) status = 0; /* continue */
                        numSegs = 0;
                     }
                  }
                  else status = LG_BAD_I2C_WLEN;
               }
               else status = LG_BAD_I2C_WLEN;
               break;

            default:
               status = LG_BAD_I2C_CMD;
         }

         if (setesc) esc = 1; else esc = 0;

         setesc = 0;
      }

      if (status >= 0)
      {
         if (numSegs) status = xSegments(i2c->fd, segs, numSegs);
      }

      if (status >= 0) status = outPos;

      lgHdlUnlock(handle);
   }

   return status;
}

