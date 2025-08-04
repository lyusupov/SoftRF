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

#include "lgpio.h"

typedef struct
{
   int error;
   char *str;
} xErrInfo_t;

static xErrInfo_t xErrInfo[]=
{
   {LG_OKAY,  "No error"},
   {LG_INIT_FAILED,  "initialisation failed"},
   {LG_BAD_MICROS,  "micros not 0-999999"},
   {LG_BAD_PATHNAME,  "can not open pathname"},
   {LG_NO_HANDLE,  "no handle available"},
   {LG_BAD_HANDLE,  "unknown handle"},
   {LG_BAD_SOCKET_PORT,  "socket port not 1024-32000"},
   {LG_NOT_PERMITTED,  "GPIO operation not permitted"},
   {LG_SOME_PERMITTED,  "one or more GPIO not permitted"},
   {LG_BAD_SCRIPT,  "invalid script"},
   {LG_BAD_TX_TYPE,  "bad tx type for GPIO and group"},
   {LG_GPIO_IN_USE,  "GPIO already in use"},
   {LG_BAD_PARAM_NUM,  "script parameter id not 0-9"},
   {LG_DUP_TAG,  "script has duplicate tag"},
   {LG_TOO_MANY_TAGS,  "script has too many tags"},
   {LG_BAD_SCRIPT_CMD,  "illegal script command"},
   {LG_BAD_VAR_NUM,  "script variable id not 0-149"},
   {LG_NO_SCRIPT_ROOM,  "no more room for scripts"},
   {LG_NO_MEMORY,  "can not allocate temporary memory"},
   {LG_SOCK_READ_FAILED,  "socket read failed"},
   {LG_SOCK_WRIT_FAILED,  "socket write failed"},
   {LG_TOO_MANY_PARAM,  "too many script parameters (> 10)"},
   {LG_SCRIPT_NOT_READY,  "script initialising"},
   {LG_BAD_TAG,  "script has unresolved tag"},
   {LG_BAD_MICS_DELAY,  "bad MICS delay (too large)"},
   {LG_BAD_MILS_DELAY,  "bad MILS delay (too large)"},
   {LG_I2C_OPEN_FAILED,  "can not open I2C device"},
   {LG_SERIAL_OPEN_FAILED,  "can not open serial device"},
   {LG_SPI_OPEN_FAILED,  "can not open SPI device"},
   {LG_BAD_I2C_BUS,  "bad I2C bus"},
   {LG_BAD_I2C_ADDR,  "bad I2C address"},
   {LG_BAD_SPI_CHANNEL,  "bad SPI channel"},
   {LG_BAD_I2C_FLAGS,  "bad I2C open flags"},
   {LG_BAD_SPI_FLAGS,  "bad SPI open flags"},
   {LG_BAD_SERIAL_FLAGS,  "bad serial open flags"},
   {LG_BAD_SPI_SPEED,  "bad SPI speed"},
   {LG_BAD_SERIAL_DEVICE,  "bad serial device name"},
   {LG_BAD_SERIAL_SPEED,  "bad serial baud rate"},
   {LG_BAD_FILE_PARAM,  "bad file parameter"},
   {LG_BAD_I2C_PARAM,  "bad I2C parameter"},
   {LG_BAD_SERIAL_PARAM,  "bad serial parameter"},
   {LG_I2C_WRITE_FAILED,  "i2c write failed"},
   {LG_I2C_READ_FAILED,  "i2c read failed"},
   {LG_BAD_SPI_COUNT,  "bad SPI count"},
   {LG_SERIAL_WRITE_FAILED,  "ser write failed"},
   {LG_SERIAL_READ_FAILED,  "ser read failed"},
   {LG_SERIAL_READ_NO_DATA,  "ser read no data available"},
   {LG_UNKNOWN_COMMAND,  "unknown command"},
   {LG_SPI_XFER_FAILED,  "spi xfer/read/write failed"},
   {LG_BAD_POINTER,  "bad (NULL) pointer"},
   {LG_MSG_TOOBIG,  "socket/pipe message too big"},
   {LG_BAD_MALLOC_MODE,  "bad memory allocation mode"},
   {LG_TOO_MANY_SEGS,  "too many I2C transaction segments"},
   {LG_BAD_I2C_SEG,  "an I2C transaction segment failed"},
   {LG_BAD_SMBUS_CMD,  "SMBus command not supported by driver"},
   {LG_BAD_I2C_WLEN,  "bad I2C write length"},
   {LG_BAD_I2C_RLEN,  "bad I2C read length"},
   {LG_BAD_I2C_CMD,  "bad I2C command"},
   {LG_FILE_OPEN_FAILED,  "file open failed"},
   {LG_BAD_FILE_MODE,  "bad file mode"},
   {LG_BAD_FILE_FLAG,  "bad file flag"},
   {LG_BAD_FILE_READ,  "bad file read"},
   {LG_BAD_FILE_WRITE,  "bad file write"},
   {LG_FILE_NOT_ROPEN,  "file not open for read"},
   {LG_FILE_NOT_WOPEN,  "file not open for write"},
   {LG_BAD_FILE_SEEK,  "bad file seek"},
   {LG_NO_FILE_MATCH,  "no files match pattern"},
   {LG_NO_FILE_ACCESS,  "no permission to access file"},
   {LG_FILE_IS_A_DIR,  "file is a directory"},
   {LG_BAD_SHELL_STATUS,  "bad shell return status"},
   {LG_BAD_SCRIPT_NAME,  "bad script name"},
   {LG_CMD_INTERRUPTED,  "Python socket command interrupted"},
   {LG_BAD_EVENT_REQUEST,  "bad event request"},
   {LG_BAD_GPIO_NUMBER,  "bad GPIO number"},
   {LG_BAD_GROUP_SIZE,  "bad group size"},
   {LG_BAD_LINEINFO_IOCTL,  "bad lineinfo IOCTL"},
   {LG_BAD_READ,  "bad GPIO read"},
   {LG_BAD_WRITE,  "bad GPIO write"},
   {LG_CANNOT_OPEN_CHIP,  "can not open gpiochip"},
   {LG_GPIO_BUSY,  "GPIO busy"},
   {LG_GPIO_NOT_ALLOCATED,  "GPIO not allocated"},
   {LG_NOT_A_GPIOCHIP,  "not a gpiochip"},
   {LG_NOT_ENOUGH_MEMORY,  "not enough memory"},
   {LG_POLL_FAILED,  "GPIO poll failed"},
   {LG_TOO_MANY_GPIOS,  "too many GPIO"},
   {LG_UNEGPECTED_ERROR,  "unexpected error"},
   {LG_BAD_PWM_MICROS,  "bad PWM micros"},
   {LG_NOT_GROUP_LEADER,  "GPIO not the group leader"},
   {LG_SPI_IOCTL_FAILED,  "SPI iOCTL failed"},
   {LG_BAD_GPIOCHIP,  "bad gpiochip"},
   {LG_BAD_CHIPINFO_IOCTL,  "bad chipinfo IOCTL"},
   {LG_BAD_CONFIG_FILE,  "bad configuration file"},
   {LG_BAD_CONFIG_VALUE,  "bad configuration value"},
   {LG_NO_PERMISSIONS,  "no permission to perform action"},
   {LG_BAD_USERNAME,  "bad user name"},
   {LG_BAD_SECRET,  "bad secret for user"},
   {LG_TX_QUEUE_FULL,  "TX queue full"},
   {LG_BAD_CONFIG_ID,  "bad configuration id"},
   {LG_BAD_DEBOUNCE_MICS,  "bad debounce microseconds"},
   {LG_BAD_WATCHDOG_MICS,  "bad watchdog microseconds"},
   {LG_BAD_SERVO_FREQ,  "bad servo frequency"},
   {LG_BAD_SERVO_WIDTH,  "bad servo pulsewidth"},
   {LG_BAD_PWM_FREQ,  "bad PWM frequency"},
   {LG_BAD_PWM_DUTY,  "bad PWM dutycycle"},
   {LG_GPIO_NOT_AN_OUTPUT,  "GPIO not set as an output"},
   {LG_INVALID_GROUP_ALERT,  "can not set a group to alert"},
};

const char *lguErrorText(int error)
{
   int i;

   for (i=0; i<(sizeof(xErrInfo)/sizeof(xErrInfo_t)); i++)
   {
      if (xErrInfo[i].error == error) return xErrInfo[i].str;
   }
   return "unknown error";
}

