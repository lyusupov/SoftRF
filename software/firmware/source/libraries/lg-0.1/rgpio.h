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

#ifndef RGPIO_H
#define RGPIO_H

#include <inttypes.h>
#include <pthread.h>

#include "lgpio.h"

#define RGPIO_VERSION 0x00010000

/*TEXT

rgpio is a C library which allows remote control of the GPIO and
other functions of Linux SBCs running the rgpiod daemon.

The rgpiod daemon must be running on the SBCs you wish to control.

*Features*

o reading and writing GPIO singly and in groups
o software timed PWM and waves
o GPIO callbacks
o pipe notification of GPIO alerts
o I2C wrapper
o SPI wrapper
o serial link wrapper
o simple file handling
o creating and running scripts on the rgpiod daemon
o a simple interface to start and stop new threads

*Usage*

Include <rgpio.h> in your source files.

Assuming your source is in prog.c use the following command to build

. .
gcc -Wall -o prog prog.c -lrgpio
. .

to run make sure the rgpiod daemon is running

. .
rgpiod&

 ./prog
. .

For examples see the lg archive file.

*Notes*

All the functions which return an int return < 0 on error

TEXT*/

/*OVERVIEW

ESSENTIAL

rgpiod_start               Connects to a rgpiod daemon
rgpiod_stop                Disconnects from a rgpiod daemon

FILES

file_open                  Opens a file
file_close                 Closes a file

file_read                  Reads bytes from a file
file_write                 Writes bytes to a file

file_seek                  Seeks to a position within a file

file_list                  List files which match a pattern

GPIO

gpiochip_open              Opens a gpiochip device
gpiochip_close             Closes a gpiochip device

gpio_get_chip_info         Gets gpiochip information
gpio_get_line_info         Gets gpiochip line information
gpio_get_mode              Gets the mode of a GPIO

gpio_claim_input           Claims a GPIO for input
gpio_claim_output          Claims a GPIO for output
gpio_claim_alert           Claims a GPIO for alerts
gpio_free                  Frees a GPIO

group_claim_input          Claims a group of GPIO for inputs
group_claim_output         Claims a group of GPIO for outputs
group_free                 Frees a group of GPIO

gpio_read                  Reads a GPIO
gpio_write                 Writes a GPIO

group_read                 Reads a group of GPIO
group_write                Writes a group of GPIO

tx_pulse                   Starts pulses on a GPIO
tx_pwm                     Starts PWM on a GPIO
tx_servo                   Starts servo pulses on a GPIO.
tx_wave                    Starts a wave on a group of GPIO
tx_busy                    See if tx is active on a GPIO or group
tx_room                    See if more room for tx on a GPIO or group

gpio_set_debounce_time     Sets the debounce time for a GPIO
gpio_set_watchdog_time     Sets the watchdog time for a GPIO

callback                   Starts a GPIO callback
callback_cancel            Stops a GPIO callback

I2C

i2c_open                   Opens an I2C device
i2c_close                  Closes an I2C device

i2c_write_quick            smbus write quick

i2c_read_byte              smbus read byte
i2c_write_byte             smbus write byte

i2c_read_byte_data         smbus read byte data
i2c_write_byte_data        smbus write byte data

i2c_read_word_data         smbus read word data
i2c_write_word_data        smbus write word data

i2c_read_block_data        smbus read block data
i2c_write_block_data       smbus write block data

i2c_read_i2c_block_data    smbus read I2C block data
i2c_write_i2c_block_data   smbus write I2C block data

i2c_read_device            Reads the raw I2C device
i2c_write_device           Writes the raw I2C device

i2c_process_call           smbus process call
i2c_block_process_call     smbus block process call

i2c_zip                    Performs multiple I2C transactions

NOTIFICATIONS

notify_open                Request a notification handle
notify_close               Close a notification
notify_pause               Pause notifications
notify_resume              Start notifications for selected GPIO

SCRIPTS

script_store               Store a script
script_run                 Run a stored script
script_update              Set a scripts parameters
script_status              Get script status and parameters
script_stop                Stop a running script
script_delete              Delete a stored script

SERIAL

serial_open                Opens a serial device
serial_close               Closes a serial device

serial_read_byte           Reads a byte from a serial device
serial_write_byte          Writes a byte to a serial device

serial_read                Reads bytes from a serial device
serial_write               Writes bytes to a serial device

serial_data_available      Returns number of bytes ready to be read

SHELL

shell                      Executes a shell command

SPI

spi_open                   Opens a SPI device
spi_close                  Closes a SPI device

spi_read                   Reads bytes from a SPI device
spi_write                  Writes bytes to a SPI device
spi_xfer                   Transfers bytes with a SPI device

THREADS

thread_start               Start a new thread
thread_stop                Stop a previously started thread

UTILITIES

lgu_get_sbc_name           Get the SBC name

lgu_get_internal           Get a SBC configuration value
lgu_set_internal           Set a SBC configuration value

lgu_time                   Returns the number of seconds since the epoch
lgu_timestamp              Returns the number of nanoseconds since the epoch

lgu_sleep                  Sleeps for a number of seconds

lgu_set_user               Set the user (and associated permissions)

lgu_set_share_id           Set the share id for a resource
lgu_use_share_id           Use this share id when asking for a resource

lgu_rgpio_version          Get the rgpio library version
lgu_error_text             Get the error text for an error code

OVERVIEW*/


#ifdef __cplusplus
extern "C" {
#endif

#define RISING_EDGE   1
#define FALLING_EDGE  2
#define BOTH_EDGES   3

typedef void (*CBFunc_t)
   (int sbc, int chip, int gpio,
   int level, uint64_t tick, void *userdata);

typedef struct callback_s callback_t;

typedef void *(lgThreadFunc_t) (void *);

/* --------------------------------------------------------- ESSENTIAL API
*/

/*F*/
int rgpiod_start(const char *addrStr, const char *portStr);
/*D
Connect to the rgpiod daemon.  Reserving command and
notification streams.

. .
addrStr: specifies the host or IP address of the SBC running the
         rgpiod daemon.  It may be NULL in which case localhost
         is used unless overridden by the LG_ADDR environment
         variable.

portStr: specifies the port address used by the SBC running the
         rgpiod daemon.  It may be NULL in which case "8889"
         is used unless overridden by the LG_PORT environment
         variable.
. .

If OK returns a sbc (>= 0).

On failure returns a negative error code.

This sbc value is passed to the other functions to specify
the SBC to be operated on.
D*/

/*F*/
void rgpiod_stop(int sbc);
/*D
Terminates the connection to a rgpiod daemon and frees
resources used by the library.

. .
sbc: >= 0 (as returned by [*rgpiod_start*]).
. .
D*/


/* -------------------------------------------------------------- FILE API
*/

#pragma GCC diagnostic push

#pragma GCC diagnostic ignored "-Wcomment"

/*F*/
int file_open(int sbc, const char *file, int mode);
/*D
This function returns a handle to a file opened in a specified mode.

This is a privileged command.  See [+permits+].

. .
 sbc: >= 0 (as returned by [*rgpiod_start*]).
file: the file to open.
mode: the file open mode.
. .

If OK returns a handle (>= 0).

On failure returns a negative error code.

File

A file may only be opened if permission is granted by an entry in
the [files] section of the permits file.  This is intended to allow
remote access to files in a controlled manner.

Mode

The mode may have the following values.

Macro         @ Value @ Meaning
LG_FILE_READ  @   1   @ open file for reading
LG_FILE_WRITE @   2   @ open file for writing
LG_FILE_RW    @   3   @ open file for reading and writing

The following values may be or'd into the mode.

Macro          @ Value @ Meaning
LG_FILE_APPEND @ 4     @ Writes append data to the end of the file
LG_FILE_CREATE @ 8     @ The file is created if it doesn't exist
LG_FILE_TRUNC  @ 16    @ The file is truncated

Newly created files are owned by the user who launched the daemon
with permissions owner read and write.

...
#include <stdio.h>
#include <rgpio.h>

int main(int argc, char *argv[])
{
   int sbc, handle, c;
   char buf[60000];

   sbc = rgpiod_start(NULL, NULL);

   if (sbc < 0) return 1;

   handle = file_open(sbc, "/ram/lg.c", LG_FILE_READ);

   if (handle >= 0)
   {
      while ((c=file_read(sbc, handle, buf, sizeof(buf)-1)))
      {
         buf[c] = 0;
         printf("%s", buf);
      }

      file_close(sbc, handle);
   }

   rgpiod_stop(sbc);
}
...
D*/

#pragma GCC diagnostic pop

/*F*/
int file_close(int sbc, int handle);
/*D
This function closes the file.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*file_open*]).
. .

If OK returns 0.

On failure returns a negative error code.

...
file_close(sbc, handle);
...
D*/


/*F*/
int file_write(int sbc, int handle, const char *buf, int count);
/*D
This function writes count bytes from buf to the the file.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*file_open*]).
   buf: the array of bytes to write.
 count: the number of bytes to write.
. .

If OK returns 0.

On failure returns a negative error code.

...
if (file_write(sbc, handle, buf, 100) == 0)
{
   // file written okay
}
else
{
   // error
}
...
D*/


/*F*/
int file_read(int sbc, int handle, char *buf, int count);
/*D
This function reads up to count bytes from the the file.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*file_open*]).
   buf: an array to receive the read data.
 count: the maximum number of bytes to read.
. .

If OK returns the count of bytes read and updates buf.

On failure returns a negative error code.

...
   bytes = file_read(sbc, handle, buf, sizeof(buf));

   if (bytes >= 0)
   {
      // process read data
   }
...
D*/


/*F*/
int file_seek(int sbc, int handle, int32_t seekOffset, int seekFrom);
/*D
This function seeks to a position within the file.

. .
       sbc: >= 0 (as returned by [*rgpiod_start*]).
    handle: >= 0 (as returned by [*file_open*]).
seekOffset: the number of bytes to move.  Positive offsets
            move forward, negative offsets backwards.
  seekFrom: one of LG_FROM_START (0), LG_FROM_CURRENT (1),
            or LG_FROM_END (2).
. .

If OK returns the new file position.

On failure returns a negative error code.

...
file_seek(sbc, handle, 123, LG_FROM_START); // Start plus 123

size = file_seek(sbc, handle, 0, LG_FROM_END); // End, return size

pos = file_seek(sbc, handle, 0, LG_FROM_CURRENT); // Current position
...
D*/

#pragma GCC diagnostic push

#pragma GCC diagnostic ignored "-Wcomment"

/*F*/
int file_list(int sbc, const char *fpat,  char *buf, int count);
/*D
This function returns a list of files which match a pattern.

. .
  sbc: >= 0 (as returned by [*rgpiod_start*]).
 fpat: file pattern to match.
  buf: an array to receive the matching file names.
count: the maximum number of bytes to read.
. .

If OK returns the count of bytes read and updates buf with
the matching filenames (the filenames are separated by newline
characters).

On failure returns a negative error code.

...
#include <stdio.h>
#include <rgpio.h>

int main(int argc, char *argv[])
{
   int sbc, handle, c;
   char buf[60000];

   sbc = rgpiod_start(NULL, NULL);

   if (sbc < 0) return 1;

   c = file_list(sbc, "/ram/p*.c", buf, sizeof(buf));

   if (c >= 0)
   {
      buf[c] = 0;
      printf("%s", buf);
   }

   rgpiod_stop(sbc);
}
...
D*/

#pragma GCC diagnostic pop

/* -------------------------------------------------------------- GPIO API
*/

/*F*/
int gpiochip_open(int sbc, int gpioDev);
/*D
This returns a handle to a gpiochip device.

This is a privileged command.  See [+permits+].

. .
    sbc: >= 0 (as returned by [*rgpiod_start*]).
gpioDev: >= 0
. .

If OK returns a handle (>= 0).

On failure returns a negative error code.

...
h = gpiochip_open(sbc, 0); // open gpiochip0

if (h >= 0)
{
   // open ok
}
else
{
   // open error
}
...

D*/

/*F*/
int gpiochip_close(int sbc, int handle);
/*D
This closes a gpiochip device.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*gpiochip_open*]).
. .

If OK returns 0.

On failure returns a negative error code.

...
status = gpiochip_close(sbc, h); // close gpiochip

if (status < 0)
{
   // close failed
}
...
D*/

/*F*/
int gpio_get_chip_info(int sbc, int handle, lgChipInfo_p chipInfo);
/*D
This returns summary information of an opened gpiochip.

. .
     sbc: >= 0 (as returned by [*rgpiod_start*]).
  handle: >= 0 (as returned by [*gpiochip_open*]).
chipInfo: address to store returned chip info.
. .

If OK returns a list of okay status, number of
lines, name, and label.

On failure returns a negative error code.
D*/

/*F*/
int gpio_get_line_info(int sbc, int handle, int gpio, lgLineInfo_p lineInfo);
/*D
This returns detailed information of a GPIO of
an opened gpiochip.

. .
     sbc: >= 0 (as returned by [*rgpiod_start*]).
  handle: >= 0 (as returned by [*gpiochip_open*]).
    gpio: the GPIO.
lineInfo: address to store returned line info.
. .

If OK returns a list of okay status, offset,
line flags, name, and user.

On failure returns a negative error code.
D*/

/*F*/
int gpio_get_mode(int sbc, int handle, int gpio);
/*D
This returns the mode of a GPIO.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*gpiochip_open*]).
  gpio: the GPIO to be read.
. .

If OK returns the mode of the GPIO.

On failure returns a negative error code.

Mode bit @ Value @ Meaning
0        @  1    @ Kernel: In use by the kernel
1        @  2    @ Kernel: Output
2        @  4    @ Kernel: Active low
3        @  8    @ Kernel: Open drain
4        @ 16    @ Kernel: Open source
5        @ 32    @ Kernel: ---
6        @ 64    @ Kernel: ---
7        @ 128   @ Kernel: ---
8        @ 256   @ LG: Input
9        @ 512   @ LG: Output
10       @ 1024  @ LG: Alert
11       @ 2048  @ LG: Group
12       @ 4096  @ LG: ---
13       @ 8192  @ LG: ---
14       @ 16384 @ LG: ---
15       @ 32768 @ LG: ---
D*/

/*F*/
int gpio_claim_input(int sbc, int handle, int lFlags, int gpio);
/*D
This claims a GPIO for input.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*gpiochip_open*]).
lFlags: line flags for the GPIO.
  gpio: the GPIO to be claimed.
. .

If OK returns 0.

On failure returns a negative error code.

The line flags may be used to set the GPIO
as active low, open drain, or open source.

...
status = gpio_claim_input(sbc, h, 0, 23); // open GPIO 23 for input
...
D*/


/*F*/
int gpio_claim_output(
   int sbc, int handle, int lFlags, int gpio, int value);
/*D
This claims a GPIO for output.
. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*gpiochip_open*]).
lFlags: line flags for the GPIO.
  gpio: the GPIO to be claimed.
 value: the initial value for the GPIO.
. .

If OK returns 0.

On failure returns a negative error code.

The line flags may be used to set the GPIO
as active low, open drain, or open source.

If value is zero the GPIO will be initialised low (0).  If any other
value is used the GPIO will be initialised high (1).

...
status = gpio_claim_output(sbc, h, 0, 35, 1); // open GPIO 35 for high output
...
D*/


/*F*/
int gpio_free(int sbc, int handle, int gpio);
/*D
This frees a GPIO.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*gpiochip_open*]).
  gpio: the GPIO to be freed.
. .

If OK returns 0.

On failure returns a negative error code.

The GPIO may now be claimed by another user or for a different purpose.
D*/


/*F*/
int group_claim_input(
   int sbc, int handle, int lFlags, int count, const int *gpios);
/*D
This claims a group of GPIO for inputs.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*gpiochip_open*]).
lFlags: line flags for each GPIO.
 count: the number of GPIO to claim.
 gpios: the group GPIO.
. .

If OK returns 0.

On failure returns a negative error code.

The line flags may be used to set the group as active low,
open drain, or open source.

gpios is an array of one or more GPIO. The first GPIO in the array
is called the group leader and is used to reference the group as a whole.
D*/

/*F*/
int group_claim_output(
   int sbc, int handle, int lFlags,
   int count, const int *gpios, const int *values);
/*D
This claims a group of GPIO to be used as outputs.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*gpiochip_open*]).
lFlags: line flags for each GPIO.
 count: the number of GPIO to claim.
 gpios: the group GPIO.
values: the initial value for each GPIO.
. .

If OK returns 0.

On failure returns a negative error code.

The line flags may be used to set the group as active low, open drain, or open source.

gpios is an array of one or more GPIO. The first GPIO in the array is
called the group leader and is used to reference the group as a whole.

values is a list of initialisation values for the GPIO. If a value
is zero the corresponding GPIO will be initialised low (0).
If any other value is used the corresponding GPIO will be
initialised high (1).
D*/

/*F*/
int group_free(int sbc, int handle, int gpio);
/*D
This frees all the group GPIO.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*gpiochip_open*]).
  gpio: the group leader.
. .

If OK returns 0.

On failure returns a negative error code.

The GPIO may now be claimed by another user or for a different purpose.
D*/

/*F*/
int gpio_read(int sbc, int handle, int gpio);
/*D
This returns the level of a GPIO.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*gpiochip_open*]).
  gpio: the GPIO to be read.
. .

If OK returns 0 (low) or 1 (high).

On failure returns a negative error code.

This command will work for any claimed GPIO (even if a member
of a group).  For an output GPIO the value returned
will be that last written to the GPIO.
D*/

/*F*/
int gpio_write(int sbc, int handle, int gpio, int value);
/*D
This sets the level of an output GPIO.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*gpiochip_open*]).
  gpio: the GPIO to be written.
 value: the value to write.
. .

If OK returns 0.

On failure returns a negative error code.

This command will work for any GPIO claimed as an output (even if
a member of a group).

If level is zero the GPIO will be set low (0). If any other value
is used the GPIO will be set high (1).
D*/

/*F*/
int group_read(
   int sbc, int handle, int gpio, uint64_t *groupBits);
/*D
This returns the levels read from a group.

. .
      sbc: >= 0 (as returned by [*rgpiod_start*]).
   handle: >= 0 (as returned by [*gpiochip_open*]).
     gpio: the offset of a member of the GPIO group to be read.
groupBits: a pointer to a 64-bit memory area for the returned value.
. .

If OK returns the group size and updates groupBits.

On failure returns a negative error code.

This command will work for an output group as well as an input group. For an output group the value returned will be that last written to the group GPIO.

Note that this command will also work on an individual GPIO claimed as an input or output as that is treated as a group with one member.

After a successful read groupBits is set as follows.

Bit 0 is the level of the group leader.
Bit 1 is the level of the second GPIO in the group.
Bit x is the level of GPIO x+1 of the group.
D*/

/*F*/
int group_write(
   int sbc, int handle, int gpio, uint64_t groupBits, uint64_t groupMask);
/*D
This sets the levels of an output output group.

. .
      sbc: >= 0 (as returned by [*rgpiod_start*]).
   handle: >= 0 (as returned by [*gpiochip_open*]).
     gpio: the offset of a member of the GPIO group to be written.
groupBits: the level to set if the corresponding bit in groupMask is set.
groupMask: a mask indicating the group GPIO to be updated.
. .

If OK returns 0.

On failure returns a negative error code.

The values of each GPIO of the group are set according to the bits
of group_bits.

Bit 0 sets the level of the group leader.
Bit 1 sets the level of the second GPIO in the group.
Bit x sets the level of GPIO x+1 in the group.

However this may be overridden by the group_mask. A GPIO is only
updated if the corresponding bit in the mask is 1.
D*/

/*F*/
int tx_pulse(
   int sbc, int handle, int gpio,
   int pulse_on, int pulse_off,
   int pulse_offset, int pulse_cycles);
/*D
This starts software timed pulses on an output GPIO.

. .
         sbc: >= 0 (as returned by [*rgpiod_start*]).
      handle: >= 0 (as returned by [*gpiochip_open*]).
        gpio:  GPIO to be written.
    pulse_on: pulse high time in microseconds.
   pulse_off: pulse low time in microseconds.
pulse_offset: offset from nominal pulse start position.
pulse_cycles: the number of pulses to be sent, 0 for infinite.
. .

If OK returns the number of entries left in the PWM queue for the GPIO.

On failure returns a negative error code.

If both pulse_on and pulse_off are zero pulses will be switched off
for that GPIO.  The active pulse, if any, will be stopped and any
queued pulses will be deleted.

Each successful call to this function consumes one PWM queue entry.

pulse_cycles cycles are transmitted (0 means infinite).
Each cycle consists of pulse_on microseconds of GPIO high
followed by pulse_off microseconds of GPIO low.

PWM is characterised by two values, its frequency
(number of cycles per second) and its duty cycle
(percentage of high time per cycle).

The set frequency will be 1000000 / (pulse_on + pulse_off) Hz.

The set duty cycle will be pulse_on / (pulse_on + pulse_off) * 100 %.

E.g. if pulse_on is 50 and pulse_off is 100 the frequency will
be 6666.67 Hz and the duty cycle will be 33.33 %.

pulse_offset is a microsecond offset from the natural start
of the pulse cycle.

For instance if the PWM frequency is 10 Hz the natural start of
each cycle is at seconds 0, then 0.1, 0.2, 0.3 etc. In this case
if the offset is 20000 microseconds the cycle will start at
seconds 0.02, 0.12, 0.22, 0.32 etc.

Another pulse command may be issued to the GPIO before the last
has finished.

If the last pulse had infinite cycles then it will be replaced by
the new settings at the end of the current cycle. Otherwise it will
be replaced by the new settings when all its cycles are compete.

Multiple pulse settings may be queued in this way.
D*/

/*F*/
int tx_pwm(
   int sbc,
   int handle,
   int gpio,
   float pwmFrequency,
   float pwmDutyCycle,
   int pwmOffset,
   int pwmCycles);
/*D
This starts software timed PWM on an output GPIO.

. .
         sbc: >= 0 (as returned by [*rgpiod_start*]).
      handle: >= 0 (as returned by [*gpiochip_open*])
        gpio: the GPIO to be pulsed
pwmFrequency: PWM frequency in Hz (0=off, 0.1-10000)
pwmDutyCycle: PWM duty cycle in % (0-100)
   pwmOffset: offset from nominal pulse start position
   pwmCycles: the number of pulses to be sent, 0 for infinite
. .

If OK returns the number of entries left in the PWM queue for the GPIO.

On failure returns a negative error code.

Each successful call to this function consumes one PWM queue entry.

PWM is characterised by two values, its frequency (number of cycles
per second) and its duty cycle (percentage of high time per cycle).

Another PWM command may be issued to the GPIO before the last has finished.

If the last pulse had infinite cycles then it will be replaced by
the new settings at the end of the current cycle. Otherwise it will
be replaced by the new settings when all its cycles are compete.

Multiple PWM settings may be queued in this way.
D*/

/*F*/
int tx_servo(
   int sbc,
   int handle,
   int gpio,
   int pulseWidth,
   int servoFrequency,
   int servoOffset,
   int servoCycles);
/*D
This starts software timed servo pulses on an output GPIO.

I would only use software timed servo pulses for testing purposes.  The
timing jitter will cause the servo to fidget.  This may cause it to
overheat and wear out prematurely.

. .
        handle: >= 0 (as returned by [*gpiochip_open*])
          gpio: the GPIO to be pulsed
    pulseWidth: pulse high time in microseconds (0=0ff, 500-2500)
servoFrequency: the number of pulses per second (40-500)
   servoOffset: offset from nominal pulse start position
   servoCycles: the number of pulses to be sent, 0 for infinite
. .

If OK returns the number of entries left in the PWM queue for the GPIO.

On failure returns a negative error code.

Each successful call to this function consumes one PWM queue entry.

Another servo command may be issued to the GPIO before the last
has finished.

If the last pulse had infinite cycles then it will be replaced by
the new settings at the end of the current cycle. Otherwise it will
be replaced by the new settings when all its cycles are complete.

Multiple servo settings may be queued in this way.
D*/


/*F*/
int tx_wave(
   int sbc, int handle, int gpio, int count, lgPulse_p pulses);
/*D
This starts a software timed wave on an output group.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*gpiochip_open*]).
  gpio:  group leader.
 count: the number of pulses in the wave.
pulses: the pulses.
. .

If OK returns the number of entries left in the wave queue for the group.

On failure returns a negative error code.

Each successful call to this function consumes one wave queue entry.

This command starts a wave of pulses.

pulses is an array of pulses to be transmitted on the group.

Each pulse is defined by the following triplet:

bits: the levels to set for the selected GPIO 
mask: the GPIO to select 
delay: the delay in microseconds before the next pulse

Another wave command may be issued to the group before the
last has finished transmission. The new wave will start
when the previous wave has competed.

Multiple waves may be queued in this way.
D*/

/*F*/
int tx_busy(int sbc, int handle, int gpio, int kind);
/*D
This returns true if transmissions of the specified kind
are active on the GPIO or group.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*gpiochip_open*]).
  gpio: the GPIO or group to be tested.
  kind: LG_TX_PWM or LG_TX_WAVE.
. .

If OK returns 1 for busy and 0 for not busy.

On failure returns a negative error code.

D*/

/*F*/
int tx_room(int sbc, int handle, int gpio, int kind);
/*D
This returns the number of entries there are to queue further
transmissions of the specified kind on a GPIO or GPIO group.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*gpiochip_open*]).
  gpio: the GPIO or group to be tested.
  kind: LG_TX_PWM or LG_TX_WAVE.
. .

If OK returns the number of free entries (0 for none).

On failure returns a negative error code.

D*/

/*F*/
int gpio_set_debounce_time(int sbc, int handle, int gpio, int debounce_us);
/*D
This sets the debounce time for a GPIO.

. .
        sbc: >= 0 (as returned by [*rgpiod_start*]).
     handle: >= 0 (as returned by [*gpiochip_open*]).
       gpio: the GPIO to be configured.
debounce_us: the debounce time in microseconds.
. .

If OK returns 0.

On failure returns a negative error code.

This only affects alerts.

An alert will only be issued if the edge has been stable for at
least debounce microseconds.

Generally this is used to debounce mechanical switches
(e.g. contact bounce).

Suppose that a square wave at 5 Hz is being generated on a GPIO.
Each edge will last 100000 microseconds. If a debounce time
of 100001 is set no alerts will be generated, If a debounce time
of 99999 is set 10 alerts will be generated per second.

Note that level changes will be timestamped debounce microseconds
after the actual level change.
D*/

/*F*/
int gpio_set_watchdog_time(int sbc, int handle, int gpio, int watchdog_us);
/*D
This sets the watchdog time for a GPIO.

. .
        sbc: >= 0 (as returned by [*rgpiod_start*]).
     handle: >= 0 (as returned by [*gpiochip_open*]).
       gpio: the GPIO to be configured.
watchdog_us: the watchdog time in microseconds.
. .

If OK returns 0.

On failure returns a negative error code.

This only affects alerts.

A watchdog alert will be sent if no edge alert has been issued
for that GPIO in the previous watchdog microseconds.

Note that only one watchdog alert will be sent per stream of
edge alerts.  The watchdog is reset by the sending of a new
edge alert.

The level is set to LG_TIMEOUT (2) for a watchdog alert.
D*/


/*F*/
int gpio_claim_alert(
   int sbc, int handle, int lFlags, int eFlags, int gpio, int nfyHandle);
/*D
This claims a GPIO to be used as a source of alerts on level changes.

. .
      sbc: >= 0 (as returned by [*rgpiod_start*]).
   handle: >= 0 (as returned by [*gpiochip_open*]).
     gpio: >= 0, as legal for the gpiochip.  
   lFlags: line flags for the GPIO.
   eFlags: event flags for the GPIO.
nfyHandle: >=0, a notification handle (use -1 for callbacks).
. .

If OK returns 0.

On failure returns a negative error code.


The line flags may be used to set the GPIO as active low,
open drain, or open source.

The event flags are used to generate alerts for a rising edge,
falling edge, or both edges.

Use a notification handle of -1 unless you plan to read the alerts
from a notification pipe you have opened.
D*/

/*F*/
int callback
   (int sbc, int handle, int gpio, int edge, CBFunc_t f, void *userdata);
/*D
This function initialises a new callback.

. .
     sbc: >= 0 (as returned by [*rgpiod_start*]).
  handle: >= 0,(as returned by [*gpiochip_open*]).
    gpio: >= 0, as legal for the gpiochip.
    edge: RISING_EDGE, FALLING_EDGE, or BOTH_EDGES.
       f: the callback function.
userdata: a pointer to arbitrary user data.
. .

If OK returns a callback id.

On failure returns a negative error code.

The user supplied callback receives the chip, GPIO, edge, timestamp,
and the userdata pointer, whenever the GPIO has the identified edge.

The reported level will be one of

0: change to low (a falling edge)
1: change to high (a rising edge)
2: no level change (a watchdog timeout)

The timestamp is when the change happened reported as the
number of nanoseconds since the epoch (start of 1970).

If you want to track the level of more than one GPIO do so by
maintaining the state in the callback.  Do not use [*gpio_read*].
Remember the alert that triggered the callback may have
happened several milliseconds before and the GPIO may have
changed level many times since then.
D*/

/*F*/
int callback_cancel(int callback_id);
/*D
This function cancels a callback identified by its id.

. .
callback_id: >= 0 (as returned by [*callback*]).
. .

If OK returns 0.

On failure returns a negative error code.

D*/


/* --------------------------------------------------------------- I2C API
*/

/*F*/
int i2c_open(int sbc, int i2c_bus, int i2c_addr, int i2c_flags);
/*D
This returns a handle for the device at address i2c_addr on bus i2c_bus.

This is a privileged command.  See [+permits+].

. .
      sbc: >= 0 (as returned by [*rgpiod_start*]).
  i2c_bus: >= 0.
 i2c_addr: 0-0x7F.
i2c_flags: 0.
. .

If OK returns a handle (>= 0).

On failure returns a negative error code.

No flags are currently defined.  This parameter should be set to zero.

For the SMBus commands the low level transactions are shown at the end
of the function description.  The following abbreviations are used.

. .
S       (1 bit) : Start bit
P       (1 bit) : Stop bit
Rd/Wr   (1 bit) : Read/Write bit. Rd equals 1, Wr equals 0.
A, NA   (1 bit) : Accept and not accept bit. 
Addr    (7 bits): I2C 7 bit address.
i2c_reg (8 bits): A byte which often selects a register.
Data    (8 bits): A data byte.
Count   (8 bits): A byte defining the length of a block operation.

[..]: Data sent by the device.
. .
D*/

/*F*/
int i2c_close(int sbc, int handle);
/*D
This closes the I2C device

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*i2c_open*]).
. .

If OK returns 0.

On failure returns a negative error code.
D*/

/*F*/
int i2c_write_quick(int sbc, int handle, int bitVal);
/*D
This sends a single bit (in the Rd/Wr bit) to the device.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*i2c_open*]).
bitVal: 0-1, the value to write.
. .

If OK returns 0.

On failure returns a negative error code.

Quick command. SMBus 2.0 5.5.1
. .
S Addr bit [A] P
. .
D*/

/*F*/
int i2c_write_byte(int sbc, int handle, int byteVal);
/*D
This sends a single byte to the device.

. .
    sbc: >= 0 (as returned by [*rgpiod_start*]).
 handle: >= 0 (as returned by [*i2c_open*]).
byteVal: 0-0xFF, the value to write.
. .

If OK returns 0.

On failure returns a negative error code.

Send byte. SMBus 2.0 5.5.2
. .
S Addr Wr [A] byteVal [A] P
. .
D*/

/*F*/
int i2c_read_byte(int sbc, int handle);
/*D
This reads a single byte from the device.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*i2c_open*]).
. .

If OK returns the byte read (0-255).

On failure returns a negative error code.

Receive byte. SMBus 2.0 5.5.3
. .
S Addr Rd [A] [Data] NA P
. .
D*/

/*F*/
int i2c_write_byte_data(
   int sbc, int handle, int i2c_reg, int byteVal);
/*D
This writes a single byte to the specified register of the device.

. .
    sbc: >= 0 (as returned by [*rgpiod_start*]).
 handle: >= 0 (as returned by [*i2c_open*]).
i2c_reg: 0-255, the register to write.
byteVal: 0-0xFF, the value to write.
. .

If OK returns 0.

On failure returns a negative error code.

Write byte. SMBus 2.0 5.5.4
. .
S Addr Wr [A] i2c_reg [A] byteVal [A] P
. .
D*/

/*F*/
int i2c_write_word_data(
   int sbc, int handle, int i2c_reg, int wordVal);
/*D
This writes a single 16 bit word to the specified register of the device.

. .
    sbc: >= 0 (as returned by [*rgpiod_start*]).
 handle: >= 0 (as returned by [*i2c_open*]).
i2c_reg: 0-255, the register to write.
wordVal: 0-0xFFFF, the value to write.
. .

If OK returns 0.

On failure returns a negative error code.

Write word. SMBus 2.0 5.5.4
. .
S Addr Wr [A] i2c_reg [A] wval_Low [A] wVal_High [A] P
. .
D*/

/*F*/
int i2c_read_byte_data(int sbc, int handle, int i2c_reg);
/*D
This reads a single byte from the specified register of the device.

. .
    sbc: >= 0 (as returned by [*rgpiod_start*]).
 handle: >= 0 (as returned by [*i2c_open*]).
i2c_reg: 0-255, the register to read.
. .

If OK returns the read byte (0-255).

On failure returns a negative error code.

Read byte. SMBus 2.0 5.5.5
. .
S Addr Wr [A] i2c_reg [A] S Addr Rd [A] [Data] NA P
. .
D*/

/*F*/
int i2c_read_word_data(int sbc, int handle, int i2c_reg);
/*D
This reads a single 16 bit word from the specified register of the device.

. .
    sbc: >= 0 (as returned by [*rgpiod_start*]).
 handle: >= 0 (as returned by [*i2c_open*]).
i2c_reg: 0-255, the register to read.
. .

If OK returns the read word (0-65535).

On failure returns a negative error code.

Read word. SMBus 2.0 5.5.5
. .
S Addr Wr [A] i2c_reg [A]
   S Addr Rd [A] [DataLow] A [DataHigh] NA P
. .
D*/

/*F*/
int i2c_process_call(int sbc, int handle, int i2c_reg, int wordVal);
/*D
This writes 16 bits of data to the specified register of the device
and reads 16 bits of data in return.

. .
    sbc: >= 0 (as returned by [*rgpiod_start*]).
 handle: >= 0 (as returned by [*i2c_open*]).
i2c_reg: 0-255, the register to write/read.
wordVal: 0-0xFFFF, the value to write.
. .

If OK returns the read word (0-65535).

On failure returns a negative error code.

Process call. SMBus 2.0 5.5.6
. .
S Addr Wr [A] i2c_reg [A] wVal_Low [A] wVal_High [A]
   S Addr Rd [A] [DataLow] A [DataHigh] NA P
. .
D*/

/*F*/
int i2c_write_block_data(
   int sbc, int handle, int i2c_reg, const char *buf, int count);
/*D
This writes up to 32 bytes to the specified register of the device.

. .
    sbc: >= 0 (as returned by [*rgpiod_start*]).
 handle: >= 0 (as returned by [*i2c_open*]).
i2c_reg: 0-255, the register to write.
    buf: an array with the data to send.
  count: 1-32, the number of bytes to write.
. .

If OK returns 0.

On failure returns a negative error code.

Block write. SMBus 2.0 5.5.7
. .
S Addr Wr [A] i2c_reg [A] count [A] buf0 [A] buf1 [A] ...
   [A] bufn [A] P
. .
D*/

/*F*/
int i2c_read_block_data(int sbc, int handle, int i2c_reg, char *buf);
/*D
This reads a block of up to 32 bytes from the specified register of
the device.

. .
    sbc: >= 0 (as returned by [*rgpiod_start*]).
 handle: >= 0 (as returned by [*i2c_open*]).
i2c_reg: 0-255, the register to read.
    buf: an array to receive the read data.
. .

If OK returns the count of bytes read and updates buf.

On failure returns a negative error code.

The amount of returned data is set by the device.

Block read. SMBus 2.0 5.5.7
. .
S Addr Wr [A] i2c_reg [A]
   S Addr Rd [A] [Count] A [buf0] A [buf1] A ... A [bufn] NA P
. .
D*/

/*F*/
int i2c_block_process_call(
   int sbc, int handle, int i2c_reg, char *buf, int count);
/*D
This writes data bytes to the specified register of the device
and reads a device specified number of bytes of data in return.

. .
    sbc: >= 0 (as returned by [*rgpiod_start*]).
 handle: >= 0 (as returned by [*i2c_open*]).
i2c_reg: 0-255, the register to write/read.
    buf: an array with the data to send and to receive the read data.
  count: 1-32, the number of bytes to write.
. .

If OK returns the count of bytes read and updates buf.

On failure returns a negative error code.

The smbus 2.0 documentation states that a minimum of 1 byte may be
sent and a minimum of 1 byte may be received.  The total number of
bytes sent/received must be 32 or less.

Block write-block read. SMBus 2.0 5.5.8
. .
S Addr Wr [A] i2c_reg [A] count [A] buf0 [A] ...
   S Addr Rd [A] [Count] A [Data] ... A P
. .
D*/

/*F*/
int i2c_read_i2c_block_data(
   int sbc, int handle, int i2c_reg, char *buf, int count);
/*D
This reads count bytes from the specified register of the device.
The count may be 1-32.

. .
    sbc: >= 0 (as returned by [*rgpiod_start*]).
 handle: >= 0 (as returned by [*i2c_open*]).
i2c_reg: 0-255, the register to read.
    buf: an array to receive the read data.
  count: 1-32, the number of bytes to read.
. .

If OK returns the count of bytes read and updates buf.

On failure returns a negative error code.

. .
S Addr Wr [A] i2c_reg [A]
   S Addr Rd [A] [buf0] A [buf1] A ... A [bufn] NA P
. .
D*/


/*F*/
int i2c_write_i2c_block_data(
   int sbc, int handle, int i2c_reg, const char *buf, int count);
/*D
This writes 1 to 32 bytes to the specified register of the device.

. .
    sbc: >= 0 (as returned by [*rgpiod_start*]).
 handle: >= 0 (as returned by [*i2c_open*]).
i2c_reg: 0-255, the register to write.
    buf: the data to write.
  count: 1-32, the number of bytes to write.
. .

If OK returns 0.

On failure returns a negative error code.

. .
S Addr Wr [A] i2c_reg [A] buf0 [A] buf1 [A] ... [A] bufn [A] P
. .
D*/

/*F*/
int i2c_read_device(int sbc, int handle, char *buf, int count);
/*D
This reads count bytes from the raw device into buf.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*i2c_open*]).
   buf: an array to receive the read data bytes.
 count: >0, the number of bytes to read.
. .

If OK returns the count of bytes read and updates buf.

On failure returns a negative error code.

. .
S Addr Rd [A] [buf0] A [buf1] A ... A [bufn] NA P
. .
D*/

/*F*/
int i2c_write_device(int sbc, int handle, const char *buf, int count);
/*D
This writes count bytes from buf to the raw device.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*i2c_open*]).
   buf: an array containing the data bytes to write.
 count: >0, the number of bytes to write.
. .

If OK returns 0.

On failure returns a negative error code.

. .
S Addr Wr [A] buf0 [A] buf1 [A] ... [A] bufn [A] P
. .
D*/

/*F*/
int i2c_zip(
   int sbc, int handle,
   const char *inBuf, int inCount, char *outBuf, int outCount);
/*D
This function executes a sequence of I2C operations.  The
operations to be performed are specified by the contents of inBuf
which contains the concatenated command codes and associated data.

. .
     sbc: >= 0 (as returned by [*rgpiod_start*]).
  handle: >= 0, as returned by a call to [*lgI2cOpen*]
   inBuf: pointer to the concatenated I2C commands, see below
 inCount: size of command buffer
  outBuf: pointer to buffer to hold returned data
outCount: size of output buffer
. .

If OK returns the count of bytes read and updates outBuf.

On failure returns a negative error code.

The following command codes are supported:

Name    @ Cmd & Data @ Meaning
End     @ 0          @ No more commands
Escape  @ 1          @ Next P is two bytes
On      @ 2          @ Switch combined flag on
Off     @ 3          @ Switch combined flag off
Address @ 4 P        @ Set I2C address to P
Flags   @ 5 lsb msb  @ Set I2C flags to lsb + (msb << 8)
Read    @ 6 P        @ Read P bytes of data
Write   @ 7 P ...    @ Write P bytes of data

The address, read, and write commands take a parameter P.
Normally P is one byte (0-255).  If the command is preceded by
the Escape command then P is two bytes (0-65535, least significant
byte first).

The address defaults to that associated with the handle.
The flags default to 0.  The address and flags maintain their
previous value until updated.

The returned I2C data is stored in consecutive locations of outBuf.

...
Set address 0x53, write 0x32, read 6 bytes
Set address 0x1E, write 0x03, read 6 bytes
Set address 0x68, write 0x1B, read 8 bytes
End

0x04 0x53   0x07 0x01 0x32   0x06 0x06
0x04 0x1E   0x07 0x01 0x03   0x06 0x06
0x04 0x68   0x07 0x01 0x1B   0x06 0x08
0x00
...

D*/



/* ----------------------------------------------------- NOTIFICATIONS API
*/

/*F*/
int notify_open(int sbc);
/*D
Get a free notification handle.

This is a privileged command.  See [+permits+].

. .
sbc: >= 0 (as returned by [*rgpiod_start*]).
. .

If OK returns a handle (>= 0).

On failure returns a negative error code.

A notification is a method for being notified of GPIO state
changes via a pipe.

Pipes are only accessible from the local machine so this function
serves no purpose if you are using the library from a remote machine.
The in-built (socket) notifications provided by [*callback*]
should be used instead.

The notification pipes are created in the library working directory.

Notifications for handle x will be available at the pipe
named .lgd-nfyx (where x is the handle number). E.g. if the
function returns 15 then the notifications must be
read from .lgd-nfy15.

Each notification occupies 16 bytes in the fifo and has the
following structure.

. .
typedef struct
{
   uint64_t timestamp; // alert time in nanoseconds
   uint8_t chip;       // gpiochip device number
   uint8_t gpio;       // offset into gpio device
   uint8_t level;      // 0=low, 1=high, 2=timeout
   uint8_t flags;      // none currently defined
} lgGpioReport_t;
. .

timestamp: the number of nanoseconds since the epoch (start of 1970)
chip: the gpiochip device number (NOT the handle). 
gpio: the GPIO. 
level: indicates the level of the GPIO 
flags: no flags are currently defined

For future proofing it is probably best to ignore any notification
with non-zero flags.

D*/

/*F*/
int notify_resume(int sbc, int handle);
/*D
Resume notifications on a handle.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*notify_open*])
. .

If OK returns 0.

On failure returns a negative error code.
D*/

/*F*/
int notify_pause(int sbc, int handle);
/*D
Pauses notifications on a handle.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*notify_open*])
. .


If OK returns 0.

On failure returns a negative error code.

Notifications for the handle are suspended until
[*notify_resume*] is called.
D*/

/*F*/
int notify_close(int sbc, int handle);
/*D
Stop notifications and releases the handle.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*notify_open*])
. .

If OK returns 0.

On failure returns a negative error code.
D*/



/* ----------------------------------------------------------- SCRIPTS API
*/

/*F*/
int script_store(int sbc, const char *script);
/*D
This function stores a script for later execution.

This is a privileged command.  See [+permits+].

See [[scripts.html]] for details.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
script: the text of the script.
. .

If OK returns a handle (>=0).

On failure returns a negative error code.
D*/

/*F*/
int script_run(int sbc, int handle, int count, const uint32_t *param);
/*D
This function runs a stored script.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*script_store*]).
 count: 0-10, the number of parameters.
 param: an array of parameters.
. .

If OK returns 0.

On failure returns a negative error code.

param is an array of up to 10 parameters which may be referenced in
the script as p0 to p9.
D*/

/*F*/
int script_update(int sbc, int handle, int count, const uint32_t *param);
/*D
This function sets the parameters of a script.  The script may or
may not be running.  The first numPar parameters of the script are
overwritten with the new values.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*script_store*]).
 count: 0-10, the number of parameters.
 param: an array of parameters.
. .

If OK returns 0.

On failure returns a negative error code.

param is an array of up to 10 parameters which may be referenced in
the script as p0 to p9.
D*/

/*F*/
int script_status(int sbc, int handle, uint32_t *param);
/*D
This function returns the run status of a stored script as well
as the current values of parameters 0 to 9.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*script_store*]).
 param: an array to hold the returned 10 parameters.
. .

If OK returns the script status and updates param.

On failure returns a negative error code.

The script status may be

. .
LG_SCRIPT_INITING
LG_SCRIPT_READY
LG_SCRIPT_RUNNING
LG_SCRIPT_WAITING
LG_SCRIPT_ENDED
LG_SCRIPT_HALTED
LG_SCRIPT_FAILED
. .

The current value of script parameters 0 to 9 are returned in param.
D*/

/*F*/
int script_stop(int sbc, int handle);
/*D
This function stops a running script.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*script_store*]).
. .

If OK returns 0.

On failure returns a negative error code.
D*/

/*F*/
int script_delete(int sbc, int handle);
/*D
This function deletes a stored script.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*script_store*]).
. .

If OK returns 0.

On failure returns a negative error code.
D*/

/* ------------------------------------------------------------ SERIAL API
*/

/*F*/
int serial_open(int sbc, const char *ser_tty, int ser_baud, int ser_flags);
/*D
This function opens a serial device at a specified baud rate
with specified flags.  The device must be present in /dev.

This is a privileged command.  See [+permits+].

. .
      sbc: >= 0 (as returned by [*rgpiod_start*]).
  ser_tty: the serial device to open.
 ser_baud: the baud rate in bits per second, see below.
ser_flags: 0.
. .

If OK returns 0.

On failure returns a negative error code.

The baud rate must be one of 50, 75, 110, 134, 150,
200, 300, 600, 1200, 1800, 2400, 4800, 9600, 19200,
38400, 57600, 115200, or 230400.

No flags are currently defined.  This parameter should be set to zero.
D*/

/*F*/
int serial_close(int sbc, int handle);
/*D
This function closes the serial device.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*serial_open*]).
. .

If OK returns 0.

On failure returns a negative error code.
D*/

/*F*/
int serial_write_byte(int sbc, int handle, int byteVal);
/*D
This function writes byteVal to the serial port.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*serial_open*]).
. .

If OK returns 0.

On failure returns a negative error code.
D*/

/*F*/
int serial_read_byte(int sbc, int handle);
/*D
This function reads a byte from the serial port.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*serial_open*]).
. .

If OK returns the read byte (0-255).

On failure returns a negative error code.
D*/

/*F*/
int serial_write(int sbc, int handle, const char *buf, int count);
/*D
This function writes count bytes from buf to the the serial port.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*serial_open*]).
   buf: the array of bytes to write.
 count: the number of bytes to write.
. .

If OK returns 0.

On failure returns a negative error code.
D*/

/*F*/
int serial_read(int sbc, int handle, char *buf, int count);
/*D
This function reads up to count bytes from the the serial port
and writes them to buf.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*serial_open*]).
   buf: an array to receive the read data.
 count: the maximum number of bytes to read.
. .

If OK returns the count of bytes read and updates buf.

On failure returns a negative error code.

If no data is ready zero is returned.
D*/

/*F*/
int serial_data_available(int sbc, int handle);
/*D
Returns the number of bytes available to be read from the
device.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*serial_open*]).
. .

If OK returns the count of bytes available.

On failure returns a negative error code.
D*/

/* ------------------------------------------------------------- SHELL API
*/

/*F*/
int shell(int sbc, const char *scriptName, const char *scriptString);
/*D
This function uses the system call to execute a shell script
with the given string as its parameter.

This is a privileged command.  See [+permits+].

. .
         sbc: >= 0 (as returned by [*rgpiod_start*]).
  scriptName: the name of the script, only alphanumeric characters,
              '-' and '_' are allowed in the name.
scriptString: the string to pass to the script.
. .

If OK returns 0.

On failure returns a negative error code.

scriptName must exist in a directory named cgi in the daemon's
configuration directory and must be executable.

The returned exit status is normally 256 times that set by the
shell script exit function.  If the script can't be found 32512 will
be returned.

The following table gives some example returned statuses.

Script exit status @ Returned system call status
1                  @ 256
5                  @ 1280
10                 @ 2560
200                @ 51200
script not found   @ 32512

...
// pass two parameters, hello and world
status = shell_(sbc, "scr1", "hello world");

// pass three parameters, hello, string with spaces, and world
status = shell_(sbc, "scr1", "hello 'string with spaces' world");

// pass one parameter, hello string with spaces world
status = shell_(sbc, "scr1", "\"hello string with spaces world\"");
...
D*/

/* --------------------------------------------------------------- SPI API
*/

/*F*/
int spi_open(
   int sbc, int spi_device, int spi_channel, int spi_baud, int spi_flags);
/*D
This function returns a handle for the SPI device on the channel.
Data will be transferred at baud bits per second.  The flags may
be used to modify the default behaviour.

This is a privileged command.  See [+permits+].

. .
        sbc: >= 0 (as returned by [*rgpiod_start*]).
 spi_device: >= 0
spi_channel: >= 0
   spi_baud: SPI speed in bits per second.
  spi_flags: see below.
. .

If OK returns a handle (>= 0).

On failure returns a negative error code.

spi_flags consists of the least significant 2 bits.

. .
1  0
m  m
. .

mm defines the SPI mode.

. .
Mode POL PHA
 0    0   0
 1    0   1
 2    1   0
 3    1   1
. .


The other bits in flags should be set to zero.
D*/

/*F*/
int spi_close(int sbc, int handle);
/*D
This functions closes the SPI device identified by the handle.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*spi_open*]).
. .

If OK returns 0.

On failure returns a negative error code.
D*/

/*F*/
int spi_read(int sbc, int handle, char *buf, int count);
/*D
This function reads count bytes of data from the SPI
device

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*spi_open*]).
   buf: an array to receive the read data bytes.
 count: the number of bytes to read.
. .

If OK returns the count of bytes read and updates buf.

On failure returns a negative error code.
D*/

/*F*/
int spi_write(int sbc, int handle, const char *buf, int count);
/*D
This function writes count bytes of data from buf to the SPI
device

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*spi_open*]).
   buf: the data bytes to write.
 count: the number of bytes to write.
. .

If OK returns the count of bytes written.

On failure returns a negative error code.
D*/

/*F*/
int spi_xfer(
   int sbc, int handle, const char *txBuf, char *rxBuf, int count);
/*D
This function transfers count bytes of data from txBuf to the SPI
device  Simultaneously count bytes of
data are read from the device and placed in rxBuf.

. .
   sbc: >= 0 (as returned by [*rgpiod_start*]).
handle: >= 0 (as returned by [*spi_open*]).
 txBuf: the data bytes to write.
 rxBuf: the received data bytes.
 count: the number of bytes to transfer.
. .

If OK returns the count of bytes transferred and updates rxBuf.

On failure returns a negative error code.
D*/


/* ----------------------------------------------------------- THREADS API
*/


/*F*/
pthread_t *thread_start(lgThreadFunc_t thread_func, void *userdata);
/*D
Starts a new thread of execution with thread_func as the main routine.

. .
thread_func: the main function for the new thread.
   userdata: a pointer to an arbitrary argument.
. .

If OK returns a pointer to a pthread_t.

On failure returns NULL.

The function is passed the single argument userdata.

The thread can be cancelled by passing the pointer to pthread_t to
[*thread_stop*].
D*/

/*F*/
void thread_stop(pthread_t *pth);
/*D
Cancels the thread pointed at by pth.

. .
pth: the thread to be stopped.
. .

No value is returned.

The thread to be stopped should have been started with [*thread_start*].
D*/

/* --------------------------------------------------------- UTILITIES API
*/

/*F*/
int lgu_get_internal(int sbc, int config_id, uint64_t *config_value);
/*D
Returns the value of a sbc configuration item.

This is a privileged command.  See [+permits+].

. .
         sbc: >= 0 (as returned by [*rgpiod_start*]).
   config_id: the configuration item.
config_value: pointer for returned value.
. .

If OK returns 0 and updates config_value.

On failure returns a negative error code.
D*/

/*F*/
int lgu_set_internal(int sbc, int config_id, uint64_t config_value);
/*D
Sets the value of a sbc configuration item.

This is a privileged command.  See [+permits+].

. .
         sbc: >= 0 (as returned by [*rgpiod_start*]).
   config_id: the configuration item.
config_value: the value to set.
. .

If OK returns 0.

On failure returns a negative error code.
D*/

/*F*/
int lgu_get_sbc_name(int sbc, char *buf, int count);
/*D
Return the lgd server name.

. .
  sbc: >= 0 (as returned by [*rgpiod_start*]).
  buf: the server name is copied to this buffer.
count: the maximum number of characters to copy.
. .

If OK returns the count of bytes copied and updates buf.

On failure returns a negative error code.
D*/

/*F*/
int lgu_set_user(int sbc, char *user, char *secretsFile);
/*D
Sets the rgpiod daemon user.  The user then has the
associated permissions.

. .
        sbc: >= 0 (as returned by [*rgpiod_start*]).
       user: the user to set ("" defaults to the default user).
secretsFile: the path to the shared secret file ("" defaults
             to "~/.lg_secret").
. .

If OK returns 1 if the user was set, 0 otherwise.

On failure returns a negative error code.

...
if (lgu_set_user(sbc, "gpio", "")
{
   printf("using user gpio permissions");
}
else
{
   printf("using default permissions");
}
...
D*/

/*F*/
int lgu_set_share_id(int sbc, int handle, int share_id);
/*D
Sets the share id of an owned object.

. .
     sbc: >= 0 (as returned by [*rgpiod_start*]).
  handle: >= 0
share_id: >= 0, 0 stops sharing.
. .

If OK returns 0.

On failure returns a negative error code.

Normally objects associated with a handle are only accessible
to the program which created them (and are automatically
deleted when the program ends).

If a non-zero share is set the object is accessible to any
software which knows the share and the handle (and are not
automatically deleted when the program ends).

...
lgu_set_share_id(sbc, handle, 23);
...
D*/

/*F*/
int lgu_use_share_id(int sbc, int share_id);
/*D
Sets the share id to be used when asking to use an object
owned by another creator.

. .
     sbc: >= 0 (as returned by [*rgpiod_start*]).
share_id: >= 0, 0 stops sharing.
. .

If OK returns 0.

On failure returns a negative error code.

Normally objects associated with a handle are only accessible
to the program which created them (and are automatically
deleted when the program ends).

If a non-zero share is set the object is accessible to any
software which knows the share and the handle.

...
lgu_use_share_id(sbc, 23);
...
D*/

/*F*/
uint32_t lgu_rgpio_version(void);
/*D
Return the rgpio version.

If OK returns the rgpio version.

On failure returns a negative error code.
D*/

/*F*/
const char *lgu_error_text(int errnum);
/*D
Return a text description for an error code.

. .
errnum: the error code.
. .
D*/

/*F*/
void lgu_sleep(double sleepSecs);
/*D
Delay execution for a given number of seconds.

. .
sleepSecs: the number of seconds to delay.
. .
D*/

/*F*/
double lgu_time(void);
/*D
Return the current time in seconds since the Epoch.
D*/

/*F*/
uint64_t lgu_timestamp(void);
/*D
Return the current time in nanoseconds since the Epoch.
D*/


/*PARAMS

*addrStr::
A string specifying the host or IP address of the SBC running
the rgpiod daemon.  It may be NULL in which case localhost
is used unless overridden by the LG_ADDR environment
variable.

bitVal::
A value of 0 or 1.

*buf::
A buffer to hold data being sent or being received.

byteVal::0-255
An 8-bit byte value.

callback_id::
A value >= 0, as returned by a call to the [*callback*].

The id is passed to [*callback_cancel*] to cancel the callback.

CBFunc_t::
. .
typedef void (*CBFunc_t)
   (int sbc, int chip, int gpio, int level, uint64_t timestamp, void * userdata);
. .

char::
A single character, an 8 bit quantity able to store 0-255.

chipInfo::
A pointer to a lgChipInfo_t object.

config_id::
A number identifying a configuration item.

. .
LG_CFG_ID_DEBUG_LEVEL 0
LG_CFG_ID_MIN_DELAY   1
. .

config_value::
The value of a configuration item.

*config_value::
The value of a configuration item.

count::
The number of bytes to be transferred in a file, I2C, SPI, or serial
command.

debounce_us::
The debounce time in microseconds.

double::
A floating point number.

edge::
Used to identify a GPIO level transition of interest.  A rising edge is
a level change from 0 to 1.  A falling edge is a level change from 1 to 0.

. .
RISING_EDGE   1
FALLING_EDGE  2
BOTH_EDGES   3
. .

eFlags::
The type of GPIO edge to generate an alert.  See [*gpio_claim_alert*].

. .
RISING_EDGE   1
FALLING_EDGE  2
BOTH_EDGES   3
. .


errnum::
A negative number indicating a function call failed and the nature
of the error.

f::
A function.

*file::
A full file path.  To be accessible the path must match an entry in
the [files] section of the permits file.

float::
A floating point number.

*fpat::
A file path which may contain wildcards.  To be accessible the path
must match an entry in the [files] section of the permits file.

gpio::
A 0 based offset of a GPIO within a gpiochip.

gpioDev :: >= 0
The device number of a gpiochip.

*gpios::
An array of GPIO numbers.

groupBits::
A 64-bit value used to set the levels of a GPIO group.

Set bit x to set GPIO x of the group high.

Clear bit x to set GPIO x of the group low.

*groupBits::
A 64-bit value denoting the levels of a GPIO group.

If bit x is set then GPIO x of the group is high.

groupMask::
A 64-bit value used to determine which members of a GPIO group
should be updated.

Set bit x to update GPIO x of the group.

Clear bit x to leave GPIO x of the group unaltered.

handle::>= 0
A number referencing an object opened by one of


[*file_open*] 
[*gpiochip_open*] 
[*i2c_open*] 
[*notify_open*] 
[*serial_open*]
[*script_store*] 
[*spi_open*]

i2c_addr::0-0x7F
The address of a device on the I2C bus.

i2c_bus::>= 0
An I2C bus number.

i2c_flags::0
Flags which modify an I2C open command.  None are currently defined.

i2c_reg:: 0-255
A register of an I2C device.

*inBuf::
A buffer used to pass data to a function.

inCount::
The size of an input buffer.

int::
A whole number, negative or positive.

int32_t::
A 32-bit signed value.

kind:: LG_TX_PWM or LG_TX_WAVE
A type of transmission: PWM or wave.

lFlags::
Line flags for the GPIO.

The following values may be or'd to form the value.

. .
LG_SET_ACTIVE_LOW
LG_SET_OPEN_DRAIN
LG_SET_OPEN_SOURCE
. .

lgChipInfo_p::
A pointer to a lgChipInfo_t object.

. .
typedef struct lgChipInfo_s
{
   uint32_t lines;                // number of GPIO
   char name[LG_GPIO_NAME_LEN];   // Linux name
   char label[LG_GPIO_LABEL_LEN]; // functional name
} lgChipInfo_t, *lgChipInfo_p;
. .

lgLineInfo_p::
A pointer to a lgLineInfo_t object.

. .
typedef struct lgLine_s
{
   uint32_t offset;               // GPIO number
   uint32_t lFlags;
   char name[LG_GPIO_NAME_LEN];   // GPIO name
   char user[LG_GPIO_USER_LEN];   // user
} lgLineInfo_t, *lgLineInfo_p;
. .


lgPulse_p::
A pointer to a lgPulse_t object.

. .
typedef struct lgPulse_s
{
   uint64_t bits;
   uint64_t mask;
   int64_t delay;
} lgPulse_t, *lgPulse_p;
. .

lgThreadFunc_t::
. .
typedef void *(lgThreadFunc_t) (void *);
. .

lineInfo::
A pointer to a lgLineInfo_t object.

mode::
A file open mode.

. .
LG_FILE_READ  1
LG_FILE_WRITE 2
LG_FILE_RW    3
. .

The following values can be or'd into the mode.

. .
LG_FILE_APPEND 4
LG_FILE_CREATE 8
LG_FILE_TRUNC  16
. .

nfyHandle:: >= 0
This associates a notification with a GPIO alert.

*outBuf::
A buffer used to return data from a function.

outCount::
The size of an output buffer.

*param::
An array of script parameters.

*portStr::
A string specifying the port address used by the SBC running
the rgpiod daemon.  It may be NULL in which case "8889"
is used unless overridden by the LG_PORT environment
variable.

*pth::
A thread identifier, returned by [*thread_start*].

pthread_t::
A thread identifier.

pulse_cycles:: >= 0
The number of pulses to generate.  A value of 0 means infinite.#

pulse_off:: >= 0
The off period for a pulse in microseconds.

pulse_offset:: >= 0
The offset in microseconds from the nominal pulse start.

pulse_on:: >= 0
The on period for a pulse in microseconds.

pulses::
An pointer to an array of lgPulse_t objects.

pulseWidth:: 0, 500-2500 microseconds
Servo pulse width

pwmCycles:: >= 0
The number of PWM pulses to generate.  A value of 0 means infinite.

pwmDutyCycle:: 0-100 %
PWM duty cycle %

pwmFrequency:: 0.1-10000 Hz
PWM frequency

pwmOffset:: >= 0
The offset in microseconds from the nominal PWM pulse start.

*rxBuf::
A pointer to a buffer to receive data.

sbc::
An integer defining a connected SBC.  The value is returned by
[*rgpiod_start*] upon success.

*script::
A pointer to the text of a script.

*scriptName::
The name of a [*shell_*] script to be executed.  The script must
be present in the cgi directory of the daemon's configuration
directory and must have execute permission.

*scriptString::
The string to be passed to a [*shell_*] script to be executed.

*secretsFile::
The file containing the shared secret for a user.  If the shared
secret for a user matches that known by the rgpiod daemon the user can
"log in" to the daemon.

seekFrom::
. .
LG_FROM_START   0
LG_FROM_CURRENT 1
LG_FROM_END     2
. .

seekOffset::
The number of bytes to move forward (positive) or backwards (negative)
from the seek position (start, current, or end of file).

ser_baud::
The speed of serial communication in bits per second.

ser_flags::
Flags which modify a serial open command.  None are currently defined.

*ser_tty::
The name of a serial tty device, e.g. /dev/ttyAMA0, /dev/ttyUSB0, /dev/tty1.

servoCycles:: >= 0
The number of servo pulses to generate.  A value of 0 means infinite.

servoFrequency:: 40-500 Hz
Servo pulse frequency

servoOffset:: >= 0
The offset in microseconds from the nominal servo pulse start.

share_id::
Objects created with a non-zero share_id are persistent and may be
used by other software which knows the share_id.

sleepSecs::
The number of seconds to delay.

spi_baud::
The speed of SPI communication in bits per second.

spi_channel:: >= 0
A SPI channel.

spi_device:: >= 0
A SPI device.

spi_flags::
See [*spi_open*] and [*bb_spi_open*].

thread_func::
A function of type gpioThreadFunc_t used as the main function of a
thread.

*txBuf::
An array of bytes to transmit.

uint32_t::
A 32-bit unsigned value.

uint64_t::
A 64-bit unsigned value.

*user::
A name known by the rgpiod daemon and associated with a set of user
permissions.

*userdata::
A pointer to arbitrary user data.  This may be used to identify the instance.

You must ensure that the pointer is in scope at the time it is processed.  If
it is a pointer to a global this is automatic.  Do not pass the address of a
local variable.  If you want to pass a transient object then use the
following technique.

In the calling function:

. .
user_type *userdata; 
user_type my_userdata;

userdata = malloc(sizeof(user_type)); 
*userdata = my_userdata;
. .

In the receiving function:

. .
user_type my_userdata = *(user_type*)userdata;

free(userdata);
. .

value:: 0-1
A GPIO level.

*values::
An array of GPIO values.

void::
Denoting no parameter is required

watchdog_us::
The watchdog time in microseconds.

wordVal::0-65535
A 16-bit word value.

PARAMS*/

/*DEF_S rgpio Error Codes*/

typedef enum
{
   lgif_bad_send           = -2000,
   lgif_bad_recv           = -2001,
   lgif_bad_getaddrinfo    = -2002,
   lgif_bad_connect        = -2003,
   lgif_bad_socket         = -2004,
   lgif_bad_noib           = -2005,
   lgif_duplicate_callback = -2006,
   lgif_bad_malloc         = -2007,
   lgif_bad_callback       = -2008,
   lgif_notify_failed      = -2009,
   lgif_callback_not_found = -2010,
   lgif_unconnected_sbc    = -2011,
   lgif_too_many_pis       = -2012,
} lgifError_t;

/*DEF_E*/

#ifdef __cplusplus
}
#endif

#endif

