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
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "lgpio.h"
#include "rgpiod.h"

#include "lgCmd.h"
#include "lgMD5.h"

/*
This program provides a socket interface to some of
the commands available from lg.
*/

#define RGS_VERSION 0x00010000

#define RGS_CONNECT_ERR 255
#define RGS_OPTION_ERR  254
#define RGS_SCRIPT_ERR  253

char text[CMD_MAX_EXTENSION];

int printFlags = 0;

int status = LG_OKAY;

#define SOCKET_OPEN_FAILED -1

#define PRINT_HEX 1
#define PRINT_ASCII 2

static char *xCmdUsage = "\n\
C                 Set share\n\
CGI cid           Get internal configuration setting\n\
CSI cid v         Set internal configuration setting\n\
\n\
FC h              File close\n\
FL pat num        File list\n\
FO file mode      File open\n\
FR h num          File read\n\
FS h num from     File seek\n\
FW h bvs          File write\n\
\n\
GBUSY h g k       GPIO or group tx busy\n\
GC h              gpiochip close device\n\
GDEB h g us       GPIO debounce time\n\
GGR h g           GPIO group read\n\
GGW h g gbits     GPIO group write (simple)\n\
GGWX h g gbits gmask  | GPIO group write\n\
GIC h             gpiochip information\n\
GIL h g           gpiochip line information\n\
GO gc             gpiochip open device\n\
GP h g mon moff   GPIO tx pulses (simple)\n\
GPX h g mon moff off cyc  | GPIO tx pulses\n\
GR h g            GPIO read\n\
GROOM h g k       GPIO or group tx entries\n\
GSA h g           GPIO claim for alerts (simple)\n\
GSAX h lf ef g nfyh  | GPIO claim for alerts\n\
GSF h g           GPIO free\n\
GSGF h g          GPIO group free\n\
GSGI h g*         GPIO group claim for inputs (simple)\n\
GSGIX h lf g*     GPIO group claim for inputs\n\
GSGO h g*         GPIO group claim for outputs (simple)\n\
GSGOX h lf g* v*  GPIO group claim for outputs\n\
GSI h g           GPIO claim for input (simple)\n\
GSIX h lf g       GPIO claim for input\n\
GSO h g           GPIO claim for output\n\
GSOX h lf g v     GPIO claim for output\n\
GW h g v          GPIO write\n\
GWAVE h g p*      GPIO group tx wave\n\
GWDOG h g us      GPIO watchdog time\n\
\n\
I2CC h            I2C close device\n\
I2CO ib id if     I2C open device\n\
I2CPC h r wv      SMB Process Call: exchange register with word\n\
I2CPK h r bvs     SMB Block Process Call: exchange data bytes with register\n\
I2CRB h r         SMB Read Byte Data: read byte from register\n\
I2CRD h num       I2C read device\n\
I2CRI h r num     SMB Read I2C Block Data: read bytes from register\n\
I2CRK h r         SMB Read Block Data: read data from register\n\
I2CRS h           SMB Read Byte: read byte\n\
I2CRW h r         SMB Read Word Data: read word from register\n\
I2CWB h r bv      SMB Write Byte Data: write byte to register\n\
I2CWD h bvs       I2C write device\n\
I2CWI h r bvs     SMB Write I2C Block Data\n\
I2CWK h r bvs     SMB Write Block Data: write data to register\n\
I2CWQ h bit       SMB Write Quick: write bit\n\
I2CWS h bv        SMB Write Byte: write byte\n\
I2CWW h r wv      SMB Write Word Data: write word to register\n\
I2CZ h bvs        I2C zip\n\
\n\
LCFG              Reload permits configuration file\n\
LGV               Get lg library version\n\
\n\
MICS v            Microseconds delay\n\
MILS v            Milliseconds delay\n\
\n\
NC h              Notification close\n\
NO                Notification open\n\
NP h              Notification pause\n\
NR h              Notification resume\n\
\n\
P h g pf pdc      GPIO tx PWM (simple)\n\
PARSE t           Script validate\n\
PCD               Print daemon configuration directory\n\
PROC t            Script store\n\
PROCD h           Script delete\n\
PROCP h           Script get status and parameters\n\
PROCR h pars      Script run\n\
PROCS h           Script stop\n\
PROCU h pars      Script update parameters\n\
PWD               Print daemon working directory\n\
PX h g pf pdc off cyc  | GPIO tx PWM\n\
\n\
S h g spw         GPIO tx servo pulses (simple)\n\
SBC               Get SBC's host name\n\
SERC h            Serial close device\n\
SERDA h           Serial data available\n\
SERO dev b sef    Serial open device\n\
SERR h num        Serial read bytes\n\
SERRB h           Serial read byte\n\
SERW h bvs        Serial write bytes\n\
SERWB h bv        Serial write byte\n\
SHELL name str    Execute a shell command\n\
SPIC h            SPI close device\n\
SPIO spd spc b spf  | SPI open device\n\
SPIR h num        SPI read bytes\n\
SPIW h bvs        SPI write bytes\n\
SPIX h bvs        SPI transfer bytes\n\
SX h g sf spw off cyc  | GPIO tx servo pulses\n\
SHARE             Set share\n\
\n\
T                 Get nanoseconds since the epoch\n\
TICK              Get nanoseconds since the epoch\n\
\n\
U                 Set user\n\
USER              Set user\n\
\n\
Numbers may be entered as hex (prefix 0x), octal (prefix 0),\n\
otherwise they are assumed to be decimal.\n\
\n\
Examples\n\
\n\
rgs u test1 s 1 i2co 1 0x20 0 # get handle to device 0x20 on I2C bus 1\n\
\n\
man rgs for full details.\n\
\n";

static uint64_t xMakeSalt(void)
{
   struct timespec xts;

   clock_gettime(CLOCK_REALTIME, &xts);

   return ((xts.tv_sec + xts.tv_nsec) * random()) + (random() + xts.tv_nsec);
}

static void xReport(int err, char *fmt, ...)
{
   char buf[128];
   va_list ap;

   if (err > status) status = err;

   va_start(ap, fmt);
   vsnprintf(buf, sizeof(buf), fmt, ap);
   va_end(ap);

   fprintf(stderr, "%s\n", buf);

   fflush(stderr);
}

static int xInitOpts(int argc, char *argv[])
{
   int opt, args;

   opterr = 0;

   args = 1;

   while ((opt = getopt(argc, argv, "ahvx")) != -1)
   {
      switch (opt)
      {
         case 'a':
            printFlags |= PRINT_ASCII;
            args++;
            break;

         case 'h':
            printf("%s", xCmdUsage);
            args++;
            break;

         case 'x':
            printFlags |= PRINT_HEX;
            args++;
            break;

         case 'v':
            printf("rgs_%d.%d.%d.%d\n",
               (RGS_VERSION>>24)&0xff, (RGS_VERSION>>16)&0xff,
               (RGS_VERSION>>8)&0xff, RGS_VERSION&0xff);
            args++;
            break;

         default:
            args++;
            xReport(RGS_OPTION_ERR, "ERROR: bad option %c", optopt);
      }
    }
   return args;
}

static int xOpenSocket(void)
{
   int sock, err;
   struct addrinfo hints, *res, *rp;
   const char *addrStr, *portStr;

   portStr = getenv(LG_ENVPORT);

   if (!portStr) portStr = LG_DEFAULT_SOCKET_PORT_STR;

   addrStr = getenv(LG_ENVADDR);

   if (!addrStr) addrStr = LG_DEFAULT_SOCKET_ADDR_STR;

   memset (&hints, 0, sizeof (hints));

   hints.ai_family   = PF_UNSPEC;
   hints.ai_socktype = SOCK_STREAM;
   hints.ai_flags   |= AI_CANONNAME;

   err = getaddrinfo(addrStr, portStr, &hints, &res);

   if (err) return SOCKET_OPEN_FAILED;

   for (rp=res; rp!=NULL; rp=rp->ai_next)
   {
      sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);

      if (sock == -1) continue;

      if (connect(sock, rp->ai_addr, rp->ai_addrlen) != -1) break;
   }

   freeaddrinfo(res);

   if (rp == NULL) return SOCKET_OPEN_FAILED;

   return sock;
}

static void xShowResult(int rv, lgCmd_p cmdP, char *cmdExt)
{
   int i, r, ch;
   uint32_t *argI=(uint32_t*)&cmdP[1];
   uint64_t *argQ=(uint64_t*)&cmdP[1];

   r = cmdP->status;

   switch (rv)
   {
      case 0:
      case 1:
         if (r < 0)
         {
            printf("%d\n", r);
            xReport(RGS_SCRIPT_ERR, "ERROR: %s", lguErrorText(r));
         }
         break;

      case 2:
         printf("%d\n", r);
         if (r < 0) xReport(RGS_SCRIPT_ERR, "ERROR: %s", lguErrorText(r));
         break;

      case 3:
         if (r < 0)
         {
            printf("%d\n", r);
            xReport(RGS_SCRIPT_ERR, "ERROR: %s", lguErrorText(r));
         }
         else printf("%"PRIu64"\n", argQ[0]);
         break;

      case 5:
         printf("lg_%d.%d.%d.%d\n",
            (cmdP->status>>24)&0xff, (cmdP->status>>16)&0xff,
            (cmdP->status>>8)&0xff, cmdP->status&0xff);
         break;

      case 6: /*
                 BI2CZ  CF2  FL  FR  I2CPK  I2CRD  I2CRI  I2CRK
                 I2CZ  SBC SERR  SLR  SPIX  SPIR USER
              */

         if ((cmdP->cmd == LG_CMD_PCD) ||
             (cmdP->cmd == LG_CMD_PWD) ||
             (cmdP->cmd == LG_CMD_SBC))
         {
            printf("%s\n", cmdExt);
            break;
         }

         printf("%d", r);
         if (r < 0) xReport(RGS_SCRIPT_ERR, "ERROR: %s", lguErrorText(r));
         if (r > 0)
         {
            if (printFlags == PRINT_ASCII) printf(" ");

            for (i=0; i<r; i++)
            {
               ch = cmdExt[i];

               if (printFlags & PRINT_HEX) printf(" %hhx", ch);

               else if (printFlags & PRINT_ASCII)
               {
                  if (isprint(ch) || (ch == '\n') || (ch == '\r'))
                     printf("%c", ch);
                  else printf("\\x%02hhx", ch);
               }
               else printf(" %hhu", cmdExt[i]);
            }
         }
         printf("\n");
         break;

      case 7: /* PROCP */
         if (r != (4 + (4*LG_MAX_SCRIPT_PARAMS)))
         {
            printf("%d", r);
            xReport(RGS_SCRIPT_ERR, "ERROR: %s", lguErrorText(r));
         }
         else
         {
            printf("%d", argI[0]);
            for (i=0; i<LG_MAX_SCRIPT_PARAMS; i++)
            {
               printf(" %d", argI[i+1]);
            }
         }
         printf("\n");
         break;

      case 9: /* GGR */
         if (r < 0)
         {
            printf("%d\n", r);
            xReport(RGS_SCRIPT_ERR, "ERROR: %s", lguErrorText(r));
         }
         else
         {
            printf("%d %"PRIu64"\n", argI[2], argQ[0]);
         }
         break;

      case 10: /* GIC */
         if (r < 0)
         {
            printf("%d\n", r);
            xReport(RGS_SCRIPT_ERR, "ERROR: %s", lguErrorText(r));
         }
         else
         {
            printf("%d \"%s\" \"%s\"\n", argI[0], cmdExt+4, cmdExt+36);
         }
         break;

      case 11: /* GIL */
         if (r < 0)
         {
            printf("%d\n", r);
            xReport(RGS_SCRIPT_ERR, "ERROR: %s", lguErrorText(r));
         }
         else
         {
            printf("%d %d \"%s\" \"%s\"\n",
               argI[0], argI[1], cmdExt+8, cmdExt+40);
         }
         break;

      default:
         printf("*** command=%d, status=%d\n", cmdP->cmd, r);
         if (r < 0) xReport(RGS_SCRIPT_ERR, "ERROR: %s", lguErrorText(r));
         break;

   }
}

static int xSendCommand(int sock, lgCmd_p cmdP, char *cmdExt)
{
   if (sock == SOCKET_OPEN_FAILED)
   {
      xReport(RGS_CONNECT_ERR, "socket connect failed");
      return -1;
   }

   if (send(sock, cmdP, sizeof(lgCmd_t), 0) != sizeof(lgCmd_t))
   {
      xReport(RGS_CONNECT_ERR, "socket send failed");
      return -1;
   }

   if (cmdP->size)
   {
      if (send(sock, cmdExt, cmdP->size, 0) != cmdP->size)
      {
         xReport(RGS_CONNECT_ERR, "socket send failed");
         return -1;
      }
   }

   if (recv(sock, cmdP, sizeof(lgCmd_t), MSG_WAITALL) !=
   sizeof(lgCmd_t))
   {
      xReport(RGS_CONNECT_ERR, "socket receive failed");
      return -1;
   }

   if (cmdP->size)
   {
      if (recv(sock, cmdExt, cmdP->size, MSG_WAITALL) !=
                     cmdP->size)
      {
         xReport(RGS_CONNECT_ERR, "socket receive failed");
         return -1;
      }
      cmdExt[cmdP->size] = 0;
   }
   return 0;
}

int main(int argc , char *argv[])
{
   int sock;
   int args, idx, i, pp, l, len;
   char salt1[LG_SALT_LEN];
   char user[LG_USER_LEN];
   cmdCtl_t ctl;
   cmdScript_t s;
   lgCmd_t cmdBuf[CMD_MAX_EXTENSION/sizeof(lgCmd_t)];
   lgCmd_p cmdP=cmdBuf;
   char *cmdExt=(char*)&cmdP[1];

   sock = xOpenSocket();

   args = xInitOpts(argc, argv);

   text[0] = 0;
   l = 0;
   pp = 0;

   for (i=args; i<argc; i++)
   {
      l += (strlen(argv[i]) + 1);
      if (l < sizeof(text)) {sprintf(text+pp, "%s ", argv[i]); pp=l;}
   }

   if (pp) {text[--pp] = 0;}

   ctl.inScript = 0;
   ctl.eaten = 0;

   len = strlen(text);
   idx = 0;

   while ((idx >= 0) && (ctl.eaten < len))
   {
      if ((idx=cmdParse(text, &ctl, cmdBuf, sizeof(cmdBuf))) >= 0)
      {
         cmdP->magic = LG_MAGIC;
         cmdP->doubles = 0;
         cmdP->longs = 0;
         cmdP->shorts = 0;

         if (cmdP->cmd < LG_CMD_SCRIPT)
         {
            if (cmdP->cmd == LG_CMD_PARSE)
            {
               cmdParseScript(cmdExt, &s, 1);
               if (s.par) free (s.par);
            }
            else if (cmdP->cmd == LG_CMD_USER)
            {
               /* need to auto login, create salt */
               snprintf(salt1, LG_SALT_LEN, "%015"PRIx64, xMakeSalt());
               /* save username */
               snprintf(user, LG_USER_LEN, "%s", cmdExt);
               /* change command to salt + user */
               sprintf(cmdExt, "%s.%s", salt1, user);
               cmdP->size = strlen(cmdExt);
               
               if (xSendCommand(sock, cmdP, cmdExt) == 0)
               {
                  /* take salt2 from message and overwrite with hash */
                  lgMd5UserHash(user, salt1, cmdExt, "", cmdExt);
                  cmdP->cmd = LG_CMD_PASSW;
                  cmdP->size = strlen(cmdExt);
                  if (xSendCommand(sock, cmdP, cmdExt) == 0)
                     xShowResult(0, cmdP, cmdExt);
               }
            }
            else
            {
               if (xSendCommand(sock, cmdP, cmdExt) == 0)
                  xShowResult(cmdInfo[idx].rv, cmdP, cmdExt);
            }
         }
         else xReport(RGS_SCRIPT_ERR,
                 "%s only allowed within a script", cmdInfo[idx].name);
      }
      else
      {
         if (idx == CMD_UNKNOWN_CMD)
            xReport(RGS_SCRIPT_ERR,
               "%s? unknown command, rgs -h for help", cmdStr());
         else
            xReport(RGS_SCRIPT_ERR,
               "%s: bad parameter, rgs -h for help", cmdStr());
      }
   }

   if (sock >= 0) close(sock);

   return status;
}

