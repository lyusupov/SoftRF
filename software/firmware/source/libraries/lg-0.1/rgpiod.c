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

#include <sys/types.h>
#include <sys/stat.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <syslog.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <sys/socket.h>
#include <netdb.h>

#include "lgpio.h"
#include "rgpiod.h"

#include "lgDbg.h"

/* globals */

int      gPermits = 0;
int      gNumSockNetAddr = 0;
uint32_t gSockNetAddr[MAX_CONNECT_ADDRESSES];
int      gFdSock = -1;

/* locals */

static int      CfgIfFlags = LG_DEFAULT_IF_FLAGS;
static int      CfgSocketPort = LG_DEFAULT_SOCKET_PORT;
static pthread_t pthSocket;

/* prototypes */

void *pthSocketThread(void *x);

/* ----------------------------------------------------------------------- */

static int xOpenSocket(void)
{
   int i;
   int opt=1;
   struct sockaddr_in server;
   struct sockaddr_in6 server6;
   char *portStr;
   int port;
   pthread_attr_t pthAttr;

   LG_DBG(LG_DEBUG_STARTUP, "");

   gFdSock = -1;

   if (pthread_attr_init(&pthAttr))
      PARAM_ERROR(LG_INIT_FAILED, "pthread_attr_init failed (%m)");

   if (pthread_attr_setstacksize(&pthAttr, STACK_SIZE))
      PARAM_ERROR(LG_INIT_FAILED, "pthread_attr_setstacksize failed (%m)");

   portStr = getenv(LG_ENVPORT);
   if (portStr) port = atoi(portStr); else port = CfgSocketPort;

   // Accept connections on IPv6, unless we have an IPv4-only whitelist
   if (!gNumSockNetAddr)
   {
      gFdSock = socket(AF_INET6, SOCK_STREAM , 0);

      if (gFdSock != -1)
      {
         bzero((char *)&server6, sizeof(server6));
         server6.sin6_family = AF_INET6;
         if (CfgIfFlags & LG_LOCALHOST_SOCK_IF)
         {
            server6.sin6_addr = in6addr_loopback;
         }
         else
         {
            server6.sin6_addr = in6addr_any;
         }
         server6.sin6_port = htons(port);

         setsockopt(gFdSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
         if (bind(gFdSock,(struct sockaddr *)&server6, sizeof(server6)) < 0)
            PARAM_ERROR(LG_INIT_FAILED, "bind to port %d failed (%m)", port);
      }
   }

   if (gNumSockNetAddr || gFdSock == -1)
   {
      gFdSock = socket(AF_INET , SOCK_STREAM , 0);

      if (gFdSock == -1)
         PARAM_ERROR(LG_INIT_FAILED, "socket failed (%m)");
      else
      {
        setsockopt(gFdSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
      }
      server.sin_family = AF_INET;
      if (CfgIfFlags & LG_LOCALHOST_SOCK_IF)
      {
         server.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      }
      else
      {
         server.sin_addr.s_addr = htonl(INADDR_ANY);
      }
      server.sin_port = htons(port);

      if (bind(gFdSock,(struct sockaddr *)&server , sizeof(server)) < 0)
         PARAM_ERROR(LG_INIT_FAILED, "bind to port %d failed (%m)", port);
   }

   if (pthread_create(&pthSocket, &pthAttr, pthSocketThread, &i))
      PARAM_ERROR(LG_INIT_FAILED, "pthread_create socket failed (%m)");

   return LG_OKAY;
}

static void xFatal(char *fmt, ...)
{
   char buf[128];
   va_list ap;

   va_start(ap, fmt);
   vsnprintf(buf, sizeof(buf), fmt, ap);
   va_end(ap);

   fprintf(stderr, "%s\n", buf);

   fflush(stderr);

   exit(EXIT_FAILURE);
}

static void xUsage()
{
   fprintf(stderr, "\n" \
      "Usage: rgpiod [OPTION] ...\n" \
      "   -c dir,     set config dir (default launch dir)\n" \
      "   -l,         localhost socket only (default local+remote)\n" \
      "   -n IP addr, allow address, name or dotted (default allow all)\n" \
      "   -p value,   socket port (1024-32000, default 8889)\n" \
      "   -v,         display rgpiod version and exit\n" \
      "   -w dir,     set working directory (default launch directory)\n" \
      "   -x,         enable access control (default off)\n" \
      "EXAMPLE\n" \
      "rgpiod -p 9000 &\n" \
      "  Start with socket port 9000\n" \
   "\n");
}

static uint64_t xGetNum(char *str, int *err)
{
   uint64_t val;
   char *endptr;

   *err = 0;
   val = strtoll(str, &endptr, 0);
   if (*endptr) {*err = 1; val = -1;}
   return val;
}

static uint32_t xCheckAddr(char *addrStr)
{
   int err;
   struct addrinfo hints, *res;
   struct sockaddr_in *sin;
   const char *portStr;
   uint32_t addr;

   portStr = getenv(LG_ENVPORT);

   if (!portStr) portStr = LG_DEFAULT_SOCKET_PORT_STR;

   memset (&hints, 0, sizeof (hints));

   hints.ai_family   = AF_INET;
   hints.ai_socktype = SOCK_STREAM;
   hints.ai_flags   |= AI_CANONNAME;

   err = getaddrinfo(addrStr, portStr, &hints, &res);

   if (err) return 0;

   sin = (struct sockaddr_in *)res->ai_addr;
   addr = sin->sin_addr.s_addr;

   freeaddrinfo(res);

   return addr;
}

static void xInitOpts(int argc, char *argv[])
{
   int opt, err, i;
   uint32_t addr;

   while ((opt = getopt(argc, argv, "c:ln:p:vw:x")) != -1)
   {
      switch (opt)
      {
         case 'c': /* configuration directory */
            lguSetConfigDir(optarg);
            break;

         case 'l':
            CfgIfFlags |= LG_LOCALHOST_SOCK_IF;
            break; 

         case 'n':
            addr = xCheckAddr(optarg);
            if (addr && (gNumSockNetAddr<MAX_CONNECT_ADDRESSES))
               gSockNetAddr[gNumSockNetAddr++] = addr;
            else xFatal("invalid -n option (%s)", optarg);
            break; 

         case 'p':
            i = xGetNum(optarg, &err);
            if ((i >= LG_MIN_SOCKET_PORT) && (i <= LG_MAX_SOCKET_PORT))
               CfgSocketPort = i;
            else xFatal("invalid -p option (%d)", i);
            break;

         case 'v':
            printf("rgpiod_%d.%d.%d.%d\n",
               (RGPIOD_VERSION>>24)&0xff, (RGPIOD_VERSION>>16)&0xff,
               (RGPIOD_VERSION>>8)&0xff, RGPIOD_VERSION&0xff);
            exit(EXIT_SUCCESS);
            break;

         case 'w': /* working directory */
            lguSetWorkDir(optarg);
            break;

         case 'x': /* enable user permissions checks */
            gPermits = 1;
            break;

        default: /* '?' */
           xUsage();
           exit(EXIT_FAILURE);
        }
    }
}

int main(int argc, char **argv)
{
   int flags;

   /* check command line parameters */

   xInitOpts(argc, argv);

   /* initialise */

   if (xOpenSocket() >= 0)
   {
      /* set stderr non-blocking */

      flags = fcntl(fileno(stderr), F_GETFL, 0);
      fcntl(fileno(stderr), F_SETFL, flags | O_NONBLOCK);

      /* sleep forever */

      while (1)
      {
         sleep(1);

         fflush(stderr);
      }
   }

   return -1;
}

