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

#include <inttypes.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "lgpio.h"
#include "rgpiod.h"

#include "lgCtx.h"
#include "lgDbg.h"
#include "lgHdl.h"

static void *xSocketThreadHandler(void *fdC)
{
   int sock = *(int*)fdC;
   int opt;
   lgCtx_p Ctx;
   lgCmd_t cmdBuf[CMD_MAX_EXTENSION/sizeof(lgCmd_t)];
   lgCmd_p cmdP=cmdBuf;
   uint32_t *arg=(uint32_t*)&cmdP[1];

   free(fdC);

   Ctx = lgCtxGet();

   if (!Ctx) return 0;

   /* Disable the Nagle algorithm. */
   opt = 1;

   setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char*)&opt, sizeof(int));

   while (1)
   {
      if (recv(sock, cmdP, sizeof(lgCmd_t), MSG_WAITALL) !=
         sizeof(lgCmd_t)) break;

      LG_DBG(LG_DEBUG_INTERNAL, "magic=%d size=%d cmd=%d Q=%d I=%d H=%d",
         cmdP->magic, cmdP->size, cmdP->cmd,
         cmdP->doubles, cmdP->longs, cmdP->shorts);

      if (cmdP->size)
      {
         if (cmdP->size < (sizeof(cmdBuf)-sizeof(lgCmd_t)))
         {
            /* read extension into buf */
            if (recv(sock, &cmdBuf[1], cmdP->size, MSG_WAITALL) != cmdP->size)
            {
               /* Serious error.  No point continuing. */

               LG_DBG(LG_DEBUG_ALWAYS,
                  "recv failed for %"PRId32" bytes, sock=%d",
                  cmdP->size, sock);

               break;
            }
         }
         else
         {
            /* Serious error.  No point continuing. */

            LG_DBG(LG_DEBUG_ALWAYS,
               "message too large %"PRId32"(%zd), sock=%d",
               cmdP->size, sizeof(cmdBuf)-sizeof(lgCmd_t), sock);

            break;
         }
      }

      if (cmdP->cmd == LG_CMD_NOIB)
      {
        /* Enable the Nagle algorithm. */
         opt = 0;
         setsockopt(
            sock, IPPROTO_TCP, TCP_NODELAY, (char*)&opt, sizeof(int));

         /* set sock as the argument */
         arg[0] = sock;
      }

      cmdP->status = lgExecCmd(cmdBuf, sizeof(cmdBuf));

      LG_DBG(LG_DEBUG_INTERNAL, "status=%d size=%d cmd=%d Q=%d I=%d H=%d",
         cmdP->status, cmdP->size, cmdP->cmd,
         cmdP->doubles, cmdP->longs, cmdP->shorts);

      if (write(sock, cmdBuf, sizeof(lgCmd_t))) ; /* ignore errors */

      if (cmdP->size)
      {
         if (write(sock, &cmdBuf[1], cmdP->size)); /* ignore errors */
      }
   }

   //lgNotifyCloseOrphans(-1, sock);

   lgHdlPurgeByOwner(Ctx->owner);

   close(sock);

   LG_DBG(LG_DEBUG_INTERNAL, "Socket %d closed", sock);

   LG_DBG(LG_DEBUG_INTERNAL, "free context memory %d", Ctx->owner);

   free(Ctx);

   return 0;
}

static int xAddrAllowed(struct sockaddr *saddr)
{
   int i;
   uint32_t addr;

   if (!gNumSockNetAddr) return 1;

   // FIXME: add IPv6 whitelisting support
   if (saddr->sa_family != AF_INET) return 0;

   addr = ((struct sockaddr_in *) saddr)->sin_addr.s_addr;

   for (i=0; i<gNumSockNetAddr; i++)
   {
      if (addr == gSockNetAddr[i]) return 1;
   }
   return 0;
}

/* ----------------------------------------------------------------------- */

void *pthSocketThread(void *x)
{
   int fdC=0, c, *sock;
   struct sockaddr_storage client;
   pthread_attr_t attr;

   if (pthread_attr_init(&attr))
      PARAM_ERROR((void*)LG_INIT_FAILED,
         "pthread_attr_init failed (%m)");

   if (pthread_attr_setstacksize(&attr, STACK_SIZE))
      PARAM_ERROR((void*)LG_INIT_FAILED,
         "pthread_attr_setstacksize failed (%m)");

   if (pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED))
      PARAM_ERROR((void*)LG_INIT_FAILED,
         "pthread_attr_setdetachstate failed (%m)");

   /* gFdSock opened in initialisation so that we can treat
      failure to bind as fatal. */

   listen(gFdSock, 100);

   c = sizeof(client);

   while (fdC >= 0)
   {
      pthread_t thr;

      fdC = accept(gFdSock, (struct sockaddr *)&client, (socklen_t*)&c);

      lgNotifyCloseOrphans(-1, fdC);

      if (xAddrAllowed((struct sockaddr *)&client))
      {
         LG_DBG(LG_DEBUG_INTERNAL, "Connection accepted on socket %d", fdC);

         sock = malloc(sizeof(int));

         *sock = fdC;

         /* Enable tcp_keepalive */
         int optval = 1;
         socklen_t optlen = sizeof(optval);

         if (setsockopt(fdC, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0)
         {
           LG_DBG(LG_DEBUG_ALWAYS, "setsockopt() fail, closing socket %d", fdC);
           close(fdC);
         }

         LG_DBG(LG_DEBUG_INTERNAL, "SO_KEEPALIVE enabled on socket %d\n", fdC);

         if (pthread_create
            (&thr, &attr, xSocketThreadHandler, (void*) sock) < 0)
            PARAM_ERROR((void*)LG_INIT_FAILED,
               "socket pthread_create failed (%m)");
      }
      else
      {
         LG_DBG(LG_DEBUG_ALWAYS, "Connection rejected, closing");
         close(fdC);
      }
   }

   if (fdC < 0)
      PARAM_ERROR((void*)LG_INIT_FAILED, "accept failed (%m)");

   return 0;
}

