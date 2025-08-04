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

#include "lgpio.h"

#include "lgDbg.h"
#include "lgCtx.h"

static pthread_key_t slgGlobalKey;

static pthread_once_t xInited = PTHREAD_ONCE_INIT;

static void xInit(void)
{
   LG_DBG(LG_DEBUG_ALLOC, "");
   (void) pthread_key_create(&slgGlobalKey, NULL);
}

lgCtx_p lgCtxGet(void)
{
   lgCtx_p ctx;

   pthread_once(&xInited, xInit);

   LG_DBG(LG_DEBUG_ALLOC, "thread=%llu", (long long int)pthread_self());

   ctx = pthread_getspecific(slgGlobalKey);

   LG_DBG(LG_DEBUG_ALLOC, "ctx=%p", ctx);

   if (ctx == NULL)
   {
      ctx = calloc(1, sizeof(lgCtx_t));

      if (ctx != NULL)
      {
         // all fields cleared by calloc
         pthread_setspecific(slgGlobalKey, ctx);
      }
   }

   LG_DBG(LG_DEBUG_ALLOC, "ctx=%p", ctx);

   return ctx;
}

