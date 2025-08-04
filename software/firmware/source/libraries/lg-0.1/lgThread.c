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
#include <inttypes.h>

#include "lgpio.h"

#include "lgDbg.h"

pthread_t *lgThreadStart(lgThreadFunc_t f, void *userdata)
{
   pthread_t *pth;
   pthread_attr_t pthAttr;

   LG_DBG(LG_DEBUG_TRACE, "f=%08"PRIXPTR", userdata=%08"PRIXPTR,
      (uintptr_t)f, (uintptr_t)userdata);

   pth = malloc(sizeof(pthread_t));

   if (pth)
   {
      if (pthread_attr_init(&pthAttr))
      {
         free(pth);
         PARAM_ERROR(NULL, "pthread_attr_init failed");
      }

      if (pthread_attr_setstacksize(&pthAttr, STACK_SIZE))
      {
         free(pth);
         PARAM_ERROR(NULL, "pthread_attr_setstacksize failed");
      }

      if (pthread_create(pth, &pthAttr, f, userdata))
      {
         free(pth);
         PARAM_ERROR(NULL, "pthread_create failed");
      }
   }
   return pth;
}


void lgThreadStop(pthread_t *pth)
{
   LG_DBG(LG_DEBUG_TRACE, "pth=%08"PRIXPTR, (uintptr_t)pth);

   if (pth)
   {
      if (pthread_self() == *pth)
      {
         free(pth);
         pthread_exit(NULL);
      }
      else
      {
         pthread_cancel(*pth);
         pthread_join(*pth, NULL);
         free(pth);
      }
   }
}

