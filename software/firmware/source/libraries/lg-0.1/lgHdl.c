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

#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include "lgpio.h"

#include "lgCtx.h"
#include "lgDbg.h"
#include "lgHdl.h"


#define LG_HDL_FREE 0
#define LG_HDL_RSVD 1

#define LG_HDL_SLOTS 1024

typedef struct
{
   uint32_t magic;
   int32_t first;
   int32_t last;
} slgHdlTypeUsage_t;

typedef struct
{
   char user[LG_USER_LEN]; // creator (defines permissions)
   void *obj;              // pointer to object
   pthread_mutex_t mutex;  // access control
   int type;               // type of object, e.g. GPIO, file, etc.
   int next;               // next handle of type
   int previous;           // previous handle of type
   uint32_t magic;         // guard to check object of correct type
   callbk_t destructor;    // used to correctly free object resources
   int owner;              // id of owning thread
   int share;              // if object can be used by non-owners
} lgHdlHdr_t, *lgHdlHdr_p;

typedef struct
{
   lgHdlHdr_p header;
   pthread_mutex_t mutex; // access control
} lgHdl_t;

static pthread_mutex_t slgHdlMutex = PTHREAD_MUTEX_INITIALIZER;

lgHdl_t lgHdl[LG_HDL_SLOTS];

static slgHdlTypeUsage_t slgHdlTypeUsage[]=
{
   {101442315, -1, -1}, {263997524, -1, -1}, {354388063, -1, -1},
   {412249733, -1, -1}, {503487673, -1, -1}, {641332553, -1, -1},
   {707085925, -1, -1}, {806156471, -1, -1}, {979542324, -1, -1},
   {128752466, -1, -1}, {271619210, -1, -1}, {380943518, -1, -1},
   {482972576, -1, -1}, {549545782, -1, -1}, {651826746, -1, -1},
   {724133862, -1, -1}, {894093461, -1, -1}, {967000257, -1, -1},
};

static pthread_once_t xInited = PTHREAD_ONCE_INIT;

static void xInit(void)
{
   int i;
   for (i=0; i<LG_HDL_SLOTS; i++)
   {
      lgHdl[i].header = NULL;
      pthread_mutex_init(&lgHdl[i].mutex, NULL);
   }
}


// return first available handle

static int xHdlGet(void)
{
   int handle;
   int i;

   handle = LG_NO_HANDLE;

   pthread_mutex_lock(&slgHdlMutex);

   for (i=0; i<LG_HDL_SLOTS; i++)
   {
      if (lgHdl[i].header == LG_HDL_FREE)
      {
         lgHdl[i].header = (void *)LG_HDL_RSVD;

         handle = i;

         break;
      }
   }

   pthread_mutex_unlock(&slgHdlMutex);

   return handle;
}

int lgHdlAlloc(
   int type, int objSize, void **objPtr, callbk_t destructor)
{
   int handle;
   int last;
   lgHdlHdr_p h;
   lgCtx_p Ctx;

   pthread_once(&xInited, xInit);

   Ctx = lgCtxGet();

   if (Ctx == NULL) return LG_NO_MEMORY;

   handle = xHdlGet();

   if (handle < 0) return handle;

   *objPtr = calloc(1, objSize);

   if (*objPtr == NULL)
   {
      lgHdl[handle].header = (void *)LG_HDL_FREE;
      ALLOC_ERROR(LG_NO_MEMORY, "");
   }

   h = calloc(1, sizeof(lgHdlHdr_t));

   if (h == NULL)
   {
      free(*objPtr);
      lgHdl[handle].header = (void *)LG_HDL_FREE;
      ALLOC_ERROR(LG_NO_MEMORY, "");
   }

   last = slgHdlTypeUsage[type].last;

   if (last >= 0)
   {
      // add handle to end of chain for type
      h->previous = last;
      h->next = -1;
      lgHdl[last].header->next = handle;
      slgHdlTypeUsage[type].last = handle;
   }
   else
   {
      h->previous = -1;
      h->next = -1;
      slgHdlTypeUsage[type].first = handle;
      slgHdlTypeUsage[type].last = handle;
   }

   h->magic = slgHdlTypeUsage[type].magic;
   h->destructor = destructor;
   h->obj = *objPtr;
   h->type = type;

   h->share = Ctx->autoSetShare;
   h->owner = Ctx->owner;
   strncpy(h->user, Ctx->user, LG_USER_LEN);

   lgHdl[handle].header = h;

   return handle;
}

int lgHdlLock(int handle)
{
   pthread_once(&xInited, xInit);

   if ((handle < 0) || (handle >= LG_HDL_SLOTS))
      PARAM_ERROR(LG_BAD_HANDLE, "bad handle (%d)", handle);

   pthread_mutex_lock(&lgHdl[handle].mutex);   

   return LG_OKAY;
}

int lgHdlUnlock(int handle)
{
   pthread_once(&xInited, xInit);

   if ((handle < 0) || (handle >= LG_HDL_SLOTS))
      PARAM_ERROR(LG_BAD_HANDLE, "bad handle (%d)", handle);

   pthread_mutex_unlock(&lgHdl[handle].mutex);   

   return LG_OKAY;
}

int lgHdlGetObj(int handle, int type, void **objPtr)
{
   lgHdlHdr_p h;

   pthread_once(&xInited, xInit);

   if ((handle < 0) || (handle >= LG_HDL_SLOTS))
      PARAM_ERROR(LG_BAD_HANDLE, "bad handle (%d)", handle);

   h = lgHdl[handle].header;
 
   if ((h == (void *)LG_HDL_FREE) || (h == (void *)LG_HDL_RSVD))
      PARAM_ERROR(LG_BAD_HANDLE, "bad handle (%d)", handle);

   if ((h->type != type) || (h->magic != slgHdlTypeUsage[type].magic))
      PARAM_ERROR(LG_BAD_HANDLE, "bad handle (%d)", handle);
   
   *objPtr = h->obj;
   
   return LG_OKAY;
}

int lgHdlGetLockedObj(int handle, int type, void **objPtr)
{
   lgHdlHdr_p h;
   lgCtx_p Ctx;

   pthread_once(&xInited, xInit);

   Ctx = lgCtxGet();

   if ((handle < 0) || (handle >= LG_HDL_SLOTS))
      PARAM_ERROR(LG_BAD_HANDLE, "bad handle (%d)", handle);

   pthread_mutex_lock(&lgHdl[handle].mutex);   

   h = lgHdl[handle].header;
 
   if ((h == (void *)LG_HDL_FREE) || (h == (void *)LG_HDL_RSVD))
   {
      pthread_mutex_unlock(&lgHdl[handle].mutex);   
      PARAM_ERROR(LG_BAD_HANDLE, "bad handle (%d)", handle);
   }

   if ((h->type != type) || (h->magic != slgHdlTypeUsage[type].magic))
   {
      pthread_mutex_unlock(&lgHdl[handle].mutex);   
      PARAM_ERROR(LG_BAD_HANDLE, "bad handle (%d)", handle);
   }

   if ((h->owner != Ctx->owner) &&
       ((h->share == 0) ||
        (h->share != Ctx->autoUseShare)  ||
        (strcmp(h->user, Ctx->user) != 0)))
   {
      pthread_mutex_unlock(&lgHdl[handle].mutex);   
      PARAM_ERROR(LG_NO_PERMISSIONS,
         "not owned or shared by user (%d)", handle);
   }

   *objPtr = h->obj;
   
   return LG_OKAY;
}

int lgHdlGetLockedObjTrusted(int handle, int type, void **objPtr)
{
   lgHdlHdr_p h;

   pthread_once(&xInited, xInit);

   pthread_mutex_lock(&lgHdl[handle].mutex);   

   h = lgHdl[handle].header;
 
   if ((h == (void *)LG_HDL_FREE) || (h == (void *)LG_HDL_RSVD))
   {
      pthread_mutex_unlock(&lgHdl[handle].mutex);   
      PARAM_ERROR(LG_BAD_HANDLE, "bad handle (%d)", handle);
   }

   if ((h->type != type) || (h->magic != slgHdlTypeUsage[type].magic))
   {
      pthread_mutex_unlock(&lgHdl[handle].mutex);   
      PARAM_ERROR(LG_BAD_HANDLE, "bad handle (%d)", handle);
   }

   *objPtr = h->obj;
   
   return LG_OKAY;
}

int lgHdlSetShare(int handle, int share)
{
   lgHdlHdr_p h;
   lgCtx_p Ctx;

   pthread_once(&xInited, xInit);

   Ctx = lgCtxGet();

   if ((handle < 0) || (handle >= LG_HDL_SLOTS))
      PARAM_ERROR(LG_BAD_HANDLE, "bad handle (%d)", handle);

   pthread_mutex_lock(&lgHdl[handle].mutex);   

   h = lgHdl[handle].header;
 
   if ((h == (void *)LG_HDL_FREE) || (h == (void *)LG_HDL_RSVD))
   {
      pthread_mutex_unlock(&lgHdl[handle].mutex);   
      PARAM_ERROR(LG_BAD_HANDLE, "bad handle (%d)", handle);
   }

   if (h->owner != Ctx->owner)
   {
      pthread_mutex_unlock(&lgHdl[handle].mutex);   
      PARAM_ERROR(LG_NO_PERMISSIONS, "not owned (%d)", handle);
   }

   h->share = share;

   pthread_mutex_unlock(&lgHdl[handle].mutex);   
   
   return LG_OKAY;
}

int lgHdlGetHandlesForType(int type, int *handles, int size)
{
   int hdl;
   int count=0;
   
   pthread_once(&xInited, xInit);

   pthread_mutex_lock(&slgHdlMutex);
   
   hdl = slgHdlTypeUsage[type].first;

   while (hdl >= 0)
   {
      if (count < size) handles[count] = hdl;
      count ++;
      hdl = lgHdl[hdl].header->next;
   }
   
   pthread_mutex_unlock(&slgHdlMutex);
   
   return count;
}

int lgHdlFree(int handle, int type)
{
   int status;
   void **dummy;
   lgHdlHdr_p h;

   pthread_once(&xInited, xInit);

   LG_DBG(LG_DEBUG_TRACE, "handle=%d type=%d", handle, type);

   pthread_mutex_lock(&slgHdlMutex);
   
   status = lgHdlGetObj(handle, type, (void **)&dummy);

   if (status == LG_OKAY)
   {
      h = lgHdl[handle].header;
      
      if (h->previous >= 0)
      {
         // not first
         lgHdl[h->previous].header->next = h->next;
      }
      else
      {
         // first
         slgHdlTypeUsage[type].first = h->next;
      }

      if (h->next >= 0)
      {
         // not last
         lgHdl[h->next].header->previous = h->previous;
      }
      else
      {
         slgHdlTypeUsage[type].last = h->previous;
      }
         
      lgHdl[handle].header = NULL;

      if (h->destructor != NULL) (h->destructor)(h->obj);

      if (h->obj != NULL) free(h->obj);
      
      free(h);
   }
   pthread_mutex_unlock(&slgHdlMutex);
   
   return status;
}

// purge all handles with a given owner
void lgHdlPurgeByOwner(int owner)
{
   int i;
   lgHdlHdr_p h;

   pthread_once(&xInited, xInit);

   for (i=0; i<LG_HDL_SLOTS; i++)
   {
      h = lgHdl[i].header;

      if ((h != (void *)LG_HDL_FREE) && (h != (void *)LG_HDL_RSVD))
      {
         if ((h->owner == owner) && (!h->share))
         {
            lgHdlFree(i, h->type);
         }
      }
   }
}

