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

#ifndef LG_CONTEXT_H
#define LG_CONTEXT_H

#include <pthread.h>

typedef struct lgPermit_s
{
   char *files;
   char *scripts;
   char *i2c;
   char *spi;
   char *serial;
   char *gpio;
   char *notify;
   char *debug;
   char *shell;
} lgPermit_t, *lgPermit_p;

typedef struct lgCtx_s
{
   char user[LG_USER_LEN];
   char salt1[LG_SALT_LEN];
   char salt2[LG_SALT_LEN];
   int owner;
   int approved;
   int autoSetShare;
   int autoUseShare;
   lgPermit_t permits;
} lgCtx_t, *lgCtx_p;

lgCtx_p lgCtxGet(void);

#endif

