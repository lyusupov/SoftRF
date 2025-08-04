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

#ifndef LG_HDL_H
#define LG_HDL_H

#include <pthread.h>

#include "lgpio.h"

#define LG_HDL_TYPE_NONE   0
#define LG_HDL_TYPE_GPIO   1
#define LG_HDL_TYPE_I2C    2
#define LG_HDL_TYPE_FILE   3
#define LG_HDL_TYPE_SERIAL 4
#define LG_HDL_TYPE_NOTIFY 5
#define LG_HDL_TYPE_SCRIPT 6
#define LG_HDL_TYPE_SPI    7

int lgHdlAlloc
   (int type, int objSize, void **objPtr, callbk_t destructor);

int lgHdlFree(int handle, int type);

void lgHdlPurgeByOwner(int owner);

int lgHdlGetObj(int handle, int type, void **objPtr);

int lgHdlGetLockedObj(int handle, int type, void **objPtr);

int lgHdlGetLockedObjTrusted(int handle, int type, void **objPtr);

int lgHdlSetShare(int handle, int share);

int lgHdlGetHandlesForType(int type, int *handles, int size);

int lgHdlLock(int handle);

int lgHdlUnlock(int handle);

#endif

