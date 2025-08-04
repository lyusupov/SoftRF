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

#ifndef LG_MD5_H
#define LG_MD5_H

/*
This code is a modification of the following:
http://openwall.info/wiki/people/solar/software/public-domain-source-code/md5
*/

/* Any 32-bit or wider unsigned integer data type will do */
typedef unsigned int MD5_u32plus;

typedef struct
{
   MD5_u32plus lo, hi;
   MD5_u32plus a, b, c, d;
   unsigned char buffer[64];
   MD5_u32plus block[16];
} lgMd5_t, *lgMd5_p;

void lgMd5Init  (lgMd5_p ctx);
void lgMd5Update(lgMd5_p ctx, const void *data, unsigned long size);
void lgMd5Final (lgMd5_p ctx, unsigned char *result);

void lgMd5UserHash(
   char *user, char *salt1, char *salt2, char *secretFile, char *hash);

#endif

