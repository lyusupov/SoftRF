/*
 * This is an OpenSSL-compatible implementation of the RSA Data Security,
 * Inc. MD5 Message-Digest Algorithm (RFC 1321).
 *
 * Written by Solar Designer <solar at openwall.com> in 2001, and placed
 * in the public domain.  There's absolutely no warranty.
 *
 * This differs from Colin Plumb's older public domain implementation in
 * that no 32-bit integer data type is required, there's no compile-time
 * endianness configuration, and the function prototypes match OpenSSL's.
 * The primary goals are portability and ease of use.
 *
 * This implementation is meant to be fast, but not as fast as possible.
 * Some known optimizations are not included to reduce source code size
 * and avoid compile-time configuration.
 */

/*
 * Updated by Scott MacVicar for arduino
 * <scott@macvicar.net>
 */

#ifndef MD5_h
#define MD5_h

#include "Arduino.h"
#include <string.h>

namespace MD5
{
	typedef unsigned long MD5_u32plus;

	typedef struct {
		MD5_u32plus lo, hi;
		MD5_u32plus a, b, c, d;
		unsigned char buffer[64];
		MD5_u32plus block[16];
	} MD5_CTX;

	class MD5
	{
	public:
		MD5();
		static unsigned char* make_hash(char *arg);
		static unsigned char* make_hash(char *arg,size_t size);
		static char* make_digest(const unsigned char *digest, int len);
		static const void *body(void *ctxBuf, const void *data, size_t size);
		static void MD5Init(void *ctxBuf);
		static void MD5Initialize(void *ctxBuf, const MD5_u32plus a,const MD5_u32plus b,const MD5_u32plus c,const MD5_u32plus d);
		static void MD5Final(unsigned char *result, void *ctxBuf);
		static void MD5Update(void *ctxBuf, const void *data, size_t size);
	};
}

#endif