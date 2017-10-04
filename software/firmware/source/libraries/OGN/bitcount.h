// Fast bit counting for (forward) error correction codes
// (c) 2003, Pawel Jalocha, Pawel.Jalocha@cern.ch

#ifndef __BITCOUNT_H__
#define __BITCOUNT_H__

#include <stdint.h>

#define BITCOUNT_USE_BUILTIN
#define BITCOUNT_SAVE_FLASH

// ==========================================================================
// a table for fast bit counting

#ifdef BITCOUNT_SAVE_FLASH
extern const uint8_t ByteCount1s[16];
#else
extern const uint8_t ByteCount1s[256];
#endif

// ==========================================================================

#ifdef BITCOUNT_USE_BUILTIN
inline uint8_t Count1s(uint8_t Byte) { return __builtin_popcount(Byte); }
#else
#ifdef BITCOUNT_SAVE_FLASH
inline uint8_t Count1s(uint8_t Byte) { return ByteCount1s[Byte&0x0F] + ByteCount1s[Byte>>4]; }
#else
inline uint8_t Count1s(uint8_t Byte) { return ByteCount1s[Byte]; }
#endif
#endif

inline uint8_t Count1s(int8_t Byte) { return Count1s((uint8_t)Byte); }

#ifdef BITCOUNT_USE_BUILTIN
inline uint8_t Count1s(uint16_t Word) { return __builtin_popcount(Word); }
#else
inline uint8_t Count1s(uint16_t Word)
{ return Count1s((uint8_t)Word)
        +Count1s((uint8_t)(Word>>8)); }
#endif

inline uint8_t Count1s(int16_t Word) { return Count1s((uint16_t)Word); }

#ifdef BITCOUNT_USE_BUILTIN
inline uint8_t Count1s(uint32_t LongWord) { return __builtin_popcountl(LongWord); }
#else
inline uint8_t Count1s(uint32_t LongWord)
{ return Count1s((uint16_t)LongWord)
        +Count1s((uint16_t)(LongWord>>16)); }
#endif

inline uint8_t Count1s(int32_t LongWord) { return Count1s((uint32_t)LongWord); }

#ifdef BITCOUNT_USE_BUILTIN
inline uint8_t Count1s(uint64_t LongWord) { return __builtin_popcountll(LongWord); }
#else
inline uint8_t Count1s(uint64_t LongWord)
{ return Count1s((uint32_t)LongWord)
        +Count1s((uint32_t)(LongWord>>32)); }
#endif

inline uint8_t Count1s(int64_t LongWord) { return Count1s((uint64_t)LongWord); }

int   Count1s(const uint8_t *Byte, int Bytes);

// ==========================================================================

// use __builtin_popcount(unsigned int) ? http://stackoverflow.com/questions/109023/how-to-count-the-number-of-set-bits-in-a-32-bit-integer

#endif // of __BITCOUNT_H__
